"""
Taller Segundo Corte — Punto B
Consola de mandos con ESP32 para mover los brazos del robot Baxter
de forma fluida, posicionar la pinza y coger / mover un objeto.

Simulación: PyBullet + modelo del Baxter de erwincoumans/pybullet_robots.

Uso:
    python baxter_esp32.py --puerto COM4 --datos ruta/a/pybullet_robots/data
    python baxter_esp32.py --sin_esp32 --datos ruta/a/pybullet_robots/data

Teclado (respaldo, en la ventana de PyBullet; se mantienen presionadas):
    Flechas = X / Y      1 / 2 = subir / bajar      3 / 4 = girar muñeca
    5 = abrir/cerrar pinza     6 = home     7 = cambiar de brazo
"""

import argparse
import os
import queue
import threading
import time

import numpy as np
import pybullet as p

try:
    import serial
except ImportError:
    serial = None

# ---------------------------------------------------------------------------
# Parámetros
# ---------------------------------------------------------------------------
DT_SIM = 1.0 / 240.0          # paso de la física
PASOS_CONTROL = 4             # IK cada 4 pasos -> 60 Hz
V_MAX = 0.15                  # velocidad máxima de la pinza [m/s]
A_MAX = 0.60                  # aceleración al arrancar [m/s^2]  (rampa = movimiento fluido)
A_FRENO = 1.50                # desaceleración al soltar el botón (frena en ~0,1 s)
W_MAX = 1.20                  # giro máximo de la muñeca [rad/s]
ALFA_W = 6.0                  # rapidez del filtro de giro [1/s]
WATCHDOG_S = 0.5              # sin mensajes de la ESP32 por 0,5 s -> se detiene

ALTURA_MESA = -0.15           # superficie de la mesa (marco del robot) [m]
LADO_CUBO = 0.03              # objeto de 3 cm
POS_CUBO = np.array([0.72, 0.30])
POS_DESTINO = np.array([0.72, -0.05])

# Espacio de trabajo de la pinza (con la pinza vertical el Baxter alcanza
# hasta x ~ 0,8 m). Cada brazo puede cruzar solo un poco al lado contrario.
LIM_MIN = np.array([0.40, -0.70, ALTURA_MESA + 0.005])
LIM_MAX = np.array([0.82, 0.70, 0.30])
CRUCE_Y = 0.15

# Definición de cada brazo (índices de articulaciones del URDF toms_baxter)
BRAZOS = {
    "IZQ": dict(articulaciones=[34, 35, 36, 37, 38, 40, 41], punta=48,
                dedos=(49, 51), puntas_dedos=(50, 52), signo_y=1),
    "DER": dict(articulaciones=[12, 13, 14, 15, 16, 18, 19], punta=26,
                dedos=(27, 29), puntas_dedos=(28, 30), signo_y=-1),
}
HOME = np.array([0.65, 0.30, 0.05])          # posición home (y se refleja para el brazo derecho)
DEDO_ABIERTO, DEDO_CERRADO = 0.020, 0.0

TECLAS_EJE = {               # tecla -> (eje, signo)   ejes: 0=x 1=y 2=z 3=giro
    p.B3G_UP_ARROW: (0, +1), p.B3G_DOWN_ARROW: (0, -1),
    p.B3G_LEFT_ARROW: (1, +1), p.B3G_RIGHT_ARROW: (1, -1),
    ord("1"): (2, +1), ord("2"): (2, -1),
    ord("3"): (3, +1), ord("4"): (3, -1),
}
TECLAS_CMD = {ord("5"): "PINZA", ord("6"): "HOME", ord("7"): "BRAZO"}


# ---------------------------------------------------------------------------
# Utilidades
# ---------------------------------------------------------------------------
def matriz_a_cuaternion(R):
    """Convierte una matriz de rotación 3x3 a cuaternión (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return [x, y, z, w]


def orientacion_pinza(giro):
    """Pinza apuntando hacia abajo, con la muñeca girada 'giro' rad alrededor de z.
    En el URDF el eje z local de la punta es la dirección de aproximación
    y los dedos se abren a lo largo del eje y local."""
    z = np.array([0.0, 0.0, -1.0])
    y = np.array([-np.sin(giro), np.cos(giro), 0.0])
    x = np.cross(y, z)
    return matriz_a_cuaternion(np.column_stack([x, y, z]))


# ---------------------------------------------------------------------------
# Cinemática inversa iterativa sobre una copia del robot
# ---------------------------------------------------------------------------
class SolverIK:
    """PyBullet resuelve la IK partiendo de la pose actual de las articulaciones y
    con una sola llamada suele quedar a varios cm del objetivo. Aquí se usa una
    copia del Baxter en un mundo aparte (sin física): se aplica la solución, se
    vuelve a calcular desde ahí y se repite hasta que la punta quede a < 0,5 mm.
    Así el robot de la simulación nunca "salta" mientras se calcula."""

    def __init__(self, datos, robot_real):
        self.cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(datos, physicsClientId=self.cid)
        self.robot = p.loadURDF("baxter_common/baxter_description/urdf/toms_baxter.urdf",
                                useFixedBase=True, physicsClientId=self.cid)
        self.movibles = [j for j in range(p.getNumJoints(self.robot, physicsClientId=self.cid))
                         if p.getJointInfo(self.robot, j, physicsClientId=self.cid)[2]
                         != p.JOINT_FIXED]
        info = [p.getJointInfo(self.robot, j, physicsClientId=self.cid) for j in self.movibles]
        self.lim_inf = [i[8] for i in info]
        self.lim_sup = [i[9] for i in info]
        self.amort = [0.01] * len(self.movibles)
        for j in self.movibles:                      # copia la pose inicial del robot real
            p.resetJointState(self.robot, j, p.getJointState(robot_real, j)[0],
                              physicsClientId=self.cid)

    def _error(self, punta, pos):
        real = p.getLinkState(self.robot, punta, computeForwardKinematics=1,
                              physicsClientId=self.cid)[4]
        return float(np.linalg.norm(np.array(real) - pos))

    def resolver(self, punta, art, dofs, pos, orn, q_inicial):
        # Arranca desde la última solución: las articulaciones cambian poco entre
        # un ciclo y el siguiente, lo que da continuidad al movimiento.
        for j, q in zip(art, q_inicial):
            p.resetJointState(self.robot, j, q, physicsClientId=self.cid)
        q = list(q_inicial)
        for _ in range(6):
            q_todas = p.calculateInverseKinematics(
                self.robot, punta, pos, orn, jointDamping=self.amort,
                maxNumIterations=50, residualThreshold=1e-5, physicsClientId=self.cid)
            # Respeta los límites físicos de cada articulación
            q = [float(np.clip(q_todas[d], self.lim_inf[d], self.lim_sup[d])) for d in dofs]
            for j, v in zip(art, q):
                p.resetJointState(self.robot, j, v, physicsClientId=self.cid)
            if self._error(punta, pos) < 5e-4:
                break
        return q, self._error(punta, pos)


# ---------------------------------------------------------------------------
# Brazo: integra la velocidad pedida, resuelve la IK y mueve la pinza
# ---------------------------------------------------------------------------
class Brazo:
    def __init__(self, robot, nombre, solver):
        self.robot, self.nombre, self.ik = robot, nombre, solver
        cfg = BRAZOS[nombre]
        self.art = cfg["articulaciones"]
        self.punta = cfg["punta"]
        self.dedos = cfg["dedos"]
        self.puntas_dedos = cfg["puntas_dedos"]
        self.dofs = [solver.movibles.index(j) for j in self.art]
        self.home = HOME * np.array([1, cfg["signo_y"], 1])
        s = cfg["signo_y"]
        self.lim_min = LIM_MIN.copy()
        self.lim_max = LIM_MAX.copy()
        if s > 0:
            self.lim_min[1] = -CRUCE_Y           # brazo izquierdo: y >= -0,15
        else:
            self.lim_max[1] = CRUCE_Y            # brazo derecho:   y <= +0,15
        self.objetivo = self.home.copy()
        self.vel = np.zeros(3)
        self.giro = 0.0
        self.w = 0.0
        self.auto = None               # destino de un movimiento automático (HOME)
        self.z_min_objeto = -np.inf    # altura mínima mientras sostiene un objeto
        self.pinza_cerrada = False
        self.q = [p.getJointState(robot, j)[0] for j in self.art]

    def colocar_en_home(self):
        """Pone el brazo instantáneamente en home (solo al iniciar)."""
        self.q, _ = self._ik(self.objetivo, self.giro)
        for j, q in zip(self.art, self.q):
            p.resetJointState(self.robot, j, q)
        self.mover_dedos(forzar=True)

    def _ik(self, pos, giro):
        return self.ik.resolver(self.punta, self.art, self.dofs, pos.tolist(),
                                orientacion_pinza(giro), self.q)

    def actualizar(self, comando, dt):
        """comando = [vx, vy, vz, w] con valores -1, 0 o 1. Devuelve True si se
        llegó a un límite (borde del espacio de trabajo o pose inalcanzable)."""
        if any(comando):
            self.auto = None                       # mover a mano cancela el HOME

        # 1) Velocidad deseada
        if self.auto is not None:
            # Movimiento automático: frena al acercarse (v = raíz(2·a·d))
            d = self.auto - self.objetivo
            dist = np.linalg.norm(d)
            rapidez = min(V_MAX, np.sqrt(2 * A_MAX * dist))
            v_deseada = d / dist * rapidez if dist > 1e-6 else np.zeros(3)
            w_deseada = float(np.clip(-3.0 * self.giro, -W_MAX, W_MAX))
            if dist < 0.001 and abs(self.giro) < 0.01:
                self.objetivo, self.giro = self.auto.copy(), 0.0
                self.vel[:], self.w, self.auto = 0.0, 0.0, None
                v_deseada, w_deseada = np.zeros(3), 0.0
        else:
            v_deseada = np.array(comando[:3], float) * V_MAX
            w_deseada = comando[3] * W_MAX

        # 2) Rampa de aceleración: arranca suave y frena un poco más rápido,
        #    para que la pinza se detenga casi donde se suelta el botón
        frenando = np.abs(v_deseada) < np.abs(self.vel)
        a = np.where(frenando, A_FRENO, A_MAX) * dt
        self.vel += np.clip(v_deseada - self.vel, -a, a)
        self.w += (w_deseada - self.w) * min(1.0, ALFA_W * dt)

        # 3) Nuevo objetivo, recortado al espacio de trabajo
        nuevo = self.objetivo + self.vel * dt
        lim_min = self.lim_min.copy()
        lim_min[2] = max(lim_min[2], self.z_min_objeto)   # no aplastar el objeto
        recortado = np.clip(nuevo, lim_min, self.lim_max)
        fuera = np.abs(recortado - nuevo) > 1e-9
        self.vel[fuera] = 0.0
        giro_nuevo = float(np.clip(self.giro + self.w * dt, -np.pi / 2, np.pi / 2))
        en_limite = bool(np.any(fuera))

        # 4) Cinemática inversa (solo si el objetivo cambió); si la pose no se
        #    alcanza, el objetivo no avanza y se avisa con LIMITE
        cambio = (np.linalg.norm(recortado - self.objetivo) > 1e-7
                  or abs(giro_nuevo - self.giro) > 1e-7)
        if cambio:
            q, err = self._ik(recortado, giro_nuevo)
            if err < 0.003:
                self.objetivo, self.giro, self.q = recortado, giro_nuevo, q
            else:
                self.vel[:], self.w = 0.0, 0.0
                en_limite = True

        # 5) Control de posición de las 7 articulaciones
        p.setJointMotorControlArray(
            self.robot, self.art, p.POSITION_CONTROL, targetPositions=self.q,
            forces=[150] * 7, positionGains=[0.1] * 7)
        return en_limite

    def ir_a_home(self):
        self.auto = self.home.copy()

    def mover_dedos(self, forzar=False):
        pos = DEDO_CERRADO if self.pinza_cerrada else DEDO_ABIERTO
        for dedo, signo in zip(self.dedos, (1, -1)):
            if forzar:
                p.resetJointState(self.robot, dedo, signo * pos)
            p.setJointMotorControl2(self.robot, dedo, p.POSITION_CONTROL,
                                    targetPosition=signo * pos, force=40, maxVelocity=0.05)

    def error_posicion(self):
        real = np.array(p.getLinkState(self.robot, self.punta)[4])
        return float(np.linalg.norm(real - self.objetivo))

    def toca(self, objeto):
        """True si los dos dedos están en contacto con el objeto."""
        lados = []
        for dedo, punta in zip(self.dedos, self.puntas_dedos):
            c = (p.getContactPoints(self.robot, objeto, dedo) +
                 p.getContactPoints(self.robot, objeto, punta))
            lados.append(len(c) > 0)
        return all(lados)


# ---------------------------------------------------------------------------
# Comunicación con la ESP32
# ---------------------------------------------------------------------------
class EnlaceESP32:
    def __init__(self, puerto, baudios=115200):
        if serial is None:
            raise RuntimeError("Falta pyserial: pip install pyserial")
        self.ser = serial.Serial(puerto, baudios, timeout=0.1)
        time.sleep(2)                       # la ESP32 se reinicia al abrir el puerto
        self.ser.reset_input_buffer()
        self.comandos = queue.Queue()
        self.vel = [0, 0, 0, 0]
        self.t_ultimo = 0.0
        self.activo = True
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        while self.activo:
            try:
                linea = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException:
                print("[!] Se perdió la conexión con la ESP32")
                break
            if linea.startswith("V:"):
                try:
                    v = [int(x) for x in linea[2:].split(",")]
                    if len(v) == 4:
                        self.vel = [max(-1, min(1, x)) for x in v]
                        self.t_ultimo = time.time()
                except ValueError:
                    pass
            elif linea.startswith("CMD:"):
                self.comandos.put(linea[4:])
                self.t_ultimo = time.time()
            elif linea:
                print(f"[ESP32] {linea}")

    def velocidad(self):
        if time.time() - self.t_ultimo > WATCHDOG_S:     # seguridad: sin datos -> quieto
            return [0, 0, 0, 0]
        return list(self.vel)

    def enviar_estado(self, estado):
        try:
            self.ser.write(f"EST:{estado}\n".encode())
        except serial.SerialException:
            pass

    def cerrar(self):
        self.activo = False
        self.ser.close()


# ---------------------------------------------------------------------------
# Escena
# ---------------------------------------------------------------------------
def crear_escena(datos):
    p.setAdditionalSearchPath(datos)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT_SIM)
    p.loadURDF("plane.urdf", [0, 0, -0.93], useFixedBase=True)
    robot = p.loadURDF("baxter_common/baxter_description/urdf/toms_baxter.urdf",
                       useFixedBase=True)

    # Mesa
    medio = [0.25, 0.55, 0.02]
    centro_mesa = [0.78, 0.0, ALTURA_MESA - medio[2]]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=medio)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=medio, rgbaColor=[0.55, 0.38, 0.22, 1])
    mesa = p.createMultiBody(0, col, vis, centro_mesa)
    p.changeDynamics(mesa, -1, lateralFriction=1.0)

    # Objeto: cubo de 3 cm, 50 g
    h = LADO_CUBO / 2
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[h] * 3)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[h] * 3, rgbaColor=[0.1, 0.4, 0.95, 1])
    cubo = p.createMultiBody(0.05, col, vis, [*POS_CUBO, ALTURA_MESA + h])
    p.changeDynamics(cubo, -1, lateralFriction=2.0, spinningFriction=0.01,
                     rollingFriction=0.001)

    # Zona de destino (marca verde, sin colisión)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.035, 0.035, 0.001],
                              rgbaColor=[0.1, 0.8, 0.2, 0.6])
    p.createMultiBody(0, -1, vis, [*POS_DESTINO, ALTURA_MESA + 0.001])
    p.addUserDebugText("DESTINO", [*POS_DESTINO, ALTURA_MESA + 0.06], [0, 0.6, 0], 1.2)

    # Más fricción en los dedos para que el agarre sea firme
    for cfg in BRAZOS.values():
        for link in (*cfg["dedos"], *cfg["puntas_dedos"]):
            p.changeDynamics(robot, link, lateralFriction=2.0)
    return robot, cubo


def preparar_brazos(robot, datos):
    # Pose de reposo: codos doblados, como el Baxter en su posición neutra.
    # Sirve de punto de partida y de "postura preferida" para la IK.
    neutra = {"s0": 0.0, "s1": -0.55, "e0": 0.0, "e1": 1.4, "w0": 0.0, "w1": 0.8, "w2": 0.0}
    for j in range(p.getNumJoints(robot)):
        nombre = p.getJointInfo(robot, j)[1].decode()
        for lado, signo in (("left", 1), ("right", -1)):
            for art, v in neutra.items():
                if nombre == f"{lado}_{art}":
                    p.resetJointState(robot, j, v * (signo if art in ("s0", "e0", "w0", "w2") else 1))
    solver = SolverIK(datos, robot)
    brazos = {n: Brazo(robot, n, solver) for n in BRAZOS}
    for b in brazos.values():
        b.colocar_en_home()
    return brazos


# ---------------------------------------------------------------------------
# Simulador: un paso de control (60 Hz) a partir de los comandos recibidos
# ---------------------------------------------------------------------------
class Simulador:
    def __init__(self, robot, cubo, brazos, informar):
        self.robot, self.cubo, self.brazos = robot, cubo, brazos
        self.informar = informar
        self.activo = "IZQ"
        self.agarre = None             # restricción fija mientras el objeto está agarrado
        self.agarre_brazo = None
        self.estado_obj = "LIBRE"
        self.en_limite_prev = False
        informar(f"BRAZO:{self.activo}")

    def paso_control(self, cmd_vel, cmds, dt):
        brazo = self.brazos[self.activo]
        for c in cmds:
            print(f"> Comando: {c}")
            if c == "PINZA":
                brazo.pinza_cerrada = not brazo.pinza_cerrada
                brazo.mover_dedos()
                self.informar("PINZA:CERRADA" if brazo.pinza_cerrada else "PINZA:ABIERTA")
            elif c == "HOME":
                brazo.ir_a_home()
            elif c == "BRAZO":
                brazo.vel[:] = 0
                self.activo = "DER" if self.activo == "IZQ" else "IZQ"
                brazo = self.brazos[self.activo]
                self.informar(f"BRAZO:{self.activo}")

        # Control de los dos brazos (el inactivo mantiene su posición)
        for nombre, b in self.brazos.items():
            lim = b.actualizar(cmd_vel if nombre == self.activo else [0, 0, 0, 0], dt)
            if nombre == self.activo:
                if lim and not self.en_limite_prev:
                    self.informar("LIMITE")
                self.en_limite_prev = lim

        # Agarre: pinza cerrada y los dos dedos tocando el cubo
        if self.agarre is None and brazo.pinza_cerrada and brazo.toca(self.cubo):
            # Se fija el cubo a la pinza en su posición relativa actual
            # (con solo fricción, en PyBullet el objeto tiende a resbalar)
            pp, po = p.getLinkState(self.robot, brazo.punta)[4:6]
            cp, co = p.getBasePositionAndOrientation(self.cubo)
            inv_p, inv_o = p.invertTransform(pp, po)
            rel_p, rel_o = p.multiplyTransforms(inv_p, inv_o, cp, co)
            self.agarre = p.createConstraint(self.robot, brazo.punta, self.cubo, -1,
                                             p.JOINT_FIXED, [0, 0, 0], rel_p, [0, 0, 0],
                                             [0, 0, 0, 1], rel_o)
            p.changeConstraint(self.agarre, maxForce=5)      # 10 veces el peso del cubo
            self.agarre_brazo = self.activo
            # Con el objeto en la mano, la pinza no puede bajar más de lo que
            # permite el objeto apoyado en la mesa
            fondo_obj = cp[2] - LADO_CUBO / 2
            brazo.z_min_objeto = ALTURA_MESA + (pp[2] - fondo_obj) + 0.002
        elif self.agarre is not None and not self.brazos[self.agarre_brazo].pinza_cerrada:
            p.removeConstraint(self.agarre)
            self.agarre = None
            self.brazos[self.agarre_brazo].z_min_objeto = -np.inf

        nuevo = "AGARRADO" if self.agarre is not None else "LIBRE"
        if nuevo != self.estado_obj:
            self.estado_obj = nuevo
            self.informar(f"OBJETO:{nuevo}")

    def en_destino(self):
        cp = np.array(p.getBasePositionAndOrientation(self.cubo)[0])
        return (np.linalg.norm(cp[:2] - POS_DESTINO) < 0.035
                and cp[2] < ALTURA_MESA + LADO_CUBO and self.agarre is None)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Baxter controlado por ESP32")
    ap.add_argument("--puerto", default="COM4")
    ap.add_argument("--sin_esp32", action="store_true", help="Usar solo el teclado")
    ap.add_argument("--datos", default=os.path.join("pybullet_robots", "data"),
                    help="Carpeta 'data' del repositorio pybullet_robots")
    args = ap.parse_args()

    urdf = os.path.join(args.datos, "baxter_common", "baxter_description", "urdf",
                        "toms_baxter.urdf")
    if not os.path.isfile(urdf):
        raise SystemExit(f"No encuentro el modelo del Baxter en:\n  {urdf}\n"
                         "Clona pybullet_robots y usa --datos <ruta>/pybullet_robots/data")

    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(1.9, 125, -30, [0.55, 0.0, -0.05])
    robot, cubo = crear_escena(args.datos)
    brazos = preparar_brazos(robot, args.datos)

    enlace = None
    if not args.sin_esp32:
        enlace = EnlaceESP32(args.puerto)
        print(f"Conectado a la ESP32 en {args.puerto}")

    def informar(estado):
        print(f"  Estado: {estado}")
        if enlace:
            enlace.enviar_estado(estado)

    sim = Simulador(robot, cubo, brazos, informar)
    print("Listo. Mantén presionados los botones para mover la pinza.")

    dt_ctrl = DT_SIM * PASOS_CONTROL
    texto_id = -1
    destino_avisado = False
    t0 = time.time()
    paso = 0
    try:
        while p.isConnected():
            if paso % PASOS_CONTROL == 0:
                cmd_vel = enlace.velocidad() if enlace else [0, 0, 0, 0]
                cmds = []
                if enlace:
                    while not enlace.comandos.empty():
                        cmds.append(enlace.comandos.get())
                teclas = p.getKeyboardEvents()
                for t, (eje, s) in TECLAS_EJE.items():
                    if t in teclas and teclas[t] & p.KEY_IS_DOWN:
                        cmd_vel[eje] = s
                for t, c in TECLAS_CMD.items():
                    if t in teclas and teclas[t] & p.KEY_WAS_TRIGGERED:
                        cmds.append(c)

                sim.paso_control(cmd_vel, cmds, dt_ctrl)

                if sim.en_destino() and not destino_avisado:
                    informar("EN_DESTINO")
                destino_avisado = sim.en_destino()

                if paso % (PASOS_CONTROL * 15) == 0:      # texto en pantalla, 4 veces/s
                    b = brazos[sim.activo]
                    o = b.objetivo
                    txt = (f"Brazo {sim.activo} | x={o[0]:.2f} y={o[1]:.2f} z={o[2]:.2f} "
                           f"giro={np.degrees(b.giro):.0f} | objeto {sim.estado_obj}")
                    texto_id = p.addUserDebugText(txt, [0.3, 0.75, 0.55], [0, 0, 0], 1.1,
                                                  replaceItemUniqueId=texto_id)

            p.stepSimulation()
            paso += 1
            espera = t0 + paso * DT_SIM - time.time()
            if espera > 0:
                time.sleep(espera)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if enlace:
            enlace.cerrar()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
