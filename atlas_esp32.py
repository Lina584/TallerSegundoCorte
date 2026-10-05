"""
Taller Segundo Corte — Punto C
Consola de mandos con ESP32 para mover el robot Atlas (Boston Dynamics)
de forma fluida en PyBullet: brazos (cada mano en X/Y/Z), sentadilla,
giro e inclinación del torso, cabeza, y animación de saludo.

Simulación: PyBullet + modelo y escena de erwincoumans/pybullet_robots
(atlas_v4_with_multisense.urdf sobre la caja "boston_box" en el laboratorio "botlab").

Uso:
    python atlas_esp32.py --puerto COM4
    python atlas_esp32.py --sin_esp32                 # solo teclado
    python atlas_esp32.py --puerto COM4 --escena simple   # sin el laboratorio (PC lento)

Teclado (respaldo, en la ventana de PyBullet; se mantienen presionadas):
    Flechas = eje 1 / eje 2 (plano A)    1/2 = eje 1 (plano B)    3/4 = eje 2 (plano B)
    5 = cambiar de parte     6 = home     7 = saludo
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
DT_SIM = 1.0 / 240.0
PASOS_CONTROL = 4                 # control a 60 Hz
WATCHDOG_S = 0.5

URDF_ATLAS = "atlas/atlas_v4_with_multisense.urdf"
POS_ATLAS = np.array([-2.0, 3.0])          # (x, y) del Atlas en el laboratorio
PISO_LAB = -2.01                           # altura del piso del laboratorio
LADO_CAJA = 1.0
SUELO = PISO_LAB + LADO_CAJA               # el Atlas se para encima de la caja
ALTURA_TOBILLO = 0.077                     # del tobillo a la planta del pie

# Índices de articulaciones del URDF
ESPALDA = {"bkz": 0, "bky": 1, "bkx": 2}
CUELLO = 10
BRAZO_IZQ = [3, 4, 5, 6, 7, 8]
BRAZO_DER = [11, 12, 13, 14, 15, 16]
MANO_IZQ, MANO_DER = 9, 17
PIERNA_IZQ = [18, 19, 20, 21, 22, 23]
PIERNA_DER = [24, 25, 26, 27, 28, 29]
PIE_IZQ, PIE_DER = 23, 29
TORSO = 2                                  # eslabón "utorso": marco de referencia de las manos

# Estado de la postura: un vector de 10 valores que se mueve con rampas suaves
#   0 altura de la pelvis   1 giro del torso   2 inclinación del torso   3 cabeza
#   4-6 mano izquierda (x, y, z en el marco del torso)   7-9 mano derecha
I_ALT, I_GIRO, I_INCL, I_CAB = 0, 1, 2, 3
I_MI = slice(4, 7)
I_MD = slice(7, 10)

V_MAX = np.array([0.15, 0.8, 0.6, 0.8] + [0.25] * 6)   # m/s o rad/s
A_MAX = np.array([0.5, 2.5, 2.0, 2.5] + [0.8] * 6)     # al arrancar
A_FRENO = A_MAX * 2.5                                  # al soltar el botón

LIM_INF = np.array([0.62, -0.60, -0.15, -0.55])
LIM_SUP = np.array([0.92, 0.60, 0.45, 0.80])
# Caja de alcance de cada mano en el marco del torso (y se refleja para la derecha)
MANO_MIN = np.array([-0.30, 0.10, -0.55])
MANO_MAX = np.array([0.85, 0.95, 1.05])

PARTES = ["MANO_IZQ", "MANO_DER", "CUERPO"]

TECLAS_EJE = {
    p.B3G_UP_ARROW: (0, +1), p.B3G_DOWN_ARROW: (0, -1),
    p.B3G_LEFT_ARROW: (1, +1), p.B3G_RIGHT_ARROW: (1, -1),
    ord("1"): (2, +1), ord("2"): (2, -1),
    ord("3"): (3, +1), ord("4"): (3, -1),
}
TECLAS_CMD = {ord("5"): "PARTE", ord("6"): "HOME", ord("7"): "SALUDO"}


def minimo_jerk(t, T):
    """Interpolación 0 -> 1 con velocidad y aceleración nulas en los extremos."""
    s = np.clip(t / T, 0.0, 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


# ---------------------------------------------------------------------------
# Cinemática: copia del Atlas en un mundo aparte (sin física)
# ---------------------------------------------------------------------------
class Cinematica:
    """Calcula los ángulos de todas las articulaciones para una postura.
    Se usa una copia del robot para que el Atlas de la simulación nunca salte
    mientras se calcula. Para resolver una extremidad sin mover el resto se
    usa un amortiguamiento alto en las demás articulaciones."""

    def __init__(self, datos):
        self.cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(datos, physicsClientId=self.cid)
        self.robot = p.loadURDF(URDF_ATLAS, useFixedBase=True, physicsClientId=self.cid)
        self.n = p.getNumJoints(self.robot, physicsClientId=self.cid)
        info = [p.getJointInfo(self.robot, j, physicsClientId=self.cid) for j in range(self.n)]
        self.lim_inf = np.array([i[8] for i in info])
        self.lim_sup = np.array([i[9] for i in info])
        self.q = np.zeros(self.n)

    def _set(self, juntas, valores):
        for j, v in zip(juntas, valores):
            v = float(np.clip(v, self.lim_inf[j], self.lim_sup[j]))
            self.q[j] = v
            p.resetJointState(self.robot, j, v, physicsClientId=self.cid)

    def colocar_base(self, pos):
        p.resetBasePositionAndOrientation(self.robot, pos, [0, 0, 0, 1],
                                          physicsClientId=self.cid)

    def pose_eslabon(self, eslabon):
        ls = p.getLinkState(self.robot, eslabon, computeForwardKinematics=1,
                            physicsClientId=self.cid)
        return ls[4], ls[5]

    def ik(self, eslabon, juntas, pos, orn=None, semilla=None, iteraciones=6, tol=5e-4):
        """IK de una extremidad: solo cambian las articulaciones 'juntas'.
        Si no converge, reintenta desde una postura 'semilla' y se queda con la
        mejor solución (evita que el brazo quede trabado en una mala postura)."""
        err = self._ik(eslabon, juntas, pos, orn, iteraciones, tol)
        if err > 0.015 and semilla is not None:
            q_prev = [self.q[j] for j in juntas]
            self._set(juntas, semilla)
            err2 = self._ik(eslabon, juntas, pos, orn, iteraciones * 2, tol)
            if err2 < err:
                err = err2
            else:
                self._set(juntas, q_prev)
        return err

    def _ik(self, eslabon, juntas, pos, orn, iteraciones, tol):
        amort = [5.0] * self.n
        for j in juntas:
            amort[j] = 0.01
        err = 1.0
        for _ in range(iteraciones):
            kw = dict(jointDamping=amort, maxNumIterations=50, residualThreshold=1e-5,
                      physicsClientId=self.cid)
            if orn is None:
                q = p.calculateInverseKinematics(self.robot, eslabon, pos, **kw)
            else:
                q = p.calculateInverseKinematics(self.robot, eslabon, pos, orn, **kw)
            self._set(juntas, [q[j] for j in juntas])        # índice de DoF = índice de junta
            err = float(np.linalg.norm(np.array(self.pose_eslabon(eslabon)[0]) - pos))
            if err < tol:
                break
        return err


# ---------------------------------------------------------------------------
# Atlas: postura deseada -> ángulos -> motores
# ---------------------------------------------------------------------------
class Atlas:
    def __init__(self, robot, cin):
        self.robot, self.cin = robot, cin
        self.pies = {
            PIE_IZQ: np.array([POS_ATLAS[0], POS_ATLAS[1] + 0.11, SUELO + ALTURA_TOBILLO]),
            PIE_DER: np.array([POS_ATLAS[0], POS_ATLAS[1] - 0.11, SUELO + ALTURA_TOBILLO]),
        }
        info = [p.getJointInfo(robot, j) for j in range(cin.n)]
        self.fuerzas = [max(i[10], 200.0) for i in info]

        # Postura neutra (brazos abajo con los codos doblados, rodillas flexionadas)
        self.semillas = {
            MANO_IZQ: [-0.3, -1.3, 1.85, 0.5, 0.0, 0.0],
            MANO_DER: [0.3, 1.3, 1.85, -0.5, 0.0, 0.0],
            PIE_IZQ: [0, 0, -0.4, 0.8, -0.4, 0],
            PIE_DER: [0, 0, -0.4, 0.8, -0.4, 0],
        }
        for juntas, k in ((BRAZO_IZQ, MANO_IZQ), (BRAZO_DER, MANO_DER),
                          (PIERNA_IZQ, PIE_IZQ), (PIERNA_DER, PIE_DER)):
            cin._set(juntas, self.semillas[k])
        self.cache = {}                       # soluciones recientes de cada extremidad

        self.home = np.zeros(10)
        self.home[I_ALT] = 0.88
        self.home[I_MI] = self._mano_local(MANO_IZQ)
        self.home[I_MD] = self._mano_local(MANO_DER)
        self.post = self.home.copy()          # postura actual (objetivo de los motores)
        self.vel = np.zeros(10)
        self.anim = None                      # animación en curso: (inicio, keyframes, t0)
        self.limite = False
        self.resolver(self.post, inicial=True)

    # -- marcos de referencia --
    def _base(self, post):
        return [POS_ATLAS[0], POS_ATLAS[1], SUELO + post[I_ALT]]

    def _mano_local(self, eslabon):
        """Posición de la mano en el marco del torso (con la copia en su estado actual)."""
        self.cin.colocar_base(self._base(self.home))
        tp, to = self.cin.pose_eslabon(TORSO)
        mp, _ = self.cin.pose_eslabon(eslabon)
        ip, io = p.invertTransform(tp, to)
        return np.array(p.multiplyTransforms(ip, io, mp, [0, 0, 0, 1])[0])

    # -- cinemática completa de una postura --
    def resolver(self, post, inicial=False):
        """Calcula los ángulos para la postura. Devuelve los errores (pies, manos)."""
        cin = self.cin
        cin.colocar_base(self._base(post))
        cin._set([ESPALDA["bkz"], ESPALDA["bky"], ESPALDA["bkx"]],
                 [post[I_GIRO], post[I_INCL], 0.0])
        cin._set([CUELLO], [post[I_CAB]])

        # Piernas: cada pie se queda plano y fijo sobre la caja
        clave_piernas = (round(post[I_ALT], 6),)
        errs_pies = [self._extremidad(pie, juntas, clave_piernas,
                                      lambda pie=pie: self.pies[pie].tolist(), [0, 0, 0, 1])
                     for pie, juntas in ((PIE_IZQ, PIERNA_IZQ), (PIE_DER, PIERNA_DER))]

        # Manos: el objetivo está en el marco del torso -> se gira con el torso
        tp, to = cin.pose_eslabon(TORSO)
        errs_manos = []
        for eslabon, juntas, idx in ((MANO_IZQ, BRAZO_IZQ, I_MI), (MANO_DER, BRAZO_DER, I_MD)):
            clave = tuple(np.round(np.r_[post[:3], post[idx]], 6))
            objetivo = lambda idx=idx: list(
                p.multiplyTransforms(tp, to, post[idx].tolist(), [0, 0, 0, 1])[0])
            errs_manos.append(self._extremidad(eslabon, juntas, clave, objetivo))
        err_pies = max(errs_pies)

        if inicial:
            p.resetBasePositionAndOrientation(self.robot, self._base(post), [0, 0, 0, 1])
            for j in range(cin.n):
                p.resetJointState(self.robot, j, cin.q[j])
        return err_pies, errs_manos

    def _extremidad(self, eslabon, juntas, clave, objetivo, orn=None):
        """Resuelve una extremidad, reutilizando la solución si los datos de los
        que depende no cambiaron (así un brazo quieto no gasta tiempo de cálculo)."""
        recientes = self.cache.setdefault(eslabon, [])
        for c, q, err in recientes:
            if c == clave:
                self.cin._set(juntas, q)
                return err
        err = self.cin.ik(eslabon, juntas, objetivo(), orn, semilla=self.semillas[eslabon])
        recientes.insert(0, (clave, [self.cin.q[j] for j in juntas], err))
        del recientes[3:]                     # se guardan las 3 últimas
        return err

    def aplicar(self):
        p.setJointMotorControlArray(self.robot, list(range(self.cin.n)), p.POSITION_CONTROL,
                                    targetPositions=self.cin.q.tolist(), forces=self.fuerzas,
                                    positionGains=[0.15] * self.cin.n)

    # -- animaciones automáticas (HOME y SALUDO) --
    def animar(self, keyframes):
        """keyframes: lista de (duración [s], postura destino)."""
        self.anim = [self.post.copy(), list(keyframes), 0.0]
        self.vel[:] = 0

    def saludo(self):
        p0 = self.post.copy()
        arriba = p0.copy()
        arriba[I_MD] = [0.25, -0.45, 0.85]            # mano derecha levantada
        arriba[I_CAB] = 0.15
        izq, der = arriba.copy(), arriba.copy()
        izq[I_MD] = [0.25, -0.25, 0.80]
        der[I_MD] = [0.25, -0.60, 0.80]
        self.animar([(1.2, arriba), (0.45, izq), (0.45, der), (0.45, izq),
                     (0.45, der), (0.45, arriba), (1.2, p0)])

    def _avanzar_animacion(self, dt):
        inicio, frames, t = self.anim
        T, destino = frames[0]
        t += dt
        s = minimo_jerk(t, T)
        self.post = inicio + (destino - inicio) * s
        if t >= T:
            frames.pop(0)
            inicio, t = destino.copy(), 0.0
        self.anim = [inicio, frames, t] if frames else None

    # -- un ciclo de control --
    def actualizar(self, v_deseada, dt):
        """v_deseada: 10 valores en [-1, 1] (fracción de la velocidad máxima)."""
        self.limite = False
        if self.anim is not None:
            if np.any(v_deseada):
                self.anim = None                    # mover a mano cancela la animación
            else:
                self._avanzar_animacion(dt)
                self.resolver(self.post)
                self.aplicar()
                return

        # Rampa de velocidad (arranque suave, frenado más rápido)
        v_obj = np.asarray(v_deseada, float) * V_MAX
        frenando = np.abs(v_obj) < np.abs(self.vel)
        a = np.where(frenando, A_FRENO, A_MAX) * dt
        self.vel += np.clip(v_obj - self.vel, -a, a)
        if not np.any(np.abs(self.vel) > 1e-6):
            self.aplicar()
            return

        nueva = self.post + self.vel * dt
        nueva = self._recortar(nueva)
        err_pies, errs_manos = self.resolver(nueva)

        # Si alguna extremidad no alcanza su objetivo, la postura no avanza
        ok = err_pies < 0.005 and max(errs_manos) < 0.008
        if ok:
            self.post = nueva
        else:
            self.vel[:] = 0
            self.limite = True
            self.resolver(self.post)
        self.aplicar()

    def _recortar(self, post):
        nueva = post.copy()
        rec = np.clip(nueva[:4], LIM_INF, LIM_SUP)
        manos = []
        for idx, signo in ((I_MI, 1), (I_MD, -1)):
            m = nueva[idx] * np.array([1, signo, 1])          # se trabaja como mano izquierda
            m = np.clip(m, MANO_MIN, MANO_MAX)
            if m[0] < 0.36 and m[1] < 0.33:                  # no atravesar el torso
                m[1] = 0.33
            manos.append(m * np.array([1, signo, 1]))
        fuera = (np.any(np.abs(rec - nueva[:4]) > 1e-9) or
                 np.any(np.abs(manos[0] - nueva[I_MI]) > 1e-9) or
                 np.any(np.abs(manos[1] - nueva[I_MD]) > 1e-9))
        nueva[:4], nueva[I_MI], nueva[I_MD] = rec, manos[0], manos[1]
        if fuera:
            self.limite = True
            cambio = np.abs(nueva - post) > 1e-9
            self.vel[cambio] = 0
        return nueva

    def error_pies(self):
        return max(float(np.linalg.norm(np.array(p.getLinkState(self.robot, pie)[4]) - pos))
                   for pie, pos in self.pies.items())


# ---------------------------------------------------------------------------
# Traducción de los 4 ejes de la consola a la postura según la parte activa
# ---------------------------------------------------------------------------
def ejes_a_postura(parte, ejes):
    """ejes = [e1A, e2A, e1B, e2B] (cada uno -1, 0 o 1) -> vector de 10 velocidades."""
    v = np.zeros(10)
    e1a, e2a, e1b, e2b = ejes
    if parte in ("MANO_IZQ", "MANO_DER"):
        idx = I_MI if parte == "MANO_IZQ" else I_MD
        v[idx] = [e1a, e2a, e1b]            # adelante/atrás, izquierda/derecha, subir/bajar
        v[I_GIRO] = e2b                     # girar el torso para alcanzar a los lados
    else:  # CUERPO
        v[I_ALT] = e1a                      # sentadilla
        v[I_GIRO] = e2a                     # girar torso
        v[I_INCL] = -e1b                    # inclinar torso (botón + = enderezar)
        v[I_CAB] = -e2b                     # cabeza (botón + = mirar arriba)
    return v


# ---------------------------------------------------------------------------
# Comunicación con la ESP32
# ---------------------------------------------------------------------------
class EnlaceESP32:
    def __init__(self, puerto, baudios=115200):
        if serial is None:
            raise RuntimeError("Falta pyserial: pip install pyserial")
        self.ser = serial.Serial(puerto, baudios, timeout=0.1)
        time.sleep(2)                        # la ESP32 se reinicia al abrir el puerto
        self.ser.reset_input_buffer()
        self.comandos = queue.Queue()
        self.ejes = [0, 0, 0, 0]
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
                        self.ejes = [max(-1, min(1, x)) for x in v]
                        self.t_ultimo = time.time()
                except ValueError:
                    pass
            elif linea.startswith("CMD:"):
                self.comandos.put(linea[4:])
                self.t_ultimo = time.time()
            elif linea:
                print(f"[ESP32] {linea}")

    def leer_ejes(self):
        if time.time() - self.t_ultimo > WATCHDOG_S:
            return [0, 0, 0, 0]
        return list(self.ejes)

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
def crear_escena(datos, escena):
    p.setAdditionalSearchPath(datos)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT_SIM)
    if escena == "botlab":
        # Laboratorio del demo atlas.py (viene con el eje Y hacia arriba: se gira)
        objs = p.loadSDF("botlab/botlab.sdf", globalScaling=2.0)
        y_a_z = p.getQuaternionFromEuler([np.pi / 2, 0, np.pi / 2])
        for o in objs:
            pos, orn = p.getBasePositionAndOrientation(o)
            pos, orn = p.multiplyTransforms([0, 0, 0], y_a_z, pos, orn)
            p.resetBasePositionAndOrientation(o, pos, orn)
    else:
        p.loadURDF("plane.urdf", [0, 0, PISO_LAB], useFixedBase=True)

    caja = p.loadURDF("boston_box.urdf", [*POS_ATLAS, PISO_LAB + LADO_CAJA / 2],
                      useFixedBase=True)
    p.changeVisualShape(caja, -1, rgbaColor=[0.25, 0.6, 0.85, 1])

    robot = p.loadURDF(URDF_ATLAS, [*POS_ATLAS, SUELO + 0.95])
    return robot


# ---------------------------------------------------------------------------
# Simulador: un ciclo de control a partir de los comandos de la consola
# ---------------------------------------------------------------------------
class Simulador:
    def __init__(self, robot, atlas, informar):
        self.robot, self.atlas, self.informar = robot, atlas, informar
        self.parte = 0
        self.t_aviso = -10.0
        self.t = 0.0
        # La pelvis se sostiene con una restricción que sigue la altura pedida
        # (el Atlas queda "fijo" en su sitio, pero las piernas soportan la pose)
        self.ancla = p.createConstraint(robot, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0],
                                        [0, 0, 0], atlas._base(atlas.post))
        p.changeConstraint(self.ancla, atlas._base(atlas.post), maxForce=20000)
        informar(f"PARTE:{PARTES[self.parte]}")

    def paso_control(self, ejes, cmds, dt):
        for c in cmds:
            print(f"> Comando: {c}")
            if c == "PARTE":
                self.parte = (self.parte + 1) % len(PARTES)
                self.atlas.vel[:] = 0
                self.informar(f"PARTE:{PARTES[self.parte]}")
            elif c == "HOME":
                self.atlas.animar([(2.0, self.atlas.home.copy())])
                self.informar("ANIM:HOME")
            elif c == "SALUDO":
                self.atlas.saludo()
                self.informar("ANIM:SALUDO")

        animando = self.atlas.anim is not None
        self.atlas.actualizar(ejes_a_postura(PARTES[self.parte], ejes), dt)
        p.changeConstraint(self.ancla, self.atlas._base(self.atlas.post), maxForce=20000)
        if animando and self.atlas.anim is None:
            self.informar("ANIM:FIN")
        self.t += dt
        if self.atlas.limite and self.t - self.t_aviso > 1.0:   # máximo un aviso por segundo
            self.informar("LIMITE")
            self.t_aviso = self.t


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Atlas controlado por ESP32")
    ap.add_argument("--puerto", default="COM4")
    ap.add_argument("--sin_esp32", action="store_true", help="Usar solo el teclado")
    ap.add_argument("--datos", default=os.path.join("pybullet_robots", "data"),
                    help="Carpeta 'data' del repositorio pybullet_robots")
    ap.add_argument("--escena", choices=["botlab", "simple"], default="botlab")
    args = ap.parse_args()

    if not os.path.isfile(os.path.join(args.datos, URDF_ATLAS)):
        raise SystemExit(f"No encuentro el modelo del Atlas en:\n  "
                         f"{os.path.join(args.datos, URDF_ATLAS)}\n"
                         "Clona pybullet_robots y usa --datos <ruta>/pybullet_robots/data")

    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = crear_escena(args.datos, args.escena)
    atlas = Atlas(robot, Cinematica(args.datos))
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.resetDebugVisualizerCamera(2.5, 120, -10, [*POS_ATLAS, SUELO + 1.0])

    enlace = None
    if not args.sin_esp32:
        enlace = EnlaceESP32(args.puerto)
        print(f"Conectado a la ESP32 en {args.puerto}")

    def informar(estado):
        print(f"  Estado: {estado}")
        if enlace:
            enlace.enviar_estado(estado)

    sim = Simulador(robot, atlas, informar)
    print("Listo. Mantén presionados los botones para mover al Atlas.")

    dt_ctrl = DT_SIM * PASOS_CONTROL
    texto_id = -1
    t0 = time.time()
    paso = 0
    try:
        while p.isConnected():
            if paso % PASOS_CONTROL == 0:
                ejes = enlace.leer_ejes() if enlace else [0, 0, 0, 0]
                cmds = []
                if enlace:
                    while not enlace.comandos.empty():
                        cmds.append(enlace.comandos.get())
                teclas = p.getKeyboardEvents()
                for t, (eje, s) in TECLAS_EJE.items():
                    if t in teclas and teclas[t] & p.KEY_IS_DOWN:
                        ejes[eje] = s
                for t, c in TECLAS_CMD.items():
                    if t in teclas and teclas[t] & p.KEY_WAS_TRIGGERED:
                        cmds.append(c)

                sim.paso_control(ejes, cmds, dt_ctrl)

                if paso % (PASOS_CONTROL * 15) == 0:
                    a = atlas.post
                    txt = (f"{PARTES[sim.parte]} | altura {a[I_ALT]:.2f} m  "
                           f"giro {np.degrees(a[I_GIRO]):.0f}  incl {np.degrees(a[I_INCL]):.0f}  "
                           f"cabeza {np.degrees(a[I_CAB]):.0f}")
                    texto_id = p.addUserDebugText(
                        txt, [POS_ATLAS[0], POS_ATLAS[1] + 0.9, SUELO + 2.2], [0, 0, 0], 1.2,
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
