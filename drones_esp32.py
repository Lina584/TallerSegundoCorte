"""
Taller Segundo Corte — Punto A
Mover un enjambre de drones entre los lugares A, B y C con el control
gestionado desde una ESP32 (consola de pulsadores por Serial USB).

Simulación: gym-pybullet-drones (CtrlAviary + DSLPIDControl).

Uso:
    python drones_esp32.py --puerto COM5          # Windows
    python drones_esp32.py --puerto /dev/ttyUSB0  # Linux
    python drones_esp32.py --sin_esp32            # prueba solo con teclado

Teclado (en la ventana de PyBullet, sirve de respaldo):
    1 = A   2 = B   3 = C   4 = Ruta A->B->C   5 = Despegar   6 = Aterrizar
"""

import argparse
import queue
import threading
import time
from collections import deque

import numpy as np
import pybullet as p

try:
    import serial
except ImportError:
    serial = None

from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync

# ---------------------------------------------------------------------------
# Parámetros del escenario
# ---------------------------------------------------------------------------
LUGARES = {                      # centro de la formación en cada lugar (x, y)
    "A": np.array([0.0, 0.0]),
    "B": np.array([2.0, 0.0]),
    "C": np.array([1.0, -1.5]),
}
ALT_TIERRA = 0.10                # altura de reposo [m]
ALT_VUELO = 1.00                 # altura de crucero [m]
VEL_CRUCERO = 0.5                # velocidad media de cada tramo [m/s]
T_MIN_TRAMO = 2.0                # duración mínima de un tramo [s]
SEPARACION = 0.35                # distancia entre drones de la formación [m]

TECLAS = {ord("1"): "A", ord("2"): "B", ord("3"): "C",
          ord("4"): "RUTA", ord("5"): "DESPEGAR", ord("6"): "ATERRIZAR"}


# ---------------------------------------------------------------------------
# Formación: rejilla cuadrada centrada en (0, 0)
# ---------------------------------------------------------------------------
def offsets_formacion(n, sep):
    lado = int(np.ceil(np.sqrt(n)))
    offs = []
    for k in range(n):
        fila, col = divmod(k, lado)
        offs.append([(col - (lado - 1) / 2) * sep,
                     (fila - (lado - 1) / 2) * sep,
                     0.0])
    return np.array(offs)


# ---------------------------------------------------------------------------
# Trayectoria de mínimo jerk entre dos puntos (movimiento fluido:
# arranca y frena con velocidad y aceleración cero)
# ---------------------------------------------------------------------------
def minimo_jerk(inicio, fin, t, T):
    tau = np.clip(t / T, 0.0, 1.0)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    pos = inicio + (fin - inicio) * s
    vel = (fin - inicio) * ds
    return pos, vel


# ---------------------------------------------------------------------------
# Planificador: convierte comandos en una cola de tramos (waypoints)
# ---------------------------------------------------------------------------
class Planificador:
    def __init__(self, centro_inicial):
        self.centro = centro_inicial.astype(float)      # objetivo actual
        self.ultimo_planeado = self.centro.copy()       # fin del último tramo en cola
        self.cola = deque()                             # (destino, etiqueta)
        self.tramo = None                               # tramo en ejecución
        self.estado = "EN_TIERRA"

    def _agregar(self, destino, etiqueta):
        self.cola.append((destino, etiqueta))
        self.ultimo_planeado = destino

    def _en_tierra_planeado(self):
        return self.ultimo_planeado[2] < ALT_VUELO - 0.05

    def _despegar(self):
        if self._en_tierra_planeado():
            d = self.ultimo_planeado.copy()
            d[2] = ALT_VUELO
            self._agregar(d, "EN_AIRE")

    def _ir_a(self, lugar):
        self._despegar()                                # despega solo si hace falta
        xy = LUGARES[lugar]
        self._agregar(np.array([xy[0], xy[1], ALT_VUELO]), f"LLEGO:{lugar}")

    def comando(self, cmd):
        if cmd in LUGARES:
            self._ir_a(cmd)
        elif cmd == "RUTA":
            for lugar in ("A", "B", "C"):
                self._ir_a(lugar)
        elif cmd == "DESPEGAR":
            self._despegar()
        elif cmd == "ATERRIZAR":
            if not self._en_tierra_planeado():
                d = self.ultimo_planeado.copy()
                d[2] = ALT_TIERRA
                self._agregar(d, "EN_TIERRA")
        else:
            print(f"[!] Comando desconocido: {cmd}")
            return False
        return True

    def actualizar(self, t):
        """Devuelve (centro_objetivo, vel_objetivo, eventos)."""
        eventos = []
        if self.tramo is None and self.cola:
            destino, etiqueta = self.cola.popleft()
            dist = np.linalg.norm(destino - self.centro)
            T = max(T_MIN_TRAMO, dist / VEL_CRUCERO)
            self.tramo = (self.centro.copy(), destino, t, T, etiqueta)
            if self.estado != "MOVIENDO":
                self.estado = "MOVIENDO"
                eventos.append("MOVIENDO")

        if self.tramo is None:
            return self.centro, np.zeros(3), eventos

        inicio, fin, t0, T, etiqueta = self.tramo
        pos, vel = minimo_jerk(inicio, fin, t - t0, T)
        if t - t0 >= T:
            self.centro = fin.copy()
            self.tramo = None
            if not self.cola:                 # solo se reporta al quedar quieto
                self.estado = etiqueta
                eventos.append(etiqueta)
            elif etiqueta.startswith("LLEGO"):
                eventos.append(etiqueta)      # paso intermedio de la ruta
            return self.centro, np.zeros(3), eventos
        return pos, vel, eventos


# ---------------------------------------------------------------------------
# Comunicación con la ESP32
# ---------------------------------------------------------------------------
class EnlaceESP32:
    def __init__(self, puerto, baudios=115200):
        if serial is None:
            raise RuntimeError("Falta pyserial: pip install pyserial")
        self.ser = serial.Serial(puerto, baudios, timeout=0.1)
        time.sleep(2)                          # la ESP32 se reinicia al abrir el puerto
        self.ser.reset_input_buffer()
        self.comandos = queue.Queue()
        self.lock = threading.Lock()
        self.activo = True
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        while self.activo:
            try:
                linea = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException:
                print("[!] Se perdió la conexión con la ESP32")
                break
            if linea.startswith("CMD:"):
                self.comandos.put(linea[4:])
            elif linea:
                print(f"[ESP32] {linea}")

    def enviar_estado(self, estado):
        with self.lock:
            self.ser.write(f"EST:{estado}\n".encode())

    def cerrar(self):
        self.activo = False
        self.ser.close()


# ---------------------------------------------------------------------------
# Marcadores visuales de A, B y C en la simulación
# ---------------------------------------------------------------------------
def dibujar_lugares(cliente):
    colores = {"A": [1, 0, 0], "B": [0, 0.6, 0], "C": [0, 0, 1]}
    for nombre, xy in LUGARES.items():
        p.addUserDebugLine([xy[0], xy[1], 0], [xy[0], xy[1], ALT_VUELO],
                           colores[nombre], lineWidth=2, physicsClientId=cliente)
        p.addUserDebugText(nombre, [xy[0], xy[1], ALT_VUELO + 0.15],
                           colores[nombre], textSize=2, physicsClientId=cliente)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Drones A-B-C controlados por ESP32")
    ap.add_argument("--puerto", default="COM5", help="Puerto serie de la ESP32")
    ap.add_argument("--sin_esp32", action="store_true", help="Usar solo el teclado")
    ap.add_argument("--num_drones", type=int, default=9)
    ap.add_argument("--gui", type=int, default=1)
    ap.add_argument("--duracion", type=float, default=0,
                    help="Segundos de simulación (0 = hasta cerrar la ventana)")
    ap.add_argument("--demo", action="store_true",
                    help="Envía la ruta A->B->C automáticamente al iniciar")
    args = ap.parse_args()

    n = args.num_drones
    offs = offsets_formacion(n, SEPARACION)
    centro0 = np.array([*LUGARES["A"], ALT_TIERRA])
    init_xyzs = centro0 + offs
    init_rpys = np.zeros((n, 3))

    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=n,
                     initial_xyzs=init_xyzs, initial_rpys=init_rpys,
                     physics=Physics.PYB, pyb_freq=240, ctrl_freq=48,
                     gui=bool(args.gui), obstacles=True)
    cliente = env.getPyBulletClient()
    ctrl = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(n)]
    if args.gui:
        dibujar_lugares(cliente)
        p.resetDebugVisualizerCamera(4.0, 35, -35, [1.0, -0.5, 0.3],
                                     physicsClientId=cliente)

    enlace = None
    if not args.sin_esp32:
        enlace = EnlaceESP32(args.puerto)
        enlace.enviar_estado("EN_TIERRA")
        print(f"Conectado a la ESP32 en {args.puerto}")

    plan = Planificador(centro0)
    if args.demo:
        plan.comando("RUTA")
    accion = np.zeros((n, 4))
    pasos_max = int(args.duracion * env.CTRL_FREQ) if args.duracion > 0 else None
    inicio = time.time()
    i = 0
    print("Listo. Esperando comandos (botones de la ESP32 o teclas 1-6)...")

    try:
        while pasos_max is None or i < pasos_max:
            # 1) Comandos entrantes
            cmds = []
            if enlace:
                while not enlace.comandos.empty():
                    cmds.append(enlace.comandos.get())
            if args.gui:
                for tecla, estado in p.getKeyboardEvents(physicsClientId=cliente).items():
                    if tecla in TECLAS and estado & p.KEY_WAS_TRIGGERED:
                        cmds.append(TECLAS[tecla])
            for c in cmds:
                if plan.comando(c):
                    print(f"> Comando: {c}")

            # 2) Trayectoria de la formación
            t = i * env.CTRL_TIMESTEP
            centro, vel, eventos = plan.actualizar(t)
            for e in eventos:
                print(f"  Estado: {e}")
                if enlace:
                    enlace.enviar_estado(e)

            # 3) Simulación + control PID de cada dron
            obs, _, _, _, _ = env.step(accion)
            for j in range(n):
                accion[j, :], _, _ = ctrl[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs[j],
                    target_pos=centro + offs[j],
                    target_vel=vel,
                    target_rpy=init_rpys[j],
                )

            if args.gui:
                sync(i, inicio, env.CTRL_TIMESTEP)
            i += 1
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if enlace:
            enlace.cerrar()
        env.close()


if __name__ == "__main__":
    main()
