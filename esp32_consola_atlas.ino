/*
 * Taller Segundo Corte — Punto C
 * Consola de mandos con ESP32 para el robot Atlas (PyBullet).
 * Mueve cada mano en X/Y/Z, hace sentadillas, gira e inclina el torso,
 * mueve la cabeza y lanza animaciones (saludo y home). Movimiento fluido.
 *
 * Mismos 6 pulsadores y pines de los puntos A y B:
 *
 *   GPIO 13  eje 1 (+)
 *   GPIO 14  eje 1 (-)
 *   GPIO 27  eje 2 (+)
 *   GPIO 32  eje 2 (-)
 *   GPIO 18  plano   | toque: cambia plano A <-> B   | mantener 0,8 s: HOME
 *   GPIO 33  parte   | toque: mano izq -> mano der -> cuerpo   | mantener 0,8 s: SALUDO
 *
 *   Qué mueve cada eje (lo decide el PC según la parte activa):
 *     Mano   plano A: eje 1 adelante/atrás, eje 2 izquierda/derecha
 *            plano B: eje 1 subir/bajar,    eje 2 girar el torso
 *     Cuerpo plano A: eje 1 sentadilla,     eje 2 girar el torso
 *            plano B: eje 1 enderezar/inclinar el torso, eje 2 cabeza arriba/abajo
 *
 * Los direccionales funcionan MIENTRAS SE MANTIENEN PRESIONADOS.
 *
 * Comunicación: Serial USB a 115200 baudios.
 *   ESP32 -> PC : "V:a1,a2,b1,b2\n"  dirección de los 4 ejes (-1, 0 o 1), cada 50 ms
 *                 (a1,a2 = plano A; b1,b2 = plano B; el plano inactivo va en 0)
 *                 "CMD:PARTE\n" | "CMD:HOME\n" | "CMD:SALUDO\n" | "PLANO:A\n" | "PLANO:B\n"
 *   PC -> ESP32 : "EST:PARTE:MANO_IZQ" | "EST:PARTE:MANO_DER" | "EST:PARTE:CUERPO"
 *                 "EST:LIMITE" | "EST:ANIM:SALUDO" | "EST:ANIM:HOME" | "EST:ANIM:FIN"
 *
 * Conexión: cada pulsador entre su GPIO y GND (INPUT_PULLUP, sin resistencias).
 * LED de la placa (GPIO 2): apagado = plano A, encendido = plano B.
 *   Al cambiar de parte parpadea 1 (mano izq), 2 (mano der) o 3 veces (cuerpo).
 *   Al llegar a un límite da 5 destellos rápidos.
 * LED externo opcional (GPIO 19 + 220 ohm a GND): encendido durante una animación.
 */

// ----------------------- Configuración -----------------------
enum { B_MAS1, B_MENOS1, B_MAS2, B_MENOS2, B_PLANO, B_PARTE, NUM_BOTONES };
const int PINES[NUM_BOTONES] = {13, 14, 27, 32, 18, 33};

const int LED_PLANO = 2;
const int LED_ANIM = 19;
const unsigned long DEBOUNCE_MS = 30;
const unsigned long PULSACION_LARGA_MS = 800;
const unsigned long PERIODO_ENVIO_MS = 50;

// ------------------------ Variables --------------------------
bool estable[NUM_BOTONES];
bool lecturaPrevia[NUM_BOTONES];
unsigned long tCambio[NUM_BOTONES];
unsigned long tPresion[NUM_BOTONES];
bool largaEnviada[NUM_BOTONES];

bool planoB = false;
int velPrevia[4] = {0, 0, 0, 0};
unsigned long tEnvio = 0;

// Secuencia de parpadeos del LED: cantidad de cambios pendientes y su duración
int cambiosLed = 0;
unsigned long periodoLed = 0;
unsigned long tLed = 0;
String bufferRx = "";

// --------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PLANO, OUTPUT);
  pinMode(LED_ANIM, OUTPUT);
  for (int i = 0; i < NUM_BOTONES; i++) {
    pinMode(PINES[i], INPUT_PULLUP);
    estable[i] = false;
    lecturaPrevia[i] = false;
    tCambio[i] = 0;
    largaEnviada[i] = false;
  }
  Serial.println("INFO:Consola Atlas lista");
  Serial.println("PLANO:A");
}

// Programa 'n' parpadeos del LED de 'ms' milisegundos cada mitad
void parpadear(int n, unsigned long ms) {
  cambiosLed = 2 * n;
  periodoLed = ms;
  tLed = millis();
  digitalWrite(LED_PLANO, LOW);
}

void toqueCorto(int b) {
  if (b == B_PLANO) {
    planoB = !planoB;
    Serial.println(planoB ? "PLANO:B" : "PLANO:A");
  } else if (b == B_PARTE) {
    Serial.println("CMD:PARTE");
  }
}

void pulsacionLarga(int b) {
  if (b == B_PLANO) Serial.println("CMD:HOME");
  else if (b == B_PARTE) Serial.println("CMD:SALUDO");
}

void leerBotones() {
  unsigned long ahora = millis();
  for (int i = 0; i < NUM_BOTONES; i++) {
    bool lectura = (digitalRead(PINES[i]) == LOW);     // LOW = presionado

    if (lectura != lecturaPrevia[i]) {
      tCambio[i] = ahora;
      lecturaPrevia[i] = lectura;
    }
    if ((ahora - tCambio[i]) > DEBOUNCE_MS && lectura != estable[i]) {
      estable[i] = lectura;
      if (estable[i]) {
        tPresion[i] = ahora;
        largaEnviada[i] = false;
      } else if (!largaEnviada[i]) {
        toqueCorto(i);
      }
    }
    if (estable[i] && !largaEnviada[i] && (i == B_PLANO || i == B_PARTE) &&
        (ahora - tPresion[i]) >= PULSACION_LARGA_MS) {
      largaEnviada[i] = true;
      pulsacionLarga(i);
    }
  }
}

void enviarVelocidad() {
  int eje1 = (int)estable[B_MAS1] - (int)estable[B_MENOS1];
  int eje2 = (int)estable[B_MAS2] - (int)estable[B_MENOS2];
  int v[4] = {0, 0, 0, 0};
  if (planoB) { v[2] = eje1; v[3] = eje2; }
  else        { v[0] = eje1; v[1] = eje2; }

  bool cambio = false;
  for (int k = 0; k < 4; k++) cambio |= (v[k] != velPrevia[k]);

  // Se envía apenas cambia y además cada 50 ms (el PC detiene al robot
  // si deja de recibir mensajes por 0,5 s)
  if (cambio || millis() - tEnvio >= PERIODO_ENVIO_MS) {
    tEnvio = millis();
    Serial.printf("V:%d,%d,%d,%d\n", v[0], v[1], v[2], v[3]);
    for (int k = 0; k < 4; k++) velPrevia[k] = v[k];
  }
}

void procesarEstado(const String& linea) {
  if (!linea.startsWith("EST:")) return;
  String est = linea.substring(4);
  if (est == "PARTE:MANO_IZQ")      parpadear(1, 200);
  else if (est == "PARTE:MANO_DER") parpadear(2, 200);
  else if (est == "PARTE:CUERPO")   parpadear(3, 200);
  else if (est == "LIMITE")         parpadear(5, 60);
  else if (est.startsWith("ANIM:")) digitalWrite(LED_ANIM, est == "ANIM:FIN" ? LOW : HIGH);
}

void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      bufferRx.trim();
      procesarEstado(bufferRx);
      bufferRx = "";
    } else if (bufferRx.length() < 64) {
      bufferRx += c;
    }
  }
}

void actualizarLed() {
  if (cambiosLed > 0) {                           // secuencia de parpadeos en curso
    if (millis() - tLed >= periodoLed) {
      tLed = millis();
      cambiosLed--;
      digitalWrite(LED_PLANO, cambiosLed % 2);
    }
  } else {
    digitalWrite(LED_PLANO, planoB ? HIGH : LOW);
  }
}

void loop() {
  leerBotones();
  enviarVelocidad();
  leerSerial();
  actualizarLed();
}
