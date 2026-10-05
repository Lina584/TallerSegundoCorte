/*
 * Taller Segundo Corte — Punto B
 * Consola de mandos con ESP32 para el robot Baxter (PyBullet).
 * Mueve la pinza de forma fluida en X, Y, Z, gira la muñeca,
 * abre/cierra la pinza para coger y mover un objeto, y cambia de brazo.
 *
 * Solo 6 pulsadores: 4 direccionales + pinza + modo.
 *
 *   GPIO 13  (+) eje 1   | modo XY: adelante (+X)   | modo Z: subir (+Z)
 *   GPIO 14  (-) eje 1   | modo XY: atrás    (-X)   | modo Z: bajar (-Z)
 *   GPIO 27  (+) eje 2   | modo XY: izquierda (+Y)  | modo Z: girar muñeca +
 *   GPIO 32  (-) eje 2   | modo XY: derecha  (-Y)   | modo Z: girar muñeca -
 *   GPIO 18  pinza       | toque: abrir/cerrar      | mantener 0,8 s: HOME
 *   GPIO 33  modo        | toque: cambia XY <-> Z   | mantener 0,8 s: cambiar de brazo
 *
 * Los direccionales funcionan MIENTRAS SE MANTIENEN PRESIONADOS: la ESP32 envía
 * la dirección y el PC mueve la pinza con aceleración suave.
 *
 * Comunicación: Serial USB a 115200 baudios.
 *   ESP32 -> PC : "V:x,y,z,g\n"   dirección de cada eje (-1, 0 o 1), cada 50 ms
 *                 "CMD:PINZA\n" | "CMD:HOME\n" | "CMD:BRAZO\n"
 *                 "MODO:XY\n" | "MODO:Z\n"   (informativo)
 *   PC -> ESP32 : "EST:OBJETO:AGARRADO" | "EST:OBJETO:LIBRE" | "EST:LIMITE" | ...
 *
 * Conexión: cada pulsador entre su GPIO y GND (INPUT_PULLUP, sin resistencias).
 * LED de la placa (GPIO 2): apagado = modo XY, encendido = modo Z/muñeca,
 *                           3 destellos = se llegó a un límite.
 * LED externo opcional (GPIO 19 + resistencia de 220 ohm a GND):
 *                           encendido = objeto agarrado.
 */

// ----------------------- Configuración -----------------------
enum { B_MAS1, B_MENOS1, B_MAS2, B_MENOS2, B_PINZA, B_MODO, NUM_BOTONES };
const int PINES[NUM_BOTONES] = {13, 14, 27, 32, 18, 33};

const int LED_MODO = 2;
const int LED_OBJETO = 19;
const unsigned long DEBOUNCE_MS = 30;
const unsigned long PULSACION_LARGA_MS = 800;
const unsigned long PERIODO_ENVIO_MS = 50;      // 20 mensajes por segundo

// ------------------------ Variables --------------------------
bool estable[NUM_BOTONES];          // true = presionado (ya filtrado)
bool lecturaPrevia[NUM_BOTONES];
unsigned long tCambio[NUM_BOTONES];
unsigned long tPresion[NUM_BOTONES];
bool largaEnviada[NUM_BOTONES];

bool modoZ = false;                 // false = XY, true = Z/muñeca
int velPrevia[4] = {0, 0, 0, 0};
unsigned long tEnvio = 0;

int destellos = 0;                  // destellos pendientes del LED (aviso de límite)
unsigned long tDestello = 0;
String bufferRx = "";

// --------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_MODO, OUTPUT);
  pinMode(LED_OBJETO, OUTPUT);
  for (int i = 0; i < NUM_BOTONES; i++) {
    pinMode(PINES[i], INPUT_PULLUP);
    estable[i] = false;
    lecturaPrevia[i] = false;
    tCambio[i] = 0;
    largaEnviada[i] = false;
  }
  Serial.println("INFO:Consola Baxter lista");
  Serial.println("MODO:XY");
}

// Acción de un toque corto (se ejecuta al soltar el botón)
void toqueCorto(int b) {
  if (b == B_PINZA) {
    Serial.println("CMD:PINZA");
  } else if (b == B_MODO) {
    modoZ = !modoZ;
    Serial.println(modoZ ? "MODO:Z" : "MODO:XY");
  }
}

// Acción de una pulsación larga (se ejecuta al cumplir 0,8 s presionado)
void pulsacionLarga(int b) {
  if (b == B_PINZA) Serial.println("CMD:HOME");
  else if (b == B_MODO) Serial.println("CMD:BRAZO");
}

void leerBotones() {
  unsigned long ahora = millis();
  for (int i = 0; i < NUM_BOTONES; i++) {
    bool lectura = (digitalRead(PINES[i]) == LOW);    // LOW = presionado

    if (lectura != lecturaPrevia[i]) {
      tCambio[i] = ahora;
      lecturaPrevia[i] = lectura;
    }
    if ((ahora - tCambio[i]) > DEBOUNCE_MS && lectura != estable[i]) {
      estable[i] = lectura;
      if (estable[i]) {                                // se presionó
        tPresion[i] = ahora;
        largaEnviada[i] = false;
      } else if (!largaEnviada[i]) {                   // se soltó antes de 0,8 s
        toqueCorto(i);
      }
    }
    // Pulsación larga: se dispara sin esperar a soltar el botón
    if (estable[i] && !largaEnviada[i] && (i == B_PINZA || i == B_MODO) &&
        (ahora - tPresion[i]) >= PULSACION_LARGA_MS) {
      largaEnviada[i] = true;
      pulsacionLarga(i);
    }
  }
}

// Calcula la dirección de cada eje según el modo y la envía al PC
void enviarVelocidad() {
  int eje1 = (int)estable[B_MAS1] - (int)estable[B_MENOS1];   // los dos a la vez = 0
  int eje2 = (int)estable[B_MAS2] - (int)estable[B_MENOS2];
  int v[4] = {0, 0, 0, 0};                                     // x, y, z, giro
  if (modoZ) { v[2] = eje1; v[3] = eje2; }
  else       { v[0] = eje1; v[1] = eje2; }

  bool cambio = false;
  for (int k = 0; k < 4; k++) cambio |= (v[k] != velPrevia[k]);

  // Se envía apenas cambia (respuesta inmediata) y además cada 50 ms;
  // si el PC deja de recibir mensajes por 0,5 s, detiene el brazo.
  if (cambio || millis() - tEnvio >= PERIODO_ENVIO_MS) {
    tEnvio = millis();
    Serial.printf("V:%d,%d,%d,%d\n", v[0], v[1], v[2], v[3]);
    for (int k = 0; k < 4; k++) velPrevia[k] = v[k];
  }
}

void procesarEstado(const String& linea) {
  if (!linea.startsWith("EST:")) return;
  String est = linea.substring(4);
  if (est == "OBJETO:AGARRADO")   digitalWrite(LED_OBJETO, HIGH);
  else if (est == "OBJETO:LIBRE") digitalWrite(LED_OBJETO, LOW);
  else if (est == "LIMITE")       destellos = 6;    // 3 destellos (encender + apagar)
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
  if (destellos > 0) {                               // aviso de límite
    if (millis() - tDestello >= 80) {
      tDestello = millis();
      destellos--;
      digitalWrite(LED_MODO, destellos % 2);
    }
  } else {
    digitalWrite(LED_MODO, modoZ ? HIGH : LOW);       // indica el modo
  }
}

void loop() {
  leerBotones();
  enviarVelocidad();
  leerSerial();
  actualizarLed();
}
