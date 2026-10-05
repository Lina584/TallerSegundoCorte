/*
 * Taller Segundo Corte — Punto A
 * Consola de mandos con ESP32 para mover un enjambre de drones
 * (gym-pybullet-drones) entre los lugares A, B y C.
 *
 * Comunicación: Serial USB a 115200 baudios.
 *   ESP32 -> PC : "CMD:<comando>\n"   (al presionar un pulsador)
 *   PC -> ESP32 : "EST:<estado>\n"    (realimentación de la simulación)
 *
 * Conexión: cada pulsador va entre su GPIO y GND (se usa INPUT_PULLUP,
 * así que NO hacen falta resistencias externas). Presionado = LOW.
 *
 * LED de estado (GPIO 2, LED azul de la placa):
 *   apagado    -> drones en tierra
 *   parpadeo   -> drones moviéndose
 *   encendido  -> drones en el aire, quietos (despegados o llegaron a A/B/C)
 */

// ----------------------- Configuración -----------------------
const int NUM_BOTONES = 6;

// Se evitan GPIO 0, 2, 12 y 15 (pines de arranque) y 34–39 (sin pull-up interno)
const int PINES[NUM_BOTONES] = {13, 14, 27, 32, 18, 33};

const char* COMANDOS[NUM_BOTONES] = {
  "A",          // GPIO 13 -> ir al lugar A
  "B",          // GPIO 14 -> ir al lugar B
  "C",          // GPIO 27 -> ir al lugar C
  "RUTA",       // GPIO 32 -> recorrido automático A -> B -> C
  "DESPEGAR",   // GPIO 18
  "ATERRIZAR"   // GPIO 33
};

const int LED_ESTADO = 2;
const unsigned long DEBOUNCE_MS = 40;
const unsigned long PARPADEO_MS = 200;

// ------------------------ Variables --------------------------
bool estadoEstable[NUM_BOTONES];     // último valor ya filtrado
bool lecturaPrevia[NUM_BOTONES];     // última lectura cruda
unsigned long tCambio[NUM_BOTONES];  // instante del último cambio crudo

enum ModoLed { LED_APAGADO, LED_PARPADEO, LED_ENCENDIDO };
ModoLed modoLed = LED_APAGADO;
unsigned long tParpadeo = 0;
bool ledOn = false;

String bufferRx = "";

// --------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_ESTADO, OUTPUT);
  digitalWrite(LED_ESTADO, LOW);

  for (int i = 0; i < NUM_BOTONES; i++) {
    pinMode(PINES[i], INPUT_PULLUP);
    estadoEstable[i] = HIGH;  // HIGH = suelto
    lecturaPrevia[i] = HIGH;
    tCambio[i] = 0;
  }
  Serial.println("INFO:Consola de drones lista");
}

// Lee los pulsadores con antirrebote por software y envía el comando
// una sola vez por pulsación (flanco de bajada).
void leerBotones() {
  unsigned long ahora = millis();
  for (int i = 0; i < NUM_BOTONES; i++) {
    bool lectura = digitalRead(PINES[i]);

    if (lectura != lecturaPrevia[i]) {   // hubo cambio: reinicia el temporizador
      tCambio[i] = ahora;
      lecturaPrevia[i] = lectura;
    }

    if ((ahora - tCambio[i]) > DEBOUNCE_MS && lectura != estadoEstable[i]) {
      estadoEstable[i] = lectura;
      if (estadoEstable[i] == LOW) {     // se acaba de presionar
        Serial.print("CMD:");
        Serial.println(COMANDOS[i]);
      }
    }
  }
}

// Interpreta los mensajes de estado que envía la simulación.
void procesarEstado(const String& linea) {
  if (!linea.startsWith("EST:")) return;
  String est = linea.substring(4);

  if (est == "MOVIENDO") {
    modoLed = LED_PARPADEO;
  } else if (est == "EN_TIERRA") {
    modoLed = LED_APAGADO;
  } else if (est == "EN_AIRE" || est.startsWith("LLEGO:")) {
    modoLed = LED_ENCENDIDO;
  }
}

// Recepción no bloqueante: arma líneas terminadas en '\n'.
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
  switch (modoLed) {
    case LED_APAGADO:   digitalWrite(LED_ESTADO, LOW);  break;
    case LED_ENCENDIDO: digitalWrite(LED_ESTADO, HIGH); break;
    case LED_PARPADEO:
      if (millis() - tParpadeo >= PARPADEO_MS) {
        tParpadeo = millis();
        ledOn = !ledOn;
        digitalWrite(LED_ESTADO, ledOn);
      }
      break;
  }
}

void loop() {
  leerBotones();
  leerSerial();
  actualizarLed();
}
