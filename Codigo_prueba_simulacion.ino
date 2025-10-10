/* ESP32 - Debug y pequeño delay para asegurar lectura completa del comando
   - Mantiene buffer circular de 100 muestras (relleno en setup o generación periódica)
   - Responde a "GET100" (sin depender de \n enviado por la app)
   - Añade logs por Serial (USB) para depurar recepción y envío
*/

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int BUFFER_SIZE = 100;
float bufferTension[BUFFER_SIZE];
float bufferCorriente[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

unsigned long lastGenTime = 0;
const unsigned long INTERVAL_MS = 1000; // si quieres regenerar, mantén esto

void setup() {
  Serial.begin(115200);
  SerialBT.begin("KEDI_ESP32");
  randomSeed(esp_random());
  Serial.println("ESP32 listo (Bluetooth SPP).");

  // Opcional: llenar buffer inicial (si quieres datos sin esperar generación)
  for (int i = 0; i < BUFFER_SIZE; i++) {
    bufferTension[i] = 110.0 + (random(-500, 501) / 100.0);
    bufferCorriente[i] = (random(1, 1001) / 100.0);
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  }
  bufferFull = true;
}

void loop() {
  // --- (opcional) generar nuevos valores cada 1s para que el histórico cambie ---
  unsigned long now = millis();
  if (now - lastGenTime >= INTERVAL_MS) {
    lastGenTime = now;
    float tension = 110.0 + (random(-500, 501) / 100.0);
    float corriente = (random(1, 1001) / 100.0);
    bufferTension[bufferIndex] = tension;
    bufferCorriente[bufferIndex] = corriente;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferIndex == 0) bufferFull = true;
  }

  // --- detectar datos entrantes por Bluetooth ---
  if (SerialBT.available() > 0) {
    // pequeño retardo para dejar que lleguen todos los bytes del comando
    delay(50);

    // Leer hasta '\n' si llegó, o lo que haya en buffer
    String comando = SerialBT.readStringUntil('\n'); // si no llega '\n' devolverá lo que haya tras timeout
    comando.trim(); // eliminar espacios/CR/LF sobrantes

    // DEBUG: mostrar lo recibido con longitud y códigos si hace falta
    Serial.print("RX raw: '"); Serial.print(comando); Serial.print("'  len=");
    Serial.println(comando.length());
    // Opcional: mostrar códigos ASCII si quieres:
    if (comando.length() > 0) {
      Serial.print("RX codes: ");
      for (unsigned int j = 0; j < comando.length(); j++) {
        Serial.print((int)comando[j]);
        Serial.print(j + 1 < comando.length() ? "," : "\n");
      }
    }

    // Normalizar a mayúsculas para comparar
    String cmdUp = comando;
    for (unsigned int i = 0; i < cmdUp.length(); i++) cmdUp[i] = toupper(cmdUp[i]);
    Serial.print("CMDUP: '"); Serial.print(cmdUp); Serial.println("'");

    if (cmdUp == "GET100") {
      Serial.println("GET100 recibido -> enviando buffer...");
      enviarBuffer();
      Serial.println("Envio completado.");
    } else {
      Serial.println("Comando no reconocido (ignorado).");
    }
  }

  delay(5);
}

void enviarBuffer() {
  int cantidad = bufferFull ? BUFFER_SIZE : bufferIndex;
  int start = bufferFull ? bufferIndex : 0;

  Serial.print("Preparando a enviar cantidad = "); Serial.println(cantidad);

  for (int k = 0; k < cantidad; k++) {
    int i = (start + k) % BUFFER_SIZE;
    SerialBT.print(bufferTension[i], 2);
    SerialBT.print(",");
    SerialBT.print(bufferCorriente[i], 2);
    if (k < cantidad - 1) SerialBT.print(",");
  }

  SerialBT.print('\n'); // único fin de línea
  Serial.println(" -> enviada linea completa por BT.");
}

