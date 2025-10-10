/*
 * ESP32 + ADS1220 + Sensores ACS758 y ZMPT101B
 * --------------------------------------------------
 * Lee señales AC (corriente y tensión) con el ADS1220 en modo diferencial
 * Calcula su valor RMS cada segundo y lo guarda en buffers circulares
 * Envío de los últimos 100 valores RMS por Bluetooth mediante comando GET100
 * --------------------------------------------------
 * Voltaje: sensor ZMPT101B (AIN2-AIN3)
 * Corriente: sensor ACS758LCB-050B (AIN0-AIN1)
 * Ambos sensores alimentados a 5V, centrados en ~2.5V, ±1.65V máx
 */

#include <SPI.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ------------------------- ADS1220 CONFIG -------------------------

#define PIN_CS   5
#define PIN_DRDY 4
#define PIN_RST  2

#define CMD_RESET   0x06
#define CMD_START   0x08
#define CMD_RDATA   0x10
#define CMD_WREG    0x40

#define VREF 3.3
#define ADS_GAIN 1.0

// ------------------------- BUFFERS EXISTENTES -------------------------

const int BUFFER_SIZE = 100;
float bufferTension[BUFFER_SIZE];
float bufferCorriente[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

unsigned long lastGenTime = 0;
const unsigned long INTERVAL_MS = 1000; // Cada segundo calcular RMS

// ------------------------- FUNCIONES ADS1220 -------------------------

void escribirRegistroADS(byte reg, byte valor) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_WREG | reg);
  SPI.transfer(0x00); // escribir solo un registro
  SPI.transfer(valor);
  digitalWrite(PIN_CS, HIGH);
}

void configurarADS1220_base() {
  /*
   * Configura ADS1220 en modo continuo, 1000SPS, PGA=1, referencia interna
   * Lectura diferencial. Luego solo cambiaremos el MUX.
   */
  digitalWrite(PIN_CS, LOW);
  byte config[4] = {
    0x01, // MUX = AIN0-AIN1 (corriente al inicio), PGA=1
    0x04, // Modo normal, 1000 SPS
    0x10, // Vref interna, modo continuo
    0x00  // DRDY activo bajo
  };
  SPI.transfer(CMD_WREG | 0x03); // Escribir 4 registros (0-3)
  for (int i = 0; i < 4; i++) SPI.transfer(config[i]);
  digitalWrite(PIN_CS, HIGH);
  delay(50); // espera para que ADC inicie
}

void cambiarMUX(byte canalMUX) {
  // Cambia solo el registro 0 (MUX)
  escribirRegistroADS(0x00, canalMUX);
  delayMicroseconds(100);
}

long leerADS1220() {
  // Espera a que DRDY esté bajo
  while (digitalRead(PIN_DRDY) == HIGH);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_RDATA);
  byte b1 = SPI.transfer(0x00);
  byte b2 = SPI.transfer(0x00);
  byte b3 = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);

  long val = ((long)b1 << 16) | ((long)b2 << 8) | b3;
  if (val & 0x800000) val |= 0xFF000000; // signo extendido (24 bits)
  return val;
}

float convertirAVoltaje(long lectura) {
  return (lectura * (VREF / ADS_GAIN)) / 8388607.0;
}

// ------------------------- SETUP -------------------------

void setup() {
  Serial.begin(115200);
  SerialBT.begin("KEDI_ESP32");

  // Configuración SPI y pines
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1)); // velocidad segura para ADS1220
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_DRDY, INPUT);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_RST, HIGH);
  delay(100);

  Serial.println("Inicializando ADS1220...");
  configurarADS1220_base(); // Configura ADC base (modo continuo, 1000 SPS)
  Serial.println("ADS1220 configurado correctamente.");
  Serial.println("ESP32 listo (Bluetooth SPP).");

  // Llenar buffer inicial (datos vacíos)
  for (int i = 0; i < BUFFER_SIZE; i++) {
    bufferTension[i] = 0.0;
    bufferCorriente[i] = 0.0;
  }
  bufferFull = false;
}

// ------------------------- LOOP PRINCIPAL -------------------------

void loop() {

  unsigned long now = millis();

  // Cada segundo calcular RMS de tensión y corriente
  if (now - lastGenTime >= INTERVAL_MS) {
    lastGenTime = now;

    const int N = 500; // muestras por canal
    double sumaTension = 0;
    double sumaCorriente = 0;

    // --- Corriente (AIN0-AIN1) ---
    cambiarMUX(0x01);        // AIN0-AIN1
    leerADS1220();           // descartar primera lectura
    for (int i = 0; i < N; i++) {
      long adcCorr = leerADS1220();
      float vCorr = convertirAVoltaje(adcCorr);
      sumaCorriente += vCorr * vCorr;
    }

    // --- Tensión (AIN2-AIN3) ---
    cambiarMUX(0x23);        // AIN2-AIN3
    leerADS1220();           // descartar primera lectura
    for (int i = 0; i < N; i++) {
      long adcTens = leerADS1220();
      float vTens = convertirAVoltaje(adcTens);
      sumaTension += vTens * vTens;
    }

    float corrienteRMS = sqrt(sumaCorriente / N);
    float tensionRMS = sqrt(sumaTension / N);

    // Guardar en buffer
    bufferTension[bufferIndex] = tensionRMS;
    bufferCorriente[bufferIndex] = corrienteRMS;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferIndex == 0) bufferFull = true;

    Serial.print("Tensión RMS: "); Serial.print(tensionRMS, 4);
    Serial.print(" V | Corriente RMS: "); Serial.print(corrienteRMS, 4);
    Serial.println(" A");
  }

  // ---------------- Bluetooth  ----------------
  if (SerialBT.available() > 0) {
    delay(50);
    String comando = SerialBT.readStringUntil('\n');
    comando.trim();

    Serial.print("RX raw: '"); Serial.print(comando); Serial.print("'  len=");
    Serial.println(comando.length());

    if (comando.length() > 0) {
      Serial.print("RX codes: ");
      for (unsigned int j = 0; j < comando.length(); j++) {
        Serial.print((int)comando[j]);
        Serial.print(j + 1 < comando.length() ? "," : "\n");
      }
    }

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

// ------------------------- ENVÍO POR BLUETOOTH -------------------------

void enviarBuffer() {
  int cantidad = bufferFull ? BUFFER_SIZE : bufferIndex;
  int start = bufferFull ? bufferIndex : 0;

  Serial.print("Preparando a enviar cantidad = "); Serial.println(cantidad);

  for (int k = 0; k < cantidad; k++) {
    int i = (start + k) % BUFFER_SIZE;
    SerialBT.print(bufferTension[i], 3);
    SerialBT.print(",");
    SerialBT.print(bufferCorriente[i], 3);
    if (k < cantidad - 1) SerialBT.print(",");
  }

  SerialBT.print('\n');
  Serial.println(" -> enviada linea completa por BT.");
}
