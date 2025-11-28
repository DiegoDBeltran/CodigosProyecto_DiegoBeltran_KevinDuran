/*
 * ESP32 + ADS1220 + Sensores ACS758 y ZMPT101B
 * --------------------------------------------------
 * Lee señales AC (corriente y tensión) con el ADS1220 en modo diferencial
 * Calcula su valor RMS cada segundo y lo guarda en buffers circulares
 * Envío de los últimos 100 valores RMS por Bluetooth mediante comando GET100
 * --------------------------------------------------
 * Voltaje: sensor ZMPT101B (AIN2-AIN3)
 * Corriente: sensor ACS758LCB-050B (AIN0-AIN1)
 * Ambos sensores alimentados a 3V, ±1.65V máx
 */

#include <SPI.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ------------------------- ADS1220 CONFIG -------------------------

#define PIN_CS   5
#define PIN_DRDY 4
#define PIN_RST  2
#define PIN_CLK  25   // salida de reloj para CLKIN del ADS131M04 (se usa para ajustar fCLKIN)

#define CMD_RESET   0x06
#define CMD_START   0x08
#define CMD_RDATA   0x10
#define CMD_WREG    0x40

#define VREF 1.2      // referencia interna del ADS131M04
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

uint8_t selectedChannel = 0; // 0 => CH0 (corriente), 2 => CH2 (tensión)

void escribirRegistroADS(byte reg, uint16_t valor) {
  // Escribe un registro de 16 bits en el ADS131M04 (CLOCK u otros) usando la secuencia WREG.
  // Implementación sencilla: genera un frame WREG + datos (24-bit word framing asumido).
  digitalWrite(PIN_CS, LOW);

  // En la práctica el WREG requiere un encuadre en palabras de 24 bits.
  SPI.transfer(0x61);             // WREG (comando base)
  SPI.transfer(0x80 | (reg & 0x3F)); // segundo byte (formato: parte del comando + dirección)
  SPI.transfer(0x00);             // tercer byte (completa la palabra de comando)
  // Ahora los 16 bits del registro (MSB primero) empaquetados en dos bytes dentro del siguiente espacio
  SPI.transfer((valor >> 8) & 0xFF);
  SPI.transfer(valor & 0xFF);
  // Un padding para completar el tamaño mínimo de trama (dependiendo del WLENGTH usado)
  SPI.transfer(0x00);

  digitalWrite(PIN_CS, HIGH);
  delay(5);
}

void configurarADS1220_base() {
  /*
   * Configura el ADS131M04 
   * - Genera CLKIN en PIN_CLK a 1.536 MHz (para obtener fDATA = 12000 SPS con OSR=64)
   * - Escribe el registro CLOCK con TBM=1 (OSR=64) y PWR=High-Resolution
   * - Habilita los 4 canales
   */
  // Inicializar pin de reloj (usamos el PWM hardware LEDC del ESP32)
  pinMode(PIN_CLK, OUTPUT);
  // setup LEDC channel for high frequency clock
  const int ledcChannel = 0;
  const int ledcResolution = 1; // no se usa resolución en bits, usamos frecuencia directa con ledcWriteTone
  // Usamos la función ledcWriteTone para fijar la frecuencia (disponible en core ESP32 Arduino)
  ledcSetup(ledcChannel, 1536000, 8); // 1.536 MHz, resolución 8 (dummy, la función ajusta el timer)
  ledcAttachPin(PIN_CLK, ledcChannel);
  ledcWriteTone(ledcChannel, 1536000); // 1.536 MHz

  // Reset físico del ADC por SYNC/RESET (PIN_RST)
  digitalWrite(PIN_RST, LOW);
  delay(5);
  digitalWrite(PIN_RST, HIGH);
  delay(50); // espera para que ADC arranque

  // SPI: asegurarse de estar en modo 1 (CPOL=0, CPHA=1)
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));

  // Construir valor del registro CLOCK:
  // Bits: [15:12] reserved=0, [11:8] CH3_EN..CH0_EN = 0b1111 (habilita 4 canales),
  // [7] RESERVED=0, [6] TBM=1 (turbo mode -> OSR=64), [5:3] OSR[2:0]=xxx (TBM override),
  // [1:0] PWR[1:0]=10 (High-resolution)
  uint16_t clockReg = 0;
  clockReg |= (0x0F << 8);   // CHx_EN = 1111
  clockReg |= (1 << 6);      // TBM = 1 -> OSR = 64
  clockReg |= 0x02;          // PWR = 10 (High-resolution)

  // Escribir registro CLOCK (dirección 0x03 en la map de registros)
  escribirRegistroADS(0x03, clockReg);

  // Después de escribir CLOCK, es buena práctica esperar a que el ADC se estabilice
  delay(20);
}

void cambiarMUX(byte canalMUX) {
  // En el ADS131M04 no hay MUX por canal: todos los canales conversan simultáneamente.
  // 0x01 -> AIN0-AIN1 (corriente)  => selectedChannel = 0 (CH0)
  // 0x23 -> AIN2-AIN3 (tensión)   => selectedChannel = 2 (CH2)
  if (canalMUX == 0x01) {
    selectedChannel = 0;
  } else if (canalMUX == 0x23) {
    selectedChannel = 2;
  } else {
    // por defecto dejar CH0
    selectedChannel = 0;
  }
  // El ADS131M04 necesita que las lecturas posteriores lean tramas completas;
  delayMicroseconds(100);
}

long leerADS1220() {
  // Frame (modo por defecto, WLENGTH=24-bits): STATUS (24b) + CH0(24) + CH1(24) + CH2(24) + CH3(24) + CRC(24)
  // Devolvemos el valor del canal seleccionado (selectedChannel) en formato signed 24-bit extendido a long.

  // Espera DRDY a bajar (transición a LOW indica dato listo)
  while (digitalRead(PIN_DRDY) == HIGH);

  digitalWrite(PIN_CS, LOW);
  // Leer 6 palabras de 24 bits => 6 * 3 = 18 bytes
  const int words = 6;
  uint8_t buf[words * 3];
  for (int i = 0; i < words * 3; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_CS, HIGH);

  // Parsear: palabra 0 = STATUS, palabra 1 = CH0, 2 = CH1, 3 = CH2, 4 = CH3, 5 = CRC
  // Seleccionar la palabra correspondiente al channel: index = 1 + selectedChannel
  int wordIndex = 1 + selectedChannel; // selectedChannel 0..3
  int byteIndex = wordIndex * 3;

  long raw = 0;
  raw = ((long)buf[byteIndex] << 16) | ((long)buf[byteIndex + 1] << 8) | (long)buf[byteIndex + 2];
  // extendido de signo (24 bits)
  if (raw & 0x800000) raw |= 0xFF000000;
  return raw;
}

float convertirAVoltaje(long lectura) {
  // La conversión básica: lectura (signed 24-bit) -> V = lectura * (VREF / (2^23 - 1)) * factor_ganancia
  // Usamos ADS_GAIN para mantener compatibilidad (PGA del ADS131M04 puede ajustarse en registros, aquí asumimos 1)
  return (lectura * (VREF / ADS_GAIN)) / 8388607.0;
}

// ------------------------- SETUP -------------------------

void setup() {
  Serial.begin(115200);
  SerialBT.begin("KEDI_ESP32");

  // Configuración SPI y pines
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1)); // velocidad segura para ADS131M04 (modo 1)
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_DRDY, INPUT);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_RST, HIGH);
  delay(100);

  Serial.println("Inicializando ADS1220...");
  configurarADS1220_base(); // Configura ADC base (ahora ADS131M04 a 12kSPS)
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

    const int N = 12000; // muestras por canal en 1 segundo (12000 SPS)
    double sumaTension = 0;
    double sumaCorriente = 0;

    // Leer y descartar una trama para sincronizar
    long d0, d1, d2, d3;
    leerFrame(d0, d1, d2, d3);

    // Leer N tramas (cada trama contiene CH0..CH3). Acumular CH0 (corriente) y CH2 (tensión)
    for (int i = 0; i < N; i++) {
      leerFrame(d0, d1, d2, d3);
      float vCorr = convertirAVoltaje(d0); // CH0 -> corriente (equivalente a AIN0-AIN1)
      float vTens = convertirAVoltaje(d2); // CH2 -> tensión (equivalente a AIN2-AIN3)
      sumaCorriente += (double)vCorr * (double)vCorr;
      sumaTension += (double)vTens * (double)vTens;
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
