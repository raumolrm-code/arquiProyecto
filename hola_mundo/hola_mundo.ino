
/*
  ALIMENTADOR DE MASCOTAS CON SEGURIDAD - Arduino UNO R3
  Arquitectura del Computador (2S 2026)

  El alimento se dispensa con un servo. El agua se simula encendiendo un LED.
  El teclado fisico 4x4 usa solamente sus primeras 3 filas y 3 columnas.

  Mapeo:
  D0 RX  <- TX Bluetooth       D1 TX -> RX Bluetooth (adaptar 5 V a 3.3 V)
  D2 = servo SG90 (comida)     D3 = LED con 220-330 ohm (agua)
  D4 = DS1302 CE               D5 = DS1302 IO
  D6 = DS1302 CLK              D7 = DHT DATA
  D8,A0,A1 = filas teclado     A2 = columnas teclado por divisor resistivo
  A3 = pulsadores analogicos   A4 = I2C SDA LCD
  A5 = I2C SCL LCD
  D9 = RC522 RST               D10 = RC522 SDA/SS
  D11 = RC522 MOSI             D12 = RC522 MISO
  D13 = RC522 SCK

  Teclado:
  - C1/C2/C3 -> A2 mediante 3.3 kohm, 10 kohm y 22 kohm.
  - A2 -> GND mediante pull-down de 10 kohm.
  Pulsadores:
  - A3 -> GND mediante pull-down de 10 kohm.
  - COMIDA: 5 V -> pulsador -> 20 kohm -> A3 (ADC aproximado 341).
  - AGUA:   5 V -> pulsador -> 5.1 kohm -> A3 (ADC aproximado 677).

  Librerias (administrador de Arduino IDE):
  - MFRC522 (miguelbalboa)
  - Rtc by Makuna
  - LiquidCrystal I2C (version con lcd.init())
  - DHT sensor library (Adafruit) + Adafruit Unified Sensor
  - Servo
  Integradas: SPI, Wire y EEPROM.

  COMANDOS (una linea ASCII terminada en '\n', 9600 baudios):
  HELP
  STATUS
  MODE,1234,AUTO          o MODE,1234,MANUAL
  TIME,1234,2026,11,06,14,30,00
  S1,1234,08,00           primera comida automatica
  S2,1234,18,00           segunda comida automatica
  W1,1234,10,00           primera agua automatica
  W2,1234,20,00           segunda agua automatica
  PORTION,1234,2000       duracion de servo/LED en ms (NO equivale a gramos)
  ENROLL,1234             acercar tarjeta en los siguientes 15 s
  RESETCARD,1234          borrar tarjeta autorizada
  FEED,1234               comida desde app, solamente modo MANUAL
  WATER,1234              agua desde app, solamente modo MANUAL
  PIN,1234,2345           modificar PIN de 4 teclas (1-9)

  IMPORTANTE: Seguridad para prototipo academico, no para uso sensible.
  UID RFID y PIN transmitido por Bluetooth clasico no son autenticacion
  criptografica robusta. Probar el servo SIN comida y supervisado.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <MFRC522.h>
#include <RtcDS1302.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <string.h>
#include <stddef.h>

// ---------- Pines ----------
const uint8_t PIN_SERVO = 2;
const uint8_t PIN_LED_AGUA = 3;
const uint8_t PIN_RTC_CE = 4;
const uint8_t PIN_RTC_IO = 5;
const uint8_t PIN_RTC_CLK = 6;
const uint8_t PIN_DHT = 7;
const uint8_t PIN_RFID_RST = 9;
const uint8_t PIN_RFID_SS = 10;
const uint8_t PIN_TECLADO_FILAS[3] = {8, A0, A1};
const uint8_t PIN_TECLADO_COLUMNAS = A2;
const uint8_t PIN_BOTONES = A3;

#define TIPO_DHT DHT11

const uint8_t LCD_DIRECCION = 0x27;
const uint8_t ANGULO_CERRADO = 0;
const uint8_t ANGULO_ABIERTO = 90;
const unsigned long TIEMPO_CIERRE_MS = 650UL;
const unsigned long TIEMPO_PIN_MS = 20000UL;
const unsigned long TIEMPO_SELECCION_MS = 20000UL;
const unsigned long TIEMPO_ENROLAR_MS = 15000UL;
const unsigned long TIEMPO_BLOQUEO_MS = 30000UL;
const unsigned long TIEMPO_DEBOUNCE_MS = 30UL;
const uint8_t MAX_INTENTOS = 3;

// Rangos iniciales. Deben calibrarse con el circuito real.
const uint16_t ADC_TECLA_C1_MIN = 690;
const uint16_t ADC_TECLA_C1_MAX = 850;
const uint16_t ADC_TECLA_C2_MIN = 430;
const uint16_t ADC_TECLA_C2_MAX = 600;
const uint16_t ADC_TECLA_C3_MIN = 240;
const uint16_t ADC_TECLA_C3_MAX = 400;
const uint16_t ADC_REPOSO_MAX = 100;
const uint16_t ADC_COMIDA_MIN = 290;
const uint16_t ADC_COMIDA_MAX = 400;
const uint16_t ADC_AGUA_MIN = 630;
const uint16_t ADC_AGUA_MAX = 710;

MFRC522 lectorRFID(PIN_RFID_SS, PIN_RFID_RST);
ThreeWire rtcWire(PIN_RTC_IO, PIN_RTC_CLK, PIN_RTC_CE);
RtcDS1302<ThreeWire> rtc(rtcWire);
LiquidCrystal_I2C lcd(LCD_DIRECCION, 16, 2);
DHT dht(PIN_DHT, TIPO_DHT);
Servo servo;

// ---------- Configuracion persistida en EEPROM ----------
enum Modo : uint8_t {
  MANUAL = 0,
  AUTOMATICO = 1
};

const uint32_t FIRMA_EEPROM_V1 = 0xA7152026UL;
const uint32_t FIRMA_EEPROM = 0xA7152027UL;

// Estructura exacta de la version anterior; se conserva para migrar EEPROM.
struct ConfiguracionV1 {
  uint32_t firma;
  char pin[5];
  uint8_t modo;
  uint8_t hora[2];
  uint8_t minuto[2];
  uint16_t aperturaMs;
  uint8_t uidTam;
  uint8_t uid[10];
  uint32_t ultimaFecha[2];
  uint16_t checksum;
};

struct Configuracion {
  uint32_t firma;
  char pin[5];
  uint8_t modo;
  uint8_t horaComida[2];
  uint8_t minutoComida[2];
  uint8_t horaAgua[2];
  uint8_t minutoAgua[2];
  uint16_t aperturaMs;
  uint8_t uidTam;
  uint8_t uid[10];
  uint32_t ultimaFechaComida[2];
  uint32_t ultimaFechaAgua[2];
  uint16_t checksum;
};
Configuracion config;

// Se conservan los valores numericos 0-4 publicados por STATUS,STATE.
enum Estado : uint8_t {
  LISTO = 0,
  ESPERANDO_PIN = 1,
  ENROLANDO_RFID = 2,
  DISPENSANDO = 3,
  CERRANDO = 4,
  ESPERANDO_SELECCION = 5,
  SIRVIENDO_AGUA = 6
};

enum TipoEntrega : uint8_t {
  NINGUNA = 0,
  COMIDA = 1,
  AGUA = 2
};

Estado estado = LISTO;
TipoEntrega accionPendiente = NINGUNA;
TipoEntrega accionActiva = NINGUNA;

// ---------- Cola de eventos automaticos ----------
struct EventoProgramado {
  uint8_t id;       // 0=S1, 1=S2, 2=W1, 3=W2
  TipoEntrega tipo;
  uint32_t fecha;
};

const uint8_t CAPACIDAD_COLA = 4;
EventoProgramado cola[CAPACIDAD_COLA];
uint8_t inicioCola = 0;
uint8_t cantidadCola = 0;
bool eventoEnCola[4] = {false, false, false, false};

// ---------- Variables de estado ----------
bool rtcValido = false;
float temperatura = NAN;
float humedad = NAN;

char pinTecleado[5] = {0};
uint8_t cantidadTecleada = 0;
uint8_t intentosFallidos = 0;
const char* origenPendiente = "";
const char* origenDispensacion = "";
unsigned long inicioEstado = 0;
unsigned long bloqueoHasta = 0;
unsigned long ultimoLCD = 0;
unsigned long ultimoDHT = 0;
unsigned long ultimoRTC = 0;
unsigned long ultimoRFID = 0;

// ---------- Variables de comunicacion Bluetooth ----------
char comandoBT[72];
uint8_t posBT = 0;
char aviso1[17] = "";
char aviso2[17] = "";
unsigned long avisoHasta = 0;

// ---------- Utilidades de configuracion ----------
uint16_t checksumDatos(const uint8_t* datos, size_t longitud) {
  uint16_t suma = 0xA5A5;
  for (size_t i = 0; i < longitud; ++i) {
    suma = static_cast<uint16_t>((suma ^ datos[i]) * 31U);
  }
  return suma;
}

uint16_t checksumConfig() {
  return checksumDatos(reinterpret_cast<const uint8_t*>(&config),
                       offsetof(Configuracion, checksum));
}

uint16_t checksumConfigV1(const ConfiguracionV1& anterior) {
  return checksumDatos(reinterpret_cast<const uint8_t*>(&anterior),
                       offsetof(ConfiguracionV1, checksum));
}

bool pinPermitido(const char* p) {
  if (!p || strlen(p) != 4) return false;
  for (uint8_t i = 0; i < 4; ++i) {
    if (p[i] < '1' || p[i] > '9') return false;
  }
  return true;
}

bool horariosValidos(const uint8_t hora[2], const uint8_t minuto[2]) {
  return hora[0] <= 23 && hora[1] <= 23 &&
         minuto[0] <= 59 && minuto[1] <= 59 &&
         !(hora[0] == hora[1] && minuto[0] == minuto[1]);
}

bool configValida() {
  return config.firma == FIRMA_EEPROM &&
         config.checksum == checksumConfig() &&
         config.pin[4] == '\0' && pinPermitido(config.pin) &&
         config.modo <= AUTOMATICO &&
         horariosValidos(config.horaComida, config.minutoComida) &&
         horariosValidos(config.horaAgua, config.minutoAgua) &&
         config.aperturaMs >= 300 && config.aperturaMs <= 10000 &&
         config.uidTam <= sizeof(config.uid);
}

bool configV1Valida(const ConfiguracionV1& anterior) {
  return anterior.firma == FIRMA_EEPROM_V1 &&
         anterior.checksum == checksumConfigV1(anterior) &&
         anterior.pin[4] == '\0' && pinPermitido(anterior.pin) &&
         anterior.modo <= AUTOMATICO &&
         anterior.hora[0] <= 23 && anterior.hora[1] <= 23 &&
         anterior.minuto[0] <= 59 && anterior.minuto[1] <= 59 &&
         anterior.aperturaMs >= 300 && anterior.aperturaMs <= 10000 &&
         anterior.uidTam <= sizeof(anterior.uid);
}

void guardarConfig() {
  config.checksum = checksumConfig();
  EEPROM.put(0, config);
}

void valoresIniciales() {
  memset(&config, 0, sizeof(config));
  config.firma = FIRMA_EEPROM;
  strcpy(config.pin, "1234");
  config.modo = MANUAL;
  config.horaComida[0] = 8;
  config.horaComida[1] = 18;
  config.horaAgua[0] = 10;
  config.horaAgua[1] = 20;
  config.aperturaMs = 2000;
  guardarConfig();
}

void migrarConfigV1(const ConfiguracionV1& anterior) {
  memset(&config, 0, sizeof(config));
  config.firma = FIRMA_EEPROM;
  memcpy(config.pin, anterior.pin, sizeof(config.pin));
  config.modo = anterior.modo;
  memcpy(config.horaComida, anterior.hora, sizeof(config.horaComida));
  memcpy(config.minutoComida, anterior.minuto, sizeof(config.minutoComida));
  config.horaAgua[0] = 10;
  config.horaAgua[1] = 20;
  config.aperturaMs = anterior.aperturaMs;
  config.uidTam = anterior.uidTam;
  memcpy(config.uid, anterior.uid, sizeof(config.uid));
  memcpy(config.ultimaFechaComida, anterior.ultimaFecha,
         sizeof(config.ultimaFechaComida));
  guardarConfig();
}

void cargarConfig() {
  EEPROM.get(0, config);
  if (configValida()) return;

  ConfiguracionV1 anterior;
  EEPROM.get(0, anterior);
  if (configV1Valida(anterior)) {
    migrarConfigV1(anterior);
    return;
  }
  valoresIniciales();
}

// ---------- Interfaz y utilidad ----------
void mostrarAviso(const char* linea1, const char* linea2, unsigned long ms) {
  strncpy(aviso1, linea1, 16);
  strncpy(aviso2, linea2, 16);
  aviso1[16] = '\0';
  aviso2[16] = '\0';
  avisoHasta = millis() + ms;
}

bool avisoActivo() {
  return static_cast<int32_t>(avisoHasta - millis()) > 0;
}

bool bloqueado() {
  return static_cast<int32_t>(bloqueoHasta - millis()) > 0;
}

void lineaLCD(uint8_t fila, const char* texto) {
  lcd.setCursor(0, fila);
  lcd.print(F("                "));
  lcd.setCursor(0, fila);
  lcd.print(texto);
}

uint32_t claveFecha(const RtcDateTime& t) {
  return static_cast<uint32_t>(t.Year()) * 10000UL +
         static_cast<uint32_t>(t.Month()) * 100UL + t.Day();
}

const __FlashStringHelper* nombreAccion(TipoEntrega tipo) {
  if (tipo == COMIDA) return F("FOOD");
  if (tipo == AGUA) return F("WATER");
  return F("NONE");
}

void actualizarLCD() {
  unsigned long ahora = millis();
  if (ahora - ultimoLCD < 500UL) return;
  ultimoLCD = ahora;

  if (estado == DISPENSANDO || estado == CERRANDO) {
    lineaLCD(0, estado == DISPENSANDO ? "Sirviendo comida" : "Cerrando puerta");
    lineaLCD(1, "Concentrado");
    return;
  }
  if (estado == SIRVIENDO_AGUA) {
    lineaLCD(0, "Sirviendo agua");
    lineaLCD(1, "LED encendido");
    return;
  }
  if (estado == ESPERANDO_PIN) {
    lineaLCD(0, "Ingrese PIN:");
    char progreso[17] = "____";
    for (uint8_t i = 0; i < cantidadTecleada; ++i) progreso[i] = '*';
    lineaLCD(1, progreso);
    return;
  }
  if (estado == ESPERANDO_SELECCION) {
    if (avisoActivo()) {
      lineaLCD(0, aviso1);
      lineaLCD(1, aviso2);
      return;
    }
    lineaLCD(0, "Elija servicio:");
    lineaLCD(1, "1:Comida 2:Agua");
    return;
  }
  if (estado == ENROLANDO_RFID) {
    lineaLCD(0, "Registrar RFID");
    lineaLCD(1, "Acerque al RC522");
    return;
  }
  if (bloqueado()) {
    lineaLCD(0, "PIN bloqueado");
    lineaLCD(1, "Espere 30 seg.");
    return;
  }
  if (avisoActivo()) {
    lineaLCD(0, aviso1);
    lineaLCD(1, aviso2);
    return;
  }
  if (!rtcValido) {
    lineaLCD(0, "RTC sin ajustar");
    lineaLCD(1, "Usar TIME por BT");
    return;
  }

  RtcDateTime t = rtc.GetDateTime();
  if (!t.IsValid()) {
    rtcValido = false;
    return;
  }
  char encabezado[17];
  snprintf(encabezado, sizeof(encabezado), "%02u:%02u %02u/%02u/%04u",
           t.Hour(), t.Minute(), t.Day(), t.Month(), t.Year());
  lineaLCD(0, encabezado);

  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(F("T:"));
  if (isnan(temperatura)) lcd.print(F("--"));
  else lcd.print(temperatura, 1);
  lcd.print(F("C H:"));
  if (isnan(humedad)) lcd.print(F("--"));
  else lcd.print(humedad, 0);
  lcd.print('%');
}

void leerSensor() {
  if (millis() - ultimoDHT < 2500UL) return;
  ultimoDHT = millis();
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(t) && !isnan(h)) {
    temperatura = t;
    humedad = h;
  }
}

// ---------- Lecturas analogicas ----------
uint16_t leerADCestable(uint8_t pin) {
  analogRead(pin);
  delayMicroseconds(80);
  return static_cast<uint16_t>(analogRead(pin));
}

int8_t columnaDesdeADC(uint16_t valor) {
  if (valor >= ADC_TECLA_C1_MIN && valor <= ADC_TECLA_C1_MAX) return 0;
  if (valor >= ADC_TECLA_C2_MIN && valor <= ADC_TECLA_C2_MAX) return 1;
  if (valor >= ADC_TECLA_C3_MIN && valor <= ADC_TECLA_C3_MAX) return 2;
  return -1;
}

char explorarTeclado() {
  static const char teclas[3][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'}
  };

  for (uint8_t fila = 0; fila < 3; ++fila) {
    for (uint8_t i = 0; i < 3; ++i) {
      digitalWrite(PIN_TECLADO_FILAS[i], LOW); // Evita pull-up al volver a INPUT.
      pinMode(PIN_TECLADO_FILAS[i], INPUT);
    }
    pinMode(PIN_TECLADO_FILAS[fila], OUTPUT);
    digitalWrite(PIN_TECLADO_FILAS[fila], HIGH);
    delayMicroseconds(80);
    int8_t columna = columnaDesdeADC(leerADCestable(PIN_TECLADO_COLUMNAS));
    digitalWrite(PIN_TECLADO_FILAS[fila], LOW);
    pinMode(PIN_TECLADO_FILAS[fila], INPUT);
    if (columna >= 0) return teclas[fila][columna];
  }
  return '\0';
}

char leerTecla() {
  static char candidato = '\0';
  static unsigned long cambio = 0;
  static bool bloqueadaHastaLiberar = false;
  char actual = explorarTeclado();
  unsigned long ahora = millis();

  if (actual != candidato) {
    candidato = actual;
    cambio = ahora;
    return '\0';
  }
  if (ahora - cambio < TIEMPO_DEBOUNCE_MS) return '\0';
  if (actual == '\0') {
    bloqueadaHastaLiberar = false;
    return '\0';
  }
  if (bloqueadaHastaLiberar) return '\0';
  bloqueadaHastaLiberar = true;
  return actual;
}

int8_t clasificarBoton(uint16_t valor) {
  if (valor < ADC_REPOSO_MAX) return NINGUNA;
  if (valor >= ADC_COMIDA_MIN && valor <= ADC_COMIDA_MAX) return COMIDA;
  if (valor >= ADC_AGUA_MIN && valor <= ADC_AGUA_MAX) return AGUA;
  return -1;
}

TipoEntrega leerBoton() {
  static int8_t candidato = NINGUNA;
  static unsigned long cambio = 0;
  static bool armado = true;
  int8_t actual = clasificarBoton(leerADCestable(PIN_BOTONES));
  unsigned long ahora = millis();

  if (actual != candidato) {
    candidato = actual;
    cambio = ahora;
    return NINGUNA;
  }
  if (ahora - cambio < TIEMPO_DEBOUNCE_MS) return NINGUNA;
  if (actual == NINGUNA) {
    armado = true;
    return NINGUNA;
  }
  if (actual < 0) {
    if (armado) {
      armado = false;
      Serial.println(F("ERROR,INVALID_BUTTON_LEVEL"));
    }
    return NINGUNA;
  }
  if (!armado) return NINGUNA;
  armado = false;
  return static_cast<TipoEntrega>(actual);
}

// ---------- Entregas ----------
void iniciarEntrega(TipoEntrega tipo, const char* origen) {
  if (estado != LISTO || tipo == NINGUNA) {
    Serial.println(F("ERROR,BUSY"));
    return;
  }
  origenDispensacion = origen;
  accionActiva = tipo;
  inicioEstado = millis();

  if (tipo == COMIDA) {
    servo.attach(PIN_SERVO);
    servo.write(ANGULO_ABIERTO);
    estado = DISPENSANDO;
    Serial.print(F("EVENT,FEED_START,"));
  } else {
    digitalWrite(PIN_LED_AGUA, HIGH);
    estado = SIRVIENDO_AGUA;
    Serial.print(F("EVENT,WATER_START,"));
  }
  Serial.println(origenDispensacion);
}

void actualizarDispensador() {
  const unsigned long ahora = millis();
  if (estado == DISPENSANDO && ahora - inicioEstado >= config.aperturaMs) {
    servo.write(ANGULO_CERRADO);
    estado = CERRANDO;
    inicioEstado = ahora;
  } else if (estado == CERRANDO && ahora - inicioEstado >= TIEMPO_CIERRE_MS) {
    servo.detach();
    estado = LISTO;
    accionActiva = NINGUNA;
    Serial.print(F("EVENT,FEED_END,"));
    Serial.println(origenDispensacion);
    mostrarAviso("Alimento servido", "Operacion lista", 1800);
  } else if (estado == SIRVIENDO_AGUA &&
             ahora - inicioEstado >= config.aperturaMs) {
    digitalWrite(PIN_LED_AGUA, LOW);
    estado = LISTO;
    accionActiva = NINGUNA;
    Serial.print(F("EVENT,WATER_END,"));
    Serial.println(origenDispensacion);
    mostrarAviso("Agua servida", "Operacion lista", 1800);
  }
}

// ---------- Autenticacion manual ----------
void limpiarSolicitud() {
  accionPendiente = NINGUNA;
  origenPendiente = "";
  memset(pinTecleado, 0, sizeof(pinTecleado));
  cantidadTecleada = 0;
}

void solicitarPin(const char* origen, TipoEntrega tipo) {
  if (estado != LISTO || config.modo != MANUAL) return;
  if (bloqueado()) {
    Serial.println(F("ERROR,PIN_LOCKED"));
    return;
  }
  memset(pinTecleado, 0, sizeof(pinTecleado));
  cantidadTecleada = 0;
  origenPendiente = origen;
  accionPendiente = tipo;
  estado = ESPERANDO_PIN;
  inicioEstado = millis();
  Serial.print(F("EVENT,PIN_REQUIRED,"));
  Serial.println(origenPendiente);
}

void errorPin() {
  ++intentosFallidos;
  estado = LISTO;
  limpiarSolicitud();
  Serial.println(F("ERROR,INVALID_PIN"));
  mostrarAviso("PIN incorrecto", "Acceso denegado", 1800);
  if (intentosFallidos >= MAX_INTENTOS) {
    intentosFallidos = 0;
    bloqueoHasta = millis() + TIEMPO_BLOQUEO_MS;
    Serial.println(F("EVENT,PIN_LOCKED"));
  }
}

void revisarTeclado() {
  if (estado != ESPERANDO_PIN && estado != ESPERANDO_SELECCION) return;

  unsigned long limite = estado == ESPERANDO_PIN ? TIEMPO_PIN_MS : TIEMPO_SELECCION_MS;
  if (millis() - inicioEstado >= limite) {
    bool eraPin = estado == ESPERANDO_PIN;
    estado = LISTO;
    limpiarSolicitud();
    Serial.println(eraPin ? F("EVENT,PIN_TIMEOUT") : F("EVENT,SELECTION_TIMEOUT"));
    mostrarAviso("Tiempo agotado", "Reintente", 1800);
    return;
  }

  char tecla = leerTecla();
  if (tecla < '1' || tecla > '9') return;

  if (estado == ESPERANDO_SELECCION) {
    if (tecla != '1' && tecla != '2') {
      Serial.println(F("ERROR,INVALID_SELECTION"));
      mostrarAviso("Opcion invalida", "Use 1 o 2", 900);
      return;
    }
    TipoEntrega elegida = tecla == '1' ? COMIDA : AGUA;
    const char* origen = origenPendiente;
    estado = LISTO;
    limpiarSolicitud();
    iniciarEntrega(elegida, origen);
    return;
  }

  if (cantidadTecleada >= 4) return;
  pinTecleado[cantidadTecleada++] = tecla;
  pinTecleado[cantidadTecleada] = '\0';
  if (cantidadTecleada != 4) return;

  if (strcmp(pinTecleado, config.pin) != 0) {
    errorPin();
    return;
  }

  intentosFallidos = 0;
  if (accionPendiente == NINGUNA) {
    estado = ESPERANDO_SELECCION;
    memset(pinTecleado, 0, sizeof(pinTecleado));
    cantidadTecleada = 0;
    inicioEstado = millis();
    Serial.print(F("EVENT,SELECTION_REQUIRED,"));
    Serial.println(origenPendiente);
  } else {
    TipoEntrega tipo = accionPendiente;
    const char* origen = origenPendiente;
    estado = LISTO;
    limpiarSolicitud();
    iniciarEntrega(tipo, origen);
  }
}

void revisarRFID() {
  if (estado != ENROLANDO_RFID &&
      !(estado == LISTO && config.modo == MANUAL)) return;
  if (millis() - ultimoRFID < 100UL) return;
  ultimoRFID = millis();
  if (!lectorRFID.PICC_IsNewCardPresent()) return;
  if (!lectorRFID.PICC_ReadCardSerial()) return;

  const uint8_t tam = lectorRFID.uid.size;
  if (estado == ENROLANDO_RFID) {
    if (tam > 0 && tam <= sizeof(config.uid)) {
      config.uidTam = tam;
      memset(config.uid, 0, sizeof(config.uid));
      memcpy(config.uid, lectorRFID.uid.uidByte, tam);
      guardarConfig();
      estado = LISTO;
      Serial.println(F("EVENT,CARD_ENROLLED"));
      mostrarAviso("RFID registrado", "Correctamente", 2000);
    } else {
      Serial.println(F("ERROR,UID_SIZE"));
    }
  } else {
    bool autorizada = config.uidTam == tam && tam > 0 &&
                       memcmp(config.uid, lectorRFID.uid.uidByte, tam) == 0;
    if (autorizada) solicitarPin("RFID", NINGUNA);
    else {
      Serial.println(F("EVENT,CARD_DENIED"));
      mostrarAviso("RFID no valido", "Acceso denegado", 1200);
    }
  }
  lectorRFID.PICC_HaltA();
  lectorRFID.PCD_StopCrypto1();
}

void revisarBotones() {
  TipoEntrega tipo = leerBoton();
  if (tipo == NINGUNA || estado != LISTO || config.modo != MANUAL) return;
  solicitarPin(tipo == COMIDA ? "BOTON_COMIDA" : "BOTON_AGUA", tipo);
}

// ---------- Cola y horarios automaticos ----------
void vaciarCola() {
  inicioCola = 0;
  cantidadCola = 0;
  memset(eventoEnCola, 0, sizeof(eventoEnCola));
}

bool encolarEvento(uint8_t id, TipoEntrega tipo, uint32_t fecha) {
  if (id >= 4 || eventoEnCola[id] || cantidadCola >= CAPACIDAD_COLA) return false;
  uint8_t posicion = (inicioCola + cantidadCola) % CAPACIDAD_COLA;
  cola[posicion].id = id;
  cola[posicion].tipo = tipo;
  cola[posicion].fecha = fecha;
  eventoEnCola[id] = true;
  ++cantidadCola;
  Serial.print(F("EVENT,SCHEDULE_QUEUED,"));
  Serial.println(id < 2 ? (id == 0 ? F("S1") : F("S2"))
                        : (id == 2 ? F("W1") : F("W2")));
  return true;
}

uint32_t ultimaFechaEvento(uint8_t id) {
  return id < 2 ? config.ultimaFechaComida[id]
                : config.ultimaFechaAgua[id - 2];
}

void registrarFechaEvento(uint8_t id, uint32_t fecha) {
  if (id < 2) config.ultimaFechaComida[id] = fecha;
  else config.ultimaFechaAgua[id - 2] = fecha;
  guardarConfig();
}

const char* origenEvento(uint8_t id) {
  static const char* const origenes[4] = {"AUTO_S1", "AUTO_S2", "AUTO_W1", "AUTO_W2"};
  return id < 4 ? origenes[id] : "AUTO";
}

void revisarHorarios() {
  if (config.modo != AUTOMATICO || !rtcValido) return;
  if (millis() - ultimoRTC < 500UL) return;
  ultimoRTC = millis();

  RtcDateTime t = rtc.GetDateTime();
  if (!t.IsValid()) {
    rtcValido = false;
    Serial.println(F("ERROR,RTC_INVALID"));
    return;
  }

  uint32_t hoy = claveFecha(t);
  for (uint8_t id = 0; id < 4; ++id) {
    uint8_t indice = id % 2;
    bool esComida = id < 2;
    uint8_t hora = esComida ? config.horaComida[indice] : config.horaAgua[indice];
    uint8_t minuto = esComida ? config.minutoComida[indice] : config.minutoAgua[indice];
    if (t.Hour() == hora && t.Minute() == minuto &&
        ultimaFechaEvento(id) != hoy && !eventoEnCola[id]) {
      if (!encolarEvento(id, esComida ? COMIDA : AGUA, hoy)) {
        Serial.println(F("ERROR,SCHEDULE_QUEUE_FULL"));
      }
    }
  }
}

void procesarCola() {
  if (estado != LISTO || cantidadCola == 0 || config.modo != AUTOMATICO) return;
  EventoProgramado evento = cola[inicioCola];
  inicioCola = (inicioCola + 1) % CAPACIDAD_COLA;
  --cantidadCola;
  eventoEnCola[evento.id] = false;

  // Registrar justo antes de accionar: un evento aun en cola sobrevive a un reinicio.
  registrarFechaEvento(evento.id, evento.fecha);
  iniciarEntrega(evento.tipo, origenEvento(evento.id));
}

// ---------- Protocolo Bluetooth ----------
bool leerNumero(const char* texto, uint16_t& resultado) {
  if (!texto || !*texto) return false;
  unsigned long valor = 0;
  for (const char* p = texto; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    valor = valor * 10UL + static_cast<unsigned long>(*p - '0');
    if (valor > 65535UL) return false;
  }
  resultado = static_cast<uint16_t>(valor);
  return true;
}

bool extraerNumero(uint16_t& n) {
  return leerNumero(strtok(NULL, ","), n);
}

bool pinCorrecto(const char* texto) {
  if (bloqueado()) {
    Serial.println(F("ERROR,PIN_LOCKED"));
    return false;
  }
  if (!texto || strcmp(texto, config.pin) != 0) {
    ++intentosFallidos;
    Serial.println(F("ERROR,INVALID_PIN"));
    if (intentosFallidos >= MAX_INTENTOS) {
      intentosFallidos = 0;
      bloqueoHasta = millis() + TIEMPO_BLOQUEO_MS;
      Serial.println(F("EVENT,PIN_LOCKED"));
    }
    return false;
  }
  intentosFallidos = 0;
  return true;
}

void enviarHorario(char prefijo, uint8_t indice, uint8_t hora, uint8_t minuto) {
  Serial.print(F("STATUS,"));
  Serial.print(prefijo);
  Serial.print(indice + 1);
  Serial.print(',');
  if (hora < 10) Serial.print('0');
  Serial.print(hora);
  Serial.print(':');
  if (minuto < 10) Serial.print('0');
  Serial.println(minuto);
}

void enviarEstado() {
  Serial.print(F("STATUS,MODE,"));
  Serial.println(config.modo == AUTOMATICO ? F("AUTO") : F("MANUAL"));
  for (uint8_t i = 0; i < 2; ++i) {
    enviarHorario('S', i, config.horaComida[i], config.minutoComida[i]);
  }
  for (uint8_t i = 0; i < 2; ++i) {
    enviarHorario('W', i, config.horaAgua[i], config.minutoAgua[i]);
  }
  Serial.print(F("STATUS,PORTION_MS,"));
  Serial.println(config.aperturaMs);
  Serial.print(F("STATUS,CARD,"));
  Serial.println(config.uidTam ? F("YES") : F("NO"));
  Serial.print(F("STATUS,STATE,"));
  Serial.println(static_cast<uint8_t>(estado));
  Serial.print(F("STATUS,ACTION,"));
  Serial.println(nombreAccion(accionActiva));
  Serial.print(F("STATUS,QUEUE,"));
  Serial.println(cantidadCola);
  if (rtcValido) {
    RtcDateTime t = rtc.GetDateTime();
    char fecha[25];
    snprintf(fecha, sizeof(fecha), "%04u-%02u-%02u %02u:%02u:%02u",
             t.Year(), t.Month(), t.Day(), t.Hour(), t.Minute(), t.Second());
    Serial.print(F("STATUS,TIME,"));
    Serial.println(fecha);
  } else {
    Serial.println(F("STATUS,TIME,UNSET"));
  }
  Serial.print(F("STATUS,TEMP,"));
  if (isnan(temperatura)) Serial.println(F("NA"));
  else Serial.println(temperatura, 1);
  Serial.print(F("STATUS,HUM,"));
  if (isnan(humedad)) Serial.println(F("NA"));
  else Serial.println(humedad, 0);
  Serial.println(F("STATUS,END"));
}

void procesarComando(char* linea) {
  char* accion = strtok(linea, ",");
  if (!accion) return;

  if (strcmp(accion, "HELP") == 0) {
    Serial.println(F("HELP: STATUS | MODE,pin,AUTO/MANUAL"));
    Serial.println(F("HELP: TIME,pin,YYYY,MM,DD,HH,MM,SS"));
    Serial.println(F("HELP: S1/S2,pin,HH,MM | W1/W2,pin,HH,MM"));
    Serial.println(F("HELP: PORTION,pin,ms | FEED,pin | WATER,pin"));
    Serial.println(F("HELP: ENROLL,pin | RESETCARD,pin"));
    Serial.println(F("HELP: PIN,pinViejo,pinNuevo"));
    return;
  }
  if (strcmp(accion, "STATUS") == 0) {
    enviarEstado();
    return;
  }

  if (estado != LISTO) {
    Serial.println(F("ERROR,BUSY"));
    return;
  }
  char* pin = strtok(NULL, ",");
  if (!pinCorrecto(pin)) return;

  if (strcmp(accion, "MODE") == 0) {
    char* modo = strtok(NULL, ",");
    if (!modo) { Serial.println(F("ERROR,PARAMS")); return; }
    if (strcmp(modo, "AUTO") == 0) config.modo = AUTOMATICO;
    else if (strcmp(modo, "MANUAL") == 0) {
      config.modo = MANUAL;
      vaciarCola();
    } else { Serial.println(F("ERROR,MODE")); return; }
    guardarConfig();
    Serial.println(F("OK,MODE"));
    return;
  }

  if (strcmp(accion, "TIME") == 0) {
    uint16_t p[6];
    for (uint8_t i = 0; i < 6; ++i) {
      if (!extraerNumero(p[i])) { Serial.println(F("ERROR,PARAMS")); return; }
    }
    if (p[0] < 2000 || p[0] > 2099 || p[1] < 1 || p[1] > 12 ||
        p[2] < 1 || p[2] > 31 || p[3] > 23 || p[4] > 59 || p[5] > 59) {
      Serial.println(F("ERROR,DATE")); return;
    }
    RtcDateTime fecha(p[0], p[1], p[2], p[3], p[4], p[5]);
    if (!fecha.IsValid()) { Serial.println(F("ERROR,DATE")); return; }
    rtc.SetIsWriteProtected(false);
    rtc.SetDateTime(fecha);
    rtc.SetIsRunning(true);
    rtcValido = rtc.IsDateTimeValid();
    vaciarCola();
    Serial.println(rtcValido ? F("OK,TIME") : F("ERROR,RTC"));
    return;
  }

  bool horarioComida = strcmp(accion, "S1") == 0 || strcmp(accion, "S2") == 0;
  bool horarioAgua = strcmp(accion, "W1") == 0 || strcmp(accion, "W2") == 0;
  if (horarioComida || horarioAgua) {
    uint8_t indice = accion[1] == '1' ? 0 : 1;
    uint16_t h, m;
    if (!extraerNumero(h) || !extraerNumero(m) || h > 23 || m > 59) {
      Serial.println(F("ERROR,SCHEDULE")); return;
    }
    uint8_t otro = 1 - indice;
    uint8_t* horas = horarioComida ? config.horaComida : config.horaAgua;
    uint8_t* minutos = horarioComida ? config.minutoComida : config.minutoAgua;
    if (h == horas[otro] && m == minutos[otro]) {
      Serial.println(F("ERROR,DUPLICATE_SCHEDULE")); return;
    }
    horas[indice] = static_cast<uint8_t>(h);
    minutos[indice] = static_cast<uint8_t>(m);
    if (horarioComida) config.ultimaFechaComida[indice] = 0;
    else config.ultimaFechaAgua[indice] = 0;
    vaciarCola();
    guardarConfig();
    Serial.println(F("OK,SCHEDULE"));
    return;
  }

  if (strcmp(accion, "PORTION") == 0) {
    uint16_t ms;
    if (!extraerNumero(ms) || ms < 300 || ms > 10000) {
      Serial.println(F("ERROR,PORTION_RANGE_300_10000")); return;
    }
    config.aperturaMs = ms;
    guardarConfig();
    Serial.println(F("OK,PORTION"));
    return;
  }

  if (strcmp(accion, "ENROLL") == 0) {
    estado = ENROLANDO_RFID;
    inicioEstado = millis();
    Serial.println(F("EVENT,SCAN_CARD"));
    return;
  }

  if (strcmp(accion, "RESETCARD") == 0) {
    config.uidTam = 0;
    memset(config.uid, 0, sizeof(config.uid));
    guardarConfig();
    Serial.println(F("OK,CARD_RESET"));
    return;
  }

  if (strcmp(accion, "FEED") == 0 || strcmp(accion, "WATER") == 0) {
    if (config.modo != MANUAL) {
      Serial.println(F("ERROR,NOT_MANUAL_MODE")); return;
    }
    iniciarEntrega(strcmp(accion, "FEED") == 0 ? COMIDA : AGUA, "BLUETOOTH");
    return;
  }

  if (strcmp(accion, "PIN") == 0) {
    char* nuevo = strtok(NULL, ",");
    if (!pinPermitido(nuevo)) {
      Serial.println(F("ERROR,PIN_MUST_BE_4_DIGITS_1_TO_9")); return;
    }
    strcpy(config.pin, nuevo);
    guardarConfig();
    Serial.println(F("OK,PIN_CHANGED"));
    return;
  }
  Serial.println(F("ERROR,UNKNOWN_COMMAND"));
}

void recibirBluetooth() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      comandoBT[posBT] = '\0';
      if (posBT != 0) procesarComando(comandoBT);
      posBT = 0;
    } else if (posBT < sizeof(comandoBT) - 1) {
      comandoBT[posBT++] = c;
    } else {
      posBT = 0;
      Serial.println(F("ERROR,LINE_TOO_LONG"));
    }
  }
}

// ---------- Ciclo principal ----------
void setup() {
  pinMode(PIN_LED_AGUA, OUTPUT);
  digitalWrite(PIN_LED_AGUA, LOW);
  pinMode(PIN_TECLADO_COLUMNAS, INPUT);
  pinMode(PIN_BOTONES, INPUT);
  for (uint8_t i = 0; i < 3; ++i) pinMode(PIN_TECLADO_FILAS[i], INPUT);
  pinMode(PIN_RFID_SS, OUTPUT);
  digitalWrite(PIN_RFID_SS, HIGH);

  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lineaLCD(0, "Alimentador UNO");
  lineaLCD(1, "Iniciando...");

  cargarConfig();
  dht.begin();
  SPI.begin();
  lectorRFID.PCD_Init();

  rtc.Begin();
  if (rtc.GetIsWriteProtected()) rtc.SetIsWriteProtected(false);
  if (!rtc.GetIsRunning()) rtc.SetIsRunning(true);
  rtcValido = rtc.IsDateTimeValid();

  servo.attach(PIN_SERVO);
  servo.write(ANGULO_CERRADO);
  delay(700);
  servo.detach();

  Serial.println(F("READY,ALIMENTADOR_UNO"));
  if (!rtcValido) Serial.println(F("ERROR,RTC_NOT_SET"));
  enviarEstado();
}

void loop() {
  recibirBluetooth();
  revisarTeclado();
  revisarBotones();
  revisarRFID();
  revisarHorarios();
  actualizarDispensador();
  procesarCola();
  leerSensor();

  if (estado == ENROLANDO_RFID &&
      millis() - inicioEstado >= TIEMPO_ENROLAR_MS) {
    estado = LISTO;
    digitalWrite(PIN_LED_AGUA, LOW);
    Serial.println(F("EVENT,ENROLL_TIMEOUT"));
    mostrarAviso("RFID no leido", "Tiempo agotado", 1800);
  }
  actualizarLCD();
}