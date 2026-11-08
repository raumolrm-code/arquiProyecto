/*
  ALIMENTADOR DE MASCOTAS CON SEGURIDAD - Arduino UNO R3
  Arquitectura del Computador (2S 2026)

  Version sin dispensador de agua: UN solo servo para concentrado.
  Teclado fisico 4x4 usando SOLO sus primeras 3 filas y 3 columnas.

  Mapeo:
  D0 RX  <- TX HC-05           D1 TX -> RX HC-05 (adaptar 5 V a 3.3 V)
  D2,D3,D4 = filas teclado    D5,D6,D7 = columnas teclado
  D8 = servo SG90             D9 = boton a GND (INPUT_PULLUP)
  D10 = RC522 SDA/SS          D11 = RC522 MOSI
  D12 = RC522 MISO            D13 = RC522 SCK
  A0 = DS1302 CE              A1 = DS1302 IO
  A2 = DS1302 CLK             A3 = DHT DATA
  A4 = I2C SDA LCD            A5 = I2C SCL LCD
  RC522 RST -> 3.3 V con pull-up 10 kohm (sin pin Arduino)

  Librerias (administrador de Arduino IDE):
  - MFRC522 (miguelbalboa)
  - Keypad (Mark Stanley/Alexander Brevig)
  - Rtc by Makuna
  - LiquidCrystal I2C (version con lcd.init())
  - DHT sensor library (Adafruit) + Adafruit Unified Sensor
  Integradas: SPI, Wire, EEPROM, Servo.

  COMANDOS (una linea ASCII terminada en '\n', 9600 baudios):
  HELP
  STATUS
  MODE,1234,AUTO          o MODE,1234,MANUAL
  TIME,1234,2026,11,06,14,30,00
  S1,1234,08,00           primera alimentacion automatica
  S2,1234,18,00           segunda alimentacion automatica
  PORTION,1234,2000      servo abierto 2000 ms (NO equivale a gramos)
  ENROLL,1234            acercar tarjeta en los siguientes 15 s
  RESETCARD,1234         borrar tarjeta autorizada
  FEED,1234              alimentar desde app, solamente modo MANUAL
  PIN,1234,2345          modificar PIN de 4 teclas (1-9)

  IMPORTANTE: Seguridad para prototipo academico, no para uso sensible.
  UID RFID y PIN transmitido por Bluetooth clasico no son autenticacion
  criptografica robusta. Primero probar el servo SIN comida y supervisado.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Keypad.h>
#include <MFRC522.h>
#include <RtcDS1302.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <string.h>
#include <stddef.h>

// ---------- Pines ----------
const uint8_t PIN_SERVO = 8;
const uint8_t PIN_BOTON = 9;
const uint8_t PIN_RFID_SS = 10;
const uint8_t PIN_RTC_CE = A0;
const uint8_t PIN_RTC_IO = A1;
const uint8_t PIN_RTC_CLK = A2;
const uint8_t PIN_DHT = A3;

// Ajustar a DHT11 si ese es el sensor del kit.
#define TIPO_DHT DHT11

// Ajustar 0x27 si el escaner I2C reporta otra direccion (p. ej. 0x3F).
const uint8_t LCD_DIRECCION = 0x27;

// Calibrar de acuerdo con la compuerta mecanica real.
const uint8_t ANGULO_CERRADO = 0;
const uint8_t ANGULO_ABIERTO = 90;
const unsigned long TIEMPO_CIERRE_MS = 650UL;
const unsigned long TIEMPO_PIN_MS = 20000UL;
const unsigned long TIEMPO_ENROLAR_MS = 15000UL;
const unsigned long TIEMPO_BLOQUEO_MS = 30000UL;
const uint8_t MAX_INTENTOS = 3;

// ---------- Teclado 3x3 ----------
const byte FILAS = 3;
const byte COLUMNAS = 3;
char teclas[FILAS][COLUMNAS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'}
};
byte pinesFilas[FILAS] = {2, 3, 4};
byte pinesColumnas[COLUMNAS] = {5, 6, 7};
Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas,
                        FILAS, COLUMNAS);             // Teclado fisico 3x3 ( Caracteres 1-9)

MFRC522 lectorRFID(PIN_RFID_SS, MFRC522::UNUSED_PIN); // Lector RFID (el de la tarjeta)
ThreeWire rtcWire(PIN_RTC_IO, PIN_RTC_CLK, PIN_RTC_CE);
RtcDS1302<ThreeWire> rtc(rtcWire);  // Reloj
LiquidCrystal_I2C lcd(LCD_DIRECCION, 16, 2);  // Pantalla LCD 16x2
DHT dht(PIN_DHT, TIPO_DHT);  // Sensor de temperatura y humedad 
Servo servo;  // Servo para abrir/cerrar compuerta

// ---------- Configuracion persistida en EEPROM ----------
enum Modo : uint8_t { 
  MANUAL = 0,
  AUTOMATICO = 1 };
const uint32_t FIRMA_EEPROM = 0xA7152026UL;   // Para detectar EEPROM no inicializada o corrupta.

struct Configuracion {
  uint32_t firma;
  char pin[5];                  // PIN de 4 digitos 1-9 + terminador NUL
  uint8_t modo;
  uint8_t hora[2];
  uint8_t minuto[2];
  uint16_t aperturaMs;
  uint8_t uidTam;
  uint8_t uid[10];
  uint32_t ultimaFecha[2];    // AAAAMMDD (no repetir alimentacion por reinicio)
  uint16_t checksum;
};
Configuracion config;

enum Estado : uint8_t {
  LISTO,            // Esperando accion
  ESPERANDO_PIN,    // Esperando cuatro numeros del PIN 
  ENROLANDO_RFID,   // esperando una tarjeta para registrar
  DISPENSANDO,      // Dispensando comida
  CERRANDO          // Cerrando compuerta
};
Estado estado = LISTO;

// --------- Variables de estado ----------
bool rtcValido = false; 
float temperatura = NAN;
float humedad = NAN;

char pinTecleado[5] = {0};    // Almacena el PIN ingresado por teclado fisico
uint8_t cantidadTecleada = 0; // Cantidad de digitos ingresados por teclado fisico
uint8_t intentosFallidos = 0; // Cantidad de intentos fallidos
const char* origenPendiente = "";
const char* origenDispensacion = "";
unsigned long inicioEstado = 0;
unsigned long bloqueoHasta = 0;
unsigned long ultimoLCD = 0;
unsigned long ultimoDHT = 0;
unsigned long ultimoRTC = 0;
unsigned long ultimoRFID = 0;
unsigned long ultimoBoton = 0;
bool botonAnterior = HIGH;

// --------- Variables de comunicacion Bluetooth ----------
char comandoBT[72]; // Texto Comando recibido por Bluetooth (terminado en '\n')
uint8_t posBT = 0;  // Posición actual en el buffer de comando
char aviso1[17] = ""; // Mensaje que se muestra en la primera linea del LCD
char aviso2[17] = ""; // Mensaje que se muestra en la segunda linea del LCD
unsigned long avisoHasta = 0; // Tiempo hasta el cual se muestra el aviso

// ---------- EEPROM ----------
uint16_t checksumConfig() {
  uint16_t suma = 0xA5A5;
  const uint8_t* datos = reinterpret_cast<const uint8_t*>(&config);   // Reinterpreta el config como un puntero de u8
  for (size_t i = 0; i < offsetof(Configuracion, checksum); ++i) {
    suma = static_cast<uint16_t>((suma ^ datos[i]) * 31U);            // Mezcla con un primo para difundir los bits
  }
  return suma;
}

// --------- Validacion de PIN ----------
bool pinPermitido(const char* p) {
  if (!p || strlen(p) != 4) return false;
  for (uint8_t i = 0; i < 4; ++i) {
    if (p[i] < '1' || p[i] > '9') return false;
  }
  return true;
}

// --------- Configuracion ----------
void guardarConfig() {
  config.checksum = checksumConfig();
  EEPROM.put(0, config);
}

// ---------- Valores iniciales de la configuracion ----------
void valoresIniciales() {
  memset(&config, 0, sizeof(config));   // Inicializa toda la estructura a cero
  config.firma = FIRMA_EEPROM;
  strcpy(config.pin, "1234");           // Cambiarlo desde Bluetooth en la configuracion inicial.
  config.modo = MANUAL;
  config.hora[0] = 8;
  config.minuto[0] = 0;
  config.hora[1] = 18;
  config.minuto[1] = 0;
  config.aperturaMs = 2000;
  guardarConfig();
}

// --------- Cargar configuracion desde EEPROM ----------
void cargarConfig() {
  EEPROM.get(0, config);
  if (config.firma != FIRMA_EEPROM ||config.checksum != checksumConfig() ||
      config.pin[4] != '\0' || !pinPermitido(config.pin) ||
      config.modo > AUTOMATICO ||
      config.hora[0] > 23 || config.hora[1] > 23 ||
      config.minuto[0] > 59 || config.minuto[1] > 59 ||
      config.aperturaMs < 300 || config.aperturaMs > 10000 ||
      config.uidTam > sizeof(config.uid)) {
    valoresIniciales();
  }
}

// ---------- Interfaz / utilidad ----------
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

// --------- Escribe en una linea del LCD ----------
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

void actualizarLCD() {
  unsigned long ahora = millis();
  if (ahora - ultimoLCD < 500UL) return;
  ultimoLCD = ahora;

  if (estado == DISPENSANDO || estado == CERRANDO) {
    lineaLCD(0, estado == DISPENSANDO ? "Sirviendo comida" : "Cerrando puerta");
    lineaLCD(1, "Concentrado");
    return;
  }
  if (estado == ESPERANDO_PIN) {
    lineaLCD(0, "Ingrese PIN:");
    char progreso[17] = "____";
    for (uint8_t i = 0; i < cantidadTecleada; ++i) progreso[i] = '*';
    lineaLCD(1, progreso);
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

// --------- Lee temperatura y humedad ----------
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

// ---------- Iniciar dispensacion ----------
void iniciarDispensacion(const char* origen) {
  if (estado != LISTO) {
    Serial.println(F("ERROR,BUSY"));
    return;
  }
  origenDispensacion = origen;
  servo.attach(PIN_SERVO);
  servo.write(ANGULO_ABIERTO);
  estado = DISPENSANDO;
  inicioEstado = millis();
  Serial.print(F("EVENT,FEED_START,"));
  Serial.println(origenDispensacion);
}

// ---------- Actualizar dispensador ----------
void actualizarDispensador() {
  const unsigned long ahora = millis();
  // Si ha pasado el tiempo de apertura, se cierra
  if (estado == DISPENSANDO && ahora - inicioEstado >= config.aperturaMs) {
    servo.write(ANGULO_CERRADO);
    estado = CERRANDO;
    inicioEstado = ahora;
  } else if (estado == CERRANDO && ahora - inicioEstado >= TIEMPO_CIERRE_MS) {  // Si ha pasado el tiempo de cierre, se libera el servo y vuelve a LISTO
    servo.detach();
    estado = LISTO;
    Serial.print(F("EVENT,FEED_END,"));
    Serial.println(origenDispensacion);
    mostrarAviso("Alimento servido", "Operacion lista", 1800);
  }
}

// ---------- Autenticacion manual ----------
void solicitarPin(const char* origen) {
  if (estado != LISTO || config.modo != MANUAL) return;
  if (bloqueado()) {
    Serial.println(F("ERROR,PIN_LOCKED"));
    return;
  }
  memset(pinTecleado, 0, sizeof(pinTecleado));
  cantidadTecleada = 0;
  origenPendiente = origen;
  estado = ESPERANDO_PIN;
  inicioEstado = millis();
  Serial.print(F("EVENT,PIN_REQUIRED,"));
  Serial.println(origenPendiente);
}

// --------- Manejo de error de PIN ----------
void errorPin() {
  ++intentosFallidos;
  estado = LISTO;
  Serial.println(F("ERROR,INVALID_PIN"));
  mostrarAviso("PIN incorrecto", "Acceso denegado", 1800);
  if (intentosFallidos >= MAX_INTENTOS) {
    intentosFallidos = 0;
    bloqueoHasta = millis() + TIEMPO_BLOQUEO_MS;
    Serial.println(F("EVENT,PIN_LOCKED"));
  }
}

// ---------- Revisar teclado ----------
void revisarTeclado() {
  if (estado != ESPERANDO_PIN) return;
  // Si el usuario no teclea nada en TIEMPO_PIN_MS, se cancela la operacion.
  if (millis() - inicioEstado >= TIEMPO_PIN_MS) {
    estado = LISTO;
    Serial.println(F("EVENT,PIN_TIMEOUT"));
    mostrarAviso("Tiempo agotado", "Reintente", 1800);
    return;
  }

  char tecla = teclado.getKey();
  if (tecla < '1' || tecla > '9') return;
  if (cantidadTecleada >= 4) return;
  // Almacena el PIN ingresado por teclado fisico y agrega terminador 
  pinTecleado[cantidadTecleada++] = tecla;
  pinTecleado[cantidadTecleada] = '\0';
  if (cantidadTecleada == 4) {
    if (strcmp(pinTecleado, config.pin) == 0) {
      intentosFallidos = 0;
      estado = LISTO;
      iniciarDispensacion(origenPendiente);
    } else {
      errorPin();
    }
  }
}

// ---------- Sensor de la tarjeta ---------
void revisarRFID() {
  if (estado != ENROLANDO_RFID &&
      !(estado == LISTO && config.modo == MANUAL)) return;
  if (millis() - ultimoRFID < 100UL) return;
  ultimoRFID = millis();
  if (!lectorRFID.PICC_IsNewCardPresent()) return;        // Detecta si es una tarjeta nueva
  if (!lectorRFID.PICC_ReadCardSerial()) return;          // No puede leer el UID de la tarjeta 

  const uint8_t tam = lectorRFID.uid.size;                // Guarda el tamaño del UID de la tarjeta leida
  if (estado == ENROLANDO_RFID) {                         // Si espera registrar una tarjeta
    if (tam > 0 && tam <= sizeof(config.uid)) {
      config.uidTam = tam;
      memset(config.uid, 0, sizeof(config.uid));          // Limpia el UID anterior
      memcpy(config.uid, lectorRFID.uid.uidByte, tam);    // Copia el nuevo UID
      guardarConfig();
      estado = LISTO;
      Serial.println(F("EVENT,CARD_ENROLLED"));
      mostrarAviso("RFID registrado", "Correctamente", 2000); // Muestra mensaje de exito en el LCD
    } else {
      Serial.println(F("ERROR,UID_SIZE"));
    }
  } else {                                                // Si no espera ninguan nueva tarjeta
    bool autorizada = config.uidTam == tam && tam > 0 &&
                       memcmp(config.uid, lectorRFID.uid.uidByte, tam) == 0;  // Revisa que la tarjeta leida sea la autorizada
    if (autorizada) solicitarPin("RFID");                 // Si es tarjeta autorizadam solicita el pin
    else {
      Serial.println(F("EVENT,CARD_DENIED"));
      mostrarAviso("RFID no valido", "Acceso denegado", 1200);
    }
  }
  lectorRFID.PICC_HaltA();
  lectorRFID.PCD_StopCrypto1();
}

// --------- Boton fisico ----------
void revisarBoton() {                                     // Revisa si el boton fisico fue presionado 
  bool actual = digitalRead(PIN_BOTON);
  if (botonAnterior == HIGH && actual == LOW &&
      millis() - ultimoBoton > 250UL) {
    ultimoBoton = millis();
    solicitarPin("BOTON");
  }
  botonAnterior = actual;
}

// ---------- Reloj / alimentacion automatica ----------
void revisarHorarios() {
  if (estado != LISTO || config.modo != AUTOMATICO || !rtcValido) return; // Revisa que sea modo automatico y reloj valido
  if (millis() - ultimoRTC < 500UL) return;
  ultimoRTC = millis();

  // Obtiene la fecha y hora del reloj
  RtcDateTime t = rtc.GetDateTime();
  if (!t.IsValid()) {
    rtcValido = false;
    Serial.println(F("ERROR,RTC_INVALID"));
    return;
  }
  uint32_t hoy = claveFecha(t);
  for (uint8_t i = 0; i < 2; ++i) {
    if (t.Hour() == config.hora[i] && t.Minute() == config.minuto[i] &&
        config.ultimaFecha[i] != hoy) {
      // Registrar ANTES de accionar para evitar doble dosis tras reinicio.
      config.ultimaFecha[i] = hoy;
      guardarConfig();
      iniciarDispensacion(i == 0 ? "AUTO1" : "AUTO2");
      break;
    }
  }
}

// ---------- Protocolo Bluetooth ----------
bool leerNumero(const char* texto, uint16_t& resultado) {                   // Convierte un texto a un uint16_t
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

// --------- Extraer un numero de la cadena strtok ----------
bool extraerNumero(uint16_t& n) {
  return leerNumero(strtok(NULL, ","), n);
}

// --------- Toma un texto y lo compara con el pin ----------
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

void enviarEstado() {
  Serial.print(F("STATUS,MODE,"));
  Serial.println(config.modo == AUTOMATICO ? F("AUTO") : F("MANUAL"));
  Serial.print(F("STATUS,S1,"));
  if (config.hora[0] < 10) Serial.print('0');
  Serial.print(config.hora[0]);
  Serial.print(':');
  if (config.minuto[0] < 10) Serial.print('0');
  Serial.println(config.minuto[0]);
  Serial.print(F("STATUS,S2,"));
  if (config.hora[1] < 10) Serial.print('0');
  Serial.print(config.hora[1]);
  Serial.print(':');
  if (config.minuto[1] < 10) Serial.print('0');
  Serial.println(config.minuto[1]);
  Serial.print(F("STATUS,PORTION_MS,"));
  Serial.println(config.aperturaMs);
  Serial.print(F("STATUS,CARD,"));
  Serial.println(config.uidTam ? F("YES") : F("NO"));
  Serial.print(F("STATUS,STATE,"));
  Serial.println(static_cast<uint8_t>(estado));
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

// --------- Procesar comando recibido como texto ----------
void procesarComando(char* linea) {
  char* accion = strtok(linea, ",");
  if (!accion) return;

  if (strcmp(accion, "HELP") == 0) {
    Serial.println(F("HELP: STATUS | MODE,pin,AUTO/MANUAL"));
    Serial.println(F("HELP: TIME,pin,YYYY,MM,DD,HH,MM,SS"));
    Serial.println(F("HELP: S1,pin,HH,MM | S2,pin,HH,MM"));
    Serial.println(F("HELP: PORTION,pin,ms | FEED,pin"));
    Serial.println(F("HELP: ENROLL,pin | RESETCARD,pin"));
    Serial.println(F("HELP: PIN,pinViejo,pinNuevo"));
    return;
  }
  if (strcmp(accion, "STATUS") == 0) {
    enviarEstado();
    return;
  }

  // Las configuraciones no pueden modificarse durante una operacion.
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
    else if (strcmp(modo, "MANUAL") == 0) config.modo = MANUAL;
    else { Serial.println(F("ERROR,MODE")); return; }
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
    Serial.println(rtcValido ? F("OK,TIME") : F("ERROR,RTC"));
    return;
  }

  if (strcmp(accion, "S1") == 0 || strcmp(accion, "S2") == 0) {
    uint8_t indice = (accion[1] == '1') ? 0 : 1;
    uint16_t h, m;
    if (!extraerNumero(h) || !extraerNumero(m) || h > 23 || m > 59) {
      Serial.println(F("ERROR,SCHEDULE")); return;
    }
    uint8_t otro = 1 - indice;
    if (h == config.hora[otro] && m == config.minuto[otro]) {
      Serial.println(F("ERROR,DUPLICATE_SCHEDULE")); return;
    }
    config.hora[indice] = h;
    config.minuto[indice] = m;
    config.ultimaFecha[indice] = 0;
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

  if (strcmp(accion, "FEED") == 0) {
    if (config.modo != MANUAL) {
      Serial.println(F("ERROR,NOT_MANUAL_MODE")); return;
    }
    iniciarDispensacion("BLUETOOTH");
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
      // Evitar desbordamientos del buffer con comandos demasiado largos.
      posBT = 0;
      Serial.println(F("ERROR,LINE_TOO_LONG"));
    }
  }
}

// ---------- Ciclo principal ----------
void setup() {
  pinMode(PIN_BOTON, INPUT_PULLUP);
  botonAnterior = digitalRead(PIN_BOTON);
  pinMode(PIN_RFID_SS, OUTPUT);
  digitalWrite(PIN_RFID_SS, HIGH);

  // D0/D1 son la UART hardware compartida con el USB del Arduino.
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

  // Posicion inicial. Al iniciar siempre se intenta cerrar la compuerta.
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
  revisarBoton();
  revisarRFID();
  revisarHorarios();
  actualizarDispensador();
  leerSensor();

  if (estado == ENROLANDO_RFID &&
      millis() - inicioEstado >= TIEMPO_ENROLAR_MS) {
    estado = LISTO;
    Serial.println(F("EVENT,ENROLL_TIMEOUT"));
    mostrarAviso("RFID no leido", "Tiempo agotado", 1800);
  }
  actualizarLCD();
}
