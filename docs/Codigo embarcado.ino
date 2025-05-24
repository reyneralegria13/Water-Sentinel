#include <WiFi.h>
#include <time.h>
#include <IOXhop_FirebaseESP32.h>
#include <ArduinoJson.h>    // v5.x
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// ---------------------------------------------------------------------
// Configurações do Firebase e WiFi / NTP
// ---------------------------------------------------------------------

const char* ssid     = "ICET-WIFI-D";
const char* password = "1ucf34tm";
const char* ntpServer         = "pool.ntp.org";
const long  gmtOffset_sec     = -4 * 3600;
const int   daylightOffset_sec= 3600;

// ---------------------------------------------------------------------
// Sensores e pinos
// ---------------------------------------------------------------------
#define DHTPIN    12
#define DHTTYPE   DHT11
DHT dht(DHTPIN, DHTTYPE);

Adafruit_BMP085 bmp180;

#define TRIG_PIN 27
#define ECHO_PIN 14

const float LADO_CM     = 7.5;   // base quadrada
const float ALTURA_CM   = 7.0;   // altura total do recipiente

const uint8_t  SAMPLES      = 11;
const uint16_t SAMPLE_DELAY = 50;

float distEmpty = 0.0;

// ---------------------------------------------------------------------
// Temporizadores não bloqueantes
// ---------------------------------------------------------------------
const unsigned long INTERVAL_LEVEL   = 1000;   // 1 s
const unsigned long INTERVAL_FULL    = 5000;   // 5 s
unsigned long prevLevelMillis = 0;
unsigned long prevFullMillis  = 0;

// ---------------------------------------------------------------------
// Helpers de leitura HC-SR04
// ---------------------------------------------------------------------
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long µ = pulseIn(ECHO_PIN, HIGH, 30000);
  if (µ == 0) return -1;
  return (µ * 0.0343f) / 2.0f;
}

float medianFilter() {
  float v[SAMPLES];
  for (uint8_t i = 0; i < SAMPLES; ++i) {
    float d = measureDistance();
    v[i] = (d > 0) ? d : (ALTURA_CM + 5);
    delay(SAMPLE_DELAY);
  }
  // bubble sort
  for (uint8_t i = 0; i < SAMPLES-1; ++i)
    for (uint8_t j = i+1; j < SAMPLES; ++j)
      if (v[j] < v[i]) { float t=v[i]; v[i]=v[j]; v[j]=t; }
  return v[SAMPLES/2];
}

// ---------------------------------------------------------------------
// Conexões e setup
// ---------------------------------------------------------------------
void connectToWifi() {
  Serial.print("Conectando ao WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println(" conectado!");
}

void setup() {
  Serial.begin(9600);
  connectToWifi();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  dht.begin();
  if (!bmp180.begin()) {
    Serial.println("Erro: BMP180 não encontrado!");
    while (1) delay(1000);
  }
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Setup concluído.");

  // calibração HC-SR04 (vazio)
  Serial.println("Calibrando distância sem água...");
  delay(2000);
  float sum = 0;
  for (uint8_t i = 0; i < SAMPLES; ++i) {
    sum += medianFilter();
    delay(100);
  }
  distEmpty = sum / SAMPLES;
  Serial.printf("Distância sem água: %.2f cm\n", distEmpty);
  Serial.println("-------------------------------");

  // inicializa temporizadores
  prevLevelMillis = millis();
  prevFullMillis  = millis();
}

// ---------------------------------------------------------------------
// Loop não bloqueante
// ---------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // —— 1) Tarefa de nível de água (1 s) ——
  if (now - prevLevelMillis >= INTERVAL_LEVEL) {
    prevLevelMillis = now;
    float rawDist  = medianFilter();
    float alturaCm = constrain(distEmpty - rawDist, 0.0, ALTURA_CM);
    float volML    = LADO_CM * LADO_CM * alturaCm;

    // envia apenas o volume
    Firebase.set("sensor/data/volume", volML);
    Serial.printf("[Nível] V=%.0fmL\n", volML);
  }

  // —— 2) Tarefa completa (5 s) ——
  if (now - prevFullMillis >= INTERVAL_FULL) {
    prevFullMillis = now;

    // timestamp
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Falha ao obter tempo");
      return;
    }
    char tsData[20], tsHora[20];
    strftime(tsData, sizeof(tsData), "%Y-%m-%d", &timeinfo);
    strftime(tsHora, sizeof(tsHora), "%H:%M:%S", &timeinfo);

    // DHT11 e BMP180
    float temperatura = dht.readTemperature();
    float umidade     = dht.readHumidity();
    float pressao     = bmp180.readSealevelPressure() / 100.0;

    // HC-SR04 (já calibrado)
    float rawDist2 = medianFilter();
    float altura2  = constrain(distEmpty - rawDist2, 0.0, ALTURA_CM);
    float volML2   = LADO_CM * LADO_CM * altura2;
    float volL2    = volML2 / 1000.0f;

    // debug
    Serial.printf("[%s %s]\n"
                  "T=%.1f°C H=%.1f%% P=%.1fhPa\n"
                  "Alt=%.1fcm V=%.0fmL(%.3fL)\n\n",
                  tsData, tsHora,
                  temperatura, umidade, pressao,
                  altura2, volML2, volL2);

    // JSON e envio
    DynamicJsonBuffer jb;
    JsonObject& timestamp = jb.createObject();
    timestamp["data"] = tsData;
    timestamp["hora"] = tsHora;

    JsonObject& data = jb.createObject();
    data["temperatura"] = temperatura;
    data["umidade"]     = umidade;
    data["pressao"]     = pressao;
    data["volume"]   = volML2;
    data["volume_L"]    = volL2;

    Firebase.set("timestamp/", timestamp);
    Firebase.set("sensor/data/", data);

    if (Firebase.failed()) {
      Serial.print("Erro: ");
      Serial.println(Firebase.error());
    } else {
      Serial.println("Dados completos enviados!\n");
    }
  }
}
