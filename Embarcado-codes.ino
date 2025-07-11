#include <WiFi.h>
#include <time.h>
#include <IOXhop_FirebaseESP32.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// ───► ML
#include "risk_model.h"                                         // array TFLite
// CORRETO
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include "tensorflow/lite/version.h"

// ------------------------------ Firebase / Wi-Fi
#define FIREBASE_HOST ""
#define FIREBASE_AUTH ""

const char* ssid              = "NAVIR_2.4G";
const char* password          = "";
const char* ntpServer         = "pool.ntp.org";
const long  gmtOffset_sec     = -5 * 3600;
const int   daylightOffset_sec= 3600;

// ------------------------------ Sensores e pinos
#define DHTPIN    25
#define DHTTYPE   DHT11
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp180;

#define NIVEL_ANALOG_PIN 34
#define BUZZER_PIN      26

const float ALTURA_CILINDRO_MM = 45.0f;
const float RAIO_CILINDRO_CM   = 3.0f;
const float VOLUME_CILINDRO_CM3 =
      3.1416f * RAIO_CILINDRO_CM * RAIO_CILINDRO_CM * (ALTURA_CILINDRO_MM / 10.0f);

const unsigned long INTERVAL_LEVEL = 1000UL;
const unsigned long INTERVAL_FULL  = 5000UL;
const unsigned long BEEP_INTERVAL  = 30000UL;

unsigned long prevLevelMillis = 0;
unsigned long prevFullMillis  = 0;
unsigned long lastBeepMillis  = 0;

// ────────────────────────────────► ML: arena, intérprete, escala
constexpr int kArenaSize = 6 * 1024;
uint8_t tensor_arena[kArenaSize];
tflite::MicroInterpreter* interpreter;

constexpr float MEAN_X = 10.54f;   //  <<< troque!
constexpr float STD_X  =  8.82f;   //  <<< troque!

const char* riskNames[5] = {
  "sem_risco", "chuva",
  "chuva_intensa", "alto_risco", "alagando"
};

// ------------------------------ Wi-Fi
void connectToWifi() {
  Serial.print("Conectando ao WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println(" conectado!");
}

// ------------------------------ Leitura do sensor de nível
float readWaterLevel() {
  const int samples = 10;
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(NIVEL_ANALOG_PIN);
    delay(10);
  }
  float voltage = (sum / samples) * (3.3f / 4095.0f);
  float nivel_mm = (voltage / 3.3f) * ALTURA_CILINDRO_MM;
  return constrain(nivel_mm, 0.0f, ALTURA_CILINDRO_MM);
}

// ────────────────────────────────► ML: setup
void setupML() {
  static tflite::AllOpsResolver resolver;
  const tflite::Model* model = tflite::GetModel(risk_model_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Versão do modelo incompatível!");
    while (true);
  }
  static tflite::MicroInterpreter static_interp(
        model, resolver, tensor_arena, kArenaSize);
  interpreter = &static_interp;
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Falha em AllocateTensors()");
    while (true);
  }
  Serial.printf("Modelo OK, arena usada: %u bytes\n",
                interpreter->arena_used_bytes());
}

// ────────────────────────────────► ML: função de inferência
uint8_t classify_risk(float x_mm) {
  float x_scaled = (x_mm - MEAN_X) / STD_X;
  interpreter->input(0)->data.f[0] = x_scaled;
  interpreter->Invoke();
  float* out = interpreter->output(0)->data.f;
  uint8_t cls = 0; float best = out[0];
  for (uint8_t i = 1; i < 5; ++i)
    if (out[i] > best) { best = out[i]; cls = i; }
  return cls;                  // 0..4
}

// ------------------------------ Setup
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT); noTone(BUZZER_PIN);

  connectToWifi();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  dht.begin();
  if (!bmp180.begin())
    Serial.println("BMP180 não encontrado, seguindo sem pressão.");
  setupML();                 // ► ML

  prevLevelMillis = prevFullMillis = lastBeepMillis = millis();
  Serial.printf("Volume máx: %.1f cm³\n", VOLUME_CILINDRO_CM3);
}

// ------------------------------ Loop
void loop() {
  unsigned long now = millis();

  // 1) Enviar nível a cada 1 s
  if (now - prevLevelMillis >= INTERVAL_LEVEL) {
    prevLevelMillis = now;
    float nivel_mm = readWaterLevel();
    float percent  = (nivel_mm / ALTURA_CILINDRO_MM) * 100.0f;
    Firebase.set("sensor/data/nivel_mm", (int)nivel_mm);
    Serial.printf("[Nível] H=%.1f mm (%.1f%%)\n", nivel_mm, percent);
  }

  // 2) Pacote completo a cada 5 s
  if (now - prevFullMillis >= INTERVAL_FULL) {
    prevFullMillis = now;

    struct tm t; if (!getLocalTime(&t)) { Serial.println("Falha NTP"); return; }
    char tsData[20], tsHora[20];
    strftime(tsData, sizeof tsData, "%Y-%m-%d", &t);
    strftime(tsHora, sizeof tsHora, "%H:%M:%S", &t);

    float temperatura   = dht.readTemperature();
    float umidade       = dht.readHumidity();
    int   pressao_hPa   = bmp180.begin() ? bmp180.readSealevelPressure()/100.0f : 0;
    float nivel_mm      = readWaterLevel();
    float percent       = (nivel_mm / ALTURA_CILINDRO_MM) * 100.0f;
    float volume_cm3    = 3.1416f * RAIO_CILINDRO_CM * RAIO_CILINDRO_CM *
                          (nivel_mm / 10.0f);

    // ► ML: classificação
    uint8_t alertLevel  = classify_risk(nivel_mm);
    const char* label   = riskNames[alertLevel];

    // ► Lógica do buzzer
    if (alertLevel == 1) {
      if (now - lastBeepMillis >= BEEP_INTERVAL) {
        tone(BUZZER_PIN, 1000, 400); lastBeepMillis = now;
      }
    } else if (alertLevel == 2) {
      if (now - lastBeepMillis >= 15000) {
        tone(BUZZER_PIN, 1500, 500); lastBeepMillis = now;
      }
    } else if (alertLevel >= 3) {
      tone(BUZZER_PIN, 2000);
    } else {
      noTone(BUZZER_PIN);
    }

    Serial.printf("[%s %s] T=%.1f°C H=%.1f%% P=%dhPa  |  H=%.1f mm  Alerta=%s\n",
                  tsData, tsHora, temperatura, umidade, pressao_hPa,
                  nivel_mm, label);

    // ► Envio Firebase
    DynamicJsonBuffer jb;
    JsonObject& ts = jb.createObject();
    ts["data"] = tsData; ts["hora"] = tsHora;

    JsonObject& data = jb.createObject();
    data["temperatura"] = temperatura;
    data["umidade"]     = umidade;
    data["pressao"]     = pressao_hPa;
    data["nivel_mm"]    = nivel_mm;
    data["percentual"]  = percent;
    data["volume"]      = volume_cm3;
    data["alertLevel"]  = alertLevel;
    data["alertLabel"]  = label;

    Firebase.set("timestamp/", ts);
    Firebase.set("sensor/data/", data);

    if (Firebase.failed())
      Serial.printf("Erro Firebase: %s\n", Firebase.error().c_str());
    else
      Serial.println("Dados enviados!");
  }
}

