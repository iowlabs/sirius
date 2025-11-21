/*
 * ESP32 → HTTP POST (JSON)
 * Envía: { "device_id":"ESP32-01", "temp_c":23.5, "hum_pc":55.2, "pm25_ugm3":12.3, "pm10_ugm3":20.1, "ts":"2025-10-09T12:00:00Z" }
 *
 * Requisitos: ESP32 + WiFi. No requiere librerías extra.
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

// ====== CONFIGURA ESTO ======
const char* WIFI_SSID = "UDD";
const char* WIFI_PASS = "udd_2019";

// Ejemplo: "http://192.168.1.100:8000/api/mediciones" o "http://tu-servidor.com/endpoint"
const char* SERVER_URL = "http://webhook.sm.cl/test-webhook";

// Identificador de tu dispositivo
const char* DEVICE_ID = "sirius-01";

// Intervalo de envío (ms)
const uint32_t POST_INTERVAL_MS = 30 * 1000;

// Si quieres timestamp UTC correcto, configura NTP (opcional pero recomendado)
const char* NTP_SERVER = "pool.ntp.org";
const long   GMT_OFFSET_SEC = 0;     // Ajusta si quieres hora local
const int    DST_OFFSET_SEC = 0;     // Horario de verano
// ============================

uint32_t lastPost = 0;

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando a WiFi");
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 60) { // ~30s
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("No se pudo conectar a WiFi.");
  }
}

String iso8601UTC() {
  // Requiere que NTP se haya configurado; si no, devuelve "null"
  time_t now;
  time(&now);
  if (now < 1700000000) { // si el tiempo no está sincronizado (~2023-11-14 epoch)
    return "null";
  }
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

bool postJSON(const String& jsonPayload) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] WiFi no conectado. Reintentando conexión...");
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return false;
  }

  HTTPClient http;
  http.setTimeout(10000); // 10s
  http.begin(SERVER_URL); // Para HTTPS con certificado propio usar WiFiClientSecure y http.begin(client, url)
  http.addHeader("Content-Type", "application/json");
  // Si tu API requiere token:
  // http.addHeader("Authorization", "Bearer TU_TOKEN");

  Serial.println("[HTTP] Enviando JSON:");
  Serial.println(jsonPayload);

  int httpCode = http.POST((uint8_t*)jsonPayload.c_str(), jsonPayload.length());
  if (httpCode > 0) {
    Serial.printf("[HTTP] Código respuesta: %d\n", httpCode);
    String resp = http.getString();
    Serial.println("[HTTP] Respuesta cuerpo:");
    Serial.println(resp);
    http.end();
    return (httpCode >= 200 && httpCode < 300);
  } else {
    Serial.printf("[HTTP] Error POST: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nESP32 HTTP JSON demo");

  connectWiFi();
  // NTP (opcional, para timestamp)
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER);
}

void loop() {
  if (millis() - lastPost >= POST_INTERVAL_MS) {
    lastPost = millis();

    // ======= AQUÍ LEE TUS SENSORES =======
    // Reemplaza estas variables con tus lecturas reales
    float temp_c     = 23.5;   // ej. DHT22, SHT31, etc.
    float hum_pc     = 55.2;   // %
    float pm25_ugm3  = 12.3;   // µg/m³ (ej. PMS7003/SDS011)
    float pm10_ugm3  = 20.1;   // µg/m³
    // =====================================

    String ts = iso8601UTC();

    // Construimos JSON sencillo sin librerías extra
    // Nota: asegúrate de usar punto decimal. String(value, 1/2/3) controla decimales.
    String payload = "{";
    payload += "\"device_id\":\"" + String(DEVICE_ID) + "\",";
    payload += "\"temp_c\":"     + String(temp_c, 2)    + ",";
    payload += "\"hum_pc\":"     + String(hum_pc, 2)    + ",";
    payload += "\"pm25_ugm3\":"  + String(pm25_ugm3, 2) + ",";
    payload += "\"pm10_ugm3\":"  + String(pm10_ugm3, 2) + ",";
    payload += "\"ts\":"         + (ts == "null" ? "null" : ("\"" + ts + "\""));
    payload += "}";

    bool ok = postJSON(payload);
    if (!ok) {
      Serial.println("Fallo al enviar. Reintentará en el próximo ciclo.");
    }
  }

  // Pequeña tarea de mantenimiento: si WiFi se cae, reintenta de vez en cuando
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck > 10000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi caído. Reintentando...");
      connectWiFi();
    }
  }

  delay(10); // cede CPU
}
