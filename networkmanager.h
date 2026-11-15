#pragma once
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"

// ==========================================
// REFERENSI VARIABEL EKSTERNAL DARI FILE UTAMA (.INO)
// ==========================================
extern int target_compartment;
extern bool boxTakenStatus[7];

// ==========================================
// KONFIGURASI JARINGAN & ENDPOINT
// ==========================================
const char* WIFI_SSID     = "wg14_b3_2g";
const char* WIFI_PASSWORD = "wg14n153mgg";

const char* NTP_SERVER          = "pool.ntp.org";
const long  GMT_OFFSET_SEC      = 7 * 3600; // UTC+7 (WIB)
const int   DAYLIGHT_OFFSET_SEC = 0;

// URL Webhook n8n Production
const char* N8N_WEBHOOK_URL = "https://scrutable-georgianne-squirtingly.ngrok-free.dev/webhook/b879c21e-8a13-4fcc-bef3-b78ce4763054";

// URL Google Apps Script Deployment
const char* APPS_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbw0YXlKOukArbcOq414OUrXVLcJAFs32o5rOgEpC1A03Iyj9vqt_IfEcPTRAJhxJBmO/exec";
const char* ID_PASIEN       = "PASIEN-01";

// --- VARIABEL JADWAL ---
int jadwal_jam    = 11;
int jadwal_menit  = 13;
bool sudah_minum_jadwal_ini = false;
int last_checked_day = -1;
// --- STRUKTUR DATA ANTREAN CORE ---
struct WebhookMessage {
  char patientId[16];
  char eventType[32];
  int boxId;
  float deltaWeight;
  char statusMsg[64];
  char waktuActual[25]; // Format: YYYY-MM-DD HH:MM:SS
  int timeDeviation;    // Nilai deviasi waktu dalam menit
};
QueueHandle_t webhookQueue;

// ==========================================
// TASK BACKGROUND CORE 0 (HTTPS Non-Blocking)
// ==========================================
void TaskKirimWebhookCore0(void *pvParameters) {
  WebhookMessage msg;

  for (;;) {
    if (xQueueReceive(webhookQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (WiFi.status() == WL_CONNECTED) {
        WiFiClientSecure secureClient;
        secureClient.setInsecure();

        HTTPClient http;
        http.setTimeout(4000);

        if (http.begin(secureClient, N8N_WEBHOOK_URL)) {
          http.addHeader("Content-Type", "application/json");
          
          StaticJsonDocument<300> doc;
          doc["id_pasien"]       = msg.patientId;
          doc["event"]           = msg.eventType;
          doc["box_id"]          = msg.boxId;
          doc["delta_berat"]     = msg.deltaWeight;
          doc["status"]          = msg.statusMsg;
          doc["waktu"]           = msg.waktuActual;
          doc["time_deviation"]  = msg.timeDeviation;

          String requestBody;
          serializeJson(doc, requestBody);

          // 1. CATAT WAKTU TEPAT SEBELUM REQUEST TERKIRIM
          unsigned long t_start = millis();

          // Eksekusi POST (menunggu TLS handshake, transmisi data, & respons server)
          int httpResponseCode = http.POST(requestBody);

          // 2. CATAT WAKTU SETELAH RESPON DITERIMA LENGKAP
          unsigned long t_end = millis();
          unsigned long rtt = t_end - t_start; // Waktu Round-Trip (ms)

          // 3. CETAK KE SERIAL MONITOR
          Serial.printf("[Core 0 - n8n] Event: %s | Respon: %d | RTT (Latensi): %lu ms | Ukuran: %u byte\n", 
                        msg.eventType, httpResponseCode, rtt, requestBody.length());

          http.end();
        } else {
          Serial.println(F("[Core 0 - n8n] Gagal inisialisasi SSL"));
        }
      } else {
        Serial.println(F("[Core 0 - n8n] Gagal: WiFi Terputus"));
      }
    }
  }
}
// ==========================================
// FUNGSI PENGIRIM WEBHOOK
// ==========================================
inline void sendWebhookN8N(String eventType, int boxId, float deltaWeight, String statusMsg) {
  WebhookMessage msg;
  
  strncpy(msg.patientId, ID_PASIEN, sizeof(msg.patientId) - 1);
  msg.patientId[sizeof(msg.patientId) - 1] = '\0';

  strncpy(msg.eventType, eventType.c_str(), sizeof(msg.eventType) - 1);
  msg.eventType[sizeof(msg.eventType) - 1] = '\0';

  msg.boxId = boxId;
  msg.deltaWeight = deltaWeight;

  strncpy(msg.statusMsg, statusMsg.c_str(), sizeof(msg.statusMsg) - 1);
  msg.statusMsg[sizeof(msg.statusMsg) - 1] = '\0';

  // 1. Dapatkan deviasi menit dihitung lokal dari RTC/NTP ESP32
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(msg.waktuActual, sizeof(msg.waktuActual), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    int currentTotalMin = (timeinfo.tm_hour * 60) + timeinfo.tm_min;
    int scheduleTotalMin = (jadwal_jam * 60) + jadwal_menit;
    msg.timeDeviation = currentTotalMin - scheduleTotalMin;
  } else {
    strncpy(msg.waktuActual, "", sizeof(msg.waktuActual));
    msg.timeDeviation = 0;
  }

  // 2. Masukkan ke Antrean FreeRTOS
  if (xQueueSend(webhookQueue, &msg, 0) != pdTRUE) {
    Serial.println(F("[Queue] Antrean Webhook Penuh!"));
  }
}
// Inisialisasi WiFi, sinkronisasi jam NTP, dan pembuatan Task Core 0
inline void setupNetwork() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int wifiTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
    delay(500);
    Serial.print(".");
    wifiTimeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\n[WiFi] Terhubung!"));
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  } else {
    Serial.println(F("\n[WiFi] Gagal! Sistem berjalan offline."));
  }

  webhookQueue = xQueueCreate(10, sizeof(WebhookMessage));

  xTaskCreatePinnedToCore(
    TaskKirimWebhookCore0,
    "TaskWebhookCore0",
    8192,
    NULL,
    1,
    NULL,
    0
  );
}

// Evaluasi Jadwal Minum Obat
inline bool isTimeToTakeMedicine() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return false;

  if (timeinfo.tm_mday != last_checked_day) {
    last_checked_day = timeinfo.tm_mday;
    sudah_minum_jadwal_ini = false;
  }

  if (!sudah_minum_jadwal_ini) {
    if (timeinfo.tm_hour == jadwal_jam && timeinfo.tm_min == jadwal_menit) {
      return true;
    }
  }
  return false;
}

// Sinkronisasi status riwayat saat alat baru menyala/booting
inline bool sinkronisasiStatusAwal() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[SYNC] WiFi belum terhubung, sinkronisasi dibatalkan."));
    return false;
  }

  WiFiClientSecure secureClient;
  secureClient.setInsecure(); // Diperlukan untuk endpoint HTTPS Google Apps Script

  HTTPClient http;
  http.setTimeout(8000); // Beri waktu lebih untuk redirect Google Apps Script
  
  String fullUrl = String(APPS_SCRIPT_URL) + "?action=check_daily_status&id_pasien=" + String(ID_PASIEN);
  
  if (!http.begin(secureClient, fullUrl)) {
    Serial.println(F("[SYNC] Gagal inisialisasi koneksi HTTPS ke Apps Script"));
    return false;
  }

  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println(F("[SYNC] Respon diterima:"));
    Serial.println(payload);

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error && doc["status"] == "success") {
      JsonObject data = doc["data"];

      bool sudahMinum = data["sudah_minum"];
      // int targetKotak = data["target_kotak"]; // Tidak perlu diambil jika penentu hari adalah NTP
      int kotakDiminum = data["kotak_diminum"];

      // 1. Pulihkan status kepatuhan harian dari basis data
      sudah_minum_jadwal_ini = sudahMinum;

      // 2. Tandai kompartemen fisik jika memang sudah diambil hari ini
      if (sudahMinum && kotakDiminum >= 1 && kotakDiminum <= 7) {
        boxTakenStatus[kotakDiminum - 1] = true;
      }

      Serial.printf("[SYNC] Berhasil | Status Minum Hari Ini: %s | Kotak Terambil: %d\n",
                    sudah_minum_jadwal_ini ? "SUDAH" : "BELUM", kotakDiminum);
                    
      http.end();
      return true;
    } else {
      Serial.println(F("[SYNC] Format JSON tidak sesuai atau status gagal"));
    }
  } else {
    Serial.printf("[SYNC] Gagal request, kode HTTP: %d\n", httpCode);
  }

  http.end();
  return false;
}
