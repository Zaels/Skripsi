#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DFRobotDFPlayerMini.h"
#include "HX711.h"

// Memanggil modul koneksi background Core 0
#include "NetworkManager.h"

// ==========================================
// 1. PIN PERANGKAT KERAS
// ==========================================
#define PIN_DF_RX          16
#define PIN_DF_TX          17
#define PIN_BUTTON         4
#define PIN_HX711_DOUT     19
#define PIN_HX711_SCK      18

// 7 Reed Switch
const int REED_PINS[7] = {32, 33, 25, 13, 14, 27, 26};

#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      64
#define OLED_RESET         -1
#define OLED_ADDR          0x3C

// ==========================================
// 2. OBJEK & VARIABEL GLOBAL
// ==========================================
HardwareSerial dfSerial(2);
DFRobotDFPlayerMini myDFPlayer;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HX711 scale;

// Kategori evaluasi dosis fisik
enum StatusDosisFisik {
  DOSIS_BELUM_DIAMBIL,
  DOSIS_KURANG,
  DOSIS_PAS,
  DOSIS_LEBIH
};
bool dosis_setengah_tercatat = false;

// Variabel penampung hasil pembacaan sensor
struct SensorDoorResult {
  int  pintu_terbuka = 0;              // 0 jika tertutup, 1-7 nomor kotak
  bool pintu_baru_buka = false;
  bool pintu_baru_tutup = false;
  float delta_berat = 0.0;
  StatusDosisFisik status_dosis = DOSIS_BELUM_DIAMBIL;
};

SensorDoorResult dataSensor;

// Variabel kontrol jendela pengembalian obat
bool sedang_menunggu_pengembalian = false;
unsigned long timer_mulai_pengembalian = 0;
int  kotak_pengembalian = 0;
bool alasan_pengembalian_salah_kotak = false;

const float BEBAN_ACUAN = 20; // Beban standar 20 gram   // 0: Tare, 1: Tunggu Beban, 2: Hitung Faktor
float calibration_factor = -1089.85;


// --- PARAMETER FILTER EMA ---
float ema_alpha = 0.2;
float prev_ema = 0.0;

// --- BUFFER STANDAR DEVIASI ---
const int STD_BUF_SIZE = 10;
float std_buffer[STD_BUF_SIZE];
int std_idx = 0;

// Variabel Penampung Sensor Real-Time
float current_raw = 0.0;
float current_ema = 0.0;
float current_std_dev = 0.0;
float perubahan_berat = 0.0;

// Variabel Penahan Baseline Cepat
float baseline_weight = 0.0;
bool baseline_terkunci = false;

float akumulasi_delta_dosis = 0.0;

// --- MEMORI & STATUS 7 KOMPARTEMEN ---
bool boxTakenStatus[7] = {false, false, false, false, false, false, false};
int activeOpenedCount = 0;
int lastOpenedBox = 0;
bool multipleDoorViolation = false;

// Parameter Medis Obat TBC
int target_compartment = 1;        // Jadwal laci aktif default
float baseline_dose_weight = 0.5;  // Target 1 dosis
float tolerance = 0.1;             // Toleransi deviasi relatif

// Pewaktu Non-blocking (millis)
unsigned long timerSensor = 0;
unsigned long timerAudioPrompt = 0;
unsigned long timerIdleDisplay = 0;


// Variabel pendukung penstabilan pintu ditutup
bool proses_stabilisasi_tutup = false;
unsigned long timer_stabil_tutup = 0;
unsigned long waktu_mulai_tutup = 0; // <-- Menghitung total durasi stabilisasi (t)

bool webhook_alarm_terkirim = false;
bool webhook_terlewat_terkirim = false;

// Menandai apakah selama siklus buka-tutup terjadi pembukaan multi-pintu
bool pernah_multi_pintu_siklus_ini = false;

// Penanda audio interaktif prioritas tinggi (agar tidak tertimpa alarm 5 detik)
bool audio_prioritas_aktif = false;
unsigned long timer_selesai_audio = 0;


bool pengembalian_berhasil_diselesaikan = false;

// Prototype Fungsi
float filterEMA(float input);
float hitungStdDev(float input_ema);
void updateSensorStream();
int scanAllCompartments();
void updateDisplay(String line2_status_minum, String line3_aksi, String line4_alat);
void playAudio(int track);
void handleDoorAndSensor();
void handleClinicalValidation();
void checkMidnightReset();
void handleCalibrationFlow();
int getTargetBoxFromDay();
void jalankanKalibrasiBlokir();
void cetakDebugBobot();
// ==========================================
// 3. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  for (int i = 0; i < 7; i++) {
    pinMode(REED_PINS[i], INPUT_PULLUP);
  }

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[Error] OLED Gagal!"));
  }

  dfSerial.begin(9600, SERIAL_8N1, PIN_DF_RX, PIN_DF_TX);
  if (myDFPlayer.begin(dfSerial)) {
    myDFPlayer.volume(25);
  }

  scale.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
  scale.set_scale(calibration_factor);
  scale.tare();

  for (int i = 0; i < STD_BUF_SIZE; i++) std_buffer[i] = 0.0;

  // 1. Hubungkan WiFi & NTP
  updateDisplay("Inisialisasi", "Koneksi WiFi...", "Menghubungkan");
  setupNetwork();




  // 2. Tarik Data Terkini dari Spreadsheet
  updateDisplay("Inisialisasi", "Sinkronisasi...", "Mengambil Data");
  sinkronisasiStatusAwal();

  target_compartment = getTargetBoxFromDay();
  Serial.printf("[SETUP] Sinkronisasi Hari NTP -> Hari Ini: Kotak %d\n", target_compartment);
  // 3. Masuk Kondisi Siaga
  String statusMinum = sudah_minum_jadwal_ini ? "Sudah Minum Obat" : "Belum Minum Obat";
  updateDisplay(statusMinum, "Sistem Siaga", "Jadwal: Kotak " + String(target_compartment));
  Serial.println(F("=== SMART PILLBOX SIAP AKTIF ==="));
}

// ==========================================
// 4. LOOP UTAMA
// ==========================================
void loop() {
  unsigned long currentMillis = millis();

  // 1. Eksekusi Pembacaan Sensor tiap 50ms (20Hz)
  if (currentMillis - timerSensor >= 50) {
    timerSensor = currentMillis;
    updateSensorStream();
  }

  checkMidnightReset();
  // 2. Eksekusi Hardware & Deteksi Sensor
  handleDoorAndSensor();

  // 3. Eksekusi Validasi Aturan Klinis & Webhook
  handleClinicalValidation();
  // 4. Polling DFPlayer
  if (myDFPlayer.available()) {
    myDFPlayer.read();
  }

  if (digitalRead(PIN_BUTTON) == LOW) {
    delay(50); // Debounce
    if (digitalRead(PIN_BUTTON) == LOW) {
      jalankanKalibrasiBlokir(); // Jalankan kalibrasi berbasis while tanpa flicker
    }
  }
  cetakDebugBobot();
}

// ==========================================
// 5. FUNGSI SENSOR & HARDWARE PINTU
// ==========================================
void handleDoorAndSensor() {
  unsigned long currentMillis = millis();
  int currentOpened = scanAllCompartments();
  static bool prevDoorOpen = false;

  dataSensor.pintu_baru_buka = false;
  dataSensor.pintu_baru_tutup = false;

  String statusMinum = sudah_minum_jadwal_ini ? "Sudah Minum Obat" : "Belum Minum Obat";

  // 1. Deteksi Transisi Pintu Dibuka
  if (!prevDoorOpen && currentOpened > 0) {
    prevDoorOpen = true;
    proses_stabilisasi_tutup = false;
    dataSensor.pintu_baru_buka = true;
    dataSensor.pintu_terbuka = currentOpened;
    lastOpenedBox = currentOpened;
    pernah_multi_pintu_siklus_ini = multipleDoorViolation; // Catat kondisi awal
    Serial.printf("[SENSOR] Pintu DIBUKA | Primary: %d | Multi: %d | Baseline: %.2f g\n", 
                  currentOpened, multipleDoorViolation, baseline_weight);
  }
  // 2. Deteksi Pintu Masih Terbuka
  else if (prevDoorOpen && currentOpened > 0) {
    dataSensor.pintu_terbuka = currentOpened;
    lastOpenedBox = currentOpened;
    if (multipleDoorViolation) {
      pernah_multi_pintu_siklus_ini = true; // Kunci flag jika sempat buka >1 pintu
    }
  }
  // 3. Deteksi Transisi Sesaat Pintu Ditutup (Seluruh laci telah tertutup)
  else if (prevDoorOpen && currentOpened == 0) {
    prevDoorOpen = false;
    proses_stabilisasi_tutup = true;
    timer_stabil_tutup = currentMillis;
    waktu_mulai_tutup = currentMillis; // Catat titik awal penutupan untuk menghitung durasi t
    if (!sedang_menunggu_pengembalian) {
      updateDisplay(statusMinum, "Menutup Kotak", "Menstabilkan...");
    }
    Serial.println(F("[SENSOR] Seluruh pintu tertutup, menstabilkan pembacaan beban..."));
  }
  // 4. Proses Stabilisasi dan Evaluasi Dosis Setelah Seluruh Pintu Tertutup
  else if (proses_stabilisasi_tutup) {
    dataSensor.pintu_terbuka = 0;

    if (current_std_dev >= 0.02) {
      timer_stabil_tutup = currentMillis;
      if (!sedang_menunggu_pengembalian && !pengembalian_berhasil_diselesaikan) {
        updateDisplay(statusMinum, "Menutup Kotak", "Menstabilkan...");
      }
    } 
    else if (currentMillis - timer_stabil_tutup >= 1000) {
      proses_stabilisasi_tutup = false;
      dataSensor.pintu_baru_tutup = true; // Memicu evaluasi klinis

      float waktu_stabilisasi_detik = (float)(currentMillis - waktu_mulai_tutup) / 1000.0;

      // JIKA INI PENUTUPAN PINTU DARI PENGEMBALIAN OBAT YANG SUDAH VALID:
      if (pengembalian_berhasil_diselesaikan) {
        dataSensor.status_dosis = DOSIS_PAS;
        dataSensor.delta_berat = baseline_dose_weight; // Kunci pada nilai target dosis sah
        Serial.printf("[SENSOR] Penimbangan STABIL (Pasca Pengembalian Sah) | Status: DOSIS PAS | Waktu: %.2f s\n", 
                      waktu_stabilisasi_detik);
        return;
      }

      // EVALUASI NORMAL (Jika bukan pasca pengembalian obat)
      float total_massa_terangkat = akumulasi_delta_dosis + perubahan_berat;

      const char* labelDosis = "";
      if (total_massa_terangkat < 0.20) {
        dataSensor.status_dosis = DOSIS_BELUM_DIAMBIL;
        labelDosis = "BELUM DIAMBIL";
      } else if (total_massa_terangkat < (baseline_dose_weight - tolerance)) {
        dataSensor.status_dosis = DOSIS_KURANG;
        labelDosis = "DOSIS KURANG";
      } else if (total_massa_terangkat > (baseline_dose_weight + tolerance + 0.005)) {
        dataSensor.status_dosis = DOSIS_LEBIH;
        labelDosis = "DOSIS LEBIH";
      } else {
        dataSensor.status_dosis = DOSIS_PAS;
        labelDosis = "DOSIS PAS";
      }

      dataSensor.delta_berat = perubahan_berat;

      Serial.printf("[SENSOR] Penimbangan STABIL | Delta Sesaat: %.2f g | Akumulasi Total: %.2f g | Waktu: %.2f s | Status: %s | MultiDoor: %d\n", 
                    dataSensor.delta_berat, total_massa_terangkat, waktu_stabilisasi_detik, labelDosis, pernah_multi_pintu_siklus_ini);
    }
  }
  // 5. Kondisi Siaga
  else {
    dataSensor.pintu_terbuka = 0;
    if (current_std_dev < 0.02 && !sedang_menunggu_pengembalian && !pengembalian_berhasil_diselesaikan) {
      baseline_weight = current_ema;
      baseline_terkunci = true;
      perubahan_berat = 0.0;
    }
  }
}
// ==========================================
// 6. VALIDASI KLINIS & JADWAL
// ==========================================
// Fungsi untuk menghitung selisih menit antara waktu saat ini dengan jam jadwal
// Return: negatif jika lebih awal, positif jika terlambat
inline int getScheduleDeviationMinutes() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return 0;

  int currentTotalMin = (timeinfo.tm_hour * 60) + timeinfo.tm_min;
  int scheduleTotalMin = (jadwal_jam * 60) + jadwal_menit;
  
  return currentTotalMin - scheduleTotalMin;
}

// ==========================================
// 6. VALIDASI KLINIS & JADWAL
// ==========================================
void handleClinicalValidation() {
  unsigned long currentMillis = millis();
  int deviasiMenit = getScheduleDeviationMinutes();

  // Batas toleransi keterlambatan maksimal 12 jam (720 menit)
  bool isWithin12Hours = (deviasiMenit <= 720);
  bool isPastSchedule = (deviasiMenit >= 0);

  String statusMinum = sudah_minum_jadwal_ini ? "Sudah Minum Obat" : "Belum Minum Obat";

 // ====================================================================
  // A. PENGINGAT ALARM JADWAL & VONIS KETERLAMBATAN (> 12 JAM)
  // ====================================================================
  if (dataSensor.pintu_terbuka == 0 && !sedang_menunggu_pengembalian && !proses_stabilisasi_tutup) {
    
    // ------------------------------------------------------------------
    // KONDISI 1: MASIH DALAM RENTANG 12 JAM (ALARM PENGINGAT)
    // ------------------------------------------------------------------
    if (!sudah_minum_jadwal_ini && isPastSchedule && isWithin12Hours) {
      if (dosis_setengah_tercatat) {
        // Pasien sudah ambil sebagian tapi belum lengkap
        updateDisplay("OBAT SETENGAH!", "AMBIL SISANYA!", "Kotak " + String(target_compartment));
      } else {
        // Pasien belum mengambil sama sekali
        updateDisplay("Belum Minum Obat", "WAKTUNYA MINUM!", "Buka Kotak " + String(target_compartment));
      }

      // Kirim webhook pengingat sekali saat jam jadwal tiba
      if (!webhook_alarm_terkirim) {
        webhook_alarm_terkirim = true;
        sendWebhookN8N("alarm_waktunya_minum", target_compartment, 0.0, "Jadwal Minum Aktif");
        Serial.printf("[WEBHOOK] Pengingat jadwal Kotak %d terkirim ke n8n\n", target_compartment);
      }

      // Cek apakah jeda tenang pasca suara peringatan (minimal 3,5 detik) sudah lewat
      if (audio_prioritas_aktif && (currentMillis - timer_selesai_audio >= 3500)) {
        audio_prioritas_aktif = false;
        timerAudioPrompt = currentMillis; // Reset timer alarm agar tidak langsung menimpa seketika
      }

      // Alarm hanya boleh berbunyi jika sistem tenang dan tidak ada audio prioritas yang aktif
      if (!audio_prioritas_aktif && !sedang_menunggu_pengembalian && dataSensor.pintu_terbuka == 0) {
        if (currentMillis - timerAudioPrompt >= 5000) {
          timerAudioPrompt = currentMillis;
          playAudio(dosis_setengah_tercatat ? 3 : 1);
        }
      }
    } 
    // ------------------------------------------------------------------
    // KONDISI 2: MELEWATI BATAS KRITIS 12 JAM (EKSEKUSI VONIS FINAL)
    // ------------------------------------------------------------------
    else if (!sudah_minum_jadwal_ini && !isWithin12Hours) {
      if (!webhook_terlewat_terkirim) {
        webhook_terlewat_terkirim = true;
        sudah_minum_jadwal_ini = true; // Kunci siklus hari ini agar tidak bisa diminum lagi
        
        if (dosis_setengah_tercatat) {
          // Kasus: Obat diminum separuh dan sisanya tidak diambil sampai lewat 12 jam
          sendWebhookN8N("dosis_tidak_lengkap", target_compartment, dataSensor.delta_berat, "Vonis: Dosis Tidak Lengkap (>12 Jam)");
          Serial.printf("[VONIS KLINIS] Batas 12 jam habis! Obat Kotak %d divonis DOSIS TIDAK LENGKAP.\n", target_compartment);
        } else {
          // Kasus: Obat sama sekali tidak disentuh
          sendWebhookN8N("dosis_terlewat", target_compartment, 0.0, "Vonis: Dosis Terlewat Penuh (>12 Jam)");
          Serial.printf("[VONIS KLINIS] Batas 12 jam habis! Dosis Kotak %d divonis TERLEWAT.\n", target_compartment);
        }
        
        playAudio(10); // Bunyikan nada peringatan pelanggaran/vonis gagal
      }

      if (currentMillis - timerIdleDisplay >= 1000) {
        timerIdleDisplay = currentMillis;
        if (dosis_setengah_tercatat) {
          updateDisplay("Dosis Tdk Lengkap", "Lewat Batas 12 Jam", "Tunggu Jadwal Besok");
        } else {
          updateDisplay("Dosis Terlewat", "Lewat Batas 12 Jam", "Tunggu Jadwal Besok");
        }
      }
    } 
    // ------------------------------------------------------------------
    // KONDISI 3: SIAGA NORMAL (Sebelum jadwal atau sudah tuntas diminum)
    // ------------------------------------------------------------------
    else {
      if (currentMillis - timerIdleDisplay >= 1000) {
        timerIdleDisplay = currentMillis;
        updateDisplay(statusMinum, "Sistem Siaga", "Jadwal: Kotak " + String(target_compartment));
      }
    }
  }
  // ====================================================================
  // B. PERINGATAN DINI SAAT PINTU BARU DIBUKA
  // ====================================================================
  if (dataSensor.pintu_baru_buka) {
    // 0. JIKA SEDANG DALAM MASA TOLERANSI PENGEMBALIAN OBAT (GRACE PERIOD)
    if (sedang_menunggu_pengembalian) {
      if (dataSensor.pintu_terbuka == kotak_pengembalian) {
        updateDisplay(statusMinum, "KEMBALIKAN OBAT", "Kotak " + String(kotak_pengembalian) + " Terbuka");
        Serial.printf("[GRACE PERIOD] Laci koreksi (Kotak %d) dibuka kembali.\n", kotak_pengembalian);
      } else {
        updateDisplay(statusMinum, "SALAH KOTAK!", "Kembalikan ke Kotak " + String(kotak_pengembalian));
        playAudio(5);
      }
    }
    // 1. Deteksi pelanggaran membuka >1 laci bersamaan
    else if (multipleDoorViolation) {
      updateDisplay(statusMinum, "PERINGATAN!", "JANGAN BUKA >1 KOTAK");
      playAudio(6);
      sendWebhookN8N("pelanggaran_pintu", dataSensor.pintu_terbuka, 0.0, "Buka >1 Kotak");
    } 
    // 2. PRIORITAS UTAMA: Pasien SUDAH MINUM OBAT hari ini (Buka laci mana saja!)
    else if (sudah_minum_jadwal_ini) {
      updateDisplay("Sudah Minum Obat", "PERINGATAN DINI!", "Anda Baru Saja Minum");
      audio_prioritas_aktif = true;
      timer_selesai_audio = currentMillis;
      playAudio(9); // Track 9: Audio "Anda sudah minum obat hari ini"
      sendWebhookN8N("akses_setelah_minum", dataSensor.pintu_terbuka, 0.0, "Buka Kotak Pasca Minum");
    } 
    // 3. Pasien belum minum, tetapi jadwal sudah lewat batas kritis 12 jam
    else if (!isWithin12Hours) {
      updateDisplay("Dosis Terlewat", "PERINGATAN!", "Terlalu Dekat Besok");
      playAudio(10);
      sendWebhookN8N("buka_lewat_12jam", dataSensor.pintu_terbuka, 0.0, "Buka Laci Melewati 12 Jam");
    } 
    // 4. Pasien belum minum dan membuka laci yang keliru (bukan jadwal hari berjalan)
    else if (dataSensor.pintu_terbuka != target_compartment) {
      updateDisplay(statusMinum, "SALAH KOTAK!", "Target: Kotak " + String(target_compartment));
      playAudio(5); // Track 5: Audio "Salah laci / kotak"
      sendWebhookN8N("salah_buka_laci", dataSensor.pintu_terbuka, 0.0, "Buka Kotak " + String(dataSensor.pintu_terbuka));
    } 
    // 5. Pembukaan laci sah sesuai jadwal hari berjalan
    else {
      sendWebhookN8N("pintu_dibuka", dataSensor.pintu_terbuka, 0.0, "Kotak Sesuai Dibuka");
    }
  }

  // ====================================================================
  // C. UMPAN BALIK REAL-TIME SAAT PINTU TERBUKA
  // ====================================================================
  if (dataSensor.pintu_terbuka > 0 && !sedang_menunggu_pengembalian) {
    // Peringatan visual jika terdeteksi multi-pintu terbuka bersamaan
    if (multipleDoorViolation) {
      updateDisplay(statusMinum, "PERINGATAN!", "2 KOTAK TERBUKA!");
      if (currentMillis - timerAudioPrompt >= 3000) {
        timerAudioPrompt = currentMillis;
        playAudio(6);
      }
    } 
    else if (current_std_dev < 0.02) {
      if (dataSensor.pintu_terbuka != target_compartment || sudah_minum_jadwal_ini || !isWithin12Hours) {
        if (perubahan_berat >= 0.2) {
          updateDisplay(statusMinum, "JANGAN AMBIL!", "Bahaya Luar Jadwal");
          if (currentMillis - timerAudioPrompt >= 3000) {
            timerAudioPrompt = currentMillis;
            playAudio(8);
          }
        } else {
          updateDisplay(statusMinum, "Kotak " + String(dataSensor.pintu_terbuka) + " Terbuka", "Jangan Ambil Obat!");
        }
      } 
    else {
        // Hitung akumulasi jika sebelumnya sudah pernah terambil sebagian
        float total_realtime = akumulasi_delta_dosis + perubahan_berat;

        if (total_realtime < 0.2) {
          updateDisplay("Belum Minum Obat", "Kotak " + String(dataSensor.pintu_terbuka) + " Terbuka", "Silakan Ambil Obat");
        } else if (total_realtime < (baseline_dose_weight - tolerance)) {
          updateDisplay("Belum Minum Obat", "OBAT KURANG!", "Total: " + String(total_realtime, 2) + "g");
          if (currentMillis - timerAudioPrompt >= 4000) {
            timerAudioPrompt = currentMillis;
            playAudio(3);
          }
        } else if (total_realtime > (baseline_dose_weight + tolerance)) {
          updateDisplay("Belum Minum Obat", "TERLALU BANYAK!", "Balikan Sebagian");
          if (currentMillis - timerAudioPrompt >= 3000) {
            timerAudioPrompt = currentMillis;
            playAudio(4);
          }
        } else {
          updateDisplay("Belum Minum Obat", "DOSIS TEPAT!", "Total: " + String(total_realtime, 2) + "g");
          if (currentMillis - timerAudioPrompt >= 4000) {
            timerAudioPrompt = currentMillis;
            playAudio(2);
          }
        }
      }
    }
  }
// ====================================================================
  // D. EVALUASI AKHIR SAAT PINTU DITUTUP (SETELAH STABIL)
  // ====================================================================
  // ====================================================================
  // D. EVALUASI AKHIR SAAT PINTU DITUTUP (SETELAH STABIL)
  // ====================================================================
  if (dataSensor.pintu_baru_tutup) {
    int kotak = lastOpenedBox;

    // 0. JIKA PENUTUPAN PINTU MERUPAKAN AKHIR DARI PENGEMBALIAN OBAT YANG SAH
    if (pengembalian_berhasil_diselesaikan) {
      pengembalian_berhasil_diselesaikan = false;
      updateDisplay("Sudah Minum Obat", "STATUS: SUKSES", "Kotak " + String(kotak) + " Ditutup");
      Serial.println(F("[KLINIS] Pintu ditutup pasca pengembalian obat berhasil. Sesi tuntas."));
    }

    // ==================================================================
    // PRIORITAS 1: PASIEN SUDAH MINUM OBAT HARI INI
    // (Buka 1 laci maupun >1 laci dan ambil obat = BAHAYA OVERDOSIS)
    // ==================================================================
    else if (sudah_minum_jadwal_ini) {
      pernah_multi_pintu_siklus_ini = false; // Reset penanda

      if (dataSensor.status_dosis != DOSIS_BELUM_DIAMBIL) {
        sedang_menunggu_pengembalian = true;
        alasan_pengembalian_salah_kotak = false; // Memicu vonis overdosis
        timer_mulai_pengembalian = currentMillis;
        kotak_pengembalian = kotak;
        playAudio(8);
      } else {
        updateDisplay("Sudah Minum Obat", "TERTUTUP", "Kotak Ditutup");
      }
    }

    // ==================================================================
    // PRIORITAS 2: PASIEN BELUM MINUM, DI LUAR JADWAL (> 12 JAM / BELUM WAKTUNYA)
    // (Buka 1 laci maupun >1 laci tetap DILARANG)
    // ==================================================================
    else if (!isWithin12Hours || !isPastSchedule) {
      int targetKotak = pernah_multi_pintu_siklus_ini ? 0 : kotak;
      bool wasMulti = pernah_multi_pintu_siklus_ini;
      pernah_multi_pintu_siklus_ini = false; // Reset penanda

      if (dataSensor.status_dosis != DOSIS_BELUM_DIAMBIL) {
        sedang_menunggu_pengembalian = true;
        alasan_pengembalian_salah_kotak = false; // Vonis bahaya jadwal
        timer_mulai_pengembalian = currentMillis;
        kotak_pengembalian = targetKotak; // Set 0 jika multi-pintu
        playAudio(8);
      } else {
        if (wasMulti) {
          updateDisplay(statusMinum, "MULTI-PINTU", "Luar Jadwal (Kosong)");
          sendWebhookN8N("multi_pintu_kosong", 0, 0.0, "Multi-Pintu Luar Jadwal (Tanpa Ambil)");
        } else {
          updateDisplay("Dosis Terlewat", "TERLEWAT", "Kotak Ditutup");
        }
      }
    }

    // ==================================================================
    // PRIORITAS 3: DALAM JADWAL SAH, TETAPI BUKA > 1 LACI (MULTI-PINTU)
    // ==================================================================
    else if (pernah_multi_pintu_siklus_ini) {
      pernah_multi_pintu_siklus_ini = false; // Reset penanda

      if (dataSensor.status_dosis == DOSIS_BELUM_DIAMBIL) {
        updateDisplay(statusMinum, "MULTI-PINTU", "Tidak Ada Obat Diambil");
        sendWebhookN8N("multi_pintu_kosong", 0, 0.0, "Multi-Pintu (Tanpa Ambil Obat)");
      } 
      else if (dataSensor.status_dosis == DOSIS_PAS) {
        sudah_minum_jadwal_ini = true; // Dosis harian pasien disahkan
        updateDisplay("Sudah Minum Obat", "DOSIS DIAMBIL", "Asal Kotak: AMBIGU");
        playAudio(2);
        sendWebhookN8N("obat_terambil_ambigu", 0, dataSensor.delta_berat, "Dosis Pas (Sumber Kotak Ambigu)");
        Serial.println(F("[KLINIS] Dosis tepat, namun sumber kotak ambigu karena buka >1 pintu!"));
      } 
      else {
        sedang_menunggu_pengembalian = true;
        alasan_pengembalian_salah_kotak = false;
        timer_mulai_pengembalian = currentMillis;
        kotak_pengembalian = 0; // Kotak ambigu
        playAudio(8); 
      }
    }

    // ==================================================================
    // PRIORITAS 4: DALAM JADWAL SAH, BUKA LACI SALAH (!= TARGET KALENDER)
    // ==================================================================
    else if (kotak != target_compartment) {
      if (dataSensor.status_dosis != DOSIS_BELUM_DIAMBIL) {
        sedang_menunggu_pengembalian = true;
        alasan_pengembalian_salah_kotak = true; // Murni salah jadwal
        timer_mulai_pengembalian = currentMillis;
        kotak_pengembalian = kotak;
        playAudio(8);
      } else {
        updateDisplay(statusMinum, "TERTUTUP", "Tidak Ada Obat Diambil");
      }
    }

    // ==================================================================
    // PRIORITAS 5: DALAM JADWAL SAH, BUKA LACI BENAR (KONDISI NORMAL)
    // ==================================================================
    else {
      float total_terangkat = akumulasi_delta_dosis + dataSensor.delta_berat;

      if (total_terangkat < 0.20) {
        updateDisplay("Belum Minum Obat", "PERINGATAN!", "Obat Belum Diambil");
        playAudio(7);
        sendWebhookN8N("obat_!diambil", kotak, total_terangkat, "Obat Tidak Diambil");
      } 
      else if (total_terangkat < (baseline_dose_weight - tolerance)) {
        dosis_setengah_tercatat = true;
        akumulasi_delta_dosis = total_terangkat;
        updateDisplay("Belum Minum Obat", "PERINGATAN!", "Dosis Kurang!");
        playAudio(3);
        sendWebhookN8N("dosis_kurang", kotak, total_terangkat, "Dosis Kurang");
      } 
      else if (total_terangkat > (baseline_dose_weight + tolerance)) {
        sedang_menunggu_pengembalian = true;
        alasan_pengembalian_salah_kotak = false;
        timer_mulai_pengembalian = currentMillis;
        timerAudioPrompt = currentMillis;
        kotak_pengembalian = kotak;
        playAudio(4);
      } 
      else {
        // Rentang dosis pas (0.40 - 0.60 g)
        sudah_minum_jadwal_ini = true;
        dosis_setengah_tercatat = false;
        akumulasi_delta_dosis = 0.0;
        boxTakenStatus[kotak - 1] = true;
        
        updateDisplay("Sudah Minum Obat", "STATUS: SUKSES", "Kotak " + String(kotak) + " Terekam");
        playAudio(2);
        sendWebhookN8N("obat_terambil", kotak, total_terangkat, "DOSIS LENGKAP");
      }
    }
  }
  /// ====================================================================
  // E. JENDELA TOLERANSI PENGEMBALIAN OBAT (15 DETIK)
  // ====================================================================
  if (sedang_menunggu_pengembalian) {
    long waktuLewat = currentMillis - timer_mulai_pengembalian;
    long sisaDetik = (20000 - waktuLewat) / 1000;
    if (sisaDetik < 0) sisaDetik = 0;

    // --- MANAJEMEN ANTRIAN AUDIO TANPA TABRAKAN ---
    if (audio_prioritas_aktif) {
      if (currentMillis - timer_selesai_audio >= 4500) {
        audio_prioritas_aktif = false;
        timerAudioPrompt = currentMillis;
      }
    } 
    else {
      if (currentMillis - timerAudioPrompt >= 4000) {
        timerAudioPrompt = currentMillis;
        myDFPlayer.playMp3Folder(8);
      }
    }

    static long lastSisaDetik = -1;
    static int lastPintuStatus = -1;
    int statusPintuSekarang = (dataSensor.pintu_terbuka == 0) ? 0 : 1;

    // Hitung total massa terkini termasuk akumulasi sebelumnya
    float total_bobot_saat_ini = akumulasi_delta_dosis + perubahan_berat;

    if (sisaDetik != lastSisaDetik || statusPintuSekarang != lastPintuStatus) {
      lastSisaDetik = sisaDetik;
      lastPintuStatus = statusPintuSekarang;

      String infoPintu = (dataSensor.pintu_terbuka == 0) ? "Tutup" : String(total_bobot_saat_ini, 1) + "g";
      updateDisplay(statusMinum, "KEMBALIKAN OBAT!", "Sisa: " + String(sisaDetik) + "s | " + infoPintu);
    }

    // KASUS A: Pengembalian akibat salah kotak, sudah minum sebelumnya, atau lewat 12 jam (Obat dikembalikan utuh)
    if (alasan_pengembalian_salah_kotak || sudah_minum_jadwal_ini || !isWithin12Hours) {
      if (current_std_dev < 0.02 && perubahan_berat < 0.20) {
        updateDisplay(statusMinum, "TERIMA KASIH", "Obat Dikembalikan!");
        playAudio(2);
        sedang_menunggu_pengembalian = false;
        pengembalian_berhasil_diselesaikan = true;
        return;
      }
    } 
    // KASUS B: Kelebihan tablet pada jadwal sah dikurangi kembali hingga masuk rentang dosis pas
    else {
      float total_koreksi = akumulasi_delta_dosis + perubahan_berat;
      
      if (current_std_dev < 0.02 && 
          total_koreksi >= (baseline_dose_weight - tolerance) && 
          total_koreksi <= (baseline_dose_weight + tolerance + 0.005)) {
        
        updateDisplay("Sudah Minum Obat", "TERIMA KASIH", "Dosis Sesuai!");
        playAudio(2);
        sudah_minum_jadwal_ini = true;
        dosis_setengah_tercatat = false;
        akumulasi_delta_dosis = 0.0; // Reset akumulasi
        boxTakenStatus[kotak_pengembalian - 1] = true;
        
        sendWebhookN8N("obat_terambil", kotak_pengembalian, total_koreksi, "DOSIS LENGKAP");
        sedang_menunggu_pengembalian = false;
        pengembalian_berhasil_diselesaikan = true;
        return;
      }
    }

    // VONIS AKHIR (Batas 15 Detik Berakhir dan Obat Tidak Dikembalikan)
    if (waktuLewat >= 20000) { // Diselaraskan ke 15 detik sesuai skripsi
      sedang_menunggu_pengembalian = false;

      float total_final_terangkat = akumulasi_delta_dosis + perubahan_berat;

      // 1. KASUS OVERDOSIS PASCA MINUM (Sudah minum hari ini, tapi ambil obat lagi)
      if (sudah_minum_jadwal_ini) {
        boxTakenStatus[kotak_pengembalian - 1] = true;
        dosis_setengah_tercatat = false;
        akumulasi_delta_dosis = 0.0;

        playAudio(4); // Audio Track 4: Peringatan bahaya dosis berlebih / overdosis!
        updateDisplay("Sudah Minum Obat", "VONIS OVERDOSIS!", "Ambil Pasca Minum");
        sendWebhookN8N("overdosis", kotak_pengembalian, total_final_terangkat, "Overdosis (Ambil Pasca Minum)");
        Serial.printf("[KLINIS] Vonis: OVERDOSIS! Pasien mengambil obat pasca minum dari Kotak %d\n", kotak_pengembalian);
      }
      // 2. KASUS MELEWATI BATAS 12 JAM (Terlalu dekat dengan jadwal besok)
      else if (!isWithin12Hours) {
        sudah_minum_jadwal_ini = true;
        boxTakenStatus[kotak_pengembalian - 1] = true;
        dosis_setengah_tercatat = false;
        akumulasi_delta_dosis = 0.0;

        playAudio(10); // Audio Track 10: Dosis terlewat batas 12 jam
        updateDisplay("Dosis Terlewat", "BAHAYA DOSIS!", "Minum Lewat 12 Jam");
        sendWebhookN8N("overdosis", kotak_pengembalian, total_final_terangkat, "Overdosis (Mepet Dosis Berikutnya)");
      }
      // 3. KASUS SALAH KOTAK SEBELUM MINUM (Kotak keliru diminum)
      else if (alasan_pengembalian_salah_kotak) {
        sudah_minum_jadwal_ini = true;
        boxTakenStatus[kotak_pengembalian - 1] = true;
        dosis_setengah_tercatat = false;
        akumulasi_delta_dosis = 0.0;
        
        playAudio(5); // Audio Track 5: Peringatan salah jadwal / salah kotak
        updateDisplay("Sudah Minum Obat", "SALAH JADWAL!", "Dosis Diminum");
        sendWebhookN8N("salah_jadwal_box", kotak_pengembalian, total_final_terangkat, "Salah Kotak (Dosis Diminum)");
      }
      // 4. KASUS OVERDOSIS PADA JADWAL SAH (Ambil > 1 tablet pada kotak target yang benar)
      else {
        sudah_minum_jadwal_ini = true;
        boxTakenStatus[kotak_pengembalian - 1] = true;
        dosis_setengah_tercatat = false;
        akumulasi_delta_dosis = 0.0;

        playAudio(4); // Audio Track 4: Peringatan bahaya dosis berlebih / overdosis!
        updateDisplay("Sudah Minum Obat", "DOSIS BERLEBIH!", "Melebihi Takaran");
        sendWebhookN8N("overdosis", kotak_pengembalian, total_final_terangkat, "Overdosis (Kelebihan Bobot)");
      }
    }
  }
}
// ==========================================
// 7. PIPELINE SENSOR & FUNGSI MATEMATIKA
// ==========================================
void updateSensorStream() {
  if (scale.is_ready()) {
    current_raw = scale.get_units(1);
    current_ema = filterEMA(current_raw);
    current_std_dev = hitungStdDev(current_ema);

    if (current_std_dev < 0.02 && baseline_terkunci) {
      float d = baseline_weight - current_ema;
      perubahan_berat = d;
    }
  }
}

float filterEMA(float input) {
  float result = (ema_alpha * input) + ((1.0 - ema_alpha) * prev_ema);
  prev_ema = result;
  return result;
}

float hitungStdDev(float input_ema) {
  std_buffer[std_idx] = input_ema;
  std_idx++;
  if (std_idx >= STD_BUF_SIZE) {
    std_idx = 0;
  }

  float sum = 0.0;
  for (int i = 0; i < STD_BUF_SIZE; i++) {
    sum += std_buffer[i];
  }
  float mean = sum / STD_BUF_SIZE;

  float sq_diff_sum = 0.0;
  for (int i = 0; i < STD_BUF_SIZE; i++) {
    float diff = std_buffer[i] - mean;
    sq_diff_sum += (diff * diff);
  }
  float variance = sq_diff_sum / STD_BUF_SIZE;
  return sqrt(variance);
}

int scanAllCompartments() {
  activeOpenedCount = 0;
  int primaryBox = 0;

  for (int i = 0; i < 7; i++) {
    if (digitalRead(REED_PINS[i]) == HIGH) {
      activeOpenedCount++;
      primaryBox = i + 1;
    }
  }

  multipleDoorViolation = (activeOpenedCount > 1);
  return primaryBox;
}

// Fungsi bantu mendapatkan string jam saat ini (HH:MM:SS)
String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "--:--:--";
  }
  char timeStr[12];
  sprintf(timeStr, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return String(timeStr);
}



// Fungsi display 4 baris anti-flicker
void updateDisplay(String line2_status_minum, String line3_aksi, String line4_alat) {
  static String lastLine1 = "";
  static String lastLine2 = "";
  static String lastLine3 = "";
  static String lastLine4 = "";

  String line1_jam = "JAM: " + getFormattedTime();

  // Hanya refresh jika ada perubahan teks (mencegah flickering)
  if (line1_jam == lastLine1 && 
      line2_status_minum == lastLine2 && 
      line3_aksi == lastLine3 && 
      line4_alat == lastLine4) {
    return;
  }

  lastLine1 = line1_jam;
  lastLine2 = line2_status_minum;
  lastLine3 = line3_aksi;
  lastLine4 = line4_alat;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // BARIS 1: Jam Real-Time + Garis Pemisah (Y: 0 s/d 11)
  display.setCursor(0, 0);
  display.print(line1_jam);
  display.drawLine(0, 11, 127, 11, SSD1306_WHITE);

  // BARIS 2: Status Minum Obat (Y: 16)
  display.setCursor(0, 16);
  display.print(line2_status_minum);

  // BARIS 3: Aksi / Event Kotak (Y: 32)
  display.setCursor(0, 32);
  display.print(line3_aksi);

  // BARIS 4: Status Alat / Beban (Y: 48)
  display.setCursor(0, 48);
  display.print(line4_alat);

  display.display();
}

void checkMidnightReset() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;

  static int lastDay = -1;

  if (lastDay == -1) {
    lastDay = timeinfo.tm_mday;
    return;
  }

  // Jika tanggal berubah (lewat jam 00:00 tengah malam)
  if (timeinfo.tm_mday != lastDay) {
    lastDay = timeinfo.tm_mday;
    
    sudah_minum_jadwal_ini = false;
    webhook_alarm_terkirim = false;
    webhook_terlewat_terkirim = false;
    dosis_setengah_tercatat = false;
    akumulasi_delta_dosis = 0.0;

    // Perbarui kompartemen langsung mengikuti hari kalender yang baru
    target_compartment = getTargetBoxFromDay();

    Serial.printf("[RESET MIDNIGHT] Hari berganti! Target aktif: Kotak %d\n", target_compartment);
  }
}



void jalankanKalibrasiBlokir() {
  Serial.println(F("\n=== MASUK MODE KALIBRASI TERPANDU ==="));

  // -------------------------------------------------------------
  // TAHAP 1: KOSONGKAN BEBAN & TARE
  // -------------------------------------------------------------
  updateDisplay("MODE KALIBRASI", "1. KOSONGKAN BEBAN", "Proses Tare...");
  playAudio(1);
  
  scale.set_scale(); // Reset skala ke 1.0
  scale.tare();      // Nol-kan ADC
  delay(2000);       // Tahan sejenak agar tara stabil

  // -------------------------------------------------------------
  // TAHAP 2: MENARUH BEBAN 20 GRAM (HITUNG FAKTOR SKALA)
  // -------------------------------------------------------------
  updateDisplay("MODE KALIBRASI", "2. TARUH BEBAN 20g", "Tunggu 5 Detik...");
  playAudio(1);
  
  // Hitung mundur 5 detik tanpa flicker
  unsigned long startWait = millis();
  while (millis() - startWait < 5000) {
    int sisa = 5 - ((millis() - startWait) / 1000);
    static int lastSisa = -1;
    if (sisa != lastSisa) {
      lastSisa = sisa;
      updateDisplay("MODE KALIBRASI", "2. TARUH BEBAN 20g", "Menimbang: " + String(sisa) + "s");
    }
    delay(50);
  }

  // Ambil nilai ADC mentah dan tentukan faktor skala baru
  long raw_adc = scale.get_value(10);
  calibration_factor = (float)raw_adc / BEBAN_ACUAN; // BEBAN_ACUAN = 20.0[cite: 1, 3]
  scale.set_scale(calibration_factor);
  
  Serial.printf("[KALIBRASI] Raw ADC: %ld | Skala Baru: %.2f\n", raw_adc, calibration_factor);
  playAudio(2); // Nada konfirmasi tahap 2 berhasil

  // -------------------------------------------------------------
  // TAHAP 3: ANGKAT BEBAN 20g, MASUKKAN SEMUA OBAT, TUNGGU TOMBOL
  // -------------------------------------------------------------
  updateDisplay("ISI SEMUA OBAT", "Angkat Beban 20g", "Tekan Tombol Bila Selesai");

  // Debounce tombol: pastikan tombol sudah dilepas dulu dari penekanan sebelumnya
  while (digitalRead(PIN_BUTTON) == LOW) {
    delay(20);
  }
  delay(300);

  // LOOP WHILE: Menunggu pengguna selesai menata obat dan menekan tombol
  while (true) {
    // Tetap update stream sensor agar nilai berat di background terhitung
    updateSensorStream();

    if (digitalRead(PIN_BUTTON) == LOW) {
      delay(50); // Debounce
      if (digitalRead(PIN_BUTTON) == LOW) {
        break; // Tombol ditekan -> selesai memasukkan obat!
      }
    }
    delay(20);
  }

  playAudio(1);

  // -------------------------------------------------------------
  // TAHAP 4: MENSTABILKAN & MENGUNCI BASELINE OBAT
  // -------------------------------------------------------------
  updateDisplay("MENYIMPAN BOBOT", "Menstabilkan Obat...", "Mohon Tunggu...");
  
  unsigned long startStable = millis();
  // Tunggu getaran tutup laci tenang selama 2 detik
  while (millis() - startStable < 2000) {
    updateSensorStream();
    delay(50);
  }

  // Kunci baseline total obat baru
  baseline_weight = current_ema;
  baseline_terkunci = true;
  perubahan_berat = 0.0;

  // Reset status 7 kompartemen untuk siklus obat baruk
  for (int i = 0; i < 7; i++) {
    boxTakenStatus[i] = false;
  }

  Serial.printf("[KALIBRASI SELESAI] Baseline Bobot Obat Terkunci: %.2f g\n", baseline_weight);
  updateDisplay("KALIBRASI SUKSES", "Obat Siap Digunakan", "Bobot: " + String(baseline_weight, 2) + "g");
  playAudio(2);
  delay(2500);

  // Kembalikan status layar ke Siaga
  String statusMinum = sudah_minum_jadwal_ini ? "Sudah Minum Obat" : "Belum Minum Obat";
  updateDisplay(statusMinum, "Sistem Siaga", "Jadwal: Kotak " + String(target_compartment));
}


int getTargetBoxFromDay() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println(F("[WAKTU] Gagal membaca hari NTP, gunakan default Kotak 1"));
    return 1;
  }
  // tm_wday: 0=Minggu, 1=Senin, ..., 6=Sabtu
  int box = (timeinfo.tm_wday == 0) ? 7 : timeinfo.tm_wday;
  return box;
}

void playAudio(int track) {
  myDFPlayer.playMp3Folder(track);
  if (track != 1 &&  track != 8) {
    audio_prioritas_aktif = true;
    timer_selesai_audio = millis();
  }
}

// ==========================================
// FUNGSI DEBUG: MONITORING DELTA & BOBOT TERKUNCI
// ==========================================
void cetakDebugBobot() {
  static unsigned long timerDebug = 0;
  
  // Cetak log telemetri setiap 200 ms (5 Hz)
  if (millis() - timerDebug >= 200) {
    timerDebug = millis();

    Serial.printf("[DEBUG LOG] Baseline (W_lock): %6.2f g | EMA Saat Ini: %6.2f g | Delta (dW): %6.2f g | StdDev: %6.4f g | Pintu: %d | W_Lock Status: %s\n",
                  baseline_weight,
                  current_ema,
                  perubahan_berat,
                  current_std_dev,
                  dataSensor.pintu_terbuka,
                  baseline_terkunci ? "TERKUNCI" : "BEBAS");
  }
}
