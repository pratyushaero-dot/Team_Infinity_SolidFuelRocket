// ============================================================
// ROCKET FLIGHT COMPUTER - Arduino IDE sketch
// Auto-generated from the PlatformIO sources. Upload to the
// ESP32 WROOM wired to BMP280 (I2C 21/22) + SX1278 (SPI).
// Board: ESP32 Dev Module | Partition: Default 4MB with spiffs
// ============================================================

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <esp_task_wdt.h>


// --- Hardware Pins (Standardized per spec) ---
#define PIN_BMP_SDA  21
#define PIN_BMP_SCL  22

#define PIN_LORA_MISO 19
#define PIN_LORA_MOSI 23
#define PIN_LORA_SCK  18
#define PIN_LORA_NSS  5
#define PIN_LORA_RST  14
#define PIN_LORA_DIO0 26

#define PIN_LED_SENS   27
#define PIN_LED_TX     25
#define PIN_LED_STATUS 33
#define PIN_BUZZER     4

// --- LoRa Configuration (FAST TELEMETRY MODE) ---
// SF7 = shortest packets & most Doppler-tolerant (right choice for rockets;
// not max-range, but range is far beyond model-rocket needs).
#define LORA_FREQ         434.5E6 // 434.500 MHz
#define LORA_BANDWIDTH    125E3   // 125 kHz
#define LORA_SPREADFACTOR 7       // SF7 (Fastest transmission)
#define LORA_CODINGRATE   8       // 4/8 Maximum Error Correction to recover corrupted packets
#define LORA_PREAMBLE     8
#define LORA_SYNC_WORD    0x12    // Standard LoRa Sync Word
#define LORA_TX_POWER     13      // 13 dBm: license-free in most regions, ~30% less TX current, ample link margin

// --- Battery Monitoring (hardware: 100k/100k divider VBAT -> GPIO34) ---
#define PIN_BATT_ADC        34    // input-only ADC pin, safe for divider tap
#define BATT_DIVIDER_RATIO  2.0f  // (Rtop + Rbottom) / Rbottom for 100k/100k

// --- Flight Dynamics & Filtering Config ---
// Barometric moving average weight (0.0 to 1.0). Lower = more smoothing, higher = less lag.
// For a fast moving rocket, we want less software lag!
#define FILTER_EMA_ALPHA       0.4f
#define MEDIAN_WINDOW_SIZE     5

// Launch Detection (TBD in spec, establishing baseline)
#define LAUNCH_ALT_THRESHOLD   10.0f  // meters above P0 to declare launch
#define LAUNCH_ACCEL_THRESHOLD 15.0f  // m/s estimated velocity to confirm launch

// Apogee Detection
// Require X consecutive falling samples before declaring apogee to prevent false positives from noise.
#define APOGEE_FALL_THRESHOLD  2.0f   // meters below max altitude
#define APOGEE_PERSISTENCE     10     // samples

// Telemetry Timing
// 10 Hz = 100 ms per packet (safer overhead for SF7 125kHz ToA)
#define TELEMETRY_INTERVAL_MS  100     // 10Hz telemetry interval
#define APOGEE_BURST_INTERVAL  100     // 10Hz
#define APOGEE_BURST_COUNT     10     // 10 packets for apogee lock mark

// IDs
#define ROCKET_ID              37     // Standard test ID

#include <stdint.h>

// Packet Types
#define PKT_TYPE_NORMAL_ALTITUDE 0x01
#define PKT_TYPE_APOGEE          0x02

// Exact 6-byte payload specified in the reference document.
// Using compiler attributes to prevent padding bytes.
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  rocket_id;
    uint8_t  type;
    uint16_t sequence;   // Stored in Big-Endian format per spec!
    uint16_t altitude;   // Stored in Big-Endian format, 0.1m resolution
};
#pragma pack(pop)

// Flight States
enum FlightState {
    STATE_BOOT = 0,
    STATE_READY,
    STATE_ASCENT,
    STATE_NEAR_APOGEE,
    STATE_APOGEE_LOCKED,
    STATE_DESCENT,
    STATE_LANDED
};


class SensorFilter {
private:
    float medianBuffer[5];
    uint8_t bufferIndex;
    bool bufferFilled;
    float emaAlpha;
    float currentFiltered;

    // Helper to sort and find median
    float calculateMedian() {
        float sorted[5];
        memcpy(sorted, medianBuffer, sizeof(medianBuffer));
        // Simple insertion sort for 5 elements
        for (int i = 1; i < 5; i++) {
            float key = sorted[i];
            int j = i - 1;
            while (j >= 0 && sorted[j] > key) {
                sorted[j + 1] = sorted[j];
                j = j - 1;
            }
            sorted[j + 1] = key;
        }
        return sorted[2]; // Middle element
    }

public:
    SensorFilter(float alpha) : bufferIndex(0), bufferFilled(false), emaAlpha(alpha), currentFiltered(0.0f) {
        for(int i=0; i<5; i++) medianBuffer[i] = 0;
    }

    float update(float newRawValue) {
        medianBuffer[bufferIndex] = newRawValue;
        bufferIndex++;
        if (bufferIndex >= 5) {
            bufferIndex = 0;
            bufferFilled = true;
        }

        if (!bufferFilled) {
            currentFiltered = newRawValue; // Not enough data yet
            return currentFiltered;
        }

        // 1. Median Filter to eliminate sudden 1-sample spikes (venturi noise, sensor glitches)
        float medianVal = calculateMedian();

        // 2. EMA Filter to smooth the remaining curve
        currentFiltered = (emaAlpha * medianVal) + ((1.0f - emaAlpha) * currentFiltered);
        return currentFiltered;
    }

    float get() { return currentFiltered; }
};

#include <LittleFS.h>

#define LOG_FILENAME "/flight_data.bin"
#define LOG_BUFFER_SIZE 10 // Batch write 10 samples at a time to reduce Flash wear and delay

// Binary struct for the blackbox log
#pragma pack(push, 1)
struct LogEntry {
    uint32_t timestamp;
    uint8_t state;
    float current_altitude;
    float max_altitude;
    float battery_v;   // VBAT via external divider (added for single-cell BOM)
};
#pragma pack(pop)

class FlashLogger {
private:
    LogEntry buffer[LOG_BUFFER_SIZE];
    uint8_t bufferIndex;
    bool initialized;
    File logFile;

    void flushBuffer() {
        if (bufferIndex == 0 || !initialized || !logFile) return;

        // Safety limit: Stop logging if file exceeds 1.3MB to prevent lfs_alloc crash
        if (logFile.size() > 1300000) { 
            Serial.println("WARNING: Flash memory limit reached. Logger disabled to protect radio.");
            initialized = false;
            bufferIndex = 0;
            logFile.close();
            return;
        }

        logFile.write((uint8_t*)buffer, sizeof(LogEntry) * bufferIndex);
        logFile.flush(); // Force sync to physical flash without closing!
        bufferIndex = 0;
    }

public:
    FlashLogger() : bufferIndex(0), initialized(false) {}

    bool begin() {
        // Format on fail = true. First time it runs it will format the partition.
        if (!LittleFS.begin(true)) {
            Serial.println("CRITICAL: LittleFS Mount Failed");
            return false;
        }
        
        // Open file ONCE and keep it open. Opening in FILE_APPEND on a huge file 
        // takes a long time because LittleFS has to walk the block chain!
        logFile = LittleFS.open(LOG_FILENAME, FILE_APPEND);
        if (!logFile) {
            Serial.println("CRITICAL: Failed to open log file.");
            return false;
        }

        initialized = true;
        Serial.println("Flash Logger Initialized (File kept open for speed).");
        return true;
    }

    void log(uint32_t ts, uint8_t state, float alt, float max_alt, float batt_v) {
        if (!initialized) return;

        buffer[bufferIndex].timestamp = ts;
        buffer[bufferIndex].state = state;
        buffer[bufferIndex].current_altitude = alt;
        buffer[bufferIndex].max_altitude = max_alt;
        buffer[bufferIndex].battery_v = batt_v;
        bufferIndex++;

        // When buffer is full, flush to physical flash memory
        if (bufferIndex >= LOG_BUFFER_SIZE) {
            flushBuffer();
        }
    }

    // Force flush for emergency or landing
    void forceFlush() {
        flushBuffer();
    }

    // Dumps the entire binary log over Serial in CSV format
    void dumpLogToSerial() {
        if (!initialized) return;
        
        logFile.close(); // Close append handle
        File readHandle = LittleFS.open(LOG_FILENAME, FILE_READ);
        if (!readHandle) {
            Serial.println("No log file found.");
            return;
        }

        Serial.println("\n--- BEGIN FLIGHT LOG DUMP ---");
        Serial.println("Timestamp_ms,State,CurrentAlt_m,MaxAlt_m,Battery_V");

        LogEntry entry;
        while (readHandle.available() >= sizeof(LogEntry)) {
            readHandle.read((uint8_t*)&entry, sizeof(LogEntry));
            Serial.printf("%lu,%d,%.2f,%.2f,%.2f\n", 
                entry.timestamp, entry.state, entry.current_altitude, entry.max_altitude, entry.battery_v);
        }
        
        readHandle.close();
        Serial.println("--- END FLIGHT LOG DUMP ---\n");
        
        // Reopen for appending
        logFile = LittleFS.open(LOG_FILENAME, FILE_APPEND);
    }
    
    void eraseLog() {
        if (logFile) logFile.close();
        if (LittleFS.exists(LOG_FILENAME)) {
            LittleFS.remove(LOG_FILENAME);
            Serial.println("Log file erased.");
        }
        // Reopen a fresh one
        logFile = LittleFS.open(LOG_FILENAME, FILE_APPEND);
        bufferIndex = 0;
    }
};

#include <LittleFS.h>

// Flash mirror of flight-critical state. RTC RAM (guarded by a magic word)
// is the primary carrier across resets; this file is the fallback for a
// cold boot (battery fully removed / brownout), where RTC RAM is garbage.
// Without it, altitude after a mid-flight power cut would silently
// recalibrate to ~0 m at whatever pressure the rocket happened to fall in.

#define PERSIST_FILENAME   "/persist.bin"
#define PERSIST_MAGIC      0x50525331u   // "PRS1"
#define PERSIST_PERIOD_MS  5000u         // refresh rate while flying

#pragma pack(push, 1)
struct PersistData {
    uint32_t magic;
    float    launch_pressure;
    float    max_altitude;
    uint8_t  state;
    uint8_t  checksum;   // byte-sum of all preceding bytes
};
#pragma pack(pop)

uint8_t persistChecksum(const PersistData& d) {
    const uint8_t* p = (const uint8_t*)&d;
    uint8_t sum = 0;
    for (uint8_t i = 0; i < sizeof(PersistData) - 1; i++) sum += p[i];
    return sum;
}

bool persistLoad(PersistData& out) {
    if (!LittleFS.exists(PERSIST_FILENAME)) return false;
    File f = LittleFS.open(PERSIST_FILENAME, FILE_READ);
    if (!f) return false;
    if (f.size() != sizeof(PersistData)) { f.close(); return false; }
    PersistData d;
    bool ok = (f.read((uint8_t*)&d, sizeof(d)) == sizeof(d));
    f.close();
    if (!ok || d.magic != PERSIST_MAGIC || d.checksum != persistChecksum(d)) return false;
    out = d;
    return true;
}

bool persistSave(const PersistData& d) {
    PersistData tmp = d;
    tmp.magic = PERSIST_MAGIC;
    tmp.checksum = persistChecksum(tmp);
    File f = LittleFS.open(PERSIST_FILENAME, FILE_WRITE);
    if (!f) return false;
    bool ok = (f.write((uint8_t*)&tmp, sizeof(tmp)) == sizeof(tmp));
    f.close();
    return ok;
}

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT_SECONDS 5

// --- RTC RAM State Retention (survives soft resets; magic-guarded) ---
#define RTC_MAGIC 0xC0FFEE42u
RTC_DATA_ATTR uint32_t rtc_magic = 0;
RTC_DATA_ATTR FlightState rtc_state = STATE_BOOT;
RTC_DATA_ATTR float rtc_launch_pressure = 0.0f;
RTC_DATA_ATTR float rtc_max_altitude = 0.0f;

// --- Global Objects ---
Adafruit_BMP280 bmp;
SensorFilter altFilter(FILTER_EMA_ALPHA);
FlashLogger logger;

// --- State Variables ---
float currentAltitude = 0.0f;
float maxAltitude = 0.0f;
uint16_t packetSequence = 0;
int apogeePersistenceCounter = 0;
int apogeeBurstCounter = 0;
uint32_t lastTelemetryTime = 0;
uint32_t lastLogTime = 0;
uint32_t lastPersistTime = 0;
int landingCounter = 0;
// Serial line assembly (replaces blocking Serial.readStringUntil)
char serialBuf[24];
uint8_t serialLen = 0;

// Endian swap utility
uint16_t swapEndian(uint16_t val) {
    return (val << 8) | (val >> 8);
}

// ---- Serial command polling (non-blocking) ----
// Reads available bytes into a small buffer; returns true with a completed
// line in `out`. Never stalls the flight loop, unlike readStringUntil().
bool pollSerialCommand(char* out, uint8_t maxlen) {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialLen > 0) {
                serialBuf[serialLen] = '\0';
                strncpy(out, serialBuf, maxlen);
                serialLen = 0;
                return true;
            }
        } else if (serialLen < sizeof(serialBuf) - 1) {
            serialBuf[serialLen++] = c;
        }
    }
    return false;
}

void persistFlightState() {
    PersistData d;
    d.launch_pressure = rtc_launch_pressure;
    d.max_altitude = rtc_max_altitude;
    d.state = (uint8_t)rtc_state;
    persistSave(d);
}

void setup() {
    Serial.begin(115200);

    // 80 MHz: this workload doesn't need 240 MHz — saves ~40-50 mA average,
    // nearly doubling endurance on the single-cell battery.
    setCpuFrequencyMhz(80);

    // Setup Watchdog Timer (compatible with arduino-esp32 core 2.x and 3.x)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_cfg);
#else
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
#endif
    esp_task_wdt_add(NULL);

    // NOTE: no LEDs/buzzer on this flight board — status is observed via
    // telemetry packets and the USB serial console.

    // Battery voltage via external 100k/100k divider on GPIO34 (input-only)
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_BATT_ADC, ADC_11db); // full ~3.3V range

    // Initialize Flash Logger
    if (!logger.begin()) {
        Serial.println("WARNING: Flash logging disabled.");
    }

    // ONLY wait for DUMP commands if this is a fresh ground boot.
    // If the battery bounced mid-flight, we need to bypass this instantly!
    if (rtc_state == STATE_BOOT || rtc_state == STATE_LANDED || rtc_state == STATE_READY) {
        delay(1000); // Give user time to open terminal
        Serial.println("Send 'DUMP' within 3 seconds to read blackbox data, or 'ERASE' to clear.");
        uint32_t bootTime = millis();
        while (millis() - bootTime < 3000) {
            char cmd[24];
            if (pollSerialCommand(cmd, sizeof(cmd))) {
                if (strcmp(cmd, "DUMP") == 0) {
                    logger.dumpLogToSerial();
                } else if (strcmp(cmd, "ERASE") == 0) {
                    logger.eraseLog();
                }
            }
            esp_task_wdt_reset();
            delay(10);
        }
    }

    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
    LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("CRITICAL: LoRa init failed. Halting.");
        while(1) {
            Serial.println("LoRa FAULT");   // heartbeat via USB instead of an LED
            delay(1000);
        }
    }

    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setSpreadingFactor(LORA_SPREADFACTOR);
    LoRa.setCodingRate4(LORA_CODINGRATE);
    LoRa.setPreambleLength(LORA_PREAMBLE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.enableCrc();

    Wire.begin(PIN_BMP_SDA, PIN_BMP_SCL);
    Wire.setTimeOut(100);

    if (!bmp.begin(0x76, BMP280_CHIPID)) {
        if (!bmp.begin(0x77, BMP280_CHIPID)) {
            Serial.println("CRITICAL: BMP280 init failed. Halting.");
            while(1) {
                Serial.println("BMP FAULT");   // heartbeat via USB instead of an LED
                delay(1000);
            }
        }
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X8,
                    Adafruit_BMP280::FILTER_X4,      // Lower hardware IIR filter to prevent lag in fast rocket
                    Adafruit_BMP280::STANDBY_MS_1);

    // ---- State recovery ----
    // RTC RAM is valid only if the magic word survived (soft reset).
    if (rtc_magic != RTC_MAGIC) {
        // Garbage RTC RAM (cold boot). Try the flash mirror before recalibrating.
        PersistData d;
        if (persistLoad(d) && d.state == STATE_ASCENT) {
            rtc_magic = RTC_MAGIC;
            rtc_state = (FlightState)d.state;
            rtc_launch_pressure = d.launch_pressure;
            rtc_max_altitude = d.max_altitude;
            maxAltitude = rtc_max_altitude;
            Serial.printf("RECOVERED from flash after cold boot! State: %d, P0: %.2f, MaxAlt: %.1f\n",
                          rtc_state, rtc_launch_pressure, rtc_max_altitude);
        } else {
            rtc_magic = RTC_MAGIC;
            rtc_state = STATE_BOOT;
        }
    } else if (rtc_state == STATE_ASCENT) {
        // Soft reset mid-flight: RTC RAM is the fresher copy; mirror it to flash.
        maxAltitude = rtc_max_altitude;
        persistFlightState();
        Serial.printf("Recovered Mid-Flight! State: %d, P0: %.2f\n", rtc_state, rtc_launch_pressure);
    } else {
        maxAltitude = rtc_max_altitude;
    }

    if (rtc_state == STATE_BOOT || rtc_state == STATE_LANDED) {
        Serial.println("Establishing new launch reference...");
        rtc_state = STATE_READY;
        float p_sum = 0;
        for (int i = 0; i < 50; i++) {
            p_sum += bmp.readPressure();
            delay(20);
        }
        rtc_launch_pressure = (p_sum / 50.0f) / 100.0f;
        rtc_max_altitude = 0.0f;
        maxAltitude = 0.0f;
        packetSequence = 0;
        apogeeBurstCounter = 0;
        apogeePersistenceCounter = 0;
        persistFlightState();   // store the new P0 immediately
        Serial.printf("Launch Pressure Set: %.2f hPa\n", rtc_launch_pressure);
    }
}

void transmitTelemetry(uint8_t type, float altitude) {
    TelemetryPacket pkt;
    pkt.rocket_id = ROCKET_ID;
    pkt.type = type;
    pkt.sequence = swapEndian(packetSequence++);

    // Prevent negative altitude noise on the launch pad from underflowing
    // the unsigned 16-bit integer and wrapping around to 6553.5 meters.
    if (altitude < 0.0f) {
        altitude = 0.0f;
    }

    uint16_t alt_encoded = (uint16_t)round(altitude * 10.0f);
    pkt.altitude = swapEndian(alt_encoded);

    LoRa.beginPacket();
    LoRa.write((uint8_t*)&pkt, sizeof(TelemetryPacket));
    LoRa.endPacket(true);
}

void loop() {
    esp_task_wdt_reset();
    uint32_t now = millis();

    // 20 Hz evaluation tick — computed BEFORE the log block updates
    // lastLogTime, so the state machine sees the same tick the logger uses.
    bool tick20 = (now - lastLogTime >= 50);

    float p = bmp.readPressure() / 100.0f;
    float raw_altitude = 44330.0 * (1.0 - pow(p / rtc_launch_pressure, 0.1903));
    currentAltitude = altFilter.update(raw_altitude);

    if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
        rtc_max_altitude = maxAltitude;
    }

    // High frequency internal logging (approx 20Hz, writing in batches of 10)
    if (tick20) {
        float battV = analogReadMilliVolts(PIN_BATT_ADC) / 1000.0f * BATT_DIVIDER_RATIO;
        logger.log(now, rtc_state, currentAltitude, maxAltitude, battV);
        lastLogTime = now;
    }

    // Periodic flash mirror of flight-critical state while flying
    if ((rtc_state == STATE_ASCENT || rtc_state == STATE_DESCENT || rtc_state == STATE_NEAR_APOGEE)
        && (now - lastPersistTime >= PERSIST_PERIOD_MS)) {
        persistFlightState();
        lastPersistTime = now;
    }

    switch (rtc_state) {
        case STATE_READY:
            if (currentAltitude > LAUNCH_ALT_THRESHOLD) {
                rtc_state = STATE_ASCENT;
                logger.forceFlush();
                persistFlightState();   // lock in P0 + ASCENT at launch
            }
            break;

        case STATE_ASCENT:
            // Apogee persistence counted at the 20 Hz log tick (~500 ms of
            // sustained fall), not raw loop iterations — immune to filter
            // ripple and single-sample glitches.
            if (tick20) {
                if (currentAltitude < maxAltitude - APOGEE_FALL_THRESHOLD) {
                    apogeePersistenceCounter++;
                    if (apogeePersistenceCounter >= APOGEE_PERSISTENCE) {
                        rtc_state = STATE_APOGEE_LOCKED;
                        logger.forceFlush();
                        apogeeBurstCounter = 0;
                    }
                } else {
                    apogeePersistenceCounter = 0;
                }
            }
            break;

        case STATE_APOGEE_LOCKED:
            rtc_state = STATE_DESCENT;
            landingCounter = 0;
            break;

        case STATE_DESCENT:
            // Landing requires persistence: 50 consecutive ticks (~2.5 s)
            // below threshold, so a tree canopy or hover does not instantly
            // end the flight in the log.
            if (tick20) {
                if (currentAltitude < 5.0f) {
                    landingCounter++;
                    if (landingCounter >= 50) {
                        rtc_state = STATE_LANDED;
                        logger.forceFlush();
                        persistFlightState();
                    }
                } else {
                    landingCounter = 0;
                }
            }
            break;
    }

    if (rtc_state == STATE_APOGEE_LOCKED || (apogeeBurstCounter > 0 && apogeeBurstCounter < APOGEE_BURST_COUNT)) {
        if (now - lastTelemetryTime >= APOGEE_BURST_INTERVAL) {
            transmitTelemetry(PKT_TYPE_APOGEE, maxAltitude);
            lastTelemetryTime = now;
            apogeeBurstCounter++;
        }
    } else {
        if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
            transmitTelemetry(PKT_TYPE_NORMAL_ALTITUDE, currentAltitude);
            lastTelemetryTime = now;
        }
    }

    // Non-blocking serial command poll (replaces readStringUntil)
    char cmd[24];
    if (pollSerialCommand(cmd, sizeof(cmd))) {
        if (strcmp(cmd, "DUMP") == 0) {
            logger.forceFlush();
            logger.dumpLogToSerial();
        }
    }

    delay(10);
}
