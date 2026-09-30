// ============================================================
// ALTITUDE BEACON — minimal BMP280 -> LoRa telemetry sketch
//
// Reads the BMP280, converts pressure to altitude above the
// power-on location, and transmits it as the SAME 6-byte packet
// the main flight firmware uses. That means the Ground Station
// sketch (and its WiFi dashboard) receive this unchanged.
//
// Use this to bench-verify the altitude path end-to-end:
//   BMP280 -> altitude math -> LoRa TX -> ground station dashboard
//
// Board:  ESP32 Dev Module
// Serial: 115200 — type 'Z' + Enter to re-zero altitude at any time
// ============================================================

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>

// --- Pins (identical to the main flight firmware) ---
#define PIN_BMP_SDA   21
#define PIN_BMP_SCL   22

#define PIN_LORA_MISO 19
#define PIN_LORA_MOSI 23
#define PIN_LORA_SCK  18
#define PIN_LORA_NSS  5
#define PIN_LORA_RST  14
#define PIN_LORA_DIO0 26

// --- LoRa settings (identical to the main flight firmware) ---
#define LORA_FREQ         434.5E6
#define LORA_BANDWIDTH    125E3
#define LORA_SPREADFACTOR 7
#define LORA_CODINGRATE   8
#define LORA_PREAMBLE     8
#define LORA_SYNC_WORD    0x12
#define LORA_TX_POWER     13

#define ROCKET_ID              37
#define TX_INTERVAL_MS         100   // 10 packets/second
#define PKT_TYPE_NORMAL_ALTITUDE 0x01

// --- Wire format: identical to the main firmware's Protocol.h ---
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  rocket_id;
    uint8_t  type;
    uint16_t sequence;   // big-endian on air
    uint16_t altitude;   // big-endian, 0.1 m resolution
};
#pragma pack(pop)

Adafruit_BMP280 bmp;
uint16_t seq = 0;
float baseline_hPa = 0.0f;   // P0: pressure at the power-on location
uint32_t lastTx = 0;

uint16_t swap16(uint16_t v) { return (v << 8) | (v >> 8); }

// Average 50 readings (~1 s) to get a stable launch-site pressure
float readBaseline_hPa() {
    float sum = 0;
    for (int i = 0; i < 50; i++) {
        sum += bmp.readPressure();
        delay(20);
    }
    return (sum / 50.0f) / 100.0f;
}

void setup() {
    Serial.begin(115200);
    delay(300);

    // ---- LoRa ----
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
    LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);
    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("CRITICAL: LoRa init failed. Halting.");
        while (1) { Serial.println("LoRa FAULT"); delay(1000); }
    }
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setSpreadingFactor(LORA_SPREADFACTOR);
    LoRa.setCodingRate4(LORA_CODINGRATE);
    LoRa.setPreambleLength(LORA_PREAMBLE);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.enableCrc();

    // ---- BMP280 ----
    Wire.begin(PIN_BMP_SDA, PIN_BMP_SCL);
    if (!bmp.begin(0x76, BMP280_CHIPID) && !bmp.begin(0x77, BMP280_CHIPID)) {
        Serial.println("CRITICAL: BMP280 init failed. Halting.");
        while (1) { Serial.println("BMP FAULT"); delay(1000); }
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,     // temperature
                    Adafruit_BMP280::SAMPLING_X8,     // pressure (the one that matters)
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_1);

    baseline_hPa = readBaseline_hPa();
    Serial.printf("Baseline pressure P0: %.2f hPa\n", baseline_hPa);
    Serial.println("Transmitting altitude at 10 Hz. Type 'Z' + Enter to re-zero.");
}

void loop() {
    // Optional re-zero: 'Z' + Enter in Serial Monitor sets current level to 0 m
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "Z") {
            baseline_hPa = readBaseline_hPa();
            Serial.printf("Re-zeroed. P0 = %.2f hPa\n", baseline_hPa);
        }
    }

    // ---- Read pressure and convert to altitude ----
    float p_hPa = bmp.readPressure() / 100.0f;
    float alt_m = 44330.0 * (1.0 - pow(p_hPa / baseline_hPa, 0.1903));

    // ---- Transmit at 10 Hz in the standard packet format ----
    if (millis() - lastTx >= TX_INTERVAL_MS) {
        lastTx = millis();

        TelemetryPacket pkt;
        pkt.rocket_id = ROCKET_ID;
        pkt.type      = PKT_TYPE_NORMAL_ALTITUDE;
        pkt.sequence  = swap16(seq++);

        if (alt_m < 0) alt_m = 0;   // avoid uint16 underflow wrapping on the pad
        pkt.altitude = swap16((uint16_t)round(alt_m * 10.0f));

        LoRa.beginPacket();
        LoRa.write((uint8_t*)&pkt, sizeof(pkt));
        LoRa.endPacket(true);

        Serial.printf("TX #%u  P=%.2f hPa  Alt=%.1f m\n",
                      (unsigned)(seq - 1), p_hPa, alt_m);
    }
}
