/*
 * PIDSender.ino
 *
 * NodeMCU ESP8266 PID bridge (write-only).
 *
 *   Python (TCP/CSV)  --WiFi-->  ESP8266  --SoftSerial 57600-->  Blackpill USART2
 *
 * UART frame (44 bytes, sólo ESP -> Blackpill):
 *   [0..35]  : 9 floats little-endian (Kp/Ki/Kd para roll, pitch, yaw)
 *   [36..39] : 1 float comp_filter_beta (peso del DMP en el filtro complementario)
 *   [40..43] : CRC32 sobre los 40 primeros bytes
 *
 * TCP protocol Python -> ESP (line-terminated):
 *   "kp_roll,ki_roll,kd_roll,kp_pitch,ki_pitch,kd_pitch,kp_yaw,ki_yaw,kd_yaw,comp_beta"
 *
 * Respuesta ESP -> Python:
 *   "OK kp,ki,kd,..."  (echo de lo enviado)
 *   "ERR:parse"        si el CSV está malformado
 *
 * Wiring:
 *   ESP D5 (GPIO14, TX) -> Blackpill PA3 (USART2_RX)
 *   GND                 -> Blackpill GND
 *
 * El blackpill aplica los gains nuevos sólo cuando está DISARMED.
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <string.h>

// ============================================================================
// USER CONFIG
// ============================================================================
#define WIFI_SSID       "CABLEVISION-d656"
#define WIFI_PASSWORD   "P1805SPABGVY"
#define PORT_TCP        8888

// ============================================================================
static const uint8_t  TRANSMITED_BYTES   = 44;
static const uint8_t  NUM_FLOATS         = 10;  // 9 PID gains + 1 comp_filter_beta
static const uint8_t  CRC_OFFSET         = 40;  // = NUM_FLOATS * 4
static const uint32_t BAUDRATE_BLACKPILL = 57600;
static const uint8_t  TX_RETRIES         = 3;

// SoftwareSerial pins
static const uint8_t  SS_TX_PIN          = 14;  // D5 -> blackpill PA3
static const uint8_t  SS_RX_PIN          = 12;  // D6 (no usado)

SoftwareSerial linkToBlackpill(SS_RX_PIN, SS_TX_PIN);

WiFiServer tcpServer(PORT_TCP);

// LED
static const uint8_t TX_LED_PIN = LED_BUILTIN;
static const uint8_t LED_ON     = LOW;
static const uint8_t LED_OFF    = HIGH;

// ============================================================================
// Utilities
// ============================================================================
static void blinkLED(uint8_t times, uint16_t period_ms)
{
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(TX_LED_PIN, LED_ON);
        delay(period_ms / 2);
        digitalWrite(TX_LED_PIN, LED_OFF);
        delay(period_ms / 2);
    }
}

// CRC32 (matches blackpill usart.c)
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL
                            : (crc >> 1);
        }
    }
    return ~crc;
}

// Build the 44-byte packet from 10 floats (9 PID gains + comp_filter_beta)
static void buildPacket(uint8_t out[TRANSMITED_BYTES], const float v[NUM_FLOATS])
{
    memcpy(out, v, NUM_FLOATS * sizeof(float));   // 40 bytes
    uint32_t crc = crc32_update(0xFFFFFFFF, out, CRC_OFFSET);
    memcpy(&out[CRC_OFFSET], &crc, 4);
}

// Forward one packet to the blackpill, with retries against SoftSerial jitter
static void sendPacketToBlackpill(const float v[NUM_FLOATS])
{
    uint8_t buf[TRANSMITED_BYTES];
    buildPacket(buf, v);

    blinkLED(2, 120);
    for (uint8_t i = 0; i < TX_RETRIES; i++) {
        linkToBlackpill.write(buf, TRANSMITED_BYTES);
        linkToBlackpill.flush();
        delay(8);
    }
}

// Parse a CSV line with exactly 9 floats. Returns true on success.
static bool parseCSV(const String &line, float out[NUM_FLOATS])
{
    int start = 0;
    for (uint8_t i = 0; i < NUM_FLOATS; i++) {
        int comma = line.indexOf(',', start);
        String tok;
        if (i < NUM_FLOATS - 1) {
            if (comma < 0) return false;
            tok = line.substring(start, comma);
        } else {
            tok = (comma >= 0) ? line.substring(start, comma) : line.substring(start);
        }
        tok.trim();
        if (tok.length() == 0) return false;
        out[i] = tok.toFloat();
        start = comma + 1;
    }
    return true;
}

// ============================================================================
// WiFi
// ============================================================================
static void connectWiFi()
{
    Serial.printf("Connecting to WiFi SSID \"%s\"", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    digitalWrite(TX_LED_PIN, LED_ON);
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print('.');
    }
    digitalWrite(TX_LED_PIN, LED_OFF);
    Serial.println();
    Serial.print("Connected. IP = ");
    Serial.println(WiFi.localIP());
    blinkLED(1, 200);
}

// ============================================================================
void setup()
{
    pinMode(TX_LED_PIN, OUTPUT);
    digitalWrite(TX_LED_PIN, LED_OFF);

    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("=== PIDSender / WiFi bridge starting ===");

    linkToBlackpill.begin(BAUDRATE_BLACKPILL);

    connectWiFi();

    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("TCP server listening on port %u\n", PORT_TCP);
    Serial.println("Format: 10 floats CSV (kp/ki/kd roll, kp/ki/kd pitch, kp/ki/kd yaw, comp_beta)");
}

// ============================================================================
void loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost, reconnecting...");
        connectWiFi();
    }

    WiFiClient client = tcpServer.available();
    if (!client) return;

    Serial.print("Client connected: ");
    Serial.println(client.remoteIP());

    client.setTimeout(2000);
    while (client.connected()) {
        if (!client.available()) {
            delay(5);
            continue;
        }

        String line = client.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        Serial.print("[RX] ");
        Serial.println(line);

        float vals[NUM_FLOATS];
        if (!parseCSV(line, vals)) {
            client.println("ERR:parse");
            Serial.println("  -> parse error");
            continue;
        }

        sendPacketToBlackpill(vals);

        client.print("OK ");
        for (uint8_t i = 0; i < NUM_FLOATS; i++) {
            client.print(vals[i], 4);
            if (i < NUM_FLOATS - 1) client.print(',');
        }
        client.println();
    }

    Serial.println("Client disconnected");
    client.stop();
}
