#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include "SPI.h"

// ---------- LoRaWAN OTAA KEYS -----------------
// Reemplaza con tus claves de TTN (en formato MSB):
static const u1_t PROGMEM APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xF0, 0x00, 0xAB, 0xCD }; // MSB
//static const u1_t PROGMEM APPEUI[8] = { 0xCD, 0xAB, 0x00, 0x0F, 0x7E, 0xD5, 0xB3, 0x70 }; // LSB

static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x45, 0x23, 0x9A, 0xCD, 0xEF, 0x11, 0x77 }; // MSB
//static const u1_t PROGMEM DEVEUI[8] = { 0x77, 0x11, 0xEF, 0xCD, 0x9A, 0x23, 0x45, 0x00 }; // LSB
static const u1_t PROGMEM APPKEY[16] = { 0xB1, 0xC2, 0xD3, 0xE4, 0xF5, 0x06, 0x17, 0x28,
                                         0x39, 0x4A, 0x5B, 0x6C, 0x7D, 0x8E, 0x9F, 0xA0 }; // MSB

void os_getArtEui(u1_t* buf)  { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf)  { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf)  { memcpy_P(buf, APPKEY, 16); }

#define SD_CS   5
#define MISO    19
#define MOSI    23
#define CLK     18
#define V_EN	33
#define BAM_EN	13
#define SDS_EN  14
#define LED 	2
#define CLEANER_PIN  	32
#define BAT_LEVEL_PIN 	35
#define MODE_SWITCH 	34


// Pines RFM95 -> ESP32
const lmic_pinmap lmic_pins = {
    .nss = 15,        // CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 25,        // Reset
    .dio = {26, 27, LMIC_UNUSED_PIN}  // DIO0, DIO1, DIO2
};


void sendMessage() {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("Esperando canal libre..."));
    } else {
        uint8_t data[1];
        data[0] = random(0, 255);
        LMIC_setTxData2(1, data, sizeof(data), 0);
        Serial.print(F("Enviando dato: "));
        Serial.println(data[0]);
    }
}

void onEvent(ev_t ev) {
    Serial.print(F("Evento: "));
    Serial.println(ev);
    if (ev == EV_JOINED) {
        Serial.println(F("¡Unido a la red TTN!"));
        // Deshabilita las siguientes líneas para evitar reintento
        //LMIC.setLinkCheckMode(0);
		sendMessage();
    }
    if (ev == EV_TXCOMPLETE) {
        Serial.println(F("Transmisión completada"));
        // Reintento tras 60s
        delay(60000);
        sendMessage();
    }
}



void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println(("Inicializando LoRaWAN OTAA con AU915 Sub Banda 2"));

    os_init();
    LMIC_reset();

    // Configura Sub-banda 2 (canales 8 a 15)
    for (uint8_t i = 0; i < 72; i++) {
        if (i < 8 || i > 15) {
            LMIC_disableChannel(i);
        }
    }

    // Intervalo de transmisión: 1 minuto
    LMIC_setDrTxpow(DR_SF10, 14);

    // Inicia OTAA Join
    LMIC_startJoining();
}

void loop() {
    os_runloop_once();
}
