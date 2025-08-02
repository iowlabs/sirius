#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ---------- TU CONFIGURACIÓN ABP ----------
static  u4_t DEVADDR = 0x260C56B6;

static  PROGMEM u1_t NWKSKEY[16] =
 {
  0x84, 0xF4, 0xD6, 0x4C, 0xDF, 0xF7, 0xB5, 0x3F,
  0x03, 0x4E, 0x3E, 0x34, 0x98, 0x3C, 0x8F, 0x0A
};

/*
 {
  0x0A, 0x8F, 0x3C, 0x98, 0x34, 0x3E, 0x4E, 0x03,
  0x3F, 0xB5, 0xF7, 0xDF, 0x4C, 0xD6, 0xF4, 0x84
};
*/
static  PROGMEM u1_t APPSKEY[16] = {
  0xB0, 0x2D, 0x63, 0x97, 0x40, 0xEA, 0x95, 0x73,
  0x55, 0x59, 0x85, 0x77, 0x3C, 0x6F, 0x96, 0x94
};
// -------------------------------------------

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;

// Pines RFM95 -> ESP32
const lmic_pinmap lmic_pins = {
    .nss = 15,        // CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 25,        // Reset
    .dio = {26, 27, LMIC_UNUSED_PIN}  // DIO0, DIO1, DIO2
};


void do_send(osjob_t* j) {
  Serial.println(F("Enviando datos..."));

  float d1 = 10.0;
  float d2 = 20.0;
  float d3 = 30.5;

  uint8_t payload[12];
  memcpy(payload,     &d1, 4);
  memcpy(payload + 4, &d2, 4);
  memcpy(payload + 8, &d3, 4);

  Serial.print("d1:");
  Serial.print(d1);
  Serial.print(",d2:");
  Serial.print(d2);
  Serial.print(",d3:");
   Serial.println(d3);

Serial.print("Payload (hex): ");
for (int i = 0; i < 12; i++) {
  if (payload[i] < 0x10) Serial.print('0');  // para poner ceros a la izquierda
  Serial.print(payload[i], HEX);
  Serial.print(' ');
}
Serial.println();

if (LMIC.opmode & OP_TXRXPEND) {
  Serial.println(F("TX is pending, not sending"));
} else {
  LMIC_setTxData2(1, payload, sizeof(payload), 0);
  Serial.println(F("Packet queued"));
}
  Serial.print(F("TXMODE: freq="));
  Serial.print(LMIC.freq);
  Serial.print(F(" Hz, DR="));
  Serial.println(LMIC.datarate);
}

void onEvent(ev_t ev) {
  Serial.print(F("Evento: "));
  Serial.println(ev);
  if (ev == EV_TXCOMPLETE) {
    Serial.println(F("¡Transmisión completada!"));
// Reenvía en 10 segundos
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(30), do_send);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println(F("Iniciando ABP"));

  os_init();
  LMIC_reset();

  // Desactivar todos los canales
  for (int i = 0; i < 72; i++) {
    LMIC_disableChannel(i);
  }
  // Activar canales de la sub-banda 2
  for (int i = 8; i <= 15; i++) {
    LMIC_enableChannel(i);
  }

  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
  LMIC_setLinkCheckMode(0); // Desa
  LMIC.seqnoUp = 0;
  LMIC_setDrTxpow(DR_SF10, 14);

  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
