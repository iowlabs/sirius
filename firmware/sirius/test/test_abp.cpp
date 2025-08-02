#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Preferences.h>


// --- Credenciales ABP en LSB
static  u4_t DEVADDR = 0x260CD1CA; // debe ir igual que TTN
// debe ir igual que TTN
static  PROGMEM u1_t NWKSKEY[16] = { 0x94, 0xC2, 0x4A, 0xF6, 0x38, 0xEB, 0x4B, 0xFE, 0x24, 0xB1, 0xF2, 0x4F, 0xE7, 0x6B, 0xEB, 0xAA};
// debe ir al reves
static  PROGMEM u1_t APPSKEY[16] = {  0xF9, 0x19, 0x77, 0xFC, 0xAB, 0xF3, 0xAB, 0x8D, 0xCC, 0x78, 0xEE, 0xFC, 0x0B, 0x4D, 0x4C, 0x1E};

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;

// Pines RFM95 -> ESP32
const lmic_pinmap lmic_pins = {
    .nss = 15,        // CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 25,        // Reset
    .dio = {26, 27, LMIC_UNUSED_PIN}  // DIO0, DIO1, DIO2
};

RTC_DATA_ATTR uint32_t seqnoUpRTC = 0;

uint32_t counter = 0;

#define SLEEP_TIME_US (1 * 60 * 1000000ULL) // 1 minutos

Preferences preferences;


void sendMessage() {
  Serial.println(F("Generando datos..."));

  float d1 = random(200, 500) / 10.0;
  float d2 = random(200, 500) / 10.0;
  float d3 = random(200, 500) / 10.0;

  uint8_t payload[12];
  memcpy(payload,     &d1, 4);
  memcpy(payload + 4, &d2, 4);
  memcpy(payload + 8, &d3, 4);

  LMIC_setTxData2(1, payload, sizeof(payload), 0);
  Serial.println(F("Datos enviados."));
}


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

  switch (ev) {
    case EV_TXCOMPLETE:
      	Serial.println(F("Transmisión completada. Entrando a deep sleep..."));
		delay(2000);
		Serial.print("seq val :");
		Serial.print(LMIC.seqnoUp);
		Serial.print(", count : ");
		Serial.println(counter);
		counter++;
		preferences.begin("lora",false);
		preferences.putUInt("seqnoUp",LMIC.seqnoUp);
		preferences.putUInt("cnt",counter);

		preferences.end();
		delay(3000);
      	esp_deep_sleep(SLEEP_TIME_US);
      	break;

    default:
      break;
  }
}

void setup() {
  	Serial.begin(115200);
  	delay(1000);

  	Serial.println(F("Iniciando LMIC con ABP..."));
	preferences.begin("lora",true);
	uint32_t savedSeq = preferences.getUInt("seqnoUp",0);
	counter = preferences.getUInt("cnt",0);

	preferences.end();

  	os_init();
  	LMIC_reset();
	LMIC.seqnoUp = counter;

	// Deshabilitar todos los canales
	for (int i = 0; i < 72; i++) {
  		LMIC_disableChannel(i);
	}

	// Habilitar canales de la sub-banda 2 (canales 8-15: 916.8 - 918.2 MHz)
	for (int i = 8; i <= 15; i++) {
  		LMIC_enableChannel(i);
	}

  // Banda AU915, sub-banda 2
  //LMIC_selectSubBand(2);


  	LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
  	LMIC_setLinkCheckMode(0); // Desa
  	LMIC.seqnoUp = counter;
  	LMIC_setDrTxpow(DR_SF10, 14);

  	do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
