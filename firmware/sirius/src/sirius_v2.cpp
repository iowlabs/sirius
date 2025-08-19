/*

Sirius main code

This code use ABP - config in the node test 01 on TTN sirius


Gateway UI a84041fdfe2ad38c
Austrialia 915-928 FSB2


DEV ADDR 0x260CD1CA  <---- must be in the same order than TTN
NwksKEY 6357E3FEAB222899D9EAA954C52E64CB
APPKEY A55A75B53AF2F3479C1EDE347F2013A3
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <SDS011.h>
#include <SparkFun_RV8803.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include <WiFi.h>
#include <Preferences.h>



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

#define ID 	"NA01"

//#define CLEAN_TIME 		700
#define CLEAN_TIME  300
#define SENSOR_BR 	 4800
#define FILENAME_TEMPORAL  		"/log_temp.txt"
#define FILENAME_HISTORICAL 	"/log_data.txt"

//default settings
#define N_CYCLES 		3 	// n TIMES
#define N_CLEAN  		6 	// N SAMPLES_TIMES. This time is a multiple of samples time
#define N_SAMPLES 		1
#define SAMPLE_TIME 	20 	// MINUTES

#define SLEEP_TIME_US (1 * 60 * 1000000ULL) // 20 minutos



// ABP credentials
// use MBS fot NWKSKEY
static  PROGMEM u1_t NWKSKEY[16] = { 0x94, 0xC2, 0x4A, 0xF6, 0x38, 0xEB, 0x4B, 0xFE, 0x24, 0xB1, 0xF2, 0x4F, 0xE7, 0x6B, 0xEB, 0xAA};
// use LBS for APPKEY
static PROGMEM u1_t APPSKEY[16] = { 0xA5, 0x5A, 0x75, 0xB5, 0x3A, 0xF2, 0xF3, 0x47, 0x9C, 0x1E, 0xDE, 0x34, 0x7F, 0x20, 0x13, 0xA3};
// use MBS for DEV ADDRESS
static u4_t DEVADDR = 0x260CD1CA;

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

//#define DEBUG 1
//Macros for enable serial prints
#if DEBUG
#define printd(s) {Serial.print((s));}
#define printlnd(s) {Serial.println((s));}
#else
#define printd(s)
#define printlnd(s)
#endif


//***************************************************
//INSTANCES

// SHT31 Sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// PM Sensor
SDS011 my_sds;
HardwareSerial port(2);

// Servo cleaner
Servo cleaner;

//RTC
RV8803 rtc;

//pemanent data
Preferences preferences;

//***************************************************
//VARIABLES

//sensor variables
float p10, p25;
float v_bat 	= 0.0;
int   v_bat_val = 0;
float t 		= 0.0;
float h 		= 0.0;

//timestamp
long timestamp 	= 0;
long new_time 	= 0;

//control variables for SDS - PM sensor
int   err 	= 0;
int try_sds = 0;

//output msg
String payload = "";

//variables guardadas de forma permanente
RTC_DATA_ATTR int counter_cycles_pub 		= 0;
RTC_DATA_ATTR int counter_cycles_cleaner 	= 0;
RTC_DATA_ATTR long last_clean_time 			= 0;

//contadores permanetes
RTC_DATA_ATTR int n_cycles 		= N_CYCLES;
RTC_DATA_ATTR int n_clean 		= N_CLEAN;
RTC_DATA_ATTR int n_samples 	= N_SAMPLES;
RTC_DATA_ATTR int sample_time 	= SAMPLE_TIME;
RTC_DATA_ATTR long clear_time 	= CLEAN_TIME;

//permanent data
RTC_DATA_ATTR uint32_t seqnoUpRTC = 0;
uint32_t counter = 0;


//***************************************************
//Functions
void disableWiFi();

//sensors funtions
void readSensors();
void readPM();
void readTemp();
void readBatt();
void clean();
String pubData();

//Lora WAN functions
void do_send(osjob_t* j);
void onEvent(ev_t ev);

// SD funtions
void initMicroSD(void);
void saveData(String _data);
void appendFile(fs::FS &fs, const char *path, const char *message);

void setup()
{
  	// Desactiva WiFi/Bluetooth para ahorrar energía
  	disableWiFi();
	Serial.begin(115200);
  	delay(5);

	//**********************
	// CONTROL - PINES

	rtc_gpio_hold_dis(GPIO_NUM_33); //control Motor PWR supply
	rtc_gpio_hold_dis(GPIO_NUM_14);

	pinMode(SD_CS, OUTPUT);
	pinMode(V_EN,  OUTPUT); 	//enable power to sd and others
	//pinMode(CLEANER_PIN,OUTPUT);
	pinMode(SDS_EN,OUTPUT);
	pinMode(BAM_EN, OUTPUT);

	digitalWrite(SD_CS ,HIGH);
	digitalWrite(V_EN,HIGH);
	digitalWrite(SDS_EN,HIGH);
	digitalWrite(BAM_EN, HIGH);


	printlnd(F("Iniciando Sirius con ABP..."));

	preferences.begin("lora",true);
	uint32_t savedSeq = preferences.getUInt("seqnoUp",0);
	counter = preferences.getUInt("cnt",0);
	preferences.end();

	Wire.begin();


	//**********************
	// Inicializar dependencias

	// inicia el limpiador
	cleaner.attach(CLEANER_PIN);
	cleaner.write(140); // Revisar segun motor
	delay(5);

	// iniciar SD
	//if(SD.begin(SD_CS)){ initMicroSD();	}
	//else{printlnd("SD problem");}

  	// Inicia sensor SHT31
  	if (!sht31.begin(0x44))
	{
    	printlnd("Error: no se detecta SHT31.");
  	}
	// inicia sensor de MP
	my_sds.begin(&port);

	// iniciar RTC
	rtc.begin();
	rtc.set24Hour();

	//**********************
	// LEER SENSORES

	//Leer un dato de los sensores
	rtc.updateTime();
	timestamp = rtc.getEpoch();
	readSensors();

	//payload = pubData(); //armar el string
	//printlnd(payload);
	//saveData(payload);

	//check if it's time for clean
	if(counter_cycles_cleaner >= n_clean )
	{
		printlnd("clear sensor");
		clean();
		counter_cycles_cleaner = 0;
	}
	counter_cycles_cleaner += 1 ;

	//**********************
	// LoRaWAN connections

  	// Inicia LoRa
  	os_init();
  	LMIC_reset();
	LMIC.seqnoUp = counter;

	// Desable all channels
	for (int i = 0; i < 72; i++) {
  		LMIC_disableChannel(i);
	}

	// enable channels on sub-band 2 (channels 8-15: 916.8 - 918.2 MHz)
	for (int i = 8; i <= 15; i++) {
  		LMIC_enableChannel(i);
	}


  	// Config ABP
  	LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

	LMIC_setLinkCheckMode(0); // Desable chek mode
  	LMIC.seqnoUp = counter;
  	LMIC_setDrTxpow(DR_SF10, 14);

  	do_send(&sendjob);








}

void loop() {
  os_runloop_once();
}

void disableWiFi()
{
	WiFi.mode(WIFI_OFF);
	btStop();
}

void readSensors()
{
	readPM();
	readBatt();
	readTemp();
}

String pubData()
{
	// Construcción de JSON simple
	//char payload[64];
	//snprintf(payload, sizeof(payload), "{\"t\":%.2f,\"h\":%.2f}", temp, hum);
	//Serial.print("Payload: ");
	//Serial.println(payload);

	StaticJsonDocument<256> doc_tx;

  	//doc_tx["id"]    	= ID;
	//doc_tx["time"]  	= timestamp;
	doc_tx["p2"]  	= (int)(p25*100 + 0.5)/100.0;
  	doc_tx["p1"] 	 	= (int)(p10*100 + 0.5)/100.0;
	doc_tx["b"]		= (int)(v_bat*100 + 0.5)/100.0;
	doc_tx["t"]			= (int)(t*100 + 0.5)/100.0;
	doc_tx["h"]			= (int)(h*100 + 0.5)/100.0;

	String json;
  	serializeJson(doc_tx, json);
	return json;
}

void readPM()
{
	delay(3000);
	err = my_sds.read(&p25, &p10);
	while( try_sds < n_cycles)
	{
		if (!err)
		{
			printlnd("P2.5: " + String(p25)+ " P10:  " + String(p10));
			try_sds = 0;
			break;
		}
		else
		{
			printlnd("error en lectura del sensor. Intentando nuevamente");
			delay(5000);
			err = my_sds.read(&p25, &p10);
			try_sds += 1;
		}
	}
	if(err)
	{
		p25 = 0.0;
		p10 = 0.0;
		try_sds = 0;
	}
}


void readBatt()
{
	v_bat_val = analogRead(BAT_LEVEL_PIN);
	printd("valor raw de la batería : ");
	printd(v_bat_val);

	v_bat = (v_bat_val/4095.0) * (4.33*3.3)+1;//100+383
	printd(" valor en volts : ");
	printlnd(v_bat);
}

void readTemp()
{
	t = sht31.readTemperature();
  	h = sht31.readHumidity();

	if (isnan(t))
	{
		t = 0.0;
	}

	if (isnan(h))
	{
		h = 0.0;
	}
}

void clean()
{
    printlnd("Cleaner on");
	cleaner.attach(CLEANER_PIN);
	cleaner.write(90);  // Ajusta segun motor
	delay(clear_time);
	cleaner.write(140);  // Ajusta segun motor
	delay(clear_time*3);
    printlnd("Cleaner off");
}

void do_send(osjob_t* j) {
  printlnd(F("Enviando datos..."));

/*
  //Valores fijos para pruebas
  float d1 = 10.0;
  float d2 = 20.0;
  float d3 = 30.5;
  float d4 = 24.5;
  float d5 = 10.4;
*/

  float d1 = p10;
  float d2 = p25;
  float d3 = t;
  float d4 = h;
  float d5 = v_bat;
  uint8_t payload[20];
  memcpy(payload,     &d1, 4);
  memcpy(payload + 4, &d2, 4);
  memcpy(payload + 8, &d3, 4);
  memcpy(payload + 12, &d4, 4);
  memcpy(payload + 16, &d5, 4);

  printd("d1:");
  printd(d1);
  printd(",d2:");
  printd(d2);
  printd(",d3:");
  printlnd(d3);

printd("Payload (hex): ");
//for (int i = 0; i < 12; i++) {
//  if (payload[i] < 0x10) Serial.print('0');  // para poner ceros a la izquierda
//  Serial.print(payload[i], HEX);
//  Serial.print(' ');
//}
printlnd(" ");

if (LMIC.opmode & OP_TXRXPEND) {
  printlnd("TX is pending, not sending");
} else {
  LMIC_setTxData2(1, payload, sizeof(payload), 0);
  printlnd("Packet queued");
}
  printd("TXMODE: freq=");
  printd(LMIC.freq);
  printd(" Hz, DR=");
  printlnd(LMIC.datarate);
}

void onEvent(ev_t ev) {
  printd("Evento: ");
  printlnd(ev);

  switch (ev) {
    case EV_TXCOMPLETE:
      	printlnd("Transmisión completada. Entrando a deep sleep...");
		delay(200);
		printd("seq val :");
		printd(LMIC.seqnoUp);
		printd(", count : ");
		printlnd(counter);
		counter++;
		preferences.begin("lora",false);
		preferences.putUInt("seqnoUp",LMIC.seqnoUp);
		preferences.putUInt("cnt",counter);

		preferences.end();
		//**********************
		// Guardar valores de las señales.

		digitalWrite(V_EN,LOW);		// disable dependencies from the iguana board
		digitalWrite(SDS_EN,HIGH);	// disable voltaje supply for SDS sensor and cleaner motor

		// Habilitar el control RTC del pin 33
		rtc_gpio_init(GPIO_NUM_33);
		rtc_gpio_set_direction(GPIO_NUM_33, RTC_GPIO_MODE_OUTPUT_ONLY);
		rtc_gpio_pulldown_en(GPIO_NUM_33);  // Habilitar el pulldown interno RTC
		rtc_gpio_hold_en(GPIO_NUM_33);     // Fijar el estado del pin durante el deep sleep

		// Habilitar el control RTC del pin 14
		rtc_gpio_init(GPIO_NUM_14);
		rtc_gpio_set_direction(GPIO_NUM_14, RTC_GPIO_MODE_OUTPUT_ONLY);
		rtc_gpio_set_level(GPIO_NUM_14, 1); // Mantener HIGH
		rtc_gpio_hold_en(GPIO_NUM_14);     // Fijar el estado del pin durant

		delay(100);
      	esp_deep_sleep(SLEEP_TIME_US);
      	break;

    default:
      break;
  }
}





void saveData( String _data )
{
	appendFile(SD, FILENAME_HISTORICAL, _data.c_str());
	appendFile(SD, FILENAME_HISTORICAL, "\n");
}

void initMicroSD()
{
    // Asegurarse de que la tarjeta
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
	{
        printlnd("¡No se ha encontrado tarjeta SD!");
        return;
    }

    // Creación del archivo
    File file = SD.open(FILENAME_TEMPORAL);
    if (!file)
    {
        printlnd("Archivo no existe");
        printlnd("Creando archivo...");
        File fileT= SD.open(FILENAME_TEMPORAL, FILE_WRITE);
        fileT.close();
    }
    else Serial.println("Archivo ya existe");
    file.close();

    // Creación del archivo
    File file2 = SD.open(FILENAME_HISTORICAL);
    if (!file2)
    {
        printlnd("Archivo no existe");
        printlnd("Creando archivo...");
        File fileT= SD.open(FILENAME_HISTORICAL, FILE_WRITE);
        fileT.close();
    }
    else printlnd("Archivo ya existe");
    file2.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
    // Agregando datos a la tarjeta SD
    //Serial.printf("Agregando al archivo: %s\n", path);
    File file = fs.open(path, FILE_APPEND);

    // Preguntando si existe el archivo
    if (!file) {
        printlnd("Fallo al abrir el archivo para agregar datos");
        return;
    }

    // Reporte
    if (!file.print(message)) printlnd("Append failed");

    // Cerrando
    file.close();
}
