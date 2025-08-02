/*


Gateway UI a84041fdfe2ad38c
Austrialia 915-928 FSB2

DEV ID 70B3D57ED0072217
DEV ADDR 260C3F1E
APPKEY A55A75B53AF2F3479C1EDE347F2013A3
NwksKEY 6357E3FEAB222899D9EAA954C52E64CB

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

// En lmic_project_config.h
#define CFG_au915 1

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

// ABP credentials (reemplaza por tus datos reales de TTN)
static  u1_t NWKSKEY[16] = { 0x63, 0x57, 0xE3, 0xFE, 0xAB, 0x22, 0x28, 0x99, 0xD9, 0xEA, 0xA9, 0x54, 0xC5, 0x2E, 0x64, 0xCB };
//static  u1_t NWKSKEY[16] = { 0xCB, 0x64, 0x2E, 0xC5, 0x54, 0xA9, 0xEA, 0xD9, 0x99, 0x28, 0x22, 0xAB, 0xFE, 0xE3, 0x57, 0x63 };

static  u1_t APPSKEY[16] = { 0xA5, 0x5A, 0x75, 0xB5, 0x3A, 0xF2, 0xF3, 0x47, 0x9C, 0x1E, 0xDE, 0x34, 0x7F, 0x20, 0x13, 0xA3};
//static  u1_t APPSKEY[16] = { 0xA3, 0x13, 0x20, 0x7F, 0x34, 0xDE, 0x1E, 0x9C, 0x47, 0xF3, 0xF2, 0x3A, 0xB5, 0x75, 0x5A, 0xA5};
static  u4_t DEVADDR = 0x260C3F1E; // ejemplo TTN

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


// Pines RFM95 -> ESP32
const lmic_pinmap lmic_pins = {
    .nss = 15,        // CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 25,        // Reset
    .dio = {26, 27, LMIC_UNUSED_PIN}  // DIO0, DIO1, DIO2
};

#define DEBUG 1
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


//***************************************************
//Functions
void disableWiFi();
void readSensors();
void readPM();
void readTemp();
void readBatt();
void clean();
String pubData();
void initMicroSD(void);
void saveData(String _data);
void appendFile(fs::FS &fs, const char *path, const char *message);

void setup()
{
  	// Desactiva WiFi/Bluetooth para ahorrar energía
  	disableWiFi();


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

  	Serial.begin(115200);
  	delay(1000);
  	Serial.println("Iniciando...");

  	Wire.begin();


	//**********************
	// Inicializar dependencias

	// inicia el limpiador
	cleaner.attach(CLEANER_PIN);
	cleaner.write(140); // Revisar segun motor
	delay(200);

	// iniciar SD
	//if(SD.begin(SD_CS)){ initMicroSD();	}
	//else{printlnd("SD problem");}

  	// Inicia sensor SHT31
  	if (!sht31.begin(0x44))
	{
    	Serial.println("Error: no se detecta SHT31.");
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
	payload = pubData(); //armar el string
	printlnd(payload);
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

  	// Configura sesión ABP
  	LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  	// Configura canales según tu región
  	LMIC_selectSubBand(2); //sub-banda 1 para 915



  	// Cola el mensaje
 	//LMIC_setLinkCheckMode(0);  // Desactiva el link check (evita reintentos OTAA)

  	LMIC_setTxData2(1, (uint8_t*)payload.c_str(), strlen(payload.c_str()), 0);
  	Serial.println("Paquete enviado. Esperando confirmación...");

  	// Espera envío (simple, no robusto)
  	while (LMIC.opmode & OP_TXRXPEND)
	{
    	os_runloop_once();
  	}
	Serial.println("Envío completado. Entrando en deep sleep...");

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


	//**********************
	// Dormir

  	// Entra en deep sleep por 10 minutos
  	esp_sleep_enable_timer_wakeup(1 * 60 * 1000000ULL);
  	esp_deep_sleep_start();
}

void loop() {
  // Nunca llega aquí por el deep sleep
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
