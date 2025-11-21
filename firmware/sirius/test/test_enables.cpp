#include <Arduino.h>

#define PMS_EN_PIN 4
#define SERVO_EN_PIN 12
#define V_EN	33
#define BAM_EN	13

void setup ()
{

	Serial.begin(115200);
	pinMode(PMS_EN_PIN,OUTPUT);
	pinMode(SERVO_EN_PIN,OUTPUT);

}


void loop()
{
	Serial.println("Encendiendo salidas");
	digitalWrite(PMS_EN_PIN,HIGH);
	digitalWrite(SERVO_EN_PIN,HIGH);
	delay(1000);
	Serial.println("Apagando salidas");
	digitalWrite(PMS_EN_PIN,LOW);
	digitalWrite(SERVO_EN_PIN,LOW);
	delay(1000);


}
