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

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	Serial.println("testing");
	delay(9600);
}
