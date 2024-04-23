/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 15/04/24

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//Pin definitions for Heltec ESP32S3 LoRa V3 SX1262 with OLED

#define LORASCK 9
#define LORAMISO 11
#define LORAMOSI 10

#define LORANSS 8
#define LORANRESET 12
#define LORABUSY 13
#define LORADIO1 14

#define LORA_DEVICE DEVICE_SX1262               //SX126XLT library needs definition of the LoRa device we are using

#define RSTOLED 21
#define SDA 17
#define SCL 18

#define LED1 35
#define LED1on HIGH
#define LED1off LOW

#define Vext 36
#define VBATREAD 1
#define USERSW 0
#define ADCCTRL 37
#define BUZZER 26


/*
  pins on edge connector

  7,6,5,4,3,2,1,38,39,40,41,42,45,46,37  44,43,RST,0,36,35,34,33,47,48,26,21,20,19


  So 'unused' pins are
  7,6,5,4,3,2,38,39,40,41,42,45,46  44,43,34,33,47,48,20,19


*/
