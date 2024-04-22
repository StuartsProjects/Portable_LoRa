/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 16/04/24

*******************************************************************************************************/

//These are the pin definitions for the TTGO TBEAM V1.2 with SX1262

#define LORAMOSI 27
#define LORASCK 5
#define LORAMISO 19

#define LORANSS 18
#define LORANRESET 23
#define LORADIO0 26
#define LORADIO1 33
#define LORABUSY 32
#define LORA_DEVICE DEVICE_SX1262        //SX126XLT library needs definition of the LoRa device we are using

#define SDSCK 25                         //you can fit an SD card to the T-Beam 
#define SDMOSI 14
#define SDMISO 13
#define SDCS 33

#define SDA 21
#define SCL 22

#define LED1 4
#define LED1on LOW
#define LED1off HIGH

#define BUZZER 15                         //set to false if not used

#define BOOT 0
#define USERSW 38

#define GPSRX 34                          //data from GPS received here
#define GPSTX 12                          //send data to GPS here

#define PMU_IRQ 35


/*
  pins on edge connector

  0,1,2,3,4,13,14,15,21,22,23,25,32,33,35,36,39

  35,36,39 are input only and need pullups

  So 'unused' pins are 2,(13),(14),(15),(25),36,39  //in brackets () indicates optional use

  Buzzer to GND,15

*/
