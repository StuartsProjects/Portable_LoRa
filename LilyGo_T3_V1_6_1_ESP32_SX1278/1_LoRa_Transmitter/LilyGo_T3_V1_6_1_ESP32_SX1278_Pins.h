/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 23/04/24

*******************************************************************************************************/

//These are the pin definitions for the LilyGo T3 V1.6.1 ESP32 SX1278


#define LORANSS 18                                  //select pin on LoRa device
#define LORANRESET 23                               //reset pin on LoRa device
#define LORADIO0 26                                 //DIO0 pin on LoRa device, used for RX and TX done 
#define LORAMOSI 27
#define LORAMISO 19
#define LORASCK  5
#define LORA_DEVICE DEVICE_SX1278                   //the LoRa device used

#define LED1 25
#define LED1on HIGH
#define LED1off LOW

#define BUZZER 4                                    //set to false if not used  

#define SDSCK 14
#define SDMOSI 15
#define SDMISO 2
#define SDCS 13

#define SDA 21
#define SCL 22


/* Pin connections on board edges

  S_VP
  S_VN
  RST
  IO34 Input only, needs pullup
  IO35 Input only, needs pullup
  IO14 SDSCK
  IO12 leave floating during programming and reset
  IO13 SDCS
  IO15 SDMOSI
  IO2 SDMISO
  IO0 Held low during programming
  IO4
  IO25 LED1
  5V
  GND
  3.3V
  GND
  IO26 LORADIO0
  LoRa1
  LoRa2
  IO19
  IO23 LORANRESET
  IO22 OLEDSCL
  RXD
  TXD
  IO21 OLEDSDA

  Free pins, 4,12,34,35
*/
