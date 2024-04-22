/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 16/04/24

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//LoRa Modem Parameters

const uint32_t Frequency = 434000000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_500;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

const uint32_t RXtimeoutmS = 60000;             //RXtimeout in mS
const uint8_t RXBUFFER_SIZE = 255;              //RX buffer size

#define DISPLAYASCII                            //enable define to display received packet in ASCII on Serial monitor
//#define DISPLAYHEX                            //enable define to display received packet in HEX on Serial monitor
