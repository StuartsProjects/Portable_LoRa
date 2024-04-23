/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 23/04/24

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  For LilyGo T3 V1.6.1 ESP32 SX1278

  Program Operation - The packet logger program listens for incoming packets using the LoRa settings in
  the 'Settings.h' file. The pins to access the LoRa device need to be defined in the 'Settings.h' file.

  There is a printout to serial monitor of the packet reception details and HEX values of a valid packet
  received. The LED will flash for each packet received and the buzzer will sound, if fitted.

  At power up the program checks for an I2C connected PCF8574 which if found is used to drive an 20x4 LCD
  connected to a PCF8574 I2C backpack. The transflective type of this doisplay has high visibility even
  in direct sunlight. If a PCF8574 is not found then the 128x64 OLED will be used.

  When the LoRa settings are displayed on the OLED, if the pin 38 user switch is pressed and held down
  until a 'BUZZER ON' message appears on the OLED the buzzer will be actived and beep at each packet
  reception.

  The display shows the number of packets received OK and the RSSI and SNR of the last received packet.
  At startup the display will show the set frequency, LoRa spreading factor and LoRa bandwidth in use.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <Wire.h>                                  //For display
#include <SPI.h>                                   //the LoRa device is SPI based so load the SPI library
#include <SX127XLT.h>                              //get library here > https://github.com/StuartsProjects/SX12XX-LoRa  
SX127XLT LoRa;                                     //create a library class instance called LoRa

#include "LilyGo_T3_V1_6_1_ESP32_SX1278_Pins.h"    //pin definitions for LilyGo T3 V1.6.1 ESP32 SX1278
#include "Settings.h"                              //LoRa settings, frequencies, program settings etc  

#include <U8g2lib.h>                               //get library here > https://github.com/olikraus/U8g2_Arduino
U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, U8X8_PIN_NONE);

uint32_t RXpacketCount;
uint32_t RXpacketErrors;
uint16_t IRQStatus;

uint8_t RXBUFFER[RXBUFFER_SIZE];                   //create the buffer that received packets are copied into
uint8_t RXPacketL;                                 //stores length of packet received
int16_t PacketRSSI;                                //stores RSSI of received packet
int8_t  PacketSNR;                                 //stores signal to noise ratio of received packet
uint16_t RXtimeout;                                //stores count of RX timeouts
bool ENABLEBUZZER = true;


void loop()
{
  RXPacketL = LoRa.receive(RXBUFFER, RXBUFFER_SIZE, RXtimeoutmS, WAIT_RX);

  digitalWrite(LED1, LED1off);                     //something has happened, so LED off

  if (ENABLEBUZZER)
  {
    buzzer_beep(500, 10, 0, 1);
  }

  PacketRSSI = LoRa.readPacketRSSI();              //read the recived RSSI value
  PacketSNR = LoRa.readPacketSNR();                //read the received SNR value
  IRQStatus = LoRa.readIrqStatus();                //read the LoRa device IRQ status register

  printElapsedTime();                              //print seconds to monitor

  if (RXPacketL == 0)                              //if the LoRa.receive() function detects an error, RXpacketL == 0
  {
    packet_is_Error();
  }
  else
  {
    packet_is_OK();
  }

  digitalWrite(LED1, LED1on);                      //LED on
}


void packet_is_OK()
{
  uint8_t index;

  RXpacketCount++;

#ifdef DISPLAYASCII
  Serial.print(F(","));
  LoRa.printASCIIPacket(RXBUFFER, RXPacketL);
#endif

  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(RXpacketErrors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);

#ifdef DISPLAYHEX
  {
    Serial.print(F("  "));
    printHEXPacket(RXBUFFER, RXPacketL);              //print the HEX values of packet
  }
#endif

  Serial.println();

  dispscreen1();

}


void printHEXPacket(uint8_t *buffer, uint8_t size)
{
  uint8_t index;

  for (index = 0; index < size; index++)
  {
    printHEXByte(buffer[index]);
    Serial.print(F(" "));
  }
}


void printHEXByte(uint8_t temp)
{
  if (temp < 0x10)
  {
    Serial.print(F("0"));
  }
  Serial.print(temp, HEX);
}


void packet_is_Error()
{
  if (IRQStatus & IRQ_RX_TIMEOUT)
  {
    RXtimeout++;
    Serial.print(F(" RXTimeout"));
    Serial.println(RXtimeout);
    dispscreen5();
    return;
  }

  RXpacketErrors++;
  Serial.print(F(" PacketError "));
  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(LoRa.readRXPacketL());               //get the real packet length
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(RXpacketErrors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  LoRa.printIrqStatus();                            //print the names of the IRQ registers set
  Serial.println();
}


void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}


void displayElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  disp.print(seconds, 0);
  disp.print(F("s"));
}


void dispscreen1()
{
  disp.clearDisplay();
  disp.setFont(u8g2_font_10x20_mf);
  disp.setCursor(0, 15);
  disp.print(F("RSSI "));
  disp.print(PacketRSSI);
  disp.print(F("dBm"));
  disp.setCursor(0, 31);
  disp.print(F("SNR  "));
  if (PacketSNR > 0)
  {
    disp.print(F("+"));
  }
  disp.print(PacketSNR);
  disp.setCursor(0, 47);
  disp.print(RXpacketCount);
  disp.print(F(" Pkts"));
  disp.setCursor(3, 63);
  //disp.setFont(u8g2_font_t0_11b_mf);
  disp.sendBuffer();
}


void dispscreen2()
{
  disp.setFont(u8g2_font_t0_11b_mf);
  disp.setCursor(0, 11);
  disp.print(Frequency);
  disp.print(F("Hz"));
  disp.setCursor(0, 23);
  disp.print(F("BW "));
  disp.print(LoRa.returnBandwidth(Bandwidth));
  disp.print(F("Hz"));
  disp.setCursor(0, 35);
  disp.print(F("SF "));
  disp.print(SpreadingFactor);
  disp.sendBuffer();
}


void dispscreen4()
{
  disp.clearDisplay();
  disp.setFont(u8g2_font_10x20_mf);
  disp.setCursor(0, 15);
  disp.print(F("Ready"));
  disp.sendBuffer();
}


void dispscreen5()
{
  disp.clearDisplay();
  disp.setFont(u8g2_font_10x20_mf);
  disp.setCursor(0, 15);
  displayElapsedTime();
  disp.print(F(" Timeout"));
  disp.sendBuffer();
}

void dispscreen6()
{
  disp.clearDisplay();
  disp.setFont(u8g2_font_10x20_mf);
  disp.setCursor(0, 15);
  disp.print(F("BUZZER ON"));
  disp.sendBuffer();
}


void dispscreen7()
{
  disp.setCursor(0, 23);
  disp.print(F("LoRa OK"));
  disp.sendBuffer();
}


void dispscreen8()
{
  disp.setCursor(0, 23);
  disp.print(F("No LoRa Device"));
  disp.sendBuffer();
}


void dispscreen9()
{
  disp.clearDisplay();
  disp.setFont(u8g2_font_10x20_mf);
  disp.setCursor(0, 15);
  disp.print(F("BUZZER OFF"));
  disp.sendBuffer();
}


void initDisplay()
{
  disp.begin();
  disp.clearDisplay();
  disp.setFont(u8g2_font_t0_11b_mf);
  disp.setCursor(0, 11);
  disp.print(F("Check LoRa"));
  disp.sendBuffer();
}


void buzzer_beep(uint16_t hertz, uint16_t onmS, uint16_t offmS, uint16_t times)
{
  uint16_t index, index2, times2;
  uint32_t hertzuS;
  uint32_t startmS;

  hertzuS = 500000 / hertz;
  times2 = (onmS * 500) / hertzuS;
  startmS = millis();

  for (index = 1; index <= times; index++)
  {
    for (index2 = 1; index2 <= times2; index2++)
    {
      digitalWrite(BUZZER, HIGH);
      delayMicroseconds(hertzuS);
      digitalWrite(BUZZER, LOW);
      delayMicroseconds(hertzuS);
    }
    delay(offmS);
  }
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, LED1off);
    delay(delaymS);
    digitalWrite(LED1, LED1on);                 //leave LED on so it acts as power indicator
    delay(delaymS);
  }
}


void setup()
{
  pinMode(LED1, OUTPUT);                    //setup pin as output for indicator LED
  led_Flash(8, 125);                        //LED flashes for two seconds to indicate program start

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("2_LoRa_Receiver starting"));
  Serial.println(F("For LilyGo T3 V1.6.1 ESP32 SX1278"));
  Serial.println();

  Wire.begin(SDA, SCL);
  initDisplay();

  SPI.begin(LORASCK, LORAMISO, LORAMOSI);

  //setup hardware pins used by LoRa device, then check if device is found
  if (LoRa.begin(LORANSS, LORANRESET, LORADIO0, LORA_DEVICE))
  {
    dispscreen7;
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);
  }
  else
  {
    dispscreen8;
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                       //long fast speed LED flash indicates device error
    }
  }

  //this function call sets up the device for LoRa using the settings from settings.h
  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  //*******************************************************************************************************
  //Detail of the full LoRa device setup. You can use this as an alternative to the simple LoRa.setupLoRa()
  //*******************************************************************************************************
  //LoRa.setMode(MODE_STDBY_RC);
  //LoRa.setRegulatorMode(USE_DCDC);
  //LoRa.setPaConfig(0x04, PAAUTO, LORA_DEVICE);
  //LoRa.setDIO3AsTCXOCtrl(TCXO_CTRL_3_3V);
  //LoRa.calibrateDevice(ALLDevices);                //is required after setting TCXO
  //LoRa.calibrateImage(Frequency);
  //LoRa.setDIO2AsRfSwitchCtrl();
  //LoRa.setPacketType(PACKET_TYPE_LORA);
  //LoRa.setRfFrequency(Frequency, Offset);
  //LoRa.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, Optimisation);
  //LoRa.setBufferBaseAddress(0, 0);
  //LoRa.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
  //LoRa.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  //LoRa.setHighSensitivity();  //set for maximum gain
  //LoRa.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
  //***************************************************************************************************

  Serial.println();
  LoRa.printModemSettings();                                   //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LoRa.printOperatingSettings();                               //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();

  dispscreen2();

  delay(2000);

  if (BUZZER)
  {
    ENABLEBUZZER = true;
    Serial.println(F("BUZZER enabled"));
    pinMode(BUZZER, OUTPUT);
    buzzer_beep(500, 100, 100, 2);
    dispscreen6();
    delay(1000);
  }
  else
  {
    ENABLEBUZZER = false;
    Serial.println(F("BUZZER disabled"));
  }

  dispscreen4();

  Serial.print(F("Receiver ready"));
  Serial.println();
}
