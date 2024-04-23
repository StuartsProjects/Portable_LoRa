/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 23/04/24

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  For Heltec V3 ESP32S3 SX1262

  Program Operation - This is a program that demonstrates the setup of a LoRa test transmitter.

  A packet containing ASCII text defined in buff[] is sent according to the frequency and LoRa settings
  specified in the 'Settings.h' file. The pins to access the LoRa device need to be defined in the
  'Settings.h' file also.

  The details of the packet sent and any transmission errors are shown on the Arduino IDE Serial Monitor,
  together with the transmit power used, the packet length and the CRC of the packet. The matching receive
  program, '2_LoRa_Receiver' can be used to check the packets are being sent correctly. The frequency and
  LoRa settings (in Settings.h) must be the same for the transmitter and receiver programs.

  When the LoRa settings are displayed on the OLED, if the pin 38 user switch is pressed the buzzer will
  be active.

  Serial monitor baud rate is set at 115200
*******************************************************************************************************/
#include <Wire.h>                                  //For OLED  display
#include <SPI.h>                                   //the LoRa device is SPI based so load the SPI library
#include <SX126XLT.h>                              //get library here > https://github.com/StuartsProjects/SX12XX-LoRa  
SX126XLT LoRa;                                     //create a library class instance called LoRa

#include "Heltec_V3_ESP32S3_SX1262_Pins.h"         //pin definitions for  Heltec V3 ESP32S3 SX1262
#include "Settings.h"                              //LoRa settings, frequencies, program settings etc 

#include <U8g2lib.h>                               //get library here > https://github.com/olikraus/U8g2_Arduino
U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, U8X8_PIN_NONE);


uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;
uint8_t buff[] = "Hello!";
bool ENABLEBUZZER = true;


void loop()
{
  Serial.print(TXpower);                           //print the transmit power defined
  Serial.print(F("dBm,"));
  Serial.print(F("Packet,"));
  Serial.flush();

  TXPacketL = sizeof(buff);                        //set TXPacketL to length of array
  LoRa.printASCIIPacket(buff, TXPacketL);          //print the buffer (the sent packet) as ASCII

  digitalWrite(LED1, LED1off);

  if (ENABLEBUZZER)
  {
    buzzer_beep(500, 10, 0, 1);
  }

  startmS =  millis();                                          //start transmit timer
  if (LoRa.transmit(buff, TXPacketL, 10000, TXpower, WAIT_TX))  //will return packet length sent if OK, otherwise 0 if transmit error
  {
    endmS = millis();                                           //packet sent, note end time
    TXPacketCount++;
    packet_is_OK();
  }
  else
  {
    packet_is_Error();                                          //transmit packet returned 0, there was an error
  }

  digitalWrite(LED1, LED1on);
  Serial.println();
  delay(packet_delay);                                          //have a delay between packets
}


void packet_is_OK()
{
  //if here packet has been sent OK
  uint16_t localCRC;

  Serial.print(F(",Bytes,"));
  Serial.print(TXPacketL);                                      //print transmitted packet length
  localCRC = LoRa.CRCCCITT(buff, TXPacketL, 0xFFFF);
  Serial.print(F(",CRC,"));
  Serial.print(localCRC, HEX);                                  //print CRC of transmitted packet
  Serial.print(F(",AirTime,"));
  Serial.print(endmS - startmS);                                //print transmit time of packet
  Serial.print(F("mS"));
  Serial.print(F(",Sent,"));
  Serial.print(TXPacketCount);                                  //print total of packets sent OK
  dispscreen1();
}


void packet_is_Error()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LoRa.readIrqStatus();                             //read the the interrupt register
  Serial.print(F(",SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                                      //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                                 //print IRQ status
}


void initDisplay()
{
  pinMode(RSTOLED, OUTPUT);
  digitalWrite(RSTOLED, HIGH);
  disp.begin();
  disp.clearDisplay();
  disp.setFont(u8g2_font_t0_11b_mf);
  disp.setCursor(0, 11);
  disp.print(F("Check LoRa"));
  disp.sendBuffer();
}


void dispscreen1()
{
  disp.clearDisplay();
  disp.setFont(u8g2_font_10x20_mf);
  disp.setCursor(0, 15);
  disp.print(F("Sent "));
  disp.print(TXPacketCount);
  disp.sendBuffer();
}


void dispscreen2()
{
  disp.clear();
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
  disp.setCursor(0, 63);
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


void dispscreen6()
{
  disp.clearDisplay();
  disp.setFont(u8g2_font_10x20_mf);
  disp.setCursor(0, 15);
  disp.print(F("BUZZER ON"));
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


void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}


void VextOFF(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}


void setup()
{
  pinMode(LED1, OUTPUT);                        //setup pin as output for indicator LED
  led_Flash(8, 125);                            //LED flashes for two seconds to indicate program start

  VextON();
  delay(100);

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("1_LoRa_Transmitter starting"));
  Serial.println(F("For Heltec V3 ESP32S3 SX1262"));
  Serial.println();

  Wire.begin(SDA, SCL);
  initDisplay();

  SPI.begin(LORASCK, LORAMISO, LORAMOSI);

  //now setup hardware pins used by LoRa device, then check if device is found
  if (LoRa.begin(LORANSS, LORANRESET, LORABUSY, LORADIO1, LORA_DEVICE))
  {
    disp.setCursor(0, 23);
    disp.print(F("LoRa OK"));
    disp.sendBuffer();
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);
  }
  else
  {
    disp.setCursor(0, 23);
    disp.print(F("No LoRa"));
    disp.sendBuffer();
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                       //long fast speed LED flash indicates device error
    }
  }

  //this function call sets up the device for LoRa using the settings from Settings.h
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
  //LoRa.setHighSensitivity();                       //set for maximum gain
  //LoRa.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
  //***************************************************************************************************

  Serial.println();
  LoRa.printModemSettings();             //reads and prints the configured LoRa settings
  Serial.println();
  LoRa.printOperatingSettings();         //reads and prints the configured operating settings
  Serial.println();
  Serial.println();

  dispscreen2();
  delay(2000);

  if (BUZZER)
  {
    pinMode(BUZZER, OUTPUT);
    ENABLEBUZZER = true;
    buzzer_beep(500, 100, 100, 2);

    pinMode(USERSW, INPUT_PULLUP);

    if (digitalRead(USERSW))
    {
      ENABLEBUZZER = false;
      Serial.println(F("BUZZER disabled"));
      dispscreen9();
      delay(1500);
    }
    else
    {
      ENABLEBUZZER = true;
      Serial.println(F("BUZZER enabled"));
      dispscreen6();
      delay(1500);
    }
  }


  dispscreen4();
  Serial.print(F("Transmitter ready"));
  Serial.println();
  delay(1000);
}
