/* FILE:    RC522_RFID_Module_Example
   DATE:    23/03/14
   VERSION: 0.2 Spark

REVISIONS:

23/03/14    Version 0.2, modified by Paul Kourany to run on Spark Core
            with added support for Software SPI
24/07/13    Created version 0.1


This is an example of how to use the RC522 RFID module. The module allows reading
and writing to various types of RFID devices and can be found in our MFRC-522 
(HCMODU0016) and Ultimate RFID (HCARDU0068) kits. This example Arduino sketch uses
the RFID library written by Miguel Balboa to read the pre-programmed serial number 
from RFID cards and tags supplied with our RFID kits. Snapshots and links to the 
library are available on our support forum.


PINOUT:

RC522 MODULE    SPARK HARD SPI  SPARK SOFT SPI
SS                  A2              ANY
SCK                 A3              ANY
MOSI                A5              ANY
MISO                A4              ANY
IRQ                 N/A             N/A
GND                 GND             GND
RST                 D9              ANY
3.3V                3.3V            3.3V


You may copy, alter and reuse this code in any way you like, but please leave
reference to HobbyComponents.com in your comments if you redistribute this code.
This software may not be used directly for the purpose of selling products that
directly compete with Hobby Components Ltd's own range of products.

THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, WHETHER
EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR LACK OF NEGLIGENCE.
HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR ANY DAMAGES,
INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY
REASON WHATSOEVER.
*/


/* Include the RFID library */
/* SEE RFID.h for selecting Hardware or Software SPI modes */
#include "RFID.h"

/* Define the pins used for the SS (SDA) and RST (reset) pins for BOTH hardware and software SPI */
/* Change as required */
#define SS_PIN      A2      // Same pin used as hardware SPI (SS)
#define RST_PIN     D2

/* Define the pins used for the DATA OUT (MOSI), DATA IN (MISO) and CLOCK (SCK) pins for SOFTWARE SPI ONLY */
/* Change as required and may be same as hardware SPI as listed in comments */
#define MOSI_PIN    D3      // hardware SPI: A5
#define MISO_PIN    D4      //     "     " : A4
#define SCK_PIN     D5      //     "     " : A3

/* Create an instance of the RFID library */
#if defined(_USE_SOFT_SPI)
    RFID(int chipSelectPin, int NRSTPD, uint8_t mosiPin, uint8_t misoPin, uint8_t clockPin);    // Software SPI
#else
    RFID RC522(SS_PIN, RST_PIN);    // Hardware SPI
#endif


void setup()
{ 
  Serial.begin(9600);
  
  /* Enable the SPI interface */
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.begin();
  
  /* Initialise the RFID reader */
  RC522.init();
}

void loop()
{
  /* Temporary loop counter */
  uint8_t i;

  /* Has a card been detected? */
  if (RC522.isCard())
  {
    /* If so then get its serial number */
    RC522.readCardSerial();

    Serial.println("Card detected:");

    /* Output the serial number to the UART */
    for(i = 0; i <= 4; i++)
    {
      Serial.print(RC522.serNum[i],HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  else
      Serial.println("Card NOT detected:");
      
  delay(1000);
}
