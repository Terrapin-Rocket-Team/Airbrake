/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "vector_nav.h"

bfs::Vn100 vn(&SPI, 10);

void setup() {
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13);
  Serial.begin(115200);
  while(!Serial) {}
  /* Initialize communication */
  SPI.begin();

  // SPI by ourselfs
  // SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));
  // pinMode(10, OUTPUT);
  // digitalWrite(10, HIGH);
  // delay(1);
  // digitalWrite(10, LOW);
  // SPI.transfer(0x01);
  // SPI.transfer(0x05);
  // SPI.transfer(0x00);
  // SPI.transfer(0x00);
  // digitalWrite(10, HIGH);
  // delayMicroseconds(150);
  // digitalWrite(10, LOW);
  // Serial.println("header");
  // Serial.println(SPI.transfer(0x00));
  // Serial.println(SPI.transfer(0x00));
  // Serial.println(SPI.transfer(0x00));
  // Serial.println(SPI.transfer(0x00));
  // Serial.println("data");
  // uint8_t message[11][4] = {0};
  // for(int i=0; i<11; i++){
  //   for(int ii=0; ii<4; ii++){
  //     message[i][ii] += SPI.transfer(0x00);
  //   }
  // }
  // digitalWrite(10, HIGH);

  // for(int ii=0; ii<11; ii++){
  //    Serial.println((float)((message[ii][0]>>4)*16 + (message[ii][0] & 0x0F)*1 + (message[ii][1]>>4)*16*16*16 + (message[ii][1] & 0x0F)*16*16 + (message[ii][2]>>4)*16*16*16*16*16 + (message[ii][2] & 0x0F)*16*16*16*16 + (message[ii][3]>>4)*16*16*16*16*16*16*16 + (message[ii][3] & 0x0F)*16*16*16*16*16*16));
  // }
  
 if (!vn.Begin()) {
   Serial.print("Error Code: ");
   Serial.println(vn.error_code());
   while (1) {}
 }
}

void loop() {
  /* Read sensor and print values */
 if (vn.Read()) {
   Serial.print(vn.yaw_rad() * 180 / 3.14159);
   Serial.print("\t");
   Serial.print(vn.pitch_rad() * 180 / 3.14159);
   Serial.print("\t");
   Serial.println(vn.roll_rad() * 180 / 3.14159);
 }
 delay(50);
}
