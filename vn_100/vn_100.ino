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
