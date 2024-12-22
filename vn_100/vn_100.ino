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

 vn.SetAccelFilter(bfs::Vn100::FILTER_COMP_ONLY, 0);
}

float theta_x = 0;
float theta_y = 0;
float theta_z = 0;

void loop() {
  /* Read sensor and print values */
 if (vn.Read()) {
   Serial.print(vn.delta_time()); Serial.print(",\t");
   Serial.print(vn.delta_velocity_x()); Serial.print(",\t");
   Serial.print(vn.delta_velocity_y()); Serial.print(",\t");
   Serial.print(vn.delta_velocity_z()); Serial.print(",\t");
   Serial.print(vn.delta_theta_x()); Serial.print(",\t");
   Serial.print(vn.delta_theta_y()); Serial.print(",\t");
   Serial.print(vn.delta_theta_z()); Serial.print(",\t");
   Serial.print(theta_x); Serial.print(",\t");
   Serial.print(theta_y); Serial.print(",\t");
   Serial.println(theta_z);
 }
 delay(50);
}
