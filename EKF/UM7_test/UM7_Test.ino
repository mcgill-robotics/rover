#include <UM7.h>
#include <SoftwareSerial.h>

UM7 imu;
SoftwareSerial mySerial(4, 5); // alternate serial port (can't use 0 and 1); RX := 8, TX := 9

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  
}

void loop() {
  if (mySerial.available() > 0) {
    if (imu.encode(mySerial.read())) {  // Reads byte from buffer.  Valid packet returns true.
        Serial.print(imu.roll);
        Serial.print(",");
        Serial.print(imu.pitch);
        Serial.print(",");
        Serial.println(imu.yaw);
    }
  }
}