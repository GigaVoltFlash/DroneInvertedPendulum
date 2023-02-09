#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "IMU.h"
#include <math.h>

#define BNO08X_RESET -1

// Adafruit_BNO08x bno08x(BNO08X_RESET);

Adafruit_BNO08x bno(BNO08X_RESET);
IMU imu;
euler_t euler;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);
  // Try to initialize!
  if (!bno.begin_I2C()) {
      // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300
      // byte UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
      Serial.println("Failed to find BNO08x chip");
      while (1) {
          delay(10);
      }
  }
  imu = IMU(&bno);
  // imu.setIMU(&bno);


}

void loop() {
  // put your main code here, to run repeatedly:
  imu.readData();
  euler = imu.getEuler();
  // float l = 62.0; // cm
  // float r_disp = l*(sin(euler.pitch));
  // float s_disp = -l*(sin(euler.roll)*cos(euler.pitch));
  Serial.print("Yaw: ");
  Serial.print(euler.yaw);
  Serial.print("  |||  Pitch: ");
  Serial.print(euler.pitch);
  Serial.print("  |||  Roll: ");
  Serial.println(euler.roll);
  // Serial.print("  |||  Displacement in World X: ");
  // Serial.print(r_disp);
  // Serial.print("  |||  Displacement in World Y: ");
  // Serial.println(s_disp);
  delay(100);
}