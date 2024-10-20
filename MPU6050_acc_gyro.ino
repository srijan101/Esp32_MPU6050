#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // Initialize I2C on ESP32 pins

  mpu.initialize();  // Initialize the MPU6050 sensor

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}

void loop() {
  // Read accelerometer data
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Read gyroscope data
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Read temperature data
  int16_t tempRaw = mpu.getTemperature();
  float tempC = tempRaw / 340.0 + 36.53;  // Convert raw data to Celsius

  // Print the values in a format for Serial Plotter
  // Comma-separated: AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, TempC
  Serial.print(ax / 16384.0);  // Acceleration X (G)
  Serial.print(", ");
  Serial.print(ay / 16384.0);  // Acceleration Y (G)
  Serial.print(", ");
  Serial.print(az / 16384.0);  // Acceleration Z (G)
  Serial.print(", ");
  Serial.print(gx / 131.0);    // Gyro X (°/s)
  Serial.print(", ");
  Serial.print(gy / 131.0);    // Gyro Y (°/s)
  Serial.print(", ");
  Serial.print(gz / 131.0);    // Gyro Z (°/s)
  Serial.print(", ");
  Serial.println(tempC);       // Temperature in Celsius

  delay(100);  // Adjust for smoother plotting
}
