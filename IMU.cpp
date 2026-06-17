#include <Arduino.h>
#include <Wire.h>

#define IMU_ADDR 0x6B

// Registers
#define WHO_AM_I  0x0F
#define CTRL1_XL  0x10
#define CTRL2_G   0x11
#define CTRL3_C   0x12
#define OUTX_L_G  0x22
#define OUTX_L_XL 0x28

// Sensor Scaling Constants
const float GYRO_SCALE = 114.28;   // For ±250 dps (8.75 mdps/LSB)
const float ACCEL_SCALE = 16384.0; // For ±2g range

// Calibration Offsets (Calculated during startup)
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0; // Z bias is omitted to preserve the gravity vector

// Filtered Accelerometer Values (For motor vibration suppression)
float ax_filtered = 0.0;
float ay_filtered = 0.0;
float az_filtered = 1.0;
const float LPF_BETA = 0.15; // Low-Pass Filter factor (0.05 to 0.20 is standard for drones)

// Attitude Outputs
float roll = 0.0, pitch = 0.0, yaw = 0.0; // Added yaw

// Loop Timing Control
unsigned long lastLoopTime = 0;
const unsigned long LOOP_PERIOD_US = 10000; // 10,000 microseconds = 10ms (100 Hz rate)

// Helper function to read multiple bytes safely over I2C
bool readBytes(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false; // I2C Bus Error
  }
  uint8_t bytesReceived = Wire.requestFrom(IMU_ADDR, len);
  if (bytesReceived == len) {
    for (int i = 0; i < len; i++) {
      buffer[i] = Wire.read();
    }
    return true;
  }
  return false;
}

// Helper function to write to a single register
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Automatically calculates static gyro and accel offsets
void calibrateSensors() {
  long gSumX = 0, gSumY = 0, gSumZ = 0;
  long aSumX = 0, aSumY = 0;
  uint8_t data[6];
  int successfulReads = 0;

  // 1. Calibrate Gyroscope
  while (successfulReads < 200) {
    if (readBytes(OUTX_L_G, data, 6)) {
      gSumX += (int16_t)((data[1] << 8) | data[0]);
      gSumY += (int16_t)((data[3] << 8) | data[2]);
      gSumZ += (int16_t)((data[5] << 8) | data[4]);
      successfulReads++;
    }
    delay(5);
  }
  gyroBiasX = gSumX / 200.0;
  gyroBiasY = gSumY / 200.0;
  gyroBiasZ = gSumZ / 200.0;

  // 2. Calibrate Accelerometer (X and Y only for leveling)
  successfulReads = 0;
  while (successfulReads < 200) {
    if (readBytes(OUTX_L_XL, data, 6)) {
      aSumX += (int16_t)((data[1] << 8) | data[0]);
      aSumY += (int16_t)((data[3] << 8) | data[2]);
      successfulReads++;
    }
    delay(5);
  }
  accelBiasX = aSumX / 200.0;
  accelBiasY = aSumY / 200.0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  delay(500);

  // Check connection
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 1);
  if (Wire.available()) {
    byte chipID = Wire.read();
    Serial.print("WHO_AM_I = 0x");
    Serial.println(chipID, HEX);
  } else {
    Serial.println("Error: IMU failed WHO_AM_I check. Program halted.");
    while (1);
  }

  // Configure IMU registers
  writeReg(CTRL1_XL, 0x60); // Accel 104 Hz, ±2g FS
  writeReg(CTRL2_G,  0x60); // Gyro 104 Hz, ±250 dps FS
  writeReg(CTRL3_C,  0x04); // Auto-increment addresses (IF_INC)

  // Wait for the MEMS sensor structure to warm up
  Serial.println("Stabilizing sensor readings...");
  delay(250);
  Serial.println("Calibrating... Keep the drone level and still.");
  calibrateSensors();
  Serial.println("Calibration complete.");

  // Initialize filter starting points with a raw reading
  uint8_t data[6];
  if (readBytes(OUTX_L_XL, data, 6)) {
    int16_t ax = (int16_t)((data[1] << 8) | data[0]);
    int16_t ay = (int16_t)((data[3] << 8) | data[2]);
    int16_t az = (int16_t)((data[5] << 8) | data[4]);
    ax_filtered = (ax - accelBiasX) / ACCEL_SCALE;
    ay_filtered = (ay - accelBiasY) / ACCEL_SCALE;
    az_filtered = az / ACCEL_SCALE;
  }
  lastLoopTime = micros();
}

void loop() {
  unsigned long currentMicros = micros();
  
  // Non-blocking execution loop (highly precise loop frequency)
  if (currentMicros - lastLoopTime >= LOOP_PERIOD_US) {
    float dt = (currentMicros - lastLoopTime) / 1000000.0;
    lastLoopTime = currentMicros;
    uint8_t data[6];

    // Read raw values
    if (readBytes(OUTX_L_XL, data, 6)) {
      int16_t raw_ax = (int16_t)((data[1] << 8) | data[0]);
      int16_t raw_ay = (int16_t)((data[3] << 8) | data[2]);
      int16_t raw_az = (int16_t)((data[5] << 8) | data[4]);

      if (readBytes(OUTX_L_G, data, 6)) {
        int16_t raw_gx = (int16_t)((data[1] << 8) | data[0]);
        int16_t raw_gy = (int16_t)((data[3] << 8) | data[2]);
        int16_t raw_gz = (int16_t)((data[5] << 8) | data[4]);

        // Convert raw gyro data to dps with bias corrections
        float Gx = (raw_gx - gyroBiasX) / GYRO_SCALE;
        float Gy = (raw_gy - gyroBiasY) / GYRO_SCALE;
        float Gz = (raw_gz - gyroBiasZ) / GYRO_SCALE;

        // Convert raw accel data to g-force units with bias corrections
        float Ax_g = (raw_ax - accelBiasX) / ACCEL_SCALE;
        float Ay_g = (raw_ay - accelBiasY) / ACCEL_SCALE;
        float Az_g = raw_az / ACCEL_SCALE;

        // Apply Low-Pass Filter (LPF) to dampen drone vibration
        ax_filtered = (ax_filtered * (1.0 - LPF_BETA)) + (Ax_g * LPF_BETA);
        ay_filtered = (ay_filtered * (1.0 - LPF_BETA)) + (Ay_g * LPF_BETA);
        az_filtered = (az_filtered * (1.0 - LPF_BETA)) + (Az_g * LPF_BETA);

        // Calculate Roll & Pitch angles from filtered accelerometer data
        float accRoll  = atan2(ay_filtered, az_filtered) * 57.2958;
        float accPitch = atan2(-ax_filtered, sqrt(ay_filtered * ay_filtered + az_filtered * az_filtered)) * 57.2958;

        // Complementary Filter for Roll and Pitch
        roll  = 0.98 * (roll + Gx * dt) + 0.02 * accRoll;
        pitch = 0.98 * (pitch + Gy * dt) + 0.02 * accPitch;

        // Integrate Z-axis gyro to compute Relative Yaw
        yaw += Gz * dt;

        // Constrain yaw angle output between -180 and +180 degrees
        if (yaw > 180.0)  yaw -= 360.0;
        if (yaw < -180.0) yaw += 360.0;

        // Output results to serial at ~10 Hz (using non-blocking print timing)
        static unsigned long lastPrintTime = 0;
        if (millis() - lastPrintTime > 100) {
          lastPrintTime = millis();
          Serial.print("Roll: ");   Serial.print(roll, 1);
          Serial.print("\tPitch: "); Serial.print(pitch, 1);
          Serial.print("\tYaw: ");   Serial.print(yaw, 1); // Print Yaw
          Serial.print("\t| Ax: ");  Serial.print(ax_filtered, 2);
          Serial.print("g  Ay: ");   Serial.print(ay_filtered, 2);
          Serial.print("g  Az: ");   Serial.print(az_filtered, 2);
          Serial.print("g\t| GX: ");  Serial.print(Gx, 2);
          Serial.print("  GY: ");   Serial.print(Gy, 2);
          Serial.print("  GZ: ");   Serial.print(Gz, 2);
          Serial.println();
        }
      }
    }
  }
}
