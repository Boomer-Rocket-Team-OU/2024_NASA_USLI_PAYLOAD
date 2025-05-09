#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <math.h>

// === Sensors ===
Adafruit_MPL3115A2 baro;                       // I2C0 (pins 18/19)
BNO080 imu;                                    // I2C2 (pins 25/24)

// === Constants ===
#define SAMPLE_INTERVAL 1000                   // 1 second
#define LAUNCH_DURATION 3000                   // 3 seconds for test (change to 240000 for 4 min)
#define BATTERY_PIN A0
#define ADC_REF_VOLTAGE 3.3
#define ADC_RESOLUTION 1023.0
#define VOLTAGE_DIVIDER_RATIO 2.0
#define ACCEL_THRESHOLD 6.0                    // Z-axis threshold in Gs

// === Runtime Variables ===
unsigned long launchTime = 0;
unsigned long lastSampleTime = 0;
float lastAltitudeFT = 0.0;
float baseAltitudeFT = 0.0;
bool logging = false;

// === Final Flight Data ===
float maxVelocity = 0.0;
float apogeeFT = 0.0;
float landingTempF = 0.0;
float landingBatteryV = 0.0;

void setup() {
  Serial.begin(115200);       // USB debug
  Serial1.begin(9600);        // UART to Pi on pins 0 (RX), 1 (TX)
  while (!Serial) delay(10);  // Wait for Serial monitor

  Serial.println("🚀 Test Mode");

  // === Initialize MPL3115A2 on I2C0 ===
  Wire.begin();
  if (!baro.begin(&Wire)) {
    Serial.println("❌ MPL3115A2 not detected.");
    while (1);
  } else {
    Serial.println("✅ MPL3115A2 OK.");
  }

  // === Initialize BNO085 on I2C2 ===
  Wire2.begin();  // SDA2 = pin 25, SCL2 = pin 24
  if (!imu.begin(0x4A, Wire2)) {
    Serial.println("❌ BNO085 not detected on I2C2.");
    while (1);
  } else {
    Serial.println("✅ BNO085 OK.");
  }

  imu.enableLinearAccelerometer(50);  // 50 Hz

  // === Calibrate base altitude ===
  Serial.print("📡 Calibrating launchpad altitude...");
  delay(1000);

  float sum = 0;
  int samples = 10;
  for (int i = 0; i < samples; i++) {
    sum += baro.getAltitude();
    delay(100);
  }
  baseAltitudeFT = (sum / samples) * 3.28084;
  lastAltitudeFT = 0.0;

  Serial.print(" Done. Launchpad = ");
  Serial.print(baseAltitudeFT);
  Serial.println(" ft");

  // === Wait for launch ===
  Serial.println("🕒 Waiting for Z-accel > 2g to begin logging...");
}

void loop() {
  if (!logging && imu.dataAvailable()) {
    float az = imu.getLinAccelZ();      // in m/s²
    float gZ = az / 9.81;               // convert to g

    if (fabs(gZ) > ACCEL_THRESHOLD) {
      logging = true;
      launchTime = millis();
      lastSampleTime = launchTime;
      Serial.println("🚀 Launch detected! Logging started...");
    }
  }

  if (logging && imu.dataAvailable()) {
    unsigned long currentTime = millis();

    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
      // Read sensors
      float rawAltitudeFT = baro.getAltitude() * 3.28084;
      float altitudeFT = rawAltitudeFT - baseAltitudeFT;
      float tempC = baro.getTemperature();
      float tempF = tempC * 9.0 / 5.0 + 32.0;

      float dt = (currentTime - lastSampleTime) / 1000.0;
      float velocityFTS = (altitudeFT - lastAltitudeFT) / dt;
      lastAltitudeFT = altitudeFT;

      int analogVal = analogRead(BATTERY_PIN);
      float rawVoltage = (analogVal / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
      float batteryVoltage = rawVoltage * VOLTAGE_DIVIDER_RATIO;

      // Track peak data
      if (velocityFTS > maxVelocity) maxVelocity = velocityFTS;
      if (altitudeFT > apogeeFT) apogeeFT = altitudeFT;
      landingTempF = tempF;
      landingBatteryV = batteryVoltage;

      lastSampleTime = currentTime;
    }

    // Final UART Summary
    if (currentTime - launchTime >= LAUNCH_DURATION) {
      Serial1.println("✅ Mission complete. Sending summary via UART...");

      Serial1.print("OUPAYLOAD: TEMP ");
      Serial1.print(landingTempF, 1);
      Serial1.print("F APOGEE ");
      Serial1.print(apogeeFT, 0);
      Serial1.print("FT VMAX ");
      Serial1.print(maxVelocity, 1);
      Serial1.print("FT/S BATT ");
      Serial1.print(landingBatteryV, 2);
      Serial1.println("V");

      Serial1.println("🛰️ Data sent to Pi. Freezing...");
      while (1);  // Freeze system after sending
    }
  }
}
