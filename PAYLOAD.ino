#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <math.h>

// === Sensors ===
Adafruit_MPL3115A2 baro;                       // I2C0 (pins 18/19)
BNO080 imu;                                    // I2C1 (pins 37/38)

// === Constants ===
#define SAMPLE_INTERVAL 1000                   // 1 second
#define LAUNCH_DURATION 240000                 // 4 minutes
#define BATTERY_PIN A0
#define ADC_REF_VOLTAGE 3.3
#define ADC_RESOLUTION 1023.0
#define VOLTAGE_DIVIDER_RATIO 2.0

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
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("🚀 Test Mode");

  // Initialize MPL3115A2
  Wire.begin();
  if (!baro.begin(&Wire)) {
    Serial.println("❌ MPL3115A2 not detected.");
    while (1);
  } else {
    Serial.println("✅ MPL3115A2 OK.");
  }

  // Initialize BNO085
  Wire1.begin();
  if (!imu.begin(0x4A, Wire1)) {
    Serial.println("❌ BNO085 not detected on I2C1.");
    while (1);
  } else {
    Serial.println("✅ BNO085 OK.");
  }

  imu.enableLinearAccelerometer(50);  // 50 Hz

  // Calibrate base altitude
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

  // Start logging
  logging = true;
  launchTime = millis();
  lastSampleTime = launchTime;
  Serial.println("🚀 Logging started...");
}

void loop() {
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

    // Final APRS-style summary
    if (currentTime - launchTime >= LAUNCH_DURATION) {
      Serial.println("✅ Mission complete. Sending summary...");

      Serial.print("OUPAYLOAD: TEMP ");
      Serial.print(landingTempF, 1);
      Serial.print("F APOGEE ");
      Serial.print(apogeeFT, 0);
      Serial.print("FT VMAX ");
      Serial.print(maxVelocity, 1);
      Serial.print("FT/S BATT ");
      Serial.print(landingBatteryV, 2);
      Serial.println("V");

      while (1);  // Freeze system after sending
    }
  }
}
