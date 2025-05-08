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
#define ACCEL_THRESHOLD 0.1554 

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
  Serial.begin(9600);  // UART to Pi on pins 0 (RX), 1 (TX)

  // === Initialize MPL3115A2 on I2C0 ===
  Wire.begin();
  if (!baro.begin(&Wire)) {
    while (1);  // Freeze if baro sensor not found
  }

  // === Initialize BNO085 on I2C2 ===
  Wire2.begin();  // SDA2 = pin 25, SCL2 = pin 24
  if (!imu.begin(0x4A, Wire2)) {
    while (1);  // Freeze if IMU not found
  }

  imu.enableLinearAccelerometer(50);  // 50 Hz

  // === Calibrate base altitude ===
  delay(1000);
  float sum = 0;
  int samples = 10;
  for (int i = 0; i < samples; i++) {
    sum += baro.getAltitude();
    delay(100);
  }
  baseAltitudeFT = (sum / samples) * 3.28084;
  lastAltitudeFT = 0.0;

  // === Start logging ===
  logging = true;
  launchTime = millis();
  lastSampleTime = launchTime;
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);   // set the LED on
  delay(1000);                  // wait for a second
  digitalWrite(13, LOW);    // set the LED off
  delay(1000);
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
      Serial.print("OUPAYLOAD: TEMP ");
      Serial.print(landingTempF, 1);
      Serial.print("F APOGEE ");
      Serial.print(apogeeFT, 0);
      Serial.print("FT VMAX ");
      Serial.print(maxVelocity, 1);
      Serial.print("FT/S BATT ");
      Serial.print(landingBatteryV, 2);
      Serial.print("V");
    }
  }
}
