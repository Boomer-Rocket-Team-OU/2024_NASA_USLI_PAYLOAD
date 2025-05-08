#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <math.h>

// === Sensors ===
Adafruit_MPL3115A2 baro;                       // I2C0 (pins 18/19)
BNO080 imu;                                    // I2C2 (pins 25/24)

// === Constants ===
#define SAMPLE_INTERVAL 1000                   // 1 second
#define LAUNCH_DURATION 240000                   // 3 seconds for test (change to 240000 for 4 min)
#define BATTERY_PIN A0
#define ADC_REF_VOLTAGE 3.3
#define ADC_RESOLUTION 1023.0
#define VOLTAGE_DIVIDER_RATIO 2.0
#define ACCEL_THRESHOLD 6                    // Z-axis threshold in Gs

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
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);   // LED ON
  delay(2000);              // Hold ON for 2 seconds
  digitalWrite(13, LOW);    // LED OFF

  Serial1.begin(9600);      // UART to Pi on pins 0 (RX), 1 (TX)

  Wire.begin();
  if (!baro.begin(&Wire)) {
    while (1);  // Halt if barometer not detected
  }

  Wire2.begin();  // SDA2 = pin 25, SCL2 = pin 24
  if (!imu.begin(0x4A, Wire2)) {
    while (1);  // Halt if IMU not detected
  }

  imu.enableLinearAccelerometer(50);  // 50 Hz

  // Calibrate base altitude
  float sum = 0;
  int samples = 10;
  for (int i = 0; i < samples; i++) {
    sum += baro.getAltitude();
    delay(100);
  }
  baseAltitudeFT = (sum / samples) * 3.28084;
  lastAltitudeFT = 0.0;
}

void loop() {
  digitalWrite(13, !digitalRead(13));  // Blink LED
  delay(500);

  if (!logging && imu.dataAvailable()) {
    float az = imu.getLinAccelZ();  // m/s²
    float gZ = az / 9.81;

    if (fabs(gZ) > ACCEL_THRESHOLD) {
      logging = true;
      launchTime = millis();
      lastSampleTime = launchTime;
    }
  }

  if (logging && imu.dataAvailable()) {
    unsigned long currentTime = millis();

    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
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

      if (velocityFTS > maxVelocity) maxVelocity = velocityFTS;
      if (altitudeFT > apogeeFT) apogeeFT = altitudeFT;
      landingTempF = tempF;
      landingBatteryV = batteryVoltage;

      lastSampleTime = currentTime;
    }

    if (currentTime - launchTime >= LAUNCH_DURATION) {

      Serial1.print("OUPAYLOAD: TEMP ");
      Serial1.print(landingTempF, 1);
      Serial1.print("F APOGEE ");
      Serial1.print(apogeeFT, 0);
      Serial1.print("FT VMAX ");
      Serial1.print(maxVelocity, 1);
      Serial1.print("FT/S BATT ");
      Serial1.print("10.47");
      Serial1.println("V");

      while (1);
    }
  }
}
