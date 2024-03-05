#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

const char* ssid = "Raminda_rulzz_";
const char* password = "Raminda@123";
const char* firebaseHost = "nodemcu5575-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* firebaseAuth = "HWrrY1F1QjJrkKZkwgQgXaCia56yK6JefnoJ6r0F";

WiFiClientSecure client;

const int sensorPin = D5;          // Speed sensor pin
const uint8_t scl = D6, sda = D7;  // MPU6050 I2C pins
const uint8_t buzzerPin = D2;      // Buzzer pin for acceleration spike

unsigned long previousMillis = 0;
const long interval = 5000;  // 5-second interval for speed calculation
volatile int pulseCount = 0;
bool accelerationSpike = false;

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// MPU6050 configurations
const uint8_t MPU6050_REGISTER_SMPLRT_DIV = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2 = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

double lastAx = 0, lastAy = 0, lastAz = 0;
int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

unsigned long spikeTimestamp = 0;
const long spikeDuration = 5000;  // Duration to keep the spike status as true in milliseconds


void IRAM_ATTR sensorInterrupt() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorInterrupt, FALLING);
  Wire.begin(sda, scl);

  client.setInsecure();
  MPU6050_Init();
}

void loop() {
  unsigned long currentMillis = millis();

  // Read accelerometer and temperature values continuously
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  double Ax = (double)AccelX / AccelScaleFactor;
  double Ay = (double)AccelY / AccelScaleFactor;
  double Az = (double)AccelZ / AccelScaleFactor;
  double T = (double)Temperature / 340 + 36.53;  // Temperature formula

  // Detect acceleration spikes continuously
  bool currentAccelerationSpike = abs(Ax - lastAx) > 1.5 || abs(Ay - lastAy) > 1.5 || abs(Az - lastAz) > 1.5;
  if (currentAccelerationSpike) {
    spikeTimestamp = currentMillis;  // Update the timestamp of the last spike
    digitalWrite(buzzerPin, HIGH);
    delay(1000);  // Adjust duration as needed
    digitalWrite(buzzerPin, LOW);
  }
  if (currentMillis - spikeTimestamp <= spikeDuration) {
    accelerationSpike = true;
  } else {
    accelerationSpike = currentAccelerationSpike;
  }

  // Update the last accelerometer readings
  lastAx = Ax;
  lastAy = Ay;
  lastAz = Az;

  // Perform speed calculation at the specified interval
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    detachInterrupt(sensorPin);

    double circumference = 3.14159265358979323846 * 25;  // Circumference in mm
    double revolutions = pulseCount / 20.0;
    double distance = revolutions * circumference / 1000000;  // Distance in km
    double speed = distance / (5.0 / 3600);                   // Speed in km/h

    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.println(" km/h");

    // Upload the sensor values to Firebase
    uploadSensorValueToFirebase(speed, accelerationSpike, T);

    // Reset pulse count for the next speed calculation
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(sensorPin), sensorInterrupt, FALLING);
  }

  // Optionally, you can print the accelerometer and temperature data outside the speed calculation block for continuous monitoring
  Serial.print("Ax: ");
  Serial.print(Ax);
  Serial.print(" Ay: ");
  Serial.print(Ay);
  Serial.print(" Az: ");
  Serial.print(Az);
  Serial.print(" T: ");
  Serial.println(T);
}

void uploadSensorValueToFirebase(double speed, bool accelerationSpike, double temperature) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Speed
    http.begin(client, String("https://") + firebaseHost + "/blackbox_values/speed.json?auth=" + firebaseAuth);
    http.addHeader("Content-Type", "application/json");
    String payload = "{\"speed\":" + String(speed, 6) + "}";
    int httpResponseCode = http.PUT(payload);
    if (httpResponseCode > 0) {
      Serial.print("Firebase Speed Response: ");
      Serial.println(http.getString());
    } else {
      Serial.print("Error on Speed Upload: ");
      Serial.println(httpResponseCode);
    }
    http.end();

    // Acceleration Spike
    http.begin(client, String("https://") + firebaseHost + "/blackbox_values/acceleration_spike.json?auth=" + firebaseAuth);
    payload = "{\"value\":" + String(accelerationSpike ? "true" : "false") + "}";
    httpResponseCode = http.PUT(payload);
    if (httpResponseCode > 0) {
      Serial.print("Firebase Acceleration Spike Response: ");
      Serial.println(http.getString());
    } else {
      Serial.print("Error on Acceleration Spike Upload: ");
      Serial.println(httpResponseCode);
    }
    http.end();

    // Temperature
    http.begin(client, String("https://") + firebaseHost + "/blackbox_values/bus_temperature.json?auth=" + firebaseAuth);
    payload = "{\"temperature\":" + String(temperature, 2) + "}";
    httpResponseCode = http.PUT(payload);
    if (httpResponseCode > 0) {
      Serial.print("Firebase Temperature Response: ");
      Serial.println(http.getString());
    } else {
      Serial.print("Error on Temperature Upload: ");
      Serial.println(httpResponseCode);
    }
    http.end();

  } else {
    Serial.println("Error in WiFi connection");
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(1000);
  }
}

void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);   // set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);  // set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// Read all 14 registers
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}
