#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <FirebaseArduino.h>
#include <Wire.h>

#define WIFI_SSID "Raminda_rulzz_"
#define WIFI_PASSWORD "Raminda@123"
#define FIREBASE_HOST "nodemcu5575-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "HWrrY1F1QjJrkKZkwgQgXaCia56yK6JefnoJ6r0F"

TinyGPSPlus gps;
SoftwareSerial SerialGPS(4, 5);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
String epochTime;
String tt, message;
float Latitude, Longitude;
double speed;
unsigned long lastTime = 0;
unsigned long timerDelay = 60000;
int sta;
const int sensorPin = D8;          // Speed sensor pin
const uint8_t scl = D6, sda = D7;  // MPU6050 I2C pins
const uint8_t buzzerPin = D3;      // Buzzer pin for acceleration spike

unsigned long previousMillis = 0;
const long interval = 5000;  // 5-second interval for speed calculation
volatile int pulseCount = 0;
bool accelerationSpike = false;
int T;
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

  Serial.begin(9600);
  // SerialGPS.begin(9600);
  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorInterrupt, FALLING);
  Wire.begin(sda, scl);
  MPU6050_Init();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  timeClient.begin();
  timeClient.setTimeOffset(0);
  timeClient.update();
  epochTime = timeClient.getEpochTime();
  Serial.println(epochTime);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void loop() {

  epochTime = timeClient.getEpochTime();
  tt = epochTime + "000";

  if ((millis() - lastTime) > timerDelay) {       // One min time delay
    pushData();
    lastTime = millis();
  }


  /////////////////////////////////////////////////////////////

  while (SerialGPS.available() > 0)
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        Latitude = gps.location.lat();
        Serial.print("Latitude : ");
        Serial.println(Latitude);
        Longitude = gps.location.lng();
        Serial.print("Longitude : ");
        Serial.println(Longitude);
        Firebase.setFloat("/blackbox_values/lat", Latitude);
        Firebase.setFloat("/blackbox_values/lng", Longitude);
      }
    }

  Firebase.setFloat("/blackbox_values/lat", Latitude);
  Firebase.setFloat("/blackbox_values/lng", Longitude);

  unsigned long currentMillis = millis();

  // Read accelerometer and temperature values continuously
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  double Ax = (double)AccelX / AccelScaleFactor;
  double Ay = (double)AccelY / AccelScaleFactor;
  double Az = (double)AccelZ / AccelScaleFactor;
  T = (int)Temperature / 340 + 36.53;  // Temperature formula
  Serial.print("T : ");
  Serial.println(T);
  Firebase.setInt("/blackbox_values/bus_temperature", T);

  if (T > 50 && sta == 0) {
    sta = 1;
    message = "FIRE DETECTED!!";
    notfy();
  } else if (T < 50 && sta == 1) {
    sta = 0;
  }

  if (T > 50 && sta == 0) {
    sta = 1;
    pushData();
  } else if (T < 50 && sta == 1) {
    sta = 0;
  }

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
    Firebase.setBool("/blackbox_values/acceleration_spike", accelerationSpike);
    message = "ACCIDENT DETECTED !!";
    notfy();
    if (accelerationSpike == true &&sta == 0) {
      sta = 1;
      pushData();
    } else if (accelerationSpike == false &&sta == 1) {
      sta = 0;
    }
  } else {
    accelerationSpike = currentAccelerationSpike;
    Firebase.setBool("/blackbox_values/acceleration_spike", accelerationSpike);
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
    speed = distance / (5.0 / 3600);                          // Speed in km/h

    if (speed > 2 && sta == 0) {
      sta = 1;
      message = "FIRE DETECTED!!";
      notfy();
    } else if (speed < 2 && sta == 1) {
      sta = 0;
    }

    if (speed > 2 && sta == 0) {
      sta = 1;
      pushData();
    } else if (speed < 2 && sta == 1) {
      sta = 0;
    }

    Firebase.setFloat("/blackbox_values/speed", speed);
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.println(" km/h");
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
void pushData() {
  StaticJsonBuffer<200> jsonBuffer2;
  JsonObject& obj2 = jsonBuffer2.createObject();
  obj2["acceleration_spike"] = accelerationSpike;
  obj2["bus_temperature"] = T;
  obj2["busno"] = "PN 2134";
  obj2["lat"] = 6.819866752601754;  //replace with - Latitude
  obj2["lng"] = 80.03957133876045;  //replace with - Longitude
  obj2["speed"] = speed;
  obj2["timestamp"] = tt;
  Firebase.set("/history/" + tt + "", obj2);
}
void notfy() {
  Firebase.setString("/notification/message", message);
  Firebase.setBool("/notification/istrue", true);
}