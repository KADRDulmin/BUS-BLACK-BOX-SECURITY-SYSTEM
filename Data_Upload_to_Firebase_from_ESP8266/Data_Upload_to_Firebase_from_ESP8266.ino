#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

const char* ssid = "Raminda_rulzz_";
const char* password = "Raminda@123";
const char* firebaseHost = "nodemcu5575-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* firebaseAuth = "HWrrY1F1QjJrkKZkwgQgXaCia56yK6JefnoJ6r0F";

WiFiClientSecure client;
const int sensorPin = D5; // Pin where the H206 sensor is connected
unsigned long previousMillis = 0;
const long interval = 5000; // Interval of 5 seconds
volatile int pulseCount = 0;

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
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorInterrupt, FALLING);

  client.setInsecure();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    detachInterrupt(sensorPin);
    
    // Calculate speed
    double circumference = 3.14159265358979323846 * 25; // mm per revolution
    double revolutions = pulseCount / 20.0; // Number of revolutions
    double distance = revolutions * circumference / 1000000; // km
    double speed = (distance / (5.0 / 3600)); // km/h

    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.println(" km/h");

    // Upload to Firebase
    uploadSensorValueToFirebase(speed);

    pulseCount = 0; // Reset pulse count
    attachInterrupt(digitalPinToInterrupt(sensorPin), sensorInterrupt, FALLING);
  }
}

void uploadSensorValueToFirebase(double speed) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(client, String("https://") + firebaseHost + "/blackbox_values/speed.json?auth=" + firebaseAuth);
    http.addHeader("Content-Type", "application/json");
    String payload = "{\"speed\":" + String(speed, 6) + "}";

    int httpResponseCode = http.PUT(payload);
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.print("Firebase response: ");
      Serial.println(response);
    } else {
      Serial.print("Error on sending PUT Request: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("Error in WiFi connection");
  }
}
