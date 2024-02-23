#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

const char* ssid = "Raminda_rulzz_"; // Replace with your WiFi SSID
const char* password = "Raminda@123"; // Replace with your WiFi password
const char* firebaseHost = "nodemcu5575-default-rtdb.asia-southeast1.firebasedatabase.app"; // Your Firebase host
const char* firebaseAuth = "HWrrY1F1QjJrkKZkwgQgXaCia56yK6JefnoJ6r0F"; // Your Firebase database secret

WiFiClientSecure client;
const int sensorPin = D5; // Pin where the H206 sensor is connected

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  pinMode(sensorPin, INPUT); // Initialize the sensor pin as an input
  client.setInsecure(); // Use this only for development or in trusted environments
}

void loop() {
  int sensorValue = readSensorValue(); // Read the sensor value
  uploadSensorValueToFirebase(sensorValue); // Upload the value to Firebase
  delay(3000); // Upload a new value every 3 seconds
}

int readSensorValue() {
  int value = digitalRead(sensorPin); // Read the digital value (high or low) from the sensor
  Serial.print("Read sensor value: ");
  Serial.println(value);
  return value;
}

void uploadSensorValueToFirebase(int value) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(client, String("https://") + firebaseHost + "/blackbox_values/speed.json?auth=" + firebaseAuth);
    http.addHeader("Content-Type", "application/json");
    String payload = String(value);

    Serial.print("Uploading sensor value to Firebase: ");
    Serial.println(payload);

    int httpResponseCode = http.PUT(payload);
    if (httpResponseCode > 0) {
      Serial.print("Response code: ");
      Serial.println(httpResponseCode);
      String response = http.getString();
      Serial.print("Firebase response: ");
      Serial.println(response);
    } else {
      Serial.print("Failed to send PUT request: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("Error in WiFi connection");
  }
}
