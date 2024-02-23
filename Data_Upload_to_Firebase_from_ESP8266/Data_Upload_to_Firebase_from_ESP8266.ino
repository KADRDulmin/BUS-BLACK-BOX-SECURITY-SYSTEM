#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

const char* ssid = "Raminda_rulzz_";
const char* password = "Raminda@123";

// Note: The Firebase host should be without "https://"
const char* firebaseHost = "nodemcu5575-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* firebaseAuth = "HWrrY1F1QjJrkKZkwgQgXaCia56yK6JefnoJ6r0F";

WiFiClientSecure client;

void setup() {
  Serial.begin(115200);
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to WiFi");

  // Set fingerprint for Firebase SSL certificate
  // This might change, so check Firebase's latest certificate fingerprint
  client.setInsecure(); // Use this only if you cannot verify the fingerprint

  uploadRandomValue();
}

void uploadRandomValue() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(client, String("https://") + firebaseHost + "/test/value.json?auth=" + firebaseAuth);
    http.addHeader("Content-Type", "application/json");

    int randomValue = random(0, 100); // Generate a random value between 0 and 100
    String payload = String(randomValue);

    Serial.print("Uploading random value: ");
    Serial.println(payload);

    int httpResponseCode = http.PUT(payload);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.print("Response code: ");
      Serial.println(httpResponseCode);
      Serial.print("Response: ");
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

void loop() {
  // Here you could call uploadRandomValue() in intervals or based on certain conditions
  delay(30000); // Example: Upload a new value every 30 seconds
}
