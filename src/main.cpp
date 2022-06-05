//$ Include Library
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

#define BLUE_LED 14
#define GREEN_LED 12
#define SENSOR_PIN 4

//$ Access Point Configuration
#define WIFI_SSID "ALiVe_AP"
#define WIFI_PASS "LeTS_ALiVe"

String reading;
String prevReading;

unsigned long prevMillis = 0;
unsigned long current_time = millis();
unsigned long last_scan = 0;
bool alreadySent = false;

WebSocketsClient webSocket;

DynamicJsonDocument data(1024);
DynamicJsonDocument receivedData(1024);

//* Device Name
const String deviceName = "sensor-2";
const String sensorType = "movementSensor";
const String centerName = "center";

void IRAM_ATTR movement_detection();
void IRAM_ATTR final_movement_detection();

void sendData();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_PIN, INPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED && millis() >= 15000) {
    digitalWrite(BLUE_LED, HIGH);
    delay(50);
    digitalWrite(BLUE_LED, LOW);
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(BLUE_LED, HIGH);
  }

  webSocket.begin("192.168.5.1", 80, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  current_time = millis();

  if (current_time - last_scan >= 10348) {
    last_scan = millis();

    if (webSocket.isConnected()) {
      Serial.println("WebSocket Connected");
    } else {
      Serial.println("Connecting (Please Connect)");
      webSocket.begin("192.168.5.1", 80, "/ws");
    }
  }

  if (digitalRead(SENSOR_PIN) == HIGH) {
    reading = "Ada Gerakan";
  } else {
    reading = "Tidak ada gerakan";
  }

  if (prevReading != reading) {
    sendData();
  }

  prevReading = reading;

  webSocket.loop();
}

void sendData() {
  data["from"] = deviceName;
  data["sensorType"] = sensorType;
  data["to"] = centerName;
  data["data"] = reading;
  String msg;
  serializeJson(data, msg);
  webSocket.sendTXT(msg);
  Serial.println("Data sent!");
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    deserializeJson(receivedData, payload);

    String myData;
    serializeJson(receivedData, myData);
    String from = receivedData["from"].as<String>();
    String to = receivedData["to"].as<String>();
    String condition = receivedData["condition"].as<String>();

    Serial.println(myData);
    Serial.println(from);
    Serial.println(to);
    Serial.println(condition);
  }
}
