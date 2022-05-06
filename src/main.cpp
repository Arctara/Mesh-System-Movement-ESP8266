/*
    PINOUT
      PIN 4 = Digital Sensor
      PIN 14 = LED 1
      PIN 12 = LED 2
*/

//$ Include Library
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

#define SENSOR_PIN 4

//$ Access Point Configuration
#define WIFI_SSID "ALiVe_AP"
#define WIFI_PASS "LeTS_ALiVe"

String reading;

unsigned long interval = 60000;
unsigned long prevInterval = 0;
unsigned long current_time = millis();
unsigned long last_trigger = 0;
boolean timer_on = false;
boolean alreadySent = false;
boolean movementDetected = false;
boolean finalMovementDetected = false;
boolean hasDetach = false;
boolean hasExtended = false;

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

  pinMode(SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), movement_detection,
                  RISING);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED && millis() >= 15000) {
    Serial.print(".");
    delay(500);
  }

  webSocket.begin("192.168.5.1", 80, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  current_time = millis();

  if (movementDetected) {
    if (!hasDetach) {
      detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
      hasDetach = true;
    }
  } else {
    if (hasDetach) {
      attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), movement_detection,
                      RISING);
      hasDetach = false;
    }
  }

  if (timer_on && (current_time - last_trigger) > (interval - 15000ul) &&
      (current_time - last_trigger) < interval) {
    noInterrupts();
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), final_movement_detection,
                    RISING);
    if (finalMovementDetected) {
      if (!hasExtended) {
        prevInterval = interval;
        interval += 60000ul;
        hasExtended = true;
      }
    }
    interrupts();
  }

  if (timer_on && (current_time - last_trigger) > prevInterval &&
      (current_time - last_trigger) < (interval - 15000ul)) {
    hasExtended = false;
    finalMovementDetected = false;
    detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
  }

  if (timer_on && (current_time - last_trigger) > interval) {
    //$ Tampilkan tulisan pada Serial
    Serial.println("Tidak ada gerakan");

    interval = 60000;
    hasExtended = false;

    movementDetected = false;
    reading = "Tidak ada gerakan";

    if (!alreadySent) {
      sendData();
      alreadySent = true;
    }

    //$ Set variabel timer on menjadi false
    timer_on = false;
  }
  webSocket.loop();
}

void IRAM_ATTR movement_detection() {
  noInterrupts();
  Serial.println("Gerakan terdeteksi!");
  Serial.println(digitalRead(SENSOR_PIN));

  movementDetected = true;
  reading = "Ada Gerakan";

  sendData();

  alreadySent = false;
  timer_on = true;
  last_trigger = millis();
  interrupts();
}

void IRAM_ATTR final_movement_detection() {
  noInterrupts();

  finalMovementDetected = true;

  interrupts();
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
