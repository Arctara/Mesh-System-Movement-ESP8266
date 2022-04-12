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
// #include <painlessMesh.h>

//$ Mesh Configuration
// #define MESH_PREFIX "ALiVe_MESH"
// #define MESH_PASSWORD "TmlhdCBzZWthbGkgYW5kYSBtZW5kZWNvZGUgaW5pIC1NZXJ6YQ=="
// #define MESH_PORT 5555

#define SENSOR_PIN 4

//$ Access Point Configuration
#define WIFI_SSID "ALiVe_AP"
#define WIFI_PASS "LeTS_ALiVe"

//*Mesh Configuration
// Scheduler userScheduler;
// painlessMesh mesh;
// int nodeNumber = 1;

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

void IRAM_ATTR movement_detection();
void IRAM_ATTR final_movement_detection();

void sendData();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
// Task taskSendMessage(TASK_SECOND * 1, TASK_FOREVER, &sendMessage);

//$ Needed for painless mesh library
// void receivedCallback(uint32_t from, String &msg);
// void newConnectionCallback(uint32_t nodeId);
// void changedConnectionCallback();
// void nodeTimeAdjustedCallback(int32_t offset);

void setup() {
  Serial.begin(115200);
  // mesh.setDebugMsgTypes(ERROR | STARTUP);

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

  // mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  // mesh.onReceive(&receivedCallback);
  // mesh.onNewConnection(&newConnectionCallback);
  // mesh.onChangedConnections(&changedConnectionCallback);
  // mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  // userScheduler.addTask(taskSendMessage);
  // taskSendMessage.enable();
}

void loop() {
  // mesh.update();
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
  data["from"] = "movement-sensor";
  data["to"] = "center";
  data["data"] = reading;
  String msg;
  serializeJson(data, msg);
  webSocket.sendTXT(msg);
  Serial.println("Data sent!");
  // mesh.sendBroadcast(msg);
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

//$ Needed for painless mesh library
// void receivedCallback(uint32_t from, String &msg) {
//   Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
//   deserializeJson(receivedData, msg);
//   if (data["from"].as<String>() == "center" &&
//       data["to"].as<String>() == "lamp-1") {
//     if (data["condition"].as<String>() == "true") {
//       plugCondition = true;
//       dimmer.setState(ON);
//     } else {
//       plugCondition = false;
//       dimmer.setState(OFF);
//     }
//   }
//   sendMessage();
// }

// void newConnectionCallback(uint32_t nodeId) {
//   Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
// }

// void changedConnectionCallback() { Serial.printf("Changed connections\n"); }

// void nodeTimeAdjustedCallback(int32_t offset) {
//   Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),
//   offset);
// }