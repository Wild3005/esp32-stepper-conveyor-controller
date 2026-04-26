#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <queue>
#include <vector>

#include "ShiftStepper.hpp"

// =================== CONFIG ===================
#define NUM_MOTORS 2

#define DATA_PIN 27
#define CLOCK_PIN 25
#define LATCH_PIN 26

#define CONVEYOR_PIN 33

#define TRIG1 12
#define ECHO1 14

#define TRIG2 18
#define ECHO2 19

#define TRIG3 21
#define ECHO3 22

// ================= STEPPER CONFIG =================
// #define STEPS_PER_MOVE 4096   // sesuaikan (1/2 putaran biasanya)
#define STEPS_PER_MOVE 2048   // sesuaikan (1/2 putaran biasanya)

//| =================== GLOBAL VARIABLE ===================
unsigned long lastQueueEmptyTime = 0;
const int stopDelay = 1000;

unsigned long lastDetectTime = 0;
const int detectCooldown = 500;

extern ShiftStepper steppers[NUM_MOTORS];

struct Task {
  int item;
  int gate;
};

struct ConveyorTask {
  int gate;
};

void controlMotor(int item);
void processQueue();
void controlConveyor();
long readUltrasonic(int trig, int echo);
void processConveyorEnd();

std::queue<Task> taskQueue;
std::queue<ConveyorTask> conveyorQueue;
bool motorBusy = false;
Task currentTask;

bool motorStarted = false;


//| =================== FUNCTION ===================
void processConveyorEnd() {
    if (conveyorQueue.empty()) return;

    long d1 = readUltrasonic(TRIG1, ECHO1);
    long d2 = readUltrasonic(TRIG2, ECHO2);
    long d3 = readUltrasonic(TRIG3, ECHO3);

    ConveyorTask ct = conveyorQueue.front();

    // threshold
    bool detect1 = (d1 > 0 && d1 < 10);
    bool detect2 = (d2 > 0 && d2 < 10);
    bool detect3 = (d3 > 0 && d3 < 10);

    if (millis() - lastDetectTime < detectCooldown) return;

    // ================= GATE 1 =================
    if (detect1 && ct.gate == 1) {
        lastDetectTime = millis();

        Serial.println("Dorong ke Gate 1");
        // servoGate1();

        conveyorQueue.pop();
    }

    // ================= GATE 2 =================
    else if (detect2 && ct.gate == 2) {
        lastDetectTime = millis();

        Serial.println("Dorong ke Gate 2");
        // servoGate2();

        conveyorQueue.pop();
    }

    // ================= GATE 3 =================
    else if (detect3 && ct.gate == 3) {
        lastDetectTime = millis();

        Serial.println("Dorong ke Gate 3");
        // servoGate3();

        conveyorQueue.pop();
    }
}

long readUltrasonic(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, 30000);
    long distance = duration * 0.034 / 2;

    return distance;
}

void controlConveyor() {
    if (!conveyorQueue.empty()) {
        digitalWrite(CONVEYOR_PIN, HIGH);
        lastQueueEmptyTime = millis();
    } else {
        if (millis() - lastQueueEmptyTime > stopDelay) {
            digitalWrite(CONVEYOR_PIN, LOW);
        }
    }
}

void controlMotor(int item) {
    if (item < 1 || item > NUM_MOTORS) return;

    int index = item - 1;

    if (steppers[index].distanceToGo() == 0) {
        ShiftStepper::_buffer = 0;
        steppers[index].setCurrentPosition(0);
        steppers[index].move(STEPS_PER_MOVE);
    }

    Serial.print("Motor ");
    Serial.print(item);
    Serial.println(" jalan");
}

void processQueue() {
    if (!motorBusy && !taskQueue.empty()) {
        currentTask = taskQueue.front();
        taskQueue.pop();

        controlMotor(currentTask.item);
        motorBusy = true;
        motorStarted = false; // reset
    }

    if (motorBusy) {
        int idx = currentTask.item - 1;

        if (idx >= 0 && idx < NUM_MOTORS) {

            // tunggu sampai benar-benar mulai bergerak
            if (!motorStarted && steppers[idx].distanceToGo() != 0) {
                motorStarted = true;
                
                ConveyorTask ct;
                ct.gate = currentTask.gate;

                conveyorQueue.push(ct);
            }

            // baru boleh selesai
            if (motorStarted && steppers[idx].distanceToGo() == 0) {
                motorBusy = false;
                Serial.println("Task selesai");
            }
        }
    }
}

void shiftOut595(uint8_t data) {
  digitalWrite(LATCH_PIN, LOW);

  for (int i = 7; i >= 0; i--) {
    digitalWrite(CLOCK_PIN, LOW);
    digitalWrite(DATA_PIN, (data >> i) & 1);
    digitalWrite(CLOCK_PIN, HIGH);
  }

  digitalWrite(LATCH_PIN, HIGH);
}

// ================= WIFI & MQTT =================
unsigned long lastReconnectAttempt = 0;

const char* ssid = "Wild";
const char* password = "12345678";

// const char* ssid = "Ppppp";
// const char* password = "12345654";

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* topic = "vending/VM001/cmd";

WiFiClient espClient;
PubSubClient client(espClient);

static const char* mqttStateToStr(int state) {
  switch (state) {
    case MQTT_CONNECTION_TIMEOUT: return "MQTT_CONNECTION_TIMEOUT";      // -4
    case MQTT_CONNECTION_LOST:    return "MQTT_CONNECTION_LOST";         // -3
    case MQTT_CONNECT_FAILED:     return "MQTT_CONNECT_FAILED";          // -2
    case MQTT_DISCONNECTED:       return "MQTT_DISCONNECTED";            // -1
    case MQTT_CONNECTED:          return "MQTT_CONNECTED";               // 0
    case MQTT_CONNECT_BAD_PROTOCOL: return "MQTT_CONNECT_BAD_PROTOCOL";  // 1
    case MQTT_CONNECT_BAD_CLIENT_ID: return "MQTT_CONNECT_BAD_CLIENT_ID";// 2
    case MQTT_CONNECT_UNAVAILABLE: return "MQTT_CONNECT_UNAVAILABLE";    // 3
    case MQTT_CONNECT_BAD_CREDENTIALS: return "MQTT_CONNECT_BAD_CREDENTIALS"; // 4
    case MQTT_CONNECT_UNAUTHORIZED: return "MQTT_CONNECT_UNAUTHORIZED";  // 5
    default: return "MQTT_UNKNOWN";
  }
}

ShiftStepper steppers[NUM_MOTORS] = {
  ShiftStepper(0, shiftOut595), // Q0–Q3
  ShiftStepper(1, shiftOut595)  // Q4–Q7
};

// ================= MOTOR CONTROL =================
void setupMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].setMaxSpeed(300);
    steppers[i].setAcceleration(100);
  }
}

void runMotors() {
  bool anyRunning = false;

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (steppers[i].distanceToGo() != 0) {
      anyRunning = true;
    }
    steppers[i].run();
  }

  if (!anyRunning) {
    shiftOut595(0); // matikan coil
  }
  
}

// ================= MQTT CALLBACK =================
// void callback(char* topic, byte* payload, unsigned int length) {
//   Serial.print("Message arrived [");
//   Serial.print(topic);
//   Serial.print("] ");

//   String message;
//   for (int i = 0; i < length; i++) {
//     message += (char)payload[i];
//   }

//   Serial.println(message);

//   StaticJsonDocument<512> doc;
//   DeserializationError error = deserializeJson(doc, message);

//   if (error) {
//     Serial.print("JSON Error: ");
//     Serial.println(error.c_str());
//     return;
//   }

//   JsonArray items = doc["items"];

//   for (int item : items) {
//     controlMotor(item);
//   }
// }


void callback(char* topic, byte* payload, unsigned int length) {

  int gate = 1; // dari topic (sementara hardcode)

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
      Serial.println("JSON parse error");
      return;
  }

  JsonArray items = doc["items"];
  if (!doc.containsKey("items")) return;

  for (int item : items) {
    Task t;
    t.item = item;
    t.gate = gate;

    taskQueue.push(t);
  }
}

// ================= WIFI =================
void setupWIFI(){
  delay(10);
  Serial.println("Connecting to WiFi");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
}

// ================= MQTT RECONNECT =================
void reconnect() {
    if (client.connected()) return;

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected, skip MQTT connect");
      return;
    }

    IPAddress brokerIp;
    if (WiFi.hostByName(mqtt_server, brokerIp)) {
      Serial.print("MQTT broker IP: ");
      Serial.println(brokerIp);
    } else {
      Serial.println("DNS lookup failed for MQTT broker");
    }

    Serial.print("Connecting to MQTT...");

    String clientId = "ESP32-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
        Serial.println("connected");
        client.subscribe(topic);
    } else {
        int st = client.state();
        Serial.print("failed, rc=");
        Serial.print(st);
        Serial.print(" (");
        Serial.print(mqttStateToStr(st));
        Serial.println(")");
    }
}
// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  pinMode(CONVEYOR_PIN, OUTPUT);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  randomSeed(micros());

  setupWIFI();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(30);
  client.setSocketTimeout(5);

  delay(1000);
  reconnect();      // CONNECT MQTT

  setupMotors();    // BARU MOTOR
}
// ================= LOOP =================
void loop() {
  if (!client.connected()) {
    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      reconnect();
    }
  } else {
    client.loop();
  }

  runMotors();
  processQueue();

  controlConveyor();
  processConveyorEnd();
}