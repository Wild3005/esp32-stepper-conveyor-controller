#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <queue>
#include <vector>
#include <algorithm>

#include "ShiftStepper.hpp"

// =================== CONFIG ===================
#define NUM_MOTORS 5
#define NUM_GATES 3

// Jumlah IC 74HC595 yang diseri (mis. tambah 2 IC dari 1 IC awal => total 3)
#define SHIFT595_COUNT 3
#define SHIFT595_BITS (SHIFT595_COUNT * 8)

#define DATA_PIN 5
#define CLOCK_PIN 19
#define LATCH_PIN 18

#define CONVEYOR_PIN 14

#define TRIG1 27
#define ECHO1 35

#define TRIG2 21
#define ECHO2 22

#define TRIG3 32
#define ECHO3 33

// ================= STEPPER CONFIG =================
// #define STEPS_PER_MOVE 4096   // sesuaikan (1/2 putaran biasanya)
#define STEPS_PER_MOVE 2048   // sesuaikan (1/2 putaran biasanya)

//| =================== GLOBAL VARIABLE ===================
Servo gateServos[NUM_GATES];
int gatePins[NUM_GATES] = {26, 25, -1};
bool gateAttached[NUM_GATES] = {false};

bool gateOpened[NUM_GATES] = {false};
unsigned long gateOpenTime[NUM_GATES] = {0};
const int gateDuration = 2000;

unsigned long lastUltrasonicRead = 0;
const int ultrasonicInterval = 50; // ms (atur 20–100)

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
float readUltrasonic(int trig, int echo);
void processConveyorEnd();

std::queue<Task> taskQueue;
std::vector<Task> task;
std::queue<ConveyorTask> conveyorQueue;
bool motorBusy = false;
Task currentTask;

bool motorStarted = false;

// ================= WIFI & MQTT =================
unsigned long lastReconnectAttempt = 0;

const char* ssid = "Wild";
const char* password = "12345678";

// const char* ssid = "Ppppp";
// const char* password = "12345654";

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
char topics[3][32] = {"vending/VM001/cmd","vending/VM002/cmd","vending/VM003/cmd"};

WiFiClient espClient;
PubSubClient client(espClient);


//| =================== FUNCTION ===================

void setupGates() {
    for (int i = 0; i < NUM_GATES; i++) {
        if (gatePins[i] != -1){
            gateServos[i].attach(gatePins[i]);
            gateServos[i].write(180);
            gateAttached[i] = true;
        }
    }
}

void updateGates() {
    for (int i = 0; i < NUM_GATES; i++) {
        if (!gateAttached[i]) continue;

        if (gateOpened[i] && millis() - gateOpenTime[i] >= gateDuration) {
            gateServos[i].write(180);
            gateOpened[i] = false;
        }
    }
}

void openGate(int gate) {
    int idx = gate - 1;

    if (idx < 0 || idx >= NUM_GATES) return;
    if (!gateAttached[idx]) return;

    if (!gateOpened[idx]) {
        gateServos[idx].write(0);
        gateOpened[idx] = true;
        gateOpenTime[idx] = millis();
    }
}

void processConveyorEnd() {
  // Serial.println("conveyor queue: " + String(conveyorQueue.size()));
    if (conveyorQueue.empty()) return;

    if (millis() - lastUltrasonicRead < ultrasonicInterval) return;
    lastUltrasonicRead = millis();

    float d1 = readUltrasonic(TRIG1, ECHO1);
    float d2 = readUltrasonic(TRIG2, ECHO2);
    float d3 = readUltrasonic(TRIG3, ECHO3);

    // if(d1 > 12){
    //   d1 = 0;
    // }

    // if(d2 > 12){
    //   d2 = 0;
    // }

    // if(d3 > 12){
    //   d3 = 0;
    // }

    Serial.print("Ultrasonic: ");
    Serial.println(d1);

    ConveyorTask ct = conveyorQueue.front();

    // bool detect1 = (d1 > 2 && d1 < 8);
    // bool detect2 = (d2 > 2 && d2 < 8);
    // bool detect3 = (d3 > 2 && d3 < 8);
    bool detect1 = (d1 < 6.3f);
    bool detect2 = (d2 < 6.3f);
    bool detect3 = (d3 < 6.3f);

    if (millis() - lastDetectTime < detectCooldown) return;

    if (detect1 && ct.gate == 1) {
        lastDetectTime = millis();
        Serial.println("Dorong ke Gate 1");
        openGate(1);
        conveyorQueue.pop();
    }
    else if (detect2 && ct.gate == 2) {
        lastDetectTime = millis();
        Serial.println("Dorong ke Gate 2");
        openGate(2);
        conveyorQueue.pop();
    }
    else if (detect3 && ct.gate == 3) {
        lastDetectTime = millis();
        Serial.println("Dorong ke Gate 3");
        openGate(3);
        conveyorQueue.pop();
    }
}

float readUltrasonic(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    float duration = pulseIn(echo, HIGH, 10000);
    float distance = duration * 0.034f / 2.0f;

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
    // if (!motorBusy && !taskQueue.empty()) {
    //     currentTask = taskQueue.front();
    //     taskQueue.pop();

    //     controlMotor(currentTask.item);
    //     motorBusy = true;
    //     motorStarted = false; // reset
    // }

    bool is_sorted = std::is_sorted(task.begin(), task.end(), [](const Task& a, const Task& b) {
        return a.item > b.item;
    });

    if(!is_sorted) {
      Serial.println("Sorting task queue...");
      std::sort(task.begin(), task.end(), [](const Task& a, const Task& b) {
          return a.item > b.item;
      });
    }

    if (!motorBusy && !task.empty()) {
        currentTask = task.front();
        task.erase(task.begin());

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
            }

            // baru boleh selesai
            if (motorStarted && steppers[idx].distanceToGo() == 0) {
                ConveyorTask ct;
                ct.gate = currentTask.gate;

                conveyorQueue.push(ct);
                
                motorBusy = false;
                Serial.println("Task selesai");
            }
        }
    }
}

void shiftOut595(uint32_t data) {
  digitalWrite(LATCH_PIN, LOW);

  for (int i = SHIFT595_BITS - 1; i >= 0; i--) {
    digitalWrite(CLOCK_PIN, LOW);
    digitalWrite(DATA_PIN, (data >> i) & 1);
    digitalWrite(CLOCK_PIN, HIGH);
  }

  digitalWrite(LATCH_PIN, HIGH);
}

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
  ShiftStepper(1, shiftOut595),  // Q4–Q7
  ShiftStepper(2, shiftOut595),  // Q8–Q11
  ShiftStepper(3, shiftOut595),  // Q12–Q15
  ShiftStepper(4, shiftOut595)   // Q16–Q19
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

  int gate;

  if(strcmp(topic, "vending/VM001/cmd")==0){
    gate = 1;
  } else if (strcmp(topic, "vending/VM002/cmd") == 0){
    gate = 2;
  } else if (strcmp(topic, "vending/VM003/cmd") == 0){
    gate = 3;
  } else {
    Serial.println("Unknown topic: " + String(topic));
    return;
  }

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

    // taskQueue.push(t);
    task.push_back(t);
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
        for(char* topic : topics) {
          client.subscribe(topic);
          Serial.println("Subscribed to topic: " + String(topic));
        }
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

  setupGates();

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

  updateGates();

  controlConveyor();
  // int temp = readUltrasonic(TRIG2, ECHO2); // baca dulu untuk update state
  // Serial.print("ini di loop: ");
  // Serial.println(temp);
  
  processConveyorEnd();
}