#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include <algorithm>
#include <queue>
#include <string>
#include <vector>

#include "ShiftStepper.hpp"

// =================== global variable before config ===================
static std::vector<int> list_avaliable = {};

// =================== CONFIG ===================
size_t NUM_MOTORS = 0;
#define NUM_GATES 3
#define NUM_TOPIC_SUB 4
#define NUM_TOPIC_PUB 2

size_t SHIFT595_COUNT = 2;

#define LED_BLUE_PIN 2
#define DATA_PIN 4
#define CLOCK_PIN 18
#define LATCH_PIN 5
#define CONVEYOR_PIN 14

#define TRIG1 21
#define TRIG2 32
#define TRIG3 26

// ================= STEPPER CONFIG =================
#define STEPS_PER_MOVE 2048

//| =================== GLOBAL VARIABLE ===================
std::vector<uint8_t> shiftBuffer;

// delay untuk pergatian item yang jauh
bool waitingDelay = false;
unsigned long delayStart = 0;
unsigned long delayDuration = 0;

static int current_index = -1;
static int prev_index = -1;

Servo gateServos[NUM_GATES];
int gatePins[NUM_GATES] = {23, 25, -1};
bool gateAttached[NUM_GATES] = {false};

bool gateOpened[NUM_GATES] = {false};
unsigned long gateOpenTime[NUM_GATES] = {0};
const int gateDuration = 450;

unsigned long lastQueueEmptyTime = 0;
const int stopDelay = 1000;

const int pushDelay = 100;
uint8_t buffer_ultrasonic = 5;
const int threshold_buffer_ultrasonic = 4;
bool detect_to_nodetect = false;
bool prev_detect_to_nodetect = false;

volatile bool ultrasonicTaskEnable = false;
volatile bool servoTaskEnable = false;

std::vector<ShiftStepper> steppers;
bool motorsConfigured = false;

struct Task
{
  int item;
  int gate;
};

struct ConveyorTask
{
  int gate;
};

bool controlMotor(int item);
int resolveMotorIndex(int item);
void processQueue();
void controlConveyor();
void processConveyorEnd();
void setupMotors();
void syncMotorResources();
void shiftOut595(uint8_t *data, size_t size);
void updateGates();

std::queue<Task> taskQueue;
std::vector<Task> task;
std::queue<ConveyorTask> conveyorQueue;
bool motorBusy = false;
Task currentTask;
bool motorStarted = false;

// ================= WIFI & MQTT =================
unsigned long lastReconnectAttempt = 0;
// const char *ssid = "realme10";
// const char *password = "paansih7";
const char *ssid = "Wild";
const char *password = "12345678";
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;

char topics[NUM_TOPIC_SUB][32] = {
    "vending/VM001/cmd",
    "vending/VM002/cmd",
    "vending/VM003/cmd",
    "vending/config"};

char pubTopics[NUM_TOPIC_PUB][32] = {
    "vending/stock",
    "vending/request_config",
};

WiFiClient espClient;
PubSubClient client(espClient);

// Mutex untuk mengamankan data bersama saat diakses antar-task
SemaphoreHandle_t xMutex;
SemaphoreHandle_t conveyorMutex;

// =================== FREE RTOS TASK HANDLES ===================
void vTaskNetwork(void *pvParameters);
void vTaskLogicAndHardware(void *pvParameters);
void vTaskUltrasonic(void *pvParameters);
void vTaskServo(void *pvParameters);
void vTaskConveyorServo(void *pvParameters);

//| =================== FUNCTION ===================

void ledBlueOn(bool isActive)
{
  if (isActive)
    digitalWrite(LED_BLUE_PIN, HIGH);
  else
    digitalWrite(LED_BLUE_PIN, LOW);
}

template <typename T, typename U>
std::string parsePubJSON(const T &key, const U &value)
{
  StaticJsonDocument<200> doc;
  doc[key] = value;
  std::string output;
  serializeJson(doc, output);
  return output;
}

std::string parsePubJSON(int item)
{
  StaticJsonDocument<200> doc;
  doc["item"] = item;
  std::string output;
  serializeJson(doc, output);
  return output;
}

void syncMotorResources()
{
  if (list_avaliable.empty())
  {
    NUM_MOTORS = 0;
    motorsConfigured = false;
    return;
  }

  NUM_MOTORS = list_avaliable.back();
  SHIFT595_COUNT = ((NUM_MOTORS + 1) / 2) > 1 ? ((NUM_MOTORS + 1) / 2) : 2;

  shiftBuffer.assign(SHIFT595_COUNT, 0);
  ShiftStepper::begin(shiftBuffer.data(), SHIFT595_COUNT);

  steppers.clear();
  steppers.resize(NUM_MOTORS);

  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    steppers[i].init(i, shiftOut595);
    steppers[i].setMaxSpeed(300);
    steppers[i].setAcceleration(100);
  }

  motorsConfigured = true;
}

int resolveMotorIndex(int item)
{
  if (item < 1)
    return -1;

  if (!list_avaliable.empty())
  {
    auto it = std::find(list_avaliable.begin(), list_avaliable.end(), item);
    if (it == list_avaliable.end())
      return -1;
    return static_cast<int>(std::distance(list_avaliable.begin(), it));
  }

  if (static_cast<size_t>(item) > NUM_MOTORS)
    return -1;
  return item - 1;
}

void setupGates()
{
  for (int i = 0; i < NUM_GATES; i++)
  {
    if (gatePins[i] != -1)
    {
      gateServos[i].attach(gatePins[i]);
      gateServos[i].write(180);
      gateAttached[i] = true;
    }
  }
}

void updateGates()
{
  for (int i = 0; i < NUM_GATES; i++)
  {
    if (!gateAttached[i])
      continue;

    if (gateOpened[i] && millis() - gateOpenTime[i] >= gateDuration)
    {
      gateServos[i].write(180);
      gateOpened[i] = false;
    }
  }
}

void openGate(int gate)
{
  int idx = gate - 1;
  if (idx < 0 || idx >= NUM_GATES)
    return;
  if (!gateAttached[idx])
    return;

  if (!gateOpened[idx])
  {
    gateServos[idx].write(0);
    gateOpened[idx] = true;
    gateOpenTime[idx] = millis();
  }
}

void controlConveyor()
{
  bool anyGateOpen = false;

  for (int i = 0; i < NUM_GATES; i++)
  {
    if (gateOpened[i])
    {
      anyGateOpen = true;
      break;
    }
  }

  if (anyGateOpen)
  {
    digitalWrite(CONVEYOR_PIN, LOW);
    return;
  }

  bool queueNotEmpty = false;

  if (xSemaphoreTake(conveyorMutex, portMAX_DELAY) == pdTRUE)
  {
    queueNotEmpty = !conveyorQueue.empty();
    xSemaphoreGive(conveyorMutex);
  }

  if (queueNotEmpty)
  {
    digitalWrite(CONVEYOR_PIN, HIGH);
    lastQueueEmptyTime = millis();
  }
  else
  {
    if (millis() - lastQueueEmptyTime > stopDelay)
    {
      digitalWrite(CONVEYOR_PIN, LOW);
    }
  }
}

bool controlMotor(int item)
{
  if (!motorsConfigured || NUM_MOTORS == 0)
  {
    Serial.println("Motor belum dikonfigurasi");
    return false;
  }

  int index = resolveMotorIndex(item);
  if (index < 0 || static_cast<size_t>(index) >= NUM_MOTORS)
  {
    Serial.print("ERROR: Motor item not active: ");
    Serial.println(item);
    return false;
  }

  if (steppers[index].distanceToGo() == 0)
  {
    if (ShiftStepper::_buffer != nullptr)
    {
      memset(ShiftStepper::_buffer, 0, SHIFT595_COUNT);
    }
    steppers[index].setCurrentPosition(0);
    steppers[index].move(STEPS_PER_MOVE);
  }

  Serial.print("Motor ");
  Serial.print(item);
  Serial.println(" jalan");
  return true;
}

void processQueue()
{
  if (!motorsConfigured)
    return;

  bool is_sorted = std::is_sorted(task.begin(), task.end(), [](const Task &a, const Task &b)
                                  { return a.item > b.item; });

  if (!is_sorted)
  {
    Serial.println("Sorting task queue...");
    std::sort(task.begin(), task.end(), [](const Task &a, const Task &b)
              { return a.item > b.item; });
  }

  if (!motorBusy && !task.empty())
  {
    currentTask = task.front();
    task.erase(task.begin());

    std::string payload = parsePubJSON(currentTask.item);
    bool ok = client.publish(pubTopics[0], payload.c_str());

    if (ok)
      Serial.println("Publish success");
    else
      Serial.println("Publish failed");

    if (controlMotor(currentTask.item))
    {
      motorBusy = true;
      motorStarted = false;
    }
  }

  if (motorBusy)
  {
    int idx = resolveMotorIndex(currentTask.item);
    if (idx < 0)
    {
      Serial.print("Skipping inactive motor item: ");
      Serial.println(currentTask.item);
      motorBusy = false;
      return;
    }

    current_index = idx;

    if (prev_index != -1)
    {
      if (prev_index < current_index)
      {
        if (current_index % 2 == 0)
        {
          int diff = abs(current_index - prev_index);
          if (!(diff < 3))
          {
            if (!waitingDelay)
            {
              delayDuration = 100 * diff;
              delayStart = millis();
              waitingDelay = true;
              Serial.print("Start delay: ");
              Serial.println(delayDuration);
            }
          }
        }

        if (current_index % 2 == 1)
        {
          int diff = abs(current_index - prev_index);
          if (!(diff < 4))
          {
            if (!waitingDelay)
            {
              delayDuration = 100 * diff;
              delayStart = millis();
              waitingDelay = true;
              Serial.print("Start delay: ");
              Serial.println(delayDuration);
            }
          }
        }
      }
    }

    if (waitingDelay)
    {
      if (millis() - delayStart >= delayDuration)
      {
        waitingDelay = false;
        Serial.println("Delay selesai");
      }
      else
      {
        return;
      }
    }

    if (idx >= 0 && static_cast<size_t>(idx) < NUM_MOTORS)
    {
      if (!motorStarted && steppers[idx].distanceToGo() != 0)
      {
        motorStarted = true;
      }

      if (motorStarted && steppers[idx].distanceToGo() == 0)
      {
        if (xSemaphoreTake(conveyorMutex, portMAX_DELAY) == pdTRUE)
        {
          ConveyorTask ct;
          ct.gate = currentTask.gate;

          conveyorQueue.push(ct);

          ultrasonicTaskEnable = true;
          servoTaskEnable = true;

          xSemaphoreGive(conveyorMutex);
        }

        motorBusy = false;
        Serial.println("Task selesai");
        prev_index = current_index;
      }
    }
  }
}

void shiftOut595(uint8_t *data, size_t size)
{
  digitalWrite(LATCH_PIN, LOW);
  for (int i = (int)size - 1; i >= 0; i--)
  {
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, data[i]);
  }
  digitalWrite(LATCH_PIN, HIGH);
}

void setupMotors()
{
  syncMotorResources();
}

void runMotors()
{
  if (!motorsConfigured)
    return;

  bool anyRunning = false;
  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    steppers[i].run();
    if (steppers[i].distanceToGo() != 0)
    {
      anyRunning = true;
    }
  }

  if (!anyRunning)
  {
    if (!shiftBuffer.empty())
    {
      memset(shiftBuffer.data(), 0, shiftBuffer.size());
      shiftOut595(shiftBuffer.data(), SHIFT595_COUNT);
    }
  }
}

// ================= MQTT CALLBACK =================
void callback(char *topic, byte *payload, unsigned int length)
{
  int gate;
  if (strcmp(topic, "vending/VM001/cmd") == 0)
    gate = 1;
  else if (strcmp(topic, "vending/VM002/cmd") == 0)
    gate = 2;
  else if (strcmp(topic, "vending/VM003/cmd") == 0)
    gate = 3;
  else if (strcmp(topic, "vending/config") == 0)
  {
    Serial.println("Received config update");
    StaticJsonDocument<512> doc_info;
    DeserializationError error_info = deserializeJson(doc_info, payload, length);

    if (error_info)
    {
      Serial.println("JSON parse error info from DB");
      return;
    }

    // Amankan resource bersama dengan Take Mutex saat modifikasi konfigurasi global
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      JsonArray items_info = doc_info["active_indexes"];
      list_avaliable.clear();
      for (int item : items_info)
      {
        list_avaliable.push_back(item);
      }

      Serial.print("Updated available list: ");
      for (int idx : list_avaliable)
      {
        Serial.print(idx);
        Serial.print(" ");
      }
      Serial.println();

      if (!list_avaliable.empty())
        ledBlueOn(true);
      else
        ledBlueOn(false);

      task.clear();
      // while (!conveyorQueue.empty())
      // {
      //   conveyorQueue.pop();
      // }

      xSemaphoreTake(conveyorMutex, portMAX_DELAY);

      while (!conveyorQueue.empty())
      {
        conveyorQueue.pop();
      }

      ultrasonicTaskEnable = false;
      servoTaskEnable = false;

      xSemaphoreGive(conveyorMutex);

      motorBusy = false;
      motorStarted = false;
      waitingDelay = false;
      current_index = -1;
      prev_index = -1;

      syncMotorResources();
      xSemaphoreGive(xMutex);
    }
    return;
  }
  else
  {
    Serial.println("Unknown topic: " + String(topic));
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error)
  {
    Serial.println("JSON parse error");
    return;
  }

  JsonArray items = doc["items"];
  if (!doc.containsKey("items"))
    return;

  if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
  {
    for (int item : items)
    {
      if (resolveMotorIndex(item) < 0)
      {
        Serial.print("Ignored invalid item: ");
        Serial.println(item);
        continue;
      }

      Task t;
      t.item = item;
      t.gate = gate;
      task.push_back(t);
    }
    xSemaphoreGive(xMutex);
  }
}

// ================= WIFI =================
void setupWIFI()
{
  delay(10);
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(500)); // Menggunakan delay RTOS saat inisialisasi awal
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
}

// ================= MQTT RECONNECT =================
void reconnect()
{
  if (client.connected())
    return;
  if (WiFi.status() != WL_CONNECTED)
    return;

  Serial.print("Connecting to MQTT...");
  String clientId = "ESP32-" + String(random(0xffff), HEX);

  if (client.connect(clientId.c_str()))
  {
    Serial.println("connected");
    for (char *topic : topics)
    {
      client.subscribe(topic);
    }
  }
}

void setup_req()
{
  std::string payload = parsePubJSON("msg", "REQUEST_DATA_BARANG");
  client.publish(pubTopics[1], payload.c_str());
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  setupGates();

  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CONVEYOR_PIN, OUTPUT);
  pinMode(TRIG1, INPUT);
  pinMode(TRIG2, INPUT);
  pinMode(TRIG3, INPUT);

  randomSeed(micros());
  setupWIFI();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(30);
  client.setSocketTimeout(5);

  delay(1000);
  reconnect();

  xMutex = xSemaphoreCreateMutex();
  conveyorMutex = xSemaphoreCreateMutex();

  setup_req();
  setupMotors();

  // Buat Mutex Semaphore

  // Pemisahan Task RTOS Multi-core ESP32
  xTaskCreatePinnedToCore(vTaskNetwork, "TaskNetwork", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTaskLogicAndHardware, "TaskHardware", 4096, NULL, 2, NULL, 1);
  // xTaskCreatePinnedToCore(vTaskUltrasonic, "Ultrasonic", 4096, NULL, 0, NULL, 0);
  // xTaskCreatePinnedToCore(vTaskServo, "Servo", 4096, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(vTaskConveyorServo, "ConveyorServo", 4096, NULL, 0, NULL, 0);
}

// ================= LOOP (KOSONG) =================
void loop()
{
}

// ================= FREE RTOS TASKS DEFINITIONS =================

void vTaskNetwork(void *pvParameters)
{
  for (;;)
  {
    if (!client.connected())
    {
      if (millis() - lastReconnectAttempt > 5000)
      {
        lastReconnectAttempt = millis();
        reconnect();
      }
    }
    else
    {
      client.loop();
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Jeda 10ms agar core 0 tidak watchdog reset
  }
}

void vTaskLogicAndHardware(void *pvParameters)
{
  for (;;)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      // runMotors();
      // processQueue();
      // updateGates();
      // controlConveyor();
      // processConveyorEnd();

      runMotors();
      processQueue();
      updateGates();
      controlConveyor();

      xSemaphoreGive(xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // Tick rate cepat (2ms) untuk kehalusan runMotors() berkelanjutan
  }
}

void vTaskConveyorServo(void *pvParameters)
{
  for (;;)
  {
    if (xSemaphoreTake(conveyorMutex, portMAX_DELAY) == pdTRUE)
    {
      if (!conveyorQueue.empty())
      {
        ConveyorTask ct = conveyorQueue.front();

        bool distance = false;

        switch (ct.gate)
        {
        case 1:
          if (digitalRead(TRIG1) == HIGH)
          {
            distance = true;
            detect_to_nodetect = true;
          }
          else
            detect_to_nodetect = false;
          break;

        case 2:
          if (digitalRead(TRIG2) == HIGH)
          {
            distance = true;
            detect_to_nodetect = true;
          }
          else
            detect_to_nodetect = false;
          break;

        case 3:
          if (digitalRead(TRIG3) == HIGH)
          {
            distance = true;
            detect_to_nodetect = true;
          }
          else
            detect_to_nodetect = false;
          break;
        }

        if (distance == true)
        {
          buffer_ultrasonic++;
        }
        else
        {
          if (buffer_ultrasonic > 0)
            buffer_ultrasonic--;
        }
        Serial.printf("Ultrasonic Gate %d: %.2f cm, Buffer: %d\n", ct.gate, distance, buffer_ultrasonic);

        if (distance < 12 && buffer_ultrasonic >= threshold_buffer_ultrasonic)
        {
          Serial.printf("Ultrasonic Gate %d: %.2f cm, Dorong ke Gate %d\n", ct.gate, distance, ct.gate);
          digitalWrite(CONVEYOR_PIN, LOW);

          xSemaphoreGive(conveyorMutex);

                    // test
          if (prev_detect_to_nodetect == true && detect_to_nodetect == false)
          {
            vTaskDelay(pdMS_TO_TICKS(pushDelay));
            xSemaphoreTake(conveyorMutex, portMAX_DELAY);

            openGate(ct.gate);

            conveyorQueue.pop();
            buffer_ultrasonic = 0;
          }

          prev_detect_to_nodetect = detect_to_nodetect;
        }
      }

      xSemaphoreGive(conveyorMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}