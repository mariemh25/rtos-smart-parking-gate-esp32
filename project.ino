#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// --- Hardware Pins ---
#define TRIG_PIN           23
#define ECHO_PIN           22
#define SERVO_PIN          26
#define PIN_EMERGENCY_BTN  32 
#define PIN_RED_LED        2   
#define PIN_GREEN_LED      4  
#define DISTANCE_THRESHOLD 60 
#define OPEN_ANGLE         90 
#define CLOSED_ANGLE       0 

// --- Configuration ---
#define STACK_CHECK_INTERVAL 5000 

// --- RTOS Objects ---
QueueHandle_t decisionQueue;
QueueHandle_t gateQueue;
SemaphoreHandle_t emergencySem;
SemaphoreHandle_t serialMutex; 
SemaphoreHandle_t servoMutex;        
volatile bool gateBusy = false;      
volatile bool isGateOpen = false;    

Servo barrierServo;

// --- Data Structures ---
struct SensorData {
  float distance_cm;
  unsigned long detection_timestamp;
  bool is_departure; 
};

struct CommandMsg {
  bool open_gate;
  unsigned long origin_timestamp;
};

// ==========================================
// Helper Functions
// =========================================
void safePrint(const String &msg) {
  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
    Serial.println(msg);
    xSemaphoreGive(serialMutex);
  }
}

void printStackHighWaterMark(const char* taskName) {
  UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
  safePrint(String("[Stack Check] ") + taskName + ": " + String(highWaterMark) + " words free");
}

// ==========================================
// ISRs
// =========================================
void IRAM_ATTR isr_emergency() {
    static uint32_t last_time = 0;
    uint32_t current_time = millis();
    if (current_time - last_time > 250) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(emergencySem, &xHigherPriorityTaskWoken);
        last_time = current_time;
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

// ==========================================
// Tasks
// ==========================================

// --- NEW TASK: Traffic Lights ---
void vTrafficLightTask(void *pvParam) {
  // Setup Pins
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);

  while(1) {
    // Check the global state
    if (isGateOpen) {
      // Gate is Open: GREEN ON, RED OFF
      digitalWrite(PIN_GREEN_LED, HIGH);
      digitalWrite(PIN_RED_LED, LOW);
    } else {
      // Gate is Closed: RED ON, GREEN OFF
      digitalWrite(PIN_GREEN_LED, LOW);
      digitalWrite(PIN_RED_LED, HIGH);
    }

    // Update every 100ms (fast enough for human eye, slow enough to save CPU)
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

// --- Task A: Vehicle Detection ---
void vVehicleDetectionTask(void *pvParam) {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  SensorData data;
  bool car_present_now = false;
  bool car_was_present_before = false;
  uint32_t car_left_timestamp = 0; 
  uint32_t last_stack_check = 0;

  while(1) {
    if (millis() - last_stack_check > STACK_CHECK_INTERVAL) {
      printStackHighWaterMark("DetectionTask");
      last_stack_check = millis();
    }

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    if (duration <= 0) {
      data.distance_cm = 10000.0; 
    } else {
      data.distance_cm = duration * 0.034 / 2.0;
    }

    if (data.distance_cm < DISTANCE_THRESHOLD && data.distance_cm > 0 && data.distance_cm < 1000) {
      car_present_now = true;
    } else {
      car_present_now = false;
    }

    if (car_present_now) {
      car_left_timestamp = millis();
      
      if (!car_was_present_before) {
        data.detection_timestamp = micros(); 
        data.is_departure = false; 
        safePrint("DETECT: New Car (" + String(data.distance_cm) + " cm)");
        
        if (decisionQueue) xQueueSend(decisionQueue, &data, pdMS_TO_TICKS(100));
        car_was_present_before = true; 
      }
    } else {
      if (car_was_present_before && (millis() - car_left_timestamp > 1000)) {
        safePrint("DETECT: Car left.");
        
        data.is_departure = true; 
        data.distance_cm = 0;     
        if (decisionQueue) xQueueSend(decisionQueue, &data, pdMS_TO_TICKS(100));

        car_was_present_before = false; 
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

// --- Task C: Authorization ---
void vAuthorizationTask(void *pvParam) {
  SensorData rxData;
  CommandMsg txMsg;
  uint32_t last_stack_check = 0;

  while(1) {
    if (millis() - last_stack_check > STACK_CHECK_INTERVAL) {
      printStackHighWaterMark("AuthTask");
      last_stack_check = millis();
    }

    if (decisionQueue && xQueueReceive(decisionQueue, &rxData, portMAX_DELAY) == pdTRUE) {
      if (rxData.is_departure) {
         safePrint("AUTH: Car departure detected. Closing gate.");
         txMsg.open_gate = false;
         if (gateQueue) xQueueSend(gateQueue, &txMsg, pdMS_TO_TICKS(200));
      } else {
         if (rxData.distance_cm > 0 && rxData.distance_cm < DISTANCE_THRESHOLD) {
            safePrint("AUTH: Car arrival authorized. Opening gate.");
            txMsg.open_gate = true;
            txMsg.origin_timestamp = rxData.detection_timestamp; 
            if (gateQueue) xQueueSend(gateQueue, &txMsg, pdMS_TO_TICKS(200));
         }
      }
      vTaskDelay(pdMS_TO_TICKS(200)); 
    }
  }
}

// --- Task B: Gate Control ---
void vGateControlTask(void *pvParam) {
  CommandMsg rxCmd;
  uint32_t last_stack_check = 0;

  barrierServo.attach(SERVO_PIN, 500, 2400); 
  barrierServo.write(CLOSED_ANGLE); 
  isGateOpen = false;

  while(1) {
    if (millis() - last_stack_check > STACK_CHECK_INTERVAL) {
      printStackHighWaterMark("GateTask");
      last_stack_check = millis();
    }

    if (gateQueue && xQueueReceive(gateQueue, &rxCmd, portMAX_DELAY) == pdTRUE) {
      gateBusy = true; 

      if (rxCmd.open_gate) {
        if (!isGateOpen) {
           unsigned long current_time = micros();
           unsigned long latency = current_time - rxCmd.origin_timestamp;
           safePrint("GATE: Opening... [Latency: " + String(latency / 1000.0) + " ms]");

           if (servoMutex && xSemaphoreTake(servoMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
             barrierServo.write(OPEN_ANGLE); 
             xSemaphoreGive(servoMutex);
             isGateOpen = true; 
           }
        } else {
           safePrint("GATE: Already Open.");
        }
      } 
      else {
        if (isGateOpen) {
           safePrint("GATE: Closing...");
           if (servoMutex && xSemaphoreTake(servoMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
             barrierServo.write(CLOSED_ANGLE); 
             xSemaphoreGive(servoMutex);
             isGateOpen = false; 
           }
        } else {
           safePrint("GATE: Already Closed.");
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1000)); 
      gateBusy = false; 
    }
  }
}

// --- Task D: Emergency ---
void vEmergencyTask(void *pvParam) {
  pinMode(PIN_EMERGENCY_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_BTN), isr_emergency, FALLING);

  while(1) {
    if (xSemaphoreTake(emergencySem, portMAX_DELAY) == pdTRUE) {
      safePrint("!!! EMERGENCY TOGGLE TRIGGERED !!!");
      gateBusy = true;

      if (servoMutex && xSemaphoreTake(servoMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        if (isGateOpen) {
          safePrint("EMER: Flipping to CLOSED");
          barrierServo.write(CLOSED_ANGLE);
          isGateOpen = false;
        } else {
          safePrint("EMER: Flipping to OPEN");
          barrierServo.write(OPEN_ANGLE);
          isGateOpen = true;
        }
        xSemaphoreGive(servoMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(500)); 
      gateBusy = false;
    }
  }
}

// ==========================================
// Setup
// =========================================
void setup() {
  Serial.begin(115200);
  
  barrierServo.attach(SERVO_PIN, 500, 2400);
  barrierServo.write(CLOSED_ANGLE); 
  delay(500); 

  Serial.println("--- Parking System: Sensor + Emergency + Traffic Lights ---");

  decisionQueue = xQueueCreate(10, sizeof(SensorData));
  gateQueue     = xQueueCreate(5, sizeof(CommandMsg));
  emergencySem  = xSemaphoreCreateBinary();
  serialMutex   = xSemaphoreCreateMutex(); 
  servoMutex    = xSemaphoreCreateMutex(); 

  // Create Tasks
  xTaskCreatePinnedToCore(vEmergencyTask, "Emergency", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vGateControlTask, "Gate", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vVehicleDetectionTask, "Detection", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAuthorizationTask, "Auth", 4096, NULL, 1, NULL, 1);
  
  // NEW: Create Traffic Light Task (Priority 1 is fine)
  xTaskCreatePinnedToCore(vTrafficLightTask, "Traffic", 1024, NULL, 1, NULL, 1);

  vTaskDelete(NULL);
}

void loop() {}