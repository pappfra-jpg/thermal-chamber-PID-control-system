#include <Arduino.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "DHT22Sensor.h"
#include "FanController.h"
#include "PIDController.h"

// ==========================================
// NETWORK CONFIGURATION
// ==========================================
const char* WIFI_SSID     = "YOUR WIFI";
const char* WIFI_PASSWORD = "YOUR PASSWORD";
const char* UDP_ADDRESS   = "192.168.1.21"; 
const int   UDP_PORT      = 5005;
WiFiUDP udp;

// ==========================================
// HARDWARE & PIN CONFIGURATION
// ==========================================
const uint8_t DHT_PIN       = 17;
const uint8_t FAN_PIN       = 26;
#define       WDT_TIMEOUT_S 10

// ==========================================
// SAFETY TIMINGS & OPERATING LIMITS (SR-01, SR-02, SR-06)
// ==========================================
const uint32_t SENSOR_TIMEOUT_MS = 6000; // SR-01/SR-06: 3 consecutive missed samples (DHT22 interval ~2s)
const float    SENSOR_T_MIN       = -40.0f; // SR-02: Minimum sensor limit
const float    SENSOR_T_MAX       = 80.0f;  // SR-02: Maximum sensor limit (TMAX)

// ==========================================
// HARDWARE PHYSICAL CONSTANTS (For Feedforward)
// ==========================================
const float MAX_RAMP_RATE_UP   = 0.0275f; // (°C/s)
const float MAX_RAMP_RATE_DOWN = 0.0160f; // (°C/s)

// ==========================================
// MULTI-STEP PROFILE STRUCTURE & TABLE
// ==========================================
enum StepType {
    STEP_RAMP, 
    STEP_HOLD  
};

struct ThermalStep {
    StepType type;
    float targetTemp;   
    float rampRate;     
    uint32_t holdTimeMs;
};

const ThermalStep PROFILE[] = {
    { STEP_RAMP, 37.0f, MAX_RAMP_RATE_UP / 1.5f, 0 },        
    { STEP_HOLD, 37.0f, 0.0f,                    4 * 60 * 1000 },
    { STEP_RAMP, 43.0f, MAX_RAMP_RATE_UP / 1.5f, 0 },       
    { STEP_HOLD, 43.0f, 0.0f,                    4 * 60 * 1000 },
    { STEP_RAMP, 40.0f, MAX_RAMP_RATE_DOWN / 2.0f, 0 },        
    { STEP_HOLD, 40.0f, 0.0f,                    3 * 60 * 1000 },
    { STEP_RAMP, 37.0f, MAX_RAMP_RATE_DOWN / 2.0f, 0 },
    { STEP_HOLD, 37.0f, 0.0f,                    5 * 60 * 1000 }
};

const uint8_t TOTAL_STEPS = sizeof(PROFILE) / sizeof(PROFILE[0]);

// Updated FSM state enum including STATE_FAULT (SR-02, SR-04)
enum FsmState {
    STATE_RAMP,
    STATE_HOLD,
    STATE_FINISHED,
    STATE_FAULT
};

// ==========================================
// OBJECT INITIALIZATIONS
// ==========================================
DHT22Sensor   dht(DHT_PIN);
FanController fan(FAN_PIN);
PIDController pi(150.0f, 0.3f, 0.0f); 

// ==========================================
// SHARED RESOURCES
// ==========================================
float g_temperature = 0.0f;
float g_humidity    = 0.0f;
float g_fan_percent = 0.0f;
float g_setpoint    = 0.0f;
uint8_t g_system_state = 0; // 0: Normal, 1: Finished, 2: Fault (SR-04 Telemetry)

SemaphoreHandle_t xMutexRAM;

// ==========================================
// TASK: THERMAL CONTROL (20 Hz Loop with Safety Handler)
// ==========================================
void TaskThermalControl(void *pvParameters) {
    esp_task_wdt_add(NULL);
    dht.begin();
    
    // 1. BOOTSTRAP: Initial valid reading setup with timeout retry limits
    float initialTemp = 0.0f;
    float initialHum  = 0.0f;
    uint32_t bootStart = millis();

    while (initialTemp == 0.0f) {
        esp_task_wdt_reset();
        if (dht.update()) {
            float temp = dht.getTemperature();
            float hum  = dht.getHumidity();
            if (!isnan(temp) && !isnan(hum) && temp >= SENSOR_T_MIN && temp <= SENSOR_T_MAX) {
                initialTemp = temp;
                initialHum  = hum;
            }
        }
        
        // SR-01 / SR-04: Boot failure timeout forces STATE_FAULT immediately
        if (millis() - bootStart > SENSOR_TIMEOUT_MS) {
            Serial.println("[SAFETY CRITICAL] Sensor failed during boot initialization! Transitioning to STATE_FAULT.");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    float currentSetpoint = (initialTemp > 0.0f) ? initialTemp : 25.0f;
    uint8_t currentStepIndex = 0;
    
    FsmState currentState = (initialTemp == 0.0f) ? STATE_FAULT : 
                            ((PROFILE[0].type == STEP_RAMP) ? STATE_RAMP : STATE_HOLD);
                            
    uint32_t stepStartTimestamp = millis();
    uint32_t lastValidSensorReadTimestamp = millis(); // SR-01 Monitoring heartbeat
    
    // Last valid readings retained for PID/Telemetry during temporary glitches
    float lastValidTemp = initialTemp;
    float lastValidHum  = initialHum;

    // 2. Initialize shared RAM
    if (xSemaphoreTake(xMutexRAM, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_temperature  = initialTemp;
        g_humidity     = initialHum;
        g_setpoint     = currentSetpoint;
        g_fan_percent  = (currentState == STATE_FAULT) ? 100.0f : 0.0f;
        g_system_state = (currentState == STATE_FAULT) ? 2 : 0;
        xSemaphoreGive(xMutexRAM);
    }

    // Deterministic 20 Hz (50 ms) timing configuration
    const uint32_t CONTROL_LOOP_MS = 50;
    const float DT_FIXED = CONTROL_LOOP_MS / 1000.0f; 
    const TickType_t xLoopPeriod = pdMS_TO_TICKS(CONTROL_LOOP_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.printf("[PROFILE] System started. Initial state: %d\n", currentState);

    for (;;) {
        esp_task_wdt_reset();
        uint32_t now = millis();
        float dSP_dt = 0.0f;

        // ----------------------------------------------------
        // SR-01 & SR-02: SENSOR ACQUISITION & VALIDATION
        // ----------------------------------------------------
        bool validNewReading = false;

        if (dht.update()) {
            float rawT = dht.getTemperature();
            float rawH = dht.getHumidity();

            // SR-02: Verification against bounds & NaN data failure
            bool isNaN = isnan(rawT) || isnan(rawH);
            bool isOutOfBounds = (rawT < SENSOR_T_MIN || rawT > SENSOR_T_MAX);

            if (!isNaN && !isOutOfBounds) {
                lastValidTemp = rawT;
                lastValidHum  = rawH;
                validNewReading = true;
                lastValidSensorReadTimestamp = now; // Refresh heartbeat (SR-01)

                // Debug print on successful read update
                Serial.printf("[SENSOR DEBUG] Valid read @ %lu ms | Temp: %.2f °C | LastValidTS: %lu ms\n", 
                              (unsigned long)now, lastValidTemp, (unsigned long)lastValidSensorReadTimestamp);
            } else {
                if (isNaN) {
                    Serial.printf("[SAFETY WARN] Sensor read returned NaN! (Pin disconnected or bad data)\n");
                } else {
                    Serial.printf("[SAFETY WARN] Sensor output out-of-bounds: %.2f °C\n", rawT);
                }
                // IMPORTANT: lastValidSensorReadTimestamp is NOT updated here!
            }
        }

        // Periodic heartbeat monitor (prints every ~1 second)
        static uint32_t lastLogTime = 0;
        if (now - lastLogTime >= 1000) {
            lastLogTime = now;
            uint32_t sensorDataAge = now - lastValidSensorReadTimestamp;
            Serial.printf("[HEARTBEAT] Now: %lu ms | LastValidTS: %lu ms | Data Age: %lu ms / %lu ms limit\n",
                          (unsigned long)now, 
                          (unsigned long)lastValidSensorReadTimestamp, 
                          (unsigned long)sensorDataAge, 
                          (unsigned long)SENSOR_TIMEOUT_MS);
        }

        // SR-01 & SR-06: FAULT DETECTION (Timeout Verification)
        if (now - lastValidSensorReadTimestamp > SENSOR_TIMEOUT_MS) {
            if (currentState != STATE_FAULT) {
                currentState = STATE_FAULT; // SR-02 / SR-06: Transition to fault state
                Serial.printf("[SAFETY CRITICAL] Sensor timeout reached! Data age (%lu ms) > Limit (%lu ms). Entering STATE_FAULT!\n", 
                              (unsigned long)(now - lastValidSensorReadTimestamp), 
                              (unsigned long)SENSOR_TIMEOUT_MS);
            }
        }

        // ----------------------------------------------------
        // 3. FSM & CONTROL EXECUTION
        // ----------------------------------------------------
        switch (currentState) {
            
            case STATE_RAMP: {
                if (currentStepIndex < TOTAL_STEPS) {
                    ThermalStep step = PROFILE[currentStepIndex];
                    float target = step.targetTemp;
                    float rateStep = step.rampRate * DT_FIXED;

                    if (currentSetpoint < target) {
                        dSP_dt = step.rampRate; 
                        currentSetpoint += rateStep;
                        if (currentSetpoint >= target) currentSetpoint = target;
                    } 
                    else if (currentSetpoint > target) {
                        dSP_dt = -step.rampRate; 
                        currentSetpoint -= rateStep;
                        if (currentSetpoint <= target) currentSetpoint = target;
                    }

                    if (fabs(currentSetpoint - target) < 0.001f) {
                        currentStepIndex++;
                        if (currentStepIndex < TOTAL_STEPS) {
                            stepStartTimestamp = now;
                            currentState = (PROFILE[currentStepIndex].type == STEP_RAMP) ? STATE_RAMP : STATE_HOLD;
                        } else {
                            currentState = STATE_FINISHED;
                        }
                    }
                }
                break;
            }

            case STATE_HOLD: {
                if (currentStepIndex < TOTAL_STEPS) {
                    ThermalStep step = PROFILE[currentStepIndex];
                    dSP_dt = 0.0f;
                    currentSetpoint = step.targetTemp;

                    if (now - stepStartTimestamp >= step.holdTimeMs) {
                        currentStepIndex++;
                        if (currentStepIndex < TOTAL_STEPS) {
                            stepStartTimestamp = now;
                            currentState = (PROFILE[currentStepIndex].type == STEP_RAMP) ? STATE_RAMP : STATE_HOLD;
                        } else {
                            currentState = STATE_FINISHED;
                        }
                    }
                }
                break;
            }

            case STATE_FINISHED:
                dSP_dt = 0.0f;
                break;

            case STATE_FAULT:
                // SR-04 SAFE-STATE ACTUATION: Force Actuator to 100% duty cycle
                fan.setSpeedPercent(100.0f);
                dSP_dt = 0.0f;
                break;
        }

        // ----------------------------------------------------
        // 4. CONTROL LOOP CALCULATION (If state is operational)
        // ----------------------------------------------------
        float currentFanCommand = 100.0f;

        if (currentState != STATE_FAULT) {
            pi.setSetpoint(currentSetpoint);
            currentFanCommand = pi.compute(lastValidTemp, dSP_dt);
            fan.setSpeedPercent(currentFanCommand);
        } else {
            currentFanCommand = 100.0f; // SR-04: Override output in fault state
        }

        // ----------------------------------------------------
        // 5. UPDATE SHARED MEMORY FOR TELEMETRY
        // ----------------------------------------------------
        if (xSemaphoreTake(xMutexRAM, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_temperature  = lastValidTemp;
            g_humidity     = lastValidHum;
            g_setpoint     = currentSetpoint;
            g_fan_percent  = currentFanCommand;
            g_system_state = (currentState == STATE_FAULT) ? 2 : ((currentState == STATE_FINISHED) ? 1 : 0);
            xSemaphoreGive(xMutexRAM);
        }

        vTaskDelayUntil(&xLastWakeTime, xLoopPeriod);
    }
}

// ==========================================
// TASK: TELEMETRY (UDP Streaming - Cadenza Fissa 1.0s)
// ==========================================
void TaskTelemetry(void *pvParameters) {
    esp_task_wdt_add(NULL);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000); 
    char packetBuffer[80];

    for (;;) {
        esp_task_wdt_reset();

        float temp_copy = 0.0f, humidity_copy = 0.0f, fan_copy = 0.0f, setpoint_copy = 0.0f;
        uint8_t state_copy = 0;

        if (xSemaphoreTake(xMutexRAM, pdMS_TO_TICKS(20)) == pdTRUE) {
            temp_copy     = g_temperature;
            humidity_copy = g_humidity;
            fan_copy      = g_fan_percent;
            setpoint_copy = g_setpoint;
            state_copy    = g_system_state;
            xSemaphoreGive(xMutexRAM);
        }

        // Telemetry frame includes system fault state ($DATA,temp,hum,setpoint,fan,state)
        snprintf(packetBuffer, sizeof(packetBuffer), "$DATA,%.2f,%.2f,%.2f,%.1f,%u",
                 temp_copy, humidity_copy, setpoint_copy, fan_copy, state_copy);

        if (WiFi.status() == WL_CONNECTED) {
            udp.beginPacket(UDP_ADDRESS, UDP_PORT);
            udp.write((const uint8_t*)packetBuffer, strlen(packetBuffer));
            udp.endPacket();
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ==========================================
// SYSTEM SETUP & INITIALIZATION
// ==========================================
void setup() {
    Serial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi Connected!");
    Serial.println(WiFi.localIP());
    
    udp.begin(UDP_PORT);
    fan.begin(128);

    pi.setPhysicalLimits(MAX_RAMP_RATE_DOWN, MAX_RAMP_RATE_UP);
    pi.begin(0.0f, 0.0f, 100.0f);

    xMutexRAM = xSemaphoreCreateMutex();

    if (xMutexRAM != NULL) {
        esp_task_wdt_init(WDT_TIMEOUT_S, true);

        xTaskCreatePinnedToCore(TaskThermalControl, "TaskThermalControl", 3072, NULL, 3, NULL, 1);
        xTaskCreatePinnedToCore(TaskTelemetry,      "TaskTelemetry",      4096, NULL, 2, NULL, 0);
    }
}

void loop() {
    vTaskDelete(NULL);
}