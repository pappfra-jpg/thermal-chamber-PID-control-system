#pragma once
#include <Arduino.h>

class SetpointProfiler {
private:
    float currentSetpoint;
    float targetSetpoint;

    // Envelope di TARGET sicuro per i test a regime stazionario
    const float MIN_TARGET_TEMP = 37.0f; // °C
    const float MAX_TARGET_TEMP = 43.0f; // °C

    // Pendenze operative
    const float rampRateCooling = 0.030f; // °C/s
    const float rampRateHeating = 0.035f; // °C/s

public:
    // Inizializza il profiler partendo dalla temperatura REALE (es. 28.0 °C ambiente)
    SetpointProfiler(float startTemp) {
        currentSetpoint = startTemp;
        targetSetpoint  = startTemp;
    }

    // Imposta il target finale da raggiungere (garantito tra 37.0 °C e 43.0 °C)
    void setTarget(float targetTemp) {
        // Clamping del target finale per non superare l'envelope sicuro
        if (targetTemp < MIN_TARGET_TEMP) targetTemp = MIN_TARGET_TEMP;
        if (targetTemp > MAX_TARGET_TEMP) targetTemp = MAX_TARGET_TEMP;

        targetSetpoint = targetTemp;
    }

    // Forzare una ripartenza da una temperatura misurata
    void resetStartTemp(float actualTemp) {
        currentSetpoint = actualTemp;
    }

    // Avanza il setpoint lungo la rampa in base al tempo trascorso (dt in secondi)
    float update(float dt) {
        if (currentSetpoint < targetSetpoint) {
            currentSetpoint += rampRateHeating * dt;
            if (currentSetpoint > targetSetpoint) {
                currentSetpoint = targetSetpoint; // Target raggiunto
            }
        } 
        else if (currentSetpoint > targetSetpoint) {
            currentSetpoint -= rampRateCooling * dt;
            if (currentSetpoint < targetSetpoint) {
                currentSetpoint = targetSetpoint; // Target raggiunto
            }
        }
        return currentSetpoint;
    }

    float getCurrentSetpoint() const { return currentSetpoint; }
    float getTargetSetpoint() const { return targetSetpoint; }
    bool isRampComplete() const { return currentSetpoint == targetSetpoint; }
};