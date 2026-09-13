#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) 
    : _kp(kp), 
      _ki(ki), 
      _kd(kd), 
      _integral(0.0f), 
      _lastError(0.0f), 
      _lastTime(0), 
      _outMin(0.0f), 
      _outMax(100.0f),
      _setpoint(0.0f),
      _physMaxCoolingRate(0.016f),     // Valore di default misurato (100% PWM)
      _physNaturalHeatingRate(0.027f)  // Valore di default misurato (0% PWM)
{}

void PIDController::begin(float setpoint, float outMin, float outMax) {
    _setpoint = setpoint;
    _outMin = outMin;
    _outMax = outMax;
    reset();
}

void PIDController::setTunings(float kp, float ki, float kd) {
    _kp = kp; 
    _ki = ki; 
    _kd = kd;
}

void PIDController::setPhysicalLimits(float maxCoolingRate, float naturalHeatingRate) {
    _physMaxCoolingRate = maxCoolingRate;
    _physNaturalHeatingRate = naturalHeatingRate;
}

void PIDController::setSetpoint(float setpoint) {
    _setpoint = setpoint;
}

// Compute standard (senza rampa, dSP_dt = 0)
float PIDController::compute(float input) {
    return compute(input, 0.0f);
}

// Compute avanzato con Feedforward basato sulla velocità della rampa (dSP_dt in °C/s)
float PIDController::compute(float input, float dSP_dt) {
    unsigned long now = millis();
    
    // Inizializzazione al primo ciclo
    if (_lastTime == 0) {
        _lastTime = now;
        return 0.0f;
    }

    float dt = (now - _lastTime) / 1000.0f; // dt in secondi
    
    // Protezione contro esecuzioni troppo ravvicinate
    if (dt <= 0.0f) return 0.0f;

    // Per raffreddamento (Ventola): Errore = Temperatura Attuale - Setpoint
    float error = input - _setpoint; 

    // 1. CALCOLO TERMINE PROPORZIONALE
    float pTerm = _kp * error;

    // 2. ACCUMULO INTEGRALE CON ANTI-WINDUP
    _integral += error * dt;
    
    if (_ki > 0.0f) {
        float maxIntegral = _outMax / _ki;
        if (_integral > maxIntegral) _integral = maxIntegral;
        if (_integral < 0.0f) _integral = 0.0f;
    }
    float integralTerm = _ki * _integral;

    // 3. CALCOLO TERMINE DERIVATIVO
    float derivative = (error - _lastError) / dt;
    float dTerm = _kd * derivative;

    // 4. CALCOLO FEEDFORWARD (MODELLO FISICO HARDWARE)
    float ffTerm = 0.0f;

    if (dSP_dt < 0.0f) {
        // DISCESA: Quanta ventola serve per ottenere la velocità richiesta?
        // Es: dSP_dt = -0.008 °C/s -> (0.008 / 0.016) * 100 = 50% PWM
        if (_physMaxCoolingRate > 0.0f) {
            ffTerm = (fabs(dSP_dt) / _physMaxCoolingRate) * 100.0f;
        }
    } 
    else if (dSP_dt > 0.0f) {
        // SALITA: Quanta ventola serve per "frenare" la salita naturale ed evitare overshoot?
        // Es: dSP_dt = +0.018 °C/s -> ((0.027 - 0.018) / 0.027) * 100 = 33.3% PWM
        if (_physNaturalHeatingRate > 0.0f) {
            float brakeRatio = (_physNaturalHeatingRate - dSP_dt) / _physNaturalHeatingRate;
            ffTerm = brakeRatio * 100.0f;
        }
    }

    // Limiti di sicurezza per il solo Feedforward
    ffTerm = constrain(ffTerm, 0.0f, 100.0f);

    // 5. OUTPUT FINALE (Baseline Fisica FF + Correzione Feedback PI/PID)
    float output = pTerm + integralTerm + dTerm + ffTerm;
    
    // Clamping dell'uscita nell'intervallo [outMin, outMax]
    output = constrain(output, _outMin, _outMax);

    _lastError = error;
    _lastTime = now;

    return output;
}

void PIDController::reset() {
    _integral = 0.0f;
    _lastError = 0.0f;
    _lastTime = 0;
}