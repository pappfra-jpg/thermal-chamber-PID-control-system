#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>

class PIDController {
private:
    float _kp;
    float _ki;
    float _kd;
    float _integral;
    float _lastError;
    unsigned long _lastTime;
    float _outMin;
    float _outMax;
    float _setpoint;

    // Costanti Fisiche Hardware per Calcolo Feedforward
    float _physMaxCoolingRate;    // °C/s a 100% PWM
    float _physNaturalHeatingRate; // °C/s a 0% PWM

public:
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

    void begin(float setpoint = 0.0f, float outMin = 0.0f, float outMax = 100.0f);
    void setTunings(float kp, float ki, float kd);
    void setPhysicalLimits(float maxCoolingRate, float naturalHeatingRate);
    void setSetpoint(float setpoint);

    // Sovraccarico compute(): con e senza derivata del setpoint (dSP_dt in °C/s)
    float compute(float input, float dSP_dt);
    float compute(float input); // Defaults a dSP_dt = 0.0f

    void reset();
};

#endif // PID_CONTROLLER_H