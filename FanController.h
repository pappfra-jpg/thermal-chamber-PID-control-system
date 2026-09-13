#ifndef FAN_CONTROLLER_H
#define FAN_CONTROLLER_H

#include <Arduino.h>

class FanController {
private:
    uint8_t _pin;
    uint8_t _channel;
    uint32_t _freq;
    uint8_t _resolution;
    uint8_t _minPwm; // Sostituire con la tua soglia (es. 128 per il 50%)

public:
    FanController(uint8_t pin, uint8_t channel = 0, uint32_t freq = 5000, uint8_t resolution = 8);
    void begin(uint8_t minPwm = 128);
    void setSpeedPercent(float percent); // Accetta da 0.0 a 100.0%
    void setPWM(uint8_t pwm);            // Accetta da 0 a 255
    void stop();
};

#endif