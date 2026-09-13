#include "FanController.h"

FanController::FanController(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t resolution)
    : _pin(pin), _channel(channel), _freq(freq), _resolution(resolution), _minPwm(122) {}

void FanController::begin(uint8_t minPwm) {
    _minPwm = minPwm;
    ledcSetup(_channel, _freq, _resolution);
    ledcAttachPin(_pin, _channel);
    stop();
}

// Passa un valore PWM diretto (1-255) e lo scala sopra la soglia minima
void FanController::setPWM(uint8_t pwm) {
    if (pwm == 0) {
        stop();
        return;
    }
    // Mappa l'intervallo 1..255 sull'intervallo utile _minPwm..255
    uint8_t effectivePwm = map(pwm, 1, 255, _minPwm, 255);
    ledcWrite(_channel, effectivePwm);
}

// Passa una percentuale (0.0% - 100.0%)
void FanController::setSpeedPercent(float percent) {
    percent = constrain(percent, 0.0f, 100.0f);
    
    if (percent <= 0.0f) {
        stop();
    } else {
        // Convertiamo la percentuale in un PWM da 1 a 255, 
        // e lasciamo che sia setPWM a fare il mapping sulla soglia _minPwm
        uint8_t rawPwm = (uint8_t)map((long)(percent * 10.0f), 10, 1000, 1, 255);
        setPWM(rawPwm);
    }
}

void FanController::stop() {
    ledcWrite(_channel, 0);
}