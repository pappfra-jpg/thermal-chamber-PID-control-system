#include "DHT22Sensor.h"

DHT22Sensor::DHT22Sensor(uint8_t gpio_pin) 
    : m_pin(gpio_pin), m_last_status(DHT_OK) {
    m_reading.temperature = 0.0f;
    m_reading.humidity = 0.0f;
    m_reading.is_valid = false;
    m_reading.last_read_time = 0;
}

void DHT22Sensor::begin() {
    dht22_init(&m_config, m_pin);
}

bool DHT22Sensor::update() {
    m_last_status = dht22_read(&m_config, &m_reading);
    return (m_last_status == DHT_OK);
}