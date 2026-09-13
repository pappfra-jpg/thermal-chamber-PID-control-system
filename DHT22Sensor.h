
#ifndef DHT22_SENSOR_H
#define DHT22_SENSOR_H

#include "dht22.h"

class DHT22Sensor {
public:
    explicit DHT22Sensor(uint8_t gpio_pin);
    
    // Inizializza l'hardware del sensore
    void begin();
    
    // Tenta di aggiornare le letture se è passato abbastanza tempo
    bool update();

    // Getters puliti (sola lettura)
    float getTemperature() const { return m_reading.temperature; }
    float getHumidity() const    { return m_reading.humidity; }
    const char* getLastError() const { return dht22_status_to_string(m_last_status); }

private:
    uint8_t        m_pin;
    dht_config_t   m_config;
    dht_reading_t  m_reading;
    dht_status_t   m_last_status;
};

#endif // DHT22_SENSOR_H