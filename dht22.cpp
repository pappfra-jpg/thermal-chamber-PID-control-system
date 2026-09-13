/**
 * @file dht22.cpp
 * @brief Implementazione del driver per sensore DHT22 su ESP32
 */

#include <Arduino.h>
#include "dht22.h"

/* ========================================================================= */
/*                       COSTANTI E CONFIGURAZIONI                           */
/* ========================================================================= */

#define DHT22_MIN_INTERVAL_MS   2000    // Intervallo minimo tra letture (2 secondi)
#define DHT22_RESPONSE_TIMEOUT  100     // Timeout in microsecondi per ogni impulso
#define DHT22_BIT_THRESHOLD_US  50      // Impulso > 50us = BIT 1, < 50us = BIT 0

/* ========================================================================= */
/*                       FUNZIONE AUSILIARIA PRIVATA                         */
/* ========================================================================= */

/**
 * @brief Attende che un pin mantenga un certo livello logico e ne misura la durata.
 * 
 * @param gpio_pin Pin dell'ESP32 da monitorare
 * @param level Livello logico da attendere (HIGH = 1, LOW = 0)
 * @param timeout_us Tempo massimo di attesa in microsecondi
 * @return uint32_t Durata dell'impulso in microsecondi, oppure 0 se va in timeout
 */
static uint32_t expect_pulse(uint8_t gpio_pin, uint8_t level, uint32_t timeout_us) {
    uint32_t start_time = micros();
    
    while (digitalRead(gpio_pin) == level) {
        if ((micros() - start_time) > timeout_us) {
            return 0; // Se supera il timeout, restituisce 0 (errore)
        }
    }
    return micros() - start_time;
}

/* ========================================================================= */
/*                       FUNZIONI PUBBLICHE DEL DRIVER                       */
/* ========================================================================= */

void dht22_init(dht_config_t *config, uint8_t gpio_pin) {
    if (config == NULL) return;

    config->gpio_pin = gpio_pin;
    
    // Configura il pin inizialmente in INPUT con resistenza di pull-up
    pinMode(config->gpio_pin, INPUT_PULLUP);
}

dht_status_t dht22_read(const dht_config_t *config, dht_reading_t *reading) {
    // 0. Controlli di sicurezza sui puntatori
    if (config == NULL || reading == NULL) {
        return DHT_ERROR_INVALID_PIN;
    }

    // 1. Controllo Frequenza di Lettura (Rate Limit)
    uint32_t current_time = millis();
    if (reading->last_read_time > 0 && (current_time - reading->last_read_time) < DHT22_MIN_INTERVAL_MS) {
        return DHT_ERROR_TOO_QUICK;
    }

    uint8_t data[5] = {0, 0, 0, 0, 0};

    // -----------------------------------------------------------------
    // FASE 1: Segnale di START dall'ESP32 al Sensore
    // -----------------------------------------------------------------
    pinMode(config->gpio_pin, OUTPUT);
    digitalWrite(config->gpio_pin, LOW);
    delay(2); // Mantiene il segnale LOW per 2 ms per svegliare il DHT22

    digitalWrite(config->gpio_pin, HIGH);
    delayMicroseconds(30); // Rilascia la linea in HIGH per 30 microsecondi
    
    pinMode(config->gpio_pin, INPUT_PULLUP);

    // -----------------------------------------------------------------
    // FASE 2: Risposta del Sensore (ACK) & Protezione Interrupt
    // -----------------------------------------------------------------
    // Blocchiamo temporaneamente gli interrupt della CPU dell'ESP32
    // per evitare sfasamenti nei tempi dei microsecondi!
    noInterrupts();

    // Il DHT22 deve rispondere portando la linea a LOW per 80us e poi HIGH per 80us
    if (expect_pulse(config->gpio_pin, LOW, DHT22_RESPONSE_TIMEOUT) == 0 ||
        expect_pulse(config->gpio_pin, HIGH, DHT22_RESPONSE_TIMEOUT) == 0) {
        interrupts(); // Riattiva gli interrupt prima di uscire!
        reading->is_valid = false;
        return DHT_ERROR_TIMEOUT;
    }

    // -----------------------------------------------------------------
    // FASE 3: Lettura dei 40 Bit (5 Byte)
    // -----------------------------------------------------------------
    

    for (int i = 0; i < 40; i++) {
        // Ciascun bit inizia con un impulso LOW di circa 50us
        if (expect_pulse(config->gpio_pin, LOW, DHT22_RESPONSE_TIMEOUT) == 0) {
            interrupts();
            reading->is_valid = false;
            return DHT_ERROR_TIMEOUT;
        }

        // Misuriamo quanto dura l'impulso HIGH successivo
        uint32_t high_duration = expect_pulse(config->gpio_pin, HIGH, DHT22_RESPONSE_TIMEOUT);
        if (high_duration == 0) {
            interrupts();
            reading->is_valid = false;
            return DHT_ERROR_TIMEOUT;
        }

        // Se l'impulso HIGH è più lungo di 50us è un 1 logico, altrimenti è uno 0
        uint8_t byte_index = i / 8;
        data[byte_index] <<= 1; // Sposta i bit a sinistra di 1 posizione
        
        if (high_duration > DHT22_BIT_THRESHOLD_US) {
            data[byte_index] |= 1; // Imposta l'ultimo bit a 1
        }
    }

    // Finito il timing ad alta precisione, riattiviamo subito gli interrupt
    interrupts();

    // -----------------------------------------------------------------
    // FASE 4: Controllo Checksum e Conversione Matematica
    // -----------------------------------------------------------------

    //I 5 Byte letti in data[5] contengono:

    //Byte 0 e 1: Umidità (moltiplicata per 10).

    //Byte 2 e 3: Temperatura (moltiplicata per 10).

    //Byte 4: Checksum di verifica.
    
    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (data[4] != checksum) {
        reading->is_valid = false;
        return DHT_ERROR_CHECKSUM;
    }

    // Calcolo Umidità Relativa (Byte 0 e 1)
    uint16_t raw_humidity = ((uint16_t)data[0] << 8) | data[1];
    reading->humidity = raw_humidity * 0.1f;

    // Calcolo Temperatura (Byte 2 e 3)
    uint16_t raw_temperature = ((uint16_t)(data[2] & 0x7F) << 8) | data[3];
    reading->temperature = raw_temperature * 0.1f;

    // Se il bit più significativo del Byte 2 è 1, la temperatura è sotto zero
    if (data[2] & 0x80) {
        reading->temperature *= -1.0f;
    }

    // Aggiorna lo stato della lettura nella struct dell'utente
    reading->is_valid = true;
    reading->last_read_time = millis();

    return DHT_OK;
}

// Conversione Errori in Testo

const char* dht22_status_to_string(dht_status_t status) {
    switch (status) {
        case DHT_OK:                 return "OK";
        case DHT_ERROR_TIMEOUT:      return "Errore: Timeout (Sensore non risponde)";
        case DHT_ERROR_CHECKSUM:     return "Errore: Checksum corrotto (disturbo sul cavo)";
        case DHT_ERROR_TOO_QUICK:    return "Errore: Lettura troppo frequente (< 2 sec)";
        case DHT_ERROR_INVALID_PIN:  return "Errore: Pin o puntatore non valido";
        default:                     return "Errore: Sconosciuto";
    }
}