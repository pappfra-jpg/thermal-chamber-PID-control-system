/**
 * @file dht22.h
 * @brief Driver in C puro per sensore DHT22 su ESP32
 * @details Gestisce la lettura ad alta precisione tramite misurazione dei microsecondi,
 *          verifica del checksum e gestione degli errori di comunicazione.
 */

#ifndef DHT22_H
#define DHT22_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                              ENUMERAZIONI                                 */
/* ========================================================================= */

/**
 * @brief Codici di stato e errore restituiti dalle funzioni del driver DHT22.
 */
typedef enum {
    DHT_OK = 0,               /**< Lettura completata con successo */
    DHT_ERROR_TIMEOUT,        /**< Il sensore non ha risposto entro i microsecondi previsti */
    DHT_ERROR_CHECKSUM,       /**< I dati ricevuti non corrispondono al byte di controllo */
    DHT_ERROR_TOO_QUICK,      /**< Tentativo di lettura troppo frequente (meno di 2 secondi dall'ultima) */
    DHT_ERROR_INVALID_PIN     /**< Il pin GPIO specificato non è valido o non configurato */
} dht_status_t;

/* ========================================================================= */
/*                                STRUTTURE                                  */
/* ========================================================================= */

/**
 * @brief Struttura contenente la telemetria letta dal sensore.
 */
typedef struct {
    float temperature;        /**< Temperatura espressa in gradi Celsius (°C) */
    float humidity;           /**< Umidità relativa espressa in percentuale (%) */
    bool is_valid;            /**< Flag: true se la lettura corrente è valida e verificata */
    uint32_t last_read_time;  /**< Timestamp (in ms) dell'ultima lettura riuscita per rispettare il rate-limit */
} dht_reading_t;

/**
 * @brief Struttura di configurazione dell'hardware DHT22.
 */
typedef struct {
    uint8_t gpio_pin;         /**< Numero del pin GPIO dell'ESP32 collegato alla linea DAT */
} dht_config_t;

/* ========================================================================= */
/*                            PROTOTIPI FUNZIONI                             */
/* ========================================================================= */

/**
 * @brief Inizializza la struttura di configurazione e il pin GPIO dell'ESP32.
 * 
 * @param config Puntatore alla struttura di configurazione
 * @param gpio_pin Pin GPIO da assegnare (es. 17, 23, ecc.)
 */
void dht22_init(dht_config_t *config, uint8_t gpio_pin);

/**
 * @brief Effettua una lettura completa dal sensore DHT22.
 * 
 * @param config Puntatore alla configurazione del sensore
 * @param reading Puntatore alla struttura in cui salvare la temperatura e l'umidità
 * @return dht_status_t Stato della lettura (DHT_OK se riuscita, codice di errore altrimenti)
 */
dht_status_t dht22_read(const dht_config_t *config, dht_reading_t *reading);

/**
 * @brief Converte un codice di errore enum in una stringa leggibile (per il debug seriale).
 * 
 * @param status Il codice di errore restituito da dht22_read()
 * @return const char* Stringa descrittiva dell'errore
 */
const char* dht22_status_to_string(dht_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* DHT22_H */