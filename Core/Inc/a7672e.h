/**
 * @file    a7672e.h
 * @brief   A7672E LTE modem driver for STM32 HAL
 *
 * Provides:
 *   - UART interrupt-driven receive ring buffer
 *   - AT command send / response with timeout
 *   - Modem power-on and network initialisation
 *   - Native MQTT (CMQTT) connect, subscribe, publish
 *   - Non-blocking URC handler for downlink messages
 *
 * Wiring assumed (STM32F411CEUx):
 *   USART2  PA2  → modem RXD        (modem ← STM32)
 *           PA3  → modem TXD        (modem → STM32)
 *   USART1  PA9/PA10               → debug / printf
 *   PWRKEY  PA8  → modem PWRKEY    (add as GPIO_Output in .ioc)
 *
 * Setup checklist before calling A7672E_Init():
 *   1. Add PA8 as GPIO_Output (label: MODEM_PWRKEY) in STM32CubeIDE .ioc.
 *   2. Enable "USART2 global interrupt" in NVIC settings (or leave it — the
 *      driver will call HAL_NVIC_EnableIRQ(USART2_IRQn) itself).
 *   3. In stm32f4xx_it.c USER CODE section, add:
 *        void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
 *   4. In main.c, add the weak-override callback:
 *        void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *            if (huart->Instance == USART2) A7672E_UART_RxCpltCallback();
 *        }
 */

#ifndef A7672E_H
#define A7672E_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdint.h>

/* ── User configuration ────────────────────────────────────────────────────
 * Adjust if your schematic differs.
 * ──────────────────────────────────────────────────────────────────────── */

/** UART handle connected to the modem (USART2: PA2 TX / PA3 RX) */
#define A7672E_UART_HANDLE       huart2

/**
 * PWRKEY GPIO — set LOW ~100 ms then HIGH to boot the module.
 * Comment out A7672E_PWRKEY_PIN if the modem is always powered (no key line).
 * The pin must be configured as GPIO_Output in .ioc / MX_GPIO_Init.
 */
#define A7672E_PWRKEY_PORT       PWR_KEY_GPIO_Port
#define A7672E_PWRKEY_PIN        PWR_KEY_Pin

/** RX ring buffer size — MUST be a power of 2, minimum 512 */
#define A7672E_RX_BUF_SIZE       1024U

/** Max length of a single AT response line (bytes) */
#define A7672E_LINE_BUF_SIZE     256U

/** Max MQTT topic string length (bytes, null-terminated) */
#define A7672E_MQTT_TOPIC_LEN    128U

/** Max MQTT payload size (bytes) */
#define A7672E_MQTT_PAYLOAD_LEN  512U

/* ── Types ─────────────────────────────────────────────────────────────── */

typedef enum {
    A7672E_OK = 0,   /**< Success                               */
    A7672E_ERR,      /**< Generic AT / protocol error           */
    A7672E_TIMEOUT,  /**< No response within the timeout window */
    A7672E_NO_SIM,   /**< SIM absent or PIN-locked              */
    A7672E_NO_NET,   /**< Could not register on network         */
} A7672E_Status_t;

/** MQTT broker / credential bundle */
typedef struct {
    char     broker[64];        /**< Hostname or IP of MQTT broker              */
    uint16_t port;              /**< TCP port (1883 plain / 8883 TLS)           */
    char     client_id[32];     /**< MQTT client identifier (must be unique)    */
    char     username[32];      /**< Broker username  (empty if not needed)     */
    char     password[32];      /**< Broker password  (empty if not needed)     */
    uint8_t  use_ssl;           /**< 0 = plain TCP, 1 = TLS/SSL                */
    /* Optional last-will message — set topic[0]='\0' to disable */
    char     will_topic[64];    /**< Last-will topic   (e.g. "device/status")   */
    char     will_payload[32];  /**< Last-will payload (e.g. "offline")         */
    uint8_t  will_qos;          /**< Last-will QoS (0/1/2)                      */
    uint8_t  will_retain;       /**< Last-will retain flag                      */
} A7672E_MqttConfig_t;

/**
 * Callback fired when a downlink arrives on a subscribed topic.
 *
 * @param topic       Null-terminated topic string
 * @param payload     Payload bytes (null-terminated for convenience)
 * @param payload_len Payload length in bytes (excludes null terminator)
 */
typedef void (*A7672E_MqttMsgCb_t)(const char *topic,
                                    const char *payload,
                                    uint16_t    payload_len);

/**
 * Callback fired when the broker drops the connection
 * (+CMQTTCONNLOST / +CMQTTNONET URC).
 * Use it to re-run A7672E_MqttConnect() + A7672E_MqttSubscribe().
 */
typedef void (*A7672E_MqttLostCb_t)(void);

/* ── Public API ────────────────────────────────────────────────────────── */

/**
 * @brief Initialise the driver: clear ring buffer, enable USART2 IRQ,
 *        arm the first single-byte IT reception.
 *        Call once after MX_USART2_UART_Init() in main().
 */
void A7672E_Init(void);

/**
 * @brief Pulse PWRKEY to boot the module (~100 ms LOW pulse).
 *        No-op when A7672E_PWRKEY_PIN is not defined.
 *        Follow with A7672E_WaitReady() to confirm modem is alive.
 */
void A7672E_PowerOn(void);

/**
 * @brief Poll "AT" until the modem responds "OK".
 * @param timeout_ms  Total wait time in ms (recommend 10 000)
 * @return A7672E_OK on success, A7672E_TIMEOUT if modem doesn't respond.
 */
A7672E_Status_t A7672E_WaitReady(uint32_t timeout_ms);

/**
 * @brief Full network init: SIM check → PIN unlock (if needed) → LTE registration → APN → PDP activate.
 * @param pin  SIM PIN string (NULL or empty to skip PIN entry)
 * @param apn  APN string for your SIM (e.g. "internet", "iot.1nce.net")
 * @return A7672E_OK, A7672E_NO_SIM, A7672E_NO_NET, or A7672E_ERR
 */
A7672E_Status_t A7672E_InitNetwork(const char *pin, const char *apn);

/**
 * @brief Upload a PEM CA certificate to the modem filesystem and configure
 *        SSL context 0.  Call once after A7672E_WaitReady(), before
 *        A7672E_MqttConnect() when use_ssl = 1.
 * @param pem      PEM string (from cacert.h — CACERT_PEM)
 * @param pem_len  Length of PEM string (CACERT_PEM_LEN)
 * @return A7672E_OK on success
 */
A7672E_Status_t A7672E_TlsUploadCert(const char *pem, uint16_t pem_len);

/**
 * @brief Connect to an MQTT broker using the modem's native CMQTT stack.
 *        Tears down any stale session first.
 *        If cfg->will_topic is set, sends AT+CMQTTWILL before connecting.
 * @param cfg  Pointer to broker / credential configuration
 * @return A7672E_OK on success
 */
A7672E_Status_t A7672E_MqttConnect(const A7672E_MqttConfig_t *cfg);

/**
 * @brief Subscribe to a topic (call after A7672E_MqttConnect).
 * @param topic  Null-terminated topic (MQTT wildcards + and # supported)
 * @param qos    Quality of service: 0, 1, or 2
 * @return A7672E_OK on success
 */
A7672E_Status_t A7672E_MqttSubscribe(const char *topic, uint8_t qos);

/**
 * @brief Publish an uplink message.
 * @param topic    Null-terminated topic string
 * @param payload  Null-terminated payload string
 * @param qos      0, 1, or 2
 * @param retain   0 = normal, 1 = retained message
 * @return A7672E_OK on success
 */
A7672E_Status_t A7672E_MqttPublish(const char *topic, const char *payload,
                                    uint8_t qos, uint8_t retain);

/**
 * @brief Register the downlink message callback.
 *        Called from A7672E_Process() when +CMQTTRX* URCs complete.
 */
void A7672E_MqttSetMsgCallback(A7672E_MqttMsgCb_t cb);

/**
 * @brief Register the connection-lost callback.
 */
void A7672E_MqttSetLostCallback(A7672E_MqttLostCb_t cb);

/**
 * @brief Drive the URC state machine — call every main-loop iteration.
 *        Drains the RX ring buffer and dispatches registered callbacks.
 */
void A7672E_Process(void);

/**
 * @brief Forward HAL_UART_RxCpltCallback here for the modem UART.
 *        Implement this in main.c:
 *
 *          void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *              if (huart->Instance == USART2) A7672E_UART_RxCpltCallback();
 *          }
 */
void A7672E_UART_RxCpltCallback(void);

/* Extern UART handles — defined by CubeIDE in main.c */
extern UART_HandleTypeDef huart1;   /* debug / printf */
extern UART_HandleTypeDef huart2;   /* modem          */

#ifdef __cplusplus
}
#endif

#endif /* A7672E_H */
