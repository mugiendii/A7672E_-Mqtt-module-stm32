/**
 * @file  payload.h
 * @brief JSON payload builder for MQTT uplinks.
 *
 * Matches the payload schema used by the A7670E MQTT reference project
 * (mugiendii/A7670E_-Mqtt-Lilygo-module).  Fields left as empty strings
 * are omitted from the JSON output.
 *
 * Topics (HiveMQ Cloud):
 *   Uplink   — mpesa/transactions
 *   Heartbeat— device/heartbeat
 *   Status   — device/status  ("online" / "offline")
 *   Downlink — device/commands  (subscribe, receive JSON commands)
 */

#ifndef PAYLOAD_H
#define PAYLOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Transaction payload fields ──────────────────────────────────────── */

typedef struct {
    char     code[13];         /**< M-Pesa transaction code, e.g. "UAD3C3UU9F" */
    char     type[9];          /**< "received" or "sent"                        */
    char     amount[16];       /**< Monetary amount string, e.g. "128.00"       */
    char     sender[32];       /**< Sender name (populated on "received")        */
    char     sender_phone[16]; /**< Sender phone number                          */
    char     recipient[32];    /**< Recipient name (populated on "sent")         */
    char     balance[16];      /**< Account balance after transaction            */
    char     ts[32];           /**< Timestamp, e.g. "26/01/13,18:02:52+12"      */
    uint32_t uptime_ms;        /**< Device uptime in milliseconds               */
} Payload_Transaction_t;

/* ── API ─────────────────────────────────────────────────────────────── */

/**
 * @brief  Build the transaction JSON payload.
 * @param  buf   Output buffer
 * @param  size  Buffer size in bytes
 * @param  tx    Pointer to filled transaction struct
 * @return Number of bytes written (excluding null terminator), 0 on error.
 *
 * Example output:
 *   {"code":"UAD3C3UU9F","type":"received","amount":"128.00",
 *    "sender":"Owen Wafula","sender_phone":"0115094171",
 *    "balance":"342.89","ts":"26/01/13,18:02:52+12","uptime_ms":12345}
 */
uint16_t Payload_BuildTransaction(char *buf, uint16_t size,
                                  const Payload_Transaction_t *tx);

/**
 * @brief  Build the heartbeat payload: {"alive":1}
 */
uint16_t Payload_BuildHeartbeat(char *buf, uint16_t size);

/**
 * @brief  Build the device status payload: {"status":"online"} or {"status":"offline"}
 * @param  online  1 = online, 0 = offline
 */
uint16_t Payload_BuildStatus(char *buf, uint16_t size, uint8_t online);

#ifdef __cplusplus
}
#endif

#endif /* PAYLOAD_H */
