/**
 * @file    a7672e.c
 * @brief   A7672E LTE modem driver — AT command layer + MQTT over CMQTT
 *
 * Architecture:
 *   - USART2 RX runs in 1-byte interrupt mode, feeding a power-of-2 ring
 *     buffer.  All AT helpers drain this buffer; no blocking DMA needed.
 *   - modem_cmd() sends a command and scans lines for the expected token or
 *     "ERROR", returning when either is found or the timeout expires.
 *   - A7672E_Process() (main-loop) runs the 4-state URC machine that
 *     assembles incoming +CMQTTRX* frames and calls the user callback.
 *   - Payload is collected byte-by-byte using the length from
 *     +CMQTTRXSTART so binary payloads containing \r\n are handled safely.
 */

#include "a7672e.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── Ring buffer ─────────────────────────────────────────────────────────
 * Size must be a power of 2 so the & mask works without a branch.
 * ──────────────────────────────────────────────────────────────────────── */

#define RB_MASK  (A7672E_RX_BUF_SIZE - 1u)

static uint8_t           s_rb[A7672E_RX_BUF_SIZE];
static volatile uint16_t s_rb_head = 0;   /* written by ISR  */
static volatile uint16_t s_rb_tail = 0;   /* read  by main   */
static uint8_t           s_rx_byte;       /* single-byte IT target */

static inline uint16_t rb_count(void)
{
    return (s_rb_head - s_rb_tail) & RB_MASK;
}

static inline uint8_t rb_get(void)
{
    uint8_t b = s_rb[s_rb_tail & RB_MASK];
    s_rb_tail++;
    return b;
}

/* Called from HAL_UART_RxCpltCallback when huart->Instance == USART2 */
void A7672E_UART_RxCpltCallback(void)
{
    s_rb[s_rb_head & RB_MASK] = s_rx_byte;
    s_rb_head++;
    HAL_UART_Receive_IT(&A7672E_UART_HANDLE, &s_rx_byte, 1);
}

/* ── Low-level modem I/O ─────────────────────────────────────────────── */

static void modem_write(const char *data, uint16_t len)
{
    HAL_UART_Transmit(&A7672E_UART_HANDLE, (const uint8_t *)data, len, 1000);
}

/* Send AT command string followed by \r\n */
static void modem_send(const char *cmd)
{
    modem_write(cmd, (uint16_t)strlen(cmd));
    modem_write("\r\n", 2);
}

/**
 * Read bytes from ring buffer into dst until '\n' or timeout.
 * Returns the number of bytes placed in dst (including '\n'), 0 on timeout.
 */
static uint16_t modem_read_line(char *dst, uint16_t max, uint32_t timeout_ms)
{
    uint16_t n = 0;
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (rb_count() == 0) continue;

        char c = (char)rb_get();
        if (n < max - 1u) dst[n++] = c;
        if (c == '\n') {
            dst[n] = '\0';
            return n;
        }
    }
    dst[n] = '\0';
    return 0;
}

/**
 * Optionally send cmd (NULL = skip), then wait up to timeout_ms for any
 * response line containing expected.  Accumulates all lines into resp_buf
 * if provided.  Returns A7672E_OK, A7672E_ERR, or A7672E_TIMEOUT.
 */
static A7672E_Status_t modem_cmd(const char *cmd,
                                  const char *expected,
                                  uint32_t    timeout_ms,
                                  char       *resp_buf,
                                  uint16_t    resp_size)
{
    char line[A7672E_LINE_BUF_SIZE];
    uint32_t t0 = HAL_GetTick();

    if (cmd) modem_send(cmd);

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (!modem_read_line(line, sizeof(line), 200)) continue;

        if (resp_buf && resp_size > 0) {
            uint16_t used = (uint16_t)strlen(resp_buf);
            uint16_t room = resp_size - used - 1u;
            if (room > 0) strncat(resp_buf, line, room);
        }
        if (strstr(line, expected)) return A7672E_OK;
        if (strstr(line, "ERROR"))  return A7672E_ERR;
    }
    return A7672E_TIMEOUT;
}

/**
 * Wait for the '>' prompt from the modem, then transmit data.
 * Used for the two-step CMQTTTOPIC / CMQTTPAYLOAD prompt sequences.
 */
static A7672E_Status_t modem_wait_prompt(const char *data, uint16_t len,
                                          uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (rb_count() == 0) continue;
        if ((char)rb_get() == '>') {
            HAL_Delay(20);                      /* brief settle */
            modem_write(data, len);
            return A7672E_OK;
        }
    }
    return A7672E_TIMEOUT;
}

/* ── MQTT URC state machine ──────────────────────────────────────────────
 *
 * The A7672E pushes four consecutive URCs when a subscribed message arrives:
 *
 *   +CMQTTRXSTART: <sess>,<topicLen>,<payloadLen>\r\n
 *   +CMQTTRXTOPIC: <sess>,<topicLen>\r\n
 *   <topic bytes — exactly topicLen, followed by \r\n>
 *   +CMQTTRXPAYLOAD: <sess>,<payloadLen>\r\n
 *   <payload bytes — exactly payloadLen, followed by \r\n>
 *   +CMQTTRXEND: <sess>\r\n
 *
 * Topic is read in line mode (topics never contain \r\n).
 * Payload is read byte-count mode so binary payloads are safe.
 * ──────────────────────────────────────────────────────────────────────── */

typedef enum {
    URC_IDLE = 0,
    URC_TOPIC_HDR,     /* +CMQTTRXSTART seen — waiting for +CMQTTRXTOPIC line */
    URC_TOPIC_DATA,    /* collecting topic bytes (line mode, len-bounded)      */
    URC_PAYLOAD_HDR,   /* topic complete — waiting for +CMQTTRXPAYLOAD line   */
    URC_PAYLOAD_DATA,  /* collecting payload bytes (byte-count mode)           */
} UrcState_t;

static UrcState_t s_urc_state   = URC_IDLE;
static uint16_t   s_topic_len   = 0;
static uint16_t   s_payload_len = 0;
static uint16_t   s_data_idx    = 0;

static char s_topic_buf  [A7672E_MQTT_TOPIC_LEN];
static char s_payload_buf[A7672E_MQTT_PAYLOAD_LEN];

static A7672E_MqttMsgCb_t  s_msg_cb  = NULL;
static A7672E_MqttLostCb_t s_lost_cb = NULL;

/* Line accumulator used by A7672E_Process() for normal lines */
static char     s_line[A7672E_LINE_BUF_SIZE];
static uint16_t s_line_len = 0;

/* Process one complete line from the ring buffer */
static void urc_handle_line(void)
{
    int sess, tlen, plen;

    /* +CMQTTRXSTART: 0,<topicLen>,<payloadLen> */
    if (sscanf(s_line, "+CMQTTRXSTART: %d,%d,%d", &sess, &tlen, &plen) == 3) {
        s_topic_len   = (uint16_t)tlen;
        s_payload_len = (uint16_t)plen;
        s_data_idx    = 0;
        s_urc_state   = URC_TOPIC_HDR;
        return;
    }

    /* +CMQTTRXTOPIC: 0,<topicLen> */
    if (s_urc_state == URC_TOPIC_HDR &&
        sscanf(s_line, "+CMQTTRXTOPIC: %d,%d", &sess, &tlen) == 2) {
        s_data_idx  = 0;
        s_urc_state = URC_TOPIC_DATA;
        return;
    }

    /* Topic data line — collect up to s_topic_len non-CR/LF bytes */
    if (s_urc_state == URC_TOPIC_DATA) {
        uint16_t i;
        for (i = 0; i < s_line_len && s_data_idx < s_topic_len; i++) {
            char c = s_line[i];
            if (c == '\r' || c == '\n') continue;
            if (s_data_idx < A7672E_MQTT_TOPIC_LEN - 1u)
                s_topic_buf[s_data_idx++] = c;
        }
        s_topic_buf[s_data_idx] = '\0';
        s_data_idx  = 0;
        s_urc_state = URC_PAYLOAD_HDR;
        return;
    }

    /* +CMQTTRXPAYLOAD: 0,<payloadLen> */
    if (s_urc_state == URC_PAYLOAD_HDR &&
        sscanf(s_line, "+CMQTTRXPAYLOAD: %d,%d", &sess, &plen) == 2) {
        s_data_idx  = 0;
        s_urc_state = URC_PAYLOAD_DATA;
        /* Payload data follows in byte-count mode — handled per-byte below */
        return;
    }

    /* +CMQTTCONNLOST / +CMQTTNONET — broker connection dropped */
    if (strstr(s_line, "+CMQTTCONNLOST:") || strstr(s_line, "+CMQTTNONET:")) {
        s_urc_state = URC_IDLE;
        if (s_lost_cb) s_lost_cb();
        return;
    }
}

/* ── Public API ──────────────────────────────────────────────────────────
 *
 * TLS note: A7672E_TlsUploadCert() must be called once after the modem is
 * ready (A7672E_WaitReady) and before A7672E_MqttConnect with use_ssl=1.
 * It uploads the PEM to the modem filesystem and binds it to SSL context 0.
 * ──────────────────────────────────────────────────────────────────────── */

void A7672E_Init(void)
{
    s_rb_head   = 0;
    s_rb_tail   = 0;
    s_urc_state = URC_IDLE;
    s_line_len  = 0;

    /* Enable USART2 interrupt and arm the first byte reception */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&A7672E_UART_HANDLE, &s_rx_byte, 1);
}

void A7672E_PowerOn(void)
{
#ifdef A7672E_PWRKEY_PIN
    /* Pull PWRKEY LOW for 100 ms then release HIGH — module begins booting */
    HAL_GPIO_WritePin(A7672E_PWRKEY_PORT, A7672E_PWRKEY_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(A7672E_PWRKEY_PORT, A7672E_PWRKEY_PIN, GPIO_PIN_SET);
    HAL_Delay(1000);   /* allow boot to progress before polling */
#endif
}

A7672E_Status_t A7672E_WaitReady(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (modem_cmd("AT", "OK", 1000, NULL, 0) == A7672E_OK)
            return A7672E_OK;
    }
    return A7672E_TIMEOUT;
}

A7672E_Status_t A7672E_InitNetwork(const char *apn)
{
    char cmd[80];
    char resp[64];

    /* 1 — SIM present and unlocked? */
    if (modem_cmd("AT+CPIN?", "READY", 5000, NULL, 0) != A7672E_OK)
        return A7672E_NO_SIM;

    /* 2 — Wait for LTE EPS registration (stat = 1 home, 5 roaming) */
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 60000u) {
        resp[0] = '\0';
        modem_cmd("AT+CEREG?", "+CEREG:", 2000, resp, sizeof(resp));
        if (strstr(resp, ",1") || strstr(resp, ",5")) goto reg_ok;
        HAL_Delay(2000);
    }
    return A7672E_NO_NET;

reg_ok:
    /* 3 — Set APN and activate PDP context 1 */
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
    modem_cmd(cmd, "OK", 3000, NULL, 0);

    if (modem_cmd("AT+CGACT=1,1", "OK", 10000, NULL, 0) != A7672E_OK)
        return A7672E_ERR;

    return A7672E_OK;
}

A7672E_Status_t A7672E_TlsUploadCert(const char *pem, uint16_t pem_len)
{
    char cmd[48];

    /* Delete any existing file (ignore error if not found) */
    modem_cmd("AT+CCERTDELE=\"cacert.pem\"", "OK", 3000, NULL, 0);

    /* Download PEM to modem filesystem */
    snprintf(cmd, sizeof(cmd), "AT+CCERTDOWN=\"cacert.pem\",%d", (int)pem_len);
    modem_send(cmd);
    if (modem_wait_prompt(pem, pem_len, 10000) != A7672E_OK) return A7672E_ERR;
    if (modem_cmd(NULL, "OK", 5000, NULL, 0)   != A7672E_OK) return A7672E_ERR;

    /* Bind CA cert to SSL context 0 */
    modem_cmd("AT+CSSLCFG=\"cacert\",0,\"cacert.pem\"", "OK", 3000, NULL, 0);

    /* Verify server certificate */
    modem_cmd("AT+CSSLCFG=\"authmode\",0,1", "OK", 2000, NULL, 0);

    /* Enable SNI — required by HiveMQ Cloud */
    modem_cmd("AT+CSSLCFG=\"enableSNI\",0,1", "OK", 2000, NULL, 0);

    return A7672E_OK;
}

A7672E_Status_t A7672E_MqttConnect(const A7672E_MqttConfig_t *cfg)
{
    char cmd[180];

    /* Tear down any stale session */
    modem_cmd("AT+CMQTTDISC=0,10", "OK", 3000, NULL, 0);
    modem_cmd("AT+CMQTTREL=0",     "OK", 1000, NULL, 0);
    modem_cmd("AT+CMQTTSTOP",      "OK", 3000, NULL, 0);

    /* Start MQTT service */
    if (modem_cmd("AT+CMQTTSTART", "OK", 5000, NULL, 0) != A7672E_OK)
        return A7672E_ERR;

    /* Acquire client session 0 */
    snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\",%d",
             cfg->client_id, (int)cfg->use_ssl);
    if (modem_cmd(cmd, "OK", 3000, NULL, 0) != A7672E_OK)
        return A7672E_ERR;

    /* Use MQTT 3.1.1 */
    modem_cmd("AT+CMQTTCFG=\"version\",0,4", "OK", 2000, NULL, 0);

    /* Optional last-will message */
    if (cfg->will_topic[0] != '\0') {
        snprintf(cmd, sizeof(cmd),
                 "AT+CMQTTWILL=0,\"%s\",%d,%d,\"%s\"",
                 cfg->will_topic, (int)cfg->will_qos,
                 (int)cfg->will_retain, cfg->will_payload);
        modem_cmd(cmd, "OK", 3000, NULL, 0);
    }

    /* Connect — wait up to 30 s for +CMQTTCONNECT: 0,0 */
    snprintf(cmd, sizeof(cmd),
             "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",60,1,\"%s\",\"%s\"",
             cfg->broker, (int)cfg->port, cfg->username, cfg->password);

    if (modem_cmd(cmd, "+CMQTTCONNECT: 0,0", 30000, NULL, 0) != A7672E_OK)
        return A7672E_ERR;

    return A7672E_OK;
}

A7672E_Status_t A7672E_MqttSubscribe(const char *topic, uint8_t qos)
{
    char     cmd[48];
    uint16_t tlen = (uint16_t)strlen(topic);

    snprintf(cmd, sizeof(cmd), "AT+CMQTTSUB=0,%d,%d", (int)tlen, (int)qos);
    modem_send(cmd);

    if (modem_wait_prompt(topic, tlen, 5000) != A7672E_OK)
        return A7672E_ERR;

    /* +CMQTTSUB: 0,0 confirms subscription */
    if (modem_cmd(NULL, "+CMQTTSUB: 0,0", 10000, NULL, 0) != A7672E_OK)
        return A7672E_ERR;

    return A7672E_OK;
}

A7672E_Status_t A7672E_MqttPublish(const char *topic, const char *payload,
                                    uint8_t qos, uint8_t retain)
{
    char     cmd[48];
    uint16_t tlen = (uint16_t)strlen(topic);
    uint16_t plen = (uint16_t)strlen(payload);

    /* Step 1 — set topic */
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", (int)tlen);
    modem_send(cmd);
    if (modem_wait_prompt(topic, tlen, 3000) != A7672E_OK) return A7672E_ERR;
    if (modem_cmd(NULL, "OK", 3000, NULL, 0)  != A7672E_OK) return A7672E_ERR;

    /* Step 2 — set payload */
    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", (int)plen);
    modem_send(cmd);
    if (modem_wait_prompt(payload, plen, 3000) != A7672E_OK) return A7672E_ERR;
    if (modem_cmd(NULL, "OK", 3000, NULL, 0)    != A7672E_OK) return A7672E_ERR;

    /* Step 3 — publish; wait for +CMQTTPUB: 0,0 */
    snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=0,%d,60,%d", (int)qos, (int)retain);
    if (modem_cmd(cmd, "+CMQTTPUB: 0,0", 10000, NULL, 0) != A7672E_OK)
        return A7672E_ERR;

    return A7672E_OK;
}

void A7672E_MqttSetMsgCallback(A7672E_MqttMsgCb_t cb)  { s_msg_cb  = cb; }
void A7672E_MqttSetLostCallback(A7672E_MqttLostCb_t cb) { s_lost_cb = cb; }

void A7672E_Process(void)
{
    while (rb_count() > 0) {
        char c = (char)rb_get();

        /* ── Payload data collection (byte-count, binary-safe) ── */
        if (s_urc_state == URC_PAYLOAD_DATA) {
            if (s_data_idx < A7672E_MQTT_PAYLOAD_LEN - 1u)
                s_payload_buf[s_data_idx] = c;
            s_data_idx++;

            if (s_data_idx >= s_payload_len) {
                s_payload_buf[s_data_idx < A7672E_MQTT_PAYLOAD_LEN
                              ? s_data_idx : A7672E_MQTT_PAYLOAD_LEN - 1u] = '\0';
                s_urc_state = URC_IDLE;
                if (s_msg_cb)
                    s_msg_cb(s_topic_buf, s_payload_buf,
                             s_data_idx < A7672E_MQTT_PAYLOAD_LEN
                             ? s_data_idx : A7672E_MQTT_PAYLOAD_LEN - 1u);
                s_data_idx = 0;
            }
            continue;
        }

        /* ── Normal line accumulation ── */
        if (s_line_len < A7672E_LINE_BUF_SIZE - 1u)
            s_line[s_line_len++] = c;

        if (c == '\n') {
            s_line[s_line_len] = '\0';
            urc_handle_line();
            s_line_len = 0;
        }
    }
}
