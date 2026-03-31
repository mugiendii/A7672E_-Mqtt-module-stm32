/**
 * @file  payload.c
 * @brief JSON payload builder — snprintf-based, no heap allocation.
 */

#include "payload.h"
#include <stdio.h>
#include <string.h>

/* Append a JSON string field only if the value is non-empty.
 * Returns the number of characters written, or 0 if the field was skipped. */
static int append_str_field(char *buf, int pos, int size,
                             const char *key, const char *val,
                             int *first)
{
    if (val[0] == '\0') return 0;
    int n = snprintf(buf + pos, (size_t)(size - pos),
                     "%s\"%s\":\"%s\"",
                     *first ? "" : ",", key, val);
    if (n > 0) *first = 0;
    return (n > 0) ? n : 0;
}

uint16_t Payload_BuildTransaction(char *buf, uint16_t size,
                                  const Payload_Transaction_t *tx)
{
    if (!buf || size < 2u) return 0;

    int pos   = 0;
    int first = 1;   /* tracks whether we need a leading comma */

    pos += snprintf(buf + pos, (size_t)(size - pos), "{");

    pos += append_str_field(buf, pos, size, "code",         tx->code,         &first);
    pos += append_str_field(buf, pos, size, "type",         tx->type,         &first);
    pos += append_str_field(buf, pos, size, "amount",       tx->amount,       &first);
    pos += append_str_field(buf, pos, size, "sender",       tx->sender,       &first);
    pos += append_str_field(buf, pos, size, "sender_phone", tx->sender_phone, &first);
    pos += append_str_field(buf, pos, size, "recipient",    tx->recipient,    &first);
    pos += append_str_field(buf, pos, size, "balance",      tx->balance,      &first);
    pos += append_str_field(buf, pos, size, "ts",           tx->ts,           &first);

    /* uptime_ms is always included */
    pos += snprintf(buf + pos, (size_t)(size - pos),
                    "%s\"uptime_ms\":%lu",
                    first ? "" : ",",
                    (unsigned long)tx->uptime_ms);

    pos += snprintf(buf + pos, (size_t)(size - pos), "}");

    return (pos > 0 && pos < size) ? (uint16_t)pos : 0u;
}

uint16_t Payload_BuildHeartbeat(char *buf, uint16_t size)
{
    int n = snprintf(buf, size, "{\"alive\":1}");
    return (n > 0 && n < size) ? (uint16_t)n : 0u;
}

uint16_t Payload_BuildStatus(char *buf, uint16_t size, uint8_t online)
{
    int n = snprintf(buf, size, "{\"status\":\"%s\"}",
                     online ? "online" : "offline");
    return (n > 0 && n < size) ? (uint16_t)n : 0u;
}
