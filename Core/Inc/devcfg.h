/**
 * @file    devcfg.h
 * @brief   This device's MQTT login, kept in flash.
 *
 * The login is not built into the firmware: one image runs on every device.
 * On first boot there is none; the device claims one from the broker
 * (zero-touch provisioning, PROVISIONING.md in the mqtt repo) and saves it
 * here. It lives in flash sector 7 (0x08060000, 128 KB), which the linker
 * script keeps free of code.
 */

#ifndef DEVCFG_H
#define DEVCFG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    char device_id[32];   /**< MQTT username = client id = topic prefix */
    char password[48];    /**< MQTT password                            */
    char host[64];        /**< broker host name                          */
    uint16_t port;        /**< broker TLS port                           */
} DevCfg_t;

/** Load the saved login. Returns 1 if a valid one is stored, else 0. */
int DevCfg_Load(DevCfg_t *cfg);

/** Save a login (erases sector 7 first). Returns 1 on success. */
int DevCfg_Save(const DevCfg_t *cfg);

/** Forget the login; the next boot claims a new one. */
void DevCfg_Erase(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVCFG_H */
