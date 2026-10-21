/**
 * @file    devcfg.c
 * @brief   This device's MQTT login in flash sector 7 (see devcfg.h).
 *
 * Record layout: magic, version, DevCfg_t, CRC-32 of everything before it.
 * A record with a wrong magic, version or CRC (erased flash, half-written
 * record) counts as "no login" and the device claims a new one.
 */

#include "devcfg.h"
#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <string.h>

#define DEVCFG_ADDR      0x08060000u       /* sector 7: reserved in the .ld */
#define DEVCFG_SECTOR    FLASH_SECTOR_7
#define DEVCFG_MAGIC     0x4E57444Cu       /* "NWDL" */
#define DEVCFG_VERSION   1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    DevCfg_t cfg;
    uint32_t crc;
} DevCfgRecord_t;

static uint32_t crc32(const uint8_t *p, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

int DevCfg_Load(DevCfg_t *cfg)
{
    const DevCfgRecord_t *rec = (const DevCfgRecord_t *)DEVCFG_ADDR;

    if (rec->magic != DEVCFG_MAGIC || rec->version != DEVCFG_VERSION)
        return 0;
    if (rec->crc != crc32((const uint8_t *)rec, offsetof(DevCfgRecord_t, crc)))
        return 0;
    /* Strings must be terminated inside their fields */
    if (!memchr(rec->cfg.device_id, '\0', sizeof(rec->cfg.device_id)) ||
        !memchr(rec->cfg.password, '\0', sizeof(rec->cfg.password)) ||
        !memchr(rec->cfg.host, '\0', sizeof(rec->cfg.host)) ||
        rec->cfg.device_id[0] == '\0')
        return 0;

    memcpy(cfg, &rec->cfg, sizeof(*cfg));
    return 1;
}

static int erase_sector(void)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = DEVCFG_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,   /* 2.7-3.6 V */
    };
    uint32_t bad_sector = 0;
    return HAL_FLASHEx_Erase(&erase, &bad_sector) == HAL_OK;
}

int DevCfg_Save(const DevCfg_t *cfg)
{
    DevCfgRecord_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic   = DEVCFG_MAGIC;
    rec.version = DEVCFG_VERSION;
    memcpy(&rec.cfg, cfg, sizeof(rec.cfg));
    rec.crc = crc32((const uint8_t *)&rec, offsetof(DevCfgRecord_t, crc));

    int ok = 0;
    HAL_FLASH_Unlock();
    if (erase_sector()) {
        const uint32_t *src = (const uint32_t *)&rec;
        ok = 1;
        for (uint32_t i = 0; i < sizeof(rec) / 4u && ok; i++)
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   DEVCFG_ADDR + i * 4u, src[i]) == HAL_OK;
    }
    HAL_FLASH_Lock();

    /* Read back: only a record that loads counts as saved */
    DevCfg_t check;
    return ok && DevCfg_Load(&check) &&
           memcmp(&check, cfg, sizeof(check)) == 0;
}

void DevCfg_Erase(void)
{
    HAL_FLASH_Unlock();
    erase_sector();
    HAL_FLASH_Lock();
}
