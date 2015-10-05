#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "Driver_NOR.h"
#include "TZ10xx.h"
#include "NOR_TZ10xx.h"
#include "SDMAC_TZ10xx.h"
#include "PMU_TZ10xx.h"

#include "config.h"

#include "TZ01_console.h"

extern TZ10XX_DRIVER_PMU    Driver_PMU;
extern TZ10XX_DRIVER_NOR    Driver_NOR0;
extern TZ10XX_DRIVER_SDMAC  Driver_SDMAC;

static HYRWGN_CONFIG stored_config;

static uint8_t msg[80];

static bool check_valid(uint8_t *cnf, uint8_t sz)
{
    bool ret = false;
    uint8_t cd = 0;
    //マジックコード確認
    if (memcmp(cnf, "hrGC", 4) == 0) {
        //チェックディジット確認
        for (int i = 0; i < sz; i++) {
            cd ^= cnf[i];
        }
        ret = (cd == 0);
    }
    
    return ret;
}

static bool write_to_addr(HYRWGN_CONFIG *config, uint32_t addr)
{
    int ret;
    uint8_t val[26];
    uint8_t cd = 0;
    
    ret = Driver_NOR0.EraseSector(addr);
    sprintf(msg, "Driver_NOR0.EraseSector(): %d\r\n", ret);
    TZ01_console_puts(msg);
    
    memcpy(val, CONFIG_MAGIC, 4);
    memcpy(&val[4], config->shortened_local_name, 20);
    for (int i = 0; i < 24; i++) {
        cd ^= val[i];
    }
    val[24] = cd;
    val[25] = '\0';
    TZ01_console_puts(val);
    
    ret = Driver_NOR0.WriteData(addr, val, 25);
    sprintf(msg, "Driver_NOR0.WriteData(): %d\r\n", ret);
    TZ01_console_puts(msg);
    
    uint8_t *pcnf = (uint8_t *)CONFIG_BASE_ADDR_1;
    uint8_t *scnf = (uint8_t *)CONFIG_BASE_ADDR_2;
    
    for (int i = 0; i < 25; i++) {
        TZ01_console_putc(pcnf[i]);
    }
    TZ01_console_puts("\r\n");
    
    for (int i = 0; i < 25; i++) {
        TZ01_console_putc(scnf[i]);
    }
    TZ01_console_puts("\r\n");
}

bool config_init(void)
{
    int ret;
    Driver_PMU.SelectClockSource(PMU_CSM_CPUST, PMU_CLOCK_SOURCE_OSC12M);
    
    ret = Driver_SDMAC.Initialize();
    sprintf(msg, "Driver_SDMAC.Initialize(): %d\r\n", ret);
    TZ01_console_puts(msg);
    Driver_SDMAC.PowerControl(ARM_POWER_FULL);
    
    ret = Driver_NOR0.Initialize(0);
    sprintf(msg, "Driver_NOR0.Initialize(): %d\r\n", ret);
    TZ01_console_puts(msg);
    Driver_NOR0.PowerControl(ARM_POWER_FULL);
    
    Driver_NOR0.WriteProtect(NOR_0KB, NOR_TOP);
    
    return true;
}

bool config_set(HYRWGN_CONFIG *config)
{
    stored_config = *config;
    return true;
}

bool config_get(HYRWGN_CONFIG *config)
{
    *config = stored_config;
    return true;
}

bool config_load(void)
{
    uint8_t *pcnf = (uint8_t *)CONFIG_BASE_ADDR_1;
    uint8_t *scnf = (uint8_t *)CONFIG_BASE_ADDR_2;
    
    for (int i = 0; i < 25; i++) {
        TZ01_console_putc(pcnf[i]);
    }
    TZ01_console_puts("\r\n");
    
    for (int i = 0; i < 25; i++) {
        TZ01_console_putc(scnf[i]);
    }
    TZ01_console_puts("\r\n");
    
    bool valid_pcnf = check_valid(pcnf, 25);
    bool valid_scnf = check_valid(scnf, 25);
    
    if (valid_pcnf) {
        /* Primary OK */
        //Primary領域から読み込み
        memset(stored_config.shortened_local_name, 0, 20);
        for (int i = 0; i < 20; i++) {
            stored_config.shortened_local_name[i] = pcnf[i + 4];
            if (pcnf[i + 4] == '\0') {
                break;
            }
        }
        
        if (valid_scnf) {
            /* Secondary OK */
            //なにもしないよ
        } else {
            /* Secondary NG */
            //Secondaryを復旧させるよ
            write_to_addr(&stored_config, CONFIG_BASE_ADDR_2);
        }
    } else {
        if (valid_scnf) {
            /* Secondary OK */
            //Secondary領域から読み込み
            memset(stored_config.shortened_local_name, 0, 20);
            for (int i = 0; i < 20; i++) {
                stored_config.shortened_local_name[i] = scnf[i + 4];
                if (pcnf[i + 4] == '\0') {
                    break;
                }
            }
            
            //Primary領域を復旧させるよ
            write_to_addr(&stored_config, CONFIG_BASE_ADDR_1);
        } else {
            /* Secondary NG */
            memset(stored_config.shortened_local_name, 0, 20);
            strncpy(stored_config.shortened_local_name, DEFAULT_SHORTENED_LOCAL_NAME, strlen(DEFAULT_SHORTENED_LOCAL_NAME));
            //Primary領域もSecondary領域もデフォルト値を書くよ
            write_to_addr(&stored_config, CONFIG_BASE_ADDR_1);
            write_to_addr(&stored_config, CONFIG_BASE_ADDR_2);
        }
    }
    //Driver_NOR0.GetID(&buf[0], &buf[1], &buf[2]);
    
    return true;
}

bool config_save(void)
{
    write_to_addr(&stored_config, CONFIG_BASE_ADDR_1);
    write_to_addr(&stored_config, CONFIG_BASE_ADDR_2);
}
