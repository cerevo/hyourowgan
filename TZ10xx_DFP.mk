SOURCES+=$(DFP_PATH)/Device/Source/system_TZ10xx.c

INCLUDE+=-I$(DFP_PATH)/Device/Include

## Drivers
#SOURCES+=$(DFP_PATH)/RTE_Driver/ACCEL_GYRO_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/ACCEL_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/ADCC12_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/ADCC24_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/ADVTMR_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/GPIO_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/GYRO_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/I2C_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/MAG_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/NOR_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/PMU_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/RNG_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/RTC_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/SDMAC_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/SPI_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/SRAMC_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/TMR_TZ10xx.c
SOURCES+=$(DFP_PATH)/RTE_Driver/UART_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/USBD_TZ10xx.c
#SOURCES+=$(DFP_PATH)/RTE_Driver/WDT_TZ10xx.c

INCLUDE+=-I$(DFP_PATH)/RTE_Driver

#Middleware
SOURCES+=$(DFP_PATH)/Middleware/TWiC/twic_hash.c
SOURCES+=$(DFP_PATH)/Middleware/TWiC/twic_interface.c
SOURCES+=$(DFP_PATH)/Middleware/TWiC/twic_service.c
SOURCES+=$(DFP_PATH)/Middleware/TWiC/tz1em.c
SOURCES+=$(DFP_PATH)/Middleware/TWiC/tz1em_service.c
SOURCES+=$(DFP_PATH)/Middleware/blelib/blelib.c
SOURCES+=$(DFP_PATH)/Middleware/blelib/blelib_callback.c
SOURCES+=$(DFP_PATH)/Middleware/blelib/blelib_peripheral_statemachine.c


INCLUDE+=-I$(DFP_PATH)/Middleware/TWiC
INCLUDE+=-I$(DFP_PATH)/Middleware/blelib
INCLUDE+=-I$(DFP_PATH)/Middleware/blelib/Config
