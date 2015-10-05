INCLUDE+=-IRTE
INCLUDE+=-IRTE/Device/TZ1001MBG

## Middleware
#SOURCES+=RTE/Middleware/TZ1001MBG/twic_button.c
#SOURCES+=RTE/Middleware/TZ1001MBG/twic_led.c
SOURCES+=RTE/Middleware/TZ1001MBG/twic_util_advertise.c
SOURCES+=RTE/Middleware/TZ1001MBG/twic_util_ancs.c
SOURCES+=RTE/Middleware/TZ1001MBG/twic_util_hrp.c
SOURCES+=RTE/Middleware/TZ1001MBG/twic_util_init.c
SOURCES+=RTE/Middleware/TZ1001MBG/twic_util_lemng.c
SOURCES+=RTE/Middleware/TZ1001MBG/twic_util_lesmp.c
SOURCES+=RTE/Middleware/TZ1001MBG/twic_util_service.c
SOURCES+=RTE/Middleware/TZ1001MBG/twic_util_udp.c
SOURCES+=RTE/Middleware/TZ1001MBG/tz1sm_hal.c


INCLUDE+=-IRTE/Middleware/TZ1001MBG/
