SOURCES+=src/boards/CDP-TZ01B/TZ01_airpressure.c
#SOURCES+=src/boards/CDP-TZ01B/TZ01_battery_charger.c
SOURCES+=src/boards/CDP-TZ01B/TZ01_console.c
SOURCES+=src/boards/CDP-TZ01B/TZ01_motion_tracker.c
SOURCES+=src/boards/CDP-TZ01B/TZ01_system.c
SOURCES+=src/drivers/BMP280.c
#SOURCES+=src/drivers/BQ24250.c
SOURCES+=src/drivers/MPU-9250.c
SOURCES+=src/utils/utils.c

INCLUDE+=-Isrc/boards/CDP-TZ01B/include
INCLUDE+=-Isrc/config
INCLUDE+=-Isrc/drivers/include
INCLUDE+=-Isrc/utils/include
