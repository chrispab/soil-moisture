#ifndef __SETTINGS_H
#define __SETTINGS_H

// #define DEBUG_WSERIAL

#define RELEASE

// #define ALT_ID

#ifdef ALT_ID
#define MQTT_CLIENT_NAME "Soil1MQTTClientALT"
#define LWT_TOPIC "Soil1ALT/LWT"
#else
#define MQTT_CLIENT_NAME "Soil1MQTTClient"
#define LWT_TOPIC "soil1/LWT"
#endif

#define ESP32_WATCHDOG_TIMEOUT_SECS 60
#define ESP32_WATCHDOG_RESET_INTERVAL_SECS 30

#define MQTT_LAST_OCTET 100

#define HOST_NAME "soil1.local"

#define SENSOR_PIN GPIO_NUM_36

// float DRY_SENSOR_MAX_RAW = 2770.0f;
// float WET_SENSOR_MIN_RAW = 2000.0f;
#define DRY_SENSOR_MAX_RAW 2770
#define WET_SENSOR_MIN_RAW 2000
// #define RAW_RANGE   DRY_SENSOR_MAX_RAW - WET_SENSOR_MIN_RAW

// for a 0-100 output range
//  get range raw / 100 ggives steps per %
// raw range = 3960 - 1530
// steps per % = raw range/100

// for 0 to top of range -> for 0 to raw_range - > inverted_reading = reading - wet_min_raw , where 0 is wet, and dry_max_raw is dry

// to flip ,flipped =  abs( (dry_max_raw - wet_min_raw) - inverted_reading ), gives o-dry to raw_range-wet
//  to scale to 0-100, scaled = (raw_range/100) * flipped
#endif // __SETTINGS_H
