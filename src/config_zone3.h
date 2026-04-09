// #ifndef __SETTINGS_H
// #define __SETTINGS_H

// adjust below as required
// #define MOISTURE_SENSOR_LOCATION "Zone 3"
#define MOISTURE_SENSOR_ZONE_LOCATION "3"

#define MOISTURE_SENSOR_ID "1"
#define SENSOR_NAME "SENSOR_1"
// end-adjust

#define MQTT_TOPIC_PREFIX "soil_moisture_sensor_"
#define MQTT_PRIMARY_TOPIC MQTT_TOPIC_PREFIX MOISTURE_SENSOR_ID

#define MQTT_VERSION_TOPIC MQTT_PRIMARY_TOPIC  "/version"

#define SENSOR_METHOD0_SINGLE_RAW_TOPIC MQTT_PRIMARY_TOPIC  "/rawReading"
#define SENSOR_METHOD5_BATCH_MOVING_AVERAGE_FLOAT_TOPIC MQTT_PRIMARY_TOPIC  "/sensor_method5_batch_moving_average_float"
#define SENSOR_MOVING_AVERAGE_TOPIC MQTT_PRIMARY_TOPIC  "/movingAverageReading"

// #define ALT_ID
// #ifdef ALT_ID
// #define MQTT_CLIENT_NAME "Soil1MQTTClientALT"
// #define LWT_TOPIC "Soil1ALT/LWT"
// #else
// #define MQTT_TOPIC_PREFIX "soil1/"
#define MQTT_CLIENT_NAME "Soil" MOISTURE_SENSOR_ZONE_LOCATION "MQTTClient"
#define LWT_TOPIC MQTT_PRIMARY_TOPIC  "/LWT"
// #endif

// #define DEBUG_WSERIAL
#define MQTT_SENSOR_READ_PERIOD_SECONDS 300
#define MQTT_TELE_PERIOD_MS (MQTT_SENSOR_READ_PERIOD_SECONDS * 1000)  // MS DELAY BETWEEN
#define MQTT_TELE_PERIOD_MS_TOPIC MQTT_PRIMARY_TOPIC  "/tele_period_ms"

#define MOVING_AVERAGE_WINDOW_SIZE 5
#define SENSOR_MOVING_AVERAGE_WINDOW_SIZE_TOPIC MQTT_PRIMARY_TOPIC "/movingAverageReadingWindowSize"
#define MAX_READINGS 256
#define RUNNING_SAMPLE_INTERVAL_MS (20 * 1000)

#define RELEASE

#define ESP32_WATCHDOG_TIMEOUT_SECS 60
#define ESP32_WATCHDOG_RESET_INTERVAL_SECS 30

#define MQTT_LAST_OCTET 100

#define HOST_NAME "soil"1 MOISTURE_SENSOR_ZONE_LOCATION ".local"

#define SENSOR_PIN GPIO_NUM_36
#define SENSOR_POWERSUPPLY_PIN GPIO_NUM_23

#define DRY_SENSOR_MAX_RAW 2135
#define WET_SENSOR_MIN_RAW 1655
// #define RAW_RANGE   DRY_SENSOR_MAX_RAW - WET_SENSOR_MIN_RAW

// for a 0-100 output range
//  get range raw / 100 ggives steps per %
// raw range = 3960 - 1530
// steps per % = raw range/100

// for 0 to top of range -> for 0 to raw_range - > inverted_reading = reading - wet_min_raw , where 0 is wet, and dry_max_raw is dry

// to flip ,flipped =  abs( (dry_max_raw - wet_min_raw) - inverted_reading ), gives o-dry to raw_range-wet
//  to scale to 0-100, scaled = (raw_range/100) * flipped
// #endif  // __SETTINGS_H
