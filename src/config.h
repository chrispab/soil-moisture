#ifndef __CONFIG_H
#define __CONFIG_H

#include "version.h"

//sensor instance specific stuff
#include "config_zone1.h"
// #include "config_zone3.h"


// common config items
#define MQTT_TOPIC_PREFIX "soil_moisture_sensor_"
// #define MQTT_PRIMARY_TOPIC MQTT_TOPIC_PREFIX MOISTURE_SENSOR_ID
#define MQTT_PRIMARY_TOPIC "Zone" ZONE_ID "/SoilMoistureSensor/" MOISTURE_SENSOR_ID

#define MQTT_VERSION_TOPIC MQTT_PRIMARY_TOPIC  "/Version"
#define MQTT_ZONE_LOCATION_TOPIC MQTT_PRIMARY_TOPIC  "/Zone"

// #define SENSOR_METHOD0_SINGLE_RAW_TOPIC MQTT_PRIMARY_TOPIC  "/rawReading"
//eg zone1/soil_moisture_sensor/3/rawReading
#define MQTT_PRIMARY_TOPIC_2 "Zone" ZONE_ID "/SoilMoistureSensor/" MOISTURE_SENSOR_ID
#define MQTT_RAW_READING_TOPIC MQTT_PRIMARY_TOPIC_2  "/ReadingRaw"

#define SENSOR_MOVING_AVERAGE_WINDOW_SIZE_TOPIC MQTT_PRIMARY_TOPIC "/ReadingMovingAverageWindowSize"

// #define SENSOR_METHOD5_BATCH_MOVING_AVERAGE_FLOAT_TOPIC MQTT_PRIMARY_TOPIC  "/sensor_method5_batch_moving_average_float"
#define SENSOR_MOVING_AVERAGE_TOPIC MQTT_PRIMARY_TOPIC  "/ReadingMovingAverage"

#define MQTT_CLIENT_NAME "Soil" MOISTURE_SENSOR_ZONE_LOCATION "MQTTClient"
#define LWT_TOPIC MQTT_PRIMARY_TOPIC  "/LWT"

#define MQTT_TELE_PERIOD_MS (MQTT_SENSOR_READ_PERIOD_SECONDS * 1000)  // MS DELAY BETWEEN
#define MQTT_TELE_PERIOD_MS_TOPIC MQTT_PRIMARY_TOPIC  "/TelePeriodMs"

#define ESP32_WATCHDOG_TIMEOUT_SECS 60
#define ESP32_WATCHDOG_RESET_INTERVAL_SECS 30

#define MQTT_LAST_OCTET 100

#define HOST_NAME "soil" MOISTURE_SENSOR_ZONE_LOCATION ".local"
// #define HOST_NAME "soil3.local"

#define SENSOR_PIN GPIO_NUM_36
#define SENSOR_POWERSUPPLY_PIN GPIO_NUM_23

#define DRY_SENSOR_MAX_RAW 2135
#define WET_SENSOR_MIN_RAW 1655


#endif  // __CONFIG_H
