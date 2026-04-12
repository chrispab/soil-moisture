#ifndef __CONFIG_H
#define __CONFIG_H

#include "version.h"

//sensor instance specific stuff
// #include "config_zone1.h"
// #include "config_zone3.h"
#if defined(USE_ZONE1_CONFIG)
    #include "config_zone1.h"
#elif defined(USE_ZONE3_CONFIG)
    #include "config_zone3.h"
#endif

// common config items
#define MQTT_TOPIC_PREFIX "soil_moisture_sensor_"
// #define MQTT_PRIMARY_TOPIC MQTT_TOPIC_PREFIX MOISTURE_SENSOR_ID
#define MQTT_PRIMARY_TOPIC "Zone" ZONE_ID "/SoilMoistureSensor/" MOISTURE_SENSOR_ID

#define MQTT_VERSION_TOPIC MQTT_PRIMARY_TOPIC  "/Version"
#define MQTT_ZONE_LOCATION_TOPIC MQTT_PRIMARY_TOPIC  "/Zone"

#define SENSOR_ID_TOPIC MQTT_PRIMARY_TOPIC  "/SensorId"
#define SENSOR_WARMUP_TIME_TOPIC MQTT_PRIMARY_TOPIC  "/SensorWarmupTimeMs"
// #define SENSOR_METHOD0_SINGLE_RAW_TOPIC MQTT_PRIMARY_TOPIC  "/rawReading"
//eg zone1/soil_moisture_sensor/3/rawReading
#define MQTT_PRIMARY_TOPIC_2 "Zone" ZONE_ID "/SoilMoistureSensor/" MOISTURE_SENSOR_ID
#define MQTT_RAW_READING_TOPIC MQTT_PRIMARY_TOPIC_2  "/ReadingRaw"
#define MQTT_COMMAND_SUBSCRIBE_TOPIC MQTT_PRIMARY_TOPIC  "/Read"
#define SENSOR_MOVING_AVERAGE_WINDOW_SIZE_TOPIC MQTT_PRIMARY_TOPIC "/ReadingMovingAverageWindowSize"

// #define SENSOR_METHOD5_BATCH_MOVING_AVERAGE_FLOAT_TOPIC MQTT_PRIMARY_TOPIC  "/sensor_method5_batch_moving_average_float"
#define SENSOR_MOVING_AVERAGE_TOPIC MQTT_PRIMARY_TOPIC  "/ReadingMovingAverage"
#define WET_SENSOR_MIN_RAW_TOPIC MQTT_PRIMARY_TOPIC  "/WetSensorMinRaw"
#define DRY_SENSOR_MAX_RAW_TOPIC MQTT_PRIMARY_TOPIC  "/DrySensorMaxRaw"
#define SENSOR_MODE_TOPIC MQTT_PRIMARY_TOPIC  "/Mode"

#define MQTT_CLIENT_NAME "Soil" MOISTURE_SENSOR_ZONE_LOCATION "MQTTClient"
#define LWT_TOPIC MQTT_PRIMARY_TOPIC  "/LWT"

#define MQTT_TELE_PERIOD_MS (MQTT_SENSOR_READ_PERIOD_SECONDS * 1000)  // MS DELAY BETWEEN
#define MQTT_TELE_PERIOD_MS_TOPIC MQTT_PRIMARY_TOPIC  "/TelePeriodMs"
#define MQTT_MOISTURE_PERCENTAGE_TOPIC MQTT_PRIMARY_TOPIC  "/MoisturePercentage"

#define ESP32_WATCHDOG_TIMEOUT_SECS 60
#define ESP32_WATCHDOG_RESET_INTERVAL_SECS 30

#define MQTT_LAST_OCTET 100

#define HOST_NAME "soil" MOISTURE_SENSOR_ZONE_LOCATION ".local"
// #define HOST_NAME "soil3.local"

#define SENSOR_PIN GPIO_NUM_36
#define SENSOR_POWERSUPPLY_PIN GPIO_NUM_23

// #define DRY_SENSOR_MAX_RAW 900.0
// #define DRY_SENSOR_MAX_RAW 2450.0
// #define DRY_SENSOR_MAX_RAW 2000.0
#define DRY_SENSOR_MAX_RAW 1950

// 
#define WET_SENSOR_MIN_RAW 0.0


#endif  // __CONFIG_H
