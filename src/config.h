#ifndef __SETTINGS_H
#define __SETTINGS_H

//#define DEBUG_WSERIAL

#define RELEASE

// #define ALT_ID

#ifdef ALT_ID
    #define MQTT_CLIENT_NAME "Soil1MQTTClientALT"
    #define LWT_TOPIC "Soil1ALT/LWT"
#else
    #define MQTT_CLIENT_NAME "Soil1MQTTClient"
    #define LWT_TOPIC "soil1/LWT"
#endif


// #define LIGHT_SENSOR_LOWER_THRESHOLD 1500
// #define LIGHT_SENSOR_UPPER_THRESHOLD 1900


// #define HEART_BEAT_TIME 1000
//#define LIGHT_SENSOR_UPPER_THRESHOLD 1900

// #define ZONE_WAIT_BEFORE_FLAG_AWAY 100   //in seconds time window to wait before classed as zone gone away
// #define ZONE_HEARTBEAT_TIMEOUT_MS (1000UL * 420UL)   //max millisces to wait if no ack from pi before power cycling pi
// #define ZONE_COLD_BOOT_TIME_MS  (1000UL * 180UL)     //estimated time for a zone controller to boot from power cycle reset

#define ESP32_WATCHDOG_TIMEOUT_SECS 60
#define ESP32_WATCHDOG_RESET_INTERVAL_SECS 30

#define MQTT_LAST_OCTET 100

#define HOST_NAME "soil1.local"
#define SOIL_SAMPLE_INTERVAL 5000 //MS DELAY BETWEEN SAMPLES

#define SENSOR_PIN GPIO_NUM_36

// #define DRY_SENSOR_MAX_RAW 3980
// #define WET_SENSOR_MIN_RAW 1530
// #define RAW_RANGE   DRY_SENSOR_MAX_RAW - WET_SENSOR_MIN_RAW

//for a 0-100 output range
// get range raw / 100 ggives steps per %
//raw range = 3960 - 1530 
//steps per % = raw range/100

//for 0 to top of range -> for 0 to raw_range - > inverted_reading = reading - wet_min_raw , where 0 is wet, and dry_max_raw is dry

//to flip ,flipped =  abs( (dry_max_raw - wet_min_raw) - inverted_reading ), gives o-dry to raw_range-wet
// to scale to 0-100, scaled = (raw_range/100) * flipped
#endif
