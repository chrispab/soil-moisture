#ifndef __SETTINGS_H
#define __SETTINGS_H

#define VERSION "V1.44"

// #define DEBUG_WSERIAL



#define MAX_READINGS 256
#define MQTT_TELE_PERIOD_MS (60 * 1000)  // MS DELAY BETWEEN
#define RUNNING_SAMPLE_INTERVAL_MS (20 * 1000)


#define RELEASE

// #define ALT_ID

#ifdef ALT_ID
#define MQTT_CLIENT_NAME "Soil1MQTTClientALT"
#define LWT_TOPIC "Soil1ALT/LWT"
#else
#define MQTT_TOPIC_PREFIX "soil1/"
#define MQTT_CLIENT_NAME "Soil1MQTTClient"
#define LWT_TOPIC "LWT"
#endif

#define SENSOR_METHOD0_SINGLE_RAW_TOPIC "soil1/sensor_method0_single_raw"
#define SENSOR_METHOD1_BATCH_AVERAGE_TOPIC "soil1/sensor_method1_batch_average"
#define SENSOR_METHOD2_BATCH_MODE_TOPIC "soil1/sensor_method2_batch_mode"
#define SENSOR_METHOD3_BATCH_OUTLIER_TOPIC "soil1/sensor_method3_batch_outlier"
#define SENSOR_METHOD3_BATCH_AVERAGE_TOPIC "soil1/sensor_method3_batch_average"
#define SENSOR_METHOD4_BATCH_MOVING_AVERAGE_TOPIC "soil1/sensor_method4_batch_moving_average"
#define SENSOR_METHOD5_BATCH_MOVING_AVERAGE_FLOAT_TOPIC "soil1/sensor_method5_batch_moving_average_float"


#define ESP32_WATCHDOG_TIMEOUT_SECS 60
#define ESP32_WATCHDOG_RESET_INTERVAL_SECS 30

#define MQTT_LAST_OCTET 100

#define HOST_NAME "soil1.local"

#define SENSOR_PIN GPIO_NUM_36

// float DRY_SENSOR_MAX_RAW = 3950.0f;
// float WET_SENSOR_MIN_RAW = 1500.0f;
// const RAW_0PC_DRY = 3300.0;
// const RAW_100PC_WET = 2500.0;
// float DRY_SENSOR_MAX_RAW = 3300.0f;
// float WET_SENSOR_MIN_RAW = 2500.0f;
// const RAW_0PC_DRY = 2770.0;
// const RAW_100PC_WET = 2066.0;
// float DRY_SENSOR_MAX_RAW = 2770.0f;
// float WET_SENSOR_MIN_RAW = 2000.0f;
// float DRY_SENSOR_MAX_RAW = 2770.0f;
// float WET_SENSOR_MIN_RAW = 2000.0f;
// #define DRY_SENSOR_MAX_RAW 2020
// #define WET_SENSOR_MIN_RAW 1700
//with plastic bag
// const RAW_0PC_DRY = 2135.0;
// const RAW_100PC_WET = 1655.0;
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
#endif  // __SETTINGS_H
