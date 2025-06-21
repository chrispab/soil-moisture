#include <Arduino.h>
#include <PubSubClient.h>
#include <assert.h>

// req for ota
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
//
#include "LedFader.h"
#include "MQTTLib.h"
#include "WiFiLib.h"
#include "config.h"
#include "version.h"

#define GREEN_LED_PIN GPIO_NUM_12  //
// #define IR_LED_PIN GPIO_NUM_13      //
#define ONBOARD_LED_PIN GPIO_NUM_2  //
// #define RX_PIN GPIO_NUM_14          //

#define HEART_BEAT_TIME 800
#define BLUE_BEAT_TIME 300

// Outlier limit for sensor readings
#define OUTLIER_LIMIT 4
#define MAX_READINGS 256

LedFader heartBeatLED(GREEN_LED_PIN, 1, 0, 255, HEART_BEAT_TIME);
LedFader blueBeatLED(ONBOARD_LED_PIN, 2, 0, 50, BLUE_BEAT_TIME);

IPAddress mqttBroker(192, 168, 0, MQTT_LAST_OCTET);
WiFiClient myWiFiClient;

// forward func declarations
void callback(char *topic, byte *payload, unsigned int length);
PubSubClient MQTTclient(mqttBroker, 1883, callback, myWiFiClient);
unsigned int readAndTxMoistureSensor();
void readAndPublishSingleRaw(const char *topic);
unsigned int readMethodsPublish(unsigned int &numReadings, unsigned int msBetweenReadings);
void getReadings(unsigned int &numReadings, unsigned int msBetweenReadings, unsigned int readings[256]);
void method_averageRaw(const char *topic, unsigned int readings[], unsigned int numReadings);
unsigned int getModeValue(unsigned int a[], unsigned int n);
unsigned int getAverageOfReadings(const unsigned int readings[], unsigned int numReadings, bool round);
float getAverageOfReadingsFloat(const unsigned int readings[], unsigned int numReadings);
float getAverageOfReadingsFloat(const float readings[], unsigned int numReadings);

void setup() {
    Serial.begin(115200);
    while (!Serial)  // Wait for the serial connection to be establised.
        delay(50);
    // delay(3000);

    Serial.println("Hi from soil1 ... Booting");
    connectWiFi();
    printWifiStatus();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.flush();

    delay(5000);
    // connectMQTT();
    reconnectMQTT();

    heartBeatLED.begin();  // initialize
    blueBeatLED.begin();   // initialize
    // you're connected now, so print out the status:

    /* we use mDNS instead of IP of ESP32 directly */
    // hostname.local
    ArduinoOTA.setHostname(HOST_NAME);

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else  // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount
            // SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)
                Serial.println("End Failed");
        });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    MQTTclient.subscribe("soil1/read");
}

void loop() {
    connectWiFi();
    if (!MQTTclient.connected()) {
        reconnectMQTT();    // Attempt to reconnect
    } else {                // Client is connected
        MQTTclient.loop();  // process any MQTT stuff, returned in callback
    }
    ArduinoOTA.handle();
    heartBeatLED.update();
    blueBeatLED.update();

    // readAndPublishSingleRaw("soil/calibration");
    // readAndPublishSingleRaw("soil1/moisture2_average_raw");
    // delay(1000);
    readAndTxMoistureSensor();
}

#define MQTT_TELE_PERIOD_MS (60 * 1000)  // MS DELAY BETWEEN
unsigned int lastMQTTTransmitMs = millis() - MQTT_TELE_PERIOD_MS - 1000;
float prevFilteredValue = 0.0;
#define RUNNING_SAMPLE_INTERVAL_MS (20 * 1000)
unsigned int lastMoistureSampleMs = millis() - RUNNING_SAMPLE_INTERVAL_MS - 1000;
unsigned int runningSensorReading = analogRead(SENSOR_PIN);  // running total reading - taken every RUNNING_SAMPLE_INTERVAL_MS
float scaleAndTransmit(unsigned int moistureReading, float drySensorMaxRaw, float wetSensorMinRaw, const char *topic);

unsigned int readAndTxMoistureSensor() {
    unsigned int now = millis();
    // unsigned int sensorValue;
    unsigned int moisture_raw;

    moisture_raw = 0;
    if (now - lastMoistureSampleMs > RUNNING_SAMPLE_INTERVAL_MS) {
        runningSensorReading = (runningSensorReading + analogRead(SENSOR_PIN)) / 2;
        // runningSensorReading = (((runningSensorReading * 80) / 100) + ((analogRead(SENSOR_PIN) * 20) / 100));
        lastMoistureSampleMs = now;
    }
    if (now - lastMQTTTransmitMs > MQTT_TELE_PERIOD_MS) {
        lastMQTTTransmitMs = now;

        // publish telemetry
        MQTTclient.publish("soil1/version", VERSION);

        // do all the different methods
        unsigned int numReadings = 255;
        moisture_raw = readMethodsPublish(numReadings, 50);
        Serial.println(moisture_raw);

        scaleAndTransmit(moisture_raw, DRY_SENSOR_MAX_RAW, WET_SENSOR_MIN_RAW, "soil1/moisture_pc");
    }
    return moisture_raw;
}

/**
 * Publishes a value to the specified MQTT topic.
 *
 * @param topic the topic to publish the value to
 * @param value the value to publish
 *
 * @throws ErrorType if there is an error publishing the value
 */
void MQTTpublishValue(const char *topic, unsigned int value) {
    char valueStr[20];  // max of 20 chars string
    utoa(value, valueStr, 10);
    MQTTclient.publish(topic, valueStr);
}

void MQTTpublishValue(const char *topic, float value) {
    char valueStr[20];
    dtostrf(value, 0, 1, valueStr);  // 1 decimal place
    MQTTclient.publish(topic, valueStr);
}

/**
 * Reads the value from the sensor connected to SENSOR_PIN and publishes it to the specified MQTT topic.
 *
 * @param topic The MQTT topic to publish the sensor value to.
 *
 * @throws None
 */
void readAndPublishSingleRaw(const char *topic) {
    unsigned int sensorValue;
    // analogRead(SENSOR_PIN);
    // delay(100);
    sensorValue = analogRead(SENSOR_PIN);
    MQTTpublishValue(topic, sensorValue);
}

/**
 * Reads a single raw analog value from the sensor connected to SENSOR_PIN.
 *
 * @return The raw sensor value as a 16-bit unsigned integer.
 */
uint16_t readRaw() {
    return analogRead(SENSOR_PIN);
}

// Publish a value to the given topic
void publishValueToTopic(const char *topic, uint16_t value) {
    MQTTpublishValue(topic, static_cast<unsigned int>(value));
}

/**
 * Reads a specified number of analog readings from a sensor pin and publishes
 * the results using different methods. The first reading is discarded and the
 * readings are spaced apart by a specified time interval. The methods used are:
 * 1. Average of raw readings.
 * 2. Finds the most common value in the readings.
 * 3. Removes outliers from the readings by replacing them with the average value
 *    and then averages the remaining values.
 *
 * @param numReadings The number of readings to take. Limited to a maximum of 256.
 * @param msBetweenReadings The time interval between each reading in milliseconds.
 *
 * @return The average value of the readings after outliers are removed.
 *
 * @throws None
 */
unsigned int readMethodsPublish(unsigned int &numReadings, unsigned int msBetweenReadings) {
    // limit readings to max of MAX_READINGS samples
    unsigned int readings[MAX_READINGS];
    getReadings(numReadings, msBetweenReadings, readings);

    // method 0 - just read raw
    uint16_t rawValue = readRaw();
    Serial.print("Raw sensor value: ");
    Serial.println(rawValue);
    publishValueToTopic("soil1/moisture_raw", rawValue);
    publishValueToTopic("soil1/sensor_method0_single_raw", rawValue);
    // readAndPublishSingleRaw("soil1/sensor_method0_single_raw");

    // method 1
    method_averageRaw("soil1/moisture_method1_average", readings, numReadings);
    method_averageRaw("soil1/sensor_method1_average", readings, numReadings);

    // method 2 - find most common value
    unsigned int modeValue = getModeValue(readings, numReadings);
    MQTTpublishValue("soil1/moisture_method2_mode", modeValue);
    MQTTpublishValue("soil1/sensor_method2_mode", modeValue);

    // method 3 - remove outliers, then average
    unsigned int averageValue = getAverageOfReadings(readings, numReadings, true);  // round the average value
    // remove outliers
    for (unsigned int i = 0; i < numReadings; i++) {
        if (abs((int)readings[i] - (int)averageValue) > OUTLIER_LIMIT) {  // outlier
            MQTTpublishValue("soil1/moisture_method3_excluded", readings[i]);
            MQTTpublishValue("soil1/sensor_method3_excluded", readings[i]);
            // replace with average
            readings[i] = averageValue;
        }
    }
    averageValue = getAverageOfReadings(readings, numReadings, true);  // round the average value
    MQTTpublishValue("soil1/moisture_method3_average", averageValue);
    MQTTpublishValue("soil1/sensor_method3_average", averageValue);

// method 4 - a moving average of the last 10 readings
#define READINGS_WINDOW 10

    static unsigned int movingAverageReadings[READINGS_WINDOW];
    static unsigned int movingAverageCount = 0;
    if (movingAverageCount < READINGS_WINDOW) {
        movingAverageReadings[movingAverageCount++] = averageValue;
    } else {
        // shift the readings to the left
        for (unsigned int i = 0; i < READINGS_WINDOW - 1; i++) {
            movingAverageReadings[i] = movingAverageReadings[i + 1];
        }
        movingAverageReadings[READINGS_WINDOW - 1] = averageValue;
    }
    unsigned int movingAverageValue = getAverageOfReadings(movingAverageReadings, movingAverageCount, true);
    MQTTpublishValue("soil1/moisture_method4_moving_average", movingAverageValue);
    MQTTpublishValue("soil1/sensor_method4_moving_average", movingAverageValue);

// method 5 - a moving average of the last 20 readings, but with floating point values for higher accuracy
#define READINGS_FLOAT_WINDOW 20

    static float movingAverageReadingsFloat[READINGS_FLOAT_WINDOW];
    static unsigned int movingAverageCountFloat = 0;
    if (movingAverageCountFloat < READINGS_FLOAT_WINDOW) {
        movingAverageReadingsFloat[movingAverageCountFloat++] = static_cast<float>(averageValue);
    } else {
        // shift the readings to the left
        for (unsigned int i = 0; i < READINGS_FLOAT_WINDOW - 1; i++) {
            movingAverageReadingsFloat[i] = movingAverageReadingsFloat[i + 1];
        }
        movingAverageReadingsFloat[READINGS_FLOAT_WINDOW - 1] = static_cast<float>(averageValue);
    }
    // float movingAverageValueFloat = getAverageOfReadingsFloat(movingAverageReadingsFloat, movingAverageCountFloat);
    float movingAverageValueFloat = getAverageOfReadingsFloat(movingAverageReadingsFloat, movingAverageCountFloat);
    // Limit to 1 decimal place
    movingAverageValueFloat = roundf(movingAverageValueFloat * 10.0f) / 10.0f;
    MQTTpublishValue("soil1/moisture_method5_moving_average_float", movingAverageValueFloat);
    MQTTpublishValue("soil1/sensor_method5_moving_average_float", movingAverageValueFloat);

    return averageValue;
}

/**
 * Reads a specified number of analog sensor readings with a specified time interval between each reading.
 *
 * @param numReadings Reference to an unsigned integer that holds the number of readings to take. Limited to a maximum of 256.
 * @param msBetweenReadings The time interval between each reading in milliseconds.
 * @param readings An array of unsigned integers that will hold the readings. The array must have a size of at least `numReadings` (does not need to be `MAX_READINGS`).
 *
 * @throws None
 */
void getReadings(unsigned int &numReadings, unsigned int msBetweenReadings, unsigned int readings[]) {
    if (numReadings > MAX_READINGS) numReadings = MAX_READINGS;
    if (numReadings == 0) numReadings = 1;

    // Discard the first reading to stabilize the ADC (common practice for more reliable sensor data)
    // delay(msBetweenReadings);
    analogRead(SENSOR_PIN);

    // read in the samples
    for (unsigned int i = 0; i < numReadings; i++) {
        delay(msBetweenReadings);
        readings[i] = analogRead(SENSOR_PIN);
    }
}

/**
 * Calculates the average value of an array of unsigned integers.
 *
 * By default, integer division is used, so the result is truncated toward zero (not rounded).
 * This may lead to loss of precision for small sample sizes or when the sum is not divisible by numReadings.
 * If higher accuracy is needed, set the 'round' parameter to true to return the nearest integer (rounded).
 *
 * @param readings An array of unsigned integers representing sensor readings.
 * @param numReadings The number of readings in the array.
 * @param round If true, the result will be rounded to the nearest integer. Default is false (truncate).
 * @return The average value of the sensor readings, or 0 if numReadings is 0.
 */
unsigned int getAverageOfReadings(const unsigned int readings[], unsigned int numReadings, bool round = false) {
    if (numReadings == 0) {
        return 0;
    }

    uint64_t valuesTotal = 0;
    for (unsigned int i = 0; i < numReadings; i++) {
        valuesTotal += readings[i];
    }
    if (round) {
        return (valuesTotal + numReadings / 2) / numReadings;
    } else {
        return valuesTotal / numReadings;
    }
}

// Returns the average as a floating point value for higher accuracy (unsigned int version)
float getAverageOfReadingsFloat(const unsigned int readings[], unsigned int numReadings) {
    if (numReadings == 0) {
        return 0.0f;
    }

    uint64_t valuesTotal = 0;
    for (unsigned int i = 0; i < numReadings; i++) {
        valuesTotal += readings[i];
    }
    return static_cast<float>(valuesTotal) / static_cast<float>(numReadings);
}

// Returns the average as a floating point value for higher accuracy (float version)
/**
 * Calculates the average value of an array of floating point numbers.
 *
 * @param readings An array of float values representing sensor readings.
 * @param numReadings The number of readings in the array.
 * @return The average value of the sensor readings as a float, or 0.0f if numReadings is 0.
 */
float getAverageOfReadingsFloat(const float readings[], unsigned int numReadings) {
    if (numReadings == 0) {
        return 0.0f;
    }

    float valuesTotal = 0.0f;
    for (unsigned int i = 0; i < numReadings; i++) {
        valuesTotal += readings[i];
    }
    return valuesTotal / static_cast<float>(numReadings);
}

/**
 * Calculates the average value of an array of unsigned integers.
 *
 * @param readings An array of unsigned integers representing sensor readings.
 * @param numReadings The number of readings in the array.
 *
 * @return The average value of the sensor readings.
 *
 * @throws None.
 */
void method_averageRaw(const char *topic, unsigned int readings[], unsigned int numReadings) {
    // method 1
    unsigned int aveSensorValue = 0;
    aveSensorValue = getAverageOfReadings(readings, numReadings);
    MQTTpublishValue(topic, aveSensorValue);
}

/**
 * Finds the value that occurs most frequently in the given array.
 *
 * @param a The array of unsigned integers.
 * @param n The size of the array.
 *
 * @return The value that occurs most frequently in the array.
 *
 * @throws None.
 */
unsigned int getModeValue(unsigned int a[], unsigned int n) {
    unsigned int maxValue = 0;
    unsigned int maxCount = 0;
    unsigned int i;
    unsigned int j;

    for (i = 0; i < n; ++i) {
        unsigned int count = 0;

        for (j = 0; j < n; ++j) {
            if (a[j] == a[i])
                ++count;
        }

        if (count > maxCount) {
            maxCount = count;
            maxValue = a[i];
        }
    }

    return maxValue;
}
// MQTT stuff

/**
 * Callback function for handling MQTT message arrival.
 *
 * @param topic The topic of the MQTT message.
 * @param payload The payload of the MQTT message.
 * @param length The length of the payload.
 *
 * @throws None
 */
void callback(char *topic, byte *payload, unsigned int length) {
    // handle message arrived
    // format and display the whole MQTT message and payload
    char fullMQTTmessage[255];  // = "MQTT rxed thisisthetopicforthismesage and
                                // finally the payload, and a bit extra to make
                                // sure there is room in the string and even
                                // more chars";
    strcpy(fullMQTTmessage, "MQTT Rxed Topic: [");
    strcat(fullMQTTmessage, topic);
    strcat(fullMQTTmessage, "], ");
    // append payload and add \o terminator
    strcat(fullMQTTmessage, "Payload: [");
    strncat(fullMQTTmessage, (char *)payload, length);
    strcat(fullMQTTmessage, "]");

    Serial.println(fullMQTTmessage);

    readAndPublishSingleRaw("soil1/moisture_raw");
    // unsigned int numReadings = 32;
    // readMethodsPublish(numReadings, 200U);
}

unsigned int limitSensorValue(unsigned int reading, unsigned int min, unsigned int max) {
    if (reading > max) {
        return max;
    }
    if (reading < min) {
        return min;
    }
    return reading;
}

float scaleAndTransmit(unsigned int moistureReading, float drySensorMaxRaw, float wetSensorMinRaw, const char *topic) {
    // for a 0-100 output range
    // get range raw / 100 ggives steps per %
    // raw range = 3960 - 1530
    // steps per % = raw range/100
    // for 0 to top of range -> for 0 to raw_range - > inverted_reading_0_to_range = reading - wet_min_raw , where 0 is wet, and dry_max_raw is dry
    // to flip ,flipped =  abs( (dry_max_raw - wet_min_raw) - inverted_reading_0_to_range ), gives o-dry to raw_range-wet
    //  to scale to 0-100, scaled = (raw_range/100) * flipped
    char normalisedSensorValueStr[17];  // max 16 chars string
    float RAW_RANGE = (drySensorMaxRaw - wetSensorMinRaw);
    unsigned int limitedSensorValue = limitSensorValue(moistureReading, wetSensorMinRaw, drySensorMaxRaw);
    // unsigned int limitedSensorValue = moistureReading;
    float normalisedSensorValue = (float)abs(RAW_RANGE - ((float)limitedSensorValue - wetSensorMinRaw)) / (RAW_RANGE / 100.0f);
    // convert float to 1dp string
    sprintf(normalisedSensorValueStr, "%.1f", normalisedSensorValue);  // make the number into string using sprintf function
    // Serial.print("Sampling Sensor.....(RAW_RANGE/100.0f)..");      // Serial.println((RAW_RANGE / 100.0f));
    Serial.print("scaleAndTransmit..");
    Serial.println(normalisedSensorValueStr);
    MQTTclient.publish(topic, normalisedSensorValueStr);
    return normalisedSensorValue;
}
