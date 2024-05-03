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

LedFader heartBeatLED(GREEN_LED_PIN, 1, 0, 255, HEART_BEAT_TIME);
LedFader blueBeatLED(ONBOARD_LED_PIN, 2, 0, 50, BLUE_BEAT_TIME);

IPAddress mqttBroker(192, 168, 0, MQTT_LAST_OCTET);
WiFiClient myWiFiClient;

// forward func declarations
void callback(char *topic, byte *payload, unsigned int length);
PubSubClient MQTTclient(mqttBroker, 1883, callback, myWiFiClient);
unsigned int readAndTxMoistureSensor();
void readAndPublishSingleRaw(const char *topic);
unsigned int readMethodsPublish(unsigned int numReadings, unsigned int msBetweenReadings);
void getReadings(unsigned int &numReadings, unsigned int msBetweenReadings, unsigned int readings[256]);
void method_averageRaw(const char *topic, unsigned int readings[], unsigned int numReadings);
unsigned int getModeValue(unsigned int a[], unsigned int n);
unsigned int getAverageOfReadings(unsigned int readings[], unsigned int numReadings);

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
        runningSensorReading = (runningSensorReading + analogRead(SENSOR_PIN))/2;
        // runningSensorReading = (((runningSensorReading * 80) / 100) + ((analogRead(SENSOR_PIN) * 20) / 100));
        lastMoistureSampleMs = now;
    }
    if (now - lastMQTTTransmitMs > MQTT_TELE_PERIOD_MS) {
        lastMQTTTransmitMs = now;

        // publish telemetry
        MQTTclient.publish("soil1/version", VERSION);

        // do all the different methods
        moisture_raw = readMethodsPublish(255, 50);
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
unsigned int readMethodsPublish(unsigned int numReadings, unsigned int msBetweenReadings) {
    // limit readings to max of 256 samples
    unsigned int readings[256];
    getReadings(numReadings, msBetweenReadings, readings);

    readAndPublishSingleRaw("soil1/moisture_method0_single");

    // method 1
    method_averageRaw("soil1/moisture_method1_average", readings, numReadings);

    // method 2 - find most common value
    unsigned int modeValue = getModeValue(readings, numReadings);
    MQTTpublishValue("soil1/moisture_method2_mode", modeValue);

    // method 3 - remove outliers, then average
    unsigned int averageValue = getAverageOfReadings(readings, numReadings);
    // remove outliers
    int outlierLimit = 4;
    for (int i = 0; i < numReadings; i++) {
        if (abs((int)readings[i] - (int)averageValue) > outlierLimit) {  // outlier
            MQTTpublishValue("soil1/moisture_method3_excluded", readings[i]);
            // replace with average
            readings[i] = averageValue;
        }
    }
    averageValue = getAverageOfReadings(readings, numReadings);
    MQTTpublishValue("soil1/moisture_method3_average", averageValue);

    return averageValue;
}

/**
 * Reads a specified number of analog sensor readings with a specified time interval between each reading.
 *
 * @param numReadings Reference to an unsigned integer that holds the number of readings to take. Limited to a maximum of 256.
 * @param msBetweenReadings The time interval between each reading in milliseconds.
 * @param readings An array of unsigned integers that will hold the readings. The array must have a size of at least `numReadings`.
 *
 * @throws None
 */
void getReadings(unsigned int &numReadings, unsigned int msBetweenReadings, unsigned int readings[256]) {
    if (numReadings > 256) numReadings = 256;

    // throw away first reading
    delay(msBetweenReadings);
    analogRead(SENSOR_PIN);

    // read in the samples
    for (int i = 0; i < numReadings; i++) {
        delay(msBetweenReadings);
        readings[i] = analogRead(SENSOR_PIN);
    }
    // return readings;
}

/**
 * Calculates the average value of an array of unsigned integers.
 *
 * @param readings An array of unsigned integers representing sensor readings.
 * @param numReadings The number of readings in the array.
 *
 * @return The average value of the readings.
 *
 * @throws None.
 */
unsigned int getAverageOfReadings(unsigned int readings[], unsigned int numReadings) {
    unsigned int valuesTotal = 0;
    for (int i = 0; i < numReadings; i++) {
        valuesTotal = valuesTotal + readings[i];
    }
    return valuesTotal / numReadings;
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
    unsigned int maxValue = 0, maxCount = 0, i, j;

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
    readMethodsPublish(16, 200);
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
