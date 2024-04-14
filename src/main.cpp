#include <Arduino.h>
// #include <IRremoteESP8266.h>
#include <assert.h>
// #include <AsyncElegantOTA.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <IRsend.h>
#include <PubSubClient.h>

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
// #include "ircodes.h"

#define GREEN_LED_PIN GPIO_NUM_12   //
#define IR_LED_PIN GPIO_NUM_13      //
#define ONBOARD_LED_PIN GPIO_NUM_2  //
#define RX_PIN GPIO_NUM_14          //

// const uint16_t kIrLed = IR_LED_PIN;  // ESP8266 GPIO pin to use.
// IRsend irsend(kIrLed);               // Set the GPIO to be used to sending the message.

#define HEART_BEAT_TIME 800
#define BLUE_BEAT_TIME 300

LedFader heartBeatLED(GREEN_LED_PIN, 1, 0, 255, HEART_BEAT_TIME);
LedFader blueBeatLED(ONBOARD_LED_PIN, 2, 0, 50, BLUE_BEAT_TIME);

IPAddress mqttBroker(192, 168, 0, MQTT_LAST_OCTET);
WiFiClient myWiFiClient;

const uint32_t kBaudRate = 115200;

void callback(char *topic, byte *payload, unsigned int length);

PubSubClient MQTTclient(mqttBroker, 1883, callback, myWiFiClient);

void MQTTpublishValue(const char *topic, unsigned int value) {
    char valueStr[20];  // max 16 chars string
    utoa(value, valueStr, 10);
    MQTTclient.publish(topic, valueStr);
}

void readAndPublishSingleRaw(const char *topic) {
    unsigned int sensorValue;
    delay(100);
    sensorValue = analogRead(SENSOR_PIN);
    MQTTpublishValue(topic, sensorValue);
}

void readAndPublishAverageRaw(unsigned int numReadings, unsigned int msBetweenReadings) {
    // method 1
    unsigned int sensorValue = 0;
    sensorValue = analogRead(SENSOR_PIN);
    for (int i = 0; i < numReadings; i++) {
        delay(msBetweenReadings);
        sensorValue = (sensorValue + analogRead(SENSOR_PIN)) / 2;
    }
    MQTTpublishValue("soil1/moisture1_average_raw", sensorValue);

    // method 2
    // try ave of several readings?
    unsigned int valuesTotal = 0;
    for (int i = 0; i < numReadings; i++) {
        delay(msBetweenReadings);
        valuesTotal = valuesTotal + analogRead(SENSOR_PIN);
    }
    unsigned int averageValue = valuesTotal / numReadings;
    MQTTpublishValue("soil1/moisture2_average_raw", averageValue);
}

// MQTT stuff
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
    readAndPublishAverageRaw(16, 200);
}

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

unsigned int limitSensorValue(unsigned int reading, unsigned int min, unsigned int max) {
    if (reading > max) {
        return max;
    }
    if (reading < min) {
        return min;
    }
    return reading;
}

float scaleAndTransmit(unsigned int moistureReading, float DRY_SENSOR_MAX_RAW, float WET_SENSOR_MIN_RAW, const char *topic) {
    // for a 0-100 output range
    // get range raw / 100 ggives steps per %
    // raw range = 3960 - 1530
    // steps per % = raw range/100
    // for 0 to top of range -> for 0 to raw_range - > inverted_reading_0_to_range = reading - wet_min_raw , where 0 is wet, and dry_max_raw is dry
    // to flip ,flipped =  abs( (dry_max_raw - wet_min_raw) - inverted_reading_0_to_range ), gives o-dry to raw_range-wet
    //  to scale to 0-100, scaled = (raw_range/100) * flipped
    char normalisedSensorValueStr[17];  // max 16 chars string
    float RAW_RANGE = (DRY_SENSOR_MAX_RAW - WET_SENSOR_MIN_RAW);
    // unsigned int limitedSensorValue = limitSensorValue(moistureReading, WET_SENSOR_MIN_RAW, DRY_SENSOR_MAX_RAW);
    unsigned int limitedSensorValue = moistureReading;
    float normalisedSensorValue = (float)abs(RAW_RANGE - ((float)limitedSensorValue - WET_SENSOR_MIN_RAW)) / (RAW_RANGE / 100.0f);
    // convert float to 1dp string
    sprintf(normalisedSensorValueStr, "%.1f", normalisedSensorValue);  // make the number into string using sprintf function
    // Serial.print("Sampling Sensor.....(RAW_RANGE/100.0f)..");      // Serial.println((RAW_RANGE / 100.0f));
    Serial.print("scaleAndTransmit..");
    Serial.println(normalisedSensorValueStr);
    MQTTclient.publish(topic, normalisedSensorValueStr);
    return normalisedSensorValue;
}

#define MQTT_TRANSMIT_INTERVAL (30 * 1000)  // MS DELAY BETWEEN SAMPLES
unsigned int lastMQTTTransmitMs = millis() - MQTT_TRANSMIT_INTERVAL;
float prevFilteredValue = 0.0;
#define RUNNING_SAMPLE_INTERVAL (20 * 1000)
unsigned int lastMoistureSampleMs = millis() - RUNNING_SAMPLE_INTERVAL;
unsigned int runningSensorReading = analogRead(SENSOR_PIN);  // running total reading - taken every RUNNING_SAMPLE_INTERVAL

unsigned int readMoistureSensor() {
    unsigned int now = millis();
    unsigned int sensorValue;
    char sensorValueStr[17];                   // max 16 chars string
    char normalisedSensorValueStr[17];         // max 16 chars string
    char normalisedRangeSensorValueStr[17];    // max 16 chars string
    char newFilterdValueStr[17];               // max 16 chars string newFilterdValue
    char normalisedRangeSensorValue_6Str[17];  // max 16 chars string

    sensorValue = 0;
    if (now - lastMoistureSampleMs > RUNNING_SAMPLE_INTERVAL) {
        // runningSensorReading = (runningSensorReading + analogRead(SENSOR_PIN))/2;
        runningSensorReading = (((runningSensorReading * 80) / 100) + ((analogRead(SENSOR_PIN) * 20) / 100));
        lastMoistureSampleMs = now;
    }
    if (now - lastMQTTTransmitMs > MQTT_TRANSMIT_INTERVAL) {
        lastMQTTTransmitMs = now;

        // publish telemetry
        MQTTclient.publish("soil1/version", VERSION);
        readAndPublishSingleRaw("soil1/moisture_raw");
        readAndPublishAverageRaw(255, 50);

        sensorValue = analogRead(SENSOR_PIN);
        unsigned int moisture_raw = sensorValue;
        Serial.println(moisture_raw);

        // float DRY_SENSOR_MAX_RAW = 3950.0f;
        // float WET_SENSOR_MIN_RAW = 1500.0f;
        // const RAW_0PC_DRY = 3300.0;
        // const RAW_100PC_WET = 2500.0;
        float DRY_SENSOR_MAX_RAW = 3300.0f;
        float WET_SENSOR_MIN_RAW = 2500.0f;
        scaleAndTransmit(moisture_raw, DRY_SENSOR_MAX_RAW, WET_SENSOR_MIN_RAW, "soil1/moisture");
    }
    return sensorValue;
}

void loop() {
    connectWiFi();

    if (!MQTTclient.connected()) {
        reconnectMQTT();    // Attempt to reconnect
    } else {                // Client is connected
        MQTTclient.loop();  // process any MQTT stuff, returned in callback
    }
    readMoistureSensor();

    ArduinoOTA.handle();

    heartBeatLED.update();
    blueBeatLED.update();
}
