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
#include "mqtt.h"
#include "sensor.h"
#include "version.h"

#define GREEN_LED_PIN GPIO_NUM_12  //
// #define IR_LED_PIN GPIO_NUM_13      //
#define ONBOARD_LED_PIN GPIO_NUM_2  //
// #define RX_PIN GPIO_NUM_14          //

#define HEART_BEAT_TIME 800
#define BLUE_BEAT_TIME 300

// Outlier limit for sensor readings
#define OUTLIER_LIMIT 4

LedFader heartBeatLED(GREEN_LED_PIN, 1, 0, 255, HEART_BEAT_TIME);
LedFader blueBeatLED(ONBOARD_LED_PIN, 2, 0, 50, BLUE_BEAT_TIME);

IPAddress mqttBroker(192, 168, 0, MQTT_LAST_OCTET);
WiFiClient myWiFiClient;

// forward func declarations
// void callback(char *topic, byte *payload, unsigned int length);
PubSubClient MQTTclient(mqttBroker, 1883, callback, myWiFiClient);
unsigned int readAndTxSensorIfDue();
uint16_t readAnalogueSensorN(unsigned int numReadings, unsigned int msBetweenReadings);
void readAndPublishSingleRaw(const char* topic);
unsigned int readMethodsPublish(unsigned int& numReadings, unsigned int msBetweenReadings);
void method_averageRaw(const char* topic, unsigned int readings[], unsigned int numReadings);
unsigned int getModeValue(unsigned int a[], unsigned int n);
unsigned int getAverageOfReadings(const unsigned int readings[], unsigned int numReadings, bool round);
// float getAverageOfReadings(const unsigned int readings[], unsigned int numReadings);
float getAverageOfReadings(const float readings[], unsigned int numReadings);
void publishValueToTopic(const char* topic, uint16_t value);

void setup() {
    Serial.begin(115200);
    while (!Serial)  // Wait for the serial connection to be establised.
        delay(50);

    Serial.println("Hi from soil1 ... Booting");
    connectWiFi();
    printWifiStatus();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.flush();

    delay(5000);
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

    // todo change to dynamic name subscription
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

    readAndTxSensorIfDue();
}

unsigned int lastMQTTTransmitMs = millis() - MQTT_TELE_PERIOD_MS - 1000;
float prevFilteredValue = 0.0;
unsigned int lastMoistureSampleMs = millis() - RUNNING_SAMPLE_INTERVAL_MS - 1000;
unsigned int runningSensorReading = analogRead(SENSOR_PIN);  // running total reading - taken every RUNNING_SAMPLE_INTERVAL_MS
float scaleAndTransmit(unsigned int moistureReading, float drySensorMaxRaw, float wetSensorMinRaw, const char* topic);
float processMovingAverageValue(unsigned int sensorValue);
static unsigned int movingAverageReadingsCount = 0;

/**
 * @brief Reads the soil moisture sensor and transmits the data over MQTT if the telemetry period has passed.
 *
 * This function checks if the time since the last MQTT transmission is greater than the configured telemetry period (MQTT_TELE_PERIOD_MS).
 * If it is, the function will:
 * 1. Publish the current firmware version.
 * 2. Call `readMethodsPublish()` to perform a series of sensor readings and publish various interpretations of the data (e.g., raw, average, mode).
 * 3. Print the final raw sensor value to the serial console.
 *
 * The function uses a non-blocking approach by checking the elapsed time with `millis()`.
 *
 * @note This function contains commented-out code that suggests alternative implementations,
 *       such as calculating a running sensor reading or scaling and transmitting a percentage value.
 *
 * @return unsigned int The last raw sensor reading, or 0 if no reading was performed in the current call.
 */
unsigned int readAndTxSensorIfDue() {
    unsigned int now = millis();
    unsigned int sensor_raw;

    sensor_raw = 0;

    if (now - lastMQTTTransmitMs > MQTT_TELE_PERIOD_MS) {
        lastMQTTTransmitMs = now;

        // commandString = "AT+CPBW=1,\"";
        // commandString += number1;
        // commandString += "\",145,\"Number1\"";
        // sendATCommand(commandString.c_str(), 1000);
        
        // String topic = MQTT_TOPIC_PREFIX + MOISTURE_SENSOR_ID;
        // MQTTclient.publish(topic.c_str(),1/version";

        // publish telemetry
        MQTTclient.publish(MQTT_VERSION_TOPIC, VERSION);
        MQTTclient.publish(MQTT_TELE_PERIOD_MS_TOPIC, String(MQTT_TELE_PERIOD_MS).c_str());

        // turn on sensor POWER
        pinMode(SENSOR_POWERSUPPLY_PIN, OUTPUT);
        digitalWrite(SENSOR_POWERSUPPLY_PIN, HIGH);
        delay(100);

        // read sensor
        // uint16_t rawValue = analogRead(SENSOR_PIN);
        uint16_t rawValue = readAnalogueSensorN(20,10); // read the sensor twenty times with a short delay between each reading and return the average        Serial.print(now);
        Serial.print(": ");
        Serial.print("Raw sensor value: ");
        Serial.println(rawValue);
        // publishValueToTopic("soil1/moisture_raw", rawValue);
        publishValueToTopic(SENSOR_METHOD0_SINGLE_RAW_TOPIC, rawValue);
        const float movingAverageValue = processMovingAverageValue(rawValue);
        publishValueToTopic(SENSOR_METHOD5_BATCH_MOVING_AVERAGE_FLOAT_TOPIC, movingAverageValue);
        publishValueToTopic(SENSOR_MOVING_AVERAGE_WINDOW_SIZE_TOPIC, movingAverageReadingsCount);
        publishValueToTopic(SENSOR_MOVING_AVERAGE_TOPIC, movingAverageValue);
        sensor_raw = rawValue;

        // turn off sensor power
        digitalWrite(SENSOR_POWERSUPPLY_PIN, LOW);
        pinMode(SENSOR_POWERSUPPLY_PIN, INPUT);
    }
    return sensor_raw;
}

/**
 * Publishes a value to the specified MQTT topic.
 *
 * @param topic the topic to publish the value to
 * @param value the value to publish
 *
 * @throws ErrorType if there is an error publishing the value
 */
void MQTTpublishValue(const char* topic, unsigned int value) {
    char valueStr[20];  // max of 20 chars string
    utoa(value, valueStr, 10);
    MQTTclient.publish(topic, valueStr);
}

void MQTTpublishValue(const char* topic, float value) {
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
void readAndPublishSingleRaw(const char* topic) {
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

/**
 * @brief Reads the analog sensor multiple times and returns the average.
 * 
 * This implementation discards the first reading to allow the ADC to stabilize
 * and then takes a series of samples with a short delay between each.
 * 
 * @param numReadings The number of samples to take and average.
 * @return uint16_t The averaged sensor value.
 */
uint16_t readAnalogueSensorN(unsigned int numReadings, unsigned int msBetweenReadings) {
    if (numReadings == 0) return 0;
    // analogSetCycles(255);//: set the number of cycles per sample. Default is 8. Range: 1 to 255.
    // analogSetSamples(3); //: set the number of samples in the range. Default is 1 sample. It has an effect of increasing sensitivity.
    analogRead(SENSOR_PIN); // Stabilize ADC
    uint32_t sum = 0;
    for (unsigned int i = 0; i < numReadings; i++) {
        delay(msBetweenReadings);
        sum += analogRead(SENSOR_PIN);
    }
    return static_cast<uint16_t>(sum / numReadings);
}

// Publish a value to the given topic
void publishValueToTopic(const char* topic, uint16_t value) {
    MQTTpublishValue(topic, static_cast<unsigned int>(value));
}



/**
 * @brief Calculates and publishes a moving average of sensor readings using floating-point values for higher accuracy.
 *
 * This function maintains a static array of floating-point readings and calculates a moving average.
 * The window size for the moving average is defined by `MOVING_AVERAGE_WINDOW_SIZE`.
 * The calculated moving average is then rounded to one decimal place and published to the MQTT topic
 * defined by `SENSOR_METHOD5_BATCH_MOVING_AVERAGE_FLOAT_TOPIC`.
 *
 * @param numReadings Not directly used in this function, but kept for signature consistency.
 * @param msBetweenReadings Not directly used in this function, but kept for signature consistency.
 */
float processMovingAverageValue(unsigned int sensorValue) {
    // method 5 - a moving average of the last 20 readings, but with floating point values for higher accuracy
    // #define MOVING_AVERAGE_WINDOW_SIZE 10

    static float movingAverageReadings[MOVING_AVERAGE_WINDOW_SIZE];
    // static unsigned int movingAverageReadingsCount = 0;

    if (movingAverageReadingsCount < MOVING_AVERAGE_WINDOW_SIZE) {
        movingAverageReadings[movingAverageReadingsCount++] = static_cast<float>(sensorValue);
    } else {
        // shift the readings to the left
        for (unsigned int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE - 1; i++) {
            movingAverageReadings[i] = movingAverageReadings[i + 1];
        }
        movingAverageReadings[MOVING_AVERAGE_WINDOW_SIZE - 1] = static_cast<float>(sensorValue);
    }
    // float movingAverageValue = getAverageOfReadings(movingAverageReadings, movingAverageReadingsCount);
    float movingAverageValue = getAverageOfReadings(movingAverageReadings, movingAverageReadingsCount);
    // Limit to 1 decimal place
    movingAverageValue = roundf(movingAverageValue * 10.0f) / 10.0f;
    // MQTTpublishValue("soil1/moisture_method5_moving_average_float", movingAverageValue);
    // MQTTpublishValue(SENSOR_METHOD5_BATCH_MOVING_AVERAGE_FLOAT_TOPIC, movingAverageValue);
    // MQTTclient.publish(, movingAverageValue);
    return movingAverageValue;
}


// Returns the average as a floating point value for higher accuracy (float version)
/**
 * Calculates the average value of an array of floating point numbers.
 *
 * @param readings An array of float values representing sensor readings.
 * @param numReadings The number of readings in the array.
 * @return The average value of the sensor readings as a float, or 0.0f if numReadings is 0.
 */
float getAverageOfReadings(const float readings[], unsigned int numReadings) {
    if (numReadings == 0) {
        return 0.0f;
    }

    float valuesTotal = 0.0f;
    for (unsigned int i = 0; i < numReadings; i++) {
        valuesTotal += readings[i];
    }
    return valuesTotal / static_cast<float>(numReadings);
}
