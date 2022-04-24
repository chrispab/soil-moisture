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

    // assume filtered for 'irbridge/amplifier/' already
    // get last part of string - the command,
    // then irsend the code version of the command -m use a struct?
    //  https://stackoverflow.com/questions/5193570/value-lookup-table-in-c-by-strings
    // char commandStr[25];

    //         // irsend.sendNEC(POWER_ON);
    //! possible incoming topics and payload: "irbridge/amplifier/standby"     "on|off"
    // else if (strcmp(topic, "irbridge/amplifier/code") == 0) {  // raw code
    //     Serial.print("plain NEC code Tx : ");
    //     unsigned long actualval;
    //     actualval = strtoul((char *)payload, NULL, 10);
    //     Serial.println(actualval);
    //     // irsend.sendNEC(actualval);
    // }
}

IPAddress mqttBroker(192, 168, 0, MQTT_LAST_OCTET);
WiFiClient myWiFiClient;
PubSubClient MQTTclient(mqttBroker, 1883, callback, myWiFiClient);

const uint32_t kBaudRate = 115200;

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
}






unsigned int limitSensorValue(unsigned int reading, unsigned int min, unsigned int max){
    if(reading > max){
        return max;
    }
    if(reading < min){
        return min;
    }
    return reading;

}


#define MQTT_TRANSMIT_INTERVAL (15 * 60 * 1000) //MS DELAY BETWEEN SAMPLES
size_t lastMQTTTransmitMs = millis() - MQTT_TRANSMIT_INTERVAL;
float prevFilteredValue = 0.0;
#define RUNNING_SAMPLE_INTERVAL ( 10 * 1000 )
size_t lastMoistureSampleMs = millis() - RUNNING_SAMPLE_INTERVAL;
size_t runningSensorReading = analogRead(SENSOR_PIN); //running total reading - taken every RUNNING_SAMPLE_INTERVAL

unsigned int readSoilSensor() {

    size_t now = millis();
    size_t sensorValue;
    char sensorValueStr[17];            // max 16 chars string
    char normalisedSensorValueStr[17];  // max 16 chars string
    char normalisedRangeSensorValueStr[17];  // max 16 chars string
    char newFilterdValueStr[17];  // max 16 chars string newFilterdValue

    sensorValue = 0;
    if(now - lastMoistureSampleMs > RUNNING_SAMPLE_INTERVAL){
        // runningSensorReading = (runningSensorReading + analogRead(SENSOR_PIN))/2;
        runningSensorReading = ( ((runningSensorReading*80)/100) + ((analogRead(SENSOR_PIN)*20)/100) );
        lastMoistureSampleMs = now;
    }
    if (now - lastMQTTTransmitMs > MQTT_TRANSMIT_INTERVAL) {
        lastMQTTTransmitMs = now;

        //publish telemetry
        MQTTclient.publish("soil1/version", VERSION);

        Serial.print("Sampling Sensor.....RAW..");

        //try ave of several readings?
        size_t count = 20;
        size_t total = 0;
        sensorValue = analogRead(SENSOR_PIN);
        size_t moisture_raw = sensorValue;
        for (size_t i = 0; i < count; i++)
        {
            delay(500);
            total = total + sensorValue;
            sensorValue = analogRead(SENSOR_PIN);
        }
        size_t moisture_4 = total/count;

        //sensorValue = analogRead(SENSOR_PIN);


        Serial.println(moisture_raw);

        utoa(moisture_raw, sensorValueStr, 10);
        MQTTclient.publish("soil1/moisture_raw", sensorValueStr);

        // for a 0-100 output range
        // get range raw / 100 ggives steps per %
        // raw range = 3960 - 1530
        // steps per % = raw range/100

        // for 0 to top of range -> for 0 to raw_range - > inverted_reading_0_to_range = reading - wet_min_raw , where 0 is wet, and dry_max_raw is dry

        // to flip ,flipped =  abs( (dry_max_raw - wet_min_raw) - inverted_reading_0_to_range ), gives o-dry to raw_range-wet
        //  to scale to 0-100, scaled = (raw_range/100) * flipped
        // #define DRY_SENSOR_MAX_RAW 3980.0f



        #define DRY_SENSOR_MAX_RAW 3950.0f
        #define WET_SENSOR_MIN_RAW 1500.0f
        #define RAW_RANGE (DRY_SENSOR_MAX_RAW - WET_SENSOR_MIN_RAW) // 2450
        unsigned int limitedSensorValue = limitSensorValue(moisture_raw, WET_SENSOR_MIN_RAW, DRY_SENSOR_MAX_RAW );

        float normalisedSensorValue = (float)abs(RAW_RANGE - ((float)limitedSensorValue - WET_SENSOR_MIN_RAW)) / (RAW_RANGE / 100.0f);
        // convert float to 1dp string
        sprintf(normalisedSensorValueStr, "%.1f", normalisedSensorValue);  // make the number into string using sprintf function
        // Serial.print("Sampling Sensor.....(RAW_RANGE/100.0f)..");      // Serial.println((RAW_RANGE / 100.0f));
        Serial.print("Sampling Sensor.....NOR..");
        Serial.println(normalisedSensorValueStr);
        MQTTclient.publish("soil1/moisture", normalisedSensorValueStr);


        //partial normalising
        #define dryMAX_RAW 3290.0f //2772.8 is eq to 74.4% normalised
        #define wetMIN_RAW 1710.0f//1685 is eq to  normalised - 30%
        //1725, 1712
        #define RAW_R (dryMAX_RAW - wetMIN_RAW)  // 1088 

        limitedSensorValue = limitSensorValue(moisture_raw, wetMIN_RAW, dryMAX_RAW );
        float normalisedRangeSensorValue = (float)abs((RAW_R - ((float)limitedSensorValue - wetMIN_RAW)) / (RAW_R / 100.0f));
        // convert float to 1dp string
        sprintf(normalisedRangeSensorValueStr, "%.1f", normalisedRangeSensorValue);  // make the number into string using sprintf function
        // Serial.print("Sampling Sensor.....(RAW_RANGE/100.0f)..");      // Serial.println((RAW_RANGE / 100.0f));
        Serial.print("Sampling Sensor.....NOR limit sub range .._2");
        Serial.println(normalisedRangeSensorValueStr);
        MQTTclient.publish("soil1/moisture_2", normalisedRangeSensorValueStr);

        //now try a filtered ver of normalisedRangeSensorValue
        //newFilterdValue = (prevFilteredValue/2) + (normalisedRangeSensorValue/2)
        if (prevFilteredValue==0){
            prevFilteredValue=normalisedRangeSensorValue;
        }
        float newFilterdValue = (prevFilteredValue/2) + (normalisedRangeSensorValue/2);
        prevFilteredValue = newFilterdValue;
        sprintf(newFilterdValueStr, "%.1f", newFilterdValue);  // make the number into string using sprintf function
        Serial.print("Sampling Sensor.....filtered limit sub range _3..");
        Serial.println(newFilterdValueStr);
        MQTTclient.publish("soil1/moisture_3", newFilterdValueStr);




        //averaged count
        //moisture_4
        limitedSensorValue = limitSensorValue(moisture_4, wetMIN_RAW, dryMAX_RAW );
        float normalisedRangeSensorValue4 = (float)abs((RAW_R - ((float)limitedSensorValue - wetMIN_RAW)) / (RAW_R / 100.0f));
        // convert float to 1dp string
        sprintf(normalisedRangeSensorValueStr, "%.1f", normalisedRangeSensorValue4);  // make the number into string using sprintf function
        // Serial.print("Sampling Sensor.....(RAW_RANGE/100.0f)..");      // Serial.println((RAW_RANGE / 100.0f));
        Serial.print("Sampling Sensor.....NOR limit sub range ..moisture_4");
        Serial.println(normalisedRangeSensorValueStr);
        MQTTclient.publish("soil1/moisture_4", normalisedRangeSensorValueStr);

        //averaged count
        //moisture_5
        limitedSensorValue = limitSensorValue(runningSensorReading, wetMIN_RAW, dryMAX_RAW );
        float normalisedRangeSensorValue5 = (float)abs((RAW_R - ((float)limitedSensorValue - wetMIN_RAW)) / (RAW_R / 100.0f));
        // convert float to 1dp string
        sprintf(normalisedRangeSensorValueStr, "%.1f", normalisedRangeSensorValue5);  // make the number into string using sprintf function
        // Serial.print("Sampling Sensor.....(RAW_RANGE/100.0f)..");      // Serial.println((RAW_RANGE / 100.0f));
        Serial.print("Sampling Sensor.....NOR limit sub range ..moisture_5");
        Serial.println(normalisedRangeSensorValueStr);
        MQTTclient.publish("soil1/moisture_5", normalisedRangeSensorValueStr);





    }
    return sensorValue;
}

void loop() {
    // connectWiFi();
    // maybe checkwifi here
    connectWiFi();

    if (!MQTTclient.connected()) {
        // Attempt to reconnect
        reconnectMQTT();  // Attempt to reconnect
    } else {
        // Client is connected
        MQTTclient.loop();  // process any MQTT stuff, returned in callback
    }
    // unsigned int soilSensorValue =
    readSoilSensor();

    ArduinoOTA.handle();

    heartBeatLED.update();
    blueBeatLED.update();
}
