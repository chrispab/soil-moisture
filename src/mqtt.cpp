// MQTT stuff
#include <Arduino.h>

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

    // readAndPublishSingleRaw("soil1/moisture_raw");
    // unsigned int numReadings = 32;
    // readMethodsPublish(numReadings, 200U);
}