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
    char fullMQTTmessage[256];
    
    // Safely format the message
    snprintf(fullMQTTmessage, sizeof(fullMQTTmessage), "MQTT Rxed Topic: [%s], Payload: [%.*s]", 
             topic, length, (char*)payload);

    Serial.println(fullMQTTmessage);
}