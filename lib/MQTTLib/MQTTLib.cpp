// #include "WebSocketLib.h"
#include <PubSubClient.h>

#include "config.h"
extern PubSubClient MQTTclient;

bool MQTTNewData = false;
int MQTTNewState = 0;      // 0 or 1
int MQTTSocketNumber = 1;  // 1-16

// MQTT stuff
// IPAddress mqttBroker(192, 168, 0, 200);
// char subscribeTopic[] = "irbridge/#";
// char subscribeTopic2[] = "irbridge/amplifier/code";
// char subscribeTopic3[] = "Zone3/#";
// char subscribeTopic[] = "#";

// char publishTempTopic[] = "433Bridge/Temperature";
// char publishHumiTopic[] = "433Bridge/Humidity";

/**
 *
 *
 *
 */
void connectMQTT() {
    bool MQTTConnectTimeout = false;
    unsigned long checkPeriodMillis = 20000;
    unsigned long timeOutMillis = 3000;
    unsigned long now;
    unsigned long nowMillis = millis();
    static unsigned long lastReconnectAttemptMillis =
        nowMillis - checkPeriodMillis - 1000;

    // is it time to chek the MQTT connection again?
    if ((nowMillis - lastReconnectAttemptMillis) > checkPeriodMillis) {
        // myWebSerial.println("ready to try MQTT reconnectMQTT...");
        Serial.print("nowMillis : ");
        Serial.println(nowMillis);
        Serial.println("Checking if MQTT needs reconnect");

        // if (!MQTTclient.connected()) {
        MQTTConnectTimeout = false;
        while (!MQTTclient.connected() && !MQTTConnectTimeout)  // loop till connected or timed out
        {
            Serial.println("MQTT not connected so Attempting MQTT connection...");
            // boolean connect(const char* id, const char* willTopic, uint8_t
            // willQos, boolean willRetain, const char* willMessage);
            if (MQTTclient.connect(MQTT_CLIENT_NAME, LWT_TOPIC, 1, true, "Offline")) {
                // myWebSerial.println("connected to MQTT server");
                MQTTclient.publish(LWT_TOPIC, "Online", true);  // ensure send online
                // MQTTclient.publish(publishLWTTopic, "Online");
                // MQTTclient.subscribe(subscribeTopic);
                // MQTTclient.subscribe(subscribeTopic2);
                // MQTTclient.subscribe(subscribeTopic3);
            } else {
                Serial.println(MQTTclient.state());
            }
            now = millis();
            MQTTConnectTimeout = ((now - nowMillis) > timeOutMillis) ? true : false;
        }
        (!MQTTConnectTimeout)
            ? Serial.println("MQTT Connection made or already active !")
            : Serial.println("MQTT Connection attempt Timed Out!");
        now = millis();
        lastReconnectAttemptMillis = now;
    }
}

unsigned long lastReconnectAttempt = 0;

bool reconnectMQTT() {
    // Serial.println("MQTT is not connected.. trying to connect now");

    unsigned long now = millis();
    if ((now - lastReconnectAttempt) > 5000) {
        lastReconnectAttempt = now;

        Serial.println("..trying to connect MQTT..");

        // Attempt to reconnect
        if (MQTTclient.connect(MQTT_CLIENT_NAME, LWT_TOPIC, 1, true, "Offline")) {
            // Once connected, publish an announcement...
            // MQTTclient.publish("outTopic", "hello world");
            MQTTclient.publish(LWT_TOPIC, "Online", true);  // ensure send online
            // ... and resubscribe
            // MQTTclient.subscribe(subscribeTopic);

            Serial.println("..MQTT is now connected");
        }
    }
    // Return true if the MQTT client is connected, false otherwise.
    return MQTTclient.connected();
}

extern char* getTimeStr();

// MQTTclient call back if mqtt messsage rxed (cos has been subscribed  to)
void MQTTRxcallback(char* topic, byte* payload, unsigned int length) {
    char fullMQTTmessage[256];
    snprintf(fullMQTTmessage, sizeof(fullMQTTmessage), "MQTT Rxed [%s]:[%.*s]", 
             topic, length, (char*)payload);

    Serial.println(fullMQTTmessage);
#ifdef DEBUG_WSERIAL
    myWebSerial.println(fullMQTTmessage);
#endif
}

// void MQTTLibSetup(void) {}
// #include "WebSerial.h"
// extern WebSerial myWebSerial;
// #include "My433Transmitter.h"
// extern My433Transmitter transmitter;
void processMQTTMessage(void) {
    // char msg[40] = "SSS == Operate Socket: ";
    char buff[10];

    // strcpy(buff, "Socket : ");
    // if socket number is  valid one -
    sprintf(buff, "%d", (MQTTSocketNumber));
    // strcat(msg, buff);
    // strcat(msg, "-");

    if (MQTTNewData) {
        // digitalWrite(ESP32_ONBOARD_BLUE_LED_PIN, MQTTNewState);
        // Serial
        // myWebSerial.println("process MQTT - MQTTSocketNumber...");

        // myWebSerial.println(buff);

        //                     myWebSerial.println("process MQTT -
        //                     MQTTNewState...");
        // sprintf(buff, "%d", (MQTTNewState));

        // myWebSerial.println(buff);

        // transmitter.operateSocket(MQTTSocketNumber - 1, MQTTNewState);
        MQTTNewData = false;  // indicate not new data now, processed
    }
}
