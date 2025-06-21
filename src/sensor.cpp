#include "sensor.h"

#include <Arduino.h>

#include "config.h"
#include <PubSubClient.h>
extern PubSubClient MQTTclient;
#
/**
 * Reads a specified number of analog sensor readings with a specified time interval between each reading.
 *
 * @param numReadings Reference to an unsigned integer that holds the number of readings to take. Limited to a maximum of 256.
 * @param msBetweenReadings The time interval between each reading in milliseconds.
 * @param readings An array of unsigned integers that will hold the readings. The array must have a size of at least `numReadings` (does not need to be `MAX_READINGS`).
 *
 * @note The `readings` parameter must be a valid pointer to an array of at least `numReadings` elements.
 *
 * @throws None
 */
void readSensorBatch(unsigned int &numReadings, unsigned int msBetweenReadings, unsigned int *readings) {
    MQTTclient.publish("soil1/status", "reading sensor batch");
    // readSensorBatch(numReadings, msBetweenReadings, readings);

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
    // return readings;  // return the readings array
    // Note: The readings array is passed by reference, so the values are updated in the caller's context.
    // The function does not return a value, but the readings are stored in the provided array.
    // If you want to return the readings array, you can change the function signature to return `unsigned int*` and return `readings`.
    // return readings;  // This line is not needed since readings is passed by reference

    MQTTclient.publish("soil1/status", "reading sensor batch complete");
}