# Soil Moisture — Developer Documentation

This document provides a concise developer-oriented reference for the project: wiring/pinout, key modules, and important functions in the codebase.

**Pinout & Hardware**
- **Sensor VCC control**: `SENSOR_POWERSUPPLY_PIN` (see [src/main.cpp](src/main.cpp)) — toggled to reduce probe corrosion and power use.
- **Sensor analog input**: `SENSOR_PIN` (see [src/main.cpp](src/main.cpp)).
- **Green heartbeat LED**: `GREEN_LED_PIN` (see [src/main.cpp](src/main.cpp)).
- **Onboard/blue LED**: `ONBOARD_LED_PIN` (see [src/main.cpp](src/main.cpp)).

See `src/config.h` for configurable constants (WiFi, MQTT topics, timing, and pin definitions).

**Key Source Files**
- [src/main.cpp](src/main.cpp): Main application logic, sensor sampling and MQTT publication.
- lib/LedFader/src/LedFader.cpp/.h: Non-blocking PWM fading for LEDs.
- lib/MQTTLib and lib/WiFiLib: Connection helpers.

**Important functions (quick reference)**

- `readAnalogueSensorN(numReadings, msBetweenReadings)` — in [src/main.cpp](src/main.cpp)
  - Purpose: Read the analog sensor multiple times, average the readings and return a 16-bit value. The sensor power is toggled elsewhere before/after sampling.
  - Usage: Called as `readAnalogueSensorN(20, 10)` to sample 20 times with 10ms between reads.
  - Notes: Includes an initial `analogRead()` to stabilize the ADC and uses a 32-bit accumulator to avoid overflow.

- `readAndTxSensorIfDue()` — in [src/main.cpp](src/main.cpp)
  - Purpose: Non-blocking periodic telemetry task. When the telemetry period has elapsed it powers the sensor, samples, computes derived values, and publishes MQTT messages.

- `processMovingAverageValue(sensorValue)` — in [src/main.cpp](src/main.cpp)
  - Purpose: Maintains a floating-point circular window (defined by `MOVING_AVERAGE_WINDOW_SIZE`) and returns a rounded moving average (1 decimal place).

- `getAverageOfReadings()` — in [src/main.cpp](src/main.cpp)
  - Purpose: Helper that computes the arithmetic mean of a float array.

- `MQTTpublishValue(topic, value)` overloads — in [src/main.cpp](src/main.cpp)
  - Purpose: Publish integer and float values to MQTT topics; floats are formatted to 1 decimal place.

**Development notes & recommendations**
- Keep the sensor powered only while sampling to reduce galvanic corrosion on probes.
- Use `readAnalogueSensorN()` instead of single `analogRead()` to reduce noise and obtain more stable values.
- If you need higher ADC sensitivity/tuning on ESP32, consider using `analogSetCycles()`/`analogSetSamples()` (comments exist in `src/main.cpp`).

**Building & Uploading**
- This project uses PlatformIO. From the project root:

```bash
# Build
pio run

# Upload (via configured environment)
pio run --target upload
```

**Where to look next**
- Sensor configuration and MQTT topic names: [src/config.h](src/config.h)
- Main sampling and publish logic: [src/main.cpp](src/main.cpp)

If you want, I can also generate Doxygen config, or expand this document with sequence diagrams and example MQTT payloads.
