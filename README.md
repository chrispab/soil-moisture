# soil-moisture

An ESP32-based soil moisture monitoring system that transmits sensor data over MQTT with advanced filtering and OTA update support.

## Features

- **Power Management**: Controls sensor power via GPIO to minimize probe corrosion and power consumption.
- **Advanced Data Processing**:
  - **Raw Readings**: Immediate analog values.
  - **Batch Processing**: Averages and Mode calculation from sample sets.
  - **Outlier Removal**: Automatically filters erroneous readings before averaging.
  - **Moving Averages**: Both integer and high-precision floating-point moving windows.
- **Connectivity**:
  - Automatic WiFi and MQTT reconnection logic.
  - LWT (Last Will and Testament) support for "Online/Offline" status.
- **OTA Updates**: Remote firmware updates via ArduinoOTA.
- **Visual Feedback**: Smooth PWM LED fading for heartbeat and status indicators.

## Hardware Configuration

| Component | Pin | Function |
|-----------|-----|----------|
| **Sensor VCC** | GPIO 23 | Power Supply Control |
| **Sensor Signal**| GPIO 36 | Analog Input (ADC) |
| **Sensor GND** | GND | Ground |
| **Green LED** | GPIO 12 | Heartbeat Indicator |
| **Blue LED** | GPIO 2 | Onboard Status LED |

## MQTT Interface

The device communicates using the `soil1/` topic prefix (configurable).

### Telemetry (Published)
- `soil1/version`: Reports current firmware version.
- `soil1/moisture_raw`: Single raw reading.
- `soil1/moisture_method1_average`: Average of a batch of readings.
- `soil1/moisture_method2_batch_mode`: The most frequent value in a batch.
- `soil1/moisture_method3_batch_average`: Average after outlier removal.
- `soil1/moisture_method5_batch_moving_average_float`: High-precision moving average (1 decimal place).
- `LWT_TOPIC`: Reports "Online" or "Offline".

### Commands (Subscribed)
- `soil1/read`: Sending a message to this topic triggers an immediate sensor reading.

## Software Dependencies

- **PubSubClient**: MQTT messaging.
- **ArduinoOTA**: Over-the-air updates.
- **LedFader**: Custom library for non-blocking PWM LED effects.
- **WiFiLib / MQTTLib**: Internal helper libraries for connection management.

## Development

This project is built using PlatformIO.

1. Configure your network settings in `src/config.h`.
2. Upload via USB or OTA using the `esp32doit-devkit-v1` environment.
