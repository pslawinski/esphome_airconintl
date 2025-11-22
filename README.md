# esphome_airconintl
Support for Aircon International and Hisense Mini-Splits via ESPHome

This ESPHome component provides full control and monitoring of Aircon International and Hisense mini-split air conditioning units via RS485 communication.

## About the Hisense AEH-W4A1 Controller

This component directly replaces the Hisense AEH-W4A1 (also labeled EH-W4A1) WiFi controller dongle. The AEH-W4A1 is the official smart controller for Hisense and Aircon International mini-split AC units.

The dongle plugs into a dedicated 4-pin port on the indoor unit's control board with the following pinout:
1. GND (black wire)
2. RS485 B-
3. RS485 A+
4. +5V (red wire)

It provides WiFi connectivity via 802.11 b/g/n (2.4 GHz only), cloud control through Hi-Smart Life or ConnectLife apps, and full remote monitoring/control. However, it requires internet connectivity and cloud services, which can be unreliable.

This ESPHome component plugs into the same 4-pin port, uses the same RS485 communication and 5V power, and speaks the identical Hisense protocol, but provides full local control through Home Assistant without any cloud dependency.

## Features

- **Climate Control**: Full temperature control in Fahrenheit or Celsius, with support for heating, cooling, fan-only, and dry modes.
- **Fan Modes**: Auto, low, medium, high, and quiet fan speeds.
- **Swing Control**: Horizontal and vertical swing options, including combined swing.
- **Presets**: Turbo (boost) and energy-saving modes.
- **Sensor Monitoring**: Real-time data from compressor frequency, outdoor/indoor temperatures, humidity, and more.
- **RS485 Communication**: Reliable serial communication with configurable RE/DE pins for half-duplex operation.

## Installation

Add the following to your ESPHome YAML configuration:

```yaml
external_components:
  - source: github://pslawinski/esphome_airconintl

climate:
  - platform: airconintl
    name: "Mini Split AC"
    temperature_unit: "F"  # or "C"
    re_pin: GPIO12
    de_pin: GPIO13
    # Optional sensors (see below)
```

## Configuration Options

- `temperature_unit`: Set to "F" for Fahrenheit or "C" for Celsius (default: "F").
- `re_pin`: GPIO pin for RS485 receive enable (optional, for RS485 control).
- `de_pin`: GPIO pin for RS485 transmit enable (optional, for RS485 control).

## Supported Sensors

Add optional sensors to monitor unit status:

- `compressor_frequency`: Current compressor frequency (Hz).
- `compressor_frequency_setting`: Target compressor frequency (Hz).
- `compressor_frequency_send`: Sent compressor frequency command (Hz).
- `outdoor_temperature`: Outdoor ambient temperature (°C).
- `outdoor_condenser_temperature`: Outdoor condenser temperature (°C).
- `compressor_exhaust_temperature`: Compressor exhaust temperature (°C).
- `target_exhaust_temperature`: Target exhaust temperature (°C).
- `indoor_pipe_temperature`: Indoor evaporator pipe temperature (°C).
- `humidity_setpoint`: Indoor humidity setpoint (%).
- `humidity_status`: Current indoor humidity (%).

Example sensor configuration:

```yaml
climate:
  - platform: airconintl
    # ... other options
    compressor_frequency:
      name: "Compressor Frequency"
    outdoor_temperature:
      name: "Outdoor Temperature"
    humidity_status:
      name: "Indoor Humidity"
```

## Hardware Setup

- **Microcontroller**: ESP8266 (e.g., ESP-12F) recommended.
- **Communication**: RS485 adapter board for UART to RS485 conversion.
- **Power**: 5V DC supply (adjustable converter recommended).
- **Connection**: Use the provided pinout to connect to the AC unit's control board.

The pinout of the connector is:
1) (Black wire) GND
2) B-
3) A+
4) (Red wire) +5V

## Credits

I referenced the work of several others to make this happen:
* https://community.home-assistant.io/t/working-on-integration-for-hisense-aeh-w4a1-module/146243
* https://github.com/deiger/AirCon/
* https://github.com/simoneluconi/hisense-aeh-w4a1

This thread in particular was quite useful:
https://github.com/deiger/AirCon/issues/1

I'm using an ESP-12F (8266) with a serial to 485 adapter board and a generic adjustable DC-DC converter. I'll assume that if you're even thinking of replicating my results you're well capable of finding the necessary components on Amazon.