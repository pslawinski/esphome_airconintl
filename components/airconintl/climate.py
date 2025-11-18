from esphome.components import climate, sensor, uart
import esphome.config_validation as cv
import esphome.codegen as cg
import esphome.pins as pins
from esphome.const import (
    CONF_ID,
    CONF_OUTDOOR_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_FREQUENCY,
    ICON_THERMOMETER,
    ICON_WATER_PERCENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_HERTZ,
)

CODEOWNERS = ["@pslawinski"]
DEPENDENCIES = ["climate"]
AUTO_LOAD = ["sensor"]
CONF_COMPRESSOR_FREQUENCY = "compressor_frequency"
CONF_COMPRESSOR_FREQUENCY_SETTING = "compressor_frequency_setting"
CONF_COMPRESSOR_FREQUENCY_SEND = "compressor_frequency_send"
CONF_OUTDOOR_CONDENSER_TEMPERATURE = "outdoor_condenser_temperature"
CONF_COMPRESSOR_EXHAUST_TEMPERATURE = "compressor_exhaust_temperature"
CONF_TARGET_EXHAUST_TEMPERATURE = "target_exhaust_temperature"
CONF_INDOOR_PIPE_TEMPERATURE = "indoor_pipe_temperature"
CONF_HUMIDITY_SETPOINT = "humidity_setpoint"
CONF_HUMIDITY_STATUS = "humidity_status"
CONF_TEMPERATURE_UNIT = "temperature_unit"
CONF_RE_PIN = "re_pin"
CONF_DE_PIN = "de_pin"

airconintl_ns = cg.esphome_ns.namespace("airconintl")
AirconClimate = airconintl_ns.class_("AirconClimate", cg.PollingComponent, climate.Climate, uart.UARTDevice)

CONFIG_SCHEMA = cv.All(
    climate.climate_schema(AirconClimate)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(AirconClimate),
            cv.Optional(CONF_TEMPERATURE_UNIT, default="F"): cv.one_of("C", "F", upper=True),
            cv.Optional(CONF_COMPRESSOR_FREQUENCY): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COMPRESSOR_FREQUENCY_SETTING): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COMPRESSOR_FREQUENCY_SEND): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_OUTDOOR_CONDENSER_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COMPRESSOR_EXHAUST_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TARGET_EXHAUST_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_INDOOR_PIPE_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY_SETPOINT): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon=ICON_WATER_PERCENT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY_STATUS): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon=ICON_WATER_PERCENT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_RE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DE_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("10s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_temperature_unit(config[CONF_TEMPERATURE_UNIT]))

    if CONF_COMPRESSOR_FREQUENCY in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_FREQUENCY])
        cg.add(var.set_compressor_frequency_sensor(sens))
    if CONF_COMPRESSOR_FREQUENCY_SETTING in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_FREQUENCY_SETTING])
        cg.add(var.set_compressor_frequency_setting_sensor(sens))
    if CONF_COMPRESSOR_FREQUENCY_SEND in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_FREQUENCY_SEND])
        cg.add(var.set_compressor_frequency_send_sensor(sens))
    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(var.set_outdoor_temperature_sensor(sens))
    if CONF_OUTDOOR_CONDENSER_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_CONDENSER_TEMPERATURE])
        cg.add(var.set_outdoor_condenser_temperature_sensor(sens))
    if CONF_COMPRESSOR_EXHAUST_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_EXHAUST_TEMPERATURE])
        cg.add(var.set_compressor_exhaust_temperature_sensor(sens))
    if CONF_TARGET_EXHAUST_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TARGET_EXHAUST_TEMPERATURE])
        cg.add(var.set_target_exhaust_temperature_sensor(sens))
    if CONF_INDOOR_PIPE_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_INDOOR_PIPE_TEMPERATURE])
        cg.add(var.set_indoor_pipe_temperature_sensor(sens))
    if CONF_HUMIDITY_STATUS in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY_STATUS])
        cg.add(var.set_indoor_humidity_status_sensor(sens))
    if CONF_HUMIDITY_SETPOINT in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY_SETPOINT])
        cg.add(var.set_indoor_humidity_setting_sensor(sens))
    if CONF_RE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RE_PIN])
        cg.add(var.set_re_pin(pin))
    if CONF_DE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_DE_PIN])
        cg.add(var.set_de_pin(pin))
