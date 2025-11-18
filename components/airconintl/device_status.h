#pragma once

#pragma pack(push, 1)
typedef struct _Device_Status
{
    uint8_t header[16];

    uint8_t wind_status; // air volume
    uint8_t sleep_status;

    uint8_t direction_status : 2; // wind direction
    uint8_t run_status : 2;
    uint8_t mode_status : 4;

    // 4
    uint8_t indoor_temperature_setting;
    uint8_t indoor_temperature_status;
    uint8_t indoor_pipe_temperature;
    // 7
    int8_t indoor_humidity_setting;
    int8_t indoor_humidity_status;

    uint8_t somatosensory_temperature; // sensible temperature
    // 10
    uint8_t somatosensory_compensation_ctrl : 3;
    uint8_t somatosensory_compensation : 5;
    // 11
    uint8_t temperature_Fahrenheit : 3; // fahrenheit display

    uint8_t temperature_compensation : 5;

    // 12
    uint8_t timer;

    // 13
    uint8_t hour;
    // 14
    uint8_t minute;
    // 15
    uint8_t poweron_hour;
    // 16
    uint8_t poweron_minute;
    // 17
    uint8_t poweroff_hour;
    // 18
    uint8_t poweroff_minute;
    // 19
    uint8_t wind_door : 4;
    uint8_t drying : 4;
    // 20
    uint8_t dual_frequency : 1;
    uint8_t efficient : 1;
    uint8_t low_electricity : 1; // save electricity
    uint8_t low_power : 1;       // energy saving
    uint8_t heat : 1;            // heating air
    uint8_t nature : 1;          // natural wind
    uint8_t left_right : 1;      // horizontal swing
    uint8_t up_down : 1;         // vertical swing

    // 21
    uint8_t smoke : 1; // smoke removal
    uint8_t voice : 1;
    uint8_t mute : 1;
    uint8_t smart_eye : 1;
    uint8_t outdoor_clear : 1; // outdoor cleaning
    uint8_t indoor_clear : 1;  // indoor cleaning
    uint8_t swap : 1;          // Change the wind
    uint8_t dew : 1;           // fresh

    // 22
    uint8_t indoor_electric : 1;
    uint8_t right_wind : 1;
    uint8_t left_wind : 1;
    uint8_t filter_reset : 1;
    uint8_t indoor_led : 1;
    uint8_t indicate_led : 1;
    uint8_t display_led : 1;
    uint8_t back_led : 1;

    // 23
    uint8_t indoor_eeprom : 1; // eeprom
    uint8_t sample : 1;
    uint8_t rev23 : 4;
    uint8_t time_lapse : 1;
    uint8_t auto_check : 1; // self-test

    // 24
    uint8_t indoor_outdoor_communication : 1;
    uint8_t indoor_zero_voltage : 1;
    uint8_t indoor_bars : 1;
    uint8_t indoor_machine_run : 1;
    uint8_t indoor_water_pump : 1;
    uint8_t indoor_humidity_sensor : 1;
    uint8_t indoor_temperature_pipe_sensor : 1;
    uint8_t indoor_temperature_sensor : 1;

    // 25
    uint8_t rev25 : 3;
    uint8_t eeprom_communication : 1;
    uint8_t electric_communication : 1;
    uint8_t keypad_communication : 1;
    uint8_t display_communication : 1;

    // 26
    uint8_t compressor_frequency;
    // 27
    uint8_t compressor_frequency_setting;
    // 28
    uint8_t compressor_frequency_send;
    // 29
    int8_t outdoor_temperature;
    // 30
    int8_t outdoor_condenser_temperature;
    // 31
    int8_t compressor_exhaust_temperature;
    // 32
    int8_t target_exhaust_temperature;
    // 33
    uint8_t expand_threshold;
    // 34
    uint8_t UAB_HIGH;
    // 35
    uint8_t UAB_LOW;
    // 36
    uint8_t UBC_HIGH;
    // 37
    uint8_t UBC_LOW;
    // 38
    uint8_t UCA_HIGH;
    // 39
    uint8_t UCA_LOW;
    // 40
    uint8_t IAB;
    // 41
    uint8_t IBC;
    // 42
    uint8_t ICA;
    // 43
    uint8_t generatrix_voltage_high;
    // 44
    uint8_t generatrix_voltage_low;
    // 45
    uint8_t IUV;
    // 46
    uint8_t wind_machine : 3;
    uint8_t outdoor_machine : 1;
    uint8_t four_way : 1;
    uint8_t rev46 : 3;

    // 47
    uint8_t rev47;
    // 48
    uint8_t rev48;
    // 49
    uint8_t rev49;
    // 50
    uint8_t rev50;
    // 51
    uint8_t rev51;
    // 52
    uint8_t rev52;
    // 53
    uint8_t rev53;
    // 54
    uint8_t rev54;
    // 55
    uint8_t rev55;
    // 56
    uint8_t rev56;

    uint8_t extra[6];
    uint16_t chk_sum;
    uint8_t foooter[2];
} Device_Status;
#pragma pack(pop)