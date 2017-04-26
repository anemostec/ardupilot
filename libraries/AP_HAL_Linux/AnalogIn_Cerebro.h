#pragma once

#include "AP_HAL_Linux.h"
#include "UARTDriver.h"
#include "ArduinoBoard.h"

#define CEREBRO_ADC_NB_CHANNELS 4

#define CEREBRO_ADC_QUANTUM 0.049f /**< Arduino Pro Mini ADC quantum. Please see https://www.arduino.cc/en/Reference/analogRead for further information */

extern const AP_HAL::HAL& hal;

namespace Linux {
    class AnalogSource_Cerebro : public AP_HAL::AnalogSource {
    public:
        AnalogSource_Cerebro(ArduinoBoard* m_arduino_board, uint8_t i_channel);

        /**
        *   Returns the average value.
        *   @return average value in quantums.
        */
        float read_average();

        /**
        *   Returns the latest sampled value.
        *   @return average value in quantums.
        */
        float read_latest();

        /**
        *   Sets the channel number.
        *   @param i_channel the ADC channel number.
        */
        void set_pin(uint8_t i_channel);
        void set_stop_pin(uint8_t p);
        void set_settle_time(uint16_t settle_time_ms);

        /**
        *   Returns the average value in Volts.
        *   @return average value in Volts.
        */
        float voltage_average();

        /**
        *   Returns the latest sample value in Volts.
        *   @return latest sample in Volts.
        */
        float voltage_latest();
        float voltage_average_ratiometric() { return voltage_average(); }
    private:
        uint8_t _i_channel;
        float _f_sum;           /*< Sum of all samples (amplitude only, not in Volts) */
        unsigned int _ui_nb_samples;      /*< Number of ADC samples */
        ArduinoBoard *_m_arduino;
    };

    class AnalogIn_Cerebro : public AP_HAL::AnalogIn {
    private:
        ArduinoBoard *_m_arduino;
    public:
        AnalogIn_Cerebro();
        void init();
        AP_HAL::AnalogSource* channel(int16_t i_ch);
        float board_voltage(void);
    };
}