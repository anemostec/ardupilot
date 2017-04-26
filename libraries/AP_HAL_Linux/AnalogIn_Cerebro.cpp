#include "AnalogIn_Cerebro.h"

#ifdef CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_CEREBRO
using namespace Linux;

AnalogSource_Cerebro::AnalogSource_Cerebro(ArduinoBoard* m_arduino_board, uint8_t i_channel) :
    _i_channel(i_channel)
{
	_m_arduino = m_arduino_board;
	_ui_nb_samples=0;
	_f_sum = 0.0;

}

float AnalogSource_Cerebro::read_average() {

	if (_ui_nb_samples == 0)
		return 0.0f;

    return (float)(_f_sum/_ui_nb_samples);
}

float AnalogSource_Cerebro::voltage_average() {
	return read_average()*CEREBRO_ADC_QUANTUM;
}

float AnalogSource_Cerebro::voltage_latest() {
    float f_last_value = (float)(_m_arduino->get_last_adc_value(_i_channel));
    return CEREBRO_ADC_QUANTUM * f_last_value;
}

float AnalogSource_Cerebro::read_latest() {
	float f_last_value = (float)(_m_arduino->get_last_adc_value(_i_channel));
	_f_sum += f_last_value;
	_ui_nb_samples++;
    return (float)(_m_arduino->get_last_adc_value(_i_channel));
}

void AnalogSource_Cerebro::set_pin(uint8_t i_channel)
{
	_i_channel = i_channel;
}

void AnalogSource_Cerebro::set_stop_pin(uint8_t p)
{
	//TODO: This is a stub at the moment. Please complete when its use is found.
}

void AnalogSource_Cerebro::set_settle_time(uint16_t settle_time_ms)
{
	//TODO: This is a stub at the moment. Please complete when its use is found.
}

/* Beginning procedures for AnalogIn_Cerebro. */
AnalogIn_Cerebro::AnalogIn_Cerebro()
{}

void AnalogIn_Cerebro::init()
{
	_m_arduino = (ArduinoBoard*)hal.externalcpu;

}

AP_HAL::AnalogSource* AnalogIn_Cerebro::channel(int16_t i_ch) {
    return new AnalogSource_Cerebro(_m_arduino, i_ch);
}

float AnalogIn_Cerebro::board_voltage(void)
{
    return 5.0f;
}

#endif
