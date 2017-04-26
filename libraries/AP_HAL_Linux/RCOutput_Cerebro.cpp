
#include "RCOutput_Cerebro.h"

#ifdef CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_CEREBRO

using namespace Linux;

constexpr uint8_t RCOutput_Cerebro::tui_pwm_pins[6];         /**< This array gives the corresponding pin of the Arduino board for a given PWM */


void RCOutput_Cerebro::init() {

	_m_arduino = (ArduinoBoard*)hal.externalcpu;

	// Obsolete. Was designed in the case we wanted to use the UART directly.
	/*struct termios p_termios;

	int i_err;

	m_uart_fd = open(tb_output_uart, O_RDWR | O_NONBLOCK | O_CLOEXEC);

	if(m_uart_fd == -1) {
		AP_HAL::panic("[RCOutput_Cerebro] Fatal error: can not open %s", tb_output_uart);
		exit(-1);
	}

	i_err = tcgetattr(m_uart_fd, p_termios);

	if(i_err == -1) {
		AP_HAL::panic("[RCOutput_Cerebro] Fatal error: can not read termios from UART1");
		exit(-1);
	}

	cfsetispeed(p_termios, B230400);

	cfsetospeed(p_termios, B230400);*/

}

uint16_t RCOutput_Cerebro::_get_base_freq(uint8_t ch)
{

	uint8_t ui_pin_no = tui_pwm_pins[ch]; /* From the point-of-view of ArduPilot, Gives the corresponding ch-th PWM pin on the Arduino board */

	switch(ui_pin_no) {		/* Considers the Arduino pin number. See http://playground.arduino.cc/Code/PwmFrequency to get the relation between the pins and the frequency */
		case 3:
		case 9:
		case 10:
		case 11:
			return 31250;
			break;

		case 5:
		case 6:
			return 62500;
			break;

		default:
			return 0;
	}
}

void RCOutput_Cerebro::set_freq(uint32_t chmask, uint16_t freq_hz) {
	uint32_t u_mask = 0x01;

	// Warning! Some PWMs are calibrated on a base frequency of 31.25kHz, while others are based on a 62.5kHz clock.
	// _get_base_freq must be called to get the corresponding frequency.
	for(int icpt=0; icpt < CEREBRO_RCOUTPUT_NB_CHANNELS; icpt++, u_mask <<= 1) {
		if((chmask & u_mask) > 0) {
			uint16_t ui_base_freq = _get_base_freq(icpt);

			uint16_t ui_divider;

			if(freq_hz > 0)
				ui_divider = ui_base_freq / freq_hz;
			else
				ui_divider = 1024; // divide by the maximum

			_m_arduino->set_pwm_freq(icpt, ui_divider);
			tui_timer_prescaler[icpt] = ui_divider;
		}
	}
}

uint16_t RCOutput_Cerebro::get_freq(uint8_t ch)
{
    return _get_base_freq(ch)*tui_timer_prescaler[ch];
}

void RCOutput_Cerebro::enable_ch(uint8_t ch)
{
	_m_arduino->enable_pwm_channel(ch);
}

void RCOutput_Cerebro::disable_ch(uint8_t ch)
{
	_m_arduino->disable_pwm_channel(ch);
}

void RCOutput_Cerebro::write(uint8_t ch, uint16_t period_us)
{
	float f_period_us = 1e6/get_freq(ch);
	uint8_t ui_duty_cycle = (uint8_t)((f_period_us/period_us)*CEREBRO_RCOUTPUT_PWM_MAX_RANGE); // The frequency is in Hz,
																								// the duty cycle ratio is between 0 and CEREBRO_RCOUTPUT_PWM_MAX_RANGE.

	if(_corked) {
		tui_duty_cycle_pending[ch] = ui_duty_cycle;		// If in cork mode, we enqueue the new duty cycle for the next push().
		tbool_duty_cycle_pending[ch] = true;
	}
	else
		_m_arduino->set_pwm_duty_cycle(ch, ui_duty_cycle);	// Otherwise we send it directly to the ArduinoBoard object.
}

uint16_t RCOutput_Cerebro::read(uint8_t ch) {
    uint16_t ui_duty_cycle = _m_arduino->get_pwm_duty_cycle(ch);

    float f_period_us = 1e6/get_freq(ch);

    uint16_t ui_duty_cycle_us = (uint16_t)(f_period_us*ui_duty_cycle/CEREBRO_RCOUTPUT_PWM_MAX_RANGE);

    return ui_duty_cycle_us;
}

void RCOutput_Cerebro::read(uint16_t* period_us, uint8_t len)
{
	if (len > CEREBRO_RCOUTPUT_NB_CHANNELS)
		len = CEREBRO_RCOUTPUT_NB_CHANNELS;

	for(int icpt=0; icpt < CEREBRO_RCOUTPUT_NB_CHANNELS; icpt++)
		period_us[icpt] = read(icpt);
}

void RCOutput_Cerebro::cork(void)
{
	_corked=true;
}

void RCOutput_Cerebro::push(void)
{
	_corked=false;
	for(int icpt = 0; icpt < CEREBRO_RCOUTPUT_NB_CHANNELS; icpt++) {
		if(tbool_duty_cycle_pending[icpt])
			_m_arduino->set_pwm_duty_cycle(icpt, tui_duty_cycle_pending[icpt]);
	}

}

#endif