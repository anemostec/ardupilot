#include "RCInput_Cerebro.h"

#include <AP_HAL/AP_HAL.h>

using namespace Linux;

RCInput_Cerebro::RCInput_Cerebro()
{
}

RCInput_Cerebro::~RCInput_Cerebro()
{
}

void RCInput_Cerebro::init()
{
    _m_arduinoboard = (ArduinoBoard*)hal.externalcpu;
}

void RCInput_Cerebro::_timer_tick()
{
    for(int icpt=0; icpt < CEREBRO_RCINPUT_NB_CHANNELS; icpt++)
        _tui_ppm_periods[icpt] = _m_arduinoboard->get_last_ppm_value(icpt);

    _update_periods(_tui_ppm_periods, CEREBRO_RCINPUT_NB_CHANNELS);
}
