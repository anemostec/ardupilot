#pragma once

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Linux.h"
#include "ArduinoBoard.h"
#include "RCInput.h"

#define CEREBRO_RCINPUT_NB_CHANNELS 1

extern const AP_HAL::HAL& hal;

namespace Linux {

class RCInput_Cerebro : public RCInput
{
public:
    RCInput_Cerebro();
    ~RCInput_Cerebro();

    void init() override;
    void _timer_tick(void) override;

private:
    ArduinoBoard *_m_arduinoboard;

    uint16_t _tui_ppm_periods[CEREBRO_RCINPUT_NB_CHANNELS];

};
}