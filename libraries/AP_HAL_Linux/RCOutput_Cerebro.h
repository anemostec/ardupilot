#pragma once

#include <AP_HAL/AP_HAL.h>
#include "ArduinoBoard.h"

#define CEREBRO_RCOUTPUT_NB_CHANNELS    6
#define CEREBRO_RCOUTPUT_PACKET_HEADER  0x57

#define CEREBRO_RCOUTPUT_PWM_MAX_RANGE  255

extern const AP_HAL::HAL& hal;

namespace Linux {
    /**
    * Cerebro uses an embedded Arduino in order to provide the PWM signals needed for all servos.
    * Communication between the NanoPi Neo Air and the Arduino is done via the UARTs modules (respectively UART1 and UART on both components).
    * RCOutput_Cerebro endorses the management of a such communication.
    */
    class RCOutput_Cerebro : public AP_HAL::RCOutput {
    private:
        
        ArduinoBoard *_m_arduino;

        constexpr static uint8_t tui_pwm_pins[6] = {3, 5, 6, 9, 10, 11};         /**< This array gives the corresponding pin of the Arduino board for a given PWM */

        uint16_t tui_timer_prescaler[CEREBRO_RCOUTPUT_NB_CHANNELS];

        uint8_t tui_duty_cycle_pending[CEREBRO_RCOUTPUT_NB_CHANNELS]; /**< Used in cork mode. Contains pending duty cycle values set by write() calls before the push() call. */

        bool tbool_duty_cycle_pending[CEREBRO_RCOUTPUT_NB_CHANNELS]; /**< Used in cork mode. It indicates if a PWM channel must be updated during the next push() call. */

        bool _corked; /**< Used in cork mode. Set to true by cork() and set to false by push(). It indicates to write() if it must write new PWM duty cycles in the
                        tui_duty_cycle_pending array or send it directly to the ArduinoBoard */

        /**
        * All PWM on an Arduino are not linked to the same clock before division (see http://playground.arduino.cc/Code/PwmFrequency )
        * This function returns the source frequency for a given PWM channel.
        * @param ch channel number
        * @return The requested PWM base frequency in Hertz
        */
        uint16_t _get_base_freq(uint8_t ch);
    public:
        /**
        *   Initialisation function.
        */
        void     init();
        void     set_freq(uint32_t chmask, uint16_t freq_hz);
        uint16_t get_freq(uint8_t ch);
        void     enable_ch(uint8_t ch);
        void     disable_ch(uint8_t ch);

        /**
        *   Sets the PWM high level for period_us microseconds.
        *   N.B.: if cork mode is engaged, these values are only transmitted to the ArduinoBoard class once push() is called.
        *   @param ch channel number
        *   @param period_us time the signal is high in microseconds.
        */
        void     write(uint8_t ch, uint16_t period_us);

        /**
        *   Returns how long the PWM signal is high within a period, in microseconds
        *   @param ch channel number
        *   @return the duty cycle in microseconds for a given PWM
        */
        uint16_t read(uint8_t ch);

        /**
        *   Returns an array containing all the periods the signal is high for all PWMs.
        *   @param *period_us array containig all duty cycle periods
        *   @param the duty cycle in microseconds for a given PWM
        */
        void     read(uint16_t* period_us, uint8_t len);

        /**
        *   Sets the m_corked boolean to true, making values set by write() only effective when the push() function is called.
        *   This procedure may be executed before calling write().
        */
        void     cork(void) override;

        /**
        *   Called when cork() has been called. It pushes all pending values set during previous write() calls to the ArduinoBoard object.
        */
        void     push(void) override;
    };
}
