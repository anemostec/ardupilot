#pragma once

#include <AP_HAL/ExternalCPU.h>
#include "AP_HAL_Linux.h"

/**
*   The NanoPi-Neo Air-to-Arduino UART protocol carries information for the ADC, PWMs and PPM signals.
*   Each packet starts with a packet type header, the size of the packet in bytes, and the data.
*
*   WARNING: for over data over 8 bits, the MSB must be transferred first.
*
*   The following terminology is used to describe the way the data is transmitted:
*   IN: from the Arduino to the NanoPi Neo Air
*   OUT: from the NanoPi Neo Air to the Arduino
*
*   Packet type headers are defined below:
*/

#define ARDUINO_BOARD_HEADER_ADC_IN_NEW_SAMPLES      0x01    /**< For n ADCs, followed by 2*n bytes. */
#define ARDUINO_BOARD_HEADER_ADC_OUT_SET_SETTLE_TIME 0x02    /* TODO */

#define ARDUINO_BOARD_HEADER_PWM_OUT_SET_FREQUENCY   0x10   /**< For n PWMs, followed by 2*n bytes. */
#define ARDUINO_BOARD_HEADER_PWM_OUT_SET_DUTYCYCLE   0x11   /**< For n PWMs, followed by n bytes. */
#define ARDUINO_BOARD_HEADER_PWM_OUT_DISABLE_CHANNEL 0x12   /**< Payload contains only the channel number */
#define ARDUINO_BOARD_HEADER_PWM_OUT_ENABLE_CHANNEL  0x13   /**< Payload contains only the channel number */

#define ARDUINO_BOARD_HEADER_PPM_IN_NEW_SAMPLES      0x20   /**< PPM values. Followed by 2*n bytes for n channels */

/**
*   Cerebro modules descriptions
*/
#define ARDUINO_BOARD_NB_ADC                        4       /**< Number of ADCs available aboard Cerebro */
#define ARDUINO_BOARD_NB_PPM                        1       /**< Number of PPM channels on Cerebro */
#define ARDUINO_BOARD_NB_PWM                        6       /**< Number of PWMs available aboard Cerebro */

/**
* Protocol specific byte positions
*/
#define ARDUINO_BOARD_PROTOCOL_HEADER_POS           0
#define ARDUINO_BOARD_PROTOCOL_PACKET_SIZE_POS      1


namespace Linux {
    class ArduinoBoard: public AP_HAL::ExternalCPU {
    public:
        //ArduinoBoard(const char *device_path);
        ArduinoBoard(AP_HAL::UARTDriver *m_uart_driver);
        ~ArduinoBoard();

        void init();

        /**
        *   This function parses any incoming packet containing ADC data.
        *   This function may be executed once all UARTs have been updated, which relates to the uart or timer thread according to HAL_LINUX_UARTS_ON_TIMER_THREAD.
        *   @see Scheduler.cpp
        */
        void _timer_tick();

        /**
        *   Returns the latest ADC value sent by the Arduino.
        *   @param i_no_channel ADC number
        *   @return the latest ADC value sent by the Arduino.
        */
        uint16_t get_last_adc_value(int i_no_channel);

        // TODO
        void set_adc_settle_time(uint16_t settle_time_ms);

        uint16_t get_last_ppm_value(uint8_t i_no_channel);
        
        /**
        *   Sets the i_channel-th PWM prescaler to the ui_freq value.
        *   @param i_channel PWM channel.
        *   @param ui_freq New PWM prescaler value.
        */
        void set_pwm_freq(int i_channel, uint16_t ui_freq);

        /**
        *   Disable i_channel-th PWM.
        *   @param i_channel PWM channel.
        */
        void disable_pwm_channel(uint8_t i_channel);

        /**
        *   Enable the PWM no i_channel.
        *   @param i_channel PWM channel.
        */
        void enable_pwm_channel(uint8_t ch);

        /**
        *   Sets ch-th PWM to a duty cycle of tick ticks.
        *   @param ch channel number
        *   @param tick duty cycle in number of ticks (between 0 and 255)
        */
        int set_pwm_duty_cycle(uint8_t ch, uint8_t tick);

        /**
        *   Returns the duty cycle (between 0 and 255) for the PWM no ch.
        *   @param ch channel number
        *   @return Value of the PWM duty cycle.
        */
        uint8_t get_pwm_duty_cycle(uint8_t ch);
    private:

        AP_HAL::UARTDriver *_m_uart_driver; /**< UARTDriver for the UART connected to the Arduino */

        /**
        *   Puts the packet into the 
        *   @param ch channel number
        *   @return Value of the PWM duty cycle.
        */
        void _send_packet(uint8_t ui_header, uint8_t *ui_payload, uint8_t ui_size);

        /**
        *   Called by _timer_tick() only. Parse the received UART RX buffer and updates the corresponding internal attributes accordingly to the headers of the messages.
        *   @param *ui_packet array of bytes containing the whole UART RX buffer.
        *   @param i_size RX buffer size
        *   @return 0 if everything went fine.
        */
        int _parse_packet(uint8_t *ui_packet, uint32_t i_size);

        /**
        *   Called by _parse_packet() only when an ARDUINO_BOARD_HEADER_ADC_IN_NEW_SAMPLES message is received.
        *   It updates _adc_latest_values.
        *   @param ui_packet incoming ADC NEW SAMPLES packet
        *   @param ui_packet_size ADC NEW SAMPLES packet size
        */
        void _update_adc_values_from_packet(uint8_t *ui_packet, uint8_t ui_packet_size);

        /**
        *   Called by _parse_packet() only when an ARDUINO_BOARD_HEADER_PPM_IN_NEW_SAMPLES message is received.
        *   It updates _ppm_latest_values.
        *   @param ui_packet incoming PPM NEW SAMPLES packet
        *   @param ui_packet_size PPM NEW SAMPLES packet size
        */
        void _update_ppm_values_from_packet(uint8_t *ui_packet, uint8_t ui_packet_size);
        

        uint16_t _adc_latest_values[ARDUINO_BOARD_NB_ADC]; /**< Latest sampled values by the ADCs. */

        uint16_t _ppm_latest_values[ARDUINO_BOARD_NB_PPM]; /**< Latest sampled values from the PPM. */


        uint8_t _pwm_duty_cycle_values[ARDUINO_BOARD_NB_PWM]; /**< values for PWMs duty cyles */
        uint8_t _pwm_frequency_values[ARDUINO_BOARD_NB_PWM]; /**< PWMs frequencies */

    };
}