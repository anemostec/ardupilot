#include "ArduinoBoard.h"

#ifdef CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_CEREBRO

#include <AP_HAL/AP_HAL.h>
#include <string.h>

using namespace Linux;

ArduinoBoard::ArduinoBoard(AP_HAL::UARTDriver *m_uart_driver)
{
    _m_uart_driver = m_uart_driver;
}

ArduinoBoard::~ArduinoBoard()
{

}

void ArduinoBoard::init()
{

}

void ArduinoBoard::_timer_tick()
{
    uint32_t ui_read_buffer_size = _m_uart_driver->available();
    uint8_t *ui_packet = new uint8_t[ui_read_buffer_size];

    for(uint8_t icpt=0; icpt < ui_read_buffer_size; icpt++)
        ui_packet[icpt]=(uint8_t)_m_uart_driver->read();

    _parse_packet(ui_packet, ui_read_buffer_size);

    delete ui_packet;
}

/* Functions related to the ADCs */
uint16_t ArduinoBoard::get_last_adc_value(int i_no_channel)
{
    if(i_no_channel < ARDUINO_BOARD_NB_ADC)
        return _adc_latest_values[i_no_channel];

    return 0;
}
void ArduinoBoard::set_adc_settle_time(uint16_t settle_time_ms){
    // TODO: this is a stub. Please feed me when you've found for what I am used for.
}

/* PPM-related functions */
uint16_t ArduinoBoard::get_last_ppm_value(uint8_t i_no_channel)
{
    if(i_no_channel < ARDUINO_BOARD_NB_PPM)
        return _ppm_latest_values[i_no_channel];

    return 0;
}

/* PWM functions */
void ArduinoBoard::set_pwm_freq(int i_channel, uint16_t ui_freq)
{
    uint8_t ui_payload[3];

    ui_payload[0] = i_channel;
    ui_payload[1] = ui_freq>>8;
    ui_payload[2] = ui_freq;
    _send_packet(ARDUINO_BOARD_HEADER_PWM_OUT_SET_FREQUENCY, ui_payload, 3);
}

void ArduinoBoard::disable_pwm_channel(uint8_t i_channel)
{
    _send_packet(ARDUINO_BOARD_HEADER_PWM_OUT_DISABLE_CHANNEL, &i_channel, 1);
}

void ArduinoBoard::enable_pwm_channel(uint8_t i_channel)
{
    _send_packet(ARDUINO_BOARD_HEADER_PWM_OUT_ENABLE_CHANNEL, &i_channel, 1);
}

int ArduinoBoard::set_pwm_duty_cycle(uint8_t ch, uint8_t tick)
{

    uint8_t ui_payload[2];

    if(ch > ARDUINO_BOARD_NB_PWM-1)
        return -1;                     // TODO manage errors.

    ui_payload[0] = ch;
    ui_payload[1] = tick;

    _pwm_duty_cycle_values[ch] = tick;

    _send_packet(ARDUINO_BOARD_HEADER_PWM_OUT_SET_DUTYCYCLE, ui_payload, 2);

    return 0;

}
uint8_t ArduinoBoard::get_pwm_duty_cycle(uint8_t ch)
{
    if(ch < ARDUINO_BOARD_NB_PWM)
        return _pwm_duty_cycle_values[ch];

    return 255;
}

void ArduinoBoard::_send_packet(uint8_t ui_header, uint8_t *ui_payload, uint8_t ui_size)
{
    uint8_t *ui_packet = new uint8_t[ui_size+2];

    // setting packet header and size at the front.
    ui_packet[ARDUINO_BOARD_PROTOCOL_HEADER_POS] = ui_header;
    ui_packet[ARDUINO_BOARD_PROTOCOL_PACKET_SIZE_POS] = ui_size;

    // copying data after the header
    memcpy(ui_packet+2, ui_payload, ui_size);

    // we send the packet in the UART buffer.
    _m_uart_driver->write(ui_packet, ui_size+2);

    delete ui_packet;

}
int ArduinoBoard::_parse_packet(uint8_t *ui_packet, uint32_t i_size)
{
    if(i_size < 2)
        return -1;  // TODO set proper constants

    int i_packet_start = 0;

    // A RX UART buffer may contain more than one packet. This loop parses the whole buffer and each packet in it.
    while(i_size > 2) {
        uint8_t ui_header = ui_packet[i_packet_start+ARDUINO_BOARD_PROTOCOL_HEADER_POS];
        uint8_t ui_size = ui_packet[i_packet_start+ARDUINO_BOARD_PROTOCOL_PACKET_SIZE_POS];

        switch(ui_header) {
            case ARDUINO_BOARD_HEADER_ADC_IN_NEW_SAMPLES:
                if(ui_size < sizeof(uint16_t)*ARDUINO_BOARD_NB_ADC)
                    return -1;  // TODO set proper error

                _update_adc_values_from_packet(ui_packet+i_packet_start+2, i_size-2);
                break;

            case ARDUINO_BOARD_HEADER_PPM_IN_NEW_SAMPLES:
                if(ui_size < sizeof(uint16_t)*ARDUINO_BOARD_NB_ADC)
                    return -1;  // TODO set proper error

                _update_ppm_values_from_packet(ui_packet+i_packet_start+2, i_size-2);
                break;

            default:
                return -1; // TODO set proper error constants

         }

         i_size -= (ui_size+2);
    }

    return 0;
}

void ArduinoBoard::_update_adc_values_from_packet(uint8_t *ui_packet, uint8_t ui_packet_size) {

    for(uint8_t icpt=0; icpt < ui_packet_size; icpt+=2) {
        _adc_latest_values[icpt] = ui_packet[icpt];
        _adc_latest_values[icpt] <<= 8;
        _adc_latest_values[icpt] += ui_packet[icpt+1];
    }
}

void ArduinoBoard::_update_ppm_values_from_packet(uint8_t *ui_packet, uint8_t ui_packet_size) {

    for(uint8_t icpt=0; icpt < ui_packet_size; icpt+=2) {
        _ppm_latest_values[icpt] = ui_packet[icpt];
        _ppm_latest_values[icpt] <<= 8;
        _ppm_latest_values[icpt] += ui_packet[icpt+1];
    }
}

#endif
