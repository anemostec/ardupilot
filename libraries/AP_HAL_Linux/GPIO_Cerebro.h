#pragma once

#include "AP_HAL_Linux.h"

#define SYSFS_GPIO_DIR "/sys/class/gpio"

/**
*   Anemos Cerebro is based on the NanoPi Neo-Air by FriendlyArm, powered by the Allwinner H3 SoC.
*   The whole mainboard provides 16 GPIO-capable pins.
*   Once assembled with its mezzanine board, Cerebro provides two GPIO-capable pins, used for the LED and resetting the Arduino.
*   
*   The Allwinner H3 supports up to 106 GPIOs, each divided into banks designated by a letter from A to L.
*   Each PIO pin is designated by P + bank letter + number.
*
*   The following table provides the GPIO pins available on Anemos Cerebro with their respective usages.
*
*   Pins | Usage
*   -----|----------------------
*   PL11 | LED control
*   PG11 | Arduino reset signal
*
*   When using /sys/class/gpio/ to configure these ports, a number is attributed to each GPIO pin, according to the following formula:
*   gpio_number = (position of the bank letter in the alphabet-1)*32+pin number
*
*   The following constants can be used to use the GPIOs with /sys/class/gpio. It already includes the required -1 offset.
*/

#define CEREBRO_SYS_CLASS_GPIO_BANK_OFFSET   32
#define CEREBRO_SYS_CLASS_GPIO_BANK_L        11
#define CEREBRO_SYS_CLASS_GPIO_BANK_G        6

#define CEREBRO_GPIO_LED            CEREBRO_SYS_CLASS_GPIO_BANK_L*CEREBRO_SYS_CLASS_GPIO_BANK_OFFSET+11
#define CEREBRO_GPIO_ARDUINO_RESET  CEREBRO_SYS_CLASS_GPIO_BANK_G*CEREBRO_SYS_CLASS_GPIO_BANK_OFFSET+11

#define LED_ON  1
#define LED_OFF 0

#define ARDUINO_RESET_ACTIVE    0
#define ARDUINO_RESET_INACTIVE  1

/**
*   Following constants define the memory addresses to configure directly the GPIOs.
*   Please refer to the Allwinner H3 datasheet for further information.
*/

#define CEREBRO_MEM_BASE_ADDR                   0x01C20800
#define CEREBRO_MEM_NB_BANKS                    7
#define CEREBRO_MEM_BANK_WHOLE_MAP_SIZE         0x24
#define CEREBRO_MEM_BANK_PORT_CONFIG_OFFSET     0x00
#define CEREBRO_MEM_BANK_PORT_CONFIG_NB_PORTS_PER_BYTE       2 /* < Each pin is configured over 3-bit + 1 separation bit. Hence 2 pins are supported per byte. */
#define CEREBRO_MEM_BANK_PORT_DATA_NB_PORTS_PER_BYTE         8 /* < DATA register is presented as a 32-bit wide register, with one bit per pin. LSb aligned (port 0 is bit 0).*/
#define CEREBRO_MEM_BANK_PORT_DATA_OFFSET       0x10
#define CEREBRO_MEM_BANK_MULTIDRIVING_OFFSET    0x14
#define CEREBRO_MEM_BANK_PULL_REGS_OFFSET       0x1C

#define CEREBRO_MEM_PORT_OUTPUT                 0x01
#define CEREBRO_MEM_PORT_INPUT                  0x00

/**
*   Represents a complete GPIO-bank structure (see p. TODO of the Allwinner H3 datasheet)
*/
struct gpio_banks{
    uint8_t *bank_base;
    uint8_t *bank_config;
    uint8_t *bank_data;
};


using namespace Linux;
/**
*   GPIO handling class for Anemos Cerebro.
*
*/
class GPIO_Cerebro : public AP_HAL::GPIO {
private:
    struct gpio_banks m_banks_addr[CEREBRO_MEM_NB_BANKS];

    /** Number of GPIO-compatible pins for each bank */
    constexpr static uint8_t tui_nb_pins_for_bank[CEREBRO_MEM_NB_BANKS] = {22, 19, 18, 16, 7, 14, 12};

    /**
    * Returns the GPIO-supporting bank number (A=0, C=1, D=2, and so on).
    *
    * @param _ui_pin pin number in /sys/class format
    * @return uint8_t Corresponding bank number.
    */
    uint8_t get_bank_number(uint8_t _ui_pin);

    /**
    * For a pin number given in the /sys/class format, returns the pin number in the related bank.
    * @param _ui_pin pin number in /sys/class format
    * @return uint8_t Corresponding pin number in the related bank.
    */
    uint8_t get_bank_pin_number(uint8_t _ui_pin);

public:

    /**
    * Constructor for GPIO_Cerebro().
    */
    GPIO_Cerebro();

    /**
    * Destructor for GPIO_Cerebro
    */
    ~GPIO_Cerebro();

    /**
    * Initialisation procedure setting all GPIO-compatible pin unused by other built-in units as GPIO.
    * @return void
    */
    void    init();

    /**
    * Sets pin as input or output
    *   @param uint8_t pin number
    *   @param uint8_t requested output value
    */
    void    pinMode(uint8_t pin, uint8_t output);

    /**
    *   analogPinToDigitalPin
    */
    int8_t  analogPinToDigitalPin(uint8_t pin);

    /**
    *   Returns the pin logical value.
    *   @param uint8_t pin number
    *   @return uint8_t pin logical value
    */
    uint8_t read(uint8_t pin);

    /**
    *   Sets the pin logical value (for outputs).
    * @param uint8_t pin number
    * @param uint8_t pin logical value
    */
    void    write(uint8_t pin, uint8_t value);

    /**
    *   Inverts the logical value for a given output pin.
    *   @param uint8_t pin number
    */
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
};