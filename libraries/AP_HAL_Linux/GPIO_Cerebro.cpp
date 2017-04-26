#include <AP_HAL/AP_HAL.h>

#ifdef CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_CEREBRO

#include "GPIO.h"
#include "GPIO_Cerebro.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

using namespace Linux;

constexpr uint8_t GPIO_Cerebro::tui_nb_pins_for_bank[CEREBRO_MEM_NB_BANKS]; // Re-declaration in CPP file is needed.

GPIO_Cerebro::GPIO_Cerebro()
{}

GPIO_Cerebro::~GPIO_Cerebro()
{
    for(int icpt = 0; icpt < CEREBRO_MEM_NB_BANKS; icpt++) {
        munmap(m_banks_addr[icpt].bank_base,
                    CEREBRO_MEM_BANK_WHOLE_MAP_SIZE*sizeof(uint8_t)
                );
    }
}

void GPIO_Cerebro::init()
{
    int i_export_file_desc, i_memmap_desc;

    /* The following code makes the GPIO pins directly available through the file system. */

    if ((i_export_file_desc = open("/sys/class/gpio/export", O_WRONLY|O_CLOEXEC)) < 0) {
        AP_HAL::panic("[GPIO_Cerebro] Fatal error: cannot open /sys/class/gpio/export");
        exit(-1);
    }

    dprintf(i_export_file_desc, "%d\n", CEREBRO_GPIO_LED);
    dprintf(i_export_file_desc, "%d\n", CEREBRO_GPIO_ARDUINO_RESET);

    close(i_export_file_desc);

    if(i_memmap_desc = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC) < 0) {
        AP_HAL::panic("[GPIO_Cerebro] Fatal error: access to /dev/mem is denied.");
        exit(-1);
    }

    for(int icpt = 0; icpt < CEREBRO_MEM_NB_BANKS; icpt++) {
        m_banks_addr[icpt].bank_base = (uint8_t *)mmap(
                    0,
                    CEREBRO_MEM_BANK_WHOLE_MAP_SIZE*sizeof(uint8_t), PROT_READ|PROT_WRITE,
                    MAP_SHARED,
                    i_memmap_desc,
                    CEREBRO_MEM_BASE_ADDR+icpt*CEREBRO_MEM_BANK_WHOLE_MAP_SIZE
                );

        m_banks_addr[icpt].bank_config = m_banks_addr[icpt].bank_base+CEREBRO_MEM_BANK_PORT_CONFIG_OFFSET;
        m_banks_addr[icpt].bank_data = m_banks_addr[icpt].bank_base+CEREBRO_MEM_BANK_PORT_DATA_OFFSET;
    }

    close(i_memmap_desc);


}

uint8_t GPIO_Cerebro::get_bank_number(uint8_t _ui_pin)
{
    uint8_t ui_sys_bank_nb = _ui_pin/32;

    switch (ui_sys_bank_nb) {
        case 0:
            return 0;
        case 2:
            return 1;
        case 3:
            return 2;
        case 4:
            return 3;
        case 5:
            return 4;
        case 6:
            return 5;
        case 12:
            return 6;
        default:
            return 255;
    };
}

uint8_t GPIO_Cerebro::get_bank_pin_number(uint8_t _ui_pin)
{
    return _ui_pin%32;
}

void GPIO_Cerebro::pinMode(uint8_t pin, uint8_t output)
{
    // Selecting the corresponding bank.
    int i_bank = get_bank_number(pin);

    if(i_bank>CEREBRO_MEM_NB_BANKS-1)
        return; 

    // Selecting the corresponding pin number into the given bank
    int i_bank_pin = get_bank_pin_number(pin);

    uint8_t *bank_config_reg
        = m_banks_addr[i_bank].bank_config+(i_bank_pin/CEREBRO_MEM_BANK_PORT_CONFIG_NB_PORTS_PER_BYTE);

    uint8_t ui_output_mask=0;

#if CEREBRO_MEM_PORT_OUTPUT == 0x01

    ui_output_mask = 0x01<<((i_bank_pin%CEREBRO_MEM_BANK_PORT_CONFIG_NB_PORTS_PER_BYTE)*4);

    if (output == HAL_GPIO_INPUT) {
        *bank_config_reg&=~(ui_output_mask);
    } else {
        *bank_config_reg|=ui_output_mask;
    }
#else

    ui_output_mask = 0x01<<((i_bank_pin%CEREBRO_MEM_BANK_PORT_CONFIG_NB_PORTS_PER_BYTE)*4);

    if (output == HAL_GPIO_INPUT) {
        *bank_config_reg|=ui_output_mask;
    } else {
        *bank_config_reg&=~(ui_output_mask);
    }
#endif

}

int8_t GPIO_Cerebro::analogPinToDigitalPin(uint8_t pin)
{
	return -1;
}


uint8_t GPIO_Cerebro::read(uint8_t pin) {
    // Selecting the corresponding bank.
    int i_bank = get_bank_number(pin);

    if(i_bank>CEREBRO_MEM_NB_BANKS-1)
        return 0;

    uint8_t i_bank_pin = get_bank_pin_number(pin);

    if(i_bank_pin>tui_nb_pins_for_bank[i_bank])
        return 255;

    // getting the corresponding data byte containing the value of the requested pin
    uint8_t ui_reg_value
        = *(m_banks_addr[i_bank].bank_data+(i_bank_pin/CEREBRO_MEM_BANK_PORT_DATA_NB_PORTS_PER_BYTE));

    uint8_t ui_pin_value = (ui_reg_value>>i_bank_pin%CEREBRO_MEM_BANK_PORT_DATA_NB_PORTS_PER_BYTE)&0x01;

    return ui_pin_value;
}

void GPIO_Cerebro::write(uint8_t pin, uint8_t value)
{
    // Selecting the corresponding bank.
    int i_bank = get_bank_number(pin);

    if(i_bank>CEREBRO_MEM_NB_BANKS-1)
        return; 

    // Selecting the corresponding pin number into the given bank
    int i_bank_pin = get_bank_pin_number(pin);

    if(i_bank_pin>tui_nb_pins_for_bank[i_bank])
        return;

    uint8_t ui_pin_value_mask=0x01<<i_bank_pin%CEREBRO_MEM_BANK_PORT_DATA_NB_PORTS_PER_BYTE;

    uint8_t *ui_data_reg = nullptr;

    ui_data_reg = m_banks_addr[i_bank_pin].bank_data+(i_bank_pin/CEREBRO_MEM_BANK_PORT_DATA_NB_PORTS_PER_BYTE);

    if(ui_data_reg == nullptr) {
        AP_HAL::panic("[Cerebro] write() fatal error: data register address returns null pointer");
        exit(-1);
    }

    if (value == 0)
        *ui_data_reg &= ui_pin_value_mask;
    else
        *ui_data_reg |= ui_pin_value_mask;
}

void GPIO_Cerebro::toggle(uint8_t pin)
{
    uint8_t val_pin = read(pin);

    val_pin=(~val_pin)&0x01;

    write(pin, val_pin);
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_Cerebro::channel(uint16_t n) {
    return new DigitalSource(0);
}

/* Interrupt interface: */
bool GPIO_Cerebro::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool GPIO_Cerebro::usb_connected(void)
{
    return false;
}

// DigitalSource::DigitalSource(uint8_t v) :
//     _v(v)
// {}

// void DigitalSource::mode(uint8_t output)
// {}

// uint8_t DigitalSource::read() {
//     return _v;
// }

// void DigitalSource::write(uint8_t value) {
//     _v = value;
// }

// void DigitalSource::toggle() {
//     _v = !_v;
// }

#endif
