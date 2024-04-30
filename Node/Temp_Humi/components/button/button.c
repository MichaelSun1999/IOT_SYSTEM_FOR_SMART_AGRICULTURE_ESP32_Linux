#include "button.h"

void configure_button(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //set interrupt edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //change gpio interrupt type for one pin
    gpio_set_intr_type(GPIO_BUTTON, GPIO_INTR_ANYEDGE);
}
