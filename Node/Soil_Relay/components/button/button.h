#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define GPIO_BUTTON    CONFIG_GPIO_BUTTON
#define GPIO_INPUT_PIN_SEL  (1ULL<<CONFIG_GPIO_BUTTON)

void configure_button(void);
