#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "led_strip.h"


led_strip_handle_t configure_matrix(void);
void LED_matrix_on(led_strip_handle_t led_strip,int flag,int red,int green,int blue);
