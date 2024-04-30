#include "LED_matrix.h"
static led_strip_handle_t led_strip;

// GPIO assignment
#define LED_STRIP_BLINK_GPIO  CONFIG_MATRIX_GPIO
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS CONFIG_MATRIX_LED_NUMBERS
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  CONFIG_MATRIX_RESOLUTION

led_strip_handle_t configure_matrix(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
#endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    return led_strip;
}
void LED_matrix_on(led_strip_handle_t led_strip,int flag,int red,int green,int blue)
{
    if(flag==1)
    {
        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) 
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue));
        }
    }
    else
    {
        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) 
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
        }
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}
