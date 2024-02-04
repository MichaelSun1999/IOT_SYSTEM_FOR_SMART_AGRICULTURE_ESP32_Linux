#include "mesh_light.h"
static const char *LED_TAG = "LED";
static led_strip_handle_t led_strip;
int brightness=10;
void configure_led(void)
{
    ESP_LOGI(LED_TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}
void blink_led(int red,int green,int blue)
{
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    led_strip_set_pixel(led_strip, 0, red,green,blue);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
}
esp_err_t mesh_light_set(int color)
{
    switch (color) {
    case MESH_LIGHT_RED:
        /* Red */
        blink_led(brightness,0,0);
        break;
    case MESH_LIGHT_GREEN:
        /* Green */
        blink_led(0,brightness,0);
        break;
    case MESH_LIGHT_BLUE:
        /* Blue */
        blink_led(0,0,brightness);
        break;
    case MESH_LIGHT_YELLOW:
        /* Yellow */
        blink_led(brightness,brightness,0);
        break;
    case MESH_LIGHT_PINK:
        /* Pink */
        blink_led(brightness,0,brightness);
        break;
    case MESH_LIGHT_INIT:
        /* can't say */
        blink_led(0,brightness,brightness);
        break;
    case MESH_LIGHT_WARNING:
        /* warning */
        blink_led(brightness,brightness,brightness);
        break;
    default:
        /* off */
        blink_led(0,0,0);
    }
    return ESP_OK;
}
void mesh_connected_indicator(int layer)
{
    switch (layer) {
    case 1:
        mesh_light_set(MESH_LIGHT_PINK);
        break;
    case 2:
        mesh_light_set(MESH_LIGHT_YELLOW);
        break;
    case 3:
        mesh_light_set(MESH_LIGHT_RED);
        break;
    case 4:
        mesh_light_set(MESH_LIGHT_BLUE);
        break;
    case 5:
        mesh_light_set(MESH_LIGHT_GREEN);
        break;
    case 6:
        mesh_light_set(MESH_LIGHT_WARNING);
        break;
    default:
        mesh_light_set(0);
    }
}
void mesh_disconnected_indicator(void)
{
    mesh_light_set(MESH_LIGHT_WARNING);
}
