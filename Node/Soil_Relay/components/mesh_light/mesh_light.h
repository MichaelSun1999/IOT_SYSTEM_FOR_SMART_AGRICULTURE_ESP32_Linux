#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "led_strip.h"

#define MESH_LIGHT_RED       (0xff)
#define MESH_LIGHT_GREEN     (0xfe)
#define MESH_LIGHT_BLUE      (0xfd)
#define MESH_LIGHT_YELLOW    (0xfc)
#define MESH_LIGHT_PINK      (0xfb)
#define MESH_LIGHT_INIT      (0xfa)
#define MESH_LIGHT_WARNING   (0xf9)

#define  MESH_TOKEN_ID       (0x0)
#define  MESH_TOKEN_VALUE    (0xbeef)
#define  MESH_CONTROL_CMD    (0x2)
#define BLINK_GPIO CONFIG_BLINK_GPIO


typedef struct {
    uint8_t cmd;
    bool on;
    uint8_t token_id;
    uint16_t token_value;
} mesh_light_ctl_t;

void configure_led(void);
void blink_led(int red,int green,int blue);
esp_err_t mesh_light_set(int color);
void mesh_connected_indicator(int layer);
void mesh_disconnected_indicator(void);
