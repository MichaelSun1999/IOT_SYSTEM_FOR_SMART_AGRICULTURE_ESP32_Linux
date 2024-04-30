/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <app_wifi.h>
#include <app_insights.h>

#include "mynvs.h"

#include "esp_bt.h"

#include "sdkconfig.h"
#include "mqtt_client.h"
//#include "esp_bridge.h"
#define LOG_MEM_INFO    (0)
#define CONFIG_MESH_ROUTE_TABLE_SIZE 50
/*******************************************************
 *                Constants
 *******************************************************/
static char *MESH_TAG = "app_main";
static char *RMAKER_TAG="rainmaker";
static char *UART_TAG = "UART";
/*******************************************************
 *                Variable Definitions
 *******************************************************/
//network
static esp_ip4_addr_t s_current_ip;
esp_netif_t *netif_sta_mesh = NULL;
//mesh
static mesh_addr_t mesh_parent_addr,mesh_root_addr;
static int mesh_layer = -1;
/* store the station info for send back to phone */
static wifi_config_t sta_config;
/*Communication for esp_mesh*/
static uint8_t tx_buf[200] = { 0, };
static uint8_t rx_buf[200] = { 0, };
/*Mesh topolgy formation*/
int Device_pointer=0;
int Light_pointer=0,Soil_pointer=0,Temp_pointer=0;
char *List_Topology[CONFIG_MESH_ROUTE_TABLE_SIZE][6];
char Device_name[20],Device_MAC[18],Function[1],Layer[2],Parent_MAC[18];
/*sensor data transmission*/
int selected_index=0;
int in_list=0;
int got_verification[CONFIG_MESH_ROUTE_TABLE_SIZE]={0};
float test_storage[CONFIG_MESH_ROUTE_TABLE_SIZE][2]={0};
/*RMAKER*/
esp_rmaker_device_t *temp_sensor_device;
esp_rmaker_device_t *humi_sensor_device;
esp_rmaker_device_t *soil_sensor_device;
esp_rmaker_device_t *light_sensor_device;
esp_rmaker_device_t *relay_device;
esp_rmaker_device_t *light_device;
float temp_RM=0,humi_RM=0,soil_RM=0,light_RM=0;
float temp_avg=0,humi_avg=0,soil_avg=0,light_avg=0;
int report_cnt=0;
bool flag_continue=false;
/*UART*/
#define BUF_SIZE (1024)
static QueueHandle_t uart_queue; //队列句柄
static uint8_t  uartbuf[BUF_SIZE];
/*******************************************************
 *                Macros
 *******************************************************/
#define EXAMPLE_BUTTON_GPIO     0
#define DEFAULT_SWITCH_POWER        true
#define DEFAULT_LIGHT_POWER         true
#define REPORTING_PERIOD            60 /* Seconds */
#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)
/*******************************************************
 *                Function Definitions
 *******************************************************/
void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
#if CONFIG_UART_ISR_IN_IRAM
int intr_alloc_flags = 0;
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
    uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    uart_set_rx_full_threshold(ECHO_UART_PORT_NUM,126);
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}
int sendData(const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(ECHO_UART_PORT_NUM, data, len);
    ESP_LOGI(UART_TAG, "Wrote %d bytes: %s", txBytes, data);
    return txBytes;
}
void UART_tx(int function, int data)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    char* print_RM_data;// =0=100=,=1=5000= 1+1+1+4+1=8
    char format[9];
    strcpy(format,  "=%d=%d=");
    asprintf(&print_RM_data,format,function,data);
    sendData(print_RM_data);
}
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE+1);
    for(;;) {
        if(xQueueReceive(uart_queue, (void * )&event,portMAX_DELAY)) {
            switch(event.type) {
                case UART_DATA:
                    ESP_LOGI(UART_TAG, "[UART DATA]: %d", event.size);
                    const int rxBytes=uart_read_bytes(ECHO_UART_PORT_NUM, uartbuf, event.size, portMAX_DELAY);
                    if (rxBytes > 0) {
                        uartbuf[rxBytes] = 0;
                        ESP_LOGI(UART_TAG, "Read %d bytes: '%s'", rxBytes, uartbuf);
                        //=temp=humi=soil=ligh=
                        int i=1,Integer=0,Decimal=0;
                        while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                        {
                            Integer=Integer*10+uartbuf[i]-'0';
                            i++;
                        }
                        i++;
                        while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                        {
                            Decimal=Decimal*10+uartbuf[i]-'0';
                            i++;
                        }
                        temp_RM=(float)Integer+(float)Decimal/100;
                        i++;Integer=0;Decimal=0;
                        while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                        {
                            Integer=Integer*10+uartbuf[i]-'0';
                            i++;
                        }
                        i++;
                        while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                        {
                            Decimal=Decimal*10+uartbuf[i]-'0';
                            i++;
                        }
                        humi_RM=(float)Integer+(float)Decimal/100;
                        i++;Integer=0;Decimal=0;
                        while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                        {
                            Integer=Integer*10+uartbuf[i]-'0';
                            i++;
                        }
                        i++;
                        while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                        {
                            Decimal=Decimal*10+uartbuf[i]-'0';
                            i++;
                        }
                        soil_RM=(float)Integer+(float)Decimal/100;
                        i++;Integer=0;Decimal=0;
                        while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                        {
                            Integer=Integer*10+uartbuf[i]-'0';
                            i++;
                        }
                        i++;
                        while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                        {
                            Decimal=Decimal*10+uartbuf[i]-'0';
                            i++;
                        }
                        light_RM=(float)Integer+(float)Decimal/100;
                        printf("%.2f,%.2f,%.2f,%.2f\n",temp_RM,humi_RM,soil_RM,light_RM);
                        temp_avg=temp_avg+temp_RM;humi_avg=humi_avg+humi_RM;soil_avg=soil_avg+soil_RM;light_avg=light_avg+light_RM;
                        report_cnt++;
                        if(report_cnt>=20)
                        {
                            temp_avg=temp_avg/20;
                            humi_avg=humi_avg/20;
                            soil_avg=soil_avg/20;
                            light_avg=light_avg/20;
                            esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),esp_rmaker_float(temp_RM));
                            esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(humi_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),esp_rmaker_float(humi_RM));
                            esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(soil_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),esp_rmaker_float(soil_RM));
                            esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(light_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),esp_rmaker_float(light_RM));
                            temp_avg=0;humi_avg=0;soil_avg=0;light_avg=0;report_cnt=0;
                        }

                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(UART_TAG, "hw fifo overflow");
                    uart_flush_input(ECHO_UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL: //环形缓冲区满
                    ESP_LOGI(UART_TAG, "ring buffer full");
                    uart_flush_input(ECHO_UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(UART_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(UART_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(UART_TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    break;
                //Others
                default:
                    ESP_LOGI(UART_TAG, "uart event type: %d", event.type);
                    break;
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void MAC_extraction(char tmp[],uint8_t MAC_out[])
{
    uint8_t MAC_tmp[12];
    for(int i=0;i<6;++i)
    {
        if((tmp[3*i]>='0')&&(tmp[3*i]<='9'))
            MAC_tmp[2*i]=tmp[3*i]-'0';
        if((tmp[3*i]>='A')&&(tmp[3*i]<='F'))
            MAC_tmp[2*i]=tmp[3*i]-'A'+10;
        if((tmp[3*i]>='a')&&(tmp[3*i]<='f'))
            MAC_tmp[2*i]=tmp[3*i]-'a'+10;

        if((tmp[3*i+1]>='0')&&(tmp[3*i+1]<='9'))
            MAC_tmp[2*i+1]=tmp[3*i+1]-'0';
        if((tmp[3*i+1]>='A')&&(tmp[3*i+1]<='F'))
            MAC_tmp[2*i+1]=tmp[3*i+1]-'A'+10;
        if((tmp[3*i+1]>='a')&&(tmp[3*i+1]<='f'))
            MAC_tmp[2*i+1]=tmp[3*i+1]-'a'+10;
    }
    for(int i=0;i<6;++i)
    {
        MAC_out[i]=(MAC_tmp[2*i]<<4)+MAC_tmp[2*i+1];
    }
}
static esp_err_t relay_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(RMAKER_TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    if (strcmp(esp_rmaker_param_get_name(param), ESP_RMAKER_DEF_POWER_NAME) == 0) {
        ESP_LOGI(RMAKER_TAG, "Received value = %s for %s - %s",
                val.val.b? "true" : "false", esp_rmaker_device_get_name(device),
                esp_rmaker_param_get_name(param));
        if(val.val.b==true)UART_tx(2,0);
        else UART_tx(2,5000);
        printf("Command to relay complete\n");
        esp_rmaker_param_update_and_report(param, val);
    }
    return ESP_OK;
}
static esp_err_t light_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(RMAKER_TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    if (strcmp(esp_rmaker_param_get_name(param), ESP_RMAKER_DEF_POWER_NAME) == 0) {
        ESP_LOGI(RMAKER_TAG, "Received value = %s for %s - %s",
                val.val.b? "true" : "false", esp_rmaker_device_get_name(device),
                esp_rmaker_param_get_name(param));
        if(val.val.b==false)UART_tx(0,0);
        else UART_tx(0,50);
        esp_rmaker_param_update_and_report(param, val);
    }else if (strcmp(esp_rmaker_param_get_name(param), ESP_RMAKER_DEF_BRIGHTNESS_NAME) == 0) {
        ESP_LOGI(RMAKER_TAG, "Received value = %d for %s - %s",
                val.val.i, esp_rmaker_device_get_name(device), esp_rmaker_param_get_name(param));
        UART_tx(0,val.val.i);
    }
    printf("Command to light complete\n");
    return ESP_OK;
}
float app_get_current_temperature()
{
    return temp_avg;
}
float app_get_current_humidity()
{
    return humi_avg;
}
float app_get_current_soil()
{
    return soil_avg;
}
float app_get_current_light()
{
    return light_avg;
}
void app_main(void)
{    
    
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    uart_init();
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    app_wifi_init();
    //esp_bridge_create_all_netif();
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Multi Device", "Multi Device");
    if (!node) {
        ESP_LOGE(MESH_TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    relay_device = esp_rmaker_switch_device_create("Relay", NULL, DEFAULT_SWITCH_POWER);
    esp_rmaker_device_add_cb(relay_device, relay_cb, NULL);
    esp_rmaker_node_add_device(node, relay_device);
    light_device = esp_rmaker_lightbulb_device_create("Light matrix", NULL, DEFAULT_LIGHT_POWER);
    esp_rmaker_device_add_cb(light_device, light_cb, NULL);
    esp_rmaker_device_add_param(light_device,esp_rmaker_brightness_param_create(ESP_RMAKER_DEF_BRIGHTNESS_NAME, 10));
    esp_rmaker_node_add_device(node, light_device);
    temp_sensor_device = esp_rmaker_temp_sensor_device_create("Temperature Sensor", NULL, app_get_current_temperature());
    esp_rmaker_node_add_device(node, temp_sensor_device);
    humi_sensor_device = esp_rmaker_temp_sensor_device_create("Humidity Sensor", NULL, app_get_current_humidity());
    esp_rmaker_node_add_device(node, humi_sensor_device);
    soil_sensor_device = esp_rmaker_temp_sensor_device_create("Soil humidity Sensor", NULL, app_get_current_soil());
    esp_rmaker_node_add_device(node, soil_sensor_device);
    light_sensor_device = esp_rmaker_temp_sensor_device_create("Light Sensor", NULL, app_get_current_light());
    esp_rmaker_node_add_device(node, light_sensor_device);
    esp_rmaker_timezone_service_enable();
    esp_rmaker_schedule_enable();
    esp_rmaker_scenes_enable();
    app_insights_enable();
    esp_rmaker_start();
    ESP_ERROR_CHECK(app_wifi_start(POP_TYPE_RANDOM));
    //ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    //ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    //ESP_ERROR_CHECK(esp_wifi_start());
}

