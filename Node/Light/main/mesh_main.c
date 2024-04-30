/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "mesh_light.h"
#include "mesh_netif.h"
#include "mynvs.h"

#include "esp_bt.h"
#include "esp_blufi_api.h"
#include "blufi_example.h"
#include "esp_blufi.h"
#include "bh1750.h"
#include "LED_matrix.h"
#include "button.h"
#include "sdkconfig.h"
/*******************************************************
 *                Macros
 *******************************************************/
#define EXAMPLE_BUTTON_GPIO     0

// commands for internal mesh communication:
// <CMD> <PAYLOAD>, where CMD is one character, payload is variable dep. on command
#define CMD_KEYPRESSED 0x55
// CMD_KEYPRESSED: payload is always 6 bytes identifying address of node sending keypress event
#define CMD_ROUTE_TABLE 0x56
// CMD_KEYPRESSED: payload is a multiple of 6 listing addresses in a routing table
/*******************************************************
 *                Constants
 *******************************************************/
static const char *MESH_TAG = "mesh_main";
static const char *HTS221_TAG = "hts221 test";
static const char *BH1750_TAG = "bh1750 test";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x76};

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static bool is_running = true;
//network
static esp_ip4_addr_t s_current_ip;
static esp_netif_t *netif_sta = NULL;
//mesh
static mesh_addr_t mesh_parent_addr,mesh_root_addr;
static mesh_addr_t s_route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
static mesh_addr_t mesh_route_table_record[8];
static int mesh_layer = -1;
static int s_route_table_size = 0;
static SemaphoreHandle_t s_recv_cb_lock = NULL,s_route_table_lock = NULL,s_data_transmission_lock=NULL;


static uint8_t example_wifi_retry = 0;

/* store the station info for send back to phone */
static wifi_config_t sta_config;
static wifi_config_t ap_config;
static bool ble_is_connected = false;
static bool SSID_got = false;
static bool password_got = false;

/*Storing data from MQTT*/
int topic_len=0,data_len=0,MQTT_msg_arrival=0,MQTT_data_arrival=0;
char *topic,*data;
/*Communication for esp_mesh*/
static uint8_t tx_buf[200] = { 0, };
static uint8_t rx_buf[200] = { 0, };
//Mesh topolgy formation
int Device_pointer=0;
char *List_Topology[CONFIG_MESH_ROUTE_TABLE_SIZE][6];
char Device_name[20],Device_MAC[18],Function[1],Layer[2],Parent_MAC[18];
//sensor data transmission
int selected_index=0;
static bool LED_strip_on=true;
static bool getting_data=true;
static bh1750_handle_t bh1750 = NULL;
led_strip_handle_t led_strip;
float bh1750_data;
int flag_avg=0;
int in_list=0;
int got_verification[CONFIG_MESH_ROUTE_TABLE_SIZE]={0};
float test_storage[CONFIG_MESH_ROUTE_TABLE_SIZE][2]={0};
int start_count=5;
float accumulate=0;
int force_brightness=0;
/*******************************************************
 *                Function Declarations
 *******************************************************/
// interaction with public mqtt broker
void mqtt_app_start(void);
void mqtt_app_publish(char* topic, char *publish_string);
/*******************************************************
 *                Function Definitions
 *******************************************************/
 //LED strip
static void LED_matrix_task(void* args)
{
    LED_strip_on = true;
    esp_err_t err;
    bool led_on_off = false;
    led_strip = configure_matrix();
    int red=CONFIG_MATRIX_BRIGHTNESS_RED;
    int green=CONFIG_MATRIX_BRIGHTNESS_GREEN;
    int blue=CONFIG_MATRIX_BRIGHTNESS_BLUE;
    LED_matrix_on(led_strip,0,red,green,blue); 
    while(flag_avg<1){}
    accumulate=accumulate/5;
    printf("average=%.2f\n",accumulate);
    while (LED_strip_on) {
        if(accumulate>0)
        {
            if(bh1750_data<accumulate*0.9)
            {
                red=red+5;
                green=green+5;
                blue=blue+5;
                if(red>255)red=255;
                if(green>255)green=255;
                if(blue>255)blue=255;
                LED_matrix_on(led_strip,1,red,green,blue); 
                printf("up:%d\n",red);
            }
            if(bh1750_data>accumulate*1.1)
            {
                red=red-5;
                green=green-5;
                blue=blue-5;
                if(red<0)red=0;
                if(green<0)green=0;
                if(blue<0)blue=0;
                LED_matrix_on(led_strip,1,red,green,blue); 
                printf("down:%d\n",red);
            }
        }
        else
        {
            printf("average error\n");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
//BH1750
void BH1750_whole_init(void)
{
    bh1750=bh1750_init();
    ESP_ERROR_CHECK(bh1750_power_on(bh1750));
}
static void BH1750_read(void* args)
{
    getting_data = true;
    esp_err_t err;
    bh1750_measure_mode_t cmd_measure;
    BH1750_whole_init();
    while (getting_data) {
        cmd_measure = BH1750_CONTINUE_4LX_RES;
        ESP_ERROR_CHECK(bh1750_set_measure_mode(bh1750, cmd_measure));
        vTaskDelay(30 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(bh1750_get_data(bh1750, &bh1750_data));
        ESP_LOGI(BH1750_TAG, "bh1750 val(continuously mode): %f\n", bh1750_data);  
        
        if(start_count>0)
        {
            if(start_count<6)accumulate=accumulate+bh1750_data;
            start_count--;
        }
        if(start_count<=0)
        {
            flag_avg=1;
        }
        vTaskDelay( 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
//button
static void check_button(void* args)
{
    static bool old_level = true;
    bool new_level;
    bool run_check_button = true;
    configure_button();
    while (run_check_button) {
        new_level = gpio_get_level(EXAMPLE_BUTTON_GPIO);
        if (!new_level && old_level) {
            //delete the info in NVS
            char Zero1[32];memset(Zero1,'\0',sizeof(Zero1));
            char Zero2[64];memset(Zero2,'\0',sizeof(Zero2));
            nvs_save_wifi_info(Zero1,Zero2);
            nvs_clear_start_flag();
            ESP_LOGI(MESH_TAG,"Please restart the node");
        }
        old_level = new_level;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}
//MESH
void esp_mesh_p2p_tx_main(void *arg)
{
    esp_err_t err;
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    mesh_data_t data;
    data.data = tx_buf;
    data.size = 200;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    static bool is_running_tx = true;

    while (is_running_tx) 
    {
        if(!esp_mesh_is_root())
        {
            if(in_list==0)
            {
                //Sending mesh topology, listened by root
                char *print_mesh_topo;
                uint8_t *my_mac=(uint8_t*)calloc(6,sizeof(uint8_t));//=mesh_netif_get_station_mac();
                ESP_ERROR_CHECK(esp_efuse_mac_get_default(my_mac));
                uint8_t MAC[6];
                memcpy(MAC,my_mac,6);
                //format:Device_name=ESP32S3==11:22:33:44:55:66==0==1==11:22:33:44:55:66=
                //                             Device_MAC Function Layer Parent_MAC
                char format[112];
                memset(format,'\0',112);
                strcpy(format,  "Device_name=%s==");//14+20
                strcat(format,MACSTR);//18(Device_MAC)
                strcat(format,"==%d==%d==");//6+2(Function+Layer)
                strcat(format,MACSTR);//18(Parent_MAC)
                strcat(format,"=");//1
                int layer=esp_mesh_get_layer();
                asprintf(&print_mesh_topo, format,CONFIG_DEVICE_NAME,MAC2STR(MAC),CONFIG_MESH_NODE_FUNCTION_INDEX,layer,MAC2STR(mesh_parent_addr.addr));
                for(int i=0;i<112;++i)
                {
                    tx_buf[i]=(uint8_t)print_mesh_topo[i];
                }
                //memcpy(tx_buf, (uint8_t *)&print_mesh_topo, sizeof(print_mesh_topo));
                printf("%s\n",(char*)tx_buf);
                ESP_LOGI(MESH_TAG, "Tried to publish %s", print_mesh_topo);
                err = esp_mesh_send(NULL, &data, MESH_DATA_P2P, NULL, 0);
                free(print_mesh_topo);
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            if(in_list)
            {
                char *print_sensor_data;
                //format:=ESP32S3=data(%.2f)=data(%.2f)=-->size=1+20+1+10+1+10+1=44
                char format[45];
                strcpy(format,  "=%s=%.2f=%.2f=");
                float tmp=bh1750_data;
                asprintf(&print_sensor_data,format,CONFIG_DEVICE_NAME,tmp,tmp);
                for(int i=0;i<45;++i)
                {
                    tx_buf[i]=(uint8_t)print_sensor_data[i];
                }
                //memcpy(tx_buf, (uint8_t *)&print_sensor_data, sizeof(print_sensor_data));
                err = esp_mesh_send(NULL, &data, MESH_DATA_P2P, NULL, 0);
                vTaskDelay(CONFIG_DATA_TRANSMISSION_RATE / portTICK_PERIOD_MS);
            }
        }
        
    }
    vTaskDelete(NULL);
}
void esp_mesh_p2p_rx_main(void *arg)
{
    esp_err_t err;
    mesh_rx_pending_t pending;
    static bool is_running_rx = true;
    char* print_receive;
    mesh_addr_t from;
    mesh_data_t data;
    data.data = rx_buf;
    data.size = 200;
    int flag = 0;
    while (is_running_rx) {
        err=esp_mesh_get_rx_pending(&pending);
        if(!esp_mesh_is_root())
        {
            if(pending.toSelf>0)
            {
                err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
                print_receive=(char*)data.data;
                if(print_receive[0]=='=')
                {
                    char tmp[20];memset(tmp,'\0',sizeof(tmp));
                    int j=1;
                    while(print_receive[j]!='=')
                    {
                        tmp[j-1]=print_receive[j];
                        j++;
                    }
                    int cnt=0;
                    for(j=0;j<sizeof(CONFIG_DEVICE_NAME);++j)
                    {
                        if(CONFIG_DEVICE_NAME[j]==tmp[j])cnt++;
                    }
                    //printf("%d\n",cnt);
                    if(cnt>=sizeof(CONFIG_DEVICE_NAME))
                    {
                        printf("Mesh topology verified\n");
                        in_list=1;
                    }
                }
                if(print_receive[0]=='C')
                {
                    if(CONFIG_MESH_NODE_FUNCTION_INDEX==0)
                    {
                        if(print_receive[8]=='0')
                        {
                            int j=10;
                            force_brightness=0;
                            while((print_receive[j]>='0')&&(print_receive[j]<='9'))
                            {
                                force_brightness=force_brightness*10+print_receive[j]-'0';
                                j++;
                            }
                            accumulate=force_brightness;
                            LED_matrix_on(led_strip,1,force_brightness,force_brightness,force_brightness);
                            printf("Force the brightness to be %d\n",force_brightness);
                        }
                    }
                }
            }
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(check_button, "button task", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
        xTaskCreate(LED_matrix_task, "LED matrix operation task", 3072, NULL, 5, NULL);
        xTaskCreate(BH1750_read, "BH1750 read task", 3072, NULL, 5, NULL);
        for(int i=0;i<CONFIG_MESH_ROUTE_TABLE_SIZE;++i)
        {
            List_Topology[i][0]=malloc(sizeof(char)*20);
            List_Topology[i][1]=malloc(sizeof(char)*18);
            List_Topology[i][2]=malloc(sizeof(char)*12);
            List_Topology[i][3]=malloc(sizeof(char)*2);
            List_Topology[i][4]=malloc(sizeof(char)*20);
            List_Topology[i][5]=malloc(sizeof(char)*18);
        }
        for(int i=0;i<CONFIG_MESH_ROUTE_TABLE_SIZE;++i)
        {
            memset(List_Topology[i][0],'\0',20);
            memset(List_Topology[i][1],'\0',18);
            memset(List_Topology[i][2],'\0',12);
            memset(List_Topology[i][3],'\0',2);
            memset(List_Topology[i][4],'\0',20);
            memset(List_Topology[i][5],'\0',18);
        }
    }
    is_comm_p2p_started=true;
    return ESP_OK;
}

void mesh_scan_done_handler(int num)
{
    int i;
    int ie_len = 0;
    mesh_assoc_t assoc;
    mesh_assoc_t parent_assoc = { .layer = CONFIG_MESH_MAX_LAYER, .rssi = -120 };
    wifi_ap_record_t record;
    wifi_ap_record_t parent_record = { 0, };
    bool parent_found = false;
    mesh_type_t my_type = MESH_IDLE;//hasn't joined the mesh network yet
    int my_layer = -1;
    wifi_config_t parent = { 0, };
    wifi_scan_config_t scan_config = { 0 };
    for (i = 0; i < num; i++) {
        esp_mesh_scan_get_ap_ie_len(&ie_len);//Get mesh networking IE length of one AP.
        esp_mesh_scan_get_ap_record(&record, &assoc);//Get AP record. Different from esp_wifi_scan_get_ap_records(), this API only gets one of APs scanned each time.
        if (ie_len == sizeof(assoc)) {
            ESP_LOGW(MESH_TAG,
                     "<MESH>[%d]%s, layer:%d/%d, assoc:%d/%d, %d, "MACSTR", channel:%u, rssi:%d, ID<"MACSTR"><%s>",
                     i, record.ssid, assoc.layer, assoc.layer_cap, assoc.assoc,
                     assoc.assoc_cap, assoc.layer2_cap, MAC2STR(record.bssid),
                     record.primary, record.rssi, MAC2STR(assoc.mesh_id), assoc.encrypted ? "IE Encrypted" : "IE Unencrypted");
            if (assoc.mesh_type != MESH_IDLE && assoc.layer_cap && assoc.assoc < assoc.assoc_cap && record.rssi > -70) {
                if (assoc.layer < parent_assoc.layer || assoc.layer2_cap < parent_assoc.layer2_cap) {
                    parent_found = true;
                    memcpy(&parent_record, &record, sizeof(record));
                    memcpy(&parent_assoc, &assoc, sizeof(assoc));
                    if (parent_assoc.layer_cap != 1) {my_type = MESH_NODE;} 
                    else {my_type = MESH_LEAF;}
                    my_layer = parent_assoc.layer + 1;
                    break;
                }
            }
        } 
    }
    esp_mesh_flush_scan_result();
    if (parent_found) {
        parent.sta.channel = parent_record.primary;//channel of AP
        memcpy(&parent.sta.ssid, &parent_record.ssid,sizeof(parent_record.ssid));
        parent.sta.bssid_set = 1;
        memcpy(&parent.sta.bssid, parent_record.bssid, 6);
        if (my_type == MESH_ROOT) {//the node is the root
            if (parent_record.authmode != WIFI_AUTH_OPEN) {
                memcpy(&parent.sta.password, CONFIG_MESH_ROUTER_PASSWD,strlen(CONFIG_MESH_ROUTER_PASSWD));
            }
            ESP_LOGW(MESH_TAG, "<PARENT>%s, "MACSTR", channel:%u, rssi:%d",parent_record.ssid, MAC2STR(parent_record.bssid),parent_record.primary, parent_record.rssi);
            ESP_ERROR_CHECK(esp_mesh_set_parent(&parent, NULL, my_type, my_layer));//Set a specified parent for the device.
        } else {
            ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(parent_record.authmode));
            if (parent_record.authmode != WIFI_AUTH_OPEN) {
                memcpy(&parent.sta.password, CONFIG_MESH_AP_PASSWD,strlen(CONFIG_MESH_AP_PASSWD));
            }
            ESP_LOGW(MESH_TAG,"<PARENT>%s, layer:%d/%d, assoc:%d/%d, %d, "MACSTR", channel:%u, rssi:%d",parent_record.ssid, parent_assoc.layer,parent_assoc.layer_cap, parent_assoc.assoc,parent_assoc.assoc_cap, parent_assoc.layer2_cap,MAC2STR(parent_record.bssid), parent_record.primary,parent_record.rssi);
            ESP_ERROR_CHECK(esp_mesh_set_parent(&parent, (mesh_addr_t *)&parent_assoc.mesh_id, my_type, my_layer));
        }
        esp_wifi_scan_stop();
    } else {
        esp_wifi_scan_stop();
        scan_config.show_hidden = 1;
        scan_config.scan_type = WIFI_SCAN_TYPE_PASSIVE;
        esp_wifi_scan_start(&scan_config, 0);
    }
}
void mesh_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint8_t last_layer = 0;
    wifi_scan_config_t scan_config = { 0 };

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        mesh_layer = esp_mesh_get_layer();
        //close self-organize,start set root
        ESP_ERROR_CHECK(esp_mesh_set_self_organized(0, 0));
        esp_wifi_scan_stop();
        /* mesh softAP is hidden */
        scan_config.show_hidden = 1;//scan AP whose SSID is hidden
        scan_config.scan_type = WIFI_SCAN_TYPE_PASSIVE;//passive scan
        esp_wifi_scan_start(&scan_config, 0);
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,"<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR"",last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),esp_mesh_is_root() ? "<ROOT>" :(mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr));
        last_layer = mesh_layer;
        //mesh_netifs_start(esp_mesh_is_root());
        mesh_connected_indicator(mesh_layer);
        //open dhcpc for root(esp_netif_create_default_wifi_mesh_netifs()just open AP and ATA,not dhcpc)
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        mesh_disconnected_indicator();
        mesh_layer = esp_mesh_get_layer();
        if (disconnected->reason == WIFI_REASON_ASSOC_TOOMANY) {
            //esp_wifi_disconnect();
            esp_wifi_scan_stop();
            scan_config.show_hidden = 1;
            scan_config.scan_type = WIFI_SCAN_TYPE_PASSIVE;
            //esp_wifi_connect();
            esp_wifi_scan_start(&scan_config, 0);
        }
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
        for(int i=0;i<6;++i)
        {
            mesh_root_addr.addr[i]=root_addr->addr[i];
        }
        mesh_root_addr.mip=root_addr->mip;
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
        mesh_scan_done_handler(scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%" PRId32 "", event_id);
        break;
    }
}
void ip_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
    s_current_ip.addr = event->ip_info.ip.addr;
}
//Blufi
static void example_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);
static esp_blufi_callbacks_t example_callbacks = {
    .event_cb = example_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};
static void example_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
{
    /* actually, should post to blufi_task handle the procedure,
     * now, as a example, we do it more simply */
    switch (event) {
    case ESP_BLUFI_EVENT_INIT_FINISH:
        BLUFI_INFO("BLUFI init finish\n");
        esp_blufi_adv_start();
        break;
    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        BLUFI_INFO("BLUFI deinit finish\n");
        break;
    case ESP_BLUFI_EVENT_BLE_CONNECT:
        BLUFI_INFO("BLUFI ble connect\n");
        ble_is_connected = true;
        esp_blufi_adv_stop();
        blufi_security_init();
        break;
    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        BLUFI_INFO("BLUFI ble disconnect\n");
        ble_is_connected = false;
        blufi_security_deinit();
        esp_blufi_adv_start();
        break;
    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        break;
    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        BLUFI_INFO("BLUFI requset wifi connect to AP\n");
        break;
    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        BLUFI_INFO("BLUFI requset wifi disconnect from AP\n");
        esp_wifi_disconnect();
        break;
    case ESP_BLUFI_EVENT_REPORT_ERROR:
        BLUFI_ERROR("BLUFI report error, error code %d\n", param->report_error.state);
        esp_blufi_send_error_info(param->report_error.state);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
        BLUFI_INFO("BLUFI get wifi status from AP\n");
        break;
    }
    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        BLUFI_INFO("blufi close a gatt connection");
        esp_blufi_disconnect();
        break;
    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
        /* TODO */
        break;
	case ESP_BLUFI_EVENT_RECV_STA_BSSID:
        memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
        sta_config.sta.bssid_set = 1;
        BLUFI_INFO("Recv STA BSSID %s\n", sta_config.sta.bssid);
        break;
	case ESP_BLUFI_EVENT_RECV_STA_SSID:
        strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
        sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
        BLUFI_INFO("Recv STA SSID %s\n", sta_config.sta.ssid);
        SSID_got=true;
        break;
	case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        strncpy((char *)sta_config.sta.password, (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
        BLUFI_INFO("Recv STA PASSWORD %s\n", sta_config.sta.password);
        password_got=true;
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
        strncpy((char *)ap_config.ap.ssid, (char *)param->softap_ssid.ssid, param->softap_ssid.ssid_len);
        ap_config.ap.ssid[param->softap_ssid.ssid_len] = '\0';
        ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
        BLUFI_INFO("Recv SOFTAP SSID %s, ssid len %d\n", ap_config.ap.ssid, ap_config.ap.ssid_len);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
        strncpy((char *)ap_config.ap.password, (char *)param->softap_passwd.passwd, param->softap_passwd.passwd_len);
        ap_config.ap.password[param->softap_passwd.passwd_len] = '\0';
        BLUFI_INFO("Recv SOFTAP PASSWORD %s len = %d\n", ap_config.ap.password, param->softap_passwd.passwd_len);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
        if (param->softap_max_conn_num.max_conn_num > 4) {
            return;
        }
        ap_config.ap.max_connection = param->softap_max_conn_num.max_conn_num;
        BLUFI_INFO("Recv SOFTAP MAX CONN NUM %d\n", ap_config.ap.max_connection);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
        if (param->softap_auth_mode.auth_mode >= WIFI_AUTH_MAX) {
            return;
        }
        ap_config.ap.authmode = param->softap_auth_mode.auth_mode;
        BLUFI_INFO("Recv SOFTAP AUTH MODE %d\n", ap_config.ap.authmode);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
        if (param->softap_channel.channel > 13) {
            return;
        }
        ap_config.ap.channel = param->softap_channel.channel;
        BLUFI_INFO("Recv SOFTAP CHANNEL %d\n", ap_config.ap.channel);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_LIST:{
        wifi_scan_config_t scanConf = {
            .ssid = NULL,
            .bssid = NULL,
            .channel = 0,
            .show_hidden = false
        };
        esp_err_t ret = esp_wifi_scan_start(&scanConf, true);
        if (ret != ESP_OK) {
            esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
        }
        break;
    }
    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
        BLUFI_INFO("Recv Custom Data %" PRIu32 "\n", param->custom_data.data_len);
        esp_log_buffer_hex("Custom Data", param->custom_data.data, param->custom_data.data_len);
        break;
	case ESP_BLUFI_EVENT_RECV_USERNAME:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CA_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
        /* Not handle currently */
        break;;
	case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
        /* Not handle currently */
        break;
    default:
        break;
    }
}
void app_main(void)
{
    configure_led();
    
    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  crete network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));

    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
    /* mesh enable IE crypto */
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memset(cfg.router.ssid,'\0',32); memset(cfg.router.password,'\0',64); memset(cfg.mesh_ap.password,'\0',64);
    memset(sta_config.sta.ssid,'\0',32); memset(sta_config.sta.password,'\0',64);
    
#if CONFIG_MESH_IE_CRYPTO_FUNCS
    /* modify IE crypto key */
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_funcs(&g_wifi_default_mesh_crypto_funcs));
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_key(CONFIG_MESH_IE_CRYPTO_KEY, strlen(CONFIG_MESH_IE_CRYPTO_KEY)));
#else
    /* disable IE crypto */
    ESP_LOGI(MESH_TAG, "<Config>disable IE crypto");
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_funcs(NULL));
#endif
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router */

//read from NVS first
    if(nvs_read_start_flag()==0)//not initialized, so initialize it using '\0'
    {
        memset(sta_config.sta.ssid,'\0',32*sizeof(uint8_t));
        memset(sta_config.sta.password,'\0',64*sizeof(uint8_t));
        nvs_save_wifi_info((char*)sta_config.sta.ssid,(char*)sta_config.sta.password);
        nvs_save_start_flag();
        printf("The very first: SSID:%s,passwd:%s",(char*)sta_config.sta.ssid,(char*)sta_config.sta.password);
    }
    else
    {
        char *read_SSID,*read_passwd;
        nvs_read_wifi_info(&read_SSID,&read_passwd);
        printf("Read from NVS: SSID:%s,passwd:%s",read_SSID,read_passwd);
        if(strlen(read_SSID)!=0)
        {
            SSID_got=true;
            cfg.router.ssid_len = strlen(read_SSID);
            memcpy((uint8_t *) &cfg.router.ssid, read_SSID, cfg.router.ssid_len);
            cfg.router.ssid[cfg.router.ssid_len+1]='\0';
        }
        if(strlen(read_passwd)!=0)
        {
            password_got=true;
            memcpy((uint8_t *) &cfg.router.password,read_passwd,strlen(read_passwd));
            cfg.router.password[strlen(read_passwd)+1]='\0';
        }
        printf("SSID:%s,%s;passwd:%s,%s\n",(char*)cfg.router.ssid,read_SSID,(char*)cfg.router.password,read_passwd);
    }

//if no result in NVS, wait for Blufi
    if((SSID_got==false)||(password_got==false))
    {
        esp_err_t ret;
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            BLUFI_ERROR("%s initialize bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            BLUFI_ERROR("%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
        ret = esp_blufi_host_and_cb_init(&example_callbacks);
        if (ret) {
            BLUFI_ERROR("%s initialise failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
        BLUFI_INFO("BLUFI VERSION %04x\n", esp_blufi_get_version());
        printf("Waiting for blufi\n");
        while((SSID_got==false)||(password_got==false)){}//wait for data
        cfg.channel = CONFIG_MESH_CHANNEL;
        
        int cnt=0;char j=sta_config.sta.ssid[0];
        while(j!='\0'){j=sta_config.sta.ssid[cnt++];}
        cfg.router.ssid_len = cnt;
        
        if(sta_config.sta.ssid[cfg.router.ssid_len-1]!='\0'){sta_config.sta.ssid[cfg.router.ssid_len]='\0';cfg.router.ssid_len+=1;}
        memcpy((uint8_t *) &cfg.router.ssid, sta_config.sta.ssid, cfg.router.ssid_len);
        
        cnt=0;j=sta_config.sta.password[0];
        while(j!='\0'){j=sta_config.sta.password[cnt++];}

        if(sta_config.sta.password[cnt-1]!='\0')sta_config.sta.password[cnt]='\0';
        memcpy((uint8_t *) &cfg.router.password, sta_config.sta.password,cnt);
        nvs_save_wifi_info((char*)sta_config.sta.ssid,(char*)sta_config.sta.password);
        ESP_ERROR_CHECK(esp_blufi_host_deinit());
        ESP_ERROR_CHECK(esp_bt_controller_disable());
        ESP_ERROR_CHECK(esp_bt_controller_deinit());
    }

    ESP_LOGI(MESH_TAG,"SSID=%s,password=%s",cfg.router.ssid,cfg.router.password);
    
    
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,strlen(CONFIG_MESH_AP_PASSWD));
    cfg.mesh_ap.password[strlen(CONFIG_MESH_AP_PASSWD)/sizeof(CONFIG_MESH_AP_PASSWD[0])]='\0';
    printf("%s,%s,%s,%c",(char*)cfg.router.ssid,(char*)cfg.router.password,(char*)cfg.mesh_ap.password,cfg.mesh_ap.password[strlen(CONFIG_MESH_AP_PASSWD)/sizeof(CONFIG_MESH_AP_PASSWD[0])]);
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s\n",  esp_get_free_heap_size(),esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed");
}
