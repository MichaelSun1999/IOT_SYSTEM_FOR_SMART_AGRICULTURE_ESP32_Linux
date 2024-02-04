/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "mesh_light.h"
//#include "button.h"
#include "mesh_netif.h"

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
static SemaphoreHandle_t s_route_table_lock = NULL;
static uint8_t s_mesh_tx_payload[CONFIG_MESH_ROUTE_TABLE_SIZE*6+1];
static int functions[8];
static int record=0;
/*******************************************************
 *                Function Declarations
 *******************************************************/
// interaction with public mqtt broker
void mqtt_app_start(void);
void mqtt_app_publish(char* topic, char *publish_string);

/*******************************************************
 *                Function Definitions
 *******************************************************/

static void initialise_button(void)
{
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(EXAMPLE_BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
}

static void recv_cb(mesh_addr_t *from, mesh_data_t *data)
{
    if (data->data[0] == CMD_ROUTE_TABLE) {
        int size =  data->size - 1;
        if (s_route_table_lock == NULL || size%6 != 0) {
            ESP_LOGE(MESH_TAG, "Error in receiving raw mesh data: Unexpected size");
            return;
        }
        xSemaphoreTake(s_route_table_lock, portMAX_DELAY);
        s_route_table_size = size / 6;
        for (int i=0; i < s_route_table_size; ++i) {
            ESP_LOGI(MESH_TAG, "Received Routing table [%d] "
                    MACSTR, i, MAC2STR(data->data + 6*i + 1));
        }
        memcpy(&s_route_table, data->data + 1, size);
        xSemaphoreGive(s_route_table_lock);
    } else if (data->data[0] == CMD_KEYPRESSED) {
        if (data->size != 7) {
            ESP_LOGE(MESH_TAG, "Error in receiving raw mesh data: Unexpected size");
            return;
        }
        ESP_LOGW(MESH_TAG, "Keypressed detected on node: "
                MACSTR, MAC2STR(data->data + 1));
    } else {
        ESP_LOGE(MESH_TAG, "Error in receiving raw mesh data: Unknown command");
    }
}

static void check_button(void* args)
{
    static bool old_level = true;
    bool new_level;
    bool run_check_button = true;
    initialise_button();
    while (run_check_button) {
        new_level = gpio_get_level(EXAMPLE_BUTTON_GPIO);
        if (!new_level && old_level) {
            printf("Enter check button2,%d\n",s_route_table_size);
            if (s_route_table_size && !esp_mesh_is_root()) {
                ESP_LOGW(MESH_TAG, "Key pressed!");
                mesh_data_t data;
                uint8_t *my_mac = mesh_netif_get_station_mac();
                uint8_t data_to_send[6+1] = { CMD_KEYPRESSED, };
                esp_err_t err;
                char print[6*3+1]; // MAC addr size + terminator
                memcpy(data_to_send + 1, my_mac, 6);
                data.size = 7;
                data.proto = MESH_PROTO_BIN;
                data.tos = MESH_TOS_P2P;
                data.data = data_to_send;
                snprintf(print, sizeof(print),MACSTR, MAC2STR(my_mac));
                mqtt_app_publish("/topic/ip_mesh/key_pressed", print);
                xSemaphoreTake(s_route_table_lock, portMAX_DELAY);
                for (int i = 0; i < s_route_table_size; i++) {
                    if (MAC_ADDR_EQUAL(s_route_table[i].addr, my_mac)) {
                        continue;
                    }
                    err = esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                    ESP_LOGI(MESH_TAG, "Sending to [%d] "
                            MACSTR ": sent with err code: %d", i, MAC2STR(s_route_table[i].addr), err);
                }
                xSemaphoreGive(s_route_table_lock);
            }
        }
        old_level = new_level;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}


void esp_mesh_mqtt_task(void *arg)
{
    is_running = true;
    char *print;
    mesh_data_t data;
    esp_err_t err;
    mqtt_app_start();
    while (is_running) {
        asprintf(&print, "layer:%d IP:" IPSTR, esp_mesh_get_layer(), IP2STR(&s_current_ip));
        ESP_LOGI(MESH_TAG, "Tried to publish %s", print);
        mqtt_app_publish("/topic/ip_mesh", print);
        free(print);
        if (esp_mesh_is_root()) {
            esp_mesh_get_routing_table((mesh_addr_t *) &s_route_table,
                                       CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &s_route_table_size);
            data.size = s_route_table_size * 6 + 1;
            data.proto = MESH_PROTO_BIN;
            data.tos = MESH_TOS_P2P;
            s_mesh_tx_payload[0] = CMD_ROUTE_TABLE;
            memcpy(s_mesh_tx_payload + 1, s_route_table, s_route_table_size*6);
            data.data = s_mesh_tx_payload;
            for (int i = 0; i < s_route_table_size; i++) {
                err = esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                ESP_LOGI(MESH_TAG, "Sending routing table tstao [%d] "
                        MACSTR ": sent with err code: %d", i, MAC2STR(s_route_table[i].addr), err);
            }
        }
        vTaskDelay(2 * 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_mqtt_task_start(void)
{
    static bool is_comm_mqtt_task_started = false;

    s_route_table_lock = xSemaphoreCreateMutex();

    if (!is_comm_mqtt_task_started) {
        xTaskCreate(esp_mesh_mqtt_task, "mqtt task", 3072, NULL, 5, NULL);
        xTaskCreate(check_button, "check button task", 3072, NULL, 5, NULL);
        is_comm_mqtt_task_started = true;
    }
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
        printf("%d,%d\n",ie_len,sizeof(assoc));
        if (ie_len == sizeof(assoc)) {
            ESP_LOGW(MESH_TAG,
                     "<MESH>[%d]%s, layer:%d/%d, assoc:%d/%d, %d, "MACSTR", channel:%u, rssi:%d, ID<"MACSTR"><%s>",
                     i, record.ssid, assoc.layer, assoc.layer_cap, assoc.assoc,
                     assoc.assoc_cap, assoc.layer2_cap, MAC2STR(record.bssid),
                     record.primary, record.rssi, MAC2STR(assoc.mesh_id), assoc.encrypted ? "IE Encrypted" : "IE Unencrypted");
#if CONFIG_MESH_SET_NODE
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
#endif
        } else {
            ESP_LOGI(MESH_TAG, "[%d]%s, "MACSTR", channel:%u, rssi:%d", i,record.ssid, MAC2STR(record.bssid), record.primary,record.rssi);
#if CONFIG_MESH_SET_ROOT
            if (!strcmp(CONFIG_MESH_ROUTER_SSID, (char *) record.ssid)) {
                parent_found = true;
                memcpy(&parent_record, &record, sizeof(record));
                my_type = MESH_ROOT;//the only sink of the mesh network. Has the ability to access external IP network
                my_layer = MESH_ROOT_LAYER;//root layer value
            }
#endif
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
    } else {
        esp_wifi_scan_stop();
        scan_config.show_hidden = 1;
        scan_config.scan_type = WIFI_SCAN_TYPE_PASSIVE;
        esp_wifi_scan_start(&scan_config, 0);
    }
}
void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
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
        #if CONFIG_MESH_SET_ROOT
            ESP_LOGI(MESH_TAG,"Root got child");
            esp_mesh_get_routing_table((mesh_addr_t *) &s_route_table, 8 * 8, &s_route_table_size);
            ESP_LOGI(MESH_TAG,"Root got child2,%d",s_route_table_size);
            esp_err_t err;
            int tmp2=0,tmp0;
            mesh_data_t data;
            for(int i=0;i<s_route_table_size;++i)
            {
                if(MAC_ADDR_EQUAL(s_route_table[i].addr, child_connected->mac)){tmp2=i;break;}
                else{ESP_LOGI(MESH_TAG,"Child mac wrong, child="MACSTR", chosen mac="MACSTR"",MAC2STR(child_connected->mac),MAC2STR(s_route_table[i].addr));}
            }
            ESP_LOGI(MESH_TAG,"Child connected index:%d, "MACSTR", "MACSTR"",tmp2,MAC2STR(child_connected->mac),MAC2STR(s_route_table[tmp2].addr));
            err=esp_mesh_recv(&s_route_table[tmp2],&data,portMAX_DELAY,&tmp0,NULL,0);
            if(err==ESP_OK)ESP_LOGI(MESH_TAG,"Corresponding child function:%d,record=%d",data.data[0],record);
            mesh_route_table_record[record]=s_route_table[tmp2];
            functions[record]=data.data[0];
            record++;
        #endif
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
        #if CONFIG_MESH_SET_NODE//if node,upload its own function to the root
        esp_err_t err;
        mesh_data_t node_index;
        uint8_t data_to_send[1]={CONFIG_MESH_NODE_FUNCTION_INDEX};
        node_index.data=data_to_send;
        node_index.size=1;
        node_index.proto=MESH_PROTO_BIN;
        node_index.tos=MESH_TOS_P2P;
        err = esp_mesh_send(&mesh_root_addr, &node_index, MESH_DATA_TODS , NULL, 0);
        ESP_LOGI(MESH_TAG,"Node Sent basic info:%d,err=%d",CONFIG_MESH_NODE_FUNCTION_INDEX,(int)err);
        #endif
        mesh_netifs_start(esp_mesh_is_root());
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

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
    s_current_ip.addr = event->ip_info.ip.addr;
#if !CONFIG_MESH_USE_GLOBAL_DNS_IP
    esp_netif_t *netif = event->esp_netif;
    esp_netif_dns_info_t dns;
    ESP_ERROR_CHECK(esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns));
    mesh_netif_start_root_ap(esp_mesh_is_root(), dns.ip.u_addr.ip4.addr);
#endif
    esp_mesh_comm_mqtt_task_start();
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
    ESP_ERROR_CHECK(mesh_netifs_init(recv_cb));

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
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
    /* mesh enable IE crypto */
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
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
    cfg.channel = CONFIG_MESH_CHANNEL;
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *) &cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
           strlen(CONFIG_MESH_ROUTER_PASSWD));
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s\n",  esp_get_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed");
}
