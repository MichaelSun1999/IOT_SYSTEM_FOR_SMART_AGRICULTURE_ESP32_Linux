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

#include "lv_demos.h"
#include "bsp/esp-bsp.h"

#include "mesh_netif.h"
#include "mynvs.h"

#include "esp_bt.h"
#include "esp_blufi_api.h"
#include "blufi_example.h"
#include "esp_blufi.h"

#include "sdkconfig.h"
#include "mqtt_client.h"
#define LOG_MEM_INFO    (0)
/*******************************************************
 *                Constants
 *******************************************************/
static char *MESH_TAG = "app_main";
static char *UART_TAG = "UART";
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
int communication_process[CONFIG_MESH_ROUTE_TABLE_SIZE]={0};
/*Mesh topolgy formation*/
int Device_pointer=0;
int Light_pointer=0,Soil_pointer=0,Temp_pointer=0;
char *List_Topology[CONFIG_MESH_ROUTE_TABLE_SIZE][6];
char Device_name[20],Device_MAC[18],Function[1],Layer[2],Parent_MAC[18];
/*sensor data transmission*/
int selected_index=0;
float sensor_data=30.00;
int in_list=0;
int got_verification[CONFIG_MESH_ROUTE_TABLE_SIZE]={0};
float test_storage[CONFIG_MESH_ROUTE_TABLE_SIZE][2]={0};
bool sensor_force=false;
/*LVGL*/
lv_obj_t *background,*project_title,*button_operation,*label_provision,*button_topo,*label_topo,*mesh_data_table;
lv_obj_t *keyboard,*Brightness_title,*input_Brightness,*Humidity_title,*input_Humidity,*button_confirm,*label_confirm;
lv_obj_t *mesh_topo_title,*button_return,*label_return,*mesh_topo_table;
/*UART*/
#define BUF_SIZE (1024)
static QueueHandle_t uart_queue; //队列句柄
static uint8_t  uartbuf[BUF_SIZE];
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
        if(esp_mesh_is_root())
        {
            esp_mesh_get_routing_table((mesh_addr_t *) &route_table,CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);
            for(int k=0;k<Device_pointer;++k)
            {
                //printf("Enter ACK\n");
                if(got_verification[k]==1)
                {
                    //printf("ACKing,router table size=%d\n",route_table_size);
                    char* print_OK;//format:=ESP32S3_2=OK=(1+20+1+2+1=25)
                    char format[25];
                    strcpy(format,  "=%s=OK=");
                    asprintf(&print_OK,format,List_Topology[k][0]);
                    for(int i=0;i<200;++i)
                    {
                        tx_buf[i]=0;
                    }
                    for(int i=0;i<25;++i)
                    {
                        tx_buf[i]=(uint8_t)print_OK[i];
                    }
                    //memcpy(tx_buf, (uint8_t *)&print_OK, sizeof(print_OK));
                    for (int i = 0; i < route_table_size; i++)
                    {
                        uint8_t MAC_tmp[6];
                        MAC_extraction(List_Topology[k][1],MAC_tmp);
                        //printf("%X:%X:%X:%X:%X:%X;%X:%X:%X:%X:%X:%X\n",MAC_tmp[0],MAC_tmp[1],MAC_tmp[2],MAC_tmp[3],MAC_tmp[4],MAC_tmp[5],(route_table[i].addr)[0],(route_table[i].addr)[1],(route_table[i].addr)[2],(route_table[i].addr)[3],(route_table[i].addr)[4],(route_table[i].addr)[5]);
                        int MAC_cnt=0;
                        for(int j=0;j<6;++j)
                        {
                            if((route_table[i].addr)[j]==MAC_tmp[j])
                            {
                                MAC_cnt++;
                            }
                        }
                        if(MAC_cnt==6)
                        {
                            //printf("TX:%s\n",List_Topology[k][0]);
                            err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                        }
                    }
                }
            }  
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
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
    int flag = 0;
    while (is_running_rx) {
        err=esp_mesh_get_rx_pending(&pending);
        if(esp_mesh_is_root())
        {
            if(pending.toSelf>0)
            {
                for(int i=0;i<200;++i)rx_buf[i]='\0';
                data.data = rx_buf;
                data.size=200;
                err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
                print_receive=(char*)data.data;
                //for(int i=0;i<200;++i)printf("%X ",data.data[i]);
                //printf("\n");
                //printf("ERR=%d,FROM:%X:%X:%X:%X:%X:%X\n",err,from.addr[0],from.addr[1],from.addr[2],from.addr[3],from.addr[4],from.addr[5]);
                //printf("Size:%d,Received:%s\n",data.size,print_receive);
                if(print_receive[0]=='=')
                {
                    char msg[45];strncpy(msg,print_receive,45);msg[44]='\0';
                    printf("Data transmission:%s\n",msg);
                    char tmp[20];memset(tmp,'\0',sizeof(tmp));
                    int j=1;
                    if(msg[0]=='=')
                    {
                        while(msg[j]!='=')
                        {
                            tmp[j-1]=msg[j];
                            j++;
                        }
                    }
                    selected_index=0;
                    for(int i=0;i<Device_pointer;++i)
                    {
                        int cnt=0,final_count=0;
                        int effective_cnt=0;
                        for(int l=0;l<sizeof(Device_name);++l)
                        {
                            if((tmp[l]==List_Topology[i][0][l])&&(tmp[l]!='\0')&&(List_Topology[i][0][l]!='\0'))
                                cnt++;
                            else final_count=cnt;
                            if((List_Topology[i][0][l]!='\0')&&(Device_name[l]!='\0'))effective_cnt++;
                        }
                        printf("%d,%d",effective_cnt,final_count);
                        if(final_count>=effective_cnt)
                        {
                            selected_index=i;
                            continue;
                        }   
                    }
                    //data1
                    int Integer_part=0,Decimal_part=0;
                    while((msg[j]<'0')||(msg[j]>'9'))j++;
                    while(msg[j]!='.')
                    {
                        Integer_part=Integer_part*10+msg[j]-'0';
                        //printf("%c,%d\n",msg[j],Integer_part);
                        j++;
                    }
                    j++;
                    while(msg[j]!='=')
                    {
                        Decimal_part=Decimal_part*10+msg[j]-'0';
                        j++;
                    }
                    //data2
                    j++;
                    int Integer_part2=0,Decimal_part2=0;
                    while((msg[j]<'0')||(msg[j]>'9'))j++;
                    while(msg[j]!='.')
                    {
                        Integer_part2=Integer_part2*10+msg[j]-'0';
                        //printf("%c,%d\n",msg[j],Integer_part2);
                        j++;
                    }
                    j++;
                    while(msg[j]!='=')
                    {
                        Decimal_part2=Decimal_part2*10+msg[j]-'0';
                        j++;
                    }
                    test_storage[selected_index][0]=(float)Integer_part+(float)Decimal_part/100.00;
                    test_storage[selected_index][1]=(float)Integer_part2+(float)Decimal_part2/100.00;
                    printf("Received data from %s(%s):%.2f,%.2f,selected index=%d\n",List_Topology[selected_index][0],List_Topology[selected_index][2],test_storage[selected_index][0],test_storage[selected_index][1],selected_index);
                    lv_event_send(mesh_data_table,LV_EVENT_REFRESH,0);
                    if(got_verification[selected_index]==1)got_verification[selected_index]=2;
                }
                else
                {
                //Split the elements in the messages
                    int Element_count=0,i=0,j=0,split_count=0;
                    memset(Device_name,'\0',sizeof(Device_name));
                    memset(Device_MAC,'\0',sizeof(Device_MAC));
                    memset(Function,'\0',sizeof(Function));
                    memset(Layer,'\0',sizeof(Layer));
                    memset(Parent_MAC,'\0',sizeof(Parent_MAC));
                    char* print=print_receive;
                    printf("Mesh topology:%s\n",print);
                    while (Element_count<5)
                    {
                        while(print[i++]!='='){}
                        split_count++;
                        if(split_count%2!=0)
                        {
                            if(Element_count==0)
                            {
                                j=0;
                                while(print[i]!='=')Device_name[j++]=print[i++];
                            }
                            if(Element_count==1)
                            {
                                j=0;
                                while(print[i]!='=')Device_MAC[j++]=print[i++];
                            }
                            if(Element_count==2)
                            {
                                j=0;
                                while(print[i]!='=')Function[j++]=print[i++];
                            }
                            if(Element_count==3)
                            {
                                j=0;
                                Layer[1]='\0';//strange bug
                                while(print[i]!='=')
                                {
                                    Layer[j++]=print[i++];
                                }
                            }
                            if(Element_count==4)
                            {
                                j=0;
                                while(print[i]!='=')
                                {
                                    Parent_MAC[j++]=print[i++];
                                }
                            }
                        }
                        else
                        {
                            Element_count++;
                        }
                    }
                    //printf("%s,%s,%s,%s,%s\n",Device_name,Device_MAC,Function,Layer,Parent_MAC);
                    int flag=0,duplication=0,cnt=0,final_count=0;
                    for(int k=0;k<Device_pointer;++k)//if not found, a new node enters; else, an old one updates
                    {
                        //printf("%d,%d,%s,%s\n",k,Device_pointer,Device_name,List_Topology[k][0]);
                        cnt=0;
                        int effective_cnt=0;
                        for(int l=0;l<sizeof(Device_name);++l)
                        {
                            if((Device_name[l]==List_Topology[k][0][l])&&(Device_name[l]!='\0')&&(List_Topology[k][0][l]!='\0'))
                                cnt++;
                            else final_count=cnt;
                            if((List_Topology[k][0][l]!='\0')||(Device_name[l]!='\0'))effective_cnt++;
                        }
                        if(final_count>=effective_cnt)
                        {
                            if(Device_name[effective_cnt]!='\0')flag=0;
                            else
                            {
                                flag=1;
                                duplication=k;
                                continue;
                            }
                        }    
                    }
                    //printf("flag=%d\n",flag);
                    if(flag==0)
                    {
                        printf("Registering new node, Device pointer=%d\n",Device_pointer);
                        strncpy(List_Topology[Device_pointer][0],Device_name,20);
                        strncpy(List_Topology[Device_pointer][1],Device_MAC,18);
                        if(Function[0]=='0')
                        {
                            List_Topology[Device_pointer][2][0]='L';
                            List_Topology[Device_pointer][2][1]='I';
                            List_Topology[Device_pointer][2][2]='G';
                            List_Topology[Device_pointer][2][3]='H';
                            List_Topology[Device_pointer][2][4]='T';     
                            Light_pointer=Device_pointer;              
                        }
                        if(Function[0]=='1')
                        {
                            List_Topology[Device_pointer][2][0]='T';
                            List_Topology[Device_pointer][2][1]='E';
                            List_Topology[Device_pointer][2][2]='M';
                            List_Topology[Device_pointer][2][3]='P';
                            List_Topology[Device_pointer][2][4]='_';   
                            List_Topology[Device_pointer][2][5]='H';
                            List_Topology[Device_pointer][2][6]='U';
                            List_Topology[Device_pointer][2][7]='M';
                            List_Topology[Device_pointer][2][8]='I';
                            Temp_pointer=Device_pointer;
                        }
                        if(Function[0]=='2')
                        {
                            List_Topology[Device_pointer][2][0]='S';
                            List_Topology[Device_pointer][2][1]='O';
                            List_Topology[Device_pointer][2][2]='I';
                            List_Topology[Device_pointer][2][3]='L';     
                            Soil_pointer=Device_pointer;        
                        }
                        if(Function[0]=='3')
                        {
                            List_Topology[Device_pointer][2][0]='R';
                            List_Topology[Device_pointer][2][1]='O';
                            List_Topology[Device_pointer][2][2]='O';
                            List_Topology[Device_pointer][2][3]='T';                
                        }
                        strncpy(List_Topology[Device_pointer][3],Layer,2);
                        strncpy(List_Topology[Device_pointer][5],Parent_MAC,18);
                        cnt=0;
                        for(int k=0;k<=Device_pointer;++k)//if the node can communicate with root its parent must have been in the list
                        {
                            cnt=0;
                            for(int l=0;l<18;++l)
                            {
                                if(Parent_MAC[l]==List_Topology[k][1][l])
                                    cnt++;
                            }
                            if(cnt>=17)
                            {
                                strncpy(List_Topology[Device_pointer][4],List_Topology[k][0],20);
                            }  
                        }
                        
                        //reply
                        if(got_verification[Device_pointer]==0)got_verification[Device_pointer]=1;
                        Device_pointer++;
                    }
                    else
                    {
                        //printf("Renewing old node,Device pointer=%d\n",Device_pointer);
                        //clean the old data first
                        memset(List_Topology[duplication][0],'\0',20);
                        memset(List_Topology[duplication][1],'\0',18);
                        memset(List_Topology[duplication][2],'\0',12);
                        memset(List_Topology[duplication][3],'\0',2);
                        memset(List_Topology[duplication][4],'\0',20);
                        memset(List_Topology[duplication][5],'\0',18);
                        strncpy(List_Topology[duplication][0],Device_name,20);
                        strncpy(List_Topology[duplication][1],Device_MAC,18);
                        if(Function[0]=='0')
                        {
                            List_Topology[duplication][2][0]='L';
                            List_Topology[duplication][2][1]='I';
                            List_Topology[duplication][2][2]='G';
                            List_Topology[duplication][2][3]='H';
                            List_Topology[duplication][2][4]='T';      
                            Light_pointer=duplication;             
                        }
                        if(Function[0]=='1')
                        {
                            List_Topology[duplication][2][0]='T';
                            List_Topology[duplication][2][1]='E';
                            List_Topology[duplication][2][2]='M';
                            List_Topology[duplication][2][3]='P';
                            List_Topology[duplication][2][4]='_';   
                            List_Topology[duplication][2][5]='H';
                            List_Topology[duplication][2][6]='U';
                            List_Topology[duplication][2][7]='M';
                            List_Topology[duplication][2][8]='I';
                            Temp_pointer=duplication;
                        }
                        if(Function[0]=='2')
                        {
                            List_Topology[duplication][2][0]='S';
                            List_Topology[duplication][2][1]='O';
                            List_Topology[duplication][2][2]='I';
                            List_Topology[duplication][2][3]='L';    
                            Soil_pointer=duplication;             
                        }
                        if(Function[0]=='3')
                        {
                            List_Topology[duplication][2][0]='R';
                            List_Topology[duplication][2][1]='O';
                            List_Topology[duplication][2][2]='O';
                            List_Topology[duplication][2][3]='T';                
                        }
                        strncpy(List_Topology[duplication][3],Layer,2);
                        strncpy(List_Topology[duplication][5],Parent_MAC,18);
                        cnt=0;
                        for(int k=0;k<=Device_pointer;++k)//if the node can communicate with root its parent must have been in the list
                        {
                            cnt=0;
                            for(int l=0;l<sizeof(Parent_MAC);++l){if(Parent_MAC[l]==List_Topology[k][1][l])cnt++;}
                            if(cnt>=17)
                            {
                                strncpy(List_Topology[duplication][4],List_Topology[k][0],20);
                            }
                        }
                    }
                    for(int k=0;k<=Device_pointer;++k)
                    {
                        printf("%d:%s\t%s\t%s\t%s\t%s\t%s\n",k,List_Topology[k][0],List_Topology[k][1],List_Topology[k][2],List_Topology[k][3],List_Topology[k][4],List_Topology[k][5]);
                    }
                    lv_event_send(mesh_topo_table,LV_EVENT_REFRESH,0);
                }
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
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
                 /*
        #if CONFIG_MESH_SET_ROOT
            esp_mesh_get_routing_table((mesh_addr_t *) &s_route_table, 8 * 8, &s_route_table_size);
            ESP_LOGI(MESH_TAG,"Root got child,%d",s_route_table_size);
            esp_err_t err;
            int tmp2=0,tmp0;
            mesh_data_t data;
            for(int i=0;i<s_route_table_size;++i)
            {
                if(MAC_ADDR_EQUAL(s_route_table[i].addr, child_connected->mac)){tmp2=i;break;}
                else{ESP_LOGI(MESH_TAG,"Child mac wrong, child="MACSTR", chosen mac="MACSTR"",MAC2STR(child_connected->mac),MAC2STR(s_route_table[i].addr));}
            }
            ESP_LOGI(MESH_TAG,"Child connected index:%d, "MACSTR", "MACSTR"",tmp2,MAC2STR(child_connected->mac),MAC2STR(s_route_table[tmp2].addr));
            //err=esp_mesh_recv(&s_route_table[tmp2],&data,portMAX_DELAY,&tmp0,NULL,0);
            //if(err==ESP_OK)ESP_LOGI(MESH_TAG,"Corresponding child function:%d,record=%d",data.data[0],record);
            //mesh_route_table_record[record]=s_route_table[tmp2];
            //functions[record]=data.data[0];
            //record++;
        #endif*/
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
        //open dhcpc for root(esp_netif_create_default_wifi_mesh_netifs()just open AP and STA,not dhcpc)
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        //mesh_netifs_start(esp_mesh_is_root());
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
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
    /*
#if !CONFIG_MESH_USE_GLOBAL_DNS_IP
    esp_netif_t *netif = event->esp_netif;
    esp_netif_dns_info_t dns;
    ESP_ERROR_CHECK(esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns));
    mesh_netif_start_root_ap(esp_mesh_is_root(), dns.ip.u_addr.ip4.addr);
#endif
*/
    //esp_mesh_comm_mqtt_task_start();
}
/*
Effects expected:
              Intial                           Wifi provision                     Show Mesh topology 
__________________________________   __________________________________   __________________________________
|IOT SYSTEM FOR SMART AGRICULTURE|   |IOT SYSTEM FOR SMART AGRICULTURE|   |IOT SYSTEM FOR SMART AGRICULTURE|
|Adjust by input|  |Mesh topology|   |Brightness       |Confirm Botton|   |_Mesh topology table_|Return btn|
|Device name| function |  value  |   |________________________________|   |Device|MAC|func|layer|parent|MAC|
|                                |   |Humidity                        |   |                                |
|                                |   |________________________________|   |                                |
|             Table              |   |                                |   |             Table              |
|                                |   |                                |   |                                |
|                                |   |            Keyboard            |   |                                |
|                                |   |                                |   |                                |
|________________________________|   |________________________________|   |________________________________|
mode=0:close;mode=1:show
esp_err_t esp_mesh_get_parent_bssid(mesh_addr_t *bssid)
int esp_mesh_get_max_layer(void)
int esp_mesh_get_layer(void)(called after finding parent)
int esp_mesh_get_total_node_num(void)
int esp_mesh_get_routing_table_size(void)
esp_err_t esp_mesh_get_routing_table(mesh_addr_t *mac, int len, int *size)
*/
void initial_display(int mode)
{
    if(mode==0)
    {
        lv_obj_add_flag(button_operation,LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(label_provision,LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(mesh_data_table,LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(button_topo,LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(label_topo,LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_flag(button_operation,LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(label_provision,LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(button_topo,LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(label_topo,LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(mesh_data_table,LV_OBJ_FLAG_HIDDEN);
    }
}
void provision_display(int mode)
{
    if(mode==0)
    {
        lv_obj_add_flag(Brightness_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(input_Brightness, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(Humidity_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(input_Humidity, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN); 
        lv_obj_add_flag(button_confirm, LV_OBJ_FLAG_HIDDEN); 
        lv_obj_add_flag(label_confirm, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_flag(Brightness_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(input_Brightness, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(Humidity_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(input_Humidity, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN); 
        lv_obj_clear_flag(button_confirm, LV_OBJ_FLAG_HIDDEN); 
        lv_obj_clear_flag(label_confirm, LV_OBJ_FLAG_HIDDEN);
    }
}
void topology_display(int mode)
{
    if(mode==0)
    {
        lv_obj_add_flag(mesh_topo_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(button_return, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(label_return, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(mesh_topo_table, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_flag(mesh_topo_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(button_return, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(label_return, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(mesh_topo_table, LV_OBJ_FLAG_HIDDEN);
    }
}
static void button_operation_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        //delete the info in NVS
        char Zero1[32];memset(Zero1,'\0',sizeof(Zero1));
        char Zero2[64];memset(Zero2,'\0',sizeof(Zero2));
        nvs_save_wifi_info(Zero1,Zero2);
        initial_display(0);
        provision_display(1);
        topology_display(0);
    }
}
static void input_Brightness_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(keyboard, input_Brightness);
    }
}
static void input_Humidity_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(keyboard, input_Humidity);
    }
}
static void button_confirm_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    esp_err_t err;
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    mesh_data_t data;
    data.data = tx_buf;
    data.size = 200;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    int router_table_light_pointer=0,router_table_soil_pointer=0;
    if(code == LV_EVENT_CLICKED) {
        esp_mesh_get_routing_table((mesh_addr_t *) &route_table,CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);
        uint8_t MAC_LIGHT[6],MAC_SOIL[6];
        int MAC_cnt=0;
        for(int k=0;k<Device_pointer;++k)
        {
            if(List_Topology[k][2][0]=='L')//Light
            {
                MAC_extraction(List_Topology[k][1],MAC_LIGHT);
                for (int i = 0; i < route_table_size; i++)
                {
                    MAC_cnt=0;
                    for(int j=0;j<6;++j)
                    {
                        if((route_table[i].addr)[j]==MAC_LIGHT[j])
                        {
                            MAC_cnt++;
                        }
                    }
                    if(MAC_cnt==6)
                    {
                        router_table_light_pointer=i;
                    }
                }
            }
            if(List_Topology[k][2][0]=='S')//Soil
            {
                MAC_extraction(List_Topology[k][1],MAC_SOIL);
                for (int i = 0; i < route_table_size; i++)
                {
                    MAC_cnt=0;
                    for(int j=0;j<6;++j)
                    {
                        if((route_table[i].addr)[j]==MAC_SOIL[j])
                        {
                            MAC_cnt++;
                        }
                    }
                    if(MAC_cnt==6)
                    {
                        router_table_soil_pointer=i;
                    }
                }
            }
        }
        char *print_sensor_data;
        char *print_sensor_data2;
        const char *record_Brightness;
        const char *record_Humidity;
        record_Brightness=lv_textarea_get_text(input_Brightness);
        record_Humidity=lv_textarea_get_text(input_Humidity);
        char record_Brightness2[4],record_Humidity2[5];
        strncpy(record_Brightness2,record_Brightness,3);
        strncpy(record_Humidity2,record_Humidity,4);
        record_Brightness2[3]='\0';record_Humidity2[4]='\0';
        ESP_LOGI(MESH_TAG,"Forced brightness:%s Forced humidity:%s\n",record_Brightness2,record_Humidity2);
        //format:Command=1=data(int)=-->size=7+1+2+4+1=15
        int force_brightness=0;
        int force_humidity=0;
        for(int i=0;i<3;++i)force_brightness=force_brightness*10+record_Brightness2[i]-'0';
        for(int i=0;i<4;++i)force_humidity=force_humidity*10+record_Humidity2[i]-'0';
        char format[16];
        strcpy(format,  "Command=0=%d=");
        asprintf(&print_sensor_data,format,force_brightness);
        for(int i=0;i<200;++i)
        {
            tx_buf[i]=0;
        }
        for(int i=0;i<16;++i)tx_buf[i]=(uint8_t)print_sensor_data[i];
        err = esp_mesh_send(&route_table[router_table_light_pointer], &data, MESH_DATA_P2P, NULL, 0);
        printf("Command to light complete\n");
        //mqtt_app_publish("/topic/ip_mesh/sensor_data", print_sensor_data);
        //format:Command=1=data(int)=-->size=7+1+2+4+1=15
        char format2[16];
        strcpy(format2,  "Command=2=%d=");
        asprintf(&print_sensor_data2,format2,force_humidity);
        for(int i=0;i<200;++i)
        {
            tx_buf[i]=0;
        }
        for(int i=0;i<16;++i)tx_buf[i]=(uint8_t)print_sensor_data2[i];
        err = esp_mesh_send(&route_table[router_table_soil_pointer], &data, MESH_DATA_P2P, NULL, 0);
        printf("Command to relay complete\n");
        //mqtt_app_publish("/topic/ip_mesh/sensor_data", print_sensor_data);
        //return to initial
        initial_display(1);
        provision_display(0);
        topology_display(0);
    }
}
static void button_topo_event_cb(lv_event_t * e)
{
     lv_event_code_t code = lv_event_get_code(e);
     if(code == LV_EVENT_CLICKED) {
        initial_display(0);
        provision_display(0);
        topology_display(1);
     }
}
static void button_return_event_cb(lv_event_t * e)
{
     lv_event_code_t code = lv_event_get_code(e);
     if(code == LV_EVENT_CLICKED) {
        initial_display(1);
        provision_display(0);
        topology_display(0);
     }
}
static void mesh_topo_refresh_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_REFRESH) {
    // Fill the following column
        for(int i=0;i<Device_pointer;++i)
        {
            lv_table_set_cell_value(mesh_topo_table,i+1, 0, List_Topology[i][0]);
            lv_table_set_cell_value(mesh_topo_table,i+1, 1, List_Topology[i][1]);
            lv_table_set_cell_value(mesh_topo_table,i+1, 2, List_Topology[i][2]);
            lv_table_set_cell_value(mesh_topo_table,i+1, 3, List_Topology[i][3]);
            lv_table_set_cell_value(mesh_topo_table,i+1, 4, List_Topology[i][4]);
            lv_table_set_cell_value(mesh_topo_table,i+1, 5, List_Topology[i][5]);
        }
    }
}
static void mesh_data_refresh_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_REFRESH) {
    // Fill the following column
        char tmp[10];
        snprintf(tmp,sizeof(tmp),"%.2f",test_storage[selected_index][0]);
        lv_table_set_cell_value(mesh_data_table,selected_index, 0, List_Topology[selected_index][0]);
        lv_table_set_cell_value(mesh_data_table,selected_index, 1, List_Topology[selected_index][2]);
        lv_table_set_cell_value(mesh_data_table,selected_index, 2, tmp);
        char tmp2[10];snprintf(tmp2,sizeof(tmp2),"%.2f",test_storage[selected_index][1]);
        lv_table_set_cell_value(mesh_data_table,selected_index, 3, tmp2);
    }
}
void lvgl_components_init(void)
{
    //background
    background=lv_obj_create(lv_scr_act());
    lv_obj_set_size(background, 480, 480);//full screen
    lv_obj_center(background);
    //title
    project_title=lv_label_create(background);
    lv_obj_set_size(project_title, 300, 46);
    lv_obj_align(project_title, LV_ALIGN_TOP_MID, 0,0);
    const char* content ="IOT SYSTEM FOR SMART AGRICULTURE";
    lv_label_set_text(project_title, content);
    //tabletton_operation
    int row=CONFIG_MESH_ROUTE_TABLE_SIZE+1;//maximum node number plus title
    mesh_data_table = lv_table_create(background);
    lv_obj_set_size(mesh_data_table, 460, 330);
    lv_obj_align(mesh_data_table,LV_ALIGN_BOTTOM_MID,10,0);
    lv_table_set_row_cnt(mesh_data_table,row);
    lv_table_set_col_cnt(mesh_data_table,4);	
    lv_obj_add_event_cb(mesh_data_table, mesh_data_refresh_cb, LV_EVENT_REFRESH, NULL);
    // Fill the first row
    lv_table_set_cell_value(mesh_data_table,0, 0, "Device Name");
    lv_table_set_cell_value(mesh_data_table,0, 1, "Function");
    lv_table_set_cell_value(mesh_data_table,0, 2, "Latest data");
    lv_table_set_cell_value(mesh_data_table,0, 3, "Latest data");

    //button_operation: Input SSID and password by hand
    button_operation=lv_btn_create(background);
    lv_obj_set_size(button_operation, 138, 46);
    lv_obj_align_to(button_operation,project_title,LV_ALIGN_OUT_BOTTOM_MID,-151,0);//place the button to the buttom of the title and move to the left
    label_provision = lv_label_create(button_operation);
    lv_label_set_text(label_provision, "Adjust by input");
    lv_obj_center(label_provision);
    lv_obj_add_event_cb(button_operation, button_operation_event_cb, LV_EVENT_ALL, NULL);
    //button_topo: SHow mesh topology by table
    button_topo=lv_btn_create(background);
    lv_obj_set_size(button_topo, 138, 46);
    lv_obj_align_to(button_topo,project_title,LV_ALIGN_OUT_BOTTOM_MID,161,0);//place the button to the buttom of the title and move to the right
    label_topo = lv_label_create(button_topo);
    lv_label_set_text(label_topo, "Show topology");
    lv_obj_center(label_topo);
    lv_obj_add_event_cb(button_topo, button_topo_event_cb, LV_EVENT_ALL, NULL);
    
    keyboard = lv_keyboard_create(background);
    lv_obj_set_align(keyboard, LV_ALIGN_BOTTOM_MID);
    lv_obj_set_size(keyboard, 440, 230);//occupy the bottom part of the screen
    lv_keyboard_set_mode(keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    //cache for input
    Brightness_title=lv_label_create(background);
    lv_obj_set_size(Brightness_title, 230, 40);
    lv_obj_align_to(Brightness_title, project_title, LV_ALIGN_OUT_BOTTOM_MID, -100,0);
    lv_label_set_text(Brightness_title, "Brightness(0-255):");
    input_Brightness = lv_textarea_create(background);
    lv_textarea_set_placeholder_text(input_Brightness, "Brightness");
    lv_obj_set_size(input_Brightness, 230, 40);
    lv_obj_align_to(input_Brightness, Brightness_title, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(input_Brightness, input_Brightness_event_cb, LV_EVENT_ALL, NULL);
    
    input_Humidity = lv_textarea_create(background);
    lv_textarea_set_placeholder_text(input_Humidity, "Humidity(0-4095)");
    lv_obj_set_size(input_Humidity, 430, 40);
    lv_obj_align_to(input_Humidity,  keyboard, LV_ALIGN_OUT_TOP_MID, 0,0);
    Humidity_title=lv_label_create(background);
    lv_obj_set_size(Humidity_title, 430, 40);
    lv_obj_align_to(Humidity_title, input_Humidity, LV_ALIGN_OUT_TOP_MID, 0,0);
    lv_label_set_text(Humidity_title, "Humidity:");
    lv_obj_add_event_cb(input_Humidity, input_Humidity_event_cb, LV_EVENT_ALL, NULL);
    lv_textarea_set_password_mode(input_Humidity,true);
    
    button_confirm=lv_btn_create(background);
    lv_obj_set_size(button_confirm, 138, 46);
    lv_obj_align_to(button_confirm,project_title,LV_ALIGN_OUT_BOTTOM_MID,141,0);
    label_confirm = lv_label_create(button_confirm);
    lv_label_set_text(label_confirm, "Confirm");
    lv_obj_center(label_confirm);
    lv_obj_add_event_cb(button_confirm, button_confirm_event_cb, LV_EVENT_ALL, NULL);
    //hide before the button is pressed
    provision_display(0);
    
    //Mesh topology related
    mesh_topo_title=lv_label_create(background);
    lv_obj_set_size(mesh_topo_title, 230, 23);
    lv_obj_align_to(mesh_topo_title, project_title,LV_ALIGN_OUT_BOTTOM_MID,-105,0);
    lv_label_set_text(mesh_topo_title, "Mesh topology table");
    
    button_return=lv_btn_create(background);
    lv_obj_set_size(button_return, 138, 23);
    lv_obj_align_to(button_return,project_title,LV_ALIGN_OUT_BOTTOM_MID,161,0);
    label_return = lv_label_create(button_return);
    lv_label_set_text(label_return, "Return");
    lv_obj_center(label_return);
    lv_obj_add_event_cb(button_return, button_return_event_cb, LV_EVENT_ALL, NULL);
    //table
    row=CONFIG_MESH_ROUTE_TABLE_SIZE+1;//maximum node number plus title
    mesh_topo_table = lv_table_create(background);
    lv_obj_set_size(mesh_topo_table, 460, 368);
    lv_obj_align(mesh_topo_table,LV_ALIGN_BOTTOM_MID,10,0);
    lv_table_set_row_cnt(mesh_topo_table,row);
    lv_table_set_col_cnt(mesh_topo_table,6);	
    lv_obj_add_event_cb(mesh_topo_table, mesh_topo_refresh_cb, LV_EVENT_REFRESH, NULL);
    // Fill the first row
    lv_table_set_cell_value(mesh_topo_table,0, 0, "Device Name");
    lv_table_set_cell_value(mesh_topo_table,0, 1, "MAC");
    lv_table_set_cell_value(mesh_topo_table,0, 2, "Function");
    lv_table_set_cell_value(mesh_topo_table,0, 3, "Layer No.");
    lv_table_set_cell_value(mesh_topo_table,0, 4, "Parent Name");
    lv_table_set_cell_value(mesh_topo_table,0, 5, "Parent MAC");

    topology_display(0);
}
/*UART*/
float app_get_current_temperature()
{
    return test_storage[Temp_pointer][0];
}
float app_get_current_humidity()
{
    return test_storage[Temp_pointer][1];
}
float app_get_current_soil()
{
    return test_storage[Soil_pointer][0];
}
float app_get_current_light()
{
    return test_storage[Light_pointer][0];
}
int sendData(const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(ECHO_UART_PORT_NUM, data, len);
    ESP_LOGI(UART_TAG, "Wrote %d bytes: %s", txBytes, data);
    return txBytes;
}
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
void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    float a=0,b=0,c=0,d=0;
    char* print_sensor_data;// =a=b=c=d= (1+6)*4+1=29
    char format[30];
    strcpy(format,  "=%.2f=%.2f=%.2f=%.2f=");
    
    while (1) {
        a=app_get_current_temperature();
        b=app_get_current_humidity();
        c=app_get_current_soil();
        d=app_get_current_light();
        asprintf(&print_sensor_data,format,a,b,c,d);
        sendData(print_sensor_data);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void locate_send_command(int function,int data_sent)
{
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    mesh_data_t data;
    data.data = tx_buf;
    data.size = 200;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    int router_table_light_pointer=0,router_table_soil_pointer=0;
    esp_mesh_get_routing_table((mesh_addr_t *) &route_table,CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);
    uint8_t MAC_LIGHT[6],MAC_SOIL[6];
    int MAC_cnt=0;
    if(function==0)
    {
        for(int k=0;k<Device_pointer;++k)
        {
            if(List_Topology[k][2][0]=='L')//Light
            {
                MAC_extraction(List_Topology[k][1],MAC_LIGHT);
                for (int i = 0; i < route_table_size; i++)
                {
                    MAC_cnt=0;
                    for(int j=0;j<6;++j)
                    {
                        if((route_table[i].addr)[j]==MAC_LIGHT[j])
                        {
                            MAC_cnt++;
                        }
                    }
                    if(MAC_cnt==6)
                    {
                        router_table_light_pointer=i;
                    }
                }
            }
        }
        char format[16];char* print_sensor_data_RM;
        strcpy(format,  "Command=0=%d=");
        asprintf(&print_sensor_data_RM,format,data_sent);
        for(int i=0;i<200;++i)
        {
            tx_buf[i]=0;
        }
        for(int i=0;i<16;++i)tx_buf[i]=(uint8_t)print_sensor_data_RM[i];
        esp_mesh_send(&route_table[router_table_light_pointer], &data, MESH_DATA_P2P, NULL, 0);
        printf("Command to light complete\n");
    }
    if(function==2)
    {
        for(int k=0;k<Device_pointer;++k)
        {
            if(List_Topology[k][2][0]=='S')//Soil
            {
                MAC_extraction(List_Topology[k][1],MAC_SOIL);
                for (int i = 0; i < route_table_size; i++)
                {
                    MAC_cnt=0;
                    for(int j=0;j<6;++j)
                    {
                        if((route_table[i].addr)[j]==MAC_SOIL[j])
                        {
                            MAC_cnt++;
                        }
                    }
                    if(MAC_cnt==6)
                    {
                        router_table_soil_pointer=i;
                    }
                }
            }
        }
        char format[16];char* print_sensor_data_RM;
        strcpy(format,  "Command=2=%d=");
        asprintf(&print_sensor_data_RM,format,data_sent);
        for(int i=0;i<200;++i)
        {
            tx_buf[i]=0;
        }
        for(int i=0;i<16;++i)tx_buf[i]=(uint8_t)print_sensor_data_RM[i];
        esp_mesh_send(&route_table[router_table_soil_pointer], &data, MESH_DATA_P2P, NULL, 0);
        printf("Command to relay complete\n");
    }
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
                        //=0=100=
                        printf("Command for fumnction:%c\n",uartbuf[1]);
                        if(uartbuf[1]=='0')
                        {
                            int i=3;
                            int force_light_by_RM=0;
                            float tmp=0;
                            while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                            {
                                tmp=tmp*10+uartbuf[i]-'0';
                                i=i+1;
                            }
                            tmp=tmp*2.55;
                            force_light_by_RM=(int)tmp;
                            locate_send_command(0,force_light_by_RM);
                        }
                        if(uartbuf[1]=='2')
                        {
                            int i=3;
                            int force_humidity_by_RM=0;
                            while((uartbuf[i]<='9')&&(uartbuf[i]>='0'))
                            {
                                force_humidity_by_RM=force_humidity_by_RM*10+uartbuf[i]-'0';
                                i=i+1;
                            }
                            locate_send_command(2,force_humidity_by_RM);
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
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //ESP_ERROR_CHECK(mesh_netifs_init(recv_cb));
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memset(cfg.router.ssid,'\0',32); memset(cfg.router.password,'\0',64); memset(cfg.mesh_ap.password,'\0',64);
    memset(sta_config.sta.ssid,'\0',32); memset(sta_config.sta.password,'\0',64);
#if CONFIG_MESH_IE_CRYPTO_FUNCS
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_funcs(&g_wifi_default_mesh_crypto_funcs));
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_key(CONFIG_MESH_IE_CRYPTO_KEY, strlen(CONFIG_MESH_IE_CRYPTO_KEY)));
#else
    ESP_LOGI(MESH_TAG, "<Config>disable IE crypto");
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_funcs(NULL));
#endif
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);

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
        while((SSID_got==false)||(password_got==false))
        {printf("%d,%d\t",SSID_got,password_got);
        }//wait for data
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
/*
    char* read_SSID="SZMiPhone";
    char* read_passwd="1029384756";
    if(strlen(read_SSID)!=0)
    {
        cfg.router.ssid_len = strlen(read_SSID);
        memcpy((uint8_t *) &cfg.router.ssid, read_SSID, cfg.router.ssid_len);
        cfg.router.ssid[cfg.router.ssid_len+1]='\0';
    }
    if(strlen(read_passwd)!=0)
    {
        memcpy((uint8_t *) &cfg.router.password,read_passwd,strlen(read_passwd));
        cfg.router.password[strlen(read_passwd)+1]='\0';
    }
*/
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,strlen(CONFIG_MESH_AP_PASSWD));
    cfg.mesh_ap.password[strlen(CONFIG_MESH_AP_PASSWD)/sizeof(CONFIG_MESH_AP_PASSWD[0])]='\0';
    printf("%s,%s,%s,%c",(char*)cfg.router.ssid,(char*)cfg.router.password,(char*)cfg.mesh_ap.password,cfg.mesh_ap.password[strlen(CONFIG_MESH_AP_PASSWD)/sizeof(CONFIG_MESH_AP_PASSWD[0])]);
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    ESP_ERROR_CHECK(esp_mesh_start());
    uart_init();
    xTaskCreate(tx_task, "uart_tx_task", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    bsp_i2c_init();
    bsp_display_start();
    /* Set display brightness to 100% */
    bsp_display_backlight_on();
    //ESP_LOGI(MESH_TAG, "Display my simplist LVGL demo");
    bsp_display_lock(0);
    lvgl_components_init();
    bsp_display_unlock();
}
