#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#define MAX_SSID_LEN 32
#define MAX_PASSWD_LEN  64
 
 
typedef struct 
{
    int ssid_len;
    int passwd_len;
    char setting_ssid[MAX_SSID_LEN];
    char setting_passwd[MAX_PASSWD_LEN];
}ROUTER_DATA_T;

void mynvs_init(void);
//input: char *a,*b
//char a[32],b[64] is not allowed
int nvs_read_wifi_info(char** SSID,char** passwd);
//input: char SSID[32];memset(SSID,'\0',sizeof(SSID));strcpy(SSID,"SZMiPhone");  
int nvs_save_wifi_info(char* SSID,char* passwd);

int nvs_read_start_flag(void);
void nvs_save_start_flag(void);
void nvs_clear_start_flag(void);