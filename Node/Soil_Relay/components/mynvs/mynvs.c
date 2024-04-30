#include "mynvs.h"
static const char* NVS_TAG = "myNVS";
void mynvs_init(void){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}


int nvs_read_wifi_info(char** SSID,char** passwd){

    esp_err_t err;
    nvs_handle_t nvs_handle;
    err = nvs_open("storage",NVS_READWRITE,&nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(NVS_TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return -1;
    }
    size_t SSIDlen=0,passwdlen=0;
    char inside_SSID[32];char inside_passwd[64];
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle,"SSID",NULL,&SSIDlen));
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle,"passwd",NULL,&passwdlen));
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle,"SSID",inside_SSID,&SSIDlen));
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle,"passwd",inside_passwd,&passwdlen));
    ESP_LOGI(NVS_TAG,"SSID=%s,Password=%s",inside_SSID,inside_passwd);
    nvs_close(nvs_handle);
    *SSID=malloc(SSIDlen);*passwd=malloc(passwdlen);
    strcpy(*SSID,inside_SSID);strcpy(*passwd,inside_passwd);
    printf("Output:%s,%s\n",*SSID,*passwd);
    return 1;
}

int nvs_save_wifi_info(char* SSID,char* passwd){
    esp_err_t err;
    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(NVS_TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return -2;
    }
    printf("Input:%s,%s\n",SSID,passwd);
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "SSID", SSID));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "passwd",passwd));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
    return 1;
    //testing
    //char SSID_test[32];char passwd_test[64];
    //return(nvs_read_wifi_info(&SSID_test,&passwd_test));
}
int nvs_read_start_flag(void)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;
    err = nvs_open("storage",NVS_READWRITE,&nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(NVS_TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return -1;
    }
    int32_t start_flag = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(nvs_handle, "start_flag", &start_flag);
    nvs_close(nvs_handle);
    int i=-1;
    switch (err) {
        case ESP_OK:
            i=start_flag;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            i=0;
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
    return i;
}
void nvs_save_start_flag(void)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(NVS_TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    int32_t start_flag=1;
    err = nvs_set_i32(nvs_handle, "start_flag", start_flag);
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}
void nvs_clear_start_flag(void)
{
    if(nvs_read_start_flag()!=0)
    {
        esp_err_t err;
        nvs_handle_t nvs_handle;
        err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGI(NVS_TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        }
        int32_t start_flag=0;
        err = nvs_set_i32(nvs_handle, "start_flag", start_flag);
        err = nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
}