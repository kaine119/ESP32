/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2018 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

#include <stdio.h>
#include <nvs_flash.h>
#include <esp_wifi_types.h>
#include <mdns.h>
#include <string.h>
#include <driver/gpio.h>
#include <lwip/apps/netbiosns.h>
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "db_esp32_control.h"
#include "http_server.h"
#include "db_esp32_comm.h"
#include "db_protocol.h"
#include "esp_vfs_semihost.h"
#include "esp_spiffs.h"
#include "globals.h"
#include "http_server.h"
#include "main.h"

#define NVS_NAMESPACE "settings"
#define USE_ALT_UART_CONFIG // for boards that have flash connected to GPIO 17/16 - will crash otherwise

static const char *TAG = "DB_ESP32";

uint8_t DB_WIFI_MODE = 0; // 0 = AP mode, 1 = STA mode
uint8_t AP_MODE_SSID[32] = "DroneBridge ESP32";
uint8_t AP_MODE_PWD[64] = "dronebridge";
uint8_t STA_MODE_SSID[32] = "";
uint8_t STA_MODE_PWD[64] = "";
char DEFAULT_AP_IP[32] = "192.168.2.1";
uint8_t DEFAULT_CHANNEL = 6;
uint8_t SERIAL_PROTOCOL = 4;  // 1=MSP, 4=MAVLink/transparent
# ifdef USE_ALT_UART_CONFIG
uint8_t DB_UART_PIN_TX = GPIO_NUM_21;
uint8_t DB_UART_PIN_RX = GPIO_NUM_20;
# else
uint8_t DB_UART_PIN_TX = GPIO_NUM_5;
uint8_t DB_UART_PIN_RX = GPIO_NUM_4;
#endif
int DB_UART_BAUD_RATE = 921600;
uint16_t TRANSPARENT_BUF_SIZE = 64;
uint8_t LTM_FRAME_NUM_BUFFER = 1;
uint8_t MSP_LTM_SAMEPORT = 0;
uint8_t CONNECTED_FLAG = 0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    switch (event_id) {
    // AP mode handlers
    case WIFI_EVENT_AP_STACONNECTED:
        {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
            ESP_LOGI(TAG, "Client connected - station:"MACSTR", AID=%d", MAC2STR(event->mac), event->aid);
            break;  
        }
    case WIFI_EVENT_AP_STADISCONNECTED:
        {    
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
            ESP_LOGI(TAG, "Client disconnected - station:"MACSTR", AID=%d", MAC2STR(event->mac), event->aid);
            break;
        }    
    case WIFI_EVENT_AP_START:
        ESP_LOGI(TAG, "AP started!");
        break;
    case WIFI_EVENT_AP_STOP:
        ESP_LOGI(TAG, "AP stopped!");
        break;

    // STA mode handlers
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "STA started, attempting to connect to %s:%s", STA_MODE_SSID, STA_MODE_PWD);
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "STA connected");
        CONNECTED_FLAG = 1;
        break;
    }
}

void start_mdns_service() {
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }
    ESP_ERROR_CHECK(mdns_hostname_set("dronebridge"));
    ESP_ERROR_CHECK(mdns_instance_name_set("DroneBridge for ESP32"));

    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_db_proxy", "_tcp", APP_PORT_PROXY, NULL, 0));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_db_comm", "_tcp", APP_PORT_COMM, NULL, 0));
    ESP_ERROR_CHECK(mdns_service_instance_name_set("_http", "_tcp", "DroneBridge for ESP32"));
}

#if CONFIG_WEB_DEPLOY_SEMIHOST
esp_err_t init_fs(void) {
    esp_err_t ret = esp_vfs_semihost_register(CONFIG_WEB_MOUNT_POINT, CONFIG_HOST_PATH_TO_MOUNT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register semihost driver (%s)!", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    return ESP_OK;
}
#endif


#if CONFIG_WEB_DEPLOY_SF

esp_err_t init_fs(void) {
    esp_vfs_spiffs_conf_t conf = {
            .base_path = CONFIG_WEB_MOUNT_POINT,
            .partition_label = NULL,
            .max_files = 5,
            .format_if_mount_failed = false
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

#endif

// If we haven't connected to a network in 90 seconds,
// there's probably something wrong with our config.
// Reboot into AP mode to allow reconfiguration.
void sta_timeout(TimerHandle_t xTimer) {
    // vTaskDelay(10 * 1000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "STA timeout reached");
    if (CONNECTED_FLAG == 0) {
        ESP_LOGI(TAG, "STA could not connect, rebooting to AP...");
        DB_WIFI_MODE = 0;
        write_settings_to_nvs();
        esp_restart();
    } else {
        ESP_LOGI(TAG, "STA connected, no action taken");
    }
}

void init_wifi(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize wifi    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register WiFi event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    esp_netif_t *esp_net;

    if (DB_WIFI_MODE == 0) {
        // Configure AP
        esp_net = esp_netif_create_default_wifi_ap();
        wifi_config_t wifi_config = {
                .ap = {
                        .ssid = "DroneBridge_ESP32_Init",
                        .ssid_len = 0,
                        .authmode = WIFI_AUTH_WPA_PSK,
                        .channel = DEFAULT_CHANNEL,
                        .ssid_hidden = 0,
                        .beacon_interval = 100,
                        .max_connection = 10
                },
        };
        memcpy(wifi_config.ap.ssid, AP_MODE_SSID, 32);
        memcpy(wifi_config.ap.password, AP_MODE_PWD, 64);

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_set_country_code("01", true));
        ESP_ERROR_CHECK(esp_wifi_start());

        // Setup DHCP server for AP mode
        esp_netif_ip_info_t ip;
        memset(&ip, 0, sizeof(esp_netif_ip_info_t));
        ip.ip.addr = ipaddr_addr(DEFAULT_AP_IP);
        ip.netmask.addr = ipaddr_addr("255.255.255.0");
        ip.gw.addr = ipaddr_addr(DEFAULT_AP_IP);
        ESP_ERROR_CHECK(esp_netif_dhcps_stop(esp_net));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_net, &ip));
        ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_net));
    } else {
        esp_net = esp_netif_create_default_wifi_sta();
        // Configure STA
        wifi_config_t wifi_config = {
                .sta = {
                        .ssid = "",
                        .password = "",
                },
        };
        memcpy(wifi_config.sta.ssid, STA_MODE_SSID, 32);
        memcpy(wifi_config.sta.password, STA_MODE_PWD, 64);
        esp_netif_ip_info_t ip_info;
        memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));
        ip_info.ip.addr = ipaddr_addr("192.168.1.120");
        ip_info.netmask.addr = ipaddr_addr("255.255.255.0");
        ip_info.gw.addr = ipaddr_addr("192.168.1.120");
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(esp_net));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_net, &ip_info));

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        // Station mode timeout
        TimerHandle_t timer = xTimerCreate("reboot_into_ap", pdMS_TO_TICKS(90 * 1000), pdFALSE, 0, &sta_timeout);
        xTimerStart(timer, 0);
    }

    ESP_ERROR_CHECK(esp_netif_set_hostname(esp_net, "DBESP32"));
}

void write_settings_to_nvs() {
    ESP_LOGI(TAG,
             "Trying to save: ssid %s\nwifi_pass %s\nwifi_chan %i\nbaud %i\ngpio_tx %i\ngpio_rx %i\nproto %i\n"
             "trans_pack_size %i\nltm_per_packet %i\nmsp_ltm %i\nap_ip %s",
             AP_MODE_SSID, AP_MODE_PWD, DEFAULT_CHANNEL, DB_UART_BAUD_RATE, DB_UART_PIN_TX, DB_UART_PIN_RX,
             SERIAL_PROTOCOL, TRANSPARENT_BUF_SIZE, LTM_FRAME_NUM_BUFFER, MSP_LTM_SAMEPORT, DEFAULT_AP_IP);
    ESP_LOGI(TAG, "Saving to NVS %s", NVS_NAMESPACE);
    nvs_handle my_handle;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ap_ssid", (char *) AP_MODE_SSID));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ap_pass", (char *) AP_MODE_PWD));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "sta_ssid", (char *) STA_MODE_SSID));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "sta_pass", (char *) STA_MODE_PWD));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "wifi_chan", DEFAULT_CHANNEL));
    ESP_ERROR_CHECK(nvs_set_i32(my_handle, "baud", DB_UART_BAUD_RATE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_tx", DB_UART_PIN_TX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "gpio_rx", DB_UART_PIN_RX));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "proto", SERIAL_PROTOCOL));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "trans_pack_size", TRANSPARENT_BUF_SIZE));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "ltm_per_packet", LTM_FRAME_NUM_BUFFER));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "msp_ltm", MSP_LTM_SAMEPORT));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ap_ip", DEFAULT_AP_IP));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "wifi_mode", DB_WIFI_MODE));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
}


void read_settings_nvs() {
    nvs_handle my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) {
        // First start
        nvs_close(my_handle);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        write_settings_to_nvs();
    } else {
        ESP_LOGI(TAG, "Reading settings from NVS");
        size_t required_size = 0;
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ap_ssid", NULL, &required_size));
        char *ap_ssid = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ap_ssid", ap_ssid, &required_size));
        memcpy(AP_MODE_SSID, ap_ssid, required_size);

        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ap_pass", NULL, &required_size));
        char *ap_pass = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ap_pass", ap_pass, &required_size));
        memcpy(AP_MODE_PWD, ap_pass, required_size);

        ESP_ERROR_CHECK(nvs_get_str(my_handle, "sta_ssid", NULL, &required_size));
        char *sta_ssid = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "sta_ssid", sta_ssid, &required_size));
        memcpy(STA_MODE_SSID, sta_ssid, required_size);

        ESP_ERROR_CHECK(nvs_get_str(my_handle, "sta_pass", NULL, &required_size));
        char *sta_pass = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "sta_pass", sta_pass, &required_size));
        memcpy(STA_MODE_PWD, sta_pass, required_size);

        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ap_ip", NULL, &required_size));
        char *ap_ip = malloc(required_size);
        ESP_ERROR_CHECK(nvs_get_str(my_handle, "ap_ip", ap_ip, &required_size));
        memcpy(DEFAULT_AP_IP, ap_ip, required_size);

        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "wifi_chan", &DEFAULT_CHANNEL));
        ESP_ERROR_CHECK(nvs_get_i32(my_handle, "baud", &DB_UART_BAUD_RATE));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "gpio_tx", &DB_UART_PIN_TX));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "gpio_rx", &DB_UART_PIN_RX));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "proto", &SERIAL_PROTOCOL));
        ESP_ERROR_CHECK(nvs_get_u16(my_handle, "trans_pack_size", &TRANSPARENT_BUF_SIZE));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "ltm_per_packet", &LTM_FRAME_NUM_BUFFER));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "msp_ltm", &MSP_LTM_SAMEPORT));
        ESP_ERROR_CHECK(nvs_get_u8(my_handle, "wifi_mode", &DB_WIFI_MODE));
        nvs_close(my_handle);
        free(ap_ssid);
        free(ap_pass);
        free(sta_ssid);
        free(sta_pass);
        free(ap_ip);
    }
}

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    read_settings_nvs();
    esp_log_level_set("*", ESP_LOG_INFO);
    init_wifi();
    start_mdns_service();
    netbiosns_init();
    netbiosns_set_name("dronebridge");

    ESP_ERROR_CHECK(init_fs());

    control_module();
    ESP_ERROR_CHECK(start_rest_server(CONFIG_WEB_MOUNT_POINT));
    communication_module();
}