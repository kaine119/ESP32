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

#ifndef DB_ESP32_GLOBALS_H
#define DB_ESP32_GLOBALS_H

#include <freertos/event_groups.h>

#define MAX_LTM_FRAMES_IN_BUFFER 5
#define BUILDVERSION 7
#define MAJOR_VERSION 1
#define MINOR_VERSION 0

// can be set by user
extern uint8_t DB_WIFI_MODE;            // 0 = AP mode, 1 = STA mode
extern uint8_t AP_MODE_SSID[32];
extern uint8_t AP_MODE_PWD[64];
extern char AP_IP_ADDR[32];
extern uint8_t STA_MODE_SSID[32];
extern uint8_t STA_MODE_PWD[64];
extern char STA_IP_ADDR[32];
extern char STA_IP_MASK[32];
extern char STA_IP_GATEWAY[32];
extern uint8_t STA_DHCP_MODE;           // 0 = Use static IP, 1 = Use DHCP
extern uint8_t DEFAULT_CHANNEL;
extern uint8_t SERIAL_PROTOCOL;         // 1=MSP, 3=MAVLink/transparent
extern uint8_t DB_UART_PIN_TX;
extern uint8_t DB_UART_PIN_RX;
extern int DB_UART_BAUD_RATE;
extern uint16_t TRANSPARENT_BUF_SIZE;
extern uint8_t LTM_FRAME_NUM_BUFFER;    // Number of LTM frames per UDP packet (min = 1; max = 5)
extern uint8_t MSP_LTM_SAMEPORT;        // 0 = no (1607 MSP, 1604 LTM); >0 = yes (1604)
extern uint16_t APP_PORT_PROXY_UDP;

extern uint32_t uart_byte_count;
extern int8_t num_connected_tcp_clients;
extern int8_t num_connected_udp_clients;

#endif //DB_ESP32_GLOBALS_H
