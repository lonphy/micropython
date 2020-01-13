/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "uart.h"
#include "py/ringbuf.h"

#ifndef MICROPY_INCLUDED_STM32_ESP826601_LL_H
#define MICROPY_INCLUDED_STM32_ESP826601_LL_H

enum {
    ESP826601_ITF_STA,
    ESP826601_ITF_AP,
};

enum {
    esp826601_state_off,
    esp826601_state_on,
};

#define _ESP826601_SOCK_NUM_ (5)
#define ESP_LINK_Mask        ((1 << _ESP826601_SOCK_NUM_) - 1)

#define ESP_TX_BUF_SIZE (192)
#define ESP_RX_BUF_SIZE (2048)

#define rsp_status_sending 3
#define rsp_status_noack 2
#define rsp_status_pending 1
#define rsp_status_success 0
#define rsp_status_error -1

typedef struct _esp_t {
    /* AT命令处理状态 */
    volatile uint8_t tx_acked_status;

    /**
     * 记录连接ID使用情况, [bit0,bit4] -> [link#0, link#4]
     */
    uint8_t link_used;

    /*
        数据包读取状态
        0 - 未开始
        1 - 读取数据长度中
        2 - 读取剩余数据中
        3 - 数据已全部读取
    */
    uint8_t data_rx_status;

    /* 当前正在接收数据的link_id */
    uint8_t worker_link_id;

    /* 当前数据包剩余长度 */
    uint16_t pkt_remaining_bytes;

    size_t rx_len;
    char rx_buf[ESP_RX_BUF_SIZE];

    /* 底层uart对象 */
    machine_uart_obj_t bus;

    /* 响应数据缓冲 */
    uint8_t ack_buf[ESP_RX_BUF_SIZE];

    /* AT命令数据缓冲 */
    uint8_t cmd_buf[ESP_TX_BUF_SIZE];

    /* 读取到的+IPD数据，存放在这里
        (检查对应link是否打开， 没打开就扔掉(目前还没实现))
        这里放置一个缓冲, 收到的数据全缓存起来，如果放满(例如上层来不及读取):
        循环覆盖， 并设置数据已丢失标记, 即数据过载
    */
    ringbuf_t tcp_rx_buf[_ESP826601_SOCK_NUM_ - 1];
    
    /* 维护每个链接的接收缓冲 */
    /* uint16_t tcp_buf_len[_ESP826601_SOCK_NUM_ - 1]; */

} esp_t;

// typedef struct _esp826601_ev_scan_result_t {
//     uint32_t _0[5];
//     uint8_t bssid[6];
//     uint16_t _1[2];
//     uint8_t ssid_len;
//     uint8_t ssid[32];
//     uint32_t _2[5];
//     uint16_t channel;
//     uint16_t _3;
//     uint8_t auth_mode;
//     int16_t rssi;
// } esp826601_ev_scan_result_t;

// typedef struct _esp826601_async_event_t {
//     uint16_t _0;
//     uint16_t flags;
//     uint32_t event_type;
//     uint32_t status;
//     uint32_t reason;
//     uint8_t _1[30];
//     uint8_t interface;
//     uint8_t _2;
//     union {
//         esp826601_ev_scan_result_t scan_result;
//     } u;
// } esp826601_async_event_t;

// typedef struct _esp826601_wifi_scan_options_t {
//     uint32_t version;
//     uint16_t action;
//     uint16_t _;
//     uint32_t ssid_len; // 0 to select all
//     uint8_t ssid[32];
//     uint8_t bssid[6];
//     int8_t bss_type; // fill with 0xff to select all
//     int8_t scan_type; // 0=active, 1=passive
//     int32_t nprobes;
//     int32_t active_time;
//     int32_t passive_time;
//     int32_t home_time;
//     int32_t channel_num;
//     uint16_t channel_list[1];
// } esp826601_wifi_scan_options_t;

typedef struct _esp826601_ip_info_t {
    mp_obj_t ip;
    mp_obj_t netmask;
    mp_obj_t gw;
} esp826601_info_t;

// ---------- error code ------------------
#define ESP_SUCCESS 0
#define ESP_FAIL -1
#define ESP_ERROR ESP_FAIL

#define ESP_AT_END "\r\n"
#define ESP_AT(__cmd__) ("AT+" __cmd__ ESP_AT_END)

#define esp_is_link_use(esp, link_id)  ((esp)->link_used & (1 << (link_id)))
#define esp_link_use(esp, link_id)     ((esp)->link_used |= (1 << (link_id)))
#define esp_link_free(esp, link_id)    ((esp)->link_used &= ~(1 << (link_id)))

static inline int8_t esp_wait_state(esp_t *esp, int8_t state) {
    esp->tx_acked_status = state;
    while (esp->tx_acked_status == state) {
        MICROPY_EVENT_POLL_HOOK
    }
    return esp->tx_acked_status;
}

// 创建esp对象
esp_t* esp_create(uint32_t port, uint baudrate);

// 执行初始化序列
int8_t esp_ctrl_init(esp_t* esp);

// 连接到指定AP
int8_t esp_ctrl_connect(esp_t* esp, const char* ssid, size_t ssid_len, const char* bssid, const char* pwd, size_t pwd_len);

// 断开当前连接AP
int8_t esp_ctrl_disconnect(esp_t* esp);

// 查询是否连接到AP
bool esp_ctrl_is_connect(esp_t* esp);

// 查询当前STATION IP信息
int8_t esp_ctrl_qry_sta_ip(esp_t* esp, esp826601_info_t* info);

// 设置当前STATION IP信息(静态)
int8_t esp_ctrl_set_static_ip(esp_t* esp, const esp826601_info_t* info);

// 查询当前DNS设置
int8_t esp_ctrl_qry_dns(esp_t* esp, char* dns, uint8_t* size);

// 设置当前STATION DNS信息(静态)
int8_t esp_ctrl_set_dns(esp_t* esp, const char* dns);

// DNS解析(IPv4)
int8_t esp_ctrl_cip_domain(esp_t* esp, const char* domain, uint8_t* n, char* ip);

#define esp_tx_strn(esp, data, len) uart_tx_strn(&((esp)->bus), (const char*)data, len)

#endif // MICROPY_INCLUDED_STM32_ESP826601_LL_H
