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
#ifndef MICROPY_INCLUDED_STM32_ESP826601_H
#define MICROPY_INCLUDED_STM32_ESP826601_H

typedef enum {
    SO_FLAG,
    SO_TTL,
    SO_TOS,
    SO_MSS,
    SO_DESTIP,
    SO_DESTPORT,
    SO_SENDBUF, /* AT+CPIBUFSTATUS: 查询TCP发包缓冲 */
    SO_RECVBUF,
    SO_STATUS,
    SO_REMAINSIZE,
    SO_PACKINFO,
} sockopt_type;

#include "esp826601_ll.h"

//------------ Socket Types ------------
enum {
    ESP_TCP,
    ESP_UDP,
    ESP_SSL,
};


/**
 * 打开ip连接
 * @param link_id  连接id
 * @param type 连接类型 ESP_TCP | ESP_UDP
 * @param ip 连接的IP
 * @param port 连接端口
 * @param keep_alive keep_alive探测时间, 0: 关闭
 */
int8_t
esp_cip_start(esp_t* esp, uint8_t link_id, uint8_t type, const char* ip, uint16_t port, uint16_t keep_alive);

/**
 * 关闭ip连接
 * @param link_id 连接id
 */
int8_t esp_cip_close(esp_t* esp, uint8_t link_id);

// 打开或关闭TCP服务器
// port 是要监听的端口
int8_t esp_cip_server(esp_t* esp, uint8_t mode, uint16_t port);

// TCP发送数据
int8_t esp_cip_send(esp_t* esp, uint8_t link_id, uint8_t* data, size_t len);

// TCP接收数据
int8_t esp_cip_recv(esp_t* esp, uint8_t link_id, uint8_t* data, uint16_t len);

#endif // MICROPY_INCLUDED_STM32_ESP826601_H
