/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Damien P. George
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "py/mperrno.h"
#include "py/mphal.h"
#include "py/objlist.h"
#include "py/runtime.h"
#include "py/stream.h"

#if MICROPY_PY_ESP826601
#include "modnetwork.h"
#include "pendsv.h"
#include "systick.h"
#include "uart.h"

#include "drivers/esp826601/esp826601.h"
#include "lib/netutils/netutils.h"

// Hook for pendsv poller to run this periodically every 512ms
#define ESP_TICK(tick) (((tick) & ~(SYSTICK_DISPATCH_NUM_SLOTS - 1) & 0x1ff) == 0)

extern int mp_nw_esp_state;

typedef struct _esp826601_obj_t {
    mp_obj_base_t base;
    uint16_t tcp_listen_port; // tcp 监听端口

    esp_t* esp;
} esp826601_obj_t;

STATIC esp826601_obj_t esp826601_obj;

typedef int (*hal_uart_rx_cb_t)(void* arg, uint8_t data);

extern void esp_uart_rx_poll(esp_t* esp);

void esp826601_poll(void)
{
    if (mp_nw_esp_state == esp826601_state_on) {
        esp_uart_rx_poll(esp826601_obj.esp);
    }

    // os_callout_process();
    // os_eventq_run_all();
}

// 基于SysTick轮询
void mod_network_esp826601_poll_wrapper(uint32_t ticks_ms)
{
    if (ESP_TICK(ticks_ms)) {
        pendsv_schedule_dispatch(PENDSV_DISPATCH_ESP826601, esp826601_poll);
    }
}

// 域名解析
STATIC int esp826601_gethostbyname(mp_obj_t nic, const char* name, mp_uint_t len, uint8_t* out_ip)
{
    if (len > 64) {
        nlr_raise(
            mp_obj_new_exception_msg_varg(&mp_type_OSError,
                "maximum length of the domain name is 64\n"));
    }
    char buf[sizeof("255.255.255.255")] = { 0 };
    uint8_t n = 0;

    // FIXME: 底层返回的IP是字符串, 这里需要转成数字数组, 实际用的时候， 又需要把数字数组转成字符串， 蛋疼
    // 方案: 在这一层统一进行转换， 底层只处理字符串
    int8_t ret = esp_ctrl_cip_domain(esp826601_obj.esp, name, &n, buf);
    if (ret != 0 || n == 0 || n > 16) {
        nlr_raise(mp_obj_new_exception_msg_varg(
            &mp_type_OSError,
            "could not get host\n"));
    }
    mp_obj_t str = mp_obj_new_str(buf, n);
    netutils_parse_ipv4_addr(str, out_ip, NETUTILS_BIG);
    return 0;
}

// 分配socket连接
STATIC int esp826601_socket_socket(mod_network_socket_obj_t* socket, int* _errno)
{
    if (socket->u_param.domain != MOD_NETWORK_AF_INET) {
        *_errno = MP_EAFNOSUPPORT;
        return -1;
    }

    switch (socket->u_param.type) {
    case MOD_NETWORK_SOCK_STREAM:
        socket->u_param.type = ESP_TCP;
        break;
    case MOD_NETWORK_SOCK_DGRAM:
        socket->u_param.type = ESP_UDP;
        break;
    default:
        *_errno = MP_EINVAL;
        return -1;
    }

    if (socket->u_param.fileno == -1) {
        esp826601_obj_t* nic = socket->nic;
        // get first unused socket number
        for (mp_uint_t link_id = 0; link_id < _ESP826601_SOCK_NUM_; link_id++) {
            if (!esp_is_link_use(nic->esp, link_id)) {
                esp_link_use(nic->esp, link_id);
                socket->u_param.fileno = link_id;
                break;
            }
        }
        if (socket->u_param.fileno == -1) {
            // too many open sockets
            *_errno = MP_EMFILE;
            return -1;
        }
    }
    socket->u_param.domain = 0;
    return 0;
}

// 关闭socket连接
STATIC void esp826601_socket_close(mod_network_socket_obj_t* socket)
{
    esp826601_obj_t* nic = socket->nic;
    uint8_t link_id = (uint8_t)socket->u_param.fileno;
    if (link_id < _ESP826601_SOCK_NUM_) {
        esp_cip_close(nic->esp, link_id);
    }
}

STATIC int esp826601_socket_bind(mod_network_socket_obj_t* socket, byte* ip, mp_uint_t port, int* _errno)
{
    esp826601_obj.tcp_listen_port = port;
    return 0;
}

// 开始监听TCP
STATIC int esp826601_socket_listen(mod_network_socket_obj_t* socket, mp_int_t backlog, int* _errno)
{
    esp826601_obj_t* nic = socket->nic;
    // backlog 在这里无效， 底层最多接受4个连接
    mp_int_t ret = esp_cip_server(nic->esp, 1, nic->tcp_listen_port);
    if (ret < 0) {
        esp826601_socket_close(socket);
        *_errno = -ret;
        return -1;
    }

    // indicate that this socket has been opened
    socket->u_param.domain = 1;
    return 0;
}

STATIC int esp826601_socket_accept(mod_network_socket_obj_t* socket, mod_network_socket_obj_t* socket2, byte* ip, mp_uint_t* port, int* _errno)
{
    return -2;
}

// 打开已配置的连接
STATIC int esp826601_socket_connect(mod_network_socket_obj_t* socket, byte* ip, mp_uint_t port, int* _errno)
{
    // socket opened
    socket->u_param.domain = 1;

    esp826601_obj_t* nic = socket->nic;
    const char* ip_str = mp_obj_str_get_str(netutils_format_ipv4_addr(ip, NETUTILS_BIG));

    MP_THREAD_GIL_EXIT();
    mp_int_t ret = esp_cip_start(nic->esp, socket->u_param.fileno, socket->u_param.type, ip_str, port, 0);
    MP_THREAD_GIL_ENTER();

    if (ret < 0) {
        esp826601_socket_close(socket);
        *_errno = -ret;
        return -1;
    }

    // success
    return 0;
}

// TCP 发送数据
STATIC mp_uint_t esp826601_socket_send(mod_network_socket_obj_t* socket, const byte* buf, mp_uint_t len, int* _errno)
{
    esp826601_obj_t* nic = socket->nic;
    if (len > 2048) {
        len = 2048;
    }
    MP_THREAD_GIL_EXIT();
    mp_int_t ret = esp_cip_send(nic->esp, socket->u_param.fileno, (uint8_t*)buf, len);
    MP_THREAD_GIL_ENTER();

    if (ret < 0) {
        esp826601_socket_close(socket);
        *_errno = -ret;
        return -1;
    }
    return len;
}

// TCP 接受数据
// 返回值表示读取到的字节数
STATIC mp_uint_t esp826601_socket_recv(mod_network_socket_obj_t* socket, byte* buf, mp_uint_t len, int* _errno)
{
    esp826601_obj_t* nic = socket->nic;

    MP_THREAD_GIL_EXIT();
    mp_int_t ret = esp_cip_recv(nic->esp, socket->u_param.fileno, buf, len);
    MP_THREAD_GIL_ENTER();

    if (ret < 0) {
        esp826601_socket_close(socket);
        *_errno = -ret;
        return -1;
    }
    return ret;
}

STATIC mp_uint_t esp826601_socket_sendto(mod_network_socket_obj_t* socket, const byte* buf, mp_uint_t len, byte* ip, mp_uint_t port, int* _errno)
{
    return -1;
}

STATIC mp_uint_t esp826601_socket_recvfrom(mod_network_socket_obj_t* socket, byte* buf, mp_uint_t len, byte* ip, mp_uint_t* port, int* _errno)
{
    return -2;
}
STATIC int esp826601_socket_setsockopt(mod_network_socket_obj_t* socket, mp_uint_t level, mp_uint_t opt, const void* optval, mp_uint_t optlen, int* _errno)
{
    *_errno = MP_EINVAL;
    return -1;
}

STATIC int esp826601_socket_settimeout(mod_network_socket_obj_t* socket, mp_uint_t timeout_ms, int* _errno)
{
    *_errno = MP_EINVAL;
    return -1;
}
STATIC int esp826601_socket_ioctl(mod_network_socket_obj_t* socket, mp_uint_t request, mp_uint_t arg, int* _errno)
{
    if (request == MP_STREAM_POLL) {
        int ret = 0;
        if (arg & MP_STREAM_POLL_RD /* && 获取读取状态 */) {
            ret |= MP_STREAM_POLL_RD;
        }
        if (arg & MP_STREAM_POLL_WR /* && 获取发送状态 */) {
            ret |= MP_STREAM_POLL_WR;
        }
        return ret;
    } else {
        *_errno = MP_EINVAL;
        return MP_STREAM_ERROR;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// \classmethod \constructor(uart_id, baudrate=115200)
// Initialise the ESP8266-01 using the given uart bus and baudrate and return a ESP8266-01 object.
STATIC mp_obj_t esp826601_make_new(const mp_obj_type_t* type, size_t n_args, size_t n_kw, const mp_obj_t* args)
{
    // check arguments
    mp_arg_check_num(n_args, n_kw, 2, 3, false);

    esp826601_obj.base.type = (mp_obj_type_t*)&mod_network_nic_type_esp826601;
    esp826601_obj.tcp_listen_port = 0;

    uint32_t uart_id = mp_obj_get_int(args[0]);
    if (!uart_exists(uart_id)) {
        nlr_raise(mp_obj_new_exception_msg_varg(
            &mp_type_OSError,
            "UART(%d) is not exists\n", uart_id));
    }

    esp826601_obj.esp = esp_create(uart_id, mp_obj_get_int(args[1]));
    esp_ctrl_init(esp826601_obj.esp);

    // register with network module
    mod_network_register_nic(&esp826601_obj);

    // return esp826601 object
    return &esp826601_obj;
}

/// \method isconnected()
/// Get connect status
STATIC mp_obj_t esp826601_isconnected(mp_obj_t self_in)
{
    esp826601_obj_t * pesp8266 = self_in;
    return mp_obj_new_bool(esp_ctrl_is_connect(pesp8266->esp));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp826601_isconnected_obj, esp826601_isconnected);

/// \method ifconfig([(ip, subnet, gateway, dns)])
/// Get/set IP address, subnet mask, gateway and DNS.
STATIC mp_obj_t esp826601_ifconfig(size_t n_args, const mp_obj_t* args)
{
    esp826601_obj_t * pesp8266 = (esp826601_obj_t*) args[0];

    esp826601_info_t ip_info = {};

    // query ip info only
    if (n_args == 1) {
        if (0 != esp_ctrl_qry_sta_ip(pesp8266->esp, &ip_info)) {
            nlr_raise(mp_obj_new_exception_msg_varg(
                &mp_type_OSError,
                "could not get ip info\n"));
        }
        char dns[16];
        uint8_t n = 0;
        esp_ctrl_qry_dns(pesp8266->esp, dns, &n);

        mp_obj_t tuple[4] = {
            ip_info.ip,
            ip_info.netmask,
            ip_info.gw,
            mp_obj_new_str(dns, n),
        };
        return mp_obj_new_tuple(4, tuple);
    } else {
        // Set static IP addresses
        mp_obj_t* items;
        mp_obj_get_array_fixed_n(args[1], 4, &items);
        ip_info.ip = items[0];
        ip_info.netmask = items[1];
        ip_info.gw = items[2];
        esp_ctrl_set_static_ip(pesp8266->esp, &ip_info);
        esp_ctrl_set_dns(pesp8266->esp, mp_obj_str_get_str(items[3]));
        return mp_const_none;
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp826601_ifconfig_obj, 1, 2, esp826601_ifconfig);

/// \method connect(ssid, key=None, *, security=WPA2, bssid=None)
STATIC mp_obj_t esp826601_connect(size_t n_args, const mp_obj_t* pos_args, mp_map_t* kw_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = MP_OBJ_NULL } },
        { MP_QSTR_pwd, MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_bssid, MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get ssid
    size_t ssid_len;
    const char* ssid = mp_obj_str_get_data(args[0].u_obj, &ssid_len);

    // get key and sec
    size_t pwd_len = 0;
    const char* pwd = NULL;
    if (args[1].u_obj != mp_const_none) {
        pwd = mp_obj_str_get_data(args[1].u_obj, &pwd_len);
    }

    // get bssid
    const char* bssid = NULL;
    if (args[2].u_obj != mp_const_none) {
        bssid = mp_obj_str_get_str(args[2].u_obj);
    }

    if (0 != esp_ctrl_connect(esp826601_obj.esp, ssid, ssid_len, bssid, pwd, pwd_len)) {
        nlr_raise(mp_obj_new_exception_msg_varg(
            &mp_type_OSError,
            "could not connect to ssid=%s, key=%s\n",
            ssid, pwd));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(esp826601_connect_obj, 1, esp826601_connect);

/// \method disconnect()
STATIC mp_obj_t esp826601_disconnect(mp_obj_t self_in)
{
    esp826601_obj_t * pesp8266 = (esp826601_obj_t*) self_in;
    esp_ctrl_disconnect(pesp8266->esp);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp826601_disconnect_obj, esp826601_disconnect);

STATIC const mp_rom_map_elem_t esp826601_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&esp826601_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect), MP_ROM_PTR(&esp826601_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&esp826601_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_ifconfig), MP_ROM_PTR(&esp826601_ifconfig_obj) },
};

STATIC MP_DEFINE_CONST_DICT(esp826601_locals_dict, esp826601_locals_dict_table);

const mod_network_nic_type_t mod_network_nic_type_esp826601 = {
    .base = {
        { &mp_type_type },
        .name = MP_QSTR_ESP826601,
        .make_new = esp826601_make_new,
        .locals_dict = (mp_obj_dict_t*)&esp826601_locals_dict,
    },
    .gethostbyname = esp826601_gethostbyname,
    .socket = esp826601_socket_socket,
    .close = esp826601_socket_close,
    .bind = esp826601_socket_bind,
    .listen = esp826601_socket_listen,
    .accept = esp826601_socket_accept,
    .connect = esp826601_socket_connect,
    .send = esp826601_socket_send,
    .recv = esp826601_socket_recv,
    .sendto = esp826601_socket_sendto,
    .recvfrom = esp826601_socket_recvfrom,
    .setsockopt = esp826601_socket_setsockopt,
    .settimeout = esp826601_socket_settimeout,
    .ioctl = esp826601_socket_ioctl,
};
#endif
