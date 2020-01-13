#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pendsv.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "lib/netutils/netutils.h"

#include "esp826601_ll.h"
#include "led.h"

#define ESP_DEBUG(...) printf(__VA_ARGS__)
// #define ESP_DEBUG(...)

// atomic operator
#define ESP_ENTER() uint32_t irq_state = raise_irq_pri(IRQ_PRI_PENDSV);
#define ESP_REENTER() irq_state = raise_irq_pri(IRQ_PRI_PENDSV);
#define ESP_EXIT() restore_irq_pri(irq_state);

static uint8_t esp_uart_rxbuf[ESP_RX_BUF_SIZE];
static esp_t esp = { 0 };

/* ESP打开状态
0 - 关闭
1 - 开启
*/
volatile uint8_t mp_nw_esp_state = esp826601_state_off;

#define IS_ESP_SENDING(esp) (esp->tx_acked_status == rsp_status_sending)
#define IS_ESP_AT_CMD_PENDING(esp) (esp->tx_acked_status == rsp_status_pending)
#define ESP_AT_CMD_END_PENDING(esp, x) (esp->tx_acked_status = (x))

#define a2n(c) ((c) - '0')

static inline bool esp_rx_any(esp_t* esp)
{
    return uart_rx_any(&esp->bus);
}
static inline uint8_t esp_rx_char(esp_t* esp)
{
    return uart_rx_char(&esp->bus);
}

int8_t wifi_cmd_raw(esp_t* esp, size_t len, void* cmd)
{
    ESP_DEBUG(cmd);
    esp_tx_strn(esp, cmd, len);
    return esp_wait_state(esp, rsp_status_pending);
}

static inline bool at_parse(esp_t* esp, char* buf, size_t len)
{
    uint8_t cmd_rsp_status = rsp_status_noack;

    if (strstr(buf, "OK\r\n")) {
        cmd_rsp_status = rsp_status_success;
    } else if (strstr(buf, "ERROR\r\n")) {
        cmd_rsp_status = rsp_status_error;
    } else if (strstr(buf, "WIFI GOT IP")) {
        return true;
    } else if (strstr(buf, "FAIL\r\n")) {
        cmd_rsp_status = rsp_status_error;
    } else {
        // 其他格式响应数据检查
    }

    // 已经有回复了
    if (cmd_rsp_status != rsp_status_noack) {
        if (IS_ESP_AT_CMD_PENDING(esp)) {
            memcpy(esp->ack_buf, buf, len);
            esp->ack_buf[len] = 0;
            ESP_AT_CMD_END_PENDING(esp, cmd_rsp_status);
        } else if (IS_ESP_SENDING(esp)) {
            ESP_AT_CMD_END_PENDING(esp, cmd_rsp_status);
        }

        else {
            ESP_DEBUG("cmd_rsp_status is [%d], but no cmd pending \n", cmd_rsp_status);
        }
        return true;
    }
    return false;
}

void esp_uart_rx_poll(esp_t* esp)
{
    uint16_t i = 0;
    uint8_t link_id;

    while (esp_rx_any(esp)) {
        led_state(4, 1);
        // 先取出数据
        esp->rx_buf[esp->rx_len] = esp_rx_char(esp);
        if (esp->rx_buf[esp->rx_len] != '\r' && esp->rx_buf[esp->rx_len] != '\n') {
            ESP_DEBUG("R|%02x|%c|%d\n", esp->rx_buf[esp->rx_len], esp->rx_buf[esp->rx_len], esp->rx_len);
        }
        led_state(4, 0);

        // 正在读取数据包长度
        if (esp->data_rx_status == 1) {

            // 长度读取结束
            if (esp->rx_buf[0] == ':') {
                esp->data_rx_status = 2;
                ESP_DEBUG("link %d <- %d bytes\n", esp->worker_link_id, esp->pkt_remaining_bytes);
            } else {
                esp->pkt_remaining_bytes += (esp->pkt_remaining_bytes * 10) + a2n(esp->rx_buf[0]);
            }
            continue;
        }

        // 正在读取剩余数据
        else if (esp->data_rx_status == 2) {
            link_id = esp->worker_link_id;
            
            // 把数据放到对应TCP缓冲, 需要检查是否已满
            ringbuf_put(&esp->tcp_rx_buf[link_id], esp->rx_buf[0]);

            // 已经读取完毕
            if (--esp->pkt_remaining_bytes == 0) {
                esp->data_rx_status = 0;
                ESP_DEBUG("tcp buf#%d size is %d\n", link_id, ringbuf_avail(&esp->tcp_rx_buf[link_id]));
            }
            continue;
        }

        // 非 asiic, 继续读
        else if (esp->rx_buf[esp->rx_len] > 0x7f) {
            ++esp->rx_len;
            continue;
        }

        // 这里碰到\n 且 已收缓冲大小大于2 就停止读取
        if (esp->rx_buf[esp->rx_len] == '\n' && esp->rx_len > 2) {
            esp->rx_buf[esp->rx_len + 1] = 0;
            if (at_parse(esp, esp->rx_buf, esp->rx_len)) {
                esp->rx_len = 0;
                return;
            }
        }

        // 正在发送数据
        else if (IS_ESP_SENDING(esp)) {
            if (esp->rx_buf[esp->rx_len] == '>') {
                ESP_AT_CMD_END_PENDING(esp, rsp_status_success);
                esp->rx_buf[0] = esp_rx_char(esp); // drop ' '(0x20)
                ESP_DEBUG("\nsend ready\n");
                esp->rx_len = 0;
                continue;
            }
        }

        // 需要检查是否有要接收的数据
        // 10 : 缓冲中至少存在 \r\n+IPD,x,y:
        else if (
            (esp->data_rx_status == 0)
            && (esp->rx_len == 10)
            && (memcmp(esp->rx_buf, "\r\n+IPD,", 7) == 0)) {

            esp->worker_link_id = a2n(esp->rx_buf[7]) & ESP_LINK_Mask;

            // 读取数据长度
            esp->pkt_remaining_bytes = a2n(esp->rx_buf[9]);

            // 长度只有1位
            if (esp->rx_buf[10] == ':') {
                esp->data_rx_status = 2;
                ESP_DEBUG("link %d <- %d bytes\n", esp->worker_link_id, esp->pkt_remaining_bytes);
            } else {
                esp->pkt_remaining_bytes += (esp->pkt_remaining_bytes * 10) + a2n(esp->rx_buf[i]);

                // 数据包长度还没读完， 下次继续
                esp->data_rx_status = 1;
            }

            // 清空读缓冲
            esp->rx_len = 0;
            continue;
        }
        ++esp->rx_len;
    }

    led_state(3, esp->rx_len);
}

int8_t esp_ctrl_connect(
    esp_t* esp,
    const char* ssid, size_t ssid_len,
    const char* bssid,
    const char* pwd, size_t pwd_len)
{
    size_t n = snprintf((char*)esp->cmd_buf, ESP_TX_BUF_SIZE,
        ESP_AT("CWJAP_CUR=\"%s\",\"%s\""),
        ssid, pwd);
    wifi_cmd_raw(esp, n, esp->cmd_buf);
    wifi_cmd_raw(esp, 13, ESP_AT("CIPMUX=1")); // 打开多连接模式
    // wifi_cmd_raw(18, ESP_AT("CIPRECVMODE=1")); // 设置TCP为被动接收模式
    return 0;
}

int8_t esp_ctrl_disconnect(esp_t* esp)
{
    return wifi_cmd_raw(esp, 10, ESP_AT("CWQAP"));
}

int8_t esp_ctrl_qry_sta_ip(esp_t* esp, esp826601_info_t* info)
{
    wifi_cmd_raw(esp, 16, ESP_AT("CIPSTA_CUR?"));
    char* p = strstr((char*)(esp->ack_buf), ":ip:\"");
    char* p1;

    // fixme:, use mp_obj_t to parse ip, need better
    if (p) {
        p += 5; // skip :ip:"

        // get end of ip
        p1 = strstr(p, "\"");
        info->ip = mp_obj_new_str(p, p1 - p);
        p = p1 + 24; // skip "\r\n+CIPSTA_CUR:gateway:"

        // get end of gateway
        p1 = strstr(p, "\"");
        info->gw = mp_obj_new_str(p, p1 - p);
        p = p1 + 24; // skip "\r\n+CIPSTA_CUR:netmask:"

        // get end of netmask
        p1 = strstr(p, "\"");
        info->netmask = mp_obj_new_str(p, p1 - p);

        return 0;
    } else {
        // no ip or in dhcp mode
        return -1;
    }
}

int8_t esp_ctrl_set_static_ip(esp_t* esp, const esp826601_info_t* info)
{
    size_t n = snprintf((char*)esp->cmd_buf, ESP_TX_BUF_SIZE,
        ESP_AT("CIPSTA_CUR=\"%s\",\"%s\",\"%s\""),
        mp_obj_str_get_str(info->ip),
        mp_obj_str_get_str(info->netmask),
        mp_obj_str_get_str(info->gw));
    return wifi_cmd_raw(esp, n, esp->cmd_buf);
}

int8_t esp_ctrl_qry_dns(esp_t* esp, char* dns, uint8_t* size)
{
    wifi_cmd_raw(esp, 16, ESP_AT("CIPDNS_CUR?"));
    char* p = strstr((char*)esp->ack_buf, "+CIPDNS_CUR:");
    p += 12;
    char* p1 = strstr(p, ESP_AT_END);
    *size = p1 - p;
    memcpy(dns, p, *size);
    return 0;
}

int8_t esp_ctrl_set_dns(esp_t* esp, const char* dns)
{
    size_t n = snprintf((char*)esp->cmd_buf, ESP_TX_BUF_SIZE,
        ESP_AT("CIPDNS_CUR=1,\"%s\""),
        dns);
    return wifi_cmd_raw(esp, n, esp->cmd_buf);
}

bool esp_ctrl_is_connect(esp_t* esp)
{
    wifi_cmd_raw(esp, 15, ESP_AT("CWJAP_CUR?"));
    if (strstr((char*)esp->ack_buf, "No AP")) {
        return false;
    }
    return true;
}

// DNS解析
int8_t esp_ctrl_cip_domain(esp_t* esp, const char* domain, uint8_t* n, char* ip)
{
    size_t len = snprintf((char*)esp->cmd_buf, ESP_TX_BUF_SIZE,
        ESP_AT("CIPDOMAIN=\"%s\""),
        domain);
    int8_t ret_code = wifi_cmd_raw(esp, len, esp->cmd_buf);

    if (ret_code != ESP_SUCCESS) {
        return (int8_t)ret_code;
    }

    // +CIPDOMAIN:<IP address>\r\nOK\r\n
    char* p = strstr((char*)esp->ack_buf, "CIPDOMAIN:");
    char* p1 = 0;
    if (p) {
        p += 10;
        p1 = strstr(p, "\r\n");
        *n = p1 - p;
        memcpy(ip, p, *n);
    }
    return 0;
}

static inline void esp_uart_configure(esp_t* esp, uint32_t port, uint32_t baudrate)
{
    esp->bus.base.type = &machine_uart_type;
    esp->bus.uart_id = port;
    esp->bus.is_static = true;
    esp->bus.timeout = 2;
    esp->bus.timeout_char = 2;
    uart_init(&esp->bus, baudrate, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_1, UART_HWCONTROL_NONE);
    uart_set_rxbuf(&esp->bus, sizeof(esp_uart_rxbuf), esp_uart_rxbuf);
    MP_STATE_PORT(machine_uart_obj_all)
    [esp->bus.uart_id - 1] = &esp->bus;
}

esp_t* esp_create(uint32_t port, uint baudrate)
{
    esp_uart_configure(&esp, port, baudrate);

    // do reset
    esp_tx_strn(&esp, ESP_AT("RST"), 8);
    mp_hal_delay_ms(2000);
    return &esp;
}

int8_t esp_ctrl_init(esp_t* esp)
{
    // drop
    while (esp_rx_any(esp)) {
        esp_rx_char(esp);
    }
    esp->rx_len = 0;
    esp->link_used = 0;

    mp_nw_esp_state = esp826601_state_on;

    wifi_cmd_raw(esp, 4, "AT\r\n"); // 回环测试
    wifi_cmd_raw(esp, 6, "ATE0\r\n"); // 取消回显 ATE0
    wifi_cmd_raw(esp, 17, ESP_AT("CWMODE_CUR=3")); // 设为AP+STA模式
    return 0;
}
