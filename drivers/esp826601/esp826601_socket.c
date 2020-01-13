#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "pendsv.h"

#include "esp826601.h"

extern int8_t wifi_cmd_raw(esp_t* esp, size_t len, void* cmd);

char* esp_net_type[] = {
    [ESP_TCP] = "TCP",
    [ESP_UDP] = "UDP",
    [ESP_SSL] = "SSL",
};

// 设置socket参数
int8_t esp_setsockopt(uint8_t link_id, sockopt_type sotype, void* arg)
{
    return 0;
}

int8_t esp_cip_start(
    esp_t* esp,
    uint8_t link_id, uint8_t type,
    const char* ip, uint16_t port,
    uint16_t keep_alive)
{

    // 分配缓冲
    ringbuf_alloc(&esp->tcp_rx_buf[link_id], MICROPY_PY_ESP826601_TCP_BUF_SIZE);

    // req: AT+CIPSTART=<link ID>,<type>,<remote IP>,<remote port>[,<TCP keep alive>]
    // rsp: OK

    size_t len = snprintf((char*)esp->cmd_buf, ESP_TX_BUF_SIZE,
        ESP_AT("CIPSTART=%u,\"%s\",\"%s\",%u,%u"),
        link_id, esp_net_type[type],
        ip, port, keep_alive);
    return wifi_cmd_raw(esp, len, esp->cmd_buf);

    // 打开后， 需要立即设置数据接收模式(只有TCP有效):
    // AT+CIPRECVMODE=<mode>
    // mode=0, 主动模式, 模块胡通过串口直接发送到MCU
    // mode=1, 被动模式, ESP8266 默认使⽤ 2920 bytes 的 buffer ⽤于缓存接收到的 TCP 数据，
    //                  被动等待MCU读取; 接收buffer满后，将阻塞对端TCP发送数据
    return 0;
}

int8_t esp_cip_close(esp_t* esp, uint8_t link_id)
{
    esp_link_free(esp, link_id);
    ringbuf_free(&esp->tcp_rx_buf[link_id]);
    // req: AT+CIPCLOSE=<link_id>
    // rsp: OK
    size_t len = snprintf((char*)esp->cmd_buf, ESP_TX_BUF_SIZE, ESP_AT("CIPCLOSE=%u"), link_id);
    return wifi_cmd_raw(esp, len, esp->cmd_buf);
}

int8_t esp_cip_server(esp_t* esp, uint8_t mode, uint16_t port)
{
    // AT+CIPSERVER=<mode>[,<port>]
    // rsp: OK
    return 0;
}

int8_t esp_cip_send(esp_t* esp, uint8_t link_id, uint8_t* data, size_t data_len)
{
    // 1. 判断link_id是否在有效范围
    // 2. link_id对应的连接必须是TCP模式
    // 3. 检查数据长度>0 及 不超过2048
    // 4. 确定当前连接已经打开, 且不是出于关闭状态
    // 5.
    size_t len = snprintf((char*)esp->cmd_buf, ESP_TX_BUF_SIZE, ESP_AT("CIPSEND=%u,%u"), link_id, data_len);

    esp_tx_strn(esp, esp->cmd_buf, len);
    esp_wait_state(esp, rsp_status_pending);

    if (esp->tx_acked_status != rsp_status_success) {
        return esp->tx_acked_status;
    }

    // 等待提示符
    esp_wait_state(esp, rsp_status_sending);

    // 发送数据, 等待发送结果
    esp_tx_strn(esp, data, data_len);
    return esp_wait_state(esp, rsp_status_pending);

    // 多连接TCP : AT+CIPSEND=<link ID>,<length>
    // 多连接UDP : AT+CIPSEND=[<link ID>,]<length> [,<remote IP>,<remote port>]

    // 执行以上命令后， 模块返回 ">" , 再开始传送数据

    // 返回值: ERROR/SEND OK/SEND FAIL
}

int8_t esp_cip_recv(esp_t* esp, uint8_t link_id, uint8_t* data, uint16_t len)
{
    uint16_t n = 0;
    int val = -1;

    // 读取期间禁止调度
    MICROPY_PY_ESP_ENTER
    ringbuf_t* rb = &esp->tcp_rx_buf[link_id];

    for (n = 0; n < len; n++) {
        val = ringbuf_get(rb);
        if (val == -1) {
            break; // buf empty
        }
        data[n] = val;
    }

    MICROPY_PY_ESP_EXIT
    return n;

    // 模块 在收到数据后，会按以下方式返回
    // (+CIPMUX=1)+IPD,<link ID>,<len>[,<remote IP>,<remote port>]:<data>
    //
    // 模块收到就会立即发送到MCU端, 所以应该放一个环形缓冲, 用户调用本方法时，从缓冲取数据

    /*
    此指令在普通指令模式下有效， ESP8266 接收到⽹络数据时向串⼝发送 +IPD 和数据。
    • [<remote	IP>]：⽹络通信对端 IP，由指令 AT+CIPDINFO=1 使能显示
    • [<remote	port>]：⽹络通信对端端⼝，由指令 AT+CIPDINFO=1 使能
    • <link	ID>：收到⽹络连接的 ID 号
    • <len>：数据⻓度
    • <data>：收到的数据
    */

    // 被动模式下, 通过: AT+CIPRECVDATA=<link_id>,<len> 接收数据
    // 响应: +CIPRECVDATA,<actual_len>:<data>\r\nOK\r\n
    //
    // 通过 AT+CIPRECVLEN? 查询各个连接的接收缓冲, 返回:
    // +CIPRECVLEN:<data length of link0>,<data	length of link1>,<data length of link2>,<data length of link3>,<data length of link4>\r\nOK
}
