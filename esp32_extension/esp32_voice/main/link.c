//=============================================================================
//  link.c —— ESP32 <-> FPGA 1 Mbps 二进制链路实现
//=============================================================================
#include "link.h"

#include <string.h>
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "link";

#define LINK_UART_NUM  UART_NUM_1
#define LINK_TX_GPIO   18
#define LINK_RX_GPIO   17
#define LINK_BAUD      1041667

static link_telem_cb_t s_cb = NULL;
static volatile int64_t s_last_telem_us = 0;   // 最近一次有效遥测的时间戳
static volatile uint32_t s_telem_cnt = 0;
static volatile uint32_t s_err_cnt = 0;
static volatile uint32_t s_rx_bytes = 0;
static uint8_t  s_dbg[64];
static int      s_dbg_len = 0;
static bool     s_dbg_done = false;

//-----------------------------------------------------------------------------
static uint16_t crc16(const uint8_t *d, int n)
{
    uint16_t c = 0xFFFF;
    for (int i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

// 逐字节累积（解析时必须用这个，不能每字节都重新 crc16）
static inline uint16_t crc16_update(uint16_t c, uint8_t d)
{
    c ^= (uint16_t)d << 8;
    for (int b = 0; b < 8; b++)
        c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    return c;
}

void link_send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t f[80];
    if (len > 64) return;
    f[0] = 0xAA; f[1] = 0x55; f[2] = type; f[3] = len;
    if (len) memcpy(&f[4], payload, len);
    uint16_t c = crc16(&f[2], 2 + len);
    f[4 + len] = (uint8_t)(c >> 8);
    f[5 + len] = (uint8_t)(c & 0xFF);
    uart_write_bytes(LINK_UART_NUM, (const char *)f, 6 + len);
}

bool link_is_alive(void)
{
    return (esp_timer_get_time() - s_last_telem_us) < 500000;   // 500 ms
}

uint32_t link_rx_bytes(void)   { return s_rx_bytes; }
uint32_t link_telem_count(void){ return s_telem_cnt; }
uint32_t link_error_count(void){ return s_err_cnt; }

//-----------------------------------------------------------------------------
// 字节流 -> 帧
//-----------------------------------------------------------------------------
typedef enum { S_HUNT = 0, S_H2, S_TYPE, S_LEN, S_DATA, S_CRCH, S_CRCL } pstate_t;

static void on_frame(uint8_t type, const uint8_t *pl, uint8_t len)
{
    if (type == T_TELEM && len >= 20) {
        link_telem_t t;
        t.seq     = pl[0];
        t.state   = pl[1];
        t.flags   = pl[2];
        t.rate    = pl[3];
        t.angle   = (int16_t)((pl[4] << 8) | pl[5]);
        t.target  = (int16_t)((pl[6] << 8) | pl[7]);
        t.pwm     = (int16_t)((pl[8] << 8) | pl[9]);
        t.enc1    = (uint16_t)((pl[10] << 8) | pl[11]);
        t.enc2    = (uint16_t)((pl[12] << 8) | pl[13]);
        t.speed   = (int16_t)((pl[14] << 8) | pl[15]);
        t.loc     = (int16_t)((pl[16] << 8) | pl[17]);
        t.loc_tgt = (int16_t)((pl[18] << 8) | pl[19]);

        s_last_telem_us = esp_timer_get_time();
        s_telem_cnt++;
        if (s_cb) s_cb(&t, pl, len);
    } else if (type == T_ACK && len >= 2) {
        if (pl[1] != 0) ESP_LOGW(TAG, "FPGA 拒绝指令 0x%02X，状态 %d", pl[0], pl[1]);
    } else if (type == T_PONG) {
        ESP_LOGD(TAG, "PONG");
    }
}

static void link_rx_task(void *arg)
{
    uint8_t b;
    pstate_t st = S_HUNT;
    uint8_t type = 0, len = 0, idx = 0, crc_h = 0;
    uint8_t pl[64];
    uint16_t crc = 0xFFFF;

    while (1) {
        int n = uart_read_bytes(LINK_UART_NUM, &b, 1, pdMS_TO_TICKS(20));
        if (n <= 0) {
            // 主动让出 CPU，避免高优先级 RX 任务饿死 IDLE1 触发任务看门狗
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        s_rx_bytes += (uint32_t)n;

        // 一次性抓取前 64 字节，判断 F21 上到底是 AA55 还是 00/FF/乱码
        if (!s_dbg_done && s_dbg_len < (int)sizeof(s_dbg)) {
            s_dbg[s_dbg_len++] = b;
            if (s_dbg_len == (int)sizeof(s_dbg)) {
                ESP_LOG_BUFFER_HEX(TAG, s_dbg, s_dbg_len);
                s_dbg_done = true;
            }
        }

        switch (st) {
        case S_HUNT: if (b == 0xAA) st = S_H2;                       break;
        case S_H2:   st = (b == 0x55) ? S_TYPE : (b == 0xAA ? S_H2 : S_HUNT); break;
        case S_TYPE: type = b; crc = crc16_update(0xFFFF, b); st = S_LEN; break;
        case S_LEN:
            len = b;
            if (len > 64) { s_err_cnt++; st = S_HUNT; }
            else {
                crc = crc16_update(crc, b);
                idx = 0;
                st  = (len == 0) ? S_CRCH : S_DATA;
            }
            break;
        case S_DATA:
            pl[idx++] = b;
            crc = crc16_update(crc, b);
            if (idx >= len) st = S_CRCH;
            break;
        case S_CRCH: crc_h = b; st = S_CRCL;                          break;
        case S_CRCL: {
            uint16_t got = (uint16_t)((crc_h << 8) | b);
            if (got == crc) on_frame(type, pl, len);
            else            s_err_cnt++;
            st = S_HUNT;
            break;
        }
        default: st = S_HUNT; break;
        }
    }
}

//-----------------------------------------------------------------------------
static void heartbeat_cb(void *arg)
{
    uint8_t z = 0;
    // FPGA 端 1 s 收不到帧就强制停机，所以心跳必须持续发
    link_send_frame(LK_HEARTBEAT, &z, 1);
}

static void link_baud_scan(void)
{
    static const int bauds[] = {
        9600, 115200, 460800, 500000, 921600,
        1000000, 1041667, 1152000, 1500000, 2000000
    };
    uint8_t head[24];

    for (int i = 0; i < (int)(sizeof(bauds) / sizeof(bauds[0])); i++) {
        if (uart_set_baudrate(LINK_UART_NUM, bauds[i]) != ESP_OK) continue;
        uart_flush_input(LINK_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(15));

        int total = 0, aa55 = 0, hl = 0;
        uint8_t prev = 0;
        int64_t end = esp_timer_get_time() + 150000;
        while (esp_timer_get_time() < end) {
            uint8_t b;
            int n = uart_read_bytes(LINK_UART_NUM, &b, 1, pdMS_TO_TICKS(5));
            if (n <= 0) continue;
            total++;
            if (hl < (int)sizeof(head)) head[hl++] = b;
            if (prev == 0xAA && b == 0x55) aa55++;
            prev = b;
        }
        ESP_LOGI(TAG, "scan %d: bytes=%d aa55=%d", bauds[i], total, aa55);
        if (aa55) ESP_LOG_BUFFER_HEX(TAG, head, hl);
    }

    uart_set_baudrate(LINK_UART_NUM, LINK_BAUD);
    uart_flush_input(LINK_UART_NUM);
}

esp_err_t link_init(link_telem_cb_t cb)
{
    s_cb = cb;

    const uart_config_t cfg = {
        .baud_rate  = LINK_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(LINK_UART_NUM, 4096, 2048, 0, NULL, 0),
                        TAG, "uart_driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(LINK_UART_NUM, &cfg), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(LINK_UART_NUM, LINK_TX_GPIO, LINK_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");
    link_baud_scan();

    xTaskCreatePinnedToCore(link_rx_task, "link_rx", 4096, NULL, 5, NULL, 1);

    const esp_timer_create_args_t targs = { .callback = heartbeat_cb, .name = "hb" };
    esp_timer_handle_t t;
    ESP_RETURN_ON_ERROR(esp_timer_create(&targs, &t), TAG, "timer_create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(t, 400 * 1000), TAG, "timer_start");

    ESP_LOGI(TAG, "UART%d %d bps, TX=%d RX=%d, 心跳 400 ms",
             LINK_UART_NUM, LINK_BAUD, LINK_TX_GPIO, LINK_RX_GPIO);
    return ESP_OK;
}
