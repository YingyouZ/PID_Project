//=============================================================================
//  link.h —— ESP32 <-> FPGA 1 Mbps 二进制链路
//
//  帧格式与 offline_link/rtl/link.v 完全一致：
//      AA 55 | TYPE | LEN | PAYLOAD | CRC16_H | CRC16_L
//      CRC16-CCITT-FALSE, poly=0x1021, init=0xFFFF, 覆盖 TYPE..PAYLOAD
//
//  接线：ESP32 GPIO18(TX) -> FPGA F20(uart_rx)
//        ESP32 GPIO17(RX) <- FPGA F21(uart_tx)
//        GND <-> GND
//=============================================================================
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// 下行指令码（与 link.v 一致）——加 LK_ 前缀，避免和 app_main.c 里的 CMD_* 冲突
enum {
    LK_SET_MODE   = 0x01,
    LK_SET_ANGLE  = 0x02,
    LK_SET_LOC    = 0x03,
    LK_SET_PWM    = 0x04,
    LK_SET_STREAM = 0x07,
    LK_PING       = 0x08,
    LK_CLEAR      = 0x09,
    LK_HEARTBEAT  = 0x0A,
    LK_STEP       = 0x0B,
    LK_START      = 0x0C,
    LK_ESTOP      = 0x0D,
};

// 上行类型码
enum { T_TELEM = 0x81, T_PONG = 0x82, T_ACK = 0x83 };

// 遥测帧解析结果（20 字节 payload）
typedef struct {
    uint8_t  seq;
    uint8_t  state;
    uint8_t  flags;      // bit0=run bit1=fault bit2=link_ok
    uint8_t  rate;       // 0=off 1=100Hz 2=500Hz
    int16_t  angle;
    int16_t  target;
    int16_t  pwm;
    uint16_t enc1, enc2;
    int16_t  speed;
    int16_t  loc;
    int16_t  loc_tgt;
} link_telem_t;

// 收到完整遥测帧时回调（在 UART 任务上下文里，不要做耗时操作）
typedef void (*link_telem_cb_t)(const link_telem_t *t, const uint8_t *raw, int raw_len);

// 初始化 UART + 解析任务 + 心跳定时器
esp_err_t link_init(link_telem_cb_t cb);

// 发送一帧给 FPGA
void link_send_frame(uint8_t type, const uint8_t *payload, uint8_t len);

// 最近 500ms 内是否收到过遥测（= FPGA 在线）
bool link_is_alive(void);

// 诊断计数：收到的字节数 / 有效遥测帧数 / 校验或长度错误数
uint32_t link_rx_bytes(void);
uint32_t link_telem_count(void);
uint32_t link_error_count(void);
