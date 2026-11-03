//=============================================================================
//  app_main.c —— 语音 + 网页 合并固件
//
//  一条固件里同时跑三件事：
//    1. WiFi 热点 + HTTP 网页 + WebSocket（手机控制与实时波形）
//    2. 内置倒立摆仿真（没有 FPGA 也能演示）
//    3. INMP441 + ESP-SR 离线语音识别
//
//  语音识别到命令后：
//    - 本地立即执行（模式/目标/步进），波形跟着变
//    - 通过 WebSocket 推一条 {"t":"voice",...} 给网页，页面显示"语音指令：启动"
//
//  接线：INMP441  SCK=GPIO4  WS=GPIO5  SD=GPIO6  L/R→GND  VDD→3V3
//=============================================================================
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

#include "esp_afe_config.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "model_path.h"
#include "link.h"          // ESP32 <-> FPGA 1 Mbps 链路

static const char *TAG = "pidvoice";

//-----------------------------------------------------------------------------
// 嵌入网页
//-----------------------------------------------------------------------------
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

//-----------------------------------------------------------------------------
// 协议（与 offline_link/rtl/link.v 一致）
//-----------------------------------------------------------------------------
#define T_TELEM 0x81
#define T_ACK   0x83

enum {
    CMD_MODE = 0x01, CMD_ANGLE = 0x02, CMD_LOC = 0x03, CMD_PWM = 0x04,
    CMD_STREAM = 0x07, CMD_PING = 0x08, CMD_CLEAR = 0x09, CMD_HB = 0x0A,
    CMD_STEP = 0x0B, CMD_START = 0x0C, CMD_ESTOP = 0x0D,
};

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

static int build_frame(uint8_t *out, uint8_t type, const uint8_t *pl, uint8_t len)
{
    out[0] = 0xAA; out[1] = 0x55; out[2] = type; out[3] = len;
    if (len) memcpy(out + 4, pl, len);
    uint16_t c = crc16(out + 2, 2 + len);
    out[4 + len] = (uint8_t)(c >> 8);
    out[5 + len] = (uint8_t)(c & 0xFF);
    return 6 + len;
}

static void put16(uint8_t *p, int v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)(v & 0xFF);
}

//-----------------------------------------------------------------------------
// 内置倒立摆仿真
//-----------------------------------------------------------------------------
#define SIM_A   4.0f     // 不稳定度：重力把摆杆推离中心
#define SIM_B   0.5f     // 自然阻尼
#define SIM_C   100.0f   // 控制力矩：必须大于 A*|x|max(=56)，否则倒到底就起不来
#define SIM_KP  0.13f    // 闭环 A-C*Kp = -9  → 自然频率 3 rad/s
#define SIM_KD  0.045f   // 阻尼 B+C*Kd ≈ 4.5 → 阻尼比约 0.75

static int   g_mode = 2, g_state = 6, g_target = 1856;   // 上电默认"保持平衡"，演示从中间那条线开始
static int   g_manual = 0, g_loc_target = 0, g_stream = 1;
static float g_pwm = 0, g_x = 0, g_vx = 0;
static int   g_angle = 1856, g_loc = 0;
static uint8_t g_seq = 0;

static void sim_step(float dt)
{
    float u = 0.0f;
    if (g_mode == 3) {
        u = g_manual / 100.0f;
        g_state = 0;
    } else if (g_mode != 0) {
        float e = (g_target - g_angle) / 100.0f;
        u = SIM_KP * e - SIM_KD * g_vx;
        if (u > 1.0f)  u = 1.0f;
        if (u < -1.0f) u = -1.0f;
        g_state = 6;
    } else {
        g_state = 0;
    }

    // 加扰动和偏置：否则 x=0、v=0 时仿真永远静止，波形是一条死线
    float noise = ((float)(esp_random() & 0xFFFF) / 65535.0f - 0.5f) * 3.0f;
    float acc = SIM_A * g_x - SIM_B * g_vx + SIM_C * u + noise + 0.20f;
    g_vx += acc * dt;
    g_x  += g_vx * dt;
    if (g_x > 14.0f)  { g_x = 14.0f;  g_vx *= -0.3f; }
    if (g_x < -14.0f) { g_x = -14.0f; g_vx *= -0.3f; }

    g_angle = 1856 + (int)(g_x * 100.0f);
    g_pwm   = u * 100.0f;
    g_loc  += (int)(g_vx * 100.0f * dt);
}

//-----------------------------------------------------------------------------
// 指令执行（网页和语音共用同一套）
//-----------------------------------------------------------------------------
static uint8_t apply_cmd(uint8_t type, const uint8_t *p, uint8_t len)
{
    int16_t v = (len >= 2) ? (int16_t)((p[0] << 8) | p[1]) : 0;

    switch (type) {
    case CMD_MODE:
        if (len < 1) return 2;
        g_mode = p[0] & 3;
        if (g_mode == 0) g_manual = 0;
        break;
    case CMD_ANGLE:  if (len < 2) return 2; g_target = v;          break;
    case CMD_LOC:    if (len < 2) return 2; g_loc_target = v;      break;
    case CMD_PWM:    if (len < 2) return 2;
                     g_manual = (v > 100) ? 100 : (v < -100 ? -100 : v); break;
    case CMD_STREAM: g_stream = (p[0] > 2) ? 2 : p[0];             break;
    case CMD_PING:
    case CMD_CLEAR:
    case CMD_HB:                                                   break;
    case CMD_STEP:   g_loc_target += (v > 0) ? 532 : -532;         break;
    case CMD_START:  g_mode = 1; g_target = 1856;                  break;
    case CMD_ESTOP:  g_mode = 0; g_manual = 0;                     break;
    default:         return 3;
    }
    return 0;
}

// FPGA 在线时把指令转发给它（ACK 由 FPGA 回），否则在本地仿真里执行
#define ST_FORWARDED 0xFE
static uint8_t execute_remote(uint8_t type, const uint8_t *p, uint8_t len)
{
    if (link_is_alive()) {
        link_send_frame(type, p, len);
        return ST_FORWARDED;
    }
    return apply_cmd(type, p, len);
}

//-----------------------------------------------------------------------------
// WebSocket 客户端
//-----------------------------------------------------------------------------
#define MAX_CLIENTS 4
static httpd_handle_t s_server = NULL;
static int s_fds[MAX_CLIENTS];

static void clients_reset(void) { for (int i = 0; i < MAX_CLIENTS; i++) s_fds[i] = -1; }

static bool client_add(int fd)
{
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (s_fds[i] < 0) { s_fds[i] = fd; return true; }
    return false;
}

static int client_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) if (s_fds[i] >= 0) n++;
    return n;
}

static int g_tx_ok = 0, g_tx_err = 0, g_drop_state = 0, g_skip = 0;

static void ws_broadcast(httpd_ws_type_t type, uint8_t *buf, int len)
{
    httpd_ws_frame_t f = { .type = type, .payload = buf, .len = len };
    for (int i = 0; i < MAX_CLIENTS; i++) {
        int fd = s_fds[i];
        if (fd < 0) continue;
        httpd_ws_client_info_t st = httpd_ws_get_fd_info(s_server, fd);
        if (st != HTTPD_WS_CLIENT_WEBSOCKET) {
            // 注意：握手刚完成那一瞬 fd 还是 HTTP 状态，不能因此踢人，
            // 只是这一拍跳过，下一拍自然就好了。
            if (st == HTTPD_WS_CLIENT_INVALID) {
                s_fds[i] = -1;
                g_drop_state = (int)st;
                ESP_LOGW(TAG, "客户端 fd=%d 失效，移除", fd);
            } else {
                g_skip++;
            }
            continue;
        }
        esp_err_t e = httpd_ws_send_frame_async(s_server, fd, &f);
        if (e == ESP_OK) {
            g_tx_ok++;
        } else {
            g_tx_err++;
            // 发不出去通常代表对端已经断了，直接移除，避免槽位被长期占用
            s_fds[i] = -1;
            if (g_tx_err < 5) ESP_LOGW(TAG, "发送失败 fd=%d: %s，已移除", fd, esp_err_to_name(e));
        }
    }
}

//-----------------------------------------------------------------------------
// FPGA 遥测回调：把真实帧原样转发给网页
//-----------------------------------------------------------------------------
static void on_fpga_telem(const link_telem_t *t, const uint8_t *payload, int len)
{
    static uint8_t buf[2][64];
    static int flip = 0;
    if (t) {
        g_angle = t->angle;
        g_target = t->target;
        g_pwm = t->pwm;
        g_state = t->state;
        g_loc = t->loc;
        g_loc_target = t->loc_tgt;
    }
    if (client_count() <= 0) return;
    uint8_t *b = buf[flip]; flip ^= 1;
    int n = build_frame(b, T_TELEM, payload, (uint8_t)len);
    ws_broadcast(HTTPD_WS_TYPE_BINARY, b, n);
}

//-----------------------------------------------------------------------------
// 100 Hz 遥测
//-----------------------------------------------------------------------------
static void telemetry_task(void *arg)
{
    uint8_t buf[2][40];
    int flip = 0;

    while (1) {
        // FPGA 在线时用真实数据（回调里已经实时转发），离线才跑本地仿真
        if (!link_is_alive()) sim_step(0.01f);

        if (!link_is_alive() && g_stream && client_count() > 0) {
            uint8_t pl[20];
            pl[0] = g_seq++;
            pl[1] = (uint8_t)g_state;
            pl[2] = (uint8_t)((g_mode ? 0x01 : 0) | 0x04);
            pl[3] = (uint8_t)g_stream;
            put16(pl + 4,  g_angle);
            put16(pl + 6,  g_target);
            put16(pl + 8,  (int)g_pwm);
            put16(pl + 10, g_loc & 0xFFFF);
            put16(pl + 12, 0);
            put16(pl + 14, (int)(g_vx * 100.0f));
            put16(pl + 16, g_loc);
            put16(pl + 18, g_loc_target);

            uint8_t *b = buf[flip]; flip ^= 1;
            int len = build_frame(b, T_TELEM, pl, 20);
            ws_broadcast(HTTPD_WS_TYPE_BINARY, b, len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//-----------------------------------------------------------------------------
// 语音命令表
//-----------------------------------------------------------------------------
typedef struct {
    int         id;
    const char *py;     // 注册用的拼音（mn6 要求拼音+空格）
    const char *cn;     // 中文，用于日志与网页显示
    const char *ev;     // 发给网页的事件名
    uint8_t     type;   // 协议命令
    int16_t     arg;
} voice_cmd_t;

static const voice_cmd_t VOICE[] = {
    { 1, "qi dong",   "启动", "start",  CMD_START,  0     },
    { 2, "xiang zuo", "向左", "left",   CMD_STEP,  -532   },
    { 3, "xiang you", "向右", "right",  CMD_STEP,   532   },
    { 4, "ting zhi",  "停止", "stop",   CMD_ESTOP,  0     },
};

static void voice_execute(const voice_cmd_t *c)
{
    uint8_t p[2];
    switch (c->type) {
    case CMD_MODE:
        p[0] = (uint8_t)c->arg;
        execute_remote(c->type, p, 1);
        break;
    case CMD_ANGLE:
    case CMD_LOC:
    case CMD_STEP:
        put16(p, c->arg);
        execute_remote(c->type, p, 2);
        break;
    default:
        execute_remote(c->type, NULL, 0);
        break;
    }

    ESP_LOGW(TAG, "语音执行: %s (%s)", c->cn, link_is_alive() ? "已转发 FPGA" : "本地仿真");

    static char json[128];
    snprintf(json, sizeof(json), "{\"t\":\"voice\",\"cmd\":\"%s\",\"text\":\"%s\",\"id\":%d}",
             c->ev, c->cn, c->id);
    ws_broadcast(HTTPD_WS_TYPE_TEXT, (uint8_t *)json, strlen(json));
}

//-----------------------------------------------------------------------------
// I2S 麦克风
//-----------------------------------------------------------------------------
#define I2S_BCLK_GPIO 4
#define I2S_WS_GPIO   5
#define I2S_DIN_GPIO  6

static i2s_chan_handle_t s_rx = NULL;
static int g_rms_l = 0, g_rms_r = 0;     // 最近一帧左右声道的能量，诊断用
static int32_t g_raw[4096];              // 最近一帧的原始 I2S 数据（诊断用）

static void i2s_mic_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_rx));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws   = I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN_GPIO,
            .invert_flags = { false, false, false },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;   // 左右都读，自动挑有声的那路

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));
    ESP_LOGI(TAG, "I2S: BCLK=%d WS=%d DIN=%d @16kHz", I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DIN_GPIO);
}

static int mic_read(int16_t *out, int samples)
{
    static int32_t raw[4096];
    size_t bytes = 0;
    if (i2s_channel_read(s_rx, raw, samples * 2 * sizeof(int32_t), &bytes, portMAX_DELAY) != ESP_OK)
        return -1;

    int n = (int)(bytes / (2 * sizeof(int32_t)));
    if (n <= 0) return -1;
    memcpy(g_raw, raw, (n * 2 > 4096 ? 4096 : n * 2) * sizeof(int32_t));

    double el = 0, er = 0;
    for (int i = 0; i < n; i++) {
        double l = raw[i * 2], r = raw[i * 2 + 1];
        el += l * l;
        er += r * r;
    }
    g_rms_l = (int)(sqrt(el / n) / 65536.0);
    g_rms_r = (int)(sqrt(er / n) / 65536.0);

    // 哪一路能量明显更大就用哪一路（防止 L/R 接反一直读静音声道）
    int ch = (er > el * 2.0) ? 1 : 0;
    for (int i = 0; i < n; i++) {
        int v = (int)(raw[i * 2 + ch] >> 16);           // 电平已经够（-20dB），不再加增益
        out[i] = (int16_t)v;
    }
    return 0;
}

//-----------------------------------------------------------------------------
// 语音识别任务（Core 1）
//-----------------------------------------------------------------------------
static void voice_task(void *arg)
{
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models) {
        ESP_LOGE(TAG, "模型分区加载失败");
        vTaskDelete(NULL);
        return;
    }
    for (int i = 0; i < models->num; i++)
        ESP_LOGI(TAG, "模型[%d]: %s", i, models->model_name[i]);

    char *wn_name  = esp_srmodel_filter(models, ESP_WN_PREFIX, "hilexin");
    char *wn2_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "hiesp");
    char *mn_name  = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    if (!wn_name) { wn_name = wn2_name; wn2_name = NULL; }

    afe_config_t *cfg = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    cfg->aec_init = false;
    cfg->se_init  = false;
    cfg->wakenet_init = true;
    cfg->wakenet_model_name   = wn_name;
    cfg->wakenet_model_name_2 = wn2_name;
    cfg->afe_perferred_core = 1;        // AFE 内部任务绑核心 1，别和 WiFi/HTTP（核心 0）抢
    cfg->afe_perferred_priority = 5;
    ESP_LOGI(TAG, "唤醒词1=%s  唤醒词2=%s", wn_name ? wn_name : "-", wn2_name ? wn2_name : "-");

    const esp_afe_sr_iface_t *afe = esp_afe_handle_from_config(cfg);
    esp_afe_sr_data_t *afe_data = afe->create_from_config(cfg);
    afe->print_pipeline(afe_data);

    // 非母语口音打分偏低，阈值从默认 0.5~0.9 降到 0.4（允许的下限）
    afe->set_wakenet_threshold(afe_data, 1, 0.4);
    if (wn2_name) afe->set_wakenet_threshold(afe_data, 2, 0.4);

    int chunksize = afe->get_feed_chunksize(afe_data);
    int channels  = afe->get_feed_channel_num(afe_data);

    // 旁路诊断：同一路音频直接喂唤醒模型，用来判断问题在 AFE 还是在模型/发音
    const esp_wn_iface_t *wn = wn_name ? esp_wn_handle_from_name(wn_name) : NULL;
    model_iface_data_t *wnd = wn ? wn->create(wn_name, DET_MODE_90) : NULL;
    if (wnd) wn->set_det_threshold(wnd, 0.4, 0);
    int wn_chunk = wnd ? wn->get_samp_chunksize(wnd) : 0;
    ESP_LOGI(TAG, "旁路 WakeNet: chunk=%d  AFE chunk=%d  %s", wn_chunk, chunksize,
             (wnd && wn_chunk == chunksize) ? "可对比" : "尺寸不一致，跳过");

    esp_mn_iface_t *mn = NULL;
    model_iface_data_t *mn_data = NULL;
    if (mn_name) {
        mn = esp_mn_handle_from_name(mn_name);
        mn_data = mn->create(mn_name, 6000);
        mn->set_det_threshold(mn_data, 0.5);

        esp_mn_commands_alloc(mn, mn_data);
        for (int i = 0; i < (int)(sizeof(VOICE) / sizeof(VOICE[0])); i++)
            esp_mn_commands_add(VOICE[i].id, VOICE[i].py);
        esp_mn_commands_update();
        ESP_LOGI(TAG, "命令词已注册 %d 条", (int)(sizeof(VOICE) / sizeof(VOICE[0])));
    }

    int16_t *feed = heap_caps_malloc(chunksize * channels * sizeof(int16_t),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!feed) feed = malloc(chunksize * channels * sizeof(int16_t));

    ESP_LOGI(TAG, "开始识别：先说唤醒词（Hi 乐鑫），再说命令");

    int tick = 0, peak_l = 0, peak_r = 0;

    while (1) {
        mic_read(feed, chunksize * channels);
        if (g_rms_l > peak_l) peak_l = g_rms_l;
        if (g_rms_r > peak_r) peak_r = g_rms_r;

        // 一次性诊断：抓住一帧大声的样本，打印原始 32bit 和换算后的 16bit
        static int dbg_done = 0;
        if (!dbg_done && g_rms_l > 1200) {
            dbg_done = 1;
            char l1[512], l2[512];
            int p1 = 0, p2 = 0;
            for (int i = 0; i < 16; i++) {
                p1 += snprintf(l1 + p1, sizeof(l1) - p1, "%ld ", (long)g_raw[i * 2]);
                p2 += snprintf(l2 + p2, sizeof(l2) - p2, "%d ", feed[i * 4]);
            }
            ESP_LOGW(TAG, "raw32: %s", l1);
            ESP_LOGW(TAG, "i16  : %s", l2);
        }

        // 旁路唤醒（不经过 AFE）
        if (wnd && wn_chunk == chunksize) {
            if (wn->detect(wnd, feed) == WAKENET_DETECTED) {
                ESP_LOGW(TAG, ">>> 旁路 WakeNet 命中（说明音频和模型都没问题，是 AFE 的锅）");
                wn->clean(wnd);
            }
        }

        afe->feed(afe_data, feed);
        afe_fetch_result_t *res = afe->fetch(afe_data);
        if (!res || res->ret_value == -1) continue;

        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGW(TAG, ">>> 唤醒！(模型 %d)", res->wakenet_model_index);
            if (mn_data) mn->clean(mn_data);
        }

        if (mn_data && res->data) {
            if (mn->detect(mn_data, res->data) == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *r2 = mn->get_results(mn_data);
                if (r2) {
                    int id = r2->command_id[0];
                    for (int i = 0; i < (int)(sizeof(VOICE) / sizeof(VOICE[0])); i++) {
                        if (VOICE[i].id == id) {
                            ESP_LOGW(TAG, ">>> 识别 \"%s\" prob=%.2f", VOICE[i].cn, r2->prob[0]);
                            voice_execute(&VOICE[i]);
                            break;
                        }
                    }
                }
            }
        }

        if (++tick >= 30) {
            tick = 0;
            ESP_LOGI(TAG, "mic L=%4d R=%4d vad=%d clients=%d tx=%d/%d drop=%d ang=%d "
                          "rxb=%lu telem=%lu lerr=%lu alive=%d",
                     peak_l, peak_r, (int)res->vad_state, client_count(),
                     g_tx_ok, g_tx_err, g_drop_state, g_angle,
                     (unsigned long)link_rx_bytes(), (unsigned long)link_telem_count(),
                     (unsigned long)link_error_count(), (int)link_is_alive());
            g_tx_ok = g_tx_err = 0;
            peak_l = peak_r = 0;
        }
    }
}

//-----------------------------------------------------------------------------
// HTTP / WebSocket
//-----------------------------------------------------------------------------
static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start,
                           (ssize_t)(index_html_end - index_html_start - 1));
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        if (client_add(fd)) ESP_LOGI(TAG, "WS 接入 fd=%d，当前 %d 个", fd, client_count());
        return ESP_OK;
    }

    httpd_ws_frame_t pkt = { .type = HTTPD_WS_TYPE_BINARY };
    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);
    if (ret != ESP_OK) return ret;
    if (pkt.len == 0 || pkt.len > 64) return ESP_OK;

    uint8_t buf[64];
    pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &pkt, pkt.len);
    if (ret != ESP_OK || pkt.type != HTTPD_WS_TYPE_BINARY) return ret;

    uint8_t st = 3;
    if (pkt.len >= 6 && buf[0] == 0xAA && buf[1] == 0x55) {
        uint8_t type = buf[2], len = buf[3];
        if (pkt.len >= 6 + len) {
            uint16_t got = (uint16_t)((buf[4 + len] << 8) | buf[5 + len]);
            if (crc16(buf + 2, 2 + len) == got) {
                st = execute_remote(type, buf + 4, len);
            } else {
                st = 1;
            }
        } else {
            st = 2;
        }
    }

    // 转发给 FPGA 时，ACK 由 FPGA 自己回，这里不再重复回
    if (st == ST_FORWARDED) return ESP_OK;

    uint8_t ack_pl[2] = { buf[2], st };
    uint8_t ack[8];
    int len = build_frame(ack, T_ACK, ack_pl, 2);
    httpd_ws_frame_t out = { .type = HTTPD_WS_TYPE_BINARY, .payload = ack, .len = len };
    httpd_ws_send_frame(req, &out);
    return ESP_OK;
}

static void start_http(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = MAX_CLIENTS + 2;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 8192;

    ESP_ERROR_CHECK(httpd_start(&s_server, &cfg));

    const httpd_uri_t u_index = { .uri = "/", .method = HTTP_GET,
                                  .handler = index_get, .user_ctx = NULL };
    const httpd_uri_t u_ws    = { .uri = "/ws", .method = HTTP_GET,
                                  .handler = ws_handler, .user_ctx = NULL,
                                  .is_websocket = true };
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &u_index));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &u_ws));
}

//-----------------------------------------------------------------------------
// WiFi 热点
//-----------------------------------------------------------------------------
static void wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wc = {0};
    const char *ssid = "PID-DEMO", *pass = "12345678";
    strncpy((char *)wc.ap.ssid, ssid, sizeof(wc.ap.ssid) - 1);
    wc.ap.ssid_len = strlen(ssid);
    strncpy((char *)wc.ap.password, pass, sizeof(wc.ap.password) - 1);
    wc.ap.channel = 1;
    wc.ap.ssid_hidden = 0;
    wc.ap.max_connection = 4;
    wc.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 提高发射功率，并打印当前 AP 配置，排除信道/隐藏/认证问题
    esp_err_t tx = esp_wifi_set_max_tx_power(40);
    ESP_LOGI(TAG, "热点: %s / %s  channel=%d hidden=%d txpower=%s",
             ssid, pass, wc.ap.channel, wc.ap.ssid_hidden,
             esp_err_to_name(tx));
}

//-----------------------------------------------------------------------------
void app_main(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    clients_reset();

    i2s_mic_init();
    wifi_ap_start();
    start_http();
    link_init(on_fpga_telem);       // 1 Mbps 链路：FPGA 在线就用真实数据

    xTaskCreatePinnedToCore(telemetry_task, "telem", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(voice_task,     "voice", 8192, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "就绪：手机连 PID-DEMO，打开 http://192.168.4.1");
}
