`timescale 1ns/1ns
//  link.v - FPGA 与 ESP32 二进制链路
//  1 Mbps 8N1，帧头 AA55，CRC16-CCITT-FALSE
//  下行：模式/角度/位置/PWM/遥测率/PING/清故障/心跳/步进/启动/急停
//  上行：遥测 20 B、PONG、ACK

module link #(
    parameter        CLK_FREQ   = 50_000_000,
    parameter        UART_BPS   = 1_000_000,
    parameter [15:0] WDT_MS     = 16'd1000,     // 1 s 无有效帧后停机
    parameter [15:0] PWM_WDT_MS = 16'd200       // 手动 PWM 200 ms 超时归零
) (
    input  wire               clk_50m,
    input  wire               rst_n,

    input  wire               uart_rx,
    output wire               uart_tx,

    // 遥测快照输入
    input  wire signed [15:0] angle_in,
    input  wire signed [15:0] target_in,
    input  wire signed [15:0] pwm_in,
    input  wire        [15:0] enc1_in,
    input  wire        [15:0] enc2_in,
    input  wire signed [15:0] speed_in,
    input  wire signed [15:0] loc_in,
    input  wire signed [15:0] loc_target_in,
    input  wire         [2:0] state_in,
    input  wire               run_in,

    //命令输出
    output reg         [1:0]  mode,             // 0 stop 1 auto 2 hold 3 manual
    output reg  signed [15:0] angle_target,     // HOLD 模式目标
    output reg  signed [15:0] loc_target,       // 与 loc_we 配合
    output reg                loc_we,
    output reg  signed [15:0] manual_pwm,
    output reg         [1:0]  stream_rate,      // 0 off 1=100Hz 2=500Hz
    output reg         [3:0]  vkey,             // 虚拟按键单周期脉冲
    output reg                cmd_stb,          // 有效指令脉冲
    output reg                fault,
    output reg                link_ok
);

    // CRC16 计算（单字节）
    function [15:0] crc16_byte;
        input [15:0] c;
        input [7:0]  d;
        integer i;
        reg [15:0] x;
        begin
            x = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc16_byte = x;
        end
    endfunction

    // 命令限幅
    function signed [15:0] clamp_angle;
        input signed [15:0] v;
        begin
            if (v < 16'sd0)         clamp_angle = 16'sd0;
            else if (v > 16'sd4095) clamp_angle = 16'sd4095;
            else                    clamp_angle = v;
        end
    endfunction

    function signed [15:0] clamp_pwm;
        input signed [15:0] v;
        begin
            if (v > 16'sd100)       clamp_pwm = 16'sd100;
            else if (v < -16'sd100) clamp_pwm = -16'sd100;
            else                    clamp_pwm = v;
        end
    endfunction

    function signed [15:0] clamp_loc;
        input signed [15:0] v;
        begin
            if (v > 16'sd5320)       clamp_loc = 16'sd5320;
            else if (v < -16'sd5320) clamp_loc = -16'sd5320;
            else                     clamp_loc = v;
        end
    endfunction

    // 1ms 节拍
    reg  [15:0] cnt_1ms;
    wire        tick_1ms;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                    cnt_1ms <= 16'd0;
        else if (cnt_1ms >= 16'd49999) cnt_1ms <= 16'd0;
        else                           cnt_1ms <= cnt_1ms + 1'b1;
    end
    assign tick_1ms = (cnt_1ms == 16'd49999);

    // 物理层 UART
    wire [7:0] u_rx_data;
    wire       u_rx_flag;
    wire [7:0] u_tx_data;
    wire       u_tx_flag;
    wire       u_tx_done;

    uart_rx #(
        .UART_BPS (UART_BPS),
        .CLK_FREQ (CLK_FREQ)
    ) u_rx (
        .sys_clk   (clk_50m),
        .sys_rst_n (rst_n),
        .rx        (uart_rx),
        .po_data   (u_rx_data),
        .po_flag   (u_rx_flag)
    );

    uart_tx #(
        .UART_BPS (UART_BPS),
        .CLK_FREQ (CLK_FREQ)
    ) u_tx (
        .sys_clk   (clk_50m),
        .sys_rst_n (rst_n),
        .pi_data   (u_tx_data),
        .pi_flag   (u_tx_flag),
        .tx        (uart_tx),
        .pi_done   (u_tx_done)
    );

    // 接收组帧
    localparam [2:0] R_HUNT = 3'd0,
                     R_H2   = 3'd1,
                     R_CMD  = 3'd2,
                     R_LEN  = 3'd3,
                     R_DATA = 3'd4,
                     R_CRCH = 3'd5,
                     R_CRCL = 3'd6;

    reg  [2:0]  rstate;
    reg  [7:0]  rx_cmd, rx_len, rx_idx, rx_crc_h;
    reg  [7:0]  pl [0:7];
    reg  [15:0] rxcrc;

    reg         ack_req;            // ACK 请求
    reg  [7:0]  ack_req_cmd;
    reg  [7:0]  ack_req_status;
    reg         pong_req;
    reg  [31:0] pong_req_nonce;

    reg         remote_armed;
    reg  [15:0] wdt_ms;
    reg  [15:0] pwm_wdt_ms;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            rstate         <= R_HUNT;
            rx_cmd         <= 8'd0;
            rx_len         <= 8'd0;
            rx_idx         <= 8'd0;
            rx_crc_h       <= 8'd0;
            rxcrc          <= 16'hFFFF;
            ack_req        <= 1'b0;
            ack_req_cmd    <= 8'd0;
            ack_req_status <= 8'd0;
            pong_req       <= 1'b0;
            pong_req_nonce <= 32'd0;
            mode           <= 2'd0;
            angle_target   <= 16'sd2010;
            loc_target     <= 16'sd0;
            loc_we         <= 1'b0;
            manual_pwm     <= 16'sd0;
            stream_rate    <= 2'd1;
            vkey           <= 4'd0;
            cmd_stb        <= 1'b0;
            fault          <= 1'b0;
            link_ok        <= 1'b0;
            remote_armed   <= 1'b0;
            wdt_ms         <= 16'd1000;
            pwm_wdt_ms     <= 16'd1000;
        end else begin
            loc_we  <= 1'b0;
            vkey    <= 4'd0;
            cmd_stb <= 1'b0;
            ack_req <= 1'b0;
            pong_req<= 1'b0;

            // 字节级组帧
            if (u_rx_flag) begin
                case (rstate)
                    R_HUNT: if (u_rx_data == 8'hAA) rstate <= R_H2;

                    R_H2: begin
                        if (u_rx_data == 8'h55)      rstate <= R_CMD;
                        else if (u_rx_data != 8'hAA) rstate <= R_HUNT;
                    end

                    R_CMD: begin
                        rx_cmd <= u_rx_data;
                        rxcrc  <= crc16_byte(16'hFFFF, u_rx_data);
                        rstate <= R_LEN;
                    end

                    R_LEN: begin
                        rx_len <= u_rx_data;
                        rx_idx <= 8'd0;
                        if (u_rx_data == 8'd0) begin
                            rxcrc  <= crc16_byte(rxcrc, u_rx_data);
                            rstate <= R_CRCH;
                        end else if (u_rx_data > 8'd8) begin
                            ack_req        <= 1'b1;
                            ack_req_cmd    <= rx_cmd;
                            ack_req_status <= 8'd2;          // 长度错误
                            rstate         <= R_HUNT;
                        end else begin
                            rxcrc  <= crc16_byte(rxcrc, u_rx_data);
                            rstate <= R_DATA;
                        end
                    end

                    R_DATA: begin
                        pl[rx_idx] <= u_rx_data;
                        rxcrc      <= crc16_byte(rxcrc, u_rx_data);
                        if (rx_idx == rx_len - 8'd1) rstate <= R_CRCH;
                        else                         rx_idx <= rx_idx + 8'd1;
                    end

                    R_CRCH: begin
                        rx_crc_h <= u_rx_data;
                        rstate   <= R_CRCL;
                    end

                    R_CRCL: begin
                        rstate       <= R_HUNT;
                        ack_req      <= 1'b1;
                        ack_req_cmd  <= rx_cmd;
                        wdt_ms       <= 16'd0;
                        remote_armed <= 1'b1;

                        if ({rx_crc_h, u_rx_data} != rxcrc) begin
                            ack_req_status <= 8'd1;          // CRC 错误
                        end else begin
                            ack_req_status <= 8'd0;
                            cmd_stb <= 1'b1;      // 有效指令
                            case (rx_cmd)
                                8'h01: begin
                                    mode <= pl[0][1:0];
                                    if (pl[0][1:0] == 2'd0) manual_pwm <= 16'sd0;
                                end
                                8'h02: angle_target <= clamp_angle({pl[1], pl[0]});
                                8'h03: begin
                                    loc_target <= clamp_loc({pl[1], pl[0]});
                                    loc_we     <= 1'b1;
                                end
                                8'h04: begin
                                    manual_pwm <= clamp_pwm({pl[1], pl[0]});
                                    pwm_wdt_ms <= 16'd0;
                                end
                                8'h05: ;                         // 预留
                                8'h06: ;                         // 预留
                                8'h07: stream_rate <= (pl[0] > 8'd2) ? 2'd2 : pl[0][1:0];
                                8'h08: begin
                                    pong_req       <= 1'b1;
                                    pong_req_nonce <= {pl[3], pl[2], pl[1], pl[0]};
                                end
                                8'h09: fault <= 1'b0;
                                8'h0A: ;                         // 心跳
                                8'h0B: begin
                                    // 按 int16 符号位判断方向
                                    
                                    if (pl[1][7])                     vkey[3] <= 1'b1;  // 负：SW4，-532
                                    else if (pl[1] != 8'd0 || pl[0] != 8'd0)
                                                                      vkey[1] <= 1'b1;  // 正：SW2，+532
                                end
                                // 启动：切 AUTO，必要时模拟 SW1
                                8'h0C: begin
                                    mode <= 2'd1;
                                    if (state_in == 3'd0) vkey[0] <= 1'b1;
                                end
                                // 急停
                                8'h0D: begin
                                    mode       <= 2'd0;
                                    manual_pwm <= 16'sd0;
                                end
                                default: ack_req_status <= 8'd3;  // 未知指令
                            endcase
                        end
                    end

                    default: rstate <= R_HUNT;
                endcase
            end

            // 看门狗
            if (tick_1ms) begin
                if (wdt_ms < 16'd60000)     wdt_ms     <= wdt_ms + 1'b1;
                if (pwm_wdt_ms < 16'd60000) pwm_wdt_ms <= pwm_wdt_ms + 1'b1;

                // 手动 PWM 超时归零
                if (pwm_wdt_ms >= PWM_WDT_MS) manual_pwm <= 16'sd0;

                // 远程接管后 1 s 无帧则停机
                if (remote_armed && (wdt_ms >= WDT_MS)) begin
                    mode       <= 2'd0;
                    manual_pwm <= 16'sd0;
                    fault      <= 1'b1;
                end
            end

            link_ok <= remote_armed && (wdt_ms < 16'd100);
        end
    end

    // 发送组帧
    localparam [7:0] T_TELEM = 8'h81,
                     T_PONG  = 8'h82,
                     T_ACK   = 8'h83;

    localparam [2:0] B_IDLE = 3'd0,
                     B_BODY = 3'd1,
                     B_CRCH = 3'd2,
                     B_CRCL = 3'd3;

    // FIFO 存储器
    reg  [7:0] fifo_mem [0:127];
    reg  [6:0] fifo_wp, fifo_rp;
    reg  [7:0] fifo_cnt;
    reg        tx_busy;

    wire fifo_full  = (fifo_cnt == 8'd128);
    wire fifo_empty = (fifo_cnt == 8'd0);
    wire [7:0] fifo_dout = fifo_mem[fifo_rp];

    // 帧构造状态
    reg  [2:0]  bstate;
    reg  [7:0]  bidx, btype, blen;
    reg  [15:0] bcrc;
    reg  [7:0]  bbyte;
    reg         wr_en;
    reg  [7:0]  wbyte;

    // 待发送队列
    reg         ack_pend, pong_pend, telem_pend;
    reg  [7:0]  ack_cmd, ack_status;
    reg  [31:0] pong_nonce;

    // 遥测快照
    reg  [7:0]  s_seq;
    reg  [3:0]  telem_div;
    reg  signed [15:0] s_angle, s_target, s_pwm, s_speed, s_loc, s_loct;
    reg  [15:0] s_enc1, s_enc2;
    reg  [2:0]  s_state;
    reg  [7:0]  s_flags;

    wire [7:0]  last_data_idx = 8'd3 + blen;

    // 发送侧取走脉冲
    wire ack_take   = (bstate == B_IDLE) && ack_pend;
    wire pong_take  = (bstate == B_IDLE) && !ack_pend && pong_pend;
    wire telem_take = (bstate == B_IDLE) && !ack_pend && !pong_pend && telem_pend;

    // 10 ms 分频：100 Hz / 500 Hz
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)       telem_div <= 4'd0;
        else if (tick_1ms) telem_div <= (telem_div == 4'd9) ? 4'd0 : telem_div + 1'b1;
    end

    wire telem_tick = tick_1ms &&
                      (((stream_rate == 2'd1) && (telem_div == 4'd9)) ||
                       ((stream_rate == 2'd2) && telem_div[0]));

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            bstate      <= B_IDLE;
            bidx        <= 8'd0;
            btype       <= 8'd0;
            blen        <= 8'd0;
            bcrc        <= 16'hFFFF;
            wr_en       <= 1'b0;
            wbyte       <= 8'd0;
            ack_pend    <= 1'b0;
            ack_cmd     <= 8'd0;
            ack_status  <= 8'd0;
            pong_pend   <= 1'b0;
            pong_nonce  <= 32'd0;
            telem_pend  <= 1'b0;
            s_seq       <= 8'd0;
            s_angle     <= 16'sd0;
            s_target    <= 16'sd0;
            s_pwm       <= 16'sd0;
            s_enc1      <= 16'd0;
            s_enc2      <= 16'd0;
            s_speed     <= 16'sd0;
            s_loc       <= 16'sd0;
            s_loct      <= 16'sd0;
            s_state     <= 3'd0;
            s_flags     <= 8'd0;
        end else begin
            wr_en <= 1'b0;

            // 排队
            if (ack_take)       ack_pend   <= 1'b0;
            if (pong_take)      pong_pend  <= 1'b0;
            if (telem_take)     telem_pend <= 1'b0;

            if (ack_req) begin
                ack_pend   <= 1'b1;
                ack_cmd    <= ack_req_cmd;
                ack_status <= ack_req_status;
            end
            if (pong_req) begin
                pong_pend  <= 1'b1;
                pong_nonce <= pong_req_nonce;
            end

            // 遥测快照
            if (telem_tick && !telem_pend) begin
                s_seq      <= s_seq + 8'd1;
                s_angle    <= angle_in;
                s_target   <= target_in;
                s_pwm      <= pwm_in;
                s_enc1     <= enc1_in;
                s_enc2     <= enc2_in;
                s_speed    <= speed_in;
                s_loc      <= loc_in;
                s_loct     <= loc_target_in;
                s_state    <= state_in;
                s_flags    <= {4'd0, remote_armed, link_ok, fault, run_in};
                telem_pend <= 1'b1;
            end

            // 帧构造
            case (bstate)
                B_IDLE: begin
                    bidx <= 8'd0;
                    bcrc <= 16'hFFFF;
                    if (ack_pend) begin
                        btype  <= T_ACK;
                        blen   <= 8'd2;
                        bstate <= B_BODY;
                    end else if (pong_pend) begin
                        btype  <= T_PONG;
                        blen   <= 8'd5;
                        bstate <= B_BODY;
                    end else if (telem_pend) begin
                        btype  <= T_TELEM;
                        blen   <= 8'd20;
                        bstate <= B_BODY;
                    end
                end

                B_BODY: begin
                    if (!fifo_full) begin
                        wr_en <= 1'b1;
                        wbyte <= bbyte;
                        if (bidx >= 8'd2) bcrc <= crc16_byte(bcrc, bbyte);
                        if (bidx == last_data_idx) bstate <= B_CRCH;
                        else                       bidx   <= bidx + 8'd1;
                    end
                end

                B_CRCH: begin
                    if (!fifo_full) begin
                        wr_en  <= 1'b1;
                        wbyte  <= bcrc[15:8];
                        bstate <= B_CRCL;
                    end
                end

                B_CRCL: begin
                    if (!fifo_full) begin
                        wr_en  <= 1'b1;
                        wbyte  <= bcrc[7:0];
                        bstate <= B_IDLE;
                    end
                end

                default: bstate <= B_IDLE;
            endcase
        end
    end

    // 当前要发送的字节
    always @(*) begin
        case (bidx)
            8'd0: bbyte = 8'hAA;
            8'd1: bbyte = 8'h55;
            8'd2: bbyte = btype;
            8'd3: bbyte = blen;
            default: begin
                case (btype)
                    T_TELEM: begin
                        case (bidx - 8'd4)
                            8'd0:  bbyte = s_seq;
                            8'd1:  bbyte = {5'd0, s_state};
                            8'd2:  bbyte = s_flags;
                            8'd3:  bbyte = {6'd0, stream_rate};
                            8'd4:  bbyte = s_angle[15:8];
                            8'd5:  bbyte = s_angle[7:0];
                            8'd6:  bbyte = s_target[15:8];
                            8'd7:  bbyte = s_target[7:0];
                            8'd8:  bbyte = s_pwm[15:8];
                            8'd9:  bbyte = s_pwm[7:0];
                            8'd10: bbyte = s_enc1[15:8];
                            8'd11: bbyte = s_enc1[7:0];
                            8'd12: bbyte = s_enc2[15:8];
                            8'd13: bbyte = s_enc2[7:0];
                            8'd14: bbyte = s_speed[15:8];
                            8'd15: bbyte = s_speed[7:0];
                            8'd16: bbyte = s_loc[15:8];
                            8'd17: bbyte = s_loc[7:0];
                            8'd18: bbyte = s_loct[15:8];
                            8'd19: bbyte = s_loct[7:0];
                            default: bbyte = 8'h00;
                        endcase
                    end
                    T_PONG: begin
                        case (bidx - 8'd4)
                            8'd0: bbyte = pong_nonce[31:24];
                            8'd1: bbyte = pong_nonce[23:16];
                            8'd2: bbyte = pong_nonce[15:8];
                            8'd3: bbyte = pong_nonce[7:0];
                            8'd4: bbyte = 8'h01;              // 协议版本
                            default: bbyte = 8'h00;
                        endcase
                    end
                    T_ACK: begin
                        case (bidx - 8'd4)
                            8'd0: bbyte = ack_cmd;
                            8'd1: bbyte = ack_status;
                            default: bbyte = 8'h00;
                        endcase
                    end
                    default: bbyte = 8'h00;
                endcase
            end
        endcase
    end

    // ---------------- FIFO ----------------
    wire wr_ok = wr_en && !fifo_full;
    wire rd_en = !tx_busy && !fifo_empty;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            fifo_wp  <= 7'd0;
            fifo_rp  <= 7'd0;
            fifo_cnt <= 8'd0;
        end else begin
            if (wr_ok) fifo_mem[fifo_wp] <= wbyte;
            if (wr_ok) fifo_wp <= fifo_wp + 7'd1;
            if (rd_en) fifo_rp <= fifo_rp + 7'd1;

            if (wr_ok && rd_en) fifo_cnt <= fifo_cnt;
            else if (wr_ok)     fifo_cnt <= fifo_cnt + 8'd1;
            else if (rd_en)     fifo_cnt <= fifo_cnt - 8'd1;
        end
    end

    // 串口发送仲裁
    reg  [7:0] tx_byte;
    reg        tx_start;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tx_busy  <= 1'b0;
            tx_start <= 1'b0;
            tx_byte  <= 8'd0;
        end else begin
            tx_start <= 1'b0;

            if (u_tx_done) tx_busy <= 1'b0;

            if (!tx_busy && !fifo_empty) begin
                tx_byte  <= fifo_dout;
                tx_start <= 1'b1;
                tx_busy  <= 1'b1;
            end
        end
    end

    assign u_tx_data = tx_byte;
    assign u_tx_flag = tx_start;

endmodule
