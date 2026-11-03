`timescale 1ns/1ns
//  top.v - J280 倒立摆远控顶层
//  50 MHz；ADC/编码器采集、串级 PID、自动起摆、link 遥测
module top(
    input        clk_50m,
    input        rst_n,

    input  [3:0] key_sw,

    output       uart_tx,
    input        uart_rx,
    output       led_run,

    output       adc_clk,
    output       adc_oe_n,
    input  [9:0] adc_data_in,

    input        enc1_a,
    input        enc1_b,
    input        enc2_a,
    input        enc2_b,

    output       AN1,
    output       AN2,
    output       PWMA,

    output       BN1,
    output       BN2,
    output       PWMB
);

    //按键消抖
    wire [3:0] swing_key;
    genvar i;
    generate
        for (i = 0; i <= 3; i = i + 1) begin : g_key
            key_filter #(
                .CNT_MAX (20'd1000_000)
            ) key_filter_inst (
                .sys_clk   (clk_50m),
                .sys_rst_n (rst_n),
                .key_in    (key_sw[i]),
                .key_flag  (swing_key[i])
            );
        end
    endgenerate

    //ADC 角度
    wire [11:0] theta_out;

    adc adc_inst (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .adc_clk     (adc_clk),
        .adc_oe_n    (adc_oe_n),
        .adc_data_in (adc_data_in),
        .theta_out   (theta_out)
    );

    wire signed [15:0] theta_signed = {4'b0, theta_out};

    //编码器
    wire [15:0] enc_pos1, enc_pos2;

    encoder encoder_1 (
        .clk_50m (clk_50m),
        .rst_n   (rst_n),
        .enc_a   (enc1_a),
        .enc_b   (enc1_b),
        .pos     (enc_pos1)
    );

    encoder encoder_2 (
        .clk_50m (clk_50m),
        .rst_n   (rst_n),
        .enc_a   (enc2_a),
        .enc_b   (enc2_b),
        .pos     (enc_pos2)
    );

    // 每 1ms 采样一次编码器位置，作差得到速度
    reg  [15:0]       cnt_1ms_spd;
    wire              tick_spd;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                     cnt_1ms_spd <= 16'd0;
        else if (cnt_1ms_spd == 16'd49999) cnt_1ms_spd <= 16'd0;
        else                            cnt_1ms_spd <= cnt_1ms_spd + 1'b1;
    end
    assign tick_spd = (cnt_1ms_spd == 16'd49999);

    reg  [15:0]       last_enc_pos_1ms;
    reg  signed [15:0] enc_speed_1ms;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            last_enc_pos_1ms <= 16'd10000;
            enc_speed_1ms    <= 16'sd0;
        end else if (tick_spd) begin
            enc_speed_1ms    <= $signed(enc_pos1) - $signed(last_enc_pos_1ms);
            last_enc_pos_1ms <= enc_pos1;
        end
    end

    //控制 + 链路
    wire signed [15:0] pwm_out;

    wire [1:0]  mode;
    wire signed [15:0] angle_target_cmd;
    wire signed [15:0] loc_target_cmd;
    wire        loc_we;
    wire signed [15:0] manual_pwm;
    wire [1:0]  stream_rate;
    wire [3:0]  vkey;
    wire        cmd_stb;
    wire        fault;
    wire        link_ok;

    wire signed [15:0] dbg_angle, dbg_target, dbg_speed, dbg_loc, dbg_loc_target;
    wire signed [15:0] dbg_angle_out, dbg_loc_out;
    wire [2:0]  dbg_state;
    wire        dbg_running;

    assign dbg_speed   = enc_speed_1ms;
    assign dbg_running = led_run;

    // 控制器：control_loop + 远程模式/目标/虚拟按键
    control_loop_ctrl control_loop_inst (
        .clk_50m            (clk_50m),
        .rst_n              (rst_n),
        .adc_angle          (theta_signed),
        .enc_speed          (enc_speed_1ms),
        .key_sw             (swing_key),
        .mode               (mode),
        .angle_target_cmd   (angle_target_cmd),
        .loc_target_cmd     (loc_target_cmd),
        .loc_we             (loc_we),
        .manual_pwm         (manual_pwm),
        .vkey               (vkey),
        .cmd_stb            (cmd_stb),
        .motor_pwm          (pwm_out),
        .led_run            (led_run),
        .debug_angle_target (dbg_target),
        .debug_angle_actual (dbg_angle),
        .debug_angle_out    (dbg_angle_out),
        .debug_loc_target   (dbg_loc_target),
        .debug_loc_actual   (dbg_loc),
        .debug_loc_out      (dbg_loc_out),
        .debug_state        (dbg_state)
    );

    link #(
        .CLK_FREQ (50_000_000),
        .UART_BPS (1_000_000)
    ) link_inst (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
        .uart_rx      (uart_rx),
        .uart_tx      (uart_tx),

        .angle_in     (dbg_angle),
        .target_in    (dbg_target),
        .pwm_in       (pwm_out),
        .enc1_in      (enc_pos1),
        .enc2_in      (enc_pos2),
        .speed_in     (enc_speed_1ms),
        .loc_in       (dbg_loc),
        .loc_target_in(dbg_loc_target),
        .state_in     (dbg_state),
        .run_in       (dbg_running),

        .mode             (mode),
        .angle_target     (angle_target_cmd),
        .loc_target       (loc_target_cmd),
        .loc_we           (loc_we),
        .manual_pwm       (manual_pwm),
        .stream_rate      (stream_rate),
        .vkey             (vkey),
        .cmd_stb          (cmd_stb),
        .fault            (fault),
        .link_ok          (link_ok)
    );

    //PID 输出 --电机驱动
    wire [1:0] motor_dir;
    wire [7:0] motor_duty;

    wire signed [15:0] pwm_clip = (pwm_out > 16'sd100)  ? 16'sd100 :
                                  (pwm_out < -16'sd100) ? -16'sd100 : pwm_out;
    wire [15:0] pwm_abs = pwm_clip[15] ? (~pwm_clip + 16'd1) : pwm_clip;

    assign motor_duty = (pwm_abs * 16'd255) / 16'd100;
    assign motor_dir  = pwm_clip[15] ? 2'b10 : 2'b01;

    motor_ctrl motor_ctrl_A (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .motor_a_sel (motor_dir),
        .duty_out    (motor_duty),
        .ain1        (AN1),
        .ain2        (AN2),
        .pwm         (PWMA)
    );

    motor_ctrl motor_ctrl_B (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .motor_a_sel (motor_dir),
        .duty_out    (motor_duty),
        .ain1        (BN1),
        .ain2        (BN2),
        .pwm         (PWMB)
    );

endmodule
