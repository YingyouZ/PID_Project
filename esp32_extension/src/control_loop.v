module control_loop(
    input  wire                clk_50m,
    input  wire                rst_n,
    input  wire signed [15:0]  adc_angle,   // ADC 角度值
    input  wire signed [15:0]  enc_speed,   // 编码器速度
    input  wire [3:0]          key_sw,      // 按键输入
    output wire signed [15:0]  motor_pwm,   // PID 输出，-100~+100
    output reg                 led_run,     // 运行/起摆指示灯
    output wire signed [15:0]  debug_angle_target,
    output wire signed [15:0]  debug_angle_actual,
    output wire signed [15:0]  debug_angle_out,
    output wire signed [15:0]  debug_loc_target,
    output wire signed [15:0]  debug_loc_actual,
    output wire signed [15:0]  debug_loc_out
);

    localparam signed [15:0] CENTER_ANGLE = 16'sd1856;  // 中心角度
    localparam signed [15:0] CENTER_RANGE = 16'sd500;   // 安全角度范围
    localparam signed [15:0] ENTRY_RANGE  = 16'sd250;   // 进入 PID 的窗口
    localparam signed [15:0] SWING_RANGE  = 16'sd200;   // 起摆峰值检测门限（判断左右侧）
    localparam signed [15:0] START_PWM    = 16'sd48;    // 起摆脉冲幅值
    localparam        [15:0] START_TIME   = 16'd95;    // 起摆脉冲时间
    localparam        [15:0] KICK_TIMEOUT = 16'd190;   //  没检测到峰值就补踢

    // ---------------- 状态定义 ----------------
    localparam [2:0] S_STOP   = 3'd0;
    localparam [2:0] S_JUDGE  = 3'd1;
    localparam [2:0] S_PULSE1 = 3'd2;
    localparam [2:0] S_WAIT1  = 3'd3;
    localparam [2:0] S_PULSE2 = 3'd4;
    localparam [2:0] S_WAIT2  = 3'd5;
    localparam [2:0] S_PID    = 3'd6;
    localparam [2:0] S_RECOVER= 3'd7;

    // ---------------- 1ms 定时 ----------------
    reg [15:0] cnt_1ms;
    wire       tick_1ms;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            cnt_1ms <= 16'd0;
        else if (cnt_1ms == 16'd49999)
            cnt_1ms <= 16'd0;
        else
            cnt_1ms <= cnt_1ms + 1'b1;
    end
    assign tick_1ms = (cnt_1ms == 16'd49999);

    // ---------------- 内部信号 ----------------
    reg  [2:0]         state;
    reg                swing_dir;    // 1: 正向起摆, 0: 反向起摆
    reg  [15:0]        count_time;
    reg  [15:0]        cnt_40ms, cnt_3ms, cnt_25ms;
    reg  [3:0]         safety_cnt;
    reg  [3:0]         entry_cnt;
    reg                auto_mode;
    reg  [15:0]        recover_time;
    reg  [15:0]        no_peak_cnt;
    reg  signed [15:0] ang0, ang1, ang2;
    reg  signed [15:0] angle_r;
    reg  signed [15:0] angle_pid_actual;
    reg  signed [15:0] loc_pid_actual;
    reg  signed [15:0] location;
    reg  signed [15:0] ang_target, loc_target;
    reg  signed [15:0] motor_pwm_r;
    wire signed [15:0] ang_pid_out, loc_pid_out;

    // PID 状态下直接输出 ang_pid_out，避免 motor_pwm_r 额外延迟 3ms
    assign motor_pwm = (state == S_PID) ? ang_pid_out : motor_pwm_r;

    // ---------------- 按键同步和边沿检测 ----------------
    reg  [3:0] key_sync, key_last;
    wire [3:0] key_posedge;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            key_sync <= 4'd0;
            key_last <= 4'd0;
        end else begin
            key_sync <= key_sw;
            key_last <= key_sync;
        end
    end
    assign key_posedge = key_sync & ~key_last;

    // ---------------- 起摆状态机 ----------------
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            // 上电停在停止状态，等待 SW1 启动
            state        <= S_STOP;
            swing_dir    <= 1'b1;
            count_time   <= 16'd0;
            cnt_40ms     <= 16'd0;
            cnt_3ms      <= 16'd0;
            cnt_25ms     <= 16'd0;
            safety_cnt   <= 4'd0;
            entry_cnt    <= 4'd0;
            auto_mode    <= 1'b0;
            recover_time <= 16'd0;
            no_peak_cnt  <= 16'd0;
            ang0         <= 16'sd0;
            ang1         <= 16'sd0;
            ang2         <= 16'sd0;
            angle_r      <= 16'sd0;
            angle_pid_actual <= 16'sd0;
            loc_pid_actual   <= 16'sd0;
            location     <= 16'sd0;
            ang_target   <= CENTER_ANGLE;
            loc_target   <= 16'sd0;
            motor_pwm_r  <= 16'sd0;
            led_run      <= 1'b0;
        end else begin
            if (tick_1ms) begin
                angle_r  <= adc_angle;
                location <= location + enc_speed;

                case (state)
                    // ---------------- 停止 ----------------
                    S_STOP: begin
                        motor_pwm_r <= 16'sd0;
                        led_run     <= 1'b0;
                        ang_target  <= CENTER_ANGLE;
                    end

                    // ---------------- 判断摆杆位置 ----------------
                    S_JUDGE: begin
                        motor_pwm_r <= 16'sd0;
                        led_run     <= 1'b1;

                        // KICK_TIMEOUT 内没检测到峰值，补一次脉冲，避免停摆
                        if (no_peak_cnt >= KICK_TIMEOUT - 1'b1) begin
                            no_peak_cnt <= 16'd0;
                            swing_dir   <= ~swing_dir;
                            state       <= S_PULSE1;
                        end else begin
                            no_peak_cnt <= no_peak_cnt + 1'b1;
                            cnt_40ms    <= cnt_40ms + 1'b1;

                            if (cnt_40ms >= 16'd39) begin   //40ms采样一次角度，判断角度处于采样最高点
                                cnt_40ms <= 16'd0;
                                ang2 <= ang1;
                                ang1 <= ang0;
                                ang0 <= angle_r;

                                // 高侧局部最高点 -> 正向起摆
                                if (ang0 > CENTER_ANGLE + SWING_RANGE &&
                                    ang1 > CENTER_ANGLE + SWING_RANGE &&
                                    ang2 > CENTER_ANGLE + SWING_RANGE &&
                                    ang1 > ang0 && ang1 > ang2) begin
                                    swing_dir   <= 1'b1;
                                    no_peak_cnt <= 16'd0;
                                    state       <= S_PULSE1;
                                end
                                // 低侧局部最低点 -> 反向起摆
                                else if (ang0 < CENTER_ANGLE - SWING_RANGE &&
                                         ang1 < CENTER_ANGLE - SWING_RANGE &&
                                         ang2 < CENTER_ANGLE - SWING_RANGE &&
                                         ang1 < ang0 && ang1 < ang2) begin
                                    swing_dir   <= 1'b0;
                                    no_peak_cnt <= 16'd0;
                                    state       <= S_PULSE1;
                                end
                                // 已经进入中心窗口 -> 切换到 PID 平衡
                                else if (ang0 > CENTER_ANGLE - CENTER_RANGE &&
                                         ang0 < CENTER_ANGLE + CENTER_RANGE &&
                                         ang1 > CENTER_ANGLE - CENTER_RANGE &&
                                         ang1 < CENTER_ANGLE + CENTER_RANGE) begin
                                    location    <= 16'sd0;
                                    angle_pid_actual <= angle_r;
                                    loc_pid_actual   <= 16'sd0;
                                    ang_target  <= CENTER_ANGLE;
                                    cnt_3ms     <= 16'd0;
                                    cnt_25ms    <= 16'd0;
                                    safety_cnt  <= 4'd0;
                                    no_peak_cnt <= 16'd0;
                                    state       <= S_PID;
                                end
                            end
                        end

                        // 1ms 快速判断：连续 5ms 在中心窗口内就进入 PID
                        if (angle_r > CENTER_ANGLE - ENTRY_RANGE &&
                            angle_r < CENTER_ANGLE + ENTRY_RANGE) begin
                            if (entry_cnt >= 4'd4) begin
                                location    <= 16'sd0;
                                    angle_pid_actual <= angle_r;
                                    loc_pid_actual   <= 16'sd0;
                                ang_target  <= CENTER_ANGLE;
                                cnt_3ms     <= 16'd0;
                                cnt_25ms    <= 16'd0;
                                safety_cnt  <= 4'd0;
                                no_peak_cnt <= 16'd0;
                                entry_cnt   <= 4'd0;
                                state       <= S_PID;
                            end else begin
                                entry_cnt <= entry_cnt + 1'b1;
                            end
                        end else begin
                            entry_cnt <= 4'd0;
                        end
                    end

//S_PULSE1、S_WAIT1、S_PULSE2、S_WAIT2 serve as a short-term shake together
                       // ---------------- 起摆脉冲 1 ----------------
                      S_PULSE1: begin
                           led_run     <= 1'b1;
                        motor_pwm_r <= swing_dir ? START_PWM : -START_PWM;
                        count_time  <= START_TIME;
                        state       <= S_WAIT1;
                    end

                    S_WAIT1: begin
                        if (count_time == 16'd0)
                            state <= S_PULSE2;
                        else
                            count_time <= count_time - 1'b1;
                    end

                    // ---------------- 起摆脉冲 2 ----------------
                    S_PULSE2: begin
                        motor_pwm_r <= swing_dir ? -START_PWM : START_PWM;
                        count_time  <= START_TIME;
                        state       <= S_WAIT2;
                    end

                    S_WAIT2: begin
                        if (count_time == 16'd0) begin
                            motor_pwm_r <= 16'sd0;
                            ang0        <= 16'sd0;
                            ang1        <= 16'sd0;
                            ang2        <= 16'sd0;
                            cnt_40ms    <= 16'd0;
                            no_peak_cnt <= 16'd0;
                            entry_cnt   <= 4'd0;
                            state       <= S_JUDGE;
                        end else begin
                            count_time <= count_time - 1'b1;
                        end
                    end

                    // ---------------- PID 平衡 ----------------
                    S_PID: begin
                        led_run <= 1'b1;

                        // 安全角度：连续 10ms 超出窗口才停止，避免单次 ADC 毛刺误停
                        if (!(angle_r > CENTER_ANGLE - CENTER_RANGE &&
                              angle_r < CENTER_ANGLE + CENTER_RANGE)) begin
                            if (safety_cnt >= 4'd9) begin
                                motor_pwm_r <= 16'sd0;
                                led_run     <= 1'b0;
                                if (auto_mode) begin
                                    state        <= S_RECOVER;
                                    recover_time <= 16'd500;
                                end else begin
                                    state <= S_STOP;
                                end
                            end else begin
                                safety_cnt <= safety_cnt + 1'b1;
                            end
                        end else begin
                            safety_cnt <= 4'd0;
                        end

                        // 角度环 3ms
                        cnt_3ms <= cnt_3ms + 1'b1;
                        if (cnt_3ms >= 16'd2) begin
                            cnt_3ms          <= 16'd0;
                            angle_pid_actual <= adc_angle;
                            motor_pwm_r      <= ang_pid_out;
                        end

                        // 位置环 25ms
                        cnt_25ms <= cnt_25ms + 1'b1;
                        if (cnt_25ms >= 16'd24) begin
                            cnt_25ms       <= 16'd0;
                            loc_pid_actual <= location;
                            ang_target     <= CENTER_ANGLE + loc_pid_out;
                        end
                    end

                    // ---------------- 倒下后自动恢复 ----------------
                    S_RECOVER: begin
                        motor_pwm_r <= 16'sd0;
                        led_run     <= 1'b0;
                        ang_target  <= CENTER_ANGLE;
                        if (recover_time == 16'd0) begin
                            state       <= S_PULSE1;
                            swing_dir   <= 1'b1;
                            count_time  <= 16'd0;
                            cnt_40ms    <= 16'd0;
                            cnt_3ms     <= 16'd0;
                            cnt_25ms    <= 16'd0;
                            safety_cnt  <= 4'd0;
                            no_peak_cnt <= 16'd0;
                            entry_cnt   <= 4'd0;
                            ang0        <= 16'sd0;
                            ang1        <= 16'sd0;
                            ang2        <= 16'sd0;
                        end else begin
                            recover_time <= recover_time - 1'b1;
                        end
                    end

                    default: state <= S_STOP;
                endcase
            end

            // ---------------- 按键处理 ----------------
            // SW1：第一次按下启动自动起摆，再按一次停止
            if (key_posedge[0]) begin
                if (!auto_mode) begin
                    auto_mode    <= 1'b1;
                    recover_time <= 16'd0;
                    state       <= S_PULSE1;
                    swing_dir   <= 1'b1;
                    count_time  <= 16'd0;
                    cnt_40ms    <= 16'd0;
                    cnt_3ms     <= 16'd0;
                    cnt_25ms    <= 16'd0;
                    safety_cnt  <= 4'd0;
                    no_peak_cnt <= 16'd0;
                    entry_cnt   <= 4'd0;
                    ang0        <= 16'sd0;
                    ang1        <= 16'sd0;
                    ang2        <= 16'sd0;
                    led_run     <= 1'b1;
                end else begin
                    auto_mode    <= 1'b0;
                    recover_time <= 16'd0;
                    state        <= S_STOP;
                    motor_pwm_r  <= 16'sd0;
                    led_run      <= 1'b0;
                    count_time   <= 16'd0;
                    cnt_40ms     <= 16'd0;
                    cnt_3ms      <= 16'd0;
                    cnt_25ms     <= 16'd0;
                    no_peak_cnt  <= 16'd0;
                    ang0         <= 16'sd0;
                    ang1         <= 16'sd0;
                    ang2         <= 16'sd0;
                end
            end

            // SW2：位置目标 +532
            if (key_posedge[1]) begin
                if (loc_target + 16'sd532 > 16'sd4000)
                    loc_target <= 16'sd4000;
                else
                    loc_target <= loc_target + 16'sd532;
            end

            // SW4：位置目标 -532
            if (key_posedge[3]) begin
                if (loc_target - 16'sd532 < -16'sd4000)
                    loc_target <= -16'sd4000;
                else
                    loc_target <= loc_target - 16'sd532;
            end
        end
    end

    // ---------------- 角度环 PID：3ms ----------------
    pid #(
        .KP(1560), .KI(55), .KD(3660),
        .OUT_MAX(100), .OUT_MIN(-100),
        .INT_MAX(30), .INT_BAND(50)
    ) angle_pid_inst (
        .clk       (clk_50m),
        .rst_n     (rst_n),
        .update_en (tick_1ms && (cnt_3ms == 16'd2)),
        .target    (ang_target),
        .actual    (adc_angle),
        .out       (ang_pid_out)
    );

    // ---------------- 位置环 PID：25ms ----------------
    pid #(
        .KP(1100), .KI(5), .KD(16000),
        .OUT_MAX(90), .OUT_MIN(-90)
    ) location_pid_inst (
        .clk       (clk_50m),
        .rst_n     (rst_n),
        .update_en (tick_1ms && (cnt_25ms == 16'd24)),
        .target    (loc_target),
        .actual    (loc_pid_actual),
        .out       (loc_pid_out)
    );

    // 16 字节调试串口数据
    assign debug_angle_target = ang_target;
    assign debug_angle_actual = angle_r;
    assign debug_angle_out    = ang_pid_out;
    assign debug_loc_target   = loc_target;
    assign debug_loc_actual   = location;
    assign debug_loc_out      = loc_pid_out;

endmodule
