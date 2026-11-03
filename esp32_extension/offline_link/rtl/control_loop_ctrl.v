module control_loop_ctrl(
    input  wire                clk_50m,
    input  wire                rst_n,
    input  wire signed [15:0]  adc_angle,   // ADC 角度值
    input  wire signed [15:0]  enc_speed,   // 编码器速度
    input  wire [3:0]          key_sw,      // 按键输入

    //远程控制接口
    input  wire [1:0]          mode,             // 0停止 1自动起摆 2角度保持 3手动PWM
    input  wire signed [15:0]  angle_target_cmd, // 保持模式的角度目标
    input  wire signed [15:0]  loc_target_cmd,   // 绝对位置目标
    input  wire                loc_we,           // 位置目标写入脉冲
    input  wire signed [15:0]  manual_pwm,       // 手动 PWM
    input  wire [3:0]          vkey,             // 远程虚拟按键（与物理按键等价）
    input  wire                cmd_stb,          // 收到有效远程指令的脉冲

    output wire signed [15:0]  motor_pwm,   // PID 输出，-100~+100
    output reg                 led_run,     // 运行/起摆指示灯
    output wire signed [15:0]  debug_angle_target,
    output wire signed [15:0]  debug_angle_actual,
    output wire signed [15:0]  debug_angle_out,
    output wire signed [15:0]  debug_loc_target,
    output wire signed [15:0]  debug_loc_actual,
    output wire signed [15:0]  debug_loc_out,
    output wire [2:0]          debug_state      // 状态机状态，供遥测显示
);

    localparam signed [15:0] CENTER_ANGLE = 16'sd1856;  // 中心角度
    localparam signed [15:0] CENTER_RANGE = 16'sd500;   // 安全角度范围
    localparam signed [15:0] ENTRY_RANGE  = 16'sd250;   // 进入 PID 的窗口
    localparam signed [15:0] SWING_RANGE  = 16'sd200;   // 起摆峰值检测门限（判断左右侧）
    localparam signed [15:0] START_PWM    = 16'sd48;    // 起摆脉冲幅值
    localparam        [15:0] START_TIME   = 16'd95;    // 起摆脉冲时间
    localparam        [15:0] KICK_TIMEOUT = 16'd190;   // 没检测到峰值就补踢

    // 状态定义
    localparam [2:0] S_STOP   = 3'd0;
    localparam [2:0] S_JUDGE  = 3'd1;
    localparam [2:0] S_PULSE1 = 3'd2;
    localparam [2:0] S_WAIT1  = 3'd3;
    localparam [2:0] S_PULSE2 = 3'd4;
    localparam [2:0] S_WAIT2  = 3'd5;
    localparam [2:0] S_PID    = 3'd6;
    localparam [2:0] S_RECOVER= 3'd7;

    // 1ms 定时
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

    // 内部信号
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

    // 按键同步和边沿检测
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

    // 物理按键与远程虚拟按键等价并联（远程按下 = 按下同一个键）
    wire [3:0] key_ev = key_posedge | vkey;

    // 远程接管标志：
    //   收到有效远程指令 --置 1（远程模式生效）
    //   按下任意物理按键 --清 0（立刻夺回控制权，之后完全按本地逻辑走）
    reg rmt_own;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)            rmt_own <= 1'b0;
        else if (|key_posedge) rmt_own <= 1'b0;
        else if (cmd_stb)      rmt_own <= 1'b1;
    end

    // 起摆状态机
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
                // 远程下发绝对位置目标（单周期脉冲，这里直接写入）
                if (loc_we) loc_target <= loc_target_cmd;

                case (state)
                    // 停止
                    S_STOP: begin
                        motor_pwm_r <= 16'sd0;
                        led_run     <= 1'b0;
                        ang_target  <= CENTER_ANGLE;
                    end

                    // 判断摆杆位置
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

                                // 高侧局部最高点 -- 正向起摆
                                if (ang0 > CENTER_ANGLE + SWING_RANGE &&
                                    ang1 > CENTER_ANGLE + SWING_RANGE &&
                                    ang2 > CENTER_ANGLE + SWING_RANGE &&
                                    ang1 > ang0 && ang1 > ang2) begin
                                    swing_dir   <= 1'b1;
                                    no_peak_cnt <= 16'd0;
                                    state       <= S_PULSE1;
                                end
                                // 低侧局部最低点 -- 反向起摆
                                else if (ang0 < CENTER_ANGLE - SWING_RANGE &&
                                         ang1 < CENTER_ANGLE - SWING_RANGE &&
                                         ang2 < CENTER_ANGLE - SWING_RANGE &&
                                         ang1 < ang0 && ang1 < ang2) begin
                                    swing_dir   <= 1'b0;
                                    no_peak_cnt <= 16'd0;
                                    state       <= S_PULSE1;
                                end
                                // 已经进入中心窗口 -- 切换到 PID 平衡
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

                        // 摇摆脉冲 
                       // 起摆脉冲 1
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

                    // 起摆脉冲 2
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

                    // PID 平衡 
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

                    // 倒下后自动恢复
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

            // 按键处理
            // SW1：第一次按下启动自动起摆，再按一次停止
            if (key_ev[0]) begin
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
            if (key_ev[1]) begin
                if (loc_target + 16'sd532 > 16'sd4000)
                    loc_target <= 16'sd4000;
                else
                    loc_target <= loc_target + 16'sd532;
            end

            // SW4：位置目标 -532
            if (key_ev[3]) begin
                if (loc_target - 16'sd532 < -16'sd4000)
                    loc_target <= -16'sd4000;
                else
                    loc_target <= loc_target - 16'sd532;
            end

            // 远程模式
            if (rmt_own && mode == 2'd0) begin      // 停止
                state       <= S_STOP;
                auto_mode   <= 1'b0;
                motor_pwm_r <= 16'sd0;
                led_run     <= 1'b0;
            end else if (rmt_own && mode == 2'd3) begin   // 手动 PWM
                state       <= S_STOP;
                auto_mode   <= 1'b0;
                motor_pwm_r <= manual_pwm;
                led_run     <= 1'b0;
            end else if (rmt_own && mode == 2'd2) begin   // 角度保持（直接进平衡态）
                auto_mode <= 1'b1;
                led_run   <= 1'b1;
                if (state != S_PID) begin
                    state    <= S_PID;
                    cnt_3ms  <= 16'd0;
                    cnt_25ms <= 16'd0;
                end
                // 目标由远程给定；给 0 表示回到中心
                ang_target <= (angle_target_cmd != 16'sd0) ? angle_target_cmd : CENTER_ANGLE;
            end
            // mode == 1（自动起摆）：不覆盖，完全走原有状态机
        end
    end

    // 角度环 PID：3ms
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

    //位置环 PID：25ms
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
    assign debug_state        = state;

endmodule
