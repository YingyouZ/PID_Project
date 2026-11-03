module top(

    input			clk_50m 	,	// 时钟输入
    input			rst_n   	,	// 复位输入（SW5，低有效）
	
	input	[3:0]	key_sw		,	// 按键 SW1~SW4	
	
	output			uart_tx		,	// 串口发送
	input			uart_rx		,	// 串口接收
	output			led_run		,	// 运行指示灯 LED1（U18，高电平点亮）

    // ADC WDD35D4 接口
    output			adc_clk		,	// ADC 采样时钟输出
    output			adc_oe_n	,	// 输出使能，低有效
    input	[9:0]	adc_data_in	,	// D9~D0 并行数据
    
	// TB6612
	input			enc1_a		,	// 电机 1 编码器 A 相
    input			enc1_b		,	// 电机 1 编码器 B 相
	input			enc2_a		,	// 电机 2 编码器 A 相
    input			enc2_b		,	// 电机 2 编码器 B 相
	
	// MG531
	output			AN1			,	// 电机控制信号 IN1
    output			AN2			,	// 电机控制信号 IN2
    output			PWMA		,	// PWM 输出
	
    output			BN1			,	// 电机控制信号 IN1
    output			BN2			,   // 电机控制信号 IN2
    output			PWMB		    // PWM 输出
    
    );
	
	wire			[03:0]	swing_key		;
	
	genvar i;
		generate for(i = 0; i <= 3; i = i + 1) begin
	
		key_filter #(
	
			.CNT_MAX ( 20'd1000_000)
		
		) key_filter_inst (
	
			.sys_clk	(clk_50m),	
			.sys_rst_n	(rst_n),	
			.key_in		(key_sw[i]),	
			
			.key_flag	(swing_key[i])
	
		);
	
		end
	endgenerate
	
	
// 参数定义
wire 			[11:0]	theta_out	;
reg		signed	[15:0]	theta_r		;
wire	signed	[15:0]	theta_out_r	;
	
// 注：3PA1030 的 STBY 为高电平待机、低电平正常工作，模块内部固定为低；
//     该信号未连接到 FPGA，因此不在顶层引出。
adc adc_inst(

   .clk_50m     (clk_50m	),	
   .rst_n		(rst_n		),
   // 3PA1030 硬件接口
   .adc_clk		(adc_clk	),	// ADC 采样时钟输出
   .adc_oe_n	(adc_oe_n	),   // 输出使能，低有效
   .adc_data_in	(adc_data_in),	// D9~D0 并行数据
   .theta_out   (theta_out	)	// 采集的实时角度
   
);   
	wire	[15:0]	enc_pos1,enc_pos2	;
	
	encoder encoder_1 (
	
		.clk_50m		(clk_50m),
		.rst_n			(rst_n),
				
		.enc_a			(enc1_a), 
		.enc_b			(enc1_b),
		
		.pos			(enc_pos1)	// 编码器采集的位置
	
	);

// 计算编码器速度

reg [15:0] cnt_1ms_top;
wire tick_1ms_top;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) cnt_1ms_top <= 0;
    else if(cnt_1ms_top == 49999) cnt_1ms_top <= 0;
    else cnt_1ms_top <= cnt_1ms_top + 1;
end
assign tick_1ms_top = (cnt_1ms_top == 49999);

// 每 1ms 采样一次编码器位置，作差得到速度
reg [15:0] last_enc_pos_1ms;
reg signed [15:0] enc_speed_1ms;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        last_enc_pos_1ms <= 16'd10000;
        enc_speed_1ms <= 0;
    end else if(tick_1ms_top) begin
        enc_speed_1ms <= $signed(enc_pos1) - $signed(last_enc_pos_1ms);
        last_enc_pos_1ms <= enc_pos1;
    end
end

	encoder encoder_2 (
	
		.clk_50m		(clk_50m),
		.rst_n			(rst_n),
				
		.enc_a			(enc2_a), 
		.enc_b			(enc2_b),
		
		.pos			(enc_pos2)	// 编码器采集的位置
	
	);	
	
// 自动起摆 + PID 控制
wire signed [15:0] theta_signed;
assign theta_signed = {4'b0, theta_out}; // 12 位 ADC 数据扩展为 16 位有符号数

wire signed [15:0] pwm_out;
wire signed [15:0] debug_angle_target;
wire signed [15:0] debug_angle_actual;
wire signed [15:0] debug_angle_out;
wire signed [15:0] debug_loc_target;
wire signed [15:0] debug_loc_actual;
wire signed [15:0] debug_loc_out;

control_loop control_comp (
    .clk_50m    (clk_50m),
    .rst_n      (rst_n),
    .adc_angle  (theta_signed),
    .enc_speed  (enc_speed_1ms),
    .key_sw     (swing_key),
    .motor_pwm  (pwm_out),
    .led_run    (led_run),
    .debug_angle_target (debug_angle_target),
    .debug_angle_actual (debug_angle_actual),
    .debug_angle_out    (debug_angle_out),
    .debug_loc_target   (debug_loc_target),
    .debug_loc_actual   (debug_loc_actual),
    .debug_loc_out      (debug_loc_out)
);

// 保存当前角度 theta_r，并做符号化处理
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        theta_r <= 0;
    end else 
        theta_r <= {1'b0,theta_out};
end

assign theta_out_r = theta_r + $signed(135);	// 为实现基于 PID 算法的定值控制功能


// 将 PID 输出转换为电机驱动的方向和占空比
wire [1:0] motor_dir;
wire [7:0] motor_duty;

// 把 PID 输出 (-100~+100) 线性映射到 (0~255)
// 取绝对值后乘以 2.55，再转为整数
wire [15:0] pwm_abs = (pwm_out >= 0) ? pwm_out : -pwm_out;
assign motor_duty = (pwm_abs * 255) / 100;
assign motor_dir = (pwm_out >= 0) ? 2'b01 : 2'b10; // 正转或反转

motor_ctrl motor_ctrl_A(
    .clk_50m (clk_50m),
    .rst_n   (rst_n),
    .motor_a_sel (motor_dir),     // 接转换后的方向
    .duty_out    (motor_duty),    // 接转换后的占空比
    .ain1 (AN1),
    .ain2 (AN2),
    .pwm  (PWMA)
);

motor_ctrl motor_ctrl_B(
    .clk_50m (clk_50m),
    .rst_n   (rst_n),
    .motor_a_sel (motor_dir),     // B 电机和 A 电机同步
    .duty_out    (motor_duty),
    .ain1 (BN1),
    .ain2 (BN2),
    .pwm  (PWMB)
);

// 20ms 调试串口：固定 1.2ms/字节，AA 55 + 6×int16 + 0D 0A
uart_debug uart_debug_inst (
    .clk_50M       (clk_50m),
    .rst_n         (rst_n),
    .angle_target  (debug_angle_target),
    .angle_actual  (debug_angle_actual),
    .angle_out     (debug_angle_out),
    .loc_target    (debug_loc_target),
    .loc_actual    (debug_loc_actual),
    .loc_out       (debug_loc_out),
    .fpga_tx       (uart_tx)
);

endmodule
