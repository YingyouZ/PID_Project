module key_filter #(

	parameter CNT_MAX = 20'd1000_000
	
)(

	input		sys_clk		,	
	input		sys_rst_n	,	
	input		key_in		,	
	
	output	reg	key_flag = 1'b0

);

reg [19:0] cnt_20ms ; 

// 异步信号同步
reg key_in_r1, key_in_r2;
always@(posedge sys_clk) begin
    key_in_r1 <= key_in;
    key_in_r2 <= key_in_r1;
end

always@(posedge sys_clk or negedge sys_rst_n) begin
	if(sys_rst_n == 1'b0)
		cnt_20ms <= 20'b0;
	else if(key_in_r2 == 1'b1) //按键松开，高电平，清零
		cnt_20ms <= 20'b0;
	else if(cnt_20ms == CNT_MAX && key_in_r2 == 1'b0)
		cnt_20ms <= cnt_20ms;           //计数器数到了 1,000,000，且按键还是低电平，就保持这个值不变，防止溢出
	else
		cnt_20ms <= cnt_20ms + 1'b1;
end

always@(posedge sys_clk or negedge sys_rst_n) begin
	if(sys_rst_n == 1'b0)
		key_flag <= 1'b0;
	else if(cnt_20ms == CNT_MAX - 1'b1)
		key_flag <= 1'b1;           //key_flag 高电平维持了一个时钟周期
	else
		key_flag <= 1'b0;
end




endmodule