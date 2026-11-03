module  adc(

    input      		clk_50m		,	
    input      		rst_n		,
    // 3PA1030 硬件接口
    output	    	adc_clk		,	// ADC 采样时钟输出
    output	    	adc_oe_n	,	// 输出使能，低有效
    output	    	adc_stby_n	,	// 待机控制：高电平待机，低电平正常工作
    input	[9:0]	adc_data_in	,	// D9~D0 并行数据：角度传感器 0~3.3V 模拟电压经 ADC 转换为 10 位数据
    output	[11:0]	theta_out       
	
    );
	
// 参数定义
reg				clk_div			; 
// 将 ADC 数据扩展到 12 位
wire	[11:0]	theta			;
reg		[11:0]	theta_out_reg	;
wire	[11:0]	theta_12bit		;  
reg		[15:0]	theta_sum		;
reg		[17:0]	cnt_3ms			;
reg		[13:0]	cnt_187us		;

// 信号定义
assign adc_oe_n		= 1'b0;
assign adc_stby_n	= 1'b0;     // 低电平：正常工作
assign theta_12bit	= {adc_data_in  , 2'b0 };   // 10 位数据左移 2 位，扩展为 12 位
                                                                         // 相当于原始数据乘以 4
assign theta_out	= theta_out_reg;

// 50MHz 二分频，产生 25MHz 的 ADC 时钟
always@(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) 
		clk_div <= 1'b0;
    else 
		clk_div <= ~clk_div;
end

assign adc_clk = clk_div; // 25MHz 输出给 3PA1030 的 CLK 引脚

// 3ms 计数器
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        cnt_3ms <= 0;
    end else if(cnt_3ms == 149999)begin
        cnt_3ms <= 0;
    end else 
        cnt_3ms <= cnt_3ms + 1;
end
// 187.5us 计数器
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        cnt_187us <= 0;
    end else if(cnt_187us == 9374 )begin
        cnt_187us <= 0;
    end else 
        cnt_187us <= cnt_187us +1;
end
// 累加 16 次 ADC 数据后求平均
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        theta_sum <= 0;
        theta_out_reg <= 0;
    end else if(cnt_187us == 9373 )begin
        theta_sum <= theta_sum + theta_12bit;   // 每 187.5us 累加一次 ADC 数据
    end else if(cnt_3ms == 149999)begin        // 每 3ms 求一次平均值，共累加 16 次
		theta_out_reg <= theta_sum >> 4;        // 右移 4 位，相当于除以 16
		theta_sum <= 0;
    end else begin
		theta_out_reg <= theta_out_reg;
		theta_sum <= theta_sum;
    end
end

endmodule
