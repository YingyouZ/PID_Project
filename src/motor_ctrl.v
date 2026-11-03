module  motor_ctrl(

    input				clk_50m 	,	//时钟输入
    input				rst_n   	,	//复位输入
		
    // Motor A Control Command
    input       [1:0]   motor_a_sel ,   // 判断转动方向，A Motor Mode Selection：00:stop 01:CW 10:CCW 11:Short brake
    input       [7:0]   duty_out	,   // A Motor PWM Duty Cycle: 0~255(0%~100%)
    
    // 电机TB6612FNG 
    output	reg			ain1        ,
    output	reg			ain2        ,	
    output              pwm       

    );
	
// 10kHz PWM Carrier Generation
// 50MHz / 5000 = 10kHz 
reg	[12:0]	pwm_cnt;
localparam	PWM_PERIOD = 13'd5000;  //0.1ms

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
        pwm_cnt <= 'd0;
    else if(pwm_cnt >= PWM_PERIOD - 1'b1)
        pwm_cnt <= 'd0;
    else
        pwm_cnt <= pwm_cnt + 'd1;
end

// Mapping：0~255 -> 0~5100
    wire [24:0] cmp_a_temp = duty_out * 20;  // 255*20 = 5100       
    wire [24:0] cmp_a = (cmp_a_temp > PWM_PERIOD) ? PWM_PERIOD : cmp_a_temp;    //限幅

//Motor A 
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        ain1 <= 1'b0;
        ain2 <= 1'b0;
    end else begin     
        case(motor_a_sel)
            2'b00 : begin ain1 <= 1'b0; ain2 <= 1'b0; end  // Stop
            2'b01 : begin ain1 <= 1'b1; ain2 <= 1'b0; end  // CW
            2'b10 : begin ain1 <= 1'b0; ain2 <= 1'b1; end  // CCW
            2'b11 : begin ain1 <= 1'b1; ain2 <= 1'b1; end  // Short brake
        endcase
    end
end

// PWM
assign pwm = (pwm_cnt < cmp_a) ? 1'b1 : 1'b0;   //占空比

endmodule

