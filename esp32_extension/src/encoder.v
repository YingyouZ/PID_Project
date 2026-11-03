   module encoder (

	input			clk_50m			,
    input			rst_n			,
			
    input			enc_a, enc_b	,
	
    output	[15:0]	pos					

);
reg 	[15:0]	pos_reg;
reg [1:0] enc_sync_a, enc_sync_b;
reg [1:0] state_now, state_last;

assign pos = pos_reg;


always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
		enc_sync_a <= 'd0;
		enc_sync_b <= 'd0;
		state_now <= 'd0;
	end else begin  //同步
		enc_sync_a <= {enc_sync_a[0], enc_a};   //enc_sync_a[0] 存入 enc_a 的当前值，enc_sync_a[1] 存入旧的 enc_sync_a[0]。第一级同步
                       //第二次时钟沿到来：enc_sync_a[0] 再次存入新的 enc_a，enc_sync_a[1] 存入上一个周期的 enc_sync_a[0]。第二级同步
		enc_sync_b <= {enc_sync_b[0], enc_b};
		state_now <= {enc_sync_a[1], enc_sync_b[1]};
	end
end


// 四倍频方向判断，累加位置
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pos_reg <= 'd10000; 
        state_last <= 2'b00;
    end else begin
        state_last <= state_now;    //state_last <= state_now 和 case({state_last, state_now}) 是同时发生
        case({state_last, state_now}) 
//{A相，B相}=00、01、11、10、00，B相先变化，记为反转
            4'b00_01: pos_reg <= pos_reg - 1;
            4'b01_11: pos_reg <= pos_reg - 1;
            4'b11_10: pos_reg <= pos_reg - 1;
            4'b10_00: pos_reg <= pos_reg - 1;
//{A相，B相}=00、10、11、01、00，A相先变化，记为正转			
            4'b00_10: pos_reg <= pos_reg + 1;
            4'b10_11: pos_reg <= pos_reg + 1;
            4'b11_01: pos_reg <= pos_reg + 1;
            4'b01_00: pos_reg <= pos_reg + 1;
            default: pos_reg <= pos_reg;	//pos_reg
        endcase //vel <= pos_reg - pos;	
		//正转一个完整循环，把 A 和 B 的上升沿和下降沿全加起来，pos_reg 加了 4 次，实现了四倍频
        
    end
end



endmodule
