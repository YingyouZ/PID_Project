module uart_debug(
    input  wire               clk_50M,
    input  wire               rst_n,
    input  wire signed [15:0] angle_target,
    input  wire signed [15:0] angle_actual,
    input  wire signed [15:0] angle_out,
    input  wire signed [15:0] loc_target,
    input  wire signed [15:0] loc_actual,
    input  wire signed [15:0] loc_out,
    output wire               fpga_tx
);

    localparam integer CLK_FREQ     = 50_000_000;
    localparam integer UART_BPS     = 9600;
    localparam integer FRAME_CYCLES = CLK_FREQ / 50;   // 20ms
    localparam integer BYTE_CYCLES  = 60_000;          // 1.2ms，大于 9600 下一个字节的 1.04ms

    // 20ms 帧节拍
    reg [31:0] cnt_20ms;
    wire       tick_20ms;
    always @(posedge clk_50M or negedge rst_n) begin
        if (!rst_n)
            cnt_20ms <= 32'd0;
        else if (cnt_20ms >= FRAME_CYCLES - 1)
            cnt_20ms <= 32'd0;
        else
            cnt_20ms <= cnt_20ms + 1'b1;
    end
    assign tick_20ms = (cnt_20ms == FRAME_CYCLES - 1);

    // uart_tx
    reg  [7:0] tx_data;
    reg        tx_flag;
    wire       tx_done;

    uart_tx #(
        .UART_BPS (UART_BPS),
        .CLK_FREQ (CLK_FREQ)
    ) uart_tx_inst (
        .sys_clk   (clk_50M),
        .sys_rst_n (rst_n),
        .pi_data   (tx_data),
        .pi_flag   (tx_flag),
        .tx        (fpga_tx),
        .pi_done   (tx_done)
    );

    // 每帧锁存一次数据
    reg signed [15:0] atgt_lat, aact_lat, aout_lat;
    reg signed [15:0] ltgt_lat, lact_lat, lout_lat;

    reg [3:0]  byte_idx;
    reg        sending;
    reg [31:0] byte_timer;

    always @(posedge clk_50M or negedge rst_n) begin
        if (!rst_n) begin
            atgt_lat   <= 16'sd0;
            aact_lat   <= 16'sd0;
            aout_lat   <= 16'sd0;
            ltgt_lat   <= 16'sd0;
            lact_lat   <= 16'sd0;
            lout_lat   <= 16'sd0;
            byte_idx   <= 4'd0;
            sending    <= 1'b0;
            byte_timer <= 32'd0;
            tx_flag    <= 1'b0;
        end else begin
            tx_flag <= 1'b0;

            if (!sending) begin
                if (tick_20ms) begin
                    atgt_lat   <= angle_target;
                    aact_lat   <= angle_actual;
                    aout_lat   <= angle_out;
                    ltgt_lat   <= loc_target;
                    lact_lat   <= loc_actual;
                    lout_lat   <= loc_out;
                    byte_idx   <= 4'd0;
                    byte_timer <= 32'd0;
                    sending    <= 1'b1;
                    tx_flag    <= 1'b1;
                end
            end else begin
                // 固定 1.2ms 发一个字节，和测试代码的发送节奏一致，
                // 避免 tx_done 背靠背启动导致 UART 停止位被压缩。
                if (byte_timer >= BYTE_CYCLES - 1) begin
                    byte_timer <= 32'd0;
                    if (byte_idx == 4'd15) begin
                        sending <= 1'b0;
                    end else begin
                        byte_idx <= byte_idx + 1'b1;
                        tx_flag  <= 1'b1;
                    end
                end else begin
                    byte_timer <= byte_timer + 1'b1;
                end
            end
        end
    end

    // 帧格式：
    // AA 55
    // 角度目标(2B) 角度实际(2B) 角度输出(2B)
    // 位置目标(2B) 位置实际(2B) 位置输出(2B)
    // 0D 0A
    always @(*) begin
        case (byte_idx)
            4'd0:  tx_data = 8'hAA;
            4'd1:  tx_data = 8'h55;
            4'd2:  tx_data = atgt_lat[15:8];
            4'd3:  tx_data = atgt_lat[7:0];
            4'd4:  tx_data = aact_lat[15:8];
            4'd5:  tx_data = aact_lat[7:0];
            4'd6:  tx_data = aout_lat[15:8];
            4'd7:  tx_data = aout_lat[7:0];
            4'd8:  tx_data = ltgt_lat[15:8];
            4'd9:  tx_data = ltgt_lat[7:0];
            4'd10: tx_data = lact_lat[15:8];
            4'd11: tx_data = lact_lat[7:0];
            4'd12: tx_data = lout_lat[15:8];
            4'd13: tx_data = lout_lat[7:0];
            4'd14: tx_data = 8'h0D;
            4'd15: tx_data = 8'h0A;
            default: tx_data = 8'h00;
        endcase
    end

endmodule
