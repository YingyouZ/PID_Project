module pid #(
    parameter signed [31:0] KP = 1229,      // Q12: 0.3 * 4096
    parameter signed [31:0] KI = 41,        // Q12: 0.01 * 4096
    parameter signed [31:0] KD = 1638,      // Q12: 0.4 * 4096
    parameter signed [31:0] OUT_MAX = 100,
    parameter signed [31:0] OUT_MIN = -100,
    parameter signed [31:0] INT_MAX = 30,   // 积分限幅
    parameter signed [15:0] INT_BAND = 50   // 积分分离
)(
    input wire clk,
    input wire rst_n,
    input wire update_en,
    input wire signed [15:0] target,
    input wire signed [15:0] actual,
    output reg signed [15:0] out
);

    // 当前的输出从error_now计算，而不是从过时的error
    reg signed [16:0] error_prev;
    reg sample_valid;
    reg signed [31:0] error_int;
    reg signed [18:0] delta_filt;

    wire signed [16:0] target_ext = {target[15], target};
    wire signed [16:0] actual_ext = {actual[15], actual};
    wire signed [16:0] error_now = target_ext - actual_ext;
    wire signed [17:0] error_delta = sample_valid ?
                                     ({error_now[16], error_now} - {error_prev[16], error_prev}) : 18'sd0;

    //在乘法之前加宽操作数，这样中间PID项就不会被加宽溢出32位
    wire signed [63:0] kp_wide = KP;
    wire signed [63:0] ki_wide = KI;
    wire signed [63:0] kd_wide = KD;
    wire signed [63:0] error_wide = error_now;
    wire signed [63:0] delta_wide = delta_filt;
    wire signed [63:0] p_term = kp_wide * error_wide;
    wire signed [63:0] d_term = kd_wide * delta_wide;

    localparam signed [31:0] INT_LIMIT = (KI != 0) ? ((INT_MAX << 12) / KI) : 0;

    reg signed [31:0] error_int_next;
    reg signed [63:0] i_term;
    reg signed [63:0] temp_out;

    // 在导数输入上轻单极低通：alpha = 1/2。
    //在KD放大前衰减高频编码器/ADC噪声。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            delta_filt <= 19'sd0;
        else if (update_en)
            delta_filt <= (delta_filt + error_delta) >>> 1;
    end

    always @* begin
        error_int_next = error_int;
        if (KI == 0) begin
            error_int_next = 0;
        end else if ((error_now <= INT_BAND) && (error_now >= -INT_BAND)) begin
            if (($signed(error_int) + $signed(error_now)) > INT_LIMIT)
                error_int_next = INT_LIMIT;
            else if (($signed(error_int) + $signed(error_now)) < -INT_LIMIT)
                error_int_next = -INT_LIMIT;
            else
                error_int_next = error_int + error_now;
        end

        i_term = ki_wide * $signed(error_int_next);
        temp_out = (p_term + i_term + d_term) >>> 12;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            error_prev <= 0;
            sample_valid <= 1'b0;
            error_int <= 0;
            out <= 0;
        end else if (update_en) begin
            error_prev <= error_now;
            sample_valid <= 1'b1;
            error_int <= error_int_next;

            if (temp_out > OUT_MAX)
                out <= OUT_MAX[15:0];
            else if (temp_out < OUT_MIN)
                out <= OUT_MIN[15:0];
            else
                out <= temp_out[15:0];
        end
    end
endmodule
