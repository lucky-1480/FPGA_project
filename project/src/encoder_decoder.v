// J280 正交编码器解码模块。
// 统计 A/B 两相全部边沿（4 倍频），累计位置，并产生 1 kHz
// 采样脉冲以及每个 1 ms 周期内的位置增量。
module encoder_decoder #(
    parameter SPEED_PERIOD = 50_000     // 50 MHz / 50,000 = 1 kHz
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enc_a,
    input  wire               enc_b,
    output reg  signed [31:0] pos_cnt,
    output reg  signed [15:0] speed,
    output reg                sample_en
);

    // 三级寄存器：前两级消亚稳态，第三级提供上一拍采样用于边沿检测。
    reg a_r0, a_r1, a_r2;
    reg b_r0, b_r1, b_r2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {a_r0, a_r1, a_r2} <= 3'b000;
            {b_r0, b_r1, b_r2} <= 3'b000;
        end else begin
            a_r0 <= enc_a;
            a_r1 <= a_r0;
            a_r2 <= a_r1;
            b_r0 <= enc_b;
            b_r1 <= b_r0;
            b_r2 <= b_r1;
        end
    end

    wire a_rise =  a_r1 & ~a_r2;
    wire a_fall = ~a_r1 &  a_r2;
    wire b_rise =  b_r1 & ~b_r2;
    wire b_fall = ~b_r1 &  b_r2;
    wire any_edge = a_rise | a_fall | b_rise | b_fall;

    wire fwd = (a_rise & ~b_r1) | (a_fall &  b_r1)
             | (b_rise &  a_r1) | (b_fall & ~a_r1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pos_cnt <= 32'sd0;
        else if (any_edge)
            pos_cnt <= pos_cnt + (fwd ? 32'sd1 : -32'sd1);
    end

    reg [31:0] div_cnt;
    reg signed [31:0] pos_last;
    reg signed [31:0] delta32;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt   <= 32'd0;
            pos_last  <= 32'sd0;
            delta32   <= 32'sd0;
            speed     <= 16'sd0;
            sample_en <= 1'b0;
        end else begin
            sample_en <= 1'b0;

            if (div_cnt == SPEED_PERIOD - 1) begin
                div_cnt   <= 32'd0;
                sample_en <= 1'b1;

                // 用 32 位算增量，只保留低 16 位作为速度。
                delta32  = pos_cnt - pos_last;
                speed    <= delta32[15:0];
                pos_last <= pos_cnt;
            end else begin
                div_cnt <= div_cnt + 32'd1;
            end
        end
    end

endmodule
