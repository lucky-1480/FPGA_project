// J280 / 3PA1030 角度采集模块。
// 上电时摆杆自然下垂，前 256 次采样确定下垂参考零点；
// 直立位置在该参考零点上方 HALF_TURN（半圈码差）处。
module angle_sensor #(
    parameter ADC_CLK_DIV = 4,       // ADC 时钟 = 50 MHz / (2 * ADC_CLK_DIV)
    parameter signed [15:0] HALF_TURN = 16'sd531
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire [9:0]         adc_d,
    input  wire               adc_otr,
    output wire               adc_clk,
    output wire               adc_oe_n,
    input  wire               sample_en,
    input  wire signed [15:0] zero_offset,
    output reg  signed [15:0] angle_diff,
    output reg                cal_done
);

    assign adc_oe_n = 1'b0;

    // 6.25 MHz ADC 采样时钟。
    reg [7:0] adc_clk_cnt;
    reg       adc_clk_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_clk_cnt <= 8'd0;
            adc_clk_reg <= 1'b0;
        end else if (adc_clk_cnt == ADC_CLK_DIV - 1) begin
            adc_clk_cnt <= 8'd0;
            adc_clk_reg <= ~adc_clk_reg;
        end else begin
            adc_clk_cnt <= adc_clk_cnt + 8'd1;
        end
    end
    assign adc_clk = adc_clk_reg;

    // 将异步 ADC 输入同步到本时钟域。
    reg [9:0] adc_d_s0, adc_d_s1;
    reg       otr_s0, otr_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_d_s0 <= 10'd0;
            adc_d_s1 <= 10'd0;
            otr_s0   <= 1'b0;
            otr_s1   <= 1'b0;
        end else begin
            adc_d_s0 <= adc_d;
            adc_d_s1 <= adc_d_s0;
            otr_s0   <= adc_otr;
            otr_s1   <= otr_s0;
        end
    end
    wire [9:0] adc_val = adc_d_s1;
    wire       otr_val = otr_s1;

    // 每个 1 kHz 采样节拍产生一个单周期脉冲。
    reg sample_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sample_sync <= 1'b0;
        else
            sample_sync <= sample_en;
    end
    wire sample_pulse = sample_en & ~sample_sync;

    localparam CAL_N     = 256;
    localparam CAL_SHIFT = 8;

    reg [17:0] cal_sum;
    reg [8:0]  cal_cnt;
    reg [9:0]  zero;

    wire signed [15:0] zero_up =
        $signed({6'd0, zero}) + HALF_TURN + zero_offset;

    // 将超量程采样钳位到有符号 ADC 范围内。
    wire signed [15:0] diff_next = otr_val
        ? (adc_val[9] ? -16'sd512 : 16'sd512)
        : ($signed({6'd0, adc_val}) - zero_up);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            zero       <= 10'd512;
            cal_cnt    <= 9'd0;
            cal_sum    <= 18'd0;
            angle_diff <= 16'sd0;
            cal_done   <= 1'b0;
        end else if (sample_pulse) begin
            // 累计满 256 次采样后再使能闭环控制。
            if (!cal_done) begin
                cal_sum <= cal_sum + {{8{1'b0}}, adc_val};
                cal_cnt <= cal_cnt + 9'd1;
                if (cal_cnt == CAL_N - 1) begin
                    zero     <= (cal_sum + {{8{1'b0}}, adc_val})
                                >> CAL_SHIFT;
                    cal_done <= 1'b1;
                end
            end

            angle_diff <= diff_next;
        end
    end

endmodule
