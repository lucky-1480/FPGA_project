// J280 低有效按键消抖，每次按下产生一个单周期脉冲。
module key_filter #(
    parameter CLK_FREQ = 50_000_000,
    parameter DB_MS    = 20
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_in,
    output reg  key_pulse
);

    localparam DB_CNT_MAX = CLK_FREQ / 1000 * DB_MS;

    reg key_s0, key_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_s0 <= 1'b1;
            key_s1 <= 1'b1;
        end else begin
            key_s0 <= key_in;
            key_s1 <= key_s0;
        end
    end

    reg [19:0] cnt;
    reg        key_stable;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt        <= 20'd0;
            key_stable <= 1'b1;
        end else if (key_s1 != key_stable) begin
            if (cnt == DB_CNT_MAX - 1) begin
                key_stable <= key_s1;
                cnt        <= 20'd0;
            end else begin
                cnt <= cnt + 20'd1;
            end
        end else begin
            cnt <= 20'd0;
        end
    end

    reg key_stable_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_stable_d <= 1'b1;
            key_pulse    <= 1'b0;
        end else begin
            key_stable_d <= key_stable;
            key_pulse    <= key_stable_d & ~key_stable;
        end
    end

endmodule
