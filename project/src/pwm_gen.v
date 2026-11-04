// 将有符号占空比指令转换为 TB6612 的 PWM 与方向输出。
// 正占空比正转，负占空比反转，零则停止。
module pwm_gen #(
    parameter DUTY_MAX  = 2500,     // 50 MHz / 2500 = 20 kHz
    parameter DEAD_TIME = 200       // 200 个时钟周期 = 4 us
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] duty_in,
    output reg                pwm_out,
    output reg                in1,
    output reg                in2
);

    wire signed [15:0] duty_lim =
        (duty_in >  DUTY_MAX) ?  16'sd2500 :
        (duty_in < -DUTY_MAX) ? -16'sd2500 : duty_in;

    wire signed [15:0] duty_abs =
        (duty_lim < 0) ? -duty_lim : duty_lim;

    reg [12:0] abs_duty_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            abs_duty_reg <= 13'd0;
        else
            abs_duty_reg <= duty_abs[12:0];
    end

    reg [12:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= 13'd0;
            pwm_out <= 1'b0;
        end else begin
            if (cnt >= DUTY_MAX - 1)
                cnt <= 13'd0;
            else
                cnt <= cnt + 13'd1;

            pwm_out <= (cnt < abs_duty_reg);
        end
    end

    reg signed [15:0] duty_d1;
    reg [15:0]        dead_cnt;
    reg               in_dead;

    wire reversing = (duty_lim > 0 && duty_d1 < 0) ||
                     (duty_lim < 0 && duty_d1 > 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_d1  <= 16'sd0;
            in1      <= 1'b0;
            in2      <= 1'b0;
            dead_cnt <= 16'd0;
            in_dead  <= 1'b0;
        end else begin
            duty_d1 <= duty_lim;

            if (reversing && !in_dead) begin
                in_dead  <= 1'b1;
                dead_cnt <= DEAD_TIME;
                in1      <= 1'b0;
                in2      <= 1'b0;
            end else if (in_dead) begin
                if (dead_cnt == 16'd0)
                    in_dead <= 1'b0;
                else
                    dead_cnt <= dead_cnt - 16'd1;
                in1 <= 1'b0;
                in2 <= 1'b0;
            end else if (duty_lim > 0) begin
                in1 <= 1'b1;
                in2 <= 1'b0;
            end else if (duty_lim < 0) begin
                in1 <= 1'b0;
                in2 <= 1'b1;
            end else begin
                in1 <= 1'b0;
                in2 <= 1'b0;
            end
        end
    end

endmodule
