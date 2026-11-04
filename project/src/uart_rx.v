// 8N1 UART 接收器，低位在前，采用位中央采样。
module uart_rx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rxd,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);
    localparam [15:0] BIT_DIV  = (CLK_FREQ / BAUD_RATE) - 1;
    localparam [15:0] HALF_DIV = (CLK_FREQ / BAUD_RATE) / 2 - 1;

    reg rxd_s1, rxd_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxd_s1 <= 1'b1;
            rxd_s2 <= 1'b1;
        end else begin
            rxd_s1 <= rxd;
            rxd_s2 <= rxd_s1;
        end
    end
    wire rxd_in = rxd_s2;

    localparam [3:0] S_IDLE  = 4'd0,
                     S_START = 4'd1,
                     S_STOP  = 4'd10;

    reg [3:0]  state;
    reg [15:0] clk_cnt;
    reg [3:0]  bit_idx;
    reg [7:0]  shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_cnt   <= 16'd0;
            bit_idx   <= 4'd0;
            shift_reg <= 8'd0;
            rx_data   <= 8'd0;
            rx_valid  <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    clk_cnt <= 16'd0;
                    bit_idx <= 4'd0;
                    if (rxd_in == 1'b0)
                        state <= S_START;
                end

                S_START: begin
                    // 在起始位中点复核其电平。
                    if (clk_cnt == HALF_DIV) begin
                        clk_cnt <= 16'd0;
                        if (rxd_in == 1'b0) begin
                            bit_idx <= 4'd0;
                            state   <= 4'd2;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, 4'd8, 4'd9: begin
                    if (clk_cnt == BIT_DIV) begin
                        clk_cnt <= 16'd0;
                        shift_reg[bit_idx] <= rxd_in;
                        if (bit_idx == 4'd7) begin
                            state   <= S_STOP;
                            bit_idx <= 4'd0;
                        end else begin
                            bit_idx <= bit_idx + 4'd1;
                            state   <= state + 4'd1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    if (clk_cnt == BIT_DIV) begin
                        clk_cnt  <= 16'd0;
                        rx_data  <= shift_reg;
                        rx_valid <= 1'b1;
                        state    <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
