// 8N1 UART 发送器，低位在前。
module uart_tx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output reg        tx_busy,
    output reg        txd
);
    localparam [15:0] BIT_DIV = (CLK_FREQ / BAUD_RATE) - 1;
    localparam [3:0] S_IDLE  = 4'd0,
                     S_START = 4'd1,
                     S_STOP  = 4'd10;

    reg [3:0]  state;
    reg [15:0] clk_cnt;
    reg [7:0]  shift_reg;
    reg [3:0]  bit_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_cnt   <= 16'd0;
            shift_reg <= 8'd0;
            bit_idx   <= 4'd0;
            tx_busy   <= 1'b0;
            txd       <= 1'b1;
        end else begin
            case (state)
                S_IDLE: begin
                    txd     <= 1'b1;
                    clk_cnt <= 16'd0;
                    bit_idx <= 4'd0;
                    if (tx_start && !tx_busy) begin
                        shift_reg <= tx_data;
                        tx_busy   <= 1'b1;
                        state     <= S_START;
                    end
                end

                S_START: begin
                    txd <= 1'b0;
                    if (clk_cnt == BIT_DIV) begin
                        clk_cnt <= 16'd0;
                        state   <= 4'd2;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, 4'd8, 4'd9: begin
                    txd <= shift_reg[bit_idx];
                    if (clk_cnt == BIT_DIV) begin
                        clk_cnt <= 16'd0;
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
                    txd <= 1'b1;
                    if (clk_cnt == BIT_DIV) begin
                        clk_cnt <= 16'd0;
                        tx_busy <= 1'b0;
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
