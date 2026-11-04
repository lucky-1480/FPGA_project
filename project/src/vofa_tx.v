// 将八个有符号 16 位通道打包为 VOFA+ JustFloat 帧：
// 八个小端 float32 值，后面跟 00 00 80 7F 尾帧。
module vofa_tx #(
    parameter N_CH = 8
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               send_pulse,
    input  wire signed [15:0] ch0,
    input  wire signed [15:0] ch1,
    input  wire signed [15:0] ch2,
    input  wire signed [15:0] ch3,
    input  wire signed [15:0] ch4,
    input  wire signed [15:0] ch5,
    input  wire signed [15:0] ch6,
    input  wire signed [15:0] ch7,
    output reg  [7:0]         tx_data,
    output reg                tx_start,
    input  wire               tx_busy
);
    localparam TOTAL_BYTES = 4 * N_CH + 4;

    // 将有符号整数转换为 IEEE-754 单精度浮点（binary32）。
    // 16 位输入足够小，尾数无需舍入。
    function [31:0] int16_to_float;
        input signed [15:0] din;
        reg        sign;
        reg [15:0] abs_v;
        reg [7:0]  exp;
        reg [22:0] mant;
        begin
            sign  = din[15];
            abs_v = din[15] ? (~din + 16'd1) : din;
            if (abs_v == 16'd0) begin
                int16_to_float = 32'h0000_0000;
            end else begin
                casez (abs_v)
                    16'b1???_????_????_????: begin exp = 8'd142; mant = {abs_v[14:0], 8'd0};  end
                    16'b01??_????_????_????: begin exp = 8'd141; mant = {abs_v[13:0], 9'd0};  end
                    16'b001?_????_????_????: begin exp = 8'd140; mant = {abs_v[12:0], 10'd0}; end
                    16'b0001_????_????_????: begin exp = 8'd139; mant = {abs_v[11:0], 11'd0}; end
                    16'b0000_1???_????_????: begin exp = 8'd138; mant = {abs_v[10:0], 12'd0}; end
                    16'b0000_01??_????_????: begin exp = 8'd137; mant = {abs_v[9:0], 13'd0};  end
                    16'b0000_001?_????_????: begin exp = 8'd136; mant = {abs_v[8:0], 14'd0};  end
                    16'b0000_0001_????_????: begin exp = 8'd135; mant = {abs_v[7:0], 15'd0};  end
                    16'b0000_0000_1???_????: begin exp = 8'd134; mant = {abs_v[6:0], 16'd0};  end
                    16'b0000_0000_01??_????: begin exp = 8'd133; mant = {abs_v[5:0], 17'd0};  end
                    16'b0000_0000_001?_????: begin exp = 8'd132; mant = {abs_v[4:0], 18'd0};  end
                    16'b0000_0000_0001_????: begin exp = 8'd131; mant = {abs_v[3:0], 19'd0};  end
                    16'b0000_0000_0000_1???: begin exp = 8'd130; mant = {abs_v[2:0], 20'd0};  end
                    16'b0000_0000_0000_01??: begin exp = 8'd129; mant = {abs_v[1:0], 21'd0};  end
                    16'b0000_0000_0000_001?: begin exp = 8'd128; mant = {abs_v[0], 22'd0};    end
                    16'b0000_0000_0000_0001: begin exp = 8'd127; mant = 23'd0;                end
                    default:                 begin exp = 8'd127; mant = 23'd0;                end
                endcase
                int16_to_float = {sign, exp, mant};
            end
        end
    endfunction

    reg [31:0] f0, f1, f2, f3, f4, f5, f6, f7;
    always @(*) begin
        f0 = int16_to_float(ch0);
        f1 = int16_to_float(ch1);
        f2 = int16_to_float(ch2);
        f3 = int16_to_float(ch3);
        f4 = int16_to_float(ch4);
        f5 = int16_to_float(ch5);
        f6 = int16_to_float(ch6);
        f7 = int16_to_float(ch7);
    end

    reg [7:0]  byte_idx;
    reg [31:0] cur_float;
    reg [7:0]  cur_byte;

    always @(*) begin
        case (byte_idx[7:2])
            6'd0: cur_float = f0;
            6'd1: cur_float = f1;
            6'd2: cur_float = f2;
            6'd3: cur_float = f3;
            6'd4: cur_float = f4;
            6'd5: cur_float = f5;
            6'd6: cur_float = f6;
            6'd7: cur_float = f7;
            default: cur_float = 32'd0;
        endcase
    end

    always @(*) begin
        if (byte_idx < 4 * N_CH) begin
            case (byte_idx[1:0])
                2'd0: cur_byte = cur_float[7:0];
                2'd1: cur_byte = cur_float[15:8];
                2'd2: cur_byte = cur_float[23:16];
                2'd3: cur_byte = cur_float[31:24];
            endcase
        end else begin
            case (byte_idx[1:0])
                2'd0: cur_byte = 8'h00;
                2'd1: cur_byte = 8'h00;
                2'd2: cur_byte = 8'h80;
                2'd3: cur_byte = 8'h7F;
            endcase
        end
    end

    localparam [2:0] S_IDLE      = 3'd0,
                     S_SEND      = 3'd1,
                     S_WAIT_BUSY = 3'd2,
                     S_WAIT_DONE = 3'd3;

    reg [2:0] state;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            byte_idx <= 8'd0;
            tx_data  <= 8'd0;
            tx_start <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (send_pulse) begin
                        byte_idx <= 8'd0;
                        state    <= S_SEND;
                    end
                end

                S_SEND: begin
                    tx_data  <= cur_byte;
                    tx_start <= 1'b1;
                    state    <= S_WAIT_BUSY;
                end

                S_WAIT_BUSY: begin
                    if (tx_busy)
                        state <= S_WAIT_DONE;
                end

                S_WAIT_DONE: begin
                    if (!tx_busy) begin
                        if (byte_idx == TOTAL_BYTES - 1) begin
                            state <= S_IDLE;
                        end else begin
                            byte_idx <= byte_idx + 8'd1;
                            state    <= S_SEND;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
