// UART 命令解析器，用于 PID 增益、零点微调与手动 PWM 测试。
//
// 命令以 LF、CR 或 CRLF 结尾：
//   KP:n  KI:n  KD:n    角度环增益
//   PK:n  PI:n  PD:n    位置环增益
//   ZF:n                角度零点微调
//   MN:n                进入手动 PWM 模式，占空比带符号 n
//   MO                  退出手动 PWM 模式
//   CL                  恢复全部默认值
module param_parser #(
    parameter signed [31:0] DEFAULT_ANGLE_KP = 32'sd8,
    parameter signed [31:0] DEFAULT_ANGLE_KI = 32'sd0,
    parameter signed [31:0] DEFAULT_ANGLE_KD = 32'sd30,
    parameter signed [31:0] DEFAULT_POS_KP   = 32'sd1,
    parameter signed [31:0] DEFAULT_POS_KI   = 32'sd0,
    parameter signed [31:0] DEFAULT_POS_KD   = 32'sd4,
    parameter signed [31:0] DEFAULT_ZERO_OFF = 32'sd0
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [7:0]        rx_data,
    input  wire              rx_valid,
    output reg signed [31:0] angle_kp,
    output reg signed [31:0] angle_ki,
    output reg signed [31:0] angle_kd,
    output reg signed [31:0] pos_kp,
    output reg signed [31:0] pos_ki,
    output reg signed [31:0] pos_kd,
    output reg signed [31:0] zero_offset,
    output reg signed [15:0] manual_duty,
    output reg               manual_mode
);

    localparam [7:0] CH_K       = 8'h4B;  // 字符 'K'
    localparam [7:0] CH_P       = 8'h50;  // 字符 'P'
    localparam [7:0] CH_I       = 8'h49;  // 字符 'I'
    localparam [7:0] CH_D       = 8'h44;  // 字符 'D'
    localparam [7:0] CH_C       = 8'h43;  // 字符 'C'
    localparam [7:0] CH_L       = 8'h4C;  // 字符 'L'
    localparam [7:0] CH_Z       = 8'h5A;  // 字符 'Z'
    localparam [7:0] CH_F       = 8'h46;  // 字符 'F'
    localparam [7:0] CH_M       = 8'h4D;  // 字符 'M'
    localparam [7:0] CH_N       = 8'h4E;  // 字符 'N'
    localparam [7:0] CH_O       = 8'h4F;  // 字符 'O'
    localparam [7:0] CH_COLON   = 8'h3A;  // 冒号 ':'
    localparam [7:0] CH_MINUS   = 8'h2D;  // 负号 '-'
    localparam [7:0] CH_NEWLINE = 8'h0A;  // 换行 LF
    localparam [7:0] CH_CR      = 8'h0D;  // 回车 CR
    localparam [7:0] CH_ZERO    = 8'h30;  // 字符 '0'

    localparam [1:0] ST_IDLE  = 2'd0,
                     ST_CMD2  = 2'd1,
                     ST_COLON = 2'd2,
                     ST_VALUE = 2'd3;

    reg [1:0]         state;
    reg [7:0]         cmd1, cmd2;
    reg signed [31:0] val_acc;
    reg               is_neg;

    wire signed [31:0] digit_ext = {28'd0, rx_data[3:0]};
    wire signed [31:0] manual_val = is_neg ? -val_acc : val_acc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            cmd1        <= 8'd0;
            cmd2        <= 8'd0;
            val_acc     <= 32'sd0;
            is_neg      <= 1'b0;
            angle_kp    <= DEFAULT_ANGLE_KP;
            angle_ki    <= DEFAULT_ANGLE_KI;
            angle_kd    <= DEFAULT_ANGLE_KD;
            pos_kp      <= DEFAULT_POS_KP;
            pos_ki      <= DEFAULT_POS_KI;
            pos_kd      <= DEFAULT_POS_KD;
            zero_offset <= DEFAULT_ZERO_OFF;
            manual_duty <= 16'sd0;
            manual_mode <= 1'b0;
        end else if (rx_valid) begin
            case (state)
                ST_IDLE: begin
                    if (rx_data == CH_K || rx_data == CH_P ||
                        rx_data == CH_C || rx_data == CH_Z ||
                        rx_data == CH_M) begin
                        cmd1  <= rx_data;
                        state <= ST_CMD2;
                    end
                end

                ST_CMD2: begin
                    case (cmd1)
                        CH_K: begin
                            if (rx_data == CH_P || rx_data == CH_I ||
                                rx_data == CH_D) begin
                                cmd2  <= rx_data;
                                state <= ST_COLON;
                            end else begin
                                state <= ST_IDLE;
                            end
                        end

                        CH_P: begin
                            if (rx_data == CH_K || rx_data == CH_I ||
                                rx_data == CH_D) begin
                                cmd2  <= rx_data;
                                state <= ST_COLON;
                            end else begin
                                state <= ST_IDLE;
                            end
                        end

                        CH_C: begin
                            if (rx_data == CH_L) begin
                                angle_kp    <= DEFAULT_ANGLE_KP;
                                angle_ki    <= DEFAULT_ANGLE_KI;
                                angle_kd    <= DEFAULT_ANGLE_KD;
                                pos_kp      <= DEFAULT_POS_KP;
                                pos_ki      <= DEFAULT_POS_KI;
                                pos_kd      <= DEFAULT_POS_KD;
                                zero_offset <= DEFAULT_ZERO_OFF;
                            end
                            state <= ST_IDLE;
                        end

                        CH_Z: begin
                            if (rx_data == CH_F) begin
                                cmd2  <= rx_data;
                                state <= ST_COLON;
                            end else begin
                                state <= ST_IDLE;
                            end
                        end

                        CH_M: begin
                            if (rx_data == CH_N) begin
                                cmd2  <= rx_data;
                                state <= ST_COLON;
                            end else if (rx_data == CH_O) begin
                                manual_mode <= 1'b0;
                                state       <= ST_IDLE;
                            end else begin
                                state <= ST_IDLE;
                            end
                        end

                        default: state <= ST_IDLE;
                    endcase
                end

                ST_COLON: begin
                    if (rx_data == CH_COLON) begin
                        val_acc <= 32'sd0;
                        is_neg  <= 1'b0;
                        state   <= ST_VALUE;
                    end else begin
                        state <= ST_IDLE;
                    end
                end

                ST_VALUE: begin
                    if (rx_data == CH_MINUS &&
                        val_acc == 32'sd0 && !is_neg) begin
                        is_neg <= 1'b1;
                    end else if (rx_data >= CH_ZERO &&
                                 rx_data <= CH_ZERO + 8'd9) begin
                        val_acc <= val_acc * 10 + digit_ext;
                    end else if (rx_data == CH_NEWLINE ||
                                 rx_data == CH_CR) begin
                        case (cmd1)
                            CH_K: begin
                                case (cmd2)
                                    CH_P: angle_kp <= is_neg ? -val_acc : val_acc;
                                    CH_I: angle_ki <= is_neg ? -val_acc : val_acc;
                                    CH_D: angle_kd <= is_neg ? -val_acc : val_acc;
                                endcase
                            end

                            CH_P: begin
                                case (cmd2)
                                    CH_K: pos_kp <= is_neg ? -val_acc : val_acc;
                                    CH_I: pos_ki <= is_neg ? -val_acc : val_acc;
                                    CH_D: pos_kd <= is_neg ? -val_acc : val_acc;
                                endcase
                            end

                            CH_Z: begin
                                if (cmd2 == CH_F)
                                    zero_offset <= is_neg ? -val_acc : val_acc;
                            end

                            CH_M: begin
                                if (cmd2 == CH_N) begin
                                    manual_duty <= manual_val[15:0];
                                    manual_mode <= 1'b1;
                                end
                            end
                        endcase
                        state <= ST_IDLE;
                    end else begin
                        state <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
