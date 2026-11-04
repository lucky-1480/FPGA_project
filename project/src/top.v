// 基于 GW2A-55 的 J280 倒立摆控制器。
//
// 按键控制：
//   SW1          启动/停止
//   SW2/SW3      位置目标 +1/-1 圈
//   SW4          位置模式 <-> 速度伺服模式
//   dip[0]       1：从起摆开始，0：直接进入稳摆模式
//   dip[1]       1：运行时反相电机方向
//
// VOFA+ 通过板载 CH340 串口以 100 Hz 发送八个通道：
//   ch0 角度目标，ch1 角度误差，ch2 角度反馈
//   ch3 位置目标，ch4 位置误差，ch5 位置反馈
//   ch6 最终 PWM 占空比，ch7 {手动模式, 状态机}
//
// UART 命令（115200 8N1，以 LF/CR 结尾）：
//   KP:n KI:n KD:n  PK:n PI:n PD:n  ZF:n  MN:n  MO  CL
module top #(
    parameter signed [15:0] FALL_TH   = 16'sd160,
    parameter signed [15:0] POS_STEP  = 16'sd1040,
    parameter signed [15:0] POS_LIMIT = 16'sd10400
)(
    input  wire        clk_50M,
    input  wire        rst_n,
    input  wire [9:0]  adc_d,
    input  wire        adc_otr,
    output wire        adc_clk,
    output wire        adc_oe_n,
    input  wire        enc_a,
    input  wire        enc_b,
    input  wire        enc_a2,
    input  wire        enc_b2,
    output wire        pwm_a,
    output wire        ain1,
    output wire        ain2,
    input  wire [3:0]  key,
    input  wire [1:0]  dip,
    output reg  [3:0]  led,
    output wire        uart_txd,
    input  wire        uart_rxd
);

    // 同步异步复位输入。
    reg rst_s0, rst_s1;
    always @(posedge clk_50M) begin
        rst_s0 <= rst_n;
        rst_s1 <= rst_s0;
    end
    wire rst_sync_n = rst_s1;

    // 两个编码器接口共用累计位置与速度。
    wire                sample_en;
    wire signed [31:0]  pos_cnt1, pos_cnt2;
    wire signed [15:0]  speed1, speed2;

    encoder_decoder u_enc1 (
        .clk       (clk_50M),
        .rst_n     (rst_sync_n),
        .enc_a     (enc_a),
        .enc_b     (enc_b),
        .pos_cnt   (pos_cnt1),
        .speed     (speed1),
        .sample_en (sample_en)
    );

    encoder_decoder u_enc2 (
        .clk       (clk_50M),
        .rst_n     (rst_sync_n),
        .enc_a     (enc_a2),
        .enc_b     (enc_b2),
        .pos_cnt   (pos_cnt2),
        .speed     (speed2),
        .sample_en ()
    );

    wire signed [31:0] pos_cnt = pos_cnt1 + pos_cnt2;
    wire signed [15:0] speed = speed1 + speed2;

    // 角度采集与上电零点校准。
    wire signed [15:0] angle_diff;
    wire               cal_done;
    wire signed [31:0] zero_offset;

    angle_sensor u_ang (
        .clk         (clk_50M),
        .rst_n       (rst_sync_n),
        .adc_d       (adc_d),
        .adc_otr     (adc_otr),
        .adc_clk     (adc_clk),
        .adc_oe_n    (adc_oe_n),
        .sample_en   (sample_en),
        .zero_offset (zero_offset[15:0]),
        .angle_diff  (angle_diff),
        .cal_done    (cal_done)
    );

    wire signed [15:0] swing_duty;
    wire               catch_flag;
    swing_up u_swing (
        .clk          (clk_50M),
        .rst_n        (rst_sync_n),
        .sample_pulse (sample_en),
        .angle_err    (angle_diff),
        .duty         (swing_duty),
        .catch_flag   (catch_flag)
    );

    // 位置目标与按键消抖。
    reg signed [31:0] pos_ref;
    reg signed [15:0] pos_target;
    reg               speed_mode;
    localparam signed [15:0] TARGET_SPEED = 16'sd2;

    wire k1_pulse, k2_pulse, k3_pulse, k4_pulse;
    key_filter u_key1 (.clk(clk_50M), .rst_n(rst_sync_n), .key_in(key[0]), .key_pulse(k1_pulse));
    key_filter u_key2 (.clk(clk_50M), .rst_n(rst_sync_n), .key_in(key[1]), .key_pulse(k2_pulse));
    key_filter u_key3 (.clk(clk_50M), .rst_n(rst_sync_n), .key_in(key[2]), .key_pulse(k3_pulse));
    key_filter u_key4 (.clk(clk_50M), .rst_n(rst_sync_n), .key_in(key[3]), .key_pulse(k4_pulse));

    wire signed [31:0] pos_rel = pos_cnt - pos_ref;

    // 目标位置每毫秒斜坡 2 个编码器计数。
    reg signed [15:0] pos_cmd;
    localparam signed [15:0] RAMP_STEP = 16'sd2;
    always @(posedge clk_50M or negedge rst_sync_n) begin
        if (!rst_sync_n) begin
            pos_cmd <= 16'sd0;
        end else if (sample_en) begin
            if (pos_cmd < pos_target) begin
                if (pos_target - pos_cmd <= RAMP_STEP)
                    pos_cmd <= pos_target;
                else
                    pos_cmd <= pos_cmd + RAMP_STEP;
            end else if (pos_cmd > pos_target) begin
                if (pos_cmd - pos_target <= RAMP_STEP)
                    pos_cmd <= pos_target;
                else
                    pos_cmd <= pos_cmd - RAMP_STEP;
            end
        end
    end

    localparam [1:0] ST_STOP  = 2'd0,
                     ST_SWING = 2'd1,
                     ST_BAL   = 2'd2;
    reg [1:0] state;
    reg       pid_clear;

    wire signed [15:0] angle_abs =
        (angle_diff >= 0) ? angle_diff : -angle_diff;
    wire fallen = (angle_abs > FALL_TH);

    always @(posedge clk_50M or negedge rst_sync_n) begin
        if (!rst_sync_n) begin
            state      <= ST_STOP;
            pid_clear  <= 1'b0;
            pos_ref    <= 32'sd0;
            pos_target <= 16'sd0;
            speed_mode <= 1'b0;
        end else begin
            pid_clear <= 1'b0;

            if (k4_pulse) begin
                speed_mode <= ~speed_mode;
                if (speed_mode) begin
                    pos_ref    <= pos_cnt;
                    pos_target <= 16'sd0;
                end
            end else if (k2_pulse && (pos_target < POS_LIMIT)) begin
                pos_target <= pos_target + POS_STEP;
            end else if (k3_pulse && (pos_target > -POS_LIMIT)) begin
                pos_target <= pos_target - POS_STEP;
            end

            case (state)
                ST_STOP: begin
                    if (k1_pulse && cal_done) begin
                        pos_ref    <= pos_cnt;
                        pos_target <= 16'sd0;
                        if (dip[0])
                            state <= ST_SWING;
                        else
                            state <= ST_BAL;
                    end
                end

                ST_SWING: begin
                    if (k1_pulse) begin
                        state <= ST_STOP;
                    end else if (catch_flag) begin
                        pos_ref   <= pos_cnt;
                        pos_target <= 16'sd0;
                        pid_clear <= 1'b1;
                        state     <= ST_BAL;
                    end
                end

                ST_BAL: begin
                    if (k1_pulse)
                        state <= ST_STOP;
                    else if (fallen)
                        state <= ST_SWING;
                end

                default: state <= ST_STOP;
            endcase
        end
    end

    // 运行时 PID 参数与 UART 接收通道。
    wire [7:0] rx_data;
    wire       rx_valid;
    wire signed [31:0] angle_kp, angle_ki, angle_kd;
    wire signed [31:0] pos_kp, pos_ki, pos_kd;
    wire signed [15:0] manual_duty;
    wire               manual_mode;

    param_parser #(
        .DEFAULT_ANGLE_KP (32'sd40),
        .DEFAULT_ANGLE_KI (32'sd0),
        .DEFAULT_ANGLE_KD (32'sd80),
        .DEFAULT_POS_KP   (32'sd40),
        .DEFAULT_POS_KI   (32'sd0),
        .DEFAULT_POS_KD   (32'sd800)
    ) u_param (
        .clk         (clk_50M),
        .rst_n       (rst_sync_n),
        .rx_data     (rx_data),
        .rx_valid    (rx_valid),
        .angle_kp    (angle_kp),
        .angle_ki    (angle_ki),
        .angle_kd    (angle_kd),
        .pos_kp      (pos_kp),
        .pos_ki      (pos_ki),
        .pos_kd      (pos_kd),
        .zero_offset (zero_offset),
        .manual_duty (manual_duty),
        .manual_mode (manual_mode)
    );

    wire signed [15:0] pid_duty;
    wire signed [15:0] dbg_pos_out;
    wire signed [15:0] dbg_ang_err;
    wire signed [15:0] dbg_pos_err;

    pid_controller u_pid (
        .clk          (clk_50M),
        .rst_n        (rst_sync_n),
        .sample_en    (sample_en),
        .angle_err    (angle_diff),
        .pos_cnt      (pos_rel),
        .pos_target   (pos_cmd),
        .speed        (speed),
        .speed_mode   (speed_mode),
        .target_speed (TARGET_SPEED),
        .balance_en   (state == ST_BAL),
        .pid_clear    (pid_clear),
        .cal_done     (cal_done),
        .angle_kp     (angle_kp),
        .angle_ki     (angle_ki),
        .angle_kd     (angle_kd),
        .pos_kp       (pos_kp),
        .pos_ki       (pos_ki),
        .pos_kd       (pos_kd),
        .pwm_duty     (pid_duty),
        .dbg_pos_out  (dbg_pos_out),
        .dbg_ang_err  (dbg_ang_err),
        .dbg_pos_err  (dbg_pos_err)
    );

    reg signed [15:0] duty_sel;
    always @(*) begin
        case (state)
            ST_SWING: duty_sel = swing_duty;
            ST_BAL:   duty_sel = pid_duty;
            default:  duty_sel = 16'sd0;
        endcase
    end

    wire signed [15:0] duty_final = manual_mode
        ? manual_duty
        : (dip[1] ? -duty_sel : duty_sel);

    pwm_gen u_pwm (
        .clk     (clk_50M),
        .rst_n   (rst_sync_n),
        .duty_in (duty_final),
        .pwm_out (pwm_a),
        .in1     (ain1),
        .in2     (ain2)
    );

    uart_rx #(
        .CLK_FREQ  (50_000_000),
        .BAUD_RATE (115_200)
    ) u_uart_rx (
        .clk      (clk_50M),
        .rst_n    (rst_sync_n),
        .rxd      (uart_rxd),
        .rx_data  (rx_data),
        .rx_valid (rx_valid)
    );

    wire [7:0] tx_data;
    wire       tx_start;
    wire       tx_busy;
    uart_tx #(
        .CLK_FREQ  (50_000_000),
        .BAUD_RATE (115_200)
    ) u_uart_tx (
        .clk      (clk_50M),
        .rst_n    (rst_sync_n),
        .tx_data  (tx_data),
        .tx_start (tx_start),
        .tx_busy  (tx_busy),
        .txd      (uart_txd)
    );

    // 每 10 ms 触发一帧 36 字节的 VOFA+ 数据。
    reg [3:0] vofa_div;
    reg       vofa_send;
    always @(posedge clk_50M or negedge rst_sync_n) begin
        if (!rst_sync_n) begin
            vofa_div  <= 4'd0;
            vofa_send <= 1'b0;
        end else begin
            vofa_send <= 1'b0;
            if (sample_en) begin
                if (vofa_div == 4'd9) begin
                    vofa_div  <= 4'd0;
                    vofa_send <= 1'b1;
                end else begin
                    vofa_div <= vofa_div + 4'd1;
                end
            end
        end
    end

    vofa_tx #(
        .N_CH (8)
    ) u_vofa (
        .clk        (clk_50M),
        .rst_n      (rst_sync_n),
        .send_pulse (vofa_send),
        .ch0        (dbg_pos_out),
        .ch1        (dbg_ang_err),
        .ch2        (angle_diff),
        .ch3        (pos_cmd),
        .ch4        (dbg_pos_err),
        .ch5        (pos_rel[15:0]),
        .ch6        (duty_final),
        .ch7        ({13'd0, manual_mode, state}),
        .tx_data    (tx_data),
        .tx_start   (tx_start),
        .tx_busy    (tx_busy)
    );

    always @(posedge clk_50M or negedge rst_sync_n) begin
        if (!rst_sync_n) begin
            led <= 4'b0000;
        end else begin
            led[0] <= (state != ST_STOP);
            led[1] <= (state == ST_SWING);
            led[2] <= (state == ST_BAL);
            led[3] <= cal_done;
        end
    end

endmodule
