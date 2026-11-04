// 旋转倒立摆串级平衡控制器。
// 位置环以 100 Hz 运行，为 1 kHz 的角度环提供目标倾角。
// 位置环阻尼通过 pos_kd 直接叠加到电机指令上。
module pid_controller #(
    parameter signed [15:0] ANGLE_OUT_MAX = 16'sd2000,
    parameter signed [15:0] POS_OUT_MAX   = 16'sd40,
    parameter ANGLE_DIV = 1,       // 角度环周期：1 ms
    parameter POS_DIV   = 10,      // 位置环周期：10 ms
    parameter MOTOR_DIR = 0
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_en,
    input  wire signed [15:0] angle_err,
    input  wire signed [31:0] pos_cnt,
    input  wire signed [15:0] pos_target,
    input  wire signed [15:0] speed,
    input  wire               speed_mode,
    input  wire signed [15:0] target_speed,
    input  wire               balance_en,
    input  wire               pid_clear,
    input  wire               cal_done,
    input  wire signed [31:0] angle_kp,
    input  wire signed [31:0] angle_ki,
    input  wire signed [31:0] angle_kd,
    input  wire signed [31:0] pos_kp,
    input  wire signed [31:0] pos_ki,
    input  wire signed [31:0] pos_kd,
    output reg  signed [15:0] pwm_duty,
    output wire signed [15:0] dbg_pos_out,
    output wire signed [15:0] dbg_ang_err,
    output wire signed [15:0] dbg_pos_err
);

    // 将单周期电平 sample_en 转成单周期脉冲。
    reg sample_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sample_d <= 1'b0;
        else
            sample_d <= sample_en;
    end
    wire sample_pulse = sample_en & ~sample_d;

    reg [7:0] ang_div;
    reg [7:0] pos_div;
    reg       ang_beat;
    reg       pos_beat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ang_div  <= 8'd0;
            pos_div  <= 8'd0;
            ang_beat <= 1'b0;
            pos_beat <= 1'b0;
        end else if (sample_pulse) begin
            if (ang_div == ANGLE_DIV - 1) begin
                ang_div  <= 8'd0;
                ang_beat <= 1'b1;
            end else begin
                ang_div  <= ang_div + 8'd1;
                ang_beat <= 1'b0;
            end

            if (pos_div == POS_DIV - 1) begin
                pos_div  <= 8'd0;
                pos_beat <= 1'b1;
            end else begin
                pos_div  <= pos_div + 8'd1;
                pos_beat <= 1'b0;
            end
        end else begin
            ang_beat <= 1'b0;
            pos_beat <= 1'b0;
        end
    end

    // 位置环。速度模式下变为速度误差环。
    reg signed [31:0] pos_err, pos_integ;
    reg signed [31:0] pos_out_raw;
    reg signed [15:0] pos_out;

    localparam signed [31:0] POS_INTEG_MAX = 32'sd8192;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_err     <= 32'sd0;
            pos_integ   <= 32'sd0;
            pos_out_raw <= 32'sd0;
        end else if (pid_clear) begin
            pos_integ   <= 32'sd0;
            pos_out_raw <= 32'sd0;
        end else if (pos_beat) begin
            if (speed_mode) begin
                pos_err <= ($signed({{16{target_speed[15]}}, target_speed}) -
                            $signed({{16{speed[15]}}, speed})) <<< 7;
                pos_integ   <= 32'sd0;
                pos_out_raw <= (pos_kp * pos_err) >>> 10;
            end else begin
                pos_err <= pos_target - pos_cnt;

                if (pos_ki != 0) begin
                    if (pos_integ + pos_err > POS_INTEG_MAX)
                        pos_integ <= POS_INTEG_MAX;
                    else if (pos_integ + pos_err < -POS_INTEG_MAX)
                        pos_integ <= -POS_INTEG_MAX;
                    else
                        pos_integ <= pos_integ + pos_err;
                end else begin
                    pos_integ <= 32'sd0;
                end

                pos_out_raw <= ((pos_kp * pos_err) >>> 10) +
                               ((pos_ki * pos_integ) >>> 10);
            end
        end
    end

    always @(*) begin
        if (pos_out_raw > POS_OUT_MAX)
            pos_out = POS_OUT_MAX;
        else if (pos_out_raw < -POS_OUT_MAX)
            pos_out = -POS_OUT_MAX;
        else
            pos_out = pos_out_raw[15:0];
    end

    // 角度环，微分采用 5 ms 窗口差分。
    reg signed [31:0] ang_err_d1, ang_err_d2, ang_err_d3;
    reg signed [31:0] ang_err_d4, ang_err_d5;
    reg signed [31:0] ang_integ;
    reg signed [31:0] ang_out_raw;

    localparam signed [31:0] ANG_INTEG_MAX = 32'sd131072;

    wire signed [31:0] ang_err_now =
        $signed({{16{pos_out[15]}}, pos_out}) -
        $signed({{16{angle_err[15]}}, angle_err});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ang_err_d1  <= 32'sd0;
            ang_err_d2  <= 32'sd0;
            ang_err_d3  <= 32'sd0;
            ang_err_d4  <= 32'sd0;
            ang_err_d5  <= 32'sd0;
            ang_integ   <= 32'sd0;
            ang_out_raw <= 32'sd0;
        end else if (pid_clear) begin
            // 预填微分历史，避免接棒瞬间产生微分冲击。
            ang_err_d1  <= ang_err_now;
            ang_err_d2  <= ang_err_now;
            ang_err_d3  <= ang_err_now;
            ang_err_d4  <= ang_err_now;
            ang_err_d5  <= ang_err_now;
            ang_integ   <= 32'sd0;
            ang_out_raw <= angle_kp * ang_err_now;
        end else if (ang_beat) begin
            ang_err_d5 <= ang_err_d4;
            ang_err_d4 <= ang_err_d3;
            ang_err_d3 <= ang_err_d2;
            ang_err_d2 <= ang_err_d1;
            ang_err_d1 <= ang_err_now;

            ang_out_raw <= angle_kp * ang_err_now
                         + ((angle_ki * ang_integ) >>> 7)
                         + angle_kd * (ang_err_now - ang_err_d5);

            if (angle_ki != 0) begin
                if (ang_integ + ang_err_now > ANG_INTEG_MAX)
                    ang_integ <= ANG_INTEG_MAX;
                else if (ang_integ + ang_err_now < -ANG_INTEG_MAX)
                    ang_integ <= -ANG_INTEG_MAX;
                else
                    ang_integ <= ang_integ + ang_err_now;
            end else begin
                ang_integ <= 32'sd0;
            end
        end
    end

    // 位置环速度阻尼直接前馈。
    wire signed [31:0] speed_ext = {{16{speed[15]}}, speed};
    wire signed [63:0] damp64    = pos_kd * speed_ext;
    wire signed [31:0] damp      = damp64[31:0];
    wire signed [31:0] total_raw = ang_out_raw - damp;

    reg signed [15:0] total_lim;
    always @(*) begin
        if (total_raw > ANGLE_OUT_MAX)
            total_lim = ANGLE_OUT_MAX;
        else if (total_raw < -ANGLE_OUT_MAX)
            total_lim = -ANGLE_OUT_MAX;
        else
            total_lim = total_raw[15:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_duty <= 16'sd0;
        end else if (balance_en && cal_done) begin
            pwm_duty <= MOTOR_DIR ? -total_lim : total_lim;
        end else begin
            pwm_duty <= 16'sd0;
        end
    end

    assign dbg_pos_out = pos_out;
    assign dbg_ang_err = ang_err_now[15:0];
    assign dbg_pos_err = pos_err[15:0];

endmodule
