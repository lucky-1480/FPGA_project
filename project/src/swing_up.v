// 以方波扫频方式将摆杆起摆。
// 接近直立时反向施力刹车；catch_flag 通知顶层 PID 可以接管。
module swing_up #(
    parameter signed [15:0] SWING_DUTY = 16'sd700,
    parameter signed [15:0] CATCH_TH   = 16'sd90,
    parameter [15:0]        OSC_MIN    = 16'd300,
    parameter [15:0]        OSC_MAX    = 16'd420,
    parameter [15:0]        OSC_STEP   = 16'd20,
    parameter signed [15:0] NEAR_TH    = 16'sd180,
    parameter signed [15:0] BRAKE_DUTY = 16'sd1200,
    parameter               DIR_INVERT = 0
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_pulse,
    input  wire signed [15:0] angle_err,
    output reg  signed [15:0] duty,
    output reg                catch_flag
);

    // 在 OSC_MIN 与 OSC_MAX 之间扫频方波半周期。
    reg [15:0] osc_timer;
    reg        osc_dir;
    reg [15:0] osc_half;
    reg        sweep_dir;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            osc_timer <= 16'd0;
            osc_dir   <= 1'b0;
            osc_half  <= OSC_MAX;
            sweep_dir <= 1'b1;
        end else if (sample_pulse) begin
            if (osc_timer >= osc_half) begin
                osc_timer <= 16'd0;
                osc_dir   <= ~osc_dir;

                if (sweep_dir) begin
                    if (osc_half <= OSC_MIN + OSC_STEP) begin
                        osc_half  <= OSC_MIN;
                        sweep_dir <= 1'b0;
                    end else begin
                        osc_half <= osc_half - OSC_STEP;
                    end
                end else begin
                    if (osc_half >= OSC_MAX - OSC_STEP) begin
                        osc_half  <= OSC_MAX;
                        sweep_dir <= 1'b1;
                    end else begin
                        osc_half <= osc_half + OSC_STEP;
                    end
                end
            end else begin
                osc_timer <= osc_timer + 16'd1;
            end
        end
    end

    wire in_brake = (angle_err > -NEAR_TH) && (angle_err < NEAR_TH);
    wire signed [15:0] osc_force =
        osc_dir ? SWING_DUTY : -SWING_DUTY;
    wire signed [15:0] brake_force =
        osc_dir ? -BRAKE_DUTY : BRAKE_DUTY;
    wire signed [15:0] swing_force =
        in_brake ? brake_force : osc_force;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty       <= 16'sd0;
            catch_flag <= 1'b0;
        end else if (sample_pulse) begin
            catch_flag <= (angle_err <  CATCH_TH) &&
                          (angle_err > -CATCH_TH);
            duty <= DIR_INVERT ? -swing_force : swing_force;
        end
    end

endmodule
