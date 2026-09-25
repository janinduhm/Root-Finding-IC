`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Testbench: tb_waveform_demo
// Drives root_finder_top with omega=1.0 and dumps a VCD waveform showing
// the bisection loop converging, for documentation screenshots.
//////////////////////////////////////////////////////////////////////////////
module tb_waveform_demo;
    parameter W = 15;

    reg        Clock, Reset, Start;
    reg  [W:0] omega, epsilon;
    reg [15:0] Nmax;
    wire [W:0] x_hat;
    wire       error, Ready;

    // Decode a binary16 word back to a real, for the closing $display.
    function real from_fp16(input [15:0] v);
        real m;
        integer k, i;
        begin
            if (v[14:0] == 15'd0) begin
                from_fp16 = 0.0;
            end else begin
                m = 1.0;
                for (i = 0; i < 10; i = i + 1)
                    if (v[i]) m = m + (1.0 / (1 << (10 - i)));
                k = v[14:10] - 15;
                if (k >= 0) m = m * (1 << k);
                else        m = m / (1 << (-k));
                from_fp16 = v[15] ? -m : m;
            end
        end
    endfunction

    root_finder_top #(W) dut (
        .Clock(Clock), .Reset(Reset), .Start(Start),
        .omega(omega), .epsilon(epsilon), .Nmax(Nmax),
        .x_hat(x_hat), .error(error), .Ready(Ready)
    );

    always #5 Clock = ~Clock;

    initial begin
        $dumpfile("sim/waveform_demo.vcd");
        $dumpvars(0, tb_waveform_demo);

        Clock = 0; Reset = 1; Start = 0;
        omega   = 16'h3C00;   // 1.0   = 0 01111 0000000000
        epsilon = 16'h1419;   // ~0.001 = 0 00101 0000011001
        Nmax    = 16'd20;

        repeat (3) @(posedge Clock);
        Reset = 0;
        @(posedge Clock);
        Start = 1;
        @(posedge Clock);
        Start = 0;

        wait (Ready == 1'b1);
        @(posedge Clock);
        $display("x_hat=%f error=%b", from_fp16(x_hat), error);
        $finish;
    end
endmodule
