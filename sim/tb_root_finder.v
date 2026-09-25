`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Testbench: tb_root_finder
// Drives root_finder_top with several omega values and checks that it
// converges to the corresponding root of f(x) = omega*ln(x+1) + (x-1) = 0.
// Expected roots were independently computed by hand (real arithmetic),
// not derived from this design, to make this an honest check.
//
// Includes one deliberately under-resourced case (Nmax too small to reach
// epsilon) to confirm the error/ERROR_ST exit path itself works, not just the
// converging path.
//////////////////////////////////////////////////////////////////////////////
module tb_root_finder;
    parameter W = 15;                 // 16-bit binary16 word

    // Half-precision carries ~11 significant bits, roughly 3 decimal digits,
    // and the ln module's Horner chain truncates at every stage.
    localparam real TOL = 0.03;

    reg        Clock;
    reg        Reset;
    reg        Start;
    reg  [W:0] omega;
    reg  [W:0] epsilon;
    reg [15:0] Nmax;
    wire [W:0] x_hat;
    wire       error;
    wire       Ready;

    //-------------------------------------------------------------------------
    // Real <-> binary16 conversion
    //
    // There is no single scale factor to multiply by -- the exponent varies per
    // value -- so the conversion normalizes explicitly, exactly as the hardware
    // does: slide the value until it is in [1,2), record how far, then store
    // the fraction and the biased exponent.
    //-------------------------------------------------------------------------
    function [15:0] to_fp16(input real r);
        real    m;
        integer k, fr;
        reg     s;
        reg [4:0] e;
        reg [9:0] f;
        begin
            if (r == 0.0) begin
                to_fp16 = 16'h0000;           // the reserved zero encoding
            end else begin
                s = (r < 0.0);
                m = s ? -r : r;
                k = 0;
                while (m >= 2.0) begin m = m / 2.0; k = k + 1; end
                while (m <  1.0) begin m = m * 2.0; k = k - 1; end
                fr = $rtoi((m - 1.0) * 1024.0 + 0.5);   // round, not truncate
                if (fr > 1023) begin fr = 0; k = k + 1; end  // rounded up to 2.0
                e = k + 15;
                f = fr;
                to_fp16 = {s, e, f};
            end
        end
    endfunction

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


    always #5 Clock = ~Clock; // 10ns clock period

    real x_hat_real;
    integer pass_count, fail_count;

    task run_case(input real omega_real, input real expected_root, input integer nmax_val, input expect_error);
        begin
            omega   = to_fp16(omega_real);
            epsilon = to_fp16(0.001);
            Nmax    = nmax_val;

            Reset = 1;
            repeat (3) @(posedge Clock);
            Reset = 0;
            @(posedge Clock);
            Start = 1;
            @(posedge Clock);
            Start = 0;

            wait (Ready == 1'b1);
            @(posedge Clock);

            x_hat_real = from_fp16(x_hat);
            if (expect_error) begin
                if (error) begin
                    $display("PASS: omega=%f Nmax=%0d correctly reported error (did not converge in time)",
                        omega_real, nmax_val);
                    pass_count = pass_count + 1;
                end else begin
                    $display("FAIL: omega=%f Nmax=%0d expected error flag, but converged to x_hat=%f",
                        omega_real, nmax_val, x_hat_real);
                    fail_count = fail_count + 1;
                end
            end else if (error) begin
                $display("FAIL: omega=%f did not converge (error flag set)", omega_real);
                fail_count = fail_count + 1;
            end else if ((x_hat_real - expected_root > TOL) || (expected_root - x_hat_real > TOL)) begin
                $display("FAIL: omega=%f x_hat=%f expected=%f (diff=%f)",
                    omega_real, x_hat_real, expected_root, x_hat_real - expected_root);
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: omega=%f x_hat=%f expected=%f (diff=%f)",
                    omega_real, x_hat_real, expected_root, x_hat_real - expected_root);
                pass_count = pass_count + 1;
            end
        end
    endtask

    initial begin
        Clock = 0;
        Start = 0;
        pass_count = 0;
        fail_count = 0;

        // Root of f(x)=omega*ln(x+1)+(x-1)=0, found by hand for each omega:
        run_case(1.0, 0.5571, 20, 1'b0);
        run_case(1.5, 0.4465, 20, 1'b0);
        run_case(0.5, 0.7270, 20, 1'b0);
        // Deliberately too few iterations to reach epsilon=0.001 from the
        // [0,1] initial bracket (each iteration halves the interval; 3
        // iterations only gets to width 0.125) -- exercises ERROR_ST.
        run_case(1.0, 0.5571, 3, 1'b1);

        $display("---- %0d passed, %0d failed ----", pass_count, fail_count);
        $finish;
    end
endmodule
