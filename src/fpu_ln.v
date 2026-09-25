`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: fpu_ln
// ln(1+x) by a 5th-order Chebyshev polynomial, IEEE-754 half-precision.
//
//   f(x) = p0 + p1*x + p2*x^2 + p3*x^3 + p4*x^4 + p5*x^5
//
// evaluated by Horner's method, which nests the powers so each term costs one
// multiply and one add rather than raising x to a power separately:
//
//   f(x) = p0 + x*(p1 + x*(p2 + x*(p3 + x*(p4 + x*p5))))
//
// WHY THIS IS AN UNROLLED CHAIN RATHER THAN A LOOP
//   Half-precision arithmetic lives in the float_multi and floatadder MODULES,
//   and module instantiation is structural -- it cannot appear inside a loop or
//   an always block, because hardware is fabricated, not executed. So the five
//   Horner stages are instantiated explicitly, one multiplier and one adder
//   each.
//
//   The chain is combinational; only the input and output are registered,
//   preserving the 2-cycle latency the MAU allocates (EVAL_LN1, EVAL_LN2).
//   Ten arithmetic units and a deep combinational path is the price; the
//   alternative was reusing one multiply-add across five cycles, which would
//   have meant widening the MAU's ln states.
//
// COEFFICIENTS
//
//     term │ fitted value │  encoded   │ encoding │ error
//     -----┼──────────────┼────────────┼──────────┼--------
//      p0  │    0.0000153 │  0         │  0x0000  │  see below
//      p1  │    0.999161  │  0.999023  │  0x3BFE  │  1.4e-4
//      p2  │   -0.489700  │ -0.489746  │  0xB7D6  │  4.6e-5
//      p3  │    0.283813  │  0.283691  │  0x348A  │  1.2e-4
//      p4  │   -0.129959  │ -0.130005  │  0xB029  │  4.6e-5
//      p5  │    0.029816  │  0.029816  │  0x27A2  │  ~0
//
//   p0 UNDERFLOWS: 1.53e-5 is 2^-16, below the smallest normal of 2^-14, so it
//   cannot be represented and is zero here. That is arguably the better value
//   anyway -- ln(1+0) = 0 exactly, so a nonzero constant term is an artifact of
//   the polynomial fit rather than something the function needs. The final
//   adder stage is kept so the structure stays uniform and a future refit can
//   drop a real constant in.
//
// ACCURACY
//   Half-precision carries 11 significant bits, about 3 decimal digits, and
//   each Horner stage truncates. The coefficient rounding errors above are
//   well below the arithmetic truncation and are not the limiting factor.
//////////////////////////////////////////////////////////////////////////////
module fpu_ln #(parameter W = 15)
(
    input               clk,
    input      [W:0]    x_in,
    output reg [W:0]    f_out
);
    // --- Chebyshev coefficients, binary16 -----------------------------------
    localparam [15:0] P0 = 16'h0000;   //  0          (fitted term underflows)
    localparam [15:0] P1 = 16'h3BFE;   //  0.999023
    localparam [15:0] P2 = 16'hB7D6;   // -0.489746
    localparam [15:0] P3 = 16'h348A;   //  0.283691
    localparam [15:0] P4 = 16'hB029;   // -0.130005
    localparam [15:0] P5 = 16'h27A2;   //  0.029816

    // --- Stage 1 (Store): capture the input ---------------------------------
    reg [15:0] x;
    always @(posedge clk) begin
        x <= x_in;
    end

    // --- Stage 2 (SOP): the Horner chain, combinational ---------------------
    // Each stage computes  s_k = p_k + x * s_{k+1},  innermost first.
    wire [15:0] m4, s4;   // x*P5      then + P4
    wire [15:0] m3, s3;   // x*s4      then + P3
    wire [15:0] m2, s2;   // x*s3      then + P2
    wire [15:0] m1, s1;   // x*s2      then + P1
    wire [15:0] m0, s0;   // x*s1      then + P0

    float_multi u_m4 (.num1(x), .num2(P5), .result(m4));
    floatadder  u_a4 (.num1(m4), .num2(P4), .result(s4));

    float_multi u_m3 (.num1(x), .num2(s4), .result(m3));
    floatadder  u_a3 (.num1(m3), .num2(P3), .result(s3));

    float_multi u_m2 (.num1(x), .num2(s3), .result(m2));
    floatadder  u_a2 (.num1(m2), .num2(P2), .result(s2));

    float_multi u_m1 (.num1(x), .num2(s2), .result(m1));
    floatadder  u_a1 (.num1(m1), .num2(P1), .result(s1));

    float_multi u_m0 (.num1(x), .num2(s1), .result(m0));
    floatadder  u_a0 (.num1(m0), .num2(P0), .result(s0));

    always @(posedge clk) begin
        f_out <= s0;
    end
endmodule
