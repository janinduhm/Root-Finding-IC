`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: fpu_compare
// Magnitude/ordering comparison of two IEEE-754 half-precision values.
//
// This is cheap thanks to a property of the format: for two values of the SAME
// sign, the 16-bit pattern read as an unsigned integer orders the same way as
// the value. Sign, exponent and fraction sit in exactly that order in the word,
// and the exponent is biased, so it is unsigned. A larger exponent gives a
// larger pattern, and within one exponent a larger fraction gives a larger
// pattern.
//
// So the comparison is three cases:
//   signs differ    -> the positive one is greater, decided by the sign bits
//                      alone; the magnitudes are irrelevant
//   both positive   -> compare the low 15 bits as unsigned
//   both negative   -> compare the low 15 bits as unsigned and INVERT, since a
//                      larger magnitude is a more negative value
//
// KNOWN LIMITATION: negative zero. 0x0000 and 0x8000 are both zero and should
// compare equal, but the signs-differ rule reports 0x0000 > 0x8000. floatadder
// can produce 0x8000 when the operands cancel exactly and the first operand is
// the negative one. Not reached by bisection, which brackets with positive
// values, but it is a real hole in the ordering.
//
// Registered (1 clock cycle latency).
//////////////////////////////////////////////////////////////////////////////
module fpu_compare #(parameter W = 15)
(
    input               clk,
    input      [W:0]    x,
    input      [W:0]    y,
    output reg          gt,   // x >  y
    output reg          ge    // x >= y
);
    wire        sx = x[15],       sy = y[15];
    wire [14:0] mx = x[14:0],     my = y[14:0];   // exponent and fraction

    wire same_sign = (sx == sy);

    // Within one sign, ordering follows the unsigned pattern; negatives invert.
    wire mag_gt = sx ? (mx < my) : (mx > my);
    wire mag_eq = (mx == my);

    always @(posedge clk) begin
        // Signs differ: x is greater exactly when y is the negative one.
        gt <= same_sign ? mag_gt            : sy;
        ge <= same_sign ? (mag_gt | mag_eq) : sy;
    end
endmodule
