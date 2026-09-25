`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: fpu_div
// IEEE-754 half-precision divider.
//
// A thin registered wrapper around the combinational `float_divi` core, which
// performs restoring division on the mantissas, restores the cancelled
// exponent bias, and normalizes. See addmut.v.
//
// Bisection itself never needs this -- it halves an interval rather than
// dividing by an arbitrary value, which fpu_divby2 does far more cheaply. It
// is provided for completeness of the arithmetic set.
//
// NOTE: division by the zero encoding is not detected -- see the comment at
// the end of float_divi.
//
// Registered (1 clock cycle latency).
//////////////////////////////////////////////////////////////////////////////
module fpu_div #(parameter W = 15)
(
    input               clk,
    input      [W:0]    a,
    input      [W:0]    b,
    output reg [W:0]    result
);
    wire [15:0] quot_comb;

    float_divi u_core (.num1(a), .num2(b), .result(quot_comb));

    always @(posedge clk) begin
        result <= quot_comb;
    end
endmodule
