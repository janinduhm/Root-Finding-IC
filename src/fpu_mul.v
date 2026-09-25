`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: fpu_mul
// IEEE-754 half-precision multiplier.
//
// A thin registered wrapper around the combinational `float_multi` core, which
// forms the full 22-bit product by shift-and-add, removes the double-counted
// exponent bias, and normalizes. See addmut.v.
//
// The scale lives in the exponent, so rescaling the product is an addition on
// the exponent field rather than a shift on the mantissa -- float_multi
// handles it internally, and nothing here has to widen or realign.
//
// Registered (1 clock cycle latency).
//////////////////////////////////////////////////////////////////////////////
module fpu_mul #(parameter W = 15)
(
    input               clk,
    input      [W:0]    a,
    input      [W:0]    b,
    output reg [W:0]    result
);
    wire [15:0] prod_comb;

    float_multi u_core (.num1(a), .num2(b), .result(prod_comb));

    always @(posedge clk) begin
        result <= prod_comb;
    end
endmodule
