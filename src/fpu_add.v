`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: fpu_add
// IEEE-754 half-precision adder.
//
// A thin registered wrapper around the combinational `floatadder` core, which
// handles alignment, the carry-out path, cancellation via `norm`, and the
// reserved zero encoding. See addmut.v for the format and the algorithm.
//
// Registered (1 clock cycle latency), consistent with every other FPU
// operation in this design -- the MAU always allocates at least one
// state/cycle for an FPU result to become valid.
//
// W defaults to 15 so [W:0] is 16 bits, the binary16 width.
//////////////////////////////////////////////////////////////////////////////
module fpu_add #(parameter W = 15)
(
    input               clk,
    input      [W:0]    a,
    input      [W:0]    b,
    output reg [W:0]    sum
);
    wire [15:0] sum_comb;

    floatadder u_core (.num1(a), .num2(b), .result(sum_comb));

    always @(posedge clk) begin
        sum <= sum_comb;
    end
endmodule


//////////////////////////////////////////////////////////////////////////////
// Module: fpu_sub
// IEEE-754 half-precision subtractor.
//
// There is no subtraction hardware: a - b is a + (-b), and negating a float is
// inverting the sign bit. So this is the same adder core with b[15] flipped on
// the way in.
//
// It exists as its own module only so the MAU can issue "subtract" directly
// rather than having to negate an operand itself. Costs one inverter.
//
// Registered (1 clock cycle latency), same as fpu_add.
//////////////////////////////////////////////////////////////////////////////
module fpu_sub #(parameter W = 15)
(
    input               clk,
    input      [W:0]    a,
    input      [W:0]    b,
    output reg [W:0]    diff
);
    wire [15:0] b_negated = {~b[15], b[14:0]};
    wire [15:0] diff_comb;

    floatadder u_core (.num1(a), .num2(b_negated), .result(diff_comb));

    always @(posedge clk) begin
        diff <= diff_comb;
    end
endmodule
