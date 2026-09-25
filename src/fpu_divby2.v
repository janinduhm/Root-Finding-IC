`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: fpu_divby2
// Halve an IEEE-754 half-precision value.
//
// The mantissa does NOT move. Halving a value means decrementing the exponent
// by one and leaving sign and fraction untouched -- the mantissa still sits in
// [1,2), so nothing needs renormalizing. A 5-bit decrement, no shifter:
//
//     value = 1.F * 2^(E-15)
//     half  = 1.F * 2^(E-16)        same 1.F, E one lower
//
// TWO CASES NEED CARE
//   E = 0  : the reserved zero encoding. Half of zero is zero, and
//            decrementing would turn it into a large finite number, so zero
//            passes through untouched.
//   E = 1  : decrementing reaches E = 0, which does not mean 2^(1-16) -- it
//            means zero. The true answer is a denormal, which this design does
//            not implement, so the result flushes to zero. Bisection on [0,1]
//            reaches E = 1 only after ~14 halvings of an already tiny
//            interval, well past any useful epsilon.
//
// Registered (1 clock cycle latency), consistent with every other FPU
// operation -- this is what makes RECOMPUTE a clean 2-cycle state (1 cycle
// add, 1 cycle divide-by-2).
//////////////////////////////////////////////////////////////////////////////
module fpu_divby2 #(parameter W = 15)
(
    input               clk,
    input      [W:0]    a,
    output reg [W:0]    result
);
    wire        sign = a[15];
    wire [4:0]  exp  = a[14:10];
    wire [9:0]  frac = a[9:0];

    always @(posedge clk) begin
        if (exp == 5'd0 || exp == 5'd1)
            result <= {sign, 5'd0, 10'd0};   // zero in, or underflow out
        else
            result <= {sign, exp - 5'd1, frac};
    end
endmodule
