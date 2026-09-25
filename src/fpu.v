`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: fpu
// Top-level Floating-Point Unit (Part 1 of the assignment spec).
//
// All sub-modules run continuously off the shared a/b operand bus and each
// expose their own dedicated result wire (no opcode, no result mux). The
// MAU -- which already knows exactly which operation it issued in which
// state -- simply reads whichever wire is relevant that cycle.
//
// (An earlier version muxed all results down to one shared `result` port,
// selected by an opcode input. That broke under simulation: the mux's
// selection is a combinational function of the CURRENT opcode, so the
// instant the MAU advances to the next operation and changes the opcode,
// the mux immediately starts showing the NEW operation's result -- even
// on the very cycle the MAU still needed to read the PREVIOUS operation's
// result. Exposing every result on its own wire removes the ambiguity.)
//
// Latency (cycles from presenting operands to a valid result):
//   add_result  = a + b   -- 1 cycle
//   sub_result  = a - b   -- 1 cycle
//   mul_result  = a * b   -- 1 cycle
//   div2_result = a / 2   -- 1 cycle
//   div_result  = a / b   -- 1 cycle
//   ln_result   = ln(a)   -- 2 cycles
//   gt = (a>b), ge = (a>=b) -- 1 cycle
//
// Every sub-module shares the same a/b bus, modelling the single shared FPU
// resource described in the spec -- this is why logically-independent
// steps (e.g. c+1 and c-1 in the f(c) evaluation chain) must still be
// serialized rather than computed simultaneously.
//
// NUMBER FORMAT: IEEE-754 half-precision (binary16), so W = 15 for a 16-bit
// word. See addmut.v for the format and the operator implementations.
//
// Subtraction costs one inverter: it is the adder with the second operand's
// sign bit flipped.
//////////////////////////////////////////////////////////////////////////////
module fpu #(parameter W = 15)
(
    input               clk,
    input      [W:0]    a,
    input      [W:0]    b,
    output     [W:0]    add_result,
    output     [W:0]    sub_result,
    output     [W:0]    mul_result,
    output     [W:0]    div2_result,
    output     [W:0]    div_result,
    output     [W:0]    ln_result,
    output              gt,
    output              ge
);
    fpu_add    #(W)     u_add  (.clk(clk), .a(a), .b(b), .sum(add_result));
    fpu_sub    #(W)     u_sub  (.clk(clk), .a(a), .b(b), .diff(sub_result));
    fpu_mul    #(W)     u_mul  (.clk(clk), .a(a), .b(b), .result(mul_result));
    fpu_divby2 #(W)     u_div2 (.clk(clk), .a(a), .result(div2_result));
    fpu_div    #(W)     u_div  (.clk(clk), .a(a), .b(b), .result(div_result));
    fpu_ln     #(.W(W)) u_ln   (.clk(clk), .x_in(a), .f_out(ln_result));
    fpu_compare#(W)     u_cmp  (.clk(clk), .x(a), .y(b), .gt(gt), .ge(ge));
endmodule
