`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: root_finder_top
// Top-level integration matching the assignment's pin diagram:
//   Inputs:  omega, Nmax, epsilon, Start, Reset, Clock
//   Outputs: x_hat, error, Ready
//
// Just wires the MAU (the algorithm/control side) to the FPU (the shared
// arithmetic resource) -- all the actual design decisions live in mau.v
// and fpu.v.
//////////////////////////////////////////////////////////////////////////////
module root_finder_top #(parameter W = 15)
(
    input               Clock,
    input               Reset,
    input               Start,
    input      [W:0] omega,
    input      [W:0] epsilon,
    input        [15:0] Nmax,
    output     [W:0] x_hat,
    output              error,
    output              Ready
);
    mau #(W) u_mau (
        .clk(Clock),
        .rst(Reset),
        .start(Start),
        .omega_in(omega),
        .epsilon_in(epsilon),
        .nmax_in(Nmax),
        .x_hat(x_hat),
        .error(error),
        .ready(Ready)
    );
endmodule
