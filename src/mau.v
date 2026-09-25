`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: mau (Master-Algorithm-Unit, Part 2 of the assignment spec)
//
// Implements the bisection root-finding algorithm for f(x) = w*ln(x+1)+(x-1),
// consulting the single shared `fpu` instance for every half-precision
// arithmetic and comparison operation.
//
// Timing convention (worked out, then corrected against simulation):
//   - A state ISSUES an FPU operation by driving a/b.
//   - Each fpu sub-module is independently registered, so its dedicated
//     result wire (add_result/mul_result/div2_result/ln_result/gt/ge)
//     reflects LAST cycle's a/b starting the very next state -- even if
//     THIS state is already driving NEW a/b for a different operation.
//     (fpu.v exposes every sub-module's result on its own wire for
//     exactly this reason -- an earlier version that muxed them down to
//     one shared `result` port broke, because the mux's selection itself
//     depended on the current opcode, so it flipped to the new operation's
//     result the instant the next operation was issued.)
//   - If a result is needed IMMEDIATELY (the very next state), it's read
//     directly off that dedicated wire -- no register needed.
//   - If it must survive an UNRELATED, intervening operation, it's
//     captured into a dedicated register (r3, c_reg).
//   - A register can never be both WRITTEN and combinationally READ for a
//     NEW value in the same cycle -- this is why RECOMPUTE needs 3 cycles
//     (issue add, issue div2, THEN capture the new c) rather than 2: the
//     very next state after the div2 (EVAL_ADD1) already needs c_reg to
//     compute c+1, so the capture can't be deferred onto it.
//
// Corrected loop continue-condition (the spec's literal "or" was a bug --
// see design discussion): continue while (|f(c)| > epsilon) AND (n <= Nmax).
//
// Nmax/n are plain unsigned integers, not floats -- they are loop counters,
// never arithmetic operands, so there is nothing for the FPU to do with them.
//////////////////////////////////////////////////////////////////////////////
module mau #(parameter W = 15)
(
    input               clk,
    input               rst,
    input               start,
    input      [W:0] omega_in,
    input      [W:0] epsilon_in,
    input        [15:0] nmax_in,      // plain integer, not a float
    output reg [W:0] x_hat,
    output reg          error,
    output reg          ready
);
    // ---------------- State encoding ----------------
    // Note: there is no separate "compute c+1" state. The ln module's own
    // polynomial already computes ln(1+x_in) (verified: evaluating its
    // coefficients at x=0.5 gives ~0.4055, matching ln(1.5), not ln(0.5))
    // -- so feeding it c_reg directly already yields ln(c+1). An earlier
    // version fed it c+1 (via an extra adder), which made it compute
    // ln(1+(c+1)) = ln(c+2) -- confirmed as the cause of bad simulation
    // results before this fix.
    localparam S_IDLE       = 4'd0;
    localparam S_INIT       = 4'd1;
    localparam S_EVAL_LN1   = 4'd2;  // issue ln(c) [Store stage] -- gives ln(c+1) internally
    localparam S_EVAL_LN2   = 4'd3;  // ln SOP stage in progress
    localparam S_EVAL_MUL   = 4'd4;  // issue omega*ln(c+1); ln_result holds ln(c+1)
    localparam S_EVAL_ADD2  = 4'd5;  // capture r3=omega*ln(c+1); issue c-1
    localparam S_EVAL_ADD3  = 4'd6;  // issue r3+(c-1); add_result holds c-1
    localparam S_UPDATE1    = 4'd7;  // add_result holds f(c); branch a/b; n++; issue |fc|>eps
    localparam S_UPDATE2    = 4'd8;  // capture fc_accurate; decide continue/exit
    localparam S_RECOMPUTE1 = 4'd9;  // issue a+b
    localparam S_RECOMPUTE2 = 4'd10; // issue (a+b)/2; add_result holds a+b
    localparam S_RECOMPUTE3 = 4'd11; // capture c_reg = div2_result
    localparam S_ERROR_ST   = 4'd12; // capture new c; set error flag
    localparam S_DONE       = 4'd13; // output x_hat, assert ready

    reg [3:0] state, next_state;

    // ---------------- Datapath registers ----------------
    reg [W:0] a_reg, b_reg, c_reg;
    reg [W:0] omega_reg, epsilon_reg;
    reg [15:0] n_reg, nmax_reg;
    reg [W:0] r3;            // only persistent register the f(c) chain needs
    reg       fc_accurate;   // stored |f(c)|<=epsilon result, reused in ERROR_ST

    // Constants, IEEE-754 half-precision
    localparam [W:0] ZERO      = 16'h0000; // E = 0 is the reserved zero encoding
    localparam [W:0] ONE       = 16'h3C00; // 1.0  = 0 01111 0000000000
    localparam [W:0] HALF      = 16'h3800; // 0.5  = 0 01110 0000000000
    localparam [W:0] MINUS_ONE = 16'hBC00; // -1.0 = same, sign bit set

    // ---------------- FPU instance ----------------
    reg  [W:0] fpu_a, fpu_b;
    wire [W:0] add_result, sub_result, mul_result;
    wire [W:0] div2_result, div_result, ln_result;
    wire       fpu_gt, fpu_ge;

    fpu #(W) u_fpu (
        .clk(clk), .a(fpu_a), .b(fpu_b),
        .add_result(add_result), .sub_result(sub_result),
        .mul_result(mul_result), .div2_result(div2_result),
        .div_result(div_result), .ln_result(ln_result),
        .gt(fpu_gt), .ge(fpu_ge)
    );

    // ---------------- State register ----------------
    always @(posedge clk or posedge rst) begin
        if (rst) state <= S_IDLE;
        else     state <= next_state;
    end

    // ---------------- Next-state logic ----------------
    always @* begin
        next_state = state; // default: stay
        case (state)
            S_IDLE:        next_state = start ? S_INIT : S_IDLE;
            S_INIT:        next_state = S_EVAL_LN1;
            S_EVAL_LN1:    next_state = S_EVAL_LN2;
            S_EVAL_LN2:    next_state = S_EVAL_MUL;
            S_EVAL_MUL:    next_state = S_EVAL_ADD2;
            S_EVAL_ADD2:   next_state = S_EVAL_ADD3;
            S_EVAL_ADD3:   next_state = S_UPDATE1;
            S_UPDATE1:     next_state = S_UPDATE2;
            // Exit-condition check, using the LIVE comparator result
            // (fpu_gt), not the fc_accurate register being written this
            // same cycle:
            S_UPDATE2:     next_state = (!fpu_gt || (n_reg >= nmax_reg))
                                         ? S_ERROR_ST : S_RECOMPUTE1;
            S_RECOMPUTE1:  next_state = S_RECOMPUTE2;
            S_RECOMPUTE2:  next_state = S_RECOMPUTE3;
            S_RECOMPUTE3:  next_state = S_EVAL_LN1; // loop back
            S_ERROR_ST:    next_state = S_DONE;
            S_DONE:        next_state = S_IDLE;
            default:       next_state = S_IDLE;
        endcase
    end

    // ---------------- FPU input driving (combinational, per state) -------
    always @* begin
        fpu_a = ZERO; fpu_b = ZERO; // safe defaults
        case (state)
            S_EVAL_LN1: begin
                // ln's polynomial already computes ln(1+x_in), so feeding
                // it c_reg directly yields ln(c+1) -- no separate +1 step.
                fpu_a = c_reg; fpu_b = ZERO;
            end
            S_EVAL_LN2: begin
                // ln's internal x register already captured c+1 during the
                // Store stage (the edge that just entered EVAL_LN2); SOP
                // computes from that latched value only, so whatever we
                // present on fpu_a here is irrelevant -- defaults to ZERO.
            end
            S_EVAL_MUL: begin
                // ln_result currently holds ln(c+1) (valid from EVAL_LN2)
                fpu_a = omega_reg; fpu_b = ln_result;
            end
            S_EVAL_ADD2: begin
                fpu_a = c_reg; fpu_b = MINUS_ONE; // c-1
            end
            S_EVAL_ADD3: begin
                // add_result currently holds c-1 (valid from EVAL_ADD2);
                // r3 (omega*ln(c+1)) was captured at entry to EVAL_ADD2.
                fpu_a = add_result; fpu_b = r3;
            end
            S_UPDATE1: begin
                // add_result currently holds f(c) (valid from EVAL_ADD3).
                // abs() on a float is just clearing the sign bit -- free, no
                // FPU needed. It is NOT a two's-complement negation: the
                // exponent and fraction fields are unsigned magnitudes and
                // must pass through untouched.
                fpu_a = {1'b0, add_result[W-1:0]};
                fpu_b = epsilon_reg;
            end
            S_RECOMPUTE1: begin
                fpu_a = a_reg; fpu_b = b_reg; // a+b
            end
            S_RECOMPUTE2: begin
                // add_result currently holds a+b (valid from RECOMPUTE1)
                fpu_a = add_result; fpu_b = ZERO;
            end
            default: ; // fpu_a/fpu_b stay at safe defaults
        endcase
    end

    // ---------------- Datapath register updates (sequential) -------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            a_reg <= ZERO; b_reg <= ZERO; c_reg <= ZERO;
            omega_reg <= ZERO; epsilon_reg <= ZERO;
            n_reg <= 16'd0; nmax_reg <= 16'd0;
            r3 <= ZERO; fc_accurate <= 1'b0;
            x_hat <= ZERO; error <= 1'b0; ready <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    ready <= 1'b0;
                end

                S_INIT: begin
                    a_reg       <= ZERO;        // a = 0
                    b_reg       <= ONE;         // b = 1
                    c_reg       <= HALF;        // c = (a+b)/2 = 0.5 (known constant)
                    n_reg       <= 16'd1;       // n = 1
                    omega_reg   <= omega_in;
                    epsilon_reg <= epsilon_in;
                    nmax_reg    <= nmax_in;
                    ready       <= 1'b0;
                end

                S_EVAL_ADD2: begin
                    // Capture omega*ln(c+1) BEFORE this state's own c-1
                    // computation overwrites the shared a/b bus.
                    r3 <= mul_result;
                end

                S_UPDATE1: begin
                    // add_result holds f(c) here. Its sign bit is free to
                    // read (no FPU needed) and decides the bracket update;
                    // we move whichever bound is on f(c)'s side to c_reg
                    // (the midpoint just evaluated) -- NOT to f(c) itself.
                    // Neither this branch nor n's increment depends on the
                    // |f(c)|>epsilon comparison also issued this cycle.
                    n_reg <= n_reg + 1;
                    if (add_result[W])
                        a_reg <= c_reg; // f(c) < 0: root is to the right of c
                    else
                        b_reg <= c_reg; // f(c) >= 0: root is to the left of c
                end

                S_UPDATE2: begin
                    // Capture the comparison result issued during UPDATE1,
                    // for later reuse in ERROR_ST (the next_state decision
                    // above already used the live fpu_gt directly).
                    fc_accurate <= !fpu_gt;
                end

                S_RECOMPUTE3: begin
                    // div2_result holds (a+b)/2 here -- the new c. EVAL_ADD1
                    // (the very next state) needs c_reg, so this capture
                    // can't be deferred any further.
                    c_reg <= div2_result;
                end

                S_ERROR_ST: begin
                    error <= fc_accurate ? 1'b0 : 1'b1;
                    x_hat <= c_reg;
                end

                S_DONE: begin
                    ready <= 1'b1;
                end

                default: ;
            endcase
        end
    end

endmodule
