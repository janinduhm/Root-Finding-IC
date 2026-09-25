`timescale 1ns / 1ps
//=============================================================================
//  IEEE-754 HALF-PRECISION (binary16) ARITHMETIC UNIT
//=============================================================================
//
//  DUPLICATED FILE -- keep in step with:
//      DSD/addmult/addmult.srcs/sources_1/new/addmut.v
//
//  That copy is what the Vivado FPGA demo (top.v, seven.v, debouncer.v)
//  compiles against; this copy is what makes RootFindingIC build standalone.
//  The contents are identical apart from this notice. Edit one, copy to the
//  other -- nothing checks, and a divergence shows up as the two projects
//  disagreeing about arithmetic rather than as a compile error.
//
//  Four combinational modules: float_multi, float_divi, floatadder, norm.
//  No clock anywhere -- every result settles in the same delta cycle as its
//  inputs. The MAU that drives these is what supplies the timing.
//
//  There is no subtractor. Subtraction is floatadder driven with the second
//  operand's sign bit inverted, since a - b = a + (-b) and negating a float is
//  a single bit flip.
//
//  FORMAT
//    bit    15 │ 14 .. 10 │ 9 .. 0
//           S  │    E     │   F
//           1  │    5     │  10
//
//    value = (-1)^S * 1.F * 2^(E - 15)
//
//    The leading 1 is IMPLICIT and never stored -- "1.F" is reconstructed as
//    {1'b1, F} before any arithmetic, giving 11 bits of precision from 10
//    bits of storage. Every module here calls that the "mantissa".
//
//    E is BIASED by 15, so E = k + 15 where k is the true exponent. Bias
//    keeps E unsigned while letting k go negative (k = -1 encodes 0.5), and
//    keeps the encoding monotonic so two positive floats compare correctly
//    as plain unsigned integers.
//
//    E = 0 is RESERVED and means zero (see the cancellation path in
//    floatadder). E = 31 would mean infinity/NaN in the full standard; that
//    is not implemented here.
//
//  EXAMPLES
//    0.25 = 0x3400    1.0 = 0x3C00    1.5 = 0x3E00
//    2.0  = 0x4000    3.0 = 0x4200    2.25 = 0x4080
//
//  KNOWN LIMITATIONS -- deliberate, documented rather than fixed
//    * No denormals. In floatadder the cancellation path computes
//      exp = exp_big - norm_shift, which UNDERFLOWS if a small exponent
//      cancels heavily. In unsigned arithmetic that wraps to a large value,
//      so a very small result silently becomes a very large one. Real
//      IEEE-754 stops normalizing at the minimum exponent and lets the
//      mantissa lose precision instead (a denormal). Values in the intended
//      use (bisection on [0,1]) stay well clear of this.
//    * No infinity or NaN handling.
//    * No rounding. The multiplier truncates the product; the aligner
//      truncates bits shifted out. Round-to-nearest-even is not implemented.
//
//=============================================================================


//-----------------------------------------------------------------------------
//  float_multi -- half-precision multiplier
//-----------------------------------------------------------------------------
//  sign     : XOR of the operand signs
//  exponent : exp1 + exp2 - 15. Both operands carry the bias, so adding them
//             counts it twice and one copy must be removed.
//  mantissa : full 11x11 -> 22-bit product via shift-and-add, then normalized.
//-----------------------------------------------------------------------------
module float_multi(num1, num2, result);
  input  [15:0] num1, num2;
  output [15:0] result;

  // --- decoded input fields -------------------------------------------------
  wire        sign1, sign2;
  wire [4:0]  exp1,  exp2;
  wire [9:0]  frac1, frac2;

  // --- mantissas, hidden bit restored ---------------------------------------
  wire [10:0] mant1, mant2;

  // --- result fields --------------------------------------------------------
  wire        sign_res;    // sign of the product
  wire [5:0]  exp_sum;     // exp1 + exp2 - 15, before normalization
  wire [5:0]  exp_res;     // after the normalization correction
  wire [9:0]  frac_res;    // 10-bit fraction sent to the output

  // --- product datapath -----------------------------------------------------
  reg  [21:0] partial [10:0];  // 11 partial products, one per bit of mant2
  reg  [21:0] prod_acc;        // running total of the partial products
  wire [21:0] prod;            // the finished 22-bit product

  integer j;

  assign {sign1, exp1, frac1} = num1;
  assign {sign2, exp2, frac2} = num2;

  assign mant1 = {1'b1, frac1};
  assign mant2 = {1'b1, frac2};

  assign sign_res = sign1 ^ sign2;
  assign exp_sum  = exp1 + exp2 - 15;   // remove the double-counted bias

  // Shift-and-add: for every set bit j of mant2, add mant1 shifted left by j.
  // mant2[10] is the hidden bit, so the loop runs 0..10 and needs no special
  // case for it. Blocking assignments, and prod_acc cleared each evaluation --
  // without the clear this infers a latch and accumulates across evaluations.
  always @(*) begin
    prod_acc = 0;
    for (j = 0; j < 11; j = j + 1) begin
      partial[j] = mant2[j] ? (mant1 << j) : 22'b0;
      prod_acc   = prod_acc + partial[j];
    end
  end

  assign prod = prod_acc;

  // Two mantissas in [1,2) give a product in [1,4), so the product is at most
  // one bit too wide and needs at most a single right shift. prod is scaled by
  // 2^20, so bit 21 set means the product reached 2.0.
  //   bit21 = 1 -> leading 1 sits at bit 21, fraction is [20:11], exponent +1
  //   bit21 = 0 -> leading 1 sits at bit 20, fraction is [19:10], exponent as-is
  // Both corrections MUST key on the same bit or mantissa and exponent drift
  // apart.
  //
  // The slice always starts one place BELOW the leading 1, because that
  // leading 1 is the hidden bit and the format re-implies it on the way back.
  //
  // The single-bit correction is safe only because both mantissas carry a
  // restored hidden bit and so are >= 1.0, bounding the product to [1,4). If
  // denormal support is ever added, mantissas fall below 1.0, the product can
  // land many bits low, and this mux must be replaced by the norm module.
  assign frac_res = prod[21] ? prod[20:11] : prod[19:10];
  assign exp_res  = prod[21] ? (exp_sum + 6'd1) : exp_sum;

  assign result = {sign_res, exp_res[4:0], frac_res};

endmodule


//-----------------------------------------------------------------------------
//  float_divi -- half-precision divider
//-----------------------------------------------------------------------------
//  sign     : XOR of the operand signs, same as the multiplier
//  exponent : exp1 - exp2 + 15. Note the asymmetry with float_multi: adding two
//             biased exponents counts the bias TWICE and one copy is removed,
//             whereas subtracting them CANCELS it and one copy must be added.
//  mantissa : restoring division, the shift-and-subtract mirror of the
//             multiplier's shift-and-add.
//
//  RESTORING DIVISION
//    Binary long division, exactly as done on paper. At each step: compare the
//    remainder against the divisor, subtract and emit a 1 if it fits, then
//    shift the remainder left one place to move on to the next place value.
//    "Bring down a zero" and "shift the remainder left" are the same act --
//    both multiply the remainder by the base.
//
//    In binary the quotient digit is only ever 0 or 1, so each position needs
//    one comparison and at most one subtraction. The divisor never moves: it
//    is the fixed comparator input, and holding it still means only one
//    register shifts and everything stays integer.
//
//    ORDER MATTERS: the comparison comes BEFORE the shift. The dividend
//    already sits at the correct place value when the loop starts, so shifting
//    first skips a position. With shift-first, 3.0/1.5 yields 4095 instead of
//    2048 -- every bit set, because the remainder never drains.
//
//  SCALING
//    quot = (mant1 / mant2) * 2^11, so the quotient is in (2^10, 2^12) and
//    needs 12 bits. rem needs 12 bits too: after a subtraction rem < mant2, so
//    rem <= 2046, and the following shift takes it to 4092.
//-----------------------------------------------------------------------------
module float_divi(num1, num2, result);
  input  [15:0] num1, num2;
  output [15:0] result;

  // --- decoded input fields -------------------------------------------------
  wire        sign1, sign2;
  wire [4:0]  exp1,  exp2;
  wire [9:0]  frac1, frac2;

  // --- mantissas, hidden bit restored ---------------------------------------
  wire [10:0] mant1, mant2;   // dividend, divisor

  // --- result fields --------------------------------------------------------
  wire        sign_res;    // sign of the quotient
  wire [5:0]  exp_sub;     // exp1 - exp2 + 15, before normalization
  wire [5:0]  exp_res;     // after the normalization correction
  wire [9:0]  frac_res;    // 10-bit fraction sent to the output

  // --- division datapath ----------------------------------------------------
  reg  [11:0] rem;         // running remainder
  reg  [11:0] quot;        // quotient, filled MSB-first

  integer j;

  assign {sign1, exp1, frac1} = num1;
  assign {sign2, exp2, frac2} = num2;

  assign mant1 = {1'b1, frac1};
  assign mant2 = {1'b1, frac2};

  assign sign_res = sign1 ^ sign2;
  assign exp_sub  = exp1 - exp2 + 15;   // restore the cancelled bias

  // One quotient bit per pass, most significant first, so pass j writes
  // quot[11-j]. The remainder starts as the dividend and is shifted at the END
  // of each pass -- see the ORDER MATTERS note above.
  always @(*) begin
    rem = mant1;
    for (j = 0; j < 12; j = j + 1) begin
      if (rem >= mant2) begin
        rem        = rem - mant2;
        quot[11-j] = 1'b1;
      end
      else begin
        quot[11-j] = 1'b0;
      end
      rem = rem << 1;
    end
  end

  // Two mantissas in [1,2) give a quotient in (0.5,2), so the quotient is at
  // most one bit too NARROW and needs at most a single left shift -- the mirror
  // of the multiplier, which can only be one bit too wide. quot is scaled by
  // 2^11, so bit 11 set means the quotient reached 1.0.
  //   bit11 = 1 -> leading 1 at bit 11, fraction is [10:1], exponent as-is
  //   bit11 = 0 -> leading 1 at bit 10, fraction is [9:0],  exponent - 1
  //
  // The slice always starts one place BELOW the leading 1, because that
  // leading 1 is the hidden bit and the format re-implies it.
  //
  // The single-bit correction is safe only because both mantissas carry a
  // restored hidden bit and so are >= 1.0. If denormal support is ever added,
  // mantissas fall below 1.0, the quotient can land many bits low, and this
  // mux must be replaced by the norm module.
  assign frac_res = quot[11] ? quot[10:1] : quot[9:0];
  assign exp_res  = quot[11] ? exp_sub : (exp_sub - 6'd1);

  assign result = {sign_res, exp_res[4:0], frac_res};

  // NOTE: division by the zero encoding (E = 0) is not detected. mant2 gets its
  // hidden bit restored unconditionally, so a zero divisor behaves as 1.0
  // rather than producing infinity.

endmodule



//-----------------------------------------------------------------------------
//  floatadder -- half-precision adder AND subtractor
//-----------------------------------------------------------------------------
//  Subtraction needs no separate module: a - b is a + (-b), and negating a
//  float is inverting bit 15. Drive num2 with its sign bit set and this module
//  subtracts.
//
//  Four stages:
//    1. order   -- pick the larger MAGNITUDE (exponent, then fraction). Sign is
//                  deliberately ignored here; alignment cares about magnitude.
//    2. align   -- shift the smaller mantissa right by the exponent difference
//                  so both sit at the same scale.
//    3. add     -- one 12-bit sum, the extra bit catching the carry-out.
//    4. normalize -- and this is where the two cases diverge, see below.
//
//  WHY mant_sum[11] IS READ TWO DIFFERENT WAYS
//    signs MATCH  : true addition. Bit 11 set means the mantissa reached 2.0,
//                   a genuine overflow -> shift right 1, exponent + 1.
//    signs DIFFER : subtraction, done as two's complement. Bit 11 set is the
//                   2^11 term that two's complement always produces when the
//                   result is non-negative -- it carries no magnitude
//                   information and is DISCARDED. What the result needs
//                   instead is left-normalization, because cancellation can
//                   wipe out many leading bits at once (1.5 - 1.25 leaves
//                   0 0100000000, which needs shifting left by 2).
//
//    Reading bit 11 as overflow in the subtraction case is wrong by a factor
//    of 9 on that example -- it was the original defect here.
//-----------------------------------------------------------------------------
module floatadder(num1, num2, result);
  input  [15:0] num1, num2;
  output [15:0] result;

  // --- magnitude ordering ---------------------------------------------------
  reg  [15:0] operand_big, operand_small;

  // --- decoded fields of the ordered operands -------------------------------
  wire        sign_big, sign_small;
  wire [4:0]  exp_big,  exp_small;
  wire [9:0]  frac_big, frac_small;

  // --- mantissas, hidden bit restored ---------------------------------------
  wire [10:0] mant_big, mant_small;

  // --- alignment ------------------------------------------------------------
  wire [5:0]  exp_delta;           // exp_big - exp_small; 6 bits because two
                                   // 5-bit exponents can differ by up to 31
  reg  [10:0] mant_small_aligned;  // after the right shift
  reg  [10:0] mant_small_signed;   // negated when the signs differ

  // --- sum ------------------------------------------------------------------
  wire [11:0] mant_sum;            // 12 bits: 11 of mantissa plus the carry

  // --- left-normalizer outputs (cancellation path) --------------------------
  wire [10:0] norm_mant;
  wire [3:0]  norm_shift;
  wire        norm_zero;

  // --- result fields --------------------------------------------------------
  reg  [9:0]  frac_res;
  reg  [5:0]  exp_res;             // 6 bits so exp_big + 1 cannot wrap

  assign {sign_big,   exp_big,   frac_big}   = operand_big;
  assign {sign_small, exp_small, frac_small} = operand_small;

  assign mant_big   = {1'b1, frac_big};
  assign mant_small = {1'b1, frac_small};

  assign exp_delta = exp_big - exp_small;
  assign mant_sum  = mant_small_signed + mant_big;

  // The normalizer is instantiated ONCE and always runs. Hardware cannot be
  // created conditionally -- the addition path simply ignores these outputs.
  // Its input is mant_sum[10:0], not mant_sum: bit 11 is the two's-complement
  // carry and is not part of the magnitude.
  norm u_norm (
    .mag       (mant_sum[10:0]),
    .norm_mag  (norm_mant),
    .shift_amt (norm_shift),
    .is_zero   (norm_zero)
  );

  assign result = {sign_big, exp_res[4:0], frac_res};

  // --- stage 4: normalization, branching on whether the signs matched -------
  always @(*) begin
    if (sign_big == sign_small) begin
      // True addition. Bit 11 is a real overflow past 2.0.
      frac_res = mant_sum[11] ? mant_sum[10:1] : mant_sum[9:0];
      exp_res  = mant_sum[11] ? (exp_big + 6'd1) : exp_big;
    end
    else begin
      // Subtraction. Bit 11 discarded; the magnitude needs left-normalizing.
      frac_res = norm_mant[9:0];
      exp_res  = exp_big - norm_shift;   // see UNDERFLOW note in the header

      // Exact cancellation (1.5 - 1.5). E = 0 is the reserved zero encoding,
      // which only became available once the exponent was biased -- before
      // that, 0x0000 meant 1.0 and zero had no representation at all.
      if (norm_zero) begin
        frac_res = 10'b0;
        exp_res  = 6'b0;
      end
    end
  end

  // --- stage 2: align the smaller mantissa ----------------------------------
  // Shifting right by n compensates for pretending the exponent is n larger.
  // Beyond 10 the smaller operand has shifted out entirely, so the default
  // returning zero is correct rather than merely safe.
  always @(*) begin
    case (exp_delta)
      0:  mant_small_aligned = mant_small;
      1:  mant_small_aligned = (mant_small >> 1);
      2:  mant_small_aligned = (mant_small >> 2);
      3:  mant_small_aligned = (mant_small >> 3);
      4:  mant_small_aligned = (mant_small >> 4);
      5:  mant_small_aligned = (mant_small >> 5);
      6:  mant_small_aligned = (mant_small >> 6);
      7:  mant_small_aligned = (mant_small >> 7);
      8:  mant_small_aligned = (mant_small >> 8);
      9:  mant_small_aligned = (mant_small >> 9);
      10: mant_small_aligned = (mant_small >> 10);
      default: mant_small_aligned = 11'b0;
    endcase
  end

  // --- stage 3 prep: negate the smaller mantissa when the signs differ ------
  // Two's complement: invert every bit, add 1. Inverting is subtracting from
  // all-ones, so invert(x) + 1 = (2^11 - 1 - x) + 1 = 2^11 - x. Adding that
  // is what produces the stray 2^11 in mant_sum[11].
  always @(*) begin
    if (sign_big != sign_small)
      mant_small_signed = ~mant_small_aligned + 11'b1;
    else
      mant_small_signed = mant_small_aligned;
  end

  // --- stage 1: order by MAGNITUDE, ignoring sign ---------------------------
  // Exponent first, fraction as tie-break. This works as an unsigned compare
  // precisely because the exponent is biased (see the header note on
  // monotonicity). The result takes sign_big, which is correct: the larger
  // magnitude decides the sign of a mixed-sign sum.
  always @(*) begin
    if (num2[14:10] > num1[14:10]) begin
      operand_big   = num2;
      operand_small = num1;
    end
    else if (num2[14:10] == num1[14:10]) begin
      if (num2[9:0] > num1[9:0]) begin
        operand_big   = num2;
        operand_small = num1;
      end
      else begin
        operand_big   = num1;
        operand_small = num2;
      end
    end
    else begin
      operand_big   = num1;
      operand_small = num2;
    end
  end

endmodule


//-----------------------------------------------------------------------------
//  norm -- left-normalizer (priority encoder + barrel shifter)
//-----------------------------------------------------------------------------
//  Finds the highest set bit of an 11-bit magnitude and shifts it up to bit 10,
//  reporting how far it moved so the caller can subtract that from the
//  exponent. Used by floatadder's cancellation path, where subtracting two
//  near-equal values can destroy many leading bits at once.
//
//  The multiplier does NOT use this: a product is at most one bit too wide and
//  never too narrow, so it needs a single right shift, not a variable search.
//
//  casez treats ? as don't-care and matches top-down, first match wins -- which
//  is exactly a priority encoder. The eleven patterns cover every input with at
//  least one bit set, so default catches exactly one case: all zeros.
//-----------------------------------------------------------------------------
module norm(
  input  wire [10:0] mag,        // magnitude, two's-complement carry already dropped
  output reg  [10:0] norm_mag,   // shifted so the leading 1 sits at bit 10
  output reg  [3:0]  shift_amt,  // distance shifted; caller subtracts from E
  output reg         is_zero     // mag was all zeros -- caller must emit zero
  );

  always @(*) begin
    casez (mag)
      11'b1?????????? : begin norm_mag = mag;       shift_amt = 4'd0;  is_zero = 1'b0; end
      11'b01????????? : begin norm_mag = mag << 1;  shift_amt = 4'd1;  is_zero = 1'b0; end
      11'b001???????? : begin norm_mag = mag << 2;  shift_amt = 4'd2;  is_zero = 1'b0; end
      11'b0001??????? : begin norm_mag = mag << 3;  shift_amt = 4'd3;  is_zero = 1'b0; end
      11'b00001?????? : begin norm_mag = mag << 4;  shift_amt = 4'd4;  is_zero = 1'b0; end
      11'b000001????? : begin norm_mag = mag << 5;  shift_amt = 4'd5;  is_zero = 1'b0; end
      11'b0000001???? : begin norm_mag = mag << 6;  shift_amt = 4'd6;  is_zero = 1'b0; end
      11'b00000001??? : begin norm_mag = mag << 7;  shift_amt = 4'd7;  is_zero = 1'b0; end
      11'b000000001?? : begin norm_mag = mag << 8;  shift_amt = 4'd8;  is_zero = 1'b0; end
      11'b0000000001? : begin norm_mag = mag << 9;  shift_amt = 4'd9;  is_zero = 1'b0; end
      11'b00000000001 : begin norm_mag = mag << 10; shift_amt = 4'd10; is_zero = 1'b0; end
      default         : begin norm_mag = 11'b0;     shift_amt = 4'd0;  is_zero = 1'b1; end
    endcase
  end

endmodule
