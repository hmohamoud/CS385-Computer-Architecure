module ALU (op,a,b,result,zero);
 input [15:0] a;
 input [15:0] b;
 input [2:0] op;
 output [15:0] result;
 output zero;
 wire c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15;
 ALU1 alu0 (a[0],b[0],op[2],op[1:0],set,op[2],c1,result[0]);
 ALU1 alu1 (a[1],b[1],op[2],op[1:0],0, c1, c2,result[1]);
 ALU1 alu2 (a[2],b[2],op[2],op[1:0],0, c2, c3,result[2]);
 ALU1 alu3 (a[3],b[3],op[2],op[1:0],0, c3, c4,result[3]);
 ALU1 alu4 (a[4],b[4],op[2],op[1:0],0, c4, c5,result[4]);
 ALU1 alu5 (a[5],b[5],op[2],op[1:0],0, c5, c6,result[5]);
 ALU1 alu6 (a[6],b[6],op[2],op[1:0],0, c6, c7,result[6]);
 ALU1 alu7 (a[7],b[7],op[2],op[1:0],0, c7, c8,result[7]);
 ALU1 alu8 (a[8],b[8],op[2],op[1:0],0, c8, c9,result[8]);
 ALU1 alu9  (a[9],b[9],op[2],op[1:0],0, c9, c10,result[9]);
 ALU1 alu10 (a[10],b[10],op[2],op[1:0],0, c10, c11,result[10]);
 ALU1 alu11 (a[11],b[11],op[2],op[1:0],0, c11, c12,result[11]);
 ALU1 alu12 (a[12],b[12],op[2],op[1:0],0, c12, c13,result[12]);
 ALU1 alu13 (a[13],b[13],op[2],op[1:0],0, c13, c14,result[13]);
 ALU1 alu14 (a[14],b[14],op[2],op[1:0],0, c14, c15,result[14]);
 ALUmsb alu15 (a[15],b[15],op[2],op[1:0],0,c15,c16,result[15],set);

 or or1(orall, result[0],result[1],result[2],result[3],result[4],result[5],
 result[6],result[7],result[8],result[9],result[10],result[11],
 result[12],result[13],result[14],result[15]);
 not n1(zero, orall);
endmodule

// 1-bit ALU for bits 0-2
module ALU1 (a,b,binvert,op,less,carryin,carryout,result);
input a,b,less,carryin,binvert;
input [1:0] op;
output carryout,result;
wire sum, a_and_b, a_or_b, b_inv;
not not1(b_inv, b);
mux2x1 mux1(b,b_inv,binvert,b1);
and and1(a_and_b, a, b);
or or1(a_or_b, a, b);
fulladder adder1(sum,carryout,a,b1,carryin);
mux4x1 mux2(a_and_b,a_or_b,sum,less,op[1:0],result);
endmodule


// 1-bit ALU for the most significant bit
module ALUmsb (a,b,binvert,op,less,carryin,carryout,result,sum);
input a,b,less,carryin,binvert;
input [1:0] op;
output carryout,result,sum;
wire sum, a_and_b, a_or_b, b_inv;
not not1(b_inv, b);
mux2x1 mux1(b,b_inv,binvert,b1);
and and1(a_and_b, a, b);
or or1(a_or_b, a, b);
fulladder adder1(sum,carryout,a,b1,carryin);
mux4x1 mux2(a_and_b,a_or_b,sum,less,op[1:0],result);
endmodule

module halfadder (S,C,x,y);
input x,y;
output S,C;
xor (S,x,y);
and (C,x,y);
endmodule

module fulladder (S,C,x,y,z);
   input x,y,z;
    output S,C;
    wire S1,D1,D2;
    halfadder HA1 (S1,D1,x,y),
              HA2 (S,D2,S1,z);
           or g1(C,D2,D1);
endmodule

// Behavioral model of MIPS, 3-stage pipeline for R-types and addi only
module reg_file (rr1,rr2,wr,wd,regwrite,rd1,rd2,clock);
  input [1:0] rr1,rr2,wr;
  input [15:0] wd;
  input regwrite,clock;
  output [15:0] rd1,rd2;
  wire [15:0] q1,q2,q3;
// registers

 registers reg1(wd,c1,q1);
 registers reg2(wd,c2,q2);
 registers reg3(wd,c3,q3);
// output port

    mux4x116BIT mux1 (0,q1,q2,q3,rr1,rd1),
    mux2 (0,q1,q2,q3,rr2,rd2);

// input port

decoder dec(wr[1],wr[0],w3,w2,w1,w0);
and a (regwrite_and_clock,regwrite,clock);
and a1 (c1,regwrite_and_clock,w1),
    a2 (c2,regwrite_and_clock,w2),
    a3 (c3,regwrite_and_clock,w3);
endmodule

module registers(D,CLK,Q);
input [15:0] D;
input CLK;
output [15:0] Q;
 D_flip_flop r1 (D[0],CLK,Q[0]);
 D_flip_flop r2 (D[1],CLK,Q[1]);
 D_flip_flop r3 (D[2],CLK,Q[2]);
 D_flip_flop r4 (D[3],CLK,Q[3]);
 D_flip_flop r5 (D[4],CLK,Q[4]);
 D_flip_flop r6 (D[5],CLK,Q[5]);
 D_flip_flop r7 (D[6],CLK,Q[6]);
 D_flip_flop r8 (D[7],CLK,Q[7]);
 D_flip_flop r9 (D[8],CLK,Q[8]);
 D_flip_flop r10 (D[9],CLK,Q[9]);
 D_flip_flop r11 (D[10],CLK,Q[10]);
 D_flip_flop r12 (D[11],CLK,Q[11]);
 D_flip_flop r13 (D[12],CLK,Q[12]);
 D_flip_flop r14 (D[13],CLK,Q[13]);
 D_flip_flop r15 (D[14],CLK,Q[14]);
 D_flip_flop r16 (D[15],CLK,Q[15]);
endmodule

// Components
module D_flip_flop(D,CLK,Q);
input D,CLK;
output Q;
wire CLK1, Y;
not not1 (CLK1,CLK);
D_latch D1(D,CLK, Y),
D2(Y,CLK1,Q);
endmodule

module D_latch(D,C,Q);
input D,C;
output Q;
wire x,y,D1,Q1;
nand nand1 (x,D, C),
     nand2 (y,D1,C),
     nand3 (Q,x,Q1),
     nand4 (Q1,y,Q);
 not not1 (D1,D);
endmodule

module decoder (S1,S0,D3,D2,D1,D0);
   input S0,S1;
    output D0,D1,D2,D3;

    not n1 (notS0,S0),
        n2 (notS1,S1);
    and a0 (D0,notS1,notS0),
        a1 (D1,notS1, S0),
        a2 (D2, S1,notS0),
         a3 (D3,S1,S0);
endmodule

module mux4x1(i0,i1,i2,i3,select,y);
input i0,i1,i2,i3;
 input [1:0] select;
output y;
wire S0,S1,w1,w2,w3,w4;
  not G1(S0,select[0]),
     G2(S1,select[1]);
 and G3(w1,i0,S1,S0),
     G4(w2,i1,S1,select[0]),
   G5(w3,i2,select[1],S0),
   G6(w4,i3,select[1],select[0]);
or G7(y,w1,w2,w3,w4); 
endmodule

module mux4x116BIT (i0,i1,i2,i3,select,y);
input [15:0] i0,i1,i2,i3;
input [1:0] select;
output [15:0] y;
mux4x1 mux1(0, i1[0], i2[0],i3[0],select[1:0], y[0]),
 mux2 (0, i1[1], i2[1],i3[1],select[1:0], y[1]),
 mux3 (0, i1[2], i2[2],i3[2], select[1:0], y[2]),
 mux4 (0, i1[3], i2[3],i3[3], select[1:0], y[3]),
 mux5 (0, i1[4], i2[4],i3[4], select[1:0], y[4]),
 mux6 (0, i1[5], i2[5],i3[5], select[1:0], y[5]),
 mux7 (0, i1[6], i2[6],i3[6], select[1:0], y[6]),
 mux8 (0, i1[7], i2[7],i3[7], select[1:0], y[7]),
 mux9(0, i1[8], i2[8],i3[8], select[1:0], y[8]),
 mux10(0, i1[9], i2[9],i3[9], select[1:0], y[9]),
  mux11(0, i1[10], i2[10],i3[10], select[1:0], y[10]),
 mux12(0, i1[11], i2[11],i3[11], select[1:0], y[11]),
 mux13(0, i1[12], i2[12], i3[12], select[1:0], y[12]),
 mux14(0, i1[13], i2[13], i3[13], select[1:0], y[13]),
 mux15(0, i1[14], i2[14], i3[14], select[1:0], y[14]),
 mux16(0, i1[15], i2[15], i3[15], select[1:0], y[15]);
endmodule

module MainControl (Op, Control);
input [3:0] Op;
output reg [5:0] Control; 
 always @(Op) case (Op) 

 4'b0000: Control <= 6'b101010; // add 
 4'b0001: Control <= 6'b101110; // sub
  4'b0010: Control <=6'b101000; // and
 4'b0011: Control <= 6'b101001; // or
 4'b0111: Control <= 6'b101111; // slt
 4'b0100: Control <= 6'b011010; // addi 
 endcase

endmodule

 module CPU (clock,PC,IFID_IR,IDEX_IR,WD);
 input clock;
 output [15:0] PC,IFID_IR,IDEX_IR,WD;

/*
 initial begin
 
// Program with nop's - no hazards
  IMemory[0]= 16'b0100_00_01_00001111; // addi $1, $0, 15 ($1=15)
  IMemory[1] = 16'b0100_00_10_00000111; // addi $2, $0,  7 ($2= 7)
  IMemory[2] = 16'b0000_0000_0000_0000; // nop
  IMemory[3] = 16'b0010_01_10_11_000000; // and $3, $1, $2 
   IMemory[4]= 16'b0000_0000_0000_0000; // nop
  IMemory[5] = 16'b0001_01_11_10_000000; // sub  $2, $1, $3 
  IMemory[6] = 16'b0000_0000_0000_0000; // nop
  IMemory[7] = 16'b0011_10_11_10_000000; // or $2, $2, $3 
  IMemory[8]= 16'b0000_0000_0000_0000; // nop
  IMemory[9] = 16'b0000_10_11_11_000000; // add $3, $2, $3 
   IMemory[10] = 16'b0000_0000_0000_0000; // nop
  IMemory[11] = 16'b0111_11_10_01_000000; // slt $1, $3, $2 
  IMemory[12] = 16'b0111_10_11_01_000000; // slt $1, $2, $3 
 end
*/

 initial begin
// Program without nop's - wrong results
 IMemory[0] = 16'b0100_00_01_00001111; // addi $1, $0,  15 ($1=15)
 IMemory[1] = 16'b0100_00_10_00000111; // addi $2, $0, 7 ($2= 7)
 IMemory[2] = 16'b0010_01_10_11_000000; // and  $3, $1, $2 
 IMemory[3] = 16'b0001_01_11_10_000000; // sub  $2, $1, $3 
 IMemory[4] = 16'b0011_10_11_10_000000; // or  $2, $2, $3
 IMemory[5] = 16'b0000_10_11_11_000000; // add $3, $2, $3
 IMemory[6] = 16'b0111_11_10_01_000000; // slt $1, $3, $2 
 IMemory[7] = 16'b0111_10_11_01_000000; // slt $1, $2, $3 
 end

// Pipeline stages
//=== IF STAGE ===
    wire [15:0] NextPC;
    reg[15:0] PC, IMemory[0:1023];
//--------------------------------
    reg[15:0] IFID_IR;
//--------------------------------
    ALU fetch (3'b010,PC,2,NextPC,Unused);
//=== ID STAGE ===
    wire [5:0] Control;
    wire [15:0] RD1,RD2,SignExtend, WD;

//----------------------------------------------------
    reg [15:0] IDEX_IR; // For monitoring the pipeline
    reg IDEX_RegWrite,IDEX_ALUSrc,IDEX_RegDst;
    reg [2:0] IDEX_ALUOp;
    reg [15:0] IDEX_RD1,IDEX_RD2;
    reg [15:0] IDEX_SignExt;
    reg [1:0] IDEX_rt,IDEX_rd;
//----------------------------------------------------
  reg_file rf (IFID_IR[11:10],IFID_IR[9:8],WR,WD,IDEX_RegWrite,RD1,RD2,clock);

MainControl MainCtr (IFID_IR[15:12],Control); //
assign SignExtend = {{8{IFID_IR[7]}},IFID_IR[7:0]};

//=== EXE STAGE ===

  output [2:0]IDEX_ALUctl;
  wire [15:0] B, ALUOut;
  reg [2:0] IDEX_ALUctl;

  wire [1:0] WR;

 wire [2:0] ALUctl;

 ALU ex (IDEX_ALUctl, IDEX_RD1, B, ALUOut, Zero);

 // assign B = (IDEX_ALUSrc) ? IDEX_SignExt: IDEX_RD2;  // ALUSrc Mux
 mux2x16Bit mux1(IDEX_RD2,IDEX_SignExt,IDEX_ALUSrc,B); // ALUSrcMux

 //assign WR = (IDEX_RegDst) ? IDEX_rd: IDEX_rt;  // RegDst Mux
  mux2x1Regmux mux(IDEX_rt,IDEX_rd,IDEX_RegDst,WR); // RegDst Mux
  assign WD = ALUOut;
  initial begin
  PC = 0;
  end
  
// Running the pipeline
  always @(negedge clock) begin
  
// Stage 1 - IF
  PC <= NextPC;
  IFID_IR <= IMemory[PC >> 1];
  
// Stage 2 - ID
 IDEX_IR <= IFID_IR; // For monitoring the pipeline
 {IDEX_RegDst,IDEX_ALUSrc,IDEX_RegWrite,IDEX_ALUctl[2:0]} <= Control; 
   IDEX_RD1 <= RD1;
  IDEX_RD2 <= RD2;
  IDEX_SignExt <= SignExtend;

  IDEX_rt <= IFID_IR[9:8]; //
  IDEX_rd <= IFID_IR[7:6];

// Stage 3 - EX
// No transfers needed here - on negedge WD is written into register WR
 end
endmodule

module mux2x1Regmux(i0,i1,select,out);
 input [1:0] i0,i1;
 input select;
 output [1:0] out;

   mux2x1 multiplexer1(i0[0],i1[0],select,out[0]);
   mux2x1 multiplexer2(i0[1],i1[1],select,out[1]);

 endmodule
 
 module mux2x16Bit(A,B,select,OUT);
 input [15:0] A,B;
 input select;
 output [15:0] OUT;
 mux2x1 mux1 (A[0], B[0], select,OUT[0]);
 mux2x1 mux2 (A[1], B[1], select,OUT[1]);
 mux2x1 mux3 (A[2], B[2], select,OUT[2]);
 mux2x1 mux4 (A[3], B[3], select,OUT[3]);
 mux2x1 mux5 (A[4], B[4], select,OUT[4]);
 mux2x1 mux6 (A[5], B[5], select,OUT[5]);
 mux2x1 mux7 (A[6], B[6], select,OUT[6]);
 mux2x1 mux8 (A[7], B[7], select,OUT[7]);
 mux2x1 mux9 (A[8], B[8], select,OUT[8]);
 mux2x1 mux10(A[9], B[9], select,OUT[9]);
 mux2x1 mux11(A[10],B[10], select,OUT[10]);
 mux2x1 mux12(A[11],B[11], select,OUT[11]);
 mux2x1 mux13(A[12],B[12], select,OUT[12]);
 mux2x1 mux14(A[13],B[13], select,OUT[13]);
 mux2x1 mux15(A[14],B[14], select,OUT[14]);
 mux2x1 mux16(A[15],B[15], select,OUT[15]);
endmodule

module mux2x1(A,B,select,OUT);
   input A,B,select;
    output OUT;
  wire S_inv, wire1, wire2;
  not G1(S_inv, select);
  and G2(wire1, A, S_inv),
 G3(wire2, B, select);
or G4(OUT,wire1,wire2);
endmodule

// Test module
module test ();
 reg clock;
 wire signed [15:0] PC,IFID_IR,IDEX_IR,WD;
 CPU test_cpu(clock,PC,IFID_IR,IDEX_IR,WD);

 always #1 clock = ~clock;

 initial begin
 $display (" PC  IFID_IR  IDEX_IR   WD");
  $monitor ("%3d  %h     %h  %d", PC,IFID_IR,IDEX_IR,WD);
 clock = 1;
 #29 $finish;
 end
endmodule