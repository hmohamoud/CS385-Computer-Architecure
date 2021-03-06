// Simplified version of MIPS register file (4 registers, 1-bit data)

// For the project MIPS (4-registers, 16-bit data):
//  1. Change the D flip-flops with 16-bit registers
//  2. Redesign mux4x1 using gate-level modeling

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


module reg_file (rr1,rr2,wr,wd,regwrite,rd1,rd2,clock);

   input [1:0] rr1,rr2,wr;
   input [15:0] wd;
   input regwrite,clock;
   output [15:0] rd1,rd2;
   wire [15:0] q1,q2,q3;
// registers

  registers reg1(wd,clock,q1)
  registers reg2(wd,clock,q2);
  registers reg3(wd,clock,q3);

// output port

   mux4x1 mux1 (0,q1,q2,q3,rr1,rd1),
          mux2 (0,q1,q2,q3,rr2,rd2);

// input port

   decoder dec(wr[1],wr[0],w3,w2,w1,w0);

   and a (regwrite_and_clock,regwrite,clock);

   and a1 (c1,regwrite_and_clock,w1),
       a2 (c2,regwrite_and_clock,w2),
       a3 (c3,regwrite_and_clock,w3);

endmodule


// Components

module D_flip_flop(D,CLK,Q);
   input D,CLK; 
   output Q; 
   wire CLK1, Y;
   not  not1 (CLK1,CLK);
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
   not  not1  (D1,D);
endmodule 

module 4x1_16bit (i0,i1,i2,i3,select,y);
input [15:0] i0,i1,i2,i3;
input [1:0] select;
output [15:0] y;

4x1_16bit mux1(0, i1[0], i2[0],i3[0],select[1:0], y[0]);
          mux2 (0, i1[1], i2[1],i3[1],select[1:0], y[1]),
          mux3 (0, i1[2], i2[2],i3[2], select[1:0], y[2]),
          mux4  (0, i1[3], i2[3],i3[3], select[1:0], y[3]),

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
   or  G7(y,w1,w2,w3,w4);	   
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


module testing ();

 reg [1:0] rr1,rr2,wr;
 reg wd;
 reg regwrite, clock;
 wire rd1,rd2;

 reg_file regs (rr1,rr2,wr,wd,regwrite,rd1,rd2,clock);

 initial 
   begin  

     #10 regwrite=1; //enable writing

     #10 wd=0;       // set write data

     #10      rr1=0;rr2=0;clock=0;
     #10 wr=1;rr1=1;rr2=1;clock=1;
     #10                  clock=0;
     #10 wr=2;rr1=2;rr2=2;clock=1;
     #10                  clock=0;
     #10 wr=3;rr1=3;rr2=3;clock=1;
     #10                  clock=0;

     #10 regwrite=0; //disable writing

     #10 wd=1;       // set write data

     #10 wr=1;rr1=1;rr2=1;clock=1;
     #10                  clock=0;
     #10 wr=2;rr1=2;rr2=2;clock=1;
     #10                  clock=0;
     #10 wr=3;rr1=3;rr2=3;clock=1;
     #10                  clock=0;

     #10 regwrite=1; //enable writing

     #10 wd=1;       // set write data

     #10 wr=1;rr1=1;rr2=1;clock=1;
     #10                  clock=0;
     #10 wr=2;rr1=2;rr2=2;clock=1;
     #10                  clock=0;
     #10 wr=3;rr1=3;rr2=3;clock=1;
     #10                  clock=0;

   end 

 initial
   $monitor ("regwrite=%d clock=%d rr1=%d rr2=%d wr=%d wd=%d rd1=%d rd2=%d",regwrite,clock,rr1,rr2,wr,wd,rd1,rd2);
 
endmodule 


/* Test results

C:\Verilog>iverilog -o t regfile.vl

C:\Verilog>vvp t
regwrite=x clock=x rr1=x rr2=x wr=x wd=x rd1=x rd2=x
regwrite=1 clock=x rr1=x rr2=x wr=x wd=x rd1=x rd2=x
regwrite=1 clock=x rr1=x rr2=x wr=x wd=0 rd1=x rd2=x
regwrite=1 clock=0 rr1=0 rr2=0 wr=x wd=0 rd1=0 rd2=0
regwrite=1 clock=1 rr1=1 rr2=1 wr=1 wd=0 rd1=x rd2=x
regwrite=1 clock=0 rr1=1 rr2=1 wr=1 wd=0 rd1=0 rd2=0
regwrite=1 clock=1 rr1=2 rr2=2 wr=2 wd=0 rd1=x rd2=x
regwrite=1 clock=0 rr1=2 rr2=2 wr=2 wd=0 rd1=0 rd2=0
regwrite=1 clock=1 rr1=3 rr2=3 wr=3 wd=0 rd1=x rd2=x
regwrite=1 clock=0 rr1=3 rr2=3 wr=3 wd=0 rd1=0 rd2=0
regwrite=0 clock=0 rr1=3 rr2=3 wr=3 wd=0 rd1=0 rd2=0
regwrite=0 clock=0 rr1=3 rr2=3 wr=3 wd=1 rd1=0 rd2=0
regwrite=0 clock=1 rr1=1 rr2=1 wr=1 wd=1 rd1=0 rd2=0
regwrite=0 clock=0 rr1=1 rr2=1 wr=1 wd=1 rd1=0 rd2=0
regwrite=0 clock=1 rr1=2 rr2=2 wr=2 wd=1 rd1=0 rd2=0
regwrite=0 clock=0 rr1=2 rr2=2 wr=2 wd=1 rd1=0 rd2=0
regwrite=0 clock=1 rr1=3 rr2=3 wr=3 wd=1 rd1=0 rd2=0
regwrite=0 clock=0 rr1=3 rr2=3 wr=3 wd=1 rd1=0 rd2=0
regwrite=1 clock=0 rr1=3 rr2=3 wr=3 wd=1 rd1=0 rd2=0
regwrite=1 clock=1 rr1=1 rr2=1 wr=1 wd=1 rd1=0 rd2=0
regwrite=1 clock=0 rr1=1 rr2=1 wr=1 wd=1 rd1=1 rd2=1
regwrite=1 clock=1 rr1=2 rr2=2 wr=2 wd=1 rd1=0 rd2=0
regwrite=1 clock=0 rr1=2 rr2=2 wr=2 wd=1 rd1=1 rd2=1
regwrite=1 clock=1 rr1=3 rr2=3 wr=3 wd=1 rd1=0 rd2=0
regwrite=1 clock=0 rr1=3 rr2=3 wr=3 wd=1 rd1=1 rd2=1

*/


