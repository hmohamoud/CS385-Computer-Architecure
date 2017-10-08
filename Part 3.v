//Main control complete but I am not so sure about the 10 bit binary values

module MainControl (Op, Control);

input [3:0] Op;
output reg [7:0] Control;

  always @(Op) case (Op)
    4'b0000: Control <= 10'b0000101011; // add    
    4'b0001: Control <= 10'b0001101110; // sub
    4'b0010: Control <= 10'b0010101000; // and
    4'b0011: Control <= 10'0011101001; // or
    4'b0111: Control <= 10'b0111101111; // slt
    4'b0100: Control <= 10'b0100100010; // addi
				   
endcase

endmodule

//not completed
module CPU (clock,ALUOut,IR);

  input clock;
  output [15:0] ALUOut,IR;
  reg[15:0] PC;
  reg[15:0] IMemory[0:1023];
  wire [15:0] IR,NextPC,A,B,ALUOut,RD2,SignExtend;
  wire [2:0] ALUctl;
  wire [2:0] ALUOp;
  wire [1:0] WR; 


// Test Program:
  initial begin 
    IMemory[0] = 16'b0100_00_01_00001111;  // addi $1, $0,  15 $1=15)
    IMemory[1] = 16'b0100_00_10_00000111;  // addi $2, $0,  7  ($2=7)
    IMemory[2] = 16'b0010_01_11_111001;  // and  $3, $1, $2    ($3=7)
    IMemory[3] = 16'b0001_01_11_10_001100 // sub  $2, $1, $3  ($2=8)
    IMemory[4] = 16'b0011_10_11_10_001111// or   $2, $2, $3   ($2=15)
    IMemory[5] = 16'b0000_10_11_11_010110// add $3, $2, $3  ($3=22
    IMemory[6] = ;16'b0111_11_10_01_000000// slt $1, $3, $2  ($1= 0)
    IMemory[7] = 16'b0111_10_11_01_000001 // slt  $1, $2, $3  ($1= 1)
  end

  initial PC = 0;

  assign IR = IMemory[PC>>2]; 

  assign WR = (RegDst) ? IR[7:6]: IR[9:8]; // RegDst Mux

  assign B  = (ALUSrc) ? SignExtend: RD2; // ALUSrc Mux 

  assign SignExtend = {{8{IR[7]}},IR[7:0]}; // sign extension unit

  reg_file rf (IR[11:10],IR[9:8],WR,ALUOut,RegWrite,A,RD2,clock);

  alu fetch (3'b010,PC,4,NextPC,Unused);

  alu ex (ALUctl, A, B, ALUOut, Zero);

  MainControl MainCtr (IR[15:12],{RegDst,ALUSrc,RegWrite,ALUOp[2:0]}); 

  ALUControl ALUCtrl(ALUOp, IR[2:0], ALUctl); // ALU control unit

  always @(negedge clock) begin 
    PC <= NextPC;
  end

endmodule

// Test module

module test ();

  reg clock;
  wire [15:0] WD,IR,PC;

  CPU test_cpu(clock,WD,IR,PC);

  always #1 clock = ~clock;
  
  initial begin
    $display ("clock PC IR       WD");
    $monitor ("%b    %2d  %h %h", clock,PC,IR,WD);
    clock = 1;
    #16 $finish;
  end

endmodule