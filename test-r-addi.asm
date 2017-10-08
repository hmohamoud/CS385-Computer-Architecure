# Test program for mips-r-type+addi.vl

  .text

  .globl __start
__start:

  IMemory[0] = 16'b0100 00 01 00001111;  //addi $t1, $0,  15   # $t1=15
  IMemory[1] = 16'b0100 00 10 00000111;  //addi $t2, $0,  7    # $t2= 7
  IMemory[2] = 16'b0010 01 10 11 000000;  //and  $t3, $t1, $t2  # $t3= 7
  IMemory[3] = 16'b0001 01 11 10 000000;  //sub  $t2, $t1, $t3  # $t2= 8
  IMemory[4] = 16'b0011 10 11 10 000000;  //or   $t2, $t2, $t3  # $t2=15
  IMemory[5] = 16'b0000 10 11 11 000000;  //add  $t3, $t2, $t3  # $t3=22
  IMemory[6] = 16'b0111 11 10 01 000000;  //slt  $t1, $t3, $t2  # $t1= 0
  IMemory[7] = 16'b0111 10 11 01 000000;  //slt  $t1, $t2, $t3  # $t1= 1
