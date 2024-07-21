`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.10.2023 12:38:32
// Design Name: 
// Module Name: delayTest
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module delayedINV #(numBuffers=10)(
input I,
output O
);
 
 (*KEEP = "true"*)   
 wire O0;
 wire [numBuffers-1:0] bufferOut;
 
  LUT1 #(
      .INIT(2'h1)  // Logic function
   )
   LUT1_inst0 (
      .O(O0),   // 1-bit output: LUT
      .I0(I)  // 1-bit input: LUT
   );
   
   
   generate
   genvar i;
   for(i=0;i<numBuffers;i=i+1)
   begin:loop
   if(i==0)
   LUT1 #(
      .INIT(2'h2)  // Logic function
   )
   LUT1_inst1 (
      .O(bufferOut[i]),   // 1-bit output: LUT
      .I0(O0)  // 1-bit input: LUT
   );
   else
   LUT1 #(
   .INIT(2'h2)  // Logic function
   )
   LUT1_inst1 (
      .O(bufferOut[i]),   // 1-bit output: LUT
      .I0(bufferOut[i-1])  // 1-bit input: LUT
   );
   end
   endgenerate
    
assign O =  bufferOut[numBuffers-1];
    
endmodule
