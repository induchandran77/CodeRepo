`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.09.2023 17:22:39
// Design Name: 
// Module Name: inverter
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



module inverter(
input I,
output O

    );
    
    
   LUT1_L #(
      .INIT(2'b01)  // Specify LUT Contents
   ) LUT1_L_inst (
      .LO(O), // LUT local output
      .I0(I)  // LUT input
   );    
   
   
   endmodule
