`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2023 09:05:40 AM
// Design Name: 
// Module Name: synchronizer
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


module synchronizer(
input   i_clk,
input   i_signal,
output reg o_signal
    );
    
    
reg tmp;
reg tmp1;

always @(posedge i_clk)
begin
    tmp <= i_signal;
    tmp1 <= tmp;
    o_signal <= tmp1;
end    
    
endmodule
