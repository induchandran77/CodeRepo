`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2023 09:49:13 AM
// Design Name: 
// Module Name: comparator
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


module comparator(
input   i_clk,
input   [3:0] i_counter0,
input   [3:0] i_counter1,
output reg key
    );
    
    
reg [3:0] cntr0p1;
reg [3:0] cntr0p2;
reg [3:0] cntr1p1;
reg [3:0] cntr1p2;

    
always @(posedge i_clk)
begin
    cntr0p1 <= i_counter0;
    cntr0p2 <= cntr0p1;
    cntr1p1 <= i_counter1;
    cntr1p2 <= cntr1p1;
end

    
always @(posedge i_clk)
begin
    if(cntr0p2 > cntr1p2)
        key <= 1'b1;
    else
        key <= 1'b0;
end    
    
endmodule
