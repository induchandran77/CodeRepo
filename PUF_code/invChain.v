`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.09.2023 17:18:44
// Design Name: 
// Module Name: invChain
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


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09.05.2023 12:23:57
// Design Name: 
// Module Name: invChain
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


module ring_osc(
input in1,
output inv_out
    );
    
wire O1I2,O2I3,O3I4,O4I5;

assign I1_In = in1 & inv_out;    

delayedINV I1(
.I(I1_In),
.O(O1I2)
);
    
    
delayedINV I2(
.I(O1I2),
.O(O2I3)
);
    
delayedINV I3(
.I(O2I3),
.O(O3I4)
);

delayedINV I4(
.I(O3I4),
.O(O4I5)
);

delayedINV I5(
.I(O4I5),
.O(inv_out)
);
    
    
endmodule
