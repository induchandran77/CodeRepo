`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.09.2023 17:16:55
// Design Name: 
// Module Name: ro_puf
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

//puf circuit
module ro_puf (
  input Enable,
  input [2:0] sel,
  input clear,

  output reg puf_out,
  output wire [7:0] C1,
  output wire ro_clk,
  input clk_100
  
);
  wire R1;
  wire R2;
  wire R3;
  wire R4;
  wire R5;
  wire R6;
  wire R7;
  wire R8;
  wire of;

  always @(posedge clk_100)
  begin
    if(clear)
        puf_out <= 0;
    else
        puf_out <= of;
  end
  

  ring_osc ro_inst1 (
  .in1(Enable),
  .inv_out(R1)
  );
  
  ring_osc ro_inst2 (
  .in1(Enable),
  .inv_out(R2)
  );
  ring_osc ro_inst3 (
  .in1(Enable),
  .inv_out(R3)
  );
  ring_osc ro_inst4 (
  .in1(Enable),
  .inv_out(R4)
  );
  ring_osc ro_inst5 (
  .in1(Enable),
  .inv_out(R5)
  );
  ring_osc ro_inst6 (
  .in1(Enable),
  .inv_out(R6)
  );
  ring_osc ro_inst7 (
  .in1(Enable),
  .inv_out(R7)
  );
  ring_osc ro_inst8 (
  .in1(Enable),
  .inv_out(R8)
  );
 
  Mux_8x1 Mux_inst1 (
    .sel( sel ),
    .m0( R1 ),
    .m1( R2 ),
    .m2( R3 ),
    .m3( R4 ),
    .m4( R5 ),
    .m5( R6 ),
    .m6( R7 ),
    .m7( R8 ),
    .m_out( ro_clk )
  );

 

  // counter1
  Counter_1 Counter_1_inst1 (
    .clk1( ro_clk ),
    .clr( clear ),
    .out( C1 ),
    .of(of)
  );

endmodule
