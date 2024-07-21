//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.05.2023 14:24:17
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
module Mux_8x1(
    input [2:0] sel,
    input m0,
    input m1,
    input m2,
    input m3,
    input m4,
    input m5,
    input m6,
    input m7,   
    output reg m_out
);
    always @ (*) begin
        case (sel)
            3'b000: m_out = m0;
            3'b001: m_out = m1;
            3'b010: m_out = m2;
            3'b011: m_out = m3;
            3'b100: m_out = m4;
            3'b101: m_out = m5;
            3'b110: m_out = m6;
            3'b111: m_out = m7;
            
            default:
                m_out = m0;
        endcase
    end
endmodule