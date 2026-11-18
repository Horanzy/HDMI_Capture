// RGB -> YUV matrices copied byte-for-byte from the IT68051 EVB
// iTE6805_CSC_Table.h.  Address order is bank1 registers 0x70..0x85.
// matrix_sel: 0=BT.601 limited, 1=BT.601 full,
//             2=BT.709 limited, 3=BT.709 full.
`timescale 1ns/1ps
module csc_matrix_rom (
    input  wire [1:0] matrix_sel,
    input  wire [4:0] byte_index,
    output reg  [7:0] byte_data
);
    always @(*) begin
        byte_data = 8'h00;
        case (matrix_sel)
            2'd0: case (byte_index)
                0:byte_data=8'h00;  1:byte_data=8'h80;  2:byte_data=8'h10;
                3:byte_data=8'hB2;  4:byte_data=8'h04;  5:byte_data=8'h65;
                6:byte_data=8'h02;  7:byte_data=8'hE9;  8:byte_data=8'h00;
                9:byte_data=8'h93; 10:byte_data=8'h3C; 11:byte_data=8'h18;
                12:byte_data=8'h04; 13:byte_data=8'h55; 14:byte_data=8'h3F;
                15:byte_data=8'h49; 16:byte_data=8'h3D; 17:byte_data=8'h9F;
                18:byte_data=8'h3E; 19:byte_data=8'h18; 20:byte_data=8'h04;
                21:byte_data=8'h00;
                default: byte_data=8'h00;
            endcase
            2'd1: case (byte_index)
                0:byte_data=8'h10;  1:byte_data=8'h80;  2:byte_data=8'h10;
                3:byte_data=8'h09;  4:byte_data=8'h04;  5:byte_data=8'h0E;
                6:byte_data=8'h02;  7:byte_data=8'hC9;  8:byte_data=8'h00;
                9:byte_data=8'h0F; 10:byte_data=8'h3D; 11:byte_data=8'h84;
                12:byte_data=8'h03; 13:byte_data=8'h6D; 14:byte_data=8'h3F;
                15:byte_data=8'hAB; 16:byte_data=8'h3D; 17:byte_data=8'hD1;
                18:byte_data=8'h3E; 19:byte_data=8'h84; 20:byte_data=8'h03;
                21:byte_data=8'h00;
                default: byte_data=8'h00;
            endcase
            2'd2: case (byte_index)
                0:byte_data=8'h00;  1:byte_data=8'h80;  2:byte_data=8'h10;
                3:byte_data=8'hB8;  4:byte_data=8'h05;  5:byte_data=8'hB4;
                6:byte_data=8'h01;  7:byte_data=8'h94;  8:byte_data=8'h00;
                9:byte_data=8'h4A; 10:byte_data=8'h3C; 11:byte_data=8'h17;
                12:byte_data=8'h04; 13:byte_data=8'h9F; 14:byte_data=8'h3F;
                15:byte_data=8'hD9; 16:byte_data=8'h3C; 17:byte_data=8'h10;
                18:byte_data=8'h3F; 19:byte_data=8'h17; 20:byte_data=8'h04;
                21:byte_data=8'h00;
                default: byte_data=8'h00;
            endcase
            default: case (byte_index)
                0:byte_data=8'h10;  1:byte_data=8'h80;  2:byte_data=8'h10;
                3:byte_data=8'hEA;  4:byte_data=8'h04;  5:byte_data=8'h77;
                6:byte_data=8'h01;  7:byte_data=8'h7F;  8:byte_data=8'h00;
                9:byte_data=8'hD0; 10:byte_data=8'h3C; 11:byte_data=8'h83;
                12:byte_data=8'h03; 13:byte_data=8'hAD; 14:byte_data=8'h3F;
                15:byte_data=8'h4B; 16:byte_data=8'h3D; 17:byte_data=8'h32;
                18:byte_data=8'h3F; 19:byte_data=8'h83; 20:byte_data=8'h03;
                21:byte_data=8'h00;
                default: byte_data=8'h00;
            endcase
        endcase
    end
endmodule
