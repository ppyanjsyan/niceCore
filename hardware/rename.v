/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Email      : wuhongbin2014@163.com
 *  Create Date: 2021.12.24
 *  Description: the register renaming stage of rv32Core
 */
`include "configure.h"

module RENAMEING
(
    input                   i_clk,
    input                   i_resetn,
    
    input  [4:0]            rs1_1,       //the original register
    input  [4:0]            rs2_1,
    input  [4:0]            rs1_2,
    input  [4:0]            rs2_2,
    
    output [31:0]           rs1_data1,   //register data
    output [31:0]           rs2_data1,
    output [31:0]           rs1_data2,
    output [31:0]           rs2_data2,
    output                  rs1_busy1,   //the busy field of register rename table
    output                  rs2_busy1,
    output                  rs1_busy2,
    output                  rs2_busy2,
    output [`RRF_ADDRW-1:0] rs1_rrftag1, //the RRFTag field of register rename table
    output [`RRF_ADDRW-1:0] rs2_rrftag1,
    output [`RRF_ADDRW-1:0] rs1_rrftag2,
    output [`RRF_ADDRW-1:0] rs2_rrftag2,
    
    input                   we1,    //commit enalbe
    input                   we2,
    input  [4:0]            wreg1,  //commit register
    input  [4:0]            wreg2,
    input  [31:0]           wdata1, //commit register data
    input  [31:0]           wdata2,
    input  [`RRF_ADDRW-1:0] wtag1,  //commit rrftag
    input  [`RRF_ADDRW-1:0] wtag2,
    
    input  [4:0]            tagbusy1_addr,
    input  [4:0]            tagbusy2_addr,
    input                   tagbusy1_we,
    input                   tagbusy2_we,
    input  [`RRF_ADDRW-1:0] rd1_renamedtag,
    input  [`RRF_ADDRW-1:0] rd2_renamedtag,
    input  [4:0]            tagbusy1_spectag,
    input  [4:0]            tagbusy2_spectag,

    input                   predict_miss,
    input                   predict_hit,
    input  [4:0]            predict_tag,
    input  [4:0]            mpft_valid1,
    input  [4:0]            mpft_valid2
);
    
    ARF afr
    (
        .i_clk    (i_clk),
        .raddr1 (rs1_1),
        .raddr2 (rs2_1),
        .raddr3 (rs1_2),
        .raddr4 (rs2_2),
        .rdata1 (rs1_data1),
        .rdata2 (rs2_data1),
        .rdata3 (rs1_data2),
        .rdata4 (rs2_data2),
        .we1    (we1),
        .we2    (we2),
        .waddr1 (wreg1),
        .waddr2 (wreg2),
        .wdata1 (wdata1),
        .wdata2 (wdata2)
    );
 
    Rename_Table renametable(
        .i_clk              (i_clk),
        .i_resetn           (i_resetn),
        .rs1_1            (rs1_1),
        .rs2_1            (rs2_1),
        .rs1_2            (rs1_2),
        .rs2_2            (rs2_2),
        .com_reg1         (wreg1),
        .com_reg2         (wreg2),
        .rs1_rrftag1         (rs1_rrftag1),
        .rs2_rrftag1         (rs2_rrftag1),
        .rs1_rrftag2         (rs1_rrftag2),
        .rs2_rrftag2         (rs2_rrftag2),
        .rs1_busy1        (rs1_busy1),
        .rs2_busy1        (rs2_busy1),
        .rs1_busy2        (rs1_busy2),
        .rs2_busy2        (rs2_busy2),
        .rd1 (tagbusy1_addr),
        .rd2 (tagbusy2_addr),
        .rd1_valid      (tagbusy1_we),
        .rd2_valid      (tagbusy2_we),
        .rd1_renamedtag          (rd1_renamedtag),
        .rd2_renamedtag          (rd2_renamedtag),
        .inst1_spectag (tagbusy1_spectag),
        .inst2_spectag (tagbusy2_spectag),
        .com_en1          (we1),
        .com_en2          (we2),
        .com_tag1         (wtag1),
        .com_tag2         (wtag2),
        .predict_miss     (predict_miss),
        .predict_hit      (predict_hit),
        .predict_tag      (predict_tag),
        .mpft_valid1      (mpft_valid1),
        .mpft_valid2      (mpft_valid2)
    );

endmodule //RENAMEING


module ARF //Architected Register File
(
    input         i_clk,
    
    input  [4:0]  raddr1,  //4-read port
    input  [4:0]  raddr2,
    input  [4:0]  raddr3,
    input  [4:0]  raddr4,
    output [31:0] rdata1,
    output [31:0] rdata2,
    output [31:0] rdata3,
    output [31:0] rdata4,
    
    input         we1,     //2-write port
    input         we2,
    input  [4:0]  waddr1,
    input  [4:0]  waddr2,
    input  [31:0] wdata1,
    input  [31:0] wdata2
);
    reg    [31:0] arfmem [0:31];
    
`ifdef ARF_INIT_ZERO
    integer i;
    initial begin
        for (i = 0; i < 32; i = i+1) begin
            arfmem[i] = 0;
        end
    end
`endif

    assign rdata1 = raddr1 ? arfmem[raddr1] : 32'b0;
    assign rdata2 = raddr2 ? arfmem[raddr2] : 32'b0;
    assign rdata3 = raddr3 ? arfmem[raddr3] : 32'b0;
    assign rdata4 = raddr4 ? arfmem[raddr4] : 32'b0;
    
    always @(posedge i_clk) begin
        if (we1) begin
            arfmem[waddr1] <= wdata1;
        end
        if (we2) begin
            arfmem[waddr2] <= wdata2;
        end
    end
endmodule //ARF

module Rename_Table //Register Renaming Table
(
    input                   i_clk,
    input                   i_resetn,
    
    input  [4:0]            rs1_1,
    input  [4:0]            rs2_1,
    input  [4:0]            rs1_2,
    input  [4:0]            rs2_2,
    output                  rs1_busy1,
    output                  rs2_busy1,
    output                  rs1_busy2,
    output                  rs2_busy2,  
    output [`RRF_ADDRW-1:0] rs1_rrftag1,
    output [`RRF_ADDRW-1:0] rs2_rrftag1,
    output [`RRF_ADDRW-1:0] rs1_rrftag2,
    output [`RRF_ADDRW-1:0] rs2_rrftag2,

    input                   com_en1,   //commit enalbe
    input                   com_en2,
    input  [4:0]            com_reg1,  //commit register
    input  [4:0]            com_reg2,
    input  [`RRF_ADDRW-1:0] com_tag1,  //commit rrftag
    input  [`RRF_ADDRW-1:0] com_tag2, 

    input                   rd1_valid,
    input                   rd2_valid,
    input  [4:0]            rd1,
    input  [4:0]            rd2,
    input  [`RRF_ADDRW-1:0] rd1_renamedtag,
    input  [`RRF_ADDRW-1:0] rd2_renamedtag,
    input  [4:0]            inst1_spectag, //speculative tag
    input  [4:0]            inst2_spectag,

    input                   predict_miss,
    input                   predict_hit,
    input  [4:0]            predict_tag,
    input  [4:0]            mpft_valid1,
    input  [4:0]            mpft_valid2
);
    //5 renaming table, busy: 1-bit, rrftag: 6-bit
    reg  [31:0] busy0;
    reg  [31:0] tag0_0;
    reg  [31:0] tag0_1;
    reg  [31:0] tag0_2;
    reg  [31:0] tag0_3;
    reg  [31:0] tag0_4;
    reg  [31:0] tag0_5;
    
    reg  [31:0] busy1;
    reg  [31:0] tag1_0;
    reg  [31:0] tag1_1;
    reg  [31:0] tag1_2;
    reg  [31:0] tag1_3;
    reg  [31:0] tag1_4;
    reg  [31:0] tag1_5;
    
    reg  [31:0] busy2;
    reg  [31:0] tag2_0;
    reg  [31:0] tag2_1;
    reg  [31:0] tag2_2;
    reg  [31:0] tag2_3;
    reg  [31:0] tag2_4;
    reg  [31:0] tag2_5;
    
    reg  [31:0] busy3;
    reg  [31:0] tag3_0;
    reg  [31:0] tag3_1;
    reg  [31:0] tag3_2;
    reg  [31:0] tag3_3;
    reg  [31:0] tag3_4;
    reg  [31:0] tag3_5;
    
    reg  [31:0] busy4;
    reg  [31:0] tag4_0;
    reg  [31:0] tag4_1;
    reg  [31:0] tag4_2;
    reg  [31:0] tag4_3;
    reg  [31:0] tag4_4;
    reg  [31:0] tag4_5;
    
    //the newest renaming table
    reg  [31:0] busy_master;
    reg  [31:0] tag_master_0;
    reg  [31:0] tag_master_1;
    reg  [31:0] tag_master_2;
    reg  [31:0] tag_master_3;
    reg  [31:0] tag_master_4;
    reg  [31:0] tag_master_5;
    
    wire rd1_valid_arb   = rd1_valid && rd2_valid && (rd1 == rd2) ? 1'b0 : rd1_valid;

    //wire com_en1_arb     = com_en1 && ~((rd1_valid && (rd1 == com_reg1)) || (rd2_valid && (rd2 == com_reg1)));
    //wire com_en2_arb     = com_en2 && ~((rd1_valid && (rd1 == com_reg2)) || (rd2_valid && (rd2 == com_reg2)));

    wire rd1_busy_master = rd1_valid_arb;
    wire rd2_busy_master = rd2_valid;
    wire com_en1_master  = com_en1 && (com_tag1 == {tag_master_5[com_reg1], tag_master_4[com_reg1], tag_master_3[com_reg1], 
                                                    tag_master_2[com_reg1], tag_master_1[com_reg1], tag_master_0[com_reg1]}) &&
                           ~((rd1_busy_master && (rd1 == com_reg1)) || (rd2_busy_master && (rd2 == com_reg1)));
    wire com_en2_master  = com_en2 && (com_tag2 == {tag_master_5[com_reg2], tag_master_4[com_reg2], tag_master_3[com_reg2], 
                                                    tag_master_2[com_reg2], tag_master_1[com_reg2], tag_master_0[com_reg2]}) &&
                           ~((rd1_busy_master && (rd1 == com_reg2)) || (rd2_busy_master && (rd2 == com_reg2)));
    
    wire rd1_busy_0      = rd1_valid_arb && ~mpft_valid1[0];
    wire rd2_busy_0      = rd2_valid     && ~mpft_valid2[0];
    wire com_en1_0       = com_en1 && (com_tag1 == {tag0_5[com_reg1], tag0_4[com_reg1], tag0_3[com_reg1],
                                                    tag0_2[com_reg1], tag0_1[com_reg1], tag0_0[com_reg1]}) &&
                           ~((rd1_busy_0 && (rd1 == com_reg1)) || (rd2_busy_0 && (rd2 == com_reg1)));
    wire com_en2_0       = com_en2 && (com_tag2 == {tag0_5[com_reg2], tag0_4[com_reg2], tag0_3[com_reg2],
                                                    tag0_2[com_reg2], tag0_1[com_reg2], tag0_0[com_reg2]}) &&
                           ~((rd1_busy_0 && (rd1 == com_reg2)) || (rd2_busy_0 && (rd2 == com_reg2)));

    wire rd1_busy_1      = rd1_valid_arb && ~mpft_valid1[1];
    wire rd2_busy_1      = rd2_valid && ~mpft_valid2[1];
    wire com_en1_1       = com_en1 && (com_tag1 == {tag1_5[com_reg1], tag1_4[com_reg1], tag1_3[com_reg1],
                                                    tag1_2[com_reg1], tag1_1[com_reg1], tag0_1[com_reg1]}) &&
                           ~((rd1_busy_1 && (rd1 == com_reg1)) || (rd2_busy_1 && (rd2 == com_reg1)));
    wire com_en2_1       = com_en2 && (com_tag2 == {tag1_5[com_reg2], tag1_4[com_reg2], tag1_3[com_reg2],
                                                    tag1_2[com_reg2], tag1_1[com_reg2], tag0_1[com_reg2]}) &&
                           ~((rd1_busy_1 && (rd1 == com_reg2)) || (rd2_busy_1 && (rd2 == com_reg2)));

    wire rd1_busy_2      = rd1_valid_arb && ~mpft_valid1[2]; 
    wire rd2_busy_2      = rd2_valid && ~mpft_valid2[2];
    wire com_en1_2       = com_en1 && (com_tag1 == {tag5_2[com_reg1], tag4_2[com_reg1], tag3_2[com_reg1],
                                                    tag2_2[com_reg1], tag1_2[com_reg1], tag0_2[com_reg1]}) &&
                           ~((rd1_busy_2 && (rd1 == com_reg1)) || (rd2_busy_2 && (rd2 == com_reg1)));
    wire com_en2_2       = com_en2 && (com_tag2 == {tag5_2[com_reg2], tag4_2[com_reg2], tag3_2[com_reg2],
                                                    tag2_2[com_reg2], tag1_2[com_reg2], tag0_2[com_reg2]}) &&
                           ~((rd1_busy_2 && (rd1 == com_reg2)) || (rd2_busy_2 && (rd2 == com_reg2)));

    wire rd1_busy_3      = rd1_valid_arb && ~mpft_valid1[3];  
    wire rd2_busy_3      = rd2_valid && ~mpft_valid2[3];
    wire com_en1_3       = com_en1 && (com_tag1 == {tag5_3[com_reg1], tag4_3[com_reg1], tag3_3[com_reg1],
                                                    tag2_3[com_reg1], tag1_3[com_reg1], tag0_3[com_reg1]}) &&
                           ~((rd1_busy_3 && (rd1 == com_reg1)) || (rd2_busy_3 && (rd2 == com_reg1)));
    wire com_en2_3       = com_en2 && (com_tag2 == {tag5_3[com_reg2], tag4_3[com_reg2], tag3_3[com_reg2],
                                                    tag2_3[com_reg2], tag1_3[com_reg2], tag0_3[com_reg2]}) &&
                           ~((rd1_busy_3 && (rd1 == com_reg2)) || (rd2_busy_3 && (rd2 == com_reg2)));

    wire rd1_busy_4      = rd1_valid_arb && ~mpft_valid1[4];
    wire rd2_busy_4      = rd2_valid && ~mpft_valid2[4];
    wire com_en1_4       = com_en1 && (com_tag1 == {tag5_4[com_reg1], tag4_4[com_reg1], tag3_4[com_reg1],
                                                    tag2_4[com_reg1], tag1_4[com_reg1], tag0_4[com_reg1]}) &&
                           ~((rd1_busy_4 && (rd1 == com_reg1)) || (rd2_busy_4 && (rd2 == com_reg1)));
    wire com_en2_4       = com_en2 && (com_tag2 == {tag5_4[com_reg2], tag4_4[com_reg2], tag3_4[com_reg2],
                                                    tag2_4[com_reg2], tag1_4[com_reg2], tag0_4[com_reg2]}) &&
                           ~((rd1_busy_4 && (rd1 == com_reg2)) || (rd2_busy_4 && (rd2 == com_reg2)));

    wire [31:0] next_busy_master = ( busy_master & ((com_en1_master) ? ~(32'b1 << com_reg1) : ~(32'b0)) &
                                   ((com_en2_master) ? ~(32'b1 << com_reg2) : ~(32'b0)) );
    wire [31:0] next_busy0       = ( busy0       & ((com_en1_0)      ? ~(32'b1 << com_reg1) : ~(32'b0)) &
                                   ((com_en2_0)      ? ~(32'b1 << com_reg2) : ~(32'b0)) );
    wire [31:0] next_busy1       = ( busy1       & ((com_en1_1)      ? ~(32'b1 << com_reg1) : ~(32'b0)) &
                                   ((com_en2_1)      ? ~(32'b1 << com_reg2) : ~(32'b0)) );
    wire [31:0] next_busy2       = ( busy2       & ((com_en1_2)      ? ~(32'b1 << com_reg1) : ~(32'b0)) &
                                   ((com_en2_2)      ? ~(32'b1 << com_reg2) : ~(32'b0)) );
    wire [31:0] next_busy3       = ( busy3       & ((com_en1_3)      ? ~(32'b1 << com_reg1) : ~(32'b0)) &
                                   ((com_en2_3)      ? ~(32'b1 << com_reg2) : ~(32'b0)) );
    wire [31:0] next_busy4       = ( busy4       & ((com_en1_4)      ? ~(32'b1 << com_reg1) : ~(32'b0)) &
                                   ((com_en2_4)      ? ~(32'b1 << com_reg2) : ~(32'b0)) );

    assign rs1_busy1    = busy_master[rs1_1];
    assign rs2_busy1    = busy_master[rs2_1];
    assign rs1_busy2    = busy_master[rs1_2];
    assign rs2_busy2    = busy_master[rs2_2];

    assign rs1_rrftag1  = {tag_master_5[rs1_1], tag_master_4[rs1_1], tag_master_3[rs1_1], tag_master_2[rs1_1], tag_master_1[rs1_1], tag_master_0[rs1_1]};
    assign rs2_rrftag1  = {tag_master_5[rs2_1], tag_master_4[rs2_1], tag_master_3[rs2_1], tag_master_2[rs2_1], tag_master_1[rs2_1], tag_master_0[rs2_1]};
    assign rs1_rrftag2  = {tag_master_5[rs1_2], tag_master_4[rs1_2], tag_master_3[rs1_2], tag_master_2[rs1_2], tag_master_1[rs1_2], tag_master_0[rs1_2]};
    assign rs2_rrftag2  = {tag_master_5[rs2_2], tag_master_4[rs2_2], tag_master_3[rs2_2], tag_master_2[rs2_2], tag_master_1[rs2_2], tag_master_0[rs2_2]};
    
    always @(posedge i_clk) begin
        if (~i_resetn) begin
            busy0       <= 32'b0;
            busy1       <= 32'b0;
            busy2       <= 32'b0;
            busy3       <= 32'b0;
            busy4       <= 32'b0;
            busy_master <= 32'b0;
        end else begin
            if (predict_hit) begin
                busy0       <= (predict_tag == 5'b00001) ? next_busy_master : (next_busy0 & busy0); 
                busy1       <= (predict_tag == 5'b00010) ? next_busy_master : (next_busy1 & busy1);
                busy2       <= (predict_tag == 5'b00100) ? next_busy_master : (next_busy2 & busy2);
                busy3       <= (predict_tag == 5'b01000) ? next_busy_master : (next_busy3 & busy3);
                busy4       <= (predict_tag == 5'b10000) ? next_busy_master : (next_busy4 & busy4);
                busy_master <= next_busy_master;       
            end else if (predict_miss) begin //if(predict_hit)
                if (predict_tag == 5'b00010) begin
                    busy0       <= busy1;
                    busy1       <= busy1;
                    busy2       <= busy1;
                    busy3       <= busy1;
                    busy4       <= busy1;
                    busy_master <= busy1;
                end else if (predict_tag == 5'b00100) begin
                    busy0       <= busy2;
                    busy1       <= busy2;
                    busy2       <= busy2;
                    busy3       <= busy2;
                    busy4       <= busy2;
                    busy_master <= busy2;
                end else if (predict_tag == 5'b01000) begin
                    busy0       <= busy3;
                    busy1       <= busy3;
                    busy2       <= busy3;
                    busy3       <= busy3;
                    busy4       <= busy3;
                    busy_master <= busy3;
                end else if (predict_tag == 5'b10000) begin
                    busy0       <= busy4;
                    busy1       <= busy4;
                    busy2       <= busy4;
                    busy3       <= busy4;
                    busy4       <= busy4;
                    busy_master <= busy4;
                end else if (predict_tag == 5'b00001) begin
                    busy0       <= busy0;
                    busy1       <= busy0;
                    busy2       <= busy0;
                    busy3       <= busy0;
                    busy4       <= busy0;
                    busy_master <= busy0;
                end
            end else begin //if(predict_miss)
                if (rd1_busy_master)
                    busy_master[rd1] <= 1'b1;
                if (rd2_busy_master)
                    busy_master[rd2] <= 1'b1;
                if (com_en1_master)
                    busy_master[com_reg1]          <= 1'b0;
                if (com_en2_master)
                    busy_master[com_reg2]          <= 1'b0;

                if (rd1_busy_0)
                    busy0[rd1]      <= 1'b1;
                if (rd2_busy_0)
                    busy0[rd2]      <= 1'b1;
                if (com_en1_0)
                    busy0[com_reg1]               <= 1'b0;
                if (com_en2_0)
                    busy0[com_reg2]               <= 1'b0;

                if (rd1_busy_1)
                    busy1[rd1]      <= 1'b1;
                if (rd2_busy_1)
                    busy1[rd2]      <= 1'b1;
                if (com_en1_1)
                    busy1[com_reg1]               <= 1'b0;
                if (com_en2_1)
                    busy1[com_reg2]               <= 1'b0;

                if (rd1_busy_2)
                    busy2[rd1]      <= 1'b1;
                if (rd2_busy_2)
                    busy2[rd2]      <= 1'b1;
                if (com_en1_2)
                    busy2[com_reg1]               <= 1'b0;
                if (com_en2_2)
                    busy2[com_reg2]               <= 1'b0;

                if (rd1_busy_3)
                    busy3[rd1]      <= 1'b1;
                if (rd2_busy_3)
                    busy3[rd2]      <= 1'b1;
                if (com_en1_3)
                    busy3[com_reg1]               <= 1'b0;
                if (com_en2_3)
                    busy3[com_reg2]         <= 1'b0;

                if (rd1_busy_4)
                    busy4[rd1] <= 1'b1;
                if (rd2_busy_4)
                    busy4[rd2] <= 1'b1;
                if (com_en1_4)
                    busy4[com_reg1]         <= 1'b0;
                if (com_en2_4)
                    busy4[com_reg2]         <= 1'b0;
            end //else:!if(predict_miss)
        end //else:!if(~i_resetn)
    end //always @(posedge i_clk)

    always @(posedge i_clk) begin
        if (~i_resetn) begin
            tag0_0 <= 0;
            tag0_1 <= 0;
            tag0_2 <= 0;
            tag0_3 <= 0;
            tag0_4 <= 0;
            tag0_5 <= 0;
            
            tag1_0 <= 0;
            tag1_1 <= 0;
            tag1_2 <= 0;
            tag1_3 <= 0;
            tag1_4 <= 0;
            tag1_5 <= 0;
            
            tag2_0 <= 0;
            tag2_1 <= 0;
            tag2_2 <= 0;
            tag2_3 <= 0;
            tag2_4 <= 0;
            tag2_5 <= 0;
            
            tag3_0 <= 0;
            tag3_1 <= 0;
            tag3_2 <= 0;
            tag3_3 <= 0;
            tag3_4 <= 0;
            tag3_5 <= 0;
            
            tag4_0 <= 0;
            tag4_1 <= 0;
            tag4_2 <= 0;
            tag4_3 <= 0;
            tag4_4 <= 0;
            tag4_5 <= 0;
            
            tag_master_0 <= 0;
            tag_master_1 <= 0;
            tag_master_2 <= 0;
            tag_master_3 <= 0;
            tag_master_4 <= 0;
            tag_master_5 <= 0;
        end else if (predict_hit) begin
            //overwrite the hit renaming table with the newest renaming table
            /***************************        
            tag_master_0 <= tag_master_0;
            tag_master_1 <= tag_master_1;
            tag_master_2 <= tag_master_2;
            tag_master_3 <= tag_master_3;
            tag_master_4 <= tag_master_4;
            tag_master_5 <= tag_master_5;
            ****************************/
            if (predict_tag == 5'b00010) begin
                tag1_0 <= tag_master_0;
                tag1_1 <= tag_master_1;
                tag1_2 <= tag_master_2;
                tag1_3 <= tag_master_3;
                tag1_4 <= tag_master_4;
                tag1_5 <= tag_master_5;
            end else if (predict_tag == 5'b00100) begin
                tag2_0 <= tag_master_0;
                tag2_1 <= tag_master_1;
                tag2_2 <= tag_master_2;
                tag2_3 <= tag_master_3;
                tag2_4 <= tag_master_4;
                tag2_5 <= tag_master_5;
            end else if (predict_tag == 5'b01000) begin
                tag3_0 <= tag_master_0;
                tag3_1 <= tag_master_1;
                tag3_2 <= tag_master_2;
                tag3_3 <= tag_master_3;
                tag3_4 <= tag_master_4;
                tag3_5 <= tag_master_5;
            end else if (predict_tag == 5'b10000) begin
                tag4_0 <= tag_master_0;
                tag4_1 <= tag_master_1;
                tag4_2 <= tag_master_2;
                tag4_3 <= tag_master_3;
                tag4_4 <= tag_master_4;
                tag4_5 <= tag_master_5;
            end else if (predict_tag == 5'b00001) begin
                tag0_0 <= tag_master_0;
                tag0_1 <= tag_master_1;
                tag0_2 <= tag_master_2;
                tag0_3 <= tag_master_3;
                tag0_4 <= tag_master_4;
                tag0_5 <= tag_master_5;
            end
        end else if (predict_miss) begin //if (predict_hit)
            //restore all renaming tables to the state before missprediction
            if (predict_tag == 5'b00010) begin
                tag0_0 <= tag1_0;
                tag0_1 <= tag1_1;
                tag0_2 <= tag1_2;
                tag0_3 <= tag1_3;
                tag0_4 <= tag1_4;
                tag0_5 <= tag1_5;
                
                /****************
                tag1_0 <= tag1_0;
                tag1_1 <= tag1_1;
                tag1_2 <= tag1_2;
                tag1_3 <= tag1_3;
                tag1_4 <= tag1_4;
                tag1_5 <= tag1_5;
                ****************/
                
                tag2_0 <= tag1_0;
                tag2_1 <= tag1_1;
                tag2_2 <= tag1_2;
                tag2_3 <= tag1_3;
                tag2_4 <= tag1_4;
                tag2_5 <= tag1_5;
                
                tag3_0 <= tag1_0;
                tag3_1 <= tag1_1;
                tag3_2 <= tag1_2;
                tag3_3 <= tag1_3;
                tag3_4 <= tag1_4;
                tag3_5 <= tag1_5;
                
                tag4_0 <= tag1_0;
                tag4_1 <= tag1_1;
                tag4_2 <= tag1_2;
                tag4_3 <= tag1_3;
                tag4_4 <= tag1_4;
                tag4_5 <= tag1_5;
                
                tag_master_0 <= tag1_0;
                tag_master_1 <= tag1_1;
                tag_master_2 <= tag1_2;
                tag_master_3 <= tag1_3;
                tag_master_4 <= tag1_4;
                tag_master_5 <= tag1_5;
            end else if (predict_tag == 5'b00100) begin
                tag0_0 <= tag2_0;
                tag0_1 <= tag2_1;
                tag0_2 <= tag2_2;
                tag0_3 <= tag2_3;
                tag0_4 <= tag2_4;
                tag0_5 <= tag2_5;
                
                tag1_0 <= tag2_0;
                tag1_1 <= tag2_1;
                tag1_2 <= tag2_2;
                tag1_3 <= tag2_3;
                tag1_4 <= tag2_4;
                tag1_5 <= tag2_5;
                
                /****************
                tag2_0 <= tag2_0;
                tag2_1 <= tag2_1;
                tag2_2 <= tag2_2;
                tag2_3 <= tag2_3;
                tag2_4 <= tag2_4;
                tag2_5 <= tag2_5;
                ****************/
                
                tag3_0 <= tag2_0;
                tag3_1 <= tag2_1;
                tag3_2 <= tag2_2;
                tag3_3 <= tag2_3;
                tag3_4 <= tag2_4;
                tag3_5 <= tag2_5;
                
                tag4_0 <= tag2_0;
                tag4_1 <= tag2_1;
                tag4_2 <= tag2_2;
                tag4_3 <= tag2_3;
                tag4_4 <= tag2_4;
                tag4_5 <= tag2_5;
                
                tag_master_0 <= tag2_0;
                tag_master_1 <= tag2_1;
                tag_master_2 <= tag2_2;
                tag_master_3 <= tag2_3;
                tag_master_4 <= tag2_4;
                tag_master_5 <= tag2_5;
            end else if (predict_tag == 5'b01000) begin
                tag0_0 <= tag3_0;
                tag0_1 <= tag3_1;
                tag0_2 <= tag3_2;
                tag0_3 <= tag3_3;
                tag0_4 <= tag3_4;
                tag0_5 <= tag3_5;
                
                tag1_0 <= tag3_0;
                tag1_1 <= tag3_1;
                tag1_2 <= tag3_2;
                tag1_3 <= tag3_3;
                tag1_4 <= tag3_4;
                tag1_5 <= tag3_5;

                tag2_0 <= tag3_0;
                tag2_1 <= tag3_1;
                tag2_2 <= tag3_2;
                tag2_3 <= tag3_3;
                tag2_4 <= tag3_4;
                tag2_5 <= tag3_5;
                
                /****************
                tag3_0 <= tag3_0;
                tag3_1 <= tag3_1;
                tag3_2 <= tag3_2;
                tag3_3 <= tag3_3;
                tag3_4 <= tag3_4;
                tag3_5 <= tag3_5;
                ****************/
                
                tag4_0 <= tag3_0;
                tag4_1 <= tag3_1;
                tag4_2 <= tag3_2;
                tag4_3 <= tag3_3;
                tag4_4 <= tag3_4;
                tag4_5 <= tag3_5;
                
                tag_master_0 <= tag3_0;
                tag_master_1 <= tag3_1;
                tag_master_2 <= tag3_2;
                tag_master_3 <= tag3_3;
                tag_master_4 <= tag3_4;
                tag_master_5 <= tag3_5;
            end else if (predict_tag == 5'b10000) begin
                tag0_0 <= tag4_0;
                tag0_1 <= tag4_1;
                tag0_2 <= tag4_2;
                tag0_3 <= tag4_3;
                tag0_4 <= tag4_4;
                tag0_5 <= tag4_5;
                
                tag1_0 <= tag4_0;
                tag1_1 <= tag4_1;
                tag1_2 <= tag4_2;
                tag1_3 <= tag4_3;
                tag1_4 <= tag4_4;
                tag1_5 <= tag4_5;

                tag2_0 <= tag4_0;
                tag2_1 <= tag4_1;
                tag2_2 <= tag4_2;
                tag2_3 <= tag4_3;
                tag2_4 <= tag4_4;
                tag2_5 <= tag4_5;

                tag3_0 <= tag4_0;
                tag3_1 <= tag4_1;
                tag3_2 <= tag4_2;
                tag3_3 <= tag4_3;
                tag3_4 <= tag4_4;
                tag3_5 <= tag4_5;
                
                /****************
                tag4_0 <= tag4_0;
                tag4_1 <= tag4_1;
                tag4_2 <= tag4_2;
                tag4_3 <= tag4_3;
                tag4_4 <= tag4_4;
                tag4_5 <= tag4_5;
                ****************/
                
                tag_master_0 <= tag4_0;
                tag_master_1 <= tag4_1;
                tag_master_2 <= tag4_2;
                tag_master_3 <= tag4_3;
                tag_master_4 <= tag4_4;
                tag_master_5 <= tag4_5;
            end else if (predict_tag == 5'b00001) begin
                /****************
                tag0_0 <= tag0_0;
                tag0_1 <= tag0_1;
                tag0_2 <= tag0_2;
                tag0_3 <= tag0_3;
                tag0_4 <= tag0_4;
                tag0_5 <= tag0_5;
                ****************/
                
                tag1_0 <= tag0_0;
                tag1_1 <= tag0_1;
                tag1_2 <= tag0_2;
                tag1_3 <= tag0_3;
                tag1_4 <= tag0_4;
                tag1_5 <= tag0_5;

                tag2_0 <= tag0_0;
                tag2_1 <= tag0_1;
                tag2_2 <= tag0_2;
                tag2_3 <= tag0_3;
                tag2_4 <= tag0_4;
                tag2_5 <= tag0_5;

                tag3_0 <= tag0_0;
                tag3_1 <= tag0_1;
                tag3_2 <= tag0_2;
                tag3_3 <= tag0_3;
                tag3_4 <= tag0_4;
                tag3_5 <= tag0_5;
                
                tag4_0 <= tag0_0;
                tag4_1 <= tag0_1;
                tag4_2 <= tag0_2;
                tag4_3 <= tag0_3;
                tag4_4 <= tag0_4;
                tag4_5 <= tag0_5;
                
                tag_master_0 <= tag0_0;
                tag_master_1 <= tag0_1;
                tag_master_2 <= tag0_2;
                tag_master_3 <= tag0_3;
                tag_master_4 <= tag0_4;
                tag_master_5 <= tag0_5;
            end 
        end else begin //if(predict_miss)
            if (rd1_valid) begin
                //TAG0
                tag_master_0[rd1] <= rd1_valid_arb ?
                                                 rd1_renamedtag[0] : tag_master_0[rd1];
                tag0_0[rd1] <= ( ~mpft_valid1[0] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[0] : tag0_0[rd1];
                tag0_1[rd1] <= ( ~mpft_valid1[1] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[0] : tag0_1[rd1];
                tag0_2[rd1] <= ( ~mpft_valid1[2] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[0] : tag0_2[rd1];
                tag0_3[rd1] <= ( ~mpft_valid1[3] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[0] : tag0_3[rd1];
                tag0_4[rd1] <= ( ~mpft_valid1[4] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[0] : tag0_4[rd1];
    
                //TAG1
                tag_master_1[rd1] <= rd1_valid_arb ?
                                                 rd1_renamedtag[1] : tag_master_1[rd1];
                tag0_1[rd1] <= ( ~mpft_valid1[0] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[1] : tag0_1[rd1];
                tag1_1[rd1] <= ( ~mpft_valid1[1] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[1] : tag1_1[rd1];
                tag1_2[rd1] <= ( ~mpft_valid1[2] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[1] : tag1_2[rd1];
                tag1_3[rd1] <= ( ~mpft_valid1[3] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[1] : tag1_3[rd1];
                tag1_4[rd1] <= ( ~mpft_valid1[4] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[1] : tag1_4[rd1];

                //TAG2
                tag_master_2[rd1] <= rd1_valid_arb ?
                                                 rd1_renamedtag[2] : tag_master_2[rd1];
                tag0_2[rd1] <= ( ~mpft_valid1[0] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[2] : tag0_2[rd1];
                tag1_2[rd1] <= ( ~mpft_valid1[1] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[2] : tag1_2[rd1];
                tag2_2[rd1] <= ( ~mpft_valid1[2] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[2] : tag2_2[rd1];
                tag2_3[rd1] <= ( ~mpft_valid1[3] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[2] : tag2_3[rd1];
                tag2_4[rd1] <= ( ~mpft_valid1[4] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[2] : tag2_4[rd1];                        
                //TAG3
                tag_master_3[rd1] <= rd1_valid_arb ?
                                                 rd1_renamedtag[3] : tag_master_3[rd1];
                tag0_3[rd1] <= ( ~mpft_valid1[0] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[3] : tag0_3[rd1];
                tag1_3[rd1] <= ( ~mpft_valid1[1] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[3] : tag1_3[rd1];
                tag3_2[rd1] <= ( ~mpft_valid1[2] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[3] : tag3_2[rd1];
                tag3_3[rd1] <= ( ~mpft_valid1[3] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[3] : tag3_3[rd1];
                tag3_4[rd1] <= ( ~mpft_valid1[4] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[3] : tag3_4[rd1];

                //TAG4
                tag_master_4[rd1] <= rd1_valid_arb ?
                                                 rd1_renamedtag[4] : tag_master_4[rd1];
                tag0_4[rd1] <= ( ~mpft_valid1[0] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[4] : tag0_4[rd1];
                tag1_4[rd1] <= ( ~mpft_valid1[1] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[4] : tag1_4[rd1];
                tag4_2[rd1] <= ( ~mpft_valid1[2] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[4] : tag4_2[rd1];
                tag4_3[rd1] <= ( ~mpft_valid1[3] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[4] : tag4_3[rd1];
                tag4_4[rd1] <= ( ~mpft_valid1[4] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[4] : tag4_4[rd1];

                //TAG5
                tag_master_5[rd1] <= rd1_valid_arb ?
                                                 rd1_renamedtag[5] : tag_master_5[rd1];
                tag0_5[rd1] <= ( ~mpft_valid1[0] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[5] : tag0_5[rd1];
                tag1_5[rd1] <= ( ~mpft_valid1[1] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[5] : tag1_5[rd1];
                tag5_2[rd1] <= ( ~mpft_valid1[2] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[5] : tag5_2[rd1];
                tag5_3[rd1] <= ( ~mpft_valid1[3] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[5] : tag5_3[rd1];
                tag5_4[rd1] <= ( ~mpft_valid1[4] & (rd1_valid_arb | (inst1_spectag != inst2_spectag))  ) ?
                                            rd1_renamedtag[5] : tag5_4[rd1];
         
            end //if(setttagbusy1)
            if (rd2_valid) begin
                //TAG0
                tag_master_0[rd2] <= rd2_renamedtag[0];
                tag0_0[rd2]      <= ~mpft_valid2[0] ? rd2_renamedtag[0] : tag0_0[rd2];
                tag0_1[rd2]      <= ~mpft_valid2[1] ? rd2_renamedtag[0] : tag0_1[rd2];
                tag0_2[rd2]      <= ~mpft_valid2[2] ? rd2_renamedtag[0] : tag0_2[rd2];
                tag0_3[rd2]      <= ~mpft_valid2[3] ? rd2_renamedtag[0] : tag0_3[rd2];
                tag0_4[rd2]      <= ~mpft_valid2[4] ? rd2_renamedtag[0] : tag0_4[rd2];
                //TAG1
                tag_master_1[rd2] <= rd2_renamedtag[1];
                tag0_1[rd2]      <= ~mpft_valid2[0] ? rd2_renamedtag[1] : tag0_1[rd2];
                tag1_1[rd2]      <= ~mpft_valid2[1] ? rd2_renamedtag[1] : tag1_1[rd2];
                tag1_2[rd2]      <= ~mpft_valid2[2] ? rd2_renamedtag[1] : tag1_2[rd2];
                tag1_3[rd2]      <= ~mpft_valid2[3] ? rd2_renamedtag[1] : tag1_3[rd2];
                tag1_4[rd2]      <= ~mpft_valid2[4] ? rd2_renamedtag[1] : tag1_4[rd2];
                //TAG2
                tag_master_2[rd2] <= rd2_renamedtag[2];
                tag0_2[rd2]      <= ~mpft_valid2[0] ? rd2_renamedtag[2] : tag0_2[rd2];
                tag1_2[rd2]      <= ~mpft_valid2[1] ? rd2_renamedtag[2] : tag1_2[rd2];
                tag2_2[rd2]      <= ~mpft_valid2[2] ? rd2_renamedtag[2] : tag2_2[rd2];
                tag2_3[rd2]      <= ~mpft_valid2[3] ? rd2_renamedtag[2] : tag2_3[rd2];
                tag2_4[rd2]      <= ~mpft_valid2[4] ? rd2_renamedtag[2] : tag2_4[rd2];
                //TAG3
                tag_master_3[rd2] <= rd2_renamedtag[3];
                tag0_3[rd2]      <= ~mpft_valid2[0] ? rd2_renamedtag[3] : tag0_3[rd2];
                tag1_3[rd2]      <= ~mpft_valid2[1] ? rd2_renamedtag[3] : tag1_3[rd2];
                tag3_2[rd2]      <= ~mpft_valid2[2] ? rd2_renamedtag[3] : tag3_2[rd2];
                tag3_3[rd2]      <= ~mpft_valid2[3] ? rd2_renamedtag[3] : tag3_3[rd2];
                tag3_4[rd2]      <= ~mpft_valid2[4] ? rd2_renamedtag[3] : tag3_4[rd2];
                //TAG4
                tag_master_4[rd2] <= rd2_renamedtag[4];
                tag0_4[rd2]      <= ~mpft_valid2[0] ? rd2_renamedtag[4] : tag0_4[rd2];
                tag1_4[rd2]      <= ~mpft_valid2[1] ? rd2_renamedtag[4] : tag1_4[rd2];
                tag4_2[rd2]      <= ~mpft_valid2[2] ? rd2_renamedtag[4] : tag4_2[rd2];
                tag4_3[rd2]      <= ~mpft_valid2[3] ? rd2_renamedtag[4] : tag4_3[rd2];
                tag4_4[rd2]      <= ~mpft_valid2[4] ? rd2_renamedtag[4] : tag4_4[rd2];
                //TAG5
                tag_master_5[rd2] <= rd2_renamedtag[5];
                tag0_5[rd2]      <= ~mpft_valid2[0] ? rd2_renamedtag[5] : tag0_5[rd2];
                tag1_5[rd2]      <= ~mpft_valid2[1] ? rd2_renamedtag[5] : tag1_5[rd2];
                tag5_2[rd2]      <= ~mpft_valid2[2] ? rd2_renamedtag[5] : tag5_2[rd2];
                tag5_3[rd2]      <= ~mpft_valid2[3] ? rd2_renamedtag[5] : tag5_3[rd2];
                tag5_4[rd2]      <= ~mpft_valid2[4] ? rd2_renamedtag[5] : tag5_4[rd2];
            end //if(rd2_valid)
        end //else:!if(predict_miss)
    end //always@(posedge i_clk)
endmodule //Rename_Table





module RRF_Freelist_Manage  //Circular queue
(
    input                       i_clk,
    input                       i_resetn,
    
    input                       invalid1,
    input                       invalid2,
    input      [1:0]            com_num,
    input      [`RRF_ADDRW-1:0] com_ptr,
    input                       predict_miss,
    input      [`RRF_ADDRW-1:0] rrftagfix,
    input                       stall_DP, //= ~o_allocatable && ~predict_miss
    
    output                      o_allocatable,
    output     [`RRF_ADDRW-1:0] o_rename_rd1,
    output     [`RRF_ADDRW-1:0] o_rename_rd2,
    output reg [`RRF_ADDRW  :0] o_free_num_q,
    output reg [`RRF_ADDRW-1:0] o_rrf_ptr_q,
    output reg                  o_next_rrf_cyc_q
); 
    wire [1:0]            req_num     = {1'b0, ~invalid1} + {1'b0, ~invalid2};
    wire                  hi          = (com_ptr > rrftagfix) ? 1'b1 : 1'b0;
    wire [`RRF_ADDRW-1:0] rrfptr_next = o_rrf_ptr_q + req_num;
   
    assign o_allocatable = (req_num <= (o_free_num_q + com_num)) ? 1'b1 : 1'b0;
    assign o_rename_rd1 = o_rrf_ptr_q;
    assign o_rename_rd2 = o_rrf_ptr_q + (~invalid1 ? 1 : 0);
   
    always @ (posedge i_clk) begin
        if (~i_resetn) begin
            o_free_num_q     <= `RRF_DEPTH;
            o_rrf_ptr_q      <= 0;
            o_next_rrf_cyc_q <= 0;
        end else if (predict_miss) begin
            o_rrf_ptr_q      <= rrftagfix; //predict_miss_rrftag+1
            o_free_num_q     <= `RRF_DEPTH - ({hi, rrftagfix} - {1'b0, com_ptr});
            o_next_rrf_cyc_q <= 0;
        end else if (stall_DP) begin
            o_rrf_ptr_q      <= o_rrf_ptr_q;
            o_free_num_q     <= o_free_num_q + com_num;
            o_next_rrf_cyc_q <= 0;
        end else begin
            o_rrf_ptr_q      <= rrfptr_next;
            o_free_num_q     <= o_free_num_q + com_num - req_num;
            o_next_rrf_cyc_q <= (o_rrf_ptr_q > rrfptr_next) ? 1'b1 : 1'b0;
        end
    end
endmodule //RRF_Freelist_Manage


module RRF //Rename Register File
(
    input                   i_clk,
    input                   i_resetn,
    
    input  [`RRF_ADDRW-1:0] rs1_rrftag1,
    input  [`RRF_ADDRW-1:0] rs2_rrftag1,
    input  [`RRF_ADDRW-1:0] rs1_rrftag2,
    input  [`RRF_ADDRW-1:0] rs2_rrftag2,
    input  [`RRF_ADDRW-1:0] com_tag1,
    input  [`RRF_ADDRW-1:0] com_tag2,
    output                  rs1_valid1,
    output                  rs2_valid1,
    output                  rs1_valid2,
    output                  rs2_valid2,
    output [31:0]           rs1_data1,
    output [31:0]           rs2_data1,
    output [31:0]           rs1_data2,
    output [31:0]           rs2_data2,
    output [31:0]           com_data1,
    output [31:0]           com_data2,
    
    input                   rrfwe1,
    input                   rrfwe2,
    input                   rrfwe3,
    input                   rrfwe4,
    input                   rrfwe5,
    input  [`RRF_ADDRW-1:0] rrfwaddr1,
    input  [`RRF_ADDRW-1:0] rrfwaddr2,
    input  [`RRF_ADDRW-1:0] rrfwaddr3,
    input  [`RRF_ADDRW-1:0] rrfwaddr4,
    input  [`RRF_ADDRW-1:0] rrfwaddr5,
    input  [31:0]           rrfwdata1,
    input  [31:0]           rrfwdata2,
    input  [31:0]           rrfwdata3,
    input  [31:0]           rrfwdata4,
    input  [31:0]           rrfwdata5,
    
    input                   dpen1,
    input                   dpen2
    input  [`RRF_ADDRW-1:0] dpaddr1,
    input  [`RRF_ADDRW-1:0] dpaddr2
);

    reg [`RRF_DEPTH-1:0] valid;                    //valid bit
    reg [31:0]           rrfmem [0:`RRF_DEPTH-1];  //rename register file

`ifdef RRF_INIT_ZERO
    integer i;
    initial begin
        for (i = 0; i < `RRF_DEPTH; i = i+1) begin
            rrfmem[i] = 0;
        end
    end
`endif

    assign rs1_data1  = rrfmem[rs1_rrftag1];
    assign rs2_data1  = rrfmem[rs2_rrftag1];
    assign rs1_data2  = rrfmem[rs1_rrftag2];
    assign rs2_data2  = rrfmem[rs2_rrftag2];
    assign com_data1  = rrfmem[com_tag1];
    assign com_data2  = rrfmem[com_tag2];

    assign rs1_valid1 = valid[rs1_rrftag1];
    assign rs2_valid1 = valid[rs2_rrftag1];
    assign rs1_valid2 = valid[rs1_rrftag2];
    assign rs2_valid2 = valid[rs2_rrftag2];

    wire [`RRF_DEPTH-1:0] or_valid  = (~rrfwe1 ?   `RRF_DEPTH'b0  :  (`RRF_DEPTH'b1 << rrfwaddr1)) |
                                      (~rrfwe2 ?   `RRF_DEPTH'b0  :  (`RRF_DEPTH'b1 << rrfwaddr2)) |
                                      (~rrfwe3 ?   `RRF_DEPTH'b0  :  (`RRF_DEPTH'b1 << rrfwaddr3)) |
                                      (~rrfwe4 ?   `RRF_DEPTH'b0  :  (`RRF_DEPTH'b1 << rrfwaddr4)) |
                                      (~rrfwe5 ?   `RRF_DEPTH'b0  :  (`RRF_DEPTH'b1 << rrfwaddr5)) ;
                                      
    wire [`RRF_DEPTH-1:0] and_valid = (~dpen1  ? ~(`RRF_DEPTH'b0) : ~(`RRF_DEPTH'b1 << dpaddr1))   & 
                                      (~dpen2  ? ~(`RRF_DEPTH'b0) : ~(`RRF_DEPTH'b1 << dpaddr2))   ;

    always @(posedge i_clk) begin
        if(~i_resetn) begin
            valid <= 0;
        end else begin
            valid <= (valid | or_valid) & and_valid;
        end
    end

    always @(posedge i_clk) begin
        if(~i_resetn) begin
            if(rrfwe1) begin
                rrfmem[rrfwaddr1] <= rrfwdata1;
            end
            if(rrfwe2) begin
                rrfmem[rrfwaddr2] <= rrfwdata2;
            end
            if(rrfwe3) begin
                rrfmem[rrfwaddr3] <= rrfwdata3;
            end
            if(rrfwe4) begin
                rrfmem[rrfwaddr4] <= rrfwdata4;
            end
            if(rrfwe5) begin
                rrfmem[rrfwaddr5] <= rrfwdata5;
            end
        end
    end
endmodule //RRF