/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Create Date: 2021.12.24
 *  Description: the issue stage of rv32Core
 */
`include "configure.h"

module SRC_MUX
(
    input  [31:0]           i_arf_data,
    input                   i_arf_busy,
    input  [`RRF_ADDRW-1:0] i_arf_rrftag,
    
    input  [31:0]           i_rrf_data,
    input                   i_rrf_valid,

    input  [`RRF_ADDRW-1:0] i_rd_renamed,
    input                   i_rs_eq_0,
    input                   i_rs_eq_rd,
    output [31:0]           o_mux_data,
    output                  o_mux_ready
);
    assign o_mux_data  = i_rs_eq_0   ? 32'b0      :      //src = x0
                         i_rs_eq_rd  ? i_rd_renamed :      //src = rd_renamed
                         ~i_arf_busy ? i_arf_data   :      //src = arf_data
                         i_rrf_valid ? i_rrf_data   :      //src = rrf_data
                                       i_arf_rrftag ;      //src = o_rrftag_q
    assign o_mux_ready = i_rs_eq_0 | (~i_rs_eq_rd & (~i_arf_busy | i_rrf_valid));
endmodule //SRC_MUX


module Imm_Generate
(
    input      [31:0] i_inst,
    input      [2:0]  i_inst_type,
    output reg [31:0] o_imm_q
);
    always @(*) begin
        (* parallel_case *)
        case (i_inst_type)
            //`R_type : ;
            `I_type : o_imm_q = $signed(i_inst[31:20]);
            `S_type : o_imm_q = $signed({i_inst[31:25], i_inst[11:7]});
            `B_type : o_imm_q = $signed({i_inst[31], i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0});
            `U_type : o_imm_q = $signed(i_inst[31:12] << 12);
            `J_type : o_imm_q = $signed({i_inst[31], i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0});
            default : o_imm_q = 1'bx;
        endcase //case(i_inst_type)
   end
endmodule //Imm_Generate



module BranchImm_Generate
(
    input  [31:0] i_inst,
    output [31:0] o_branch_imm
);
    wire [6:0]  opcode      = i_inst[6:0];
    wire [31:0] br_offset   = $signed({i_inst[31], i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0};
    wire [31:0] jal_offset  = $signed({i_inst[31], i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0});
    wire [31:0] jalr_offset = $signed(i_inst[31:20]);
   
    assign o_branch_imm = (opcode == `BRANCH) ? br_offset   :
                          (opcode == `JAL)    ? jal_offset  :
                          (opcode == `JALR)   ? jalr_offset : 32'b0;
endmodule //BranchImm_Generate



module RAW_Resolve
(
    //data in ARF or RRF
    input  [31:0]           i_mux_data,
    input                   i_mux_ready,
    
    //execution stage data forwarding
    input  [31:0]           i_exe_result1,  //from ALU1
    input  [`RRF_ADDRW-1:0] i_exe_rrftag1,
    input                   i_flush_spec1,
    input  [31:0]           i_exe_result2,  //from ALU2
    input  [`RRF_ADDRW-1:0] i_exe_rrftag2,
    input                   i_flush_spec2,
    input  [31:0]           i_exe_result3,  //from LDST
    input  [`RRF_ADDRW-1:0] i_exe_rrftag3,
    input                   i_flush_spec3,
    input  [31:0]           i_exe_result4,  //from BRANCH
    input  [`RRF_ADDRW-1:0] i_exe_rrftag4,
    input                   i_flush_spec4,
    input  [31:0]           i_exe_result5,  //from MUL
    input  [`RRF_ADDRW-1:0] i_exe_rrftag5,
    input                   i_flush_spec5,
    input  [31:0]           i_exe_result6,  //from DIV
    input  [`RRF_ADDRW-1:0] i_exe_rrftag6,
    input                   i_flush_spec6,
    
    //the final data of source operand
    output [31:0]           o_src_data,
    output                  o_src_ready
);
    wire exe_ready1 = ~i_flush_spec1 & (i_exe_rrftag1 == i_mux_data);
    wire exe_ready2 = ~i_flush_spec2 & (i_exe_rrftag2 == i_mux_data);
    wire exe_ready3 = ~i_flush_spec3 & (i_exe_rrftag3 == i_mux_data);
    wire exe_ready4 = ~i_flush_spec4 & (i_exe_rrftag4 == i_mux_data);
    wire exe_ready5 = ~i_flush_spec5 & (i_exe_rrftag5 == i_mux_data);
    wire exe_ready6 = ~i_flush_spec6 & (i_exe_rrftag6 == i_mux_data);
    
    assign o_src_data = i_mux_ready  ? i_mux_data    :
                        i_exe_ready1 ? i_exe_result1 :
                        i_exe_ready2 ? i_exe_result2 :
                        i_exe_ready3 ? i_exe_result3 :
                        i_exe_ready4 ? i_exe_result4 :
                        i_exe_ready5 ? i_exe_result5 : 
                        i_exe_ready6 ? i_exe_result6 : 
                                       i_mux_data    ;
    
    assign o_src_ready = i_mux_ready | i_exe_ready1 | i_exe_ready2 | i_exe_ready3 | 
                                       i_exe_ready4 | i_exe_ready5 | i_exe_ready5 ;
endmodule //RAW_Resolve


module RS_ReqGenerate
(
    //from decoder
    input  [2:0] i_rs_type1,
    input  [2:0] i_rs_type2,
    
    output       o_alu_req1,
    output       o_alu_req2,
    output [1:0] o_alu_reqNum,
    
    output       o_branch_req1,
    output       o_branch_req2,
    output [1:0] o_branch_reqNum,
    
    output       o_mul_req1,
    output       o_mul_req2,
    output [1:0] o_mul_reqNum,
    
    output       o_div_req1,
    output       o_div_req2,
    output [1:0] o_div_reqNum,
    
    output       o_ldst_req1,
    output       o_ldst_req2,
    output [1:0] o_ldst_reqNum
);
    assign o_alu_req1      = (i_rs_type1 == `RS_ALU)    ? 1'b1 : 1'b0;
    assign o_alu_req2      = (i_rs_type2 == `RS_ALU)    ? 1'b1 : 1'b0;
    assign o_alu_reqNum    = o_alu_req1 + o_alu_req2;

    assign o_branch_req1   = (i_rs_type1 == `RS_BRANCH) ? 1'b1 : 1'b0;
    assign o_branch_req2   = (i_rs_type2 == `RS_BRANCH) ? 1'b1 : 1'b0;
    assign o_branch_reqNum = o_branch_req1 + o_branch_req2;

    assign o_mul_req1      = (i_rs_type1 == `RS_MUL)    ? 1'b1 : 1'b0;
    assign o_mul_req2      = (i_rs_type2 == `RS_MUL)    ? 1'b1 : 1'b0;
    assign o_mul_reqNum    = o_mul_req1 + o_mul_req2;
    
    assign o_div_req1      = (i_rs_type1 == `RS_DIV)    ? 1'b1 : 1'b0;
    assign o_div_req2      = (i_rs_type2 == `RS_DIV)    ? 1'b1 : 1'b0;
    assign o_div_reqNum    = o_div_req1 + o_div_req2;

    assign o_ldst_req1     = (i_rs_type1 == `RS_LDST)   ? 1'b1 : 1'b0;
    assign o_ldst_req2     = (i_rs_type2 == `RS_LDST)   ? 1'b1 : 1'b0;
    assign o_ldst_reqNum   = o_ldst_req1 + o_ldst_req2;  
endmodule //RS_ReqGenerate


module ENTRY_SEL 
#(
    parameter ENTRY_NUM   = 2,
    parameter ENTRY_ADDRW = 1   //log2(ENTRY_NUM)
)
(
    input      [ENTRY_NUM-1:0]   i_busy_vector,
    output reg                   o_entry_en_q
    output reg [ENTRY_ADDRW-1:0] o_entry_addr_q,
);
    integer i;
    always @ (*) begin
        o_entry_en_q   = 1'b0;
        o_entry_addr_q = 0;
        
        //lower addresses have higher priority
        for (i = ENTRY_NUM-1 ; i >= 0 ; i = i-1) begin
            if (~i_busy_vector[i]) begin
                o_entry_en_q   = 1'b1;
                o_entry_addr_q = i;                
            end
        end
    end
endmodule //ENTRY_SEL


module ENTRY_MASK
#(
    parameter ENTRY_NUM   = 2,
    parameter ENTRY_ADDRW = 1   //log2(ENTRY_NUM)
)
(
    input      [ENTRY_ADDRW-1:0] i_selected_entry,
    output reg [ENTRY_NUM-1:0]   o_busy_mask_q      //1:mask, 0:unmask
);
    
    //mask all equal and lower addresses than selected entry
    integer i;
    always @ (*) begin
        o_busy_mask_q = 0;
        for (i = 0 ; i < ENTRY_NUM ; i = i+1) begin
            o_busy_mask_q[i] = (i_selected_entry < i) ? 1'b0 : 1'b1;
        end
    end
endmodule  //ENTRY_MASK


module AllocateUnit
#(
    parameter ENTRY_NUM   = 2,
    parameter ENTRY_ADDRW = 1
)
(
    input  [ENTRY_NUM-1:0]   i_busy_vector,
    input  [1:0]             i_reqnum,
    
    output                   o_en1,
    output                   o_en2,
    output [ENTRY_ADDRW-1:0] o_free_entry1,
    output [ENTRY_ADDRW-1:0] o_free_entry2,
    output                   o_allocatable
);
   
    wire [ENTRY_NUM-1:0] busy_mask;
    assign o_allocatable = (i_reqnum > ({1'b0,o_en1}+{1'b0,o_en2})) ? 1'b0 : 1'b1;
    
    ENTRY_SEL
    #(
        .ENTRY_NUM   (ENTRY_NUM  ),
        .ENTRY_ADDRW (ENTRY_ADDRW)
    )
    entry_sel1
    (
        .i_busy_vector   (i_busy_vector),
        .o_entry_en_q    (o_en1        ),
        .o_entry_addr_q  (o_free_entry1)
    );

    ENTRY_MASK
    #(
        .ENTRY_NUM      (ENTRY_NUM  ),
        .ENTRY_ADDRW    (ENTRY_ADDRW)
    )
    entry_mask
    (
        .i_selected_entry (o_free_entry1),
        .o_busy_mask_q    (busy_mask  )
    );
    
    ENTRY_SEL
    #(
        .ENTRY_NUM   (ENTRY_NUM  ),
        .ENTRY_ADDRW (ENTRY_ADDRW)
    )
    entry_sel2
    (
        .i_busy_vector (i_busy_vector | busy_mask),
        .o_entry_en_q    (o_en2                    ),
        .o_entry_addr_q  (o_free_entry2            )
    );
endmodule //AllocateUnit



/*
module Oldest_Finder2  //find the oldest entry of 2 entries
#(
    parameter ENTRY_ADDRW = 1,
    parameter ENTRY_WIDTH = 8
)
(
    input  [2*ENTRY_ADDRW-1:0] i_entry_vector,
    input  [2*ENTRY_WIDTH-1:0] i_value_vector,
    output [ENTRY_ADDRW-1:0]   o_oldest_entry,
    output [ENTRY_WIDTH-1:0]   o_oldest_value
);
    wire   [ENTRY_ADDRW-1:0]   entry2 = i_entry_vector[ENTRY_ADDRW+:ENTRY_ADDRW];
    wire   [ENTRY_ADDRW-1:0]   entry1 = i_entry_vector[0          +:ENTRY_ADDRW];
    wire   [ENTRY_WIDTH-1:0]   value2 = i_value_vector[ENTRY_WIDTH+:ENTRY_WIDTH];
    wire   [ENTRY_WIDTH-1:0]   value1 = i_value_vector[0          +:ENTRY_WIDTH];
    
    //entry_value = {~o_data_ready, reg_cycle_lable_vector, rrftag[5:0]}
    assign o_oldest_entry = (value2 < value1) ? entry2 : entry1;
    assign o_oldest_value = (value2 < value1) ? value2 : value1;
endmodule //Oldest_Finder2

module Oldest_Finder4  //find the oldest entry of 4 entries
#(
    parameter ENTRY_ADDRW = 2,
    parameter ENTRY_WIDTH = 8
)
(
    input  [4*ENTRY_ADDRW-1:0] i_entry_vector,
    input  [4*ENTRY_WIDTH-1:0] i_value_vector,
    output [ENTRY_ADDRW-1:0]   o_oldest_entry,
    output [ENTRY_WIDTH-1:0]   o_oldest_value
);
    wire   [ENTRY_ADDRW-1:0]   oldest_entry1;
    wire   [ENTRY_ADDRW-1:0]   oldest_entry2;
    wire   [ENTRY_WIDTH-1:0]   oldest_value1;
    wire   [ENTRY_WIDTH-1:0]   oldest_value2;

    Oldest_Finder2 
    #(
        .ENTRY_ADDRW(ENTRY_ADDRW),
        .ENTRY_WIDTH(ENTRY_WIDTH)
    ) 
    oldest_finder2_entry01
    (
        .i_entry_vector(i_entry_vector[0+:2*ENTRY_ADDRW]),
        .i_value_vector(i_value_vector[0+:2*ENTRY_WIDTH]),
        .o_oldest_entry(oldest_entry1),
        .o_oldest_value(oldest_value1)
    );

    Oldest_Finder2
    #(
        .ENTRY_ADDRW(ENTRY_ADDRW),
        .ENTRY_WIDTH(ENTRY_WIDTH)
    )
    oldest_finder2_entry23
    (
        .i_entry_vector(i_entry_vector[2*ENTRY_ADDRW+:2*ENTRY_ADDRW]),
        .i_value_vector(i_value_vector[2*ENTRY_WIDTH+:2*ENTRY_WIDTH]),
        .o_oldest_entry(oldest_entry2),
        .o_oldest_value(oldest_value2)
    );

    Oldest_Finder2
    #(
        .ENTRY_ADDRW(ENTRY_ADDRW),
        .ENTRY_WIDTH(ENTRY_WIDTH)
    )
    oldest_finder2_entry01_23
    (
        .i_entry_vector({oldest_entry2, oldest_entry1}),
        .i_value_vector({oldest_value2, oldest_value1}),
        .o_oldest_entry(o_oldest_entry),
        .o_oldest_value(o_oldest_value)
    ); 
endmodule //Oldest_Finder4

module Oldest_Finder8
#(
    parameter ENTRY_ADDRW = 3,
    parameter ENTRY_WIDTH = 8
)
(
    input  [8*ENTRY_ADDRW-1:0] i_entry_vector,
    input  [8*ENTRY_WIDTH-1:0] i_value_vector,
    output [ENTRY_ADDRW-1:0]   o_oldest_entry,
    output [ENTRY_WIDTH-1:0]   o_oldest_value
   );
    wire   [ENTRY_ADDRW-1:0]   oldest_entry1;
    wire   [ENTRY_ADDRW-1:0]   oldest_entry2;
    wire   [ENTRY_WIDTH-1:0]   oldest_value1;
    wire   [ENTRY_WIDTH-1:0]   oldest_value2;
   
    Oldest_Finder4
    #(
        .ENTRY_ADDRW(ENTRY_ADDRW),
        .ENTRY_WIDTH(ENTRY_WIDTH)
    )
    oldest_finder4_entry0123
    (
        .i_entry_vector(i_entry_vector[0+:4*ENTRY_ADDRW]),
        .i_value_vector(i_value_vector[0+:4*ENTRY_WIDTH]),
        .o_oldest_entry(oldest_entry1),
        .o_oldest_value(oldest_value1)
    );

    Oldest_Finder4
    #(
        .ENTRY_ADDRW(ENTRY_ADDRW),
        .ENTRY_WIDTH(ENTRY_WIDTH)
    )
    oldest_finder4_entry4567
    (
        .i_entry_vector(i_entry_vector[4+:4*ENTRY_ADDRW]),
        .i_value_vector(i_value_vector[4+:4*ENTRY_WIDTH]),
        .o_oldest_entry(oldest_entry2),
        .o_oldest_value(oldest_value2)
    );

    Oldest_Finder2
    #(
        .ENTRY_ADDRW(ENTRY_ADDRW),
        .ENTRY_WIDTH(ENTRY_WIDTH)
    )
    oldest_finder2_entry0123_4567
    (
        .i_entry_vector({oldest_entry2, oldest_entry1}),
        .i_value_vector({oldest_value2, oldest_value1}),
        .o_oldest_entry(o_oldest_entry),
        .o_oldest_value(o_oldest_value)
    ); 
endmodule //Oldest_Finder8

*/

module RS_ALU_Entry  //the entry of an alu reservation station
(
    input                       i_clk,
    input                       i_resetn,    
    
    input                       i_busy,
    input                       i_we,
    input      [31:0]           i_PC,
    input      [31:0]           i_src1_data,
    input      [31:0]           i_src2_data,
    input                       i_src1_ready,
    input                       i_src2_ready,
    input      [31:0]           i_imm,
    input      [`RRF_ADDRW-1:0] i_rename_rd,
    input                       i_rd_valid,
    input      [1:0]            i_src1_sel,
    input      [1:0]            i_src2_sel,
    input      [3:0]            i_alu_op,
    input      [4:0]            i_spectag,
 
    output reg [31:0]           o_PC_q,
    output reg [31:0]           o_imm_q,
    output reg [`RRF_ADDRW-1:0] o_rrftag_q,
    output reg                  o_rrftag_valid_q,
    output reg [1:0]            o_src1_sel_q,
    output reg [1:0]            o_src2_sel_q,
    output reg [3:0]            o_alu_op_q,
    output reg [4:0]            o_spectag_q,
    
    output reg                  o_data_ready_q,
    output reg [31:0]           o_src1_data_q,
    output reg [31:0]           o_src2_data_q,
    
    //execution result forwarding
    input      [31:0]           i_exe_result1,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag1,
    input                       i_flush_spec1,
    input      [31:0]           i_exe_result2,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag2,
    input                       i_flush_spec2,
    input      [31:0]           i_exe_result3,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag3,
    input                       i_flush_spec3,
    input      [31:0]           i_exe_result4,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag4,
    input                       i_flush_spec4,
    input      [31:0]           i_exe_result5,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag5,
    input                       i_flush_spec5
);
    wire [31:0] src1_data;
    wire [31:0] src2_data;
    wire        src1_ready;
    wire        src2_ready;

    wire  data_ready = i_busy & src1_ready & src2_ready;
   
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_PC_q           <= 32'b0;
            o_imm_q          <= 32'b0;
            o_rrftag_q       <= `RRF_ADDRW'b0;
            o_rrftag_valid_q <= 1'b0;
            o_src1_sel_q     <= 2'b0;
            o_src2_sel_q     <= 2'b0;
            o_alu_op_q       <= 4'b0;
            o_spectag_q      <= 5'b0;
            
            o_data_ready_q   <= 1'b0;
            o_src1_data_q    <= 32'b0;
            o_src2_data_q    <= 32'b0;
        end else if (i_we) begin //if (i_resetn)
            o_PC_q           <= i_PC;
            o_imm_q          <= i_imm;
            o_rrftag_q       <= i_rename_rd;
            o_rrftag_valid_q <= i_rd_valid;
            o_src1_sel_q     <= i_src1_sel;
            o_src2_sel_q     <= i_src2_sel;
            o_alu_op_q       <= i_alu_op;
            o_spectag_q      <= i_spectag;

            o_data_ready_q   <= data_ready;
            o_src1_data_q    <= src1_data;
            o_src2_data_q    <= src2_data;
        end //if(i_we)
    end //always @(posedge i_clk)
   
    RAW_Resolve rs_alu_raw_resolve1
    (
        .i_mux_data    (i_src1_data  ),
        .i_mux_ready   (i_src1_ready ),
        .i_exe_result1 (i_exe_result1),
        .i_exe_rrftag1 (i_exe_rrftag1),
        .i_flush_spec1 (i_flush_spec1),
        .i_exe_result2 (i_exe_result2),
        .i_exe_rrftag2 (i_exe_rrftag2),
        .i_flush_spec2 (i_flush_spec2),
        .i_exe_result3 (i_exe_result3),
        .i_exe_rrftag3 (i_exe_rrftag3),
        .i_flush_spec3 (i_flush_spec3),
        .i_exe_result4 (i_exe_result4),
        .i_exe_rrftag4 (i_exe_rrftag4),
        .i_flush_spec4 (i_flush_spec4),
        .i_exe_result5 (i_exe_result5),
        .i_exe_rrftag5 (i_exe_rrftag5),
        .i_flush_spec5 (i_flush_spec5),
        .o_src_data    (src1_data    ),
        .o_src_ready   (src1_ready   )
    );

    RAW_Resolve rs_alu_raw_resolve2
    (
        .i_mux_data    (i_src2_data  ),
        .i_mux_ready   (i_src2_ready ),
        .i_exe_result1 (i_exe_result1),
        .i_exe_rrftag1 (i_exe_rrftag1),
        .i_flush_spec1 (i_flush_spec1),
        .i_exe_result2 (i_exe_result2),
        .i_exe_rrftag2 (i_exe_rrftag2),
        .i_flush_spec2 (i_flush_spec2),
        .i_exe_result3 (i_exe_result3),
        .i_exe_rrftag3 (i_exe_rrftag3),
        .i_flush_spec3 (i_flush_spec3),
        .i_exe_result4 (i_exe_result4),
        .i_exe_rrftag4 (i_exe_rrftag4),
        .i_flush_spec4 (i_flush_spec4),
        .i_exe_result5 (i_exe_result5),
        .i_exe_rrftag5 (i_exe_rrftag5),
        .i_flush_spec5 (i_flush_spec5),
        .o_src_data    (src2_data    ),
        .o_src_ready   (src2_ready   )
    );  
endmodule //RS_ALU_Entry

module AGE_Matrix  //select the oldest ready entry(instruction) in reservation station to issue
(
    input        i_clk,
    input        i_resetn,
    input        i_dispatch1_en,
    input        i_dispatch2_en,
    input  [2:0] i_dispatch1_addr,
    input  [2:0] i_dispatch2_addr,
    input  [7:0] i_busy_vector,
    input  [7:0] i_data_ready_vector,
    output       o_issue_en,
    output [2:0] o_issue_addr
);
    reg  [7:0] reg_age_matrix [0:7];
    reg  [7:0] wire_oldest_entry;
    always @(posedge i_clk) begin
        if (i_resetn) begin
            reg_age_matrix[0] <= 8'b0;
            reg_age_matrix[1] <= 8'b0;
            reg_age_matrix[2] <= 8'b0;
            reg_age_matrix[3] <= 8'b0;
            reg_age_matrix[4] <= 8'b0;
            reg_age_matrix[5] <= 8'b0;
            reg_age_matrix[6] <= 8'b0;
            reg_age_matrix[7] <= 8'b0;
        end else begin
            if (i_dispatch1_en) begin
                reg_age_matrix[i_dispatch1_addr] <= i_busy_vector;
            end
            if (i_dispatch2_en) begin
                reg_age_matrix[i_dispatch2_addr] <= i_busy_vector | 
                                                    (i_dispatch1_en ? (8'b1 << i_dispatch1_addr) : 8'b0);
            end     
        end
    end

    assign wire_oldest_entry[0] =  (~(
                                   (reg_age_matrix[0][1] & i_busy_vector[1]) |
                                   (reg_age_matrix[0][2] & i_busy_vector[2]) |
                                   (reg_age_matrix[0][3] & i_busy_vector[3]) |
                                   (reg_age_matrix[0][4] & i_busy_vector[4]) |
                                   (reg_age_matrix[0][5] & i_busy_vector[5]) |
                                   (reg_age_matrix[0][6] & i_busy_vector[6]) |
                                   (reg_age_matrix[0][7] & i_busy_vector[7]) )) &
                                   i_busy_vector[0] ;

    assign wire_oldest_entry[1] =  (~(
                                   (reg_age_matrix[1][0] & i_busy_vector[0]) |
                                   (reg_age_matrix[1][2] & i_busy_vector[2]) |
                                   (reg_age_matrix[1][3] & i_busy_vector[3]) |
                                   (reg_age_matrix[1][4] & i_busy_vector[4]) |
                                   (reg_age_matrix[1][5] & i_busy_vector[5]) |
                                   (reg_age_matrix[1][6] & i_busy_vector[6]) |
                                   (reg_age_matrix[1][7] & i_busy_vector[7]) )) &
                                   i_busy_vector[1] ;

    assign wire_oldest_entry[2] =  (~(
                                   (reg_age_matrix[2][0] & i_busy_vector[0]) |
                                   (reg_age_matrix[2][1] & i_busy_vector[1]) |
                                   (reg_age_matrix[2][3] & i_busy_vector[3]) |
                                   (reg_age_matrix[2][4] & i_busy_vector[4]) |
                                   (reg_age_matrix[2][5] & i_busy_vector[5]) |
                                   (reg_age_matrix[2][6] & i_busy_vector[6]) |
                                   (reg_age_matrix[2][7] & i_busy_vector[7]) )) &
                                   i_busy_vector[2] ;

    assign wire_oldest_entry[3] =  (~(
                                   (reg_age_matrix[3][0] & i_busy_vector[0]) |
                                   (reg_age_matrix[3][1] & i_busy_vector[1]) |
                                   (reg_age_matrix[3][2] & i_busy_vector[2]) |
                                   (reg_age_matrix[3][4] & i_busy_vector[4]) |
                                   (reg_age_matrix[3][5] & i_busy_vector[5]) |
                                   (reg_age_matrix[3][6] & i_busy_vector[6]) |
                                   (reg_age_matrix[3][7] & i_busy_vector[7]) )) &
                                   i_busy_vector[3] ;

    assign wire_oldest_entry[4] =  (~(
                                   (reg_age_matrix[4][0] & i_busy_vector[0]) |
                                   (reg_age_matrix[4][1] & i_busy_vector[1]) |
                                   (reg_age_matrix[4][2] & i_busy_vector[2]) |
                                   (reg_age_matrix[4][3] & i_busy_vector[3]) |
                                   (reg_age_matrix[4][5] & i_busy_vector[5]) |
                                   (reg_age_matrix[4][6] & i_busy_vector[6]) |
                                   (reg_age_matrix[4][7] & i_busy_vector[7]) )) &
                                   i_busy_vector[4] ;

    assign wire_oldest_entry[5] =  (~(
                                   (reg_age_matrix[5][0] & i_busy_vector[0]) |
                                   (reg_age_matrix[5][1] & i_busy_vector[1]) |
                                   (reg_age_matrix[5][2] & i_busy_vector[2]) |
                                   (reg_age_matrix[5][3] & i_busy_vector[3]) |
                                   (reg_age_matrix[5][4] & i_busy_vector[4]) |
                                   (reg_age_matrix[5][6] & i_busy_vector[6]) |
                                   (reg_age_matrix[5][7] & i_busy_vector[7]) )) &
                                   i_busy_vector[5] ;

    assign wire_oldest_entry[6] =  (~(
                                   (reg_age_matrix[6][0] & i_busy_vector[0]) |
                                   (reg_age_matrix[6][1] & i_busy_vector[1]) |
                                   (reg_age_matrix[6][2] & i_busy_vector[2]) |
                                   (reg_age_matrix[6][3] & i_busy_vector[3]) |
                                   (reg_age_matrix[6][4] & i_busy_vector[4]) |
                                   (reg_age_matrix[6][5] & i_busy_vector[5]) |
                                   (reg_age_matrix[6][7] & i_busy_vector[7]) )) &
                                   i_busy_vector[6] ;

    assign wire_oldest_entry[7] =  (~(
                                   (reg_age_matrix[7][0] & i_busy_vector[0]) |
                                   (reg_age_matrix[7][1] & i_busy_vector[1]) |
                                   (reg_age_matrix[7][2] & i_busy_vector[2]) |
                                   (reg_age_matrix[7][3] & i_busy_vector[3]) |
                                   (reg_age_matrix[7][4] & i_busy_vector[4]) |
                                   (reg_age_matrix[7][5] & i_busy_vector[5]) |
                                   (reg_age_matrix[7][6] & i_busy_vector[6]) )) &
                                   i_busy_vector[7] ;
    
    always @ (*) begin
        o_issue_en   = 1'b0;
        o_issue_addr = 3'b0;
        if (wire_oldest_entry[7] == 1'b1 && i_data_ready_vector[7]) begin
            o_issue_en   = 1'b1;
            o_issue_addr = 3'b111;
        end
        if (wire_oldest_entry[6] == 1'b1 && i_data_ready_vector[6]) begin
            o_issue_en   = 1'b1;
            o_issue_addr = 3'b110;
        end
        if (wire_oldest_entry[5] == 1'b1 && i_data_ready_vector[5]) begin
            o_issue_en   = 1'b1;
            o_issue_addr = 3'b101;
        end
        if (wire_oldest_entry[4] == 1'b1 && i_data_ready_vector[4]) begin
            o_issue_en   = 1'b1;
            o_issue_addr = 3'b100;
        end
        if (wire_oldest_entry[3] == 1'b1 && i_data_ready_vector[3]) begin
            o_issue_en   = 1'b1;
            o_issue_addr = 3'b011;
        end
        if (wire_oldest_entry[1] == 1'b1 && i_data_ready_vector[2]) begin
            o_issue_en   = 1'b1;
            o_issue_addr = 3'b010;
        end
        if (wire_oldest_entry[1] == 1'b1 && i_data_ready_vector[1]) begin
            o_issue_en   = 1'b1;
            o_issue_addr = 3'b001;
        end
        if (wire_oldest_entry[0] == 1'b1 && i_data_ready_vector[0]) begin
            o_issue_en   = 1'b1;
            o_issue_addr = 3'b000;
        end
    end
endmodule  //AGE_Matrix

module RS_ALU  //alu reservation station
(
    input                             i_clk,
    input                             i_resetn,

    input                             i_predict_miss,
    input                             i_predict_hit,
    input      [4:0]                  i_predict_tag,
    input      [4:0]                  i_specfixtag,
    input                             i_next_rrf_cyc, 
    output reg [7:0]                  o_busy_vector_q,
    //output     [8*(`RRF_ADDRW+2)-1:0] o_entry_state_vector,

    //dispatch and issue signal
    input                             i_issue_en,
    input      [2:0]                  i_issue_addr,
    input                             i_dispatch1_en,
    input                             i_dispatch2_en,
    input      [2:0]                  i_dispatch1_addr,
    input      [2:0]                  i_dispatch2_addr,
    
    //input 1
    input      [31:0]                 i_PC_1,
    input      [31:0]                 i_src1_data_1,
    input      [31:0]                 i_src2_data_1,
    input                             i_src1_ready_1,
    input                             i_src2_ready_1,
    input      [31:0]                 i_imm_1,
    input      [`RRF_ADDRW-1:0]       i_rename_rd_1,
    input                             i_rd_valid_1,
    input      [1:0]                  i_src1_sel_1,
    input      [1:0]                  i_src2_sel_1,
    input      [3:0]                  i_alu_op_1,
    input      [4:0]                  i_spectag_1,
    input                             i_spectag_valid_1,
    
    //input 2
    input      [31:0]                 i_PC_2,
    input      [31:0]                 i_src1_data_2,
    input      [31:0]                 i_src2_data_2,
    input                             i_src1_ready_2,
    input                             i_src2_ready_2,
    input      [31:0]                 i_imm_2,
    input      [`RRF_ADDRW-1:0]       i_rename_rd_2,
    input                             i_rd_valid_2,
    input      [1:0]                  i_src1_sel_2,
    input      [1:0]                  i_src2_sel_2,
    input      [3:0]                  i_alu_op_2,
    input      [4:0]                  i_spectag_2,
    input                             i_spectag_valid_2,

    //output
    output     [31:0]                 o_src1_data,
    output     [31:0]                 o_src2_data,
    output     [7:0]                  o_data_ready,
    output     [31:0]                 o_PC,
    output     [31:0]                 o_imm,
    output     [`RRF_ADDRW-1:0]       o_rrftag,
    output                            o_rrftag_valid,
    output     [1:0]                  o_src1_sel,
    output     [1:0]                  o_src2_sel,
    output     [3:0]                  o_alu_op,
    output     [4:0]                  o_spectag,
    output                            o_spectag_valid,
  
    //execution result forwarding
    input      [31:0]                 i_exe_result1,
    input      [`RRF_ADDRW-1:0]       i_exe_rrftag1,
    input                             i_flush_spec1,
    input      [31:0]                 i_exe_result2,
    input      [`RRF_ADDRW-1:0]       i_exe_rrftag2,
    input                             i_flush_spec2,
    input      [31:0]                 i_exe_result3,
    input      [`RRF_ADDRW-1:0]       i_exe_rrftag3,
    input                             i_flush_spec3,
    input      [31:0]                 i_exe_result4,
    input      [`RRF_ADDRW-1:0]       i_exe_rrftag4,
    input                             i_flush_spec4,
    input      [31:0]                 i_exe_result5,
    input      [`RRF_ADDRW-1:0]       i_exe_rrftag5,
    input                             i_flush_spec5
);
    reg  [7:0] reg_spectag_valid_vector;
    reg  [7:0] reg_cycle_lable_vector;
   
    wire [7:0] wire_busy_set_vector          = {(wire_o_spectag_7 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                                (wire_o_spectag_6 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                                (wire_o_spectag_5 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                                (wire_o_spectag_4 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                                (wire_o_spectag_3 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                                (wire_o_spectag_2 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                                (wire_o_spectag_1 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                                (wire_o_spectag_0 & i_specfixtag) == 0 ? 1'b1 : 1'b0};

    wire [7:0] wire_spectag_valid_set_vector = {(wire_o_spectag_7 == i_predict_tag) ? 1'b0 : 1'b1,
                                                (wire_o_spectag_6 == i_predict_tag) ? 1'b0 : 1'b1,
                                                (wire_o_spectag_5 == i_predict_tag) ? 1'b0 : 1'b1,
                                                (wire_o_spectag_4 == i_predict_tag) ? 1'b0 : 1'b1,
                                                (wire_o_spectag_3 == i_predict_tag) ? 1'b0 : 1'b1,
                                                (wire_o_spectag_2 == i_predict_tag) ? 1'b0 : 1'b1,
                                                (wire_o_spectag_1 == i_predict_tag) ? 1'b0 : 1'b1,
                                                (wire_o_spectag_0 == i_predict_tag) ? 1'b0 : 1'b1};

    wire [7:0] wire_spectag_valid_vector_next = (wire_spectag_valid_set_vector & reg_spectag_valid_vector);

    assign o_spectag_valid = i_predict_hit ? wire_spectag_valid_vector_next[i_issue_addr] : reg_spectag_valid_vector[i_issue_addr];
    assign o_data_ready    = {wire_o_data_ready_7, wire_o_data_ready_6, wire_o_data_ready_5, wire_o_data_ready_4, 
                              wire_o_data_ready_3, wire_o_data_ready_2, wire_o_data_ready_1, wire_o_data_ready_0};

    /*
    assign o_entry_state_vector = { {~wire_o_data_ready_7, reg_cycle_lable_vector[7], wire_o_rrftag_7},
                                    {~wire_o_data_ready_6, reg_cycle_lable_vector[6], wire_o_rrftag_6},
                                    {~wire_o_data_ready_5, reg_cycle_lable_vector[5], wire_o_rrftag_5},
                                    {~wire_o_data_ready_4, reg_cycle_lable_vector[4], wire_o_rrftag_4},
                                    {~wire_o_data_ready_3, reg_cycle_lable_vector[3], wire_o_rrftag_3},
                                    {~wire_o_data_ready_2, reg_cycle_lable_vector[2], wire_o_rrftag_2},
                                    {~wire_o_data_ready_1, reg_cycle_lable_vector[1], wire_o_rrftag_1},
                                    {~wire_o_data_ready_0, reg_cycle_lable_vector[0], wire_o_rrftag_0} };
    
    always @(posedge i_clk) begin
        if (i_resetn) begin
            reg_cycle_lable_vector <= 8'b1;
        end else if (i_next_rrf_cyc) begin
            reg_cycle_lable_vector <= (i_dispatch1_en ? (8'b1 << i_dispatch1_addr) : 8'b0) |
                                      (i_dispatch2_en ? (8'b1 << i_dispatch2_addr) : 8'b0) ;
        end else begin
            if (i_dispatch1_en) begin
                reg_cycle_lable_vector[i_dispatch1_addr] <= 1'b1;
            end
            if (i_dispatch2_en) begin
                reg_cycle_lable_vector[i_dispatch2_addr] <= 1'b1;
            end
        end
    end
    */
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_busy_vector_q          <= 8'b0;
            reg_spectag_valid_vector <= 8'b0;
        end else begin //if (i_resetn)
            if (i_predict_miss) begin
                o_busy_vector_q          <= wire_busy_set_vector & o_busy_vector_q;
                reg_spectag_valid_vector <= 8'b0;
            end else if (i_predict_hit) begin //if (i_predict_miss)
                reg_spectag_valid_vector <= wire_spectag_valid_vector_next;
                if (i_issue_en) begin
                    o_busy_vector_q[i_issue_addr] <= 1'b0;
                end
            end else begin //else if (i_predict_hit)
                if (i_dispatch1_en) begin
                    o_busy_vector_q[i_dispatch1_addr]          <= 1'b1;
                    reg_spectag_valid_vector[i_dispatch1_addr] <= i_spectag_valid_1;
                end
                if (i_dispatch2_en) begin
                    o_busy_vector_q[i_dispatch2_addr]          <= 1'b1;
                    reg_spectag_valid_vector[i_dispatch2_addr] <= i_spectag_valid_2;
                end
                if (i_issue_en) begin
                    o_busy_vector_q[i_issue_addr]              <= 1'b0;
                end
            end //else: !if (i_predict_hit)
        end
    end




    wire                  wire_dispatch1_en_0   = i_dispatch1_en && (i_dispatch1_addr == 3'b000);
    wire                  wire_dispatch2_en_0   = i_dispatch2_en && (i_dispatch2_addr == 3'b000);
    wire                  wire_i_we_0           = wire_dispatch1_en_0 || wire_dispatch2_en_0;
    wire [31:0]           wire_i_PC_0           = wire_dispatch1_en_0 ? i_PC_1         : i_PC_2;
    wire [31:0]           wire_i_src1_data_0    = wire_dispatch1_en_0 ? i_src1_data_1  : i_src1_data_2;
    wire [31:0]           wire_i_src2_data_0    = wire_dispatch1_en_0 ? i_src2_data_1  : i_src2_data_2;
    wire                  wire_i_src1_ready_0   = wire_dispatch1_en_0 ? i_src1_ready_1 : i_src1_ready_2;
    wire                  wire_i_src2_ready_0   = wire_dispatch1_en_0 ? i_src2_ready_1 : i_src2_ready_2;
    wire [31:0]           wire_i_imm_0          = wire_dispatch1_en_0 ? i_imm_1        : i_imm_2;
    wire [`RRF_ADDRW-1:0] wire_i_rename_rd_0    = wire_dispatch1_en_0 ? i_rename_rd_1  : i_rename_rd_2;
    wire                  wire_i_rd_valid_0     = wire_dispatch1_en_0 ? i_rd_valid_1   : i_rd_valid_2;
    wire [1:0]            wire_i_src1_sel_0     = wire_dispatch1_en_0 ? i_src1_sel_1   : i_src1_sel_2;
    wire [1:0]            wire_i_src2_sel_0     = wire_dispatch1_en_0 ? i_src2_sel_1   : i_src2_sel_2;
    wire [3:0]            wire_i_alu_op_0       = wire_dispatch1_en_0 ? i_alu_op_1     : i_alu_op_2;
    wire [4:0]            wire_i_spectag_0      = wire_dispatch1_en_0 ? i_spectag_1    : i_spectag_2;
    
    wire [31:0]           wire_o_PC_0;
    wire [31:0]           wire_o_imm_0;
    wire [`RRF_ADDRW-1:0] wire_o_rrftag_0;
    wire                  wire_o_rrftag_valid_0;
    wire [1:0]            wire_o_src1_sel_0;
    wire [1:0]            wire_o_src2_sel_0;
    wire [3:0]            wire_o_alu_op_0;
    wire [4:0]            wire_o_spectag_0;
    wire [31:0]           wire_o_src1_data_0;
    wire [31:0]           wire_o_src2_data_0;
    wire                  wire_o_data_ready_0;
    
    RS_ALU_Entry rs_alu_entry0
    (
        .i_clk            (i_clk                ),
        .i_resetn         (i_resetn             ),
        .i_busy           (o_busy_vector_q[0]   ),
        .i_we             (wire_i_we_0          ),
        .i_PC             (wire_i_PC_0          ),
        .i_src1_data      (wire_i_src1_data_0   ),
        .i_src2_data      (wire_i_src2_data_0   ),
        .i_src1_ready     (wire_i_src1_ready_0  ),
        .i_src2_ready     (wire_i_src2_ready_0  ),
        .i_imm            (wire_i_imm_0         ),
        .i_rename_rd      (wire_i_rename_rd_0   ),
        .i_rd_valid       (wire_i_rd_valid_0    ),
        .i_src1_sel       (wire_i_src1_sel_0    ),
        .i_src2_sel       (wire_i_src2_sel_0    ),
        .i_alu_op         (wire_i_alu_op_0      ),
        .i_spectag        (wire_i_spectag_0     ),

        .o_PC_q           (wire_o_PC_0          ),
        .o_imm_q          (wire_o_imm_0         ),
        .o_rrftag_q       (wire_o_rrftag_0      ),
        .o_rrftag_valid_q (wire_o_rrftag_valid_0),
        .o_src1_sel_q     (wire_o_src1_sel_0    ),
        .o_src2_sel_q     (wire_o_src2_sel_0    ),
        .o_alu_op_q       (wire_o_alu_op_0      ),
        .o_spectag_q      (wire_o_spectag_0     ),

        .o_src1_data      (wire_o_src1_data_0   ),
        .o_src2_data      (wire_o_src2_data_0   ),
        .o_data_ready     (wire_o_data_ready_0  ),
        
        .i_exe_result1    (i_exe_result1        ),
        .i_exe_rrftag1    (i_exe_rrftag1        ),
        .i_flush_spec1    (i_flush_spec1        ),
        .i_exe_result2    (i_exe_result2        ),
        .i_exe_rrftag2    (i_exe_rrftag2        ),
        .i_flush_spec2    (i_flush_spec2        ),
        .i_exe_result3    (i_exe_result3        ),
        .i_exe_rrftag3    (i_exe_rrftag3        ),
        .i_flush_spec3    (i_flush_spec3        ),
        .i_exe_result4    (i_exe_result4        ),
        .i_exe_rrftag4    (i_exe_rrftag4        ),
        .i_flush_spec4    (i_flush_spec4        ),
        .i_exe_result5    (i_exe_result5        ),
        .i_exe_rrftag5    (i_exe_rrftag5        ),
        .i_flush_spec5    (i_flush_spec5        )
    );

    wire                  wire_dispatch1_en_1   = i_dispatch1_en && (i_dispatch1_addr == 3'b001);
    wire                  wire_dispatch2_en_1   = i_dispatch2_en && (i_dispatch2_addr == 3'b001);
    wire                  wire_i_we_1           = wire_dispatch1_en_1 || wire_dispatch2_en_1;
    wire [31:0]           wire_i_PC_1           = wire_dispatch1_en_1 ? i_PC_1         : i_PC_2;
    wire [31:0]           wire_i_src1_data_1    = wire_dispatch1_en_1 ? i_src1_data_1  : i_src1_data_2;
    wire [31:0]           wire_i_src2_data_1    = wire_dispatch1_en_1 ? i_src2_data_1  : i_src2_data_2;
    wire                  wire_i_src1_ready_1   = wire_dispatch1_en_1 ? i_src1_ready_1 : i_src1_ready_2;
    wire                  wire_i_src2_ready_1   = wire_dispatch1_en_1 ? i_src2_ready_1 : i_src2_ready_2;
    wire [31:0]           wire_i_imm_1          = wire_dispatch1_en_1 ? i_imm_1        : i_imm_2;
    wire [`RRF_ADDRW-1:0] wire_i_rename_rd_1    = wire_dispatch1_en_1 ? i_rename_rd_1  : i_rename_rd_2;
    wire                  wire_i_rd_valid_1     = wire_dispatch1_en_1 ? i_rd_valid_1   : i_rd_valid_2;
    wire [1:0]            wire_i_src1_sel_1     = wire_dispatch1_en_1 ? i_src1_sel_1   : i_src1_sel_2;
    wire [1:0]            wire_i_src2_sel_1     = wire_dispatch1_en_1 ? i_src2_sel_1   : i_src2_sel_2;
    wire [3:0]            wire_i_alu_op_1       = wire_dispatch1_en_1 ? i_alu_op_1     : i_alu_op_2;
    wire [4:0]            wire_i_spectag_1      = wire_dispatch1_en_1 ? i_spectag_1    : i_spectag_2;
    
    wire [31:0]           wire_o_PC_1;
    wire [31:0]           wire_o_imm_1;
    wire [`RRF_ADDRW-1:0] wire_o_rrftag_1;
    wire                  wire_o_rrftag_valid_1;
    wire [1:0]            wire_o_src1_sel_1;
    wire [1:0]            wire_o_src2_sel_1;
    wire [3:0]            wire_o_alu_op_1;
    wire [4:0]            wire_o_spectag_1;
    wire [31:0]           wire_o_src1_data_1;
    wire [31:0]           wire_o_src2_data_1;
    wire                  wire_o_data_ready_1;
    
    RS_ALU_Entry rs_alu_entry1
    (
        .i_clk            (i_clk                ),
        .i_resetn         (i_resetn             ),
        .i_busy           (o_busy_vector_q[1]   ),
        .i_we             (wire_i_we_1          ),
        .i_PC             (wire_i_PC_1          ),
        .i_src1_data      (wire_i_src1_data_1   ),
        .i_src2_data      (wire_i_src2_data_1   ),
        .i_src1_ready     (wire_i_src1_ready_1  ),
        .i_src2_ready     (wire_i_src2_ready_1  ),
        .i_imm            (wire_i_imm_1         ),
        .i_rename_rd      (wire_i_rename_rd_1   ),
        .i_rd_valid       (wire_i_rd_valid_1    ),
        .i_src1_sel       (wire_i_src1_sel_1    ),
        .i_src2_sel       (wire_i_src2_sel_1    ),
        .i_alu_op         (wire_i_alu_op_1      ),
        .i_spectag        (wire_i_spectag_1     ),

        .o_PC_q           (wire_o_PC_1          ),
        .o_imm_q          (wire_o_imm_1         ),
        .o_rrftag_q       (wire_o_rrftag_1      ),
        .o_rrftag_valid_q (wire_o_rrftag_valid_1),
        .o_src1_sel_q     (wire_o_src1_sel_1    ),
        .o_src2_sel_q     (wire_o_src2_sel_1    ),
        .o_alu_op_q       (wire_o_alu_op_1      ),
        .o_spectag_q      (wire_o_spectag_1     ),

        .o_src1_data      (wire_o_src1_data_1   ),
        .o_src2_data      (wire_o_src2_data_1   ),
        .o_data_ready     (wire_o_data_ready_1  ),
        
        .i_exe_result1    (i_exe_result1        ),
        .i_exe_rrftag1    (i_exe_rrftag1        ),
        .i_flush_spec1    (i_flush_spec1        ),
        .i_exe_result2    (i_exe_result2        ),
        .i_exe_rrftag2    (i_exe_rrftag2        ),
        .i_flush_spec2    (i_flush_spec2        ),
        .i_exe_result3    (i_exe_result3        ),
        .i_exe_rrftag3    (i_exe_rrftag3        ),
        .i_flush_spec3    (i_flush_spec3        ),
        .i_exe_result4    (i_exe_result4        ),
        .i_exe_rrftag4    (i_exe_rrftag4        ),
        .i_flush_spec4    (i_flush_spec4        ),
        .i_exe_result5    (i_exe_result5        ),
        .i_exe_rrftag5    (i_exe_rrftag5        ),
        .i_flush_spec5    (i_flush_spec5        )
    );

    wire                  wire_dispatch1_en_2   = i_dispatch1_en && (i_dispatch1_addr == 3'b010);
    wire                  wire_dispatch2_en_2   = i_dispatch2_en && (i_dispatch2_addr == 3'b010);
    wire                  wire_i_we_2           = wire_dispatch1_en_2 || wire_dispatch2_en_2;
    wire [31:0]           wire_i_PC_2           = wire_dispatch1_en_2 ? i_PC_1         : i_PC_2;
    wire [31:0]           wire_i_src1_data_2    = wire_dispatch1_en_2 ? i_src1_data_1  : i_src1_data_2;
    wire [31:0]           wire_i_src2_data_2    = wire_dispatch1_en_2 ? i_src2_data_1  : i_src2_data_2;
    wire                  wire_i_src1_ready_2   = wire_dispatch1_en_2 ? i_src1_ready_1 : i_src1_ready_2;
    wire                  wire_i_src2_ready_2   = wire_dispatch1_en_2 ? i_src2_ready_1 : i_src2_ready_2;
    wire [31:0]           wire_i_imm_2          = wire_dispatch1_en_2 ? i_imm_1        : i_imm_2;
    wire [`RRF_ADDRW-1:0] wire_i_rename_rd_2    = wire_dispatch1_en_2 ? i_rename_rd_1  : i_rename_rd_2;
    wire                  wire_i_rd_valid_2     = wire_dispatch1_en_2 ? i_rd_valid_1   : i_rd_valid_2;
    wire [1:0]            wire_i_src1_sel_2     = wire_dispatch1_en_2 ? i_src1_sel_1   : i_src1_sel_2;
    wire [1:0]            wire_i_src2_sel_2     = wire_dispatch1_en_2 ? i_src2_sel_1   : i_src2_sel_2;
    wire [3:0]            wire_i_alu_op_2       = wire_dispatch1_en_2 ? i_alu_op_1     : i_alu_op_2;
    wire [4:0]            wire_i_spectag_2      = wire_dispatch1_en_2 ? i_spectag_1    : i_spectag_2;
    
    wire [31:0]           wire_o_PC_2;
    wire [31:0]           wire_o_imm_2;
    wire [`RRF_ADDRW-1:0] wire_o_rrftag_2;
    wire                  wire_o_rrftag_valid_2;
    wire [1:0]            wire_o_src1_sel_2;
    wire [1:0]            wire_o_src2_sel_2;
    wire [3:0]            wire_o_alu_op_2;
    wire [4:0]            wire_o_spectag_2;
    wire [31:0]           wire_o_src1_data_2;
    wire [31:0]           wire_o_src2_data_2;
    wire                  wire_o_data_ready_2;
    
    RS_ALU_Entry rs_alu_entry2
    (
        .i_clk            (i_clk                ),
        .i_resetn         (i_resetn             ),
        .i_busy           (o_busy_vector_q[2]   ),
        .i_we             (wire_i_we_2          ),
        .i_PC             (wire_i_PC_2          ),
        .i_src1_data      (wire_i_src1_data_2   ),
        .i_src2_data      (wire_i_src2_data_2   ),
        .i_src1_ready     (wire_i_src1_ready_2  ),
        .i_src2_ready     (wire_i_src2_ready_2  ),
        .i_imm            (wire_i_imm_2         ),
        .i_rename_rd      (wire_i_rename_rd_2   ),
        .i_rd_valid       (wire_i_rd_valid_2    ),
        .i_src1_sel       (wire_i_src1_sel_2    ),
        .i_src2_sel       (wire_i_src2_sel_2    ),
        .i_alu_op         (wire_i_alu_op_2      ),
        .i_spectag        (wire_i_spectag_2     ),

        .o_PC_q           (wire_o_PC_2          ),
        .o_imm_q          (wire_o_imm_2         ),
        .o_rrftag_q       (wire_o_rrftag_2      ),
        .o_rrftag_valid_q (wire_o_rrftag_valid_2),
        .o_src1_sel_q     (wire_o_src1_sel_2    ),
        .o_src2_sel_q     (wire_o_src2_sel_2    ),
        .o_alu_op_q       (wire_o_alu_op_2      ),
        .o_spectag_q      (wire_o_spectag_2     ),

        .o_src1_data      (wire_o_src1_data_2   ),
        .o_src2_data      (wire_o_src2_data_2   ),
        .o_data_ready     (wire_o_data_ready_2  ),
        
        .i_exe_result1    (i_exe_result1        ),
        .i_exe_rrftag1    (i_exe_rrftag1        ),
        .i_flush_spec1    (i_flush_spec1        ),
        .i_exe_result2    (i_exe_result2        ),
        .i_exe_rrftag2    (i_exe_rrftag2        ),
        .i_flush_spec2    (i_flush_spec2        ),
        .i_exe_result3    (i_exe_result3        ),
        .i_exe_rrftag3    (i_exe_rrftag3        ),
        .i_flush_spec3    (i_flush_spec3        ),
        .i_exe_result4    (i_exe_result4        ),
        .i_exe_rrftag4    (i_exe_rrftag4        ),
        .i_flush_spec4    (i_flush_spec4        ),
        .i_exe_result5    (i_exe_result5        ),
        .i_exe_rrftag5    (i_exe_rrftag5        ),
        .i_flush_spec5    (i_flush_spec5        )
    );
    
    wire                  wire_dispatch1_en_3   = i_dispatch1_en && (i_dispatch1_addr == 3'b011);
    wire                  wire_dispatch2_en_3   = i_dispatch2_en && (i_dispatch2_addr == 3'b011);
    wire                  wire_i_we_3           = wire_dispatch1_en_3 || wire_dispatch2_en_3;
    wire [31:0]           wire_i_PC_3           = wire_dispatch1_en_3 ? i_PC_1         : i_PC_2;
    wire [31:0]           wire_i_src1_data_3    = wire_dispatch1_en_3 ? i_src1_data_1  : i_src1_data_2;
    wire [31:0]           wire_i_src2_data_3    = wire_dispatch1_en_3 ? i_src2_data_1  : i_src2_data_2;
    wire                  wire_i_src1_ready_3   = wire_dispatch1_en_3 ? i_src1_ready_1 : i_src1_ready_2;
    wire                  wire_i_src2_ready_3   = wire_dispatch1_en_3 ? i_src2_ready_1 : i_src2_ready_2;
    wire [31:0]           wire_i_imm_3          = wire_dispatch1_en_3 ? i_imm_1        : i_imm_2;
    wire [`RRF_ADDRW-1:0] wire_i_rename_rd_3    = wire_dispatch1_en_3 ? i_rename_rd_1  : i_rename_rd_2;
    wire                  wire_i_rd_valid_3     = wire_dispatch1_en_3 ? i_rd_valid_1   : i_rd_valid_2;
    wire [1:0]            wire_i_src1_sel_3     = wire_dispatch1_en_3 ? i_src1_sel_1   : i_src1_sel_2;
    wire [1:0]            wire_i_src2_sel_3     = wire_dispatch1_en_3 ? i_src2_sel_1   : i_src2_sel_2;
    wire [3:0]            wire_i_alu_op_3       = wire_dispatch1_en_3 ? i_alu_op_1     : i_alu_op_2;
    wire [4:0]            wire_i_spectag_3      = wire_dispatch1_en_3 ? i_spectag_1    : i_spectag_2;
    
    wire [31:0]           wire_o_PC_3;
    wire [31:0]           wire_o_imm_3;
    wire [`RRF_ADDRW-1:0] wire_o_rrftag_3;
    wire                  wire_o_rrftag_valid_3;
    wire [1:0]            wire_o_src1_sel_3;
    wire [1:0]            wire_o_src2_sel_3;
    wire [3:0]            wire_o_alu_op_3;
    wire [4:0]            wire_o_spectag_3;
    wire [31:0]           wire_o_src1_data_3;
    wire [31:0]           wire_o_src2_data_3;
    wire                  wire_o_data_ready_3;
    
    RS_ALU_Entry rs_alu_entry3
    (
        .i_clk            (i_clk                ),
        .i_resetn         (i_resetn             ),
        .i_busy           (o_busy_vector_q[3]   ),
        .i_we             (wire_i_we_3          ),
        .i_PC             (wire_i_PC_3          ),
        .i_src1_data      (wire_i_src1_data_3   ),
        .i_src2_data      (wire_i_src2_data_3   ),
        .i_src1_ready     (wire_i_src1_ready_3  ),
        .i_src2_ready     (wire_i_src2_ready_3  ),
        .i_imm            (wire_i_imm_3         ),
        .i_rename_rd      (wire_i_rename_rd_3   ),
        .i_rd_valid       (wire_i_rd_valid_3    ),
        .i_src1_sel       (wire_i_src1_sel_3    ),
        .i_src2_sel       (wire_i_src2_sel_3    ),
        .i_alu_op         (wire_i_alu_op_3      ),
        .i_spectag        (wire_i_spectag_3     ),

        .o_PC_q           (wire_o_PC_3          ),
        .o_imm_q          (wire_o_imm_3         ),
        .o_rrftag_q       (wire_o_rrftag_3      ),
        .o_rrftag_valid_q (wire_o_rrftag_valid_3),
        .o_src1_sel_q     (wire_o_src1_sel_3    ),
        .o_src2_sel_q     (wire_o_src2_sel_3    ),
        .o_alu_op_q       (wire_o_alu_op_3      ),
        .o_spectag_q      (wire_o_spectag_3     ),

        .o_src1_data      (wire_o_src1_data_3   ),
        .o_src2_data      (wire_o_src2_data_3   ),
        .o_data_ready     (wire_o_data_ready_3  ),
        
        .i_exe_result1    (i_exe_result1        ),
        .i_exe_rrftag1    (i_exe_rrftag1        ),
        .i_flush_spec1    (i_flush_spec1        ),
        .i_exe_result2    (i_exe_result2        ),
        .i_exe_rrftag2    (i_exe_rrftag2        ),
        .i_flush_spec2    (i_flush_spec2        ),
        .i_exe_result3    (i_exe_result3        ),
        .i_exe_rrftag3    (i_exe_rrftag3        ),
        .i_flush_spec3    (i_flush_spec3        ),
        .i_exe_result4    (i_exe_result4        ),
        .i_exe_rrftag4    (i_exe_rrftag4        ),
        .i_flush_spec4    (i_flush_spec4        ),
        .i_exe_result5    (i_exe_result5        ),
        .i_exe_rrftag5    (i_exe_rrftag5        ),
        .i_flush_spec5    (i_flush_spec5        )
    );

    wire                  wire_dispatch1_en_4   = i_dispatch1_en && (i_dispatch1_addr == 3'b100);
    wire                  wire_dispatch2_en_4   = i_dispatch2_en && (i_dispatch2_addr == 3'b100);
    wire                  wire_i_we_4           = wire_dispatch1_en_4 || wire_dispatch2_en_4;
    wire [31:0]           wire_i_PC_4           = wire_dispatch1_en_4 ? i_PC_1         : i_PC_2;
    wire [31:0]           wire_i_src1_data_4    = wire_dispatch1_en_4 ? i_src1_data_1  : i_src1_data_2;
    wire [31:0]           wire_i_src2_data_4    = wire_dispatch1_en_4 ? i_src2_data_1  : i_src2_data_2;
    wire                  wire_i_src1_ready_4   = wire_dispatch1_en_4 ? i_src1_ready_1 : i_src1_ready_2;
    wire                  wire_i_src2_ready_4   = wire_dispatch1_en_4 ? i_src2_ready_1 : i_src2_ready_2;
    wire [31:0]           wire_i_imm_4          = wire_dispatch1_en_4 ? i_imm_1        : i_imm_2;
    wire [`RRF_ADDRW-1:0] wire_i_rename_rd_4    = wire_dispatch1_en_4 ? i_rename_rd_1  : i_rename_rd_2;
    wire                  wire_i_rd_valid_4     = wire_dispatch1_en_4 ? i_rd_valid_1   : i_rd_valid_2;
    wire [1:0]            wire_i_src1_sel_4     = wire_dispatch1_en_4 ? i_src1_sel_1   : i_src1_sel_2;
    wire [1:0]            wire_i_src2_sel_4     = wire_dispatch1_en_4 ? i_src2_sel_1   : i_src2_sel_2;
    wire [3:0]            wire_i_alu_op_4       = wire_dispatch1_en_4 ? i_alu_op_1     : i_alu_op_2;
    wire [4:0]            wire_i_spectag_4      = wire_dispatch1_en_4 ? i_spectag_1    : i_spectag_2;
    
    wire [31:0]           wire_o_PC_4;
    wire [31:0]           wire_o_imm_4;
    wire [`RRF_ADDRW-1:0] wire_o_rrftag_4;
    wire                  wire_o_rrftag_valid_4;
    wire [1:0]            wire_o_src1_sel_4;
    wire [1:0]            wire_o_src2_sel_4;
    wire [3:0]            wire_o_alu_op_4;
    wire [4:0]            wire_o_spectag_4;
    wire [31:0]           wire_o_src1_data_4;
    wire [31:0]           wire_o_src2_data_4;
    wire                  wire_o_data_ready_4;
    
    RS_ALU_Entry rs_alu_entry4
    (
        .i_clk            (i_clk                ),
        .i_resetn         (i_resetn             ),
        .i_busy           (o_busy_vector_q[4]   ),
        .i_we             (wire_i_we_4          ),
        .i_PC             (wire_i_PC_4          ),
        .i_src1_data      (wire_i_src1_data_4   ),
        .i_src2_data      (wire_i_src2_data_4   ),
        .i_src1_ready     (wire_i_src1_ready_4  ),
        .i_src2_ready     (wire_i_src2_ready_4  ),
        .i_imm            (wire_i_imm_4         ),
        .i_rename_rd      (wire_i_rename_rd_4   ),
        .i_rd_valid       (wire_i_rd_valid_4    ),
        .i_src1_sel       (wire_i_src1_sel_4    ),
        .i_src2_sel       (wire_i_src2_sel_4    ),
        .i_alu_op         (wire_i_alu_op_4      ),
        .i_spectag        (wire_i_spectag_4     ),

        .o_PC_q           (wire_o_PC_4          ),
        .o_imm_q          (wire_o_imm_4         ),
        .o_rrftag_q       (wire_o_rrftag_4      ),
        .o_rrftag_valid_q (wire_o_rrftag_valid_4),
        .o_src1_sel_q     (wire_o_src1_sel_4    ),
        .o_src2_sel_q     (wire_o_src2_sel_4    ),
        .o_alu_op_q       (wire_o_alu_op_4      ),
        .o_spectag_q      (wire_o_spectag_4     ),

        .o_src1_data      (wire_o_src1_data_4   ),
        .o_src2_data      (wire_o_src2_data_4   ),
        .o_data_ready     (wire_o_data_ready_4  ),
        
        .i_exe_result1    (i_exe_result1        ),
        .i_exe_rrftag1    (i_exe_rrftag1        ),
        .i_flush_spec1    (i_flush_spec1        ),
        .i_exe_result2    (i_exe_result2        ),
        .i_exe_rrftag2    (i_exe_rrftag2        ),
        .i_flush_spec2    (i_flush_spec2        ),
        .i_exe_result3    (i_exe_result3        ),
        .i_exe_rrftag3    (i_exe_rrftag3        ),
        .i_flush_spec3    (i_flush_spec3        ),
        .i_exe_result4    (i_exe_result4        ),
        .i_exe_rrftag4    (i_exe_rrftag4        ),
        .i_flush_spec4    (i_flush_spec4        ),
        .i_exe_result5    (i_exe_result5        ),
        .i_exe_rrftag5    (i_exe_rrftag5        ),
        .i_flush_spec5    (i_flush_spec5        )
    );

    wire                  wire_dispatch1_en_5   = i_dispatch1_en && (i_dispatch1_addr == 3'b101);
    wire                  wire_dispatch2_en_5   = i_dispatch2_en && (i_dispatch2_addr == 3'b101);
    wire                  wire_i_we_5           = wire_dispatch1_en_5 || wire_dispatch2_en_5;
    wire [31:0]           wire_i_PC_5           = wire_dispatch1_en_5 ? i_PC_1         : i_PC_2;
    wire [31:0]           wire_i_src1_data_5    = wire_dispatch1_en_5 ? i_src1_data_1  : i_src1_data_2;
    wire [31:0]           wire_i_src2_data_5    = wire_dispatch1_en_5 ? i_src2_data_1  : i_src2_data_2;
    wire                  wire_i_src1_ready_5   = wire_dispatch1_en_5 ? i_src1_ready_1 : i_src1_ready_2;
    wire                  wire_i_src2_ready_5   = wire_dispatch1_en_5 ? i_src2_ready_1 : i_src2_ready_2;
    wire [31:0]           wire_i_imm_5          = wire_dispatch1_en_5 ? i_imm_1        : i_imm_2;
    wire [`RRF_ADDRW-1:0] wire_i_rename_rd_5    = wire_dispatch1_en_5 ? i_rename_rd_1  : i_rename_rd_2;
    wire                  wire_i_rd_valid_5     = wire_dispatch1_en_5 ? i_rd_valid_1   : i_rd_valid_2;
    wire [1:0]            wire_i_src1_sel_5     = wire_dispatch1_en_5 ? i_src1_sel_1   : i_src1_sel_2;
    wire [1:0]            wire_i_src2_sel_5     = wire_dispatch1_en_5 ? i_src2_sel_1   : i_src2_sel_2;
    wire [3:0]            wire_i_alu_op_5       = wire_dispatch1_en_5 ? i_alu_op_1     : i_alu_op_2;
    wire [4:0]            wire_i_spectag_5      = wire_dispatch1_en_5 ? i_spectag_1    : i_spectag_2;
    
    wire [31:0]           wire_o_PC_5;
    wire [31:0]           wire_o_imm_5;
    wire [`RRF_ADDRW-1:0] wire_o_rrftag_5;
    wire                  wire_o_rrftag_valid_5;
    wire [1:0]            wire_o_src1_sel_5;
    wire [1:0]            wire_o_src2_sel_5;
    wire [3:0]            wire_o_alu_op_5;
    wire [4:0]            wire_o_spectag_5;
    wire [31:0]           wire_o_src1_data_5;
    wire [31:0]           wire_o_src2_data_5;
    wire                  wire_o_data_ready_5;
    
    RS_ALU_Entry rs_alu_entry5
    (
        .i_clk            (i_clk                ),
        .i_resetn         (i_resetn             ),
        .i_busy           (o_busy_vector_q[5]   ),
        .i_we             (wire_i_we_5          ),
        .i_PC             (wire_i_PC_5          ),
        .i_src1_data      (wire_i_src1_data_5   ),
        .i_src2_data      (wire_i_src2_data_5   ),
        .i_src1_ready     (wire_i_src1_ready_5  ),
        .i_src2_ready     (wire_i_src2_ready_5  ),
        .i_imm            (wire_i_imm_5         ),
        .i_rename_rd      (wire_i_rename_rd_5   ),
        .i_rd_valid       (wire_i_rd_valid_5    ),
        .i_src1_sel       (wire_i_src1_sel_5    ),
        .i_src2_sel       (wire_i_src2_sel_5    ),
        .i_alu_op         (wire_i_alu_op_5      ),
        .i_spectag        (wire_i_spectag_5     ),

        .o_PC_q           (wire_o_PC_5          ),
        .o_imm_q          (wire_o_imm_5         ),
        .o_rrftag_q       (wire_o_rrftag_5      ),
        .o_rrftag_valid_q (wire_o_rrftag_valid_5),
        .o_src1_sel_q     (wire_o_src1_sel_5    ),
        .o_src2_sel_q     (wire_o_src2_sel_5    ),
        .o_alu_op_q       (wire_o_alu_op_5      ),
        .o_spectag_q      (wire_o_spectag_5     ),

        .o_src1_data      (wire_o_src1_data_5   ),
        .o_src2_data      (wire_o_src2_data_5   ),
        .o_data_ready     (wire_o_data_ready_5  ),
        
        .i_exe_result1    (i_exe_result1        ),
        .i_exe_rrftag1    (i_exe_rrftag1        ),
        .i_flush_spec1    (i_flush_spec1        ),
        .i_exe_result2    (i_exe_result2        ),
        .i_exe_rrftag2    (i_exe_rrftag2        ),
        .i_flush_spec2    (i_flush_spec2        ),
        .i_exe_result3    (i_exe_result3        ),
        .i_exe_rrftag3    (i_exe_rrftag3        ),
        .i_flush_spec3    (i_flush_spec3        ),
        .i_exe_result4    (i_exe_result4        ),
        .i_exe_rrftag4    (i_exe_rrftag4        ),
        .i_flush_spec4    (i_flush_spec4        ),
        .i_exe_result5    (i_exe_result5        ),
        .i_exe_rrftag5    (i_exe_rrftag5        ),
        .i_flush_spec5    (i_flush_spec5        )
    );

    wire                  wire_dispatch1_en_6   = i_dispatch1_en && (i_dispatch1_addr == 3'b110);
    wire                  wire_dispatch2_en_6   = i_dispatch2_en && (i_dispatch2_addr == 3'b110);
    wire                  wire_i_we_6           = wire_dispatch1_en_6 || wire_dispatch2_en_6;
    wire [31:0]           wire_i_PC_6           = wire_dispatch1_en_6 ? i_PC_1         : i_PC_2;
    wire [31:0]           wire_i_src1_data_6    = wire_dispatch1_en_6 ? i_src1_data_1  : i_src1_data_2;
    wire [31:0]           wire_i_src2_data_6    = wire_dispatch1_en_6 ? i_src2_data_1  : i_src2_data_2;
    wire                  wire_i_src1_ready_6   = wire_dispatch1_en_6 ? i_src1_ready_1 : i_src1_ready_2;
    wire                  wire_i_src2_ready_6   = wire_dispatch1_en_6 ? i_src2_ready_1 : i_src2_ready_2;
    wire [31:0]           wire_i_imm_6          = wire_dispatch1_en_6 ? i_imm_1        : i_imm_2;
    wire [`RRF_ADDRW-1:0] wire_i_rename_rd_6    = wire_dispatch1_en_6 ? i_rename_rd_1  : i_rename_rd_2;
    wire                  wire_i_rd_valid_6     = wire_dispatch1_en_6 ? i_rd_valid_1   : i_rd_valid_2;
    wire [1:0]            wire_i_src1_sel_6     = wire_dispatch1_en_6 ? i_src1_sel_1   : i_src1_sel_2;
    wire [1:0]            wire_i_src2_sel_6     = wire_dispatch1_en_6 ? i_src2_sel_1   : i_src2_sel_2;
    wire [3:0]            wire_i_alu_op_6       = wire_dispatch1_en_6 ? i_alu_op_1     : i_alu_op_2;
    wire [4:0]            wire_i_spectag_6      = wire_dispatch1_en_6 ? i_spectag_1    : i_spectag_2;
    
    wire [31:0]           wire_o_PC_6;
    wire [31:0]           wire_o_imm_6;
    wire [`RRF_ADDRW-1:0] wire_o_rrftag_6;
    wire                  wire_o_rrftag_valid_6;
    wire [1:0]            wire_o_src1_sel_6;
    wire [1:0]            wire_o_src2_sel_6;
    wire [3:0]            wire_o_alu_op_6;
    wire [4:0]            wire_o_spectag_6;
    wire [31:0]           wire_o_src1_data_6;
    wire [31:0]           wire_o_src2_data_6;
    wire                  wire_o_data_ready_6;
    
    RS_ALU_Entry rs_alu_entry6
    (
        .i_clk            (i_clk                ),
        .i_resetn         (i_resetn             ),
        .i_busy           (o_busy_vector_q[6]   ),
        .i_we             (wire_i_we_6          ),
        .i_PC             (wire_i_PC_6          ),
        .i_src1_data      (wire_i_src1_data_6   ),
        .i_src2_data      (wire_i_src2_data_6   ),
        .i_src1_ready     (wire_i_src1_ready_6  ),
        .i_src2_ready     (wire_i_src2_ready_6  ),
        .i_imm            (wire_i_imm_6         ),
        .i_rename_rd      (wire_i_rename_rd_6   ),
        .i_rd_valid       (wire_i_rd_valid_6    ),
        .i_src1_sel       (wire_i_src1_sel_6    ),
        .i_src2_sel       (wire_i_src2_sel_6    ),
        .i_alu_op         (wire_i_alu_op_6      ),
        .i_spectag        (wire_i_spectag_6     ),

        .o_PC_q           (wire_o_PC_6          ),
        .o_imm_q          (wire_o_imm_6         ),
        .o_rrftag_q       (wire_o_rrftag_6      ),
        .o_rrftag_valid_q (wire_o_rrftag_valid_6),
        .o_src1_sel_q     (wire_o_src1_sel_6    ),
        .o_src2_sel_q     (wire_o_src2_sel_6    ),
        .o_alu_op_q       (wire_o_alu_op_6      ),
        .o_spectag_q      (wire_o_spectag_6     ),

        .o_src1_data      (wire_o_src1_data_6   ),
        .o_src2_data      (wire_o_src2_data_6   ),
        .o_data_ready     (wire_o_data_ready_6  ),
        
        .i_exe_result1    (i_exe_result1        ),
        .i_exe_rrftag1    (i_exe_rrftag1        ),
        .i_flush_spec1    (i_flush_spec1        ),
        .i_exe_result2    (i_exe_result2        ),
        .i_exe_rrftag2    (i_exe_rrftag2        ),
        .i_flush_spec2    (i_flush_spec2        ),
        .i_exe_result3    (i_exe_result3        ),
        .i_exe_rrftag3    (i_exe_rrftag3        ),
        .i_flush_spec3    (i_flush_spec3        ),
        .i_exe_result4    (i_exe_result4        ),
        .i_exe_rrftag4    (i_exe_rrftag4        ),
        .i_flush_spec4    (i_flush_spec4        ),
        .i_exe_result5    (i_exe_result5        ),
        .i_exe_rrftag5    (i_exe_rrftag5        ),
        .i_flush_spec5    (i_flush_spec5        )
    );

    wire                  wire_dispatch1_en_7   = i_dispatch1_en && (i_dispatch1_addr == 3'b111);
    wire                  wire_dispatch2_en_7   = i_dispatch2_en && (i_dispatch2_addr == 3'b111);
    wire                  wire_i_we_7           = wire_dispatch1_en_7 || wire_dispatch2_en_7;
    wire [31:0]           wire_i_PC_7           = wire_dispatch1_en_7 ? i_PC_1         : i_PC_2;
    wire [31:0]           wire_i_src1_data_7    = wire_dispatch1_en_7 ? i_src1_data_1  : i_src1_data_2;
    wire [31:0]           wire_i_src2_data_7    = wire_dispatch1_en_7 ? i_src2_data_1  : i_src2_data_2;
    wire                  wire_i_src1_ready_7   = wire_dispatch1_en_7 ? i_src1_ready_1 : i_src1_ready_2;
    wire                  wire_i_src2_ready_7   = wire_dispatch1_en_7 ? i_src2_ready_1 : i_src2_ready_2;
    wire [31:0]           wire_i_imm_7          = wire_dispatch1_en_7 ? i_imm_1        : i_imm_2;
    wire [`RRF_ADDRW-1:0] wire_i_rename_rd_7    = wire_dispatch1_en_7 ? i_rename_rd_1  : i_rename_rd_2;
    wire                  wire_i_rd_valid_7     = wire_dispatch1_en_7 ? i_rd_valid_1   : i_rd_valid_2;
    wire [1:0]            wire_i_src1_sel_7     = wire_dispatch1_en_7 ? i_src1_sel_1   : i_src1_sel_2;
    wire [1:0]            wire_i_src2_sel_7     = wire_dispatch1_en_7 ? i_src2_sel_1   : i_src2_sel_2;
    wire [3:0]            wire_i_alu_op_7       = wire_dispatch1_en_7 ? i_alu_op_1     : i_alu_op_2;
    wire [4:0]            wire_i_spectag_7      = wire_dispatch1_en_7 ? i_spectag_1    : i_spectag_2;
    
    wire [31:0]           wire_o_PC_7;
    wire [31:0]           wire_o_imm_7;
    wire [`RRF_ADDRW-1:0] wire_o_rrftag_7;
    wire                  wire_o_rrftag_valid_7;
    wire [1:0]            wire_o_src1_sel_7;
    wire [1:0]            wire_o_src2_sel_7;
    wire [3:0]            wire_o_alu_op_7;
    wire [4:0]            wire_o_spectag_7;
    wire [31:0]           wire_o_src1_data_7;
    wire [31:0]           wire_o_src2_data_7;
    wire                  wire_o_data_ready_7;
    
    RS_ALU_Entry rs_alu_entry7
    (
        .i_clk            (i_clk                ),
        .i_resetn         (i_resetn             ),
        .i_busy           (o_busy_vector_q[7]   ),
        .i_we             (wire_i_we_7          ),
        .i_PC             (wire_i_PC_7          ),
        .i_src1_data      (wire_i_src1_data_7   ),
        .i_src2_data      (wire_i_src2_data_7   ),
        .i_src1_ready     (wire_i_src1_ready_7  ),
        .i_src2_ready     (wire_i_src2_ready_7  ),
        .i_imm            (wire_i_imm_7         ),
        .i_rename_rd      (wire_i_rename_rd_7   ),
        .i_rd_valid       (wire_i_rd_valid_7    ),
        .i_src1_sel       (wire_i_src1_sel_7    ),
        .i_src2_sel       (wire_i_src2_sel_7    ),
        .i_alu_op         (wire_i_alu_op_7      ),
        .i_spectag        (wire_i_spectag_7     ),

        .o_PC_q           (wire_o_PC_7          ),
        .o_imm_q          (wire_o_imm_7         ),
        .o_rrftag_q       (wire_o_rrftag_7      ),
        .o_rrftag_valid_q (wire_o_rrftag_valid_7),
        .o_src1_sel_q     (wire_o_src1_sel_7    ),
        .o_src2_sel_q     (wire_o_src2_sel_7    ),
        .o_alu_op_q       (wire_o_alu_op_7      ),
        .o_spectag_q      (wire_o_spectag_7     ),

        .o_src1_data      (wire_o_src1_data_7   ),
        .o_src2_data      (wire_o_src2_data_7   ),
        .o_data_ready     (wire_o_data_ready_7  ),
        
        .i_exe_result1    (i_exe_result1        ),
        .i_exe_rrftag1    (i_exe_rrftag1        ),
        .i_flush_spec1    (i_flush_spec1        ),
        .i_exe_result2    (i_exe_result2        ),
        .i_exe_rrftag2    (i_exe_rrftag2        ),
        .i_flush_spec2    (i_flush_spec2        ),
        .i_exe_result3    (i_exe_result3        ),
        .i_exe_rrftag3    (i_exe_rrftag3        ),
        .i_flush_spec3    (i_flush_spec3        ),
        .i_exe_result4    (i_exe_result4        ),
        .i_exe_rrftag4    (i_exe_rrftag4        ),
        .i_flush_spec4    (i_flush_spec4        ),
        .i_exe_result5    (i_exe_result5        ),
        .i_exe_rrftag5    (i_exe_rrftag5        ),
        .i_flush_spec5    (i_flush_spec5        )
    );

   
    assign o_src1_data    = (i_issue_addr == 0) ? wire_o_src1_data_0 :
                            (i_issue_addr == 1) ? wire_o_src1_data_1 :
                            (i_issue_addr == 2) ? wire_o_src1_data_2 :
                            (i_issue_addr == 3) ? wire_o_src1_data_3 :
                            (i_issue_addr == 4) ? wire_o_src1_data_4 :
                            (i_issue_addr == 5) ? wire_o_src1_data_5 :
                            (i_issue_addr == 6) ? wire_o_src1_data_6 : 
                                                  wire_o_src1_data_7 ;

    assign o_src2_data    = (i_issue_addr == 0) ? wire_o_src2_data_0 :
                            (i_issue_addr == 1) ? wire_o_src2_data_1 :
                            (i_issue_addr == 2) ? wire_o_src2_data_2 :
                            (i_issue_addr == 3) ? wire_o_src2_data_3 :
                            (i_issue_addr == 4) ? wire_o_src2_data_4 :
                            (i_issue_addr == 5) ? wire_o_src2_data_5 :
                            (i_issue_addr == 6) ? wire_o_src2_data_6 : 
                                                  wire_o_src2_data_7 ;

    assign o_PC           = (i_issue_addr == 0) ? wire_o_PC_0 :
                            (i_issue_addr == 1) ? wire_o_PC_1 :
                            (i_issue_addr == 2) ? wire_o_PC_2 :
                            (i_issue_addr == 3) ? wire_o_PC_3 :
                            (i_issue_addr == 4) ? wire_o_PC_4 :
                            (i_issue_addr == 5) ? wire_o_PC_5 :
                            (i_issue_addr == 6) ? wire_o_PC_6 : 
                                                  wire_o_PC_7 ;
   
    assign o_imm          = (i_issue_addr == 0) ? wire_o_imm_0 :
                            (i_issue_addr == 1) ? wire_o_imm_1 :
                            (i_issue_addr == 2) ? wire_o_imm_2 :
                            (i_issue_addr == 3) ? wire_o_imm_3 :
                            (i_issue_addr == 4) ? wire_o_imm_4 :
                            (i_issue_addr == 5) ? wire_o_imm_5 :
                            (i_issue_addr == 6) ? wire_o_imm_6 : 
                                                  wire_o_imm_7 ;
   
    assign o_rrftag       = (i_issue_addr == 0) ? wire_o_rrftag_0 :
                            (i_issue_addr == 1) ? wire_o_rrftag_0 :
                            (i_issue_addr == 2) ? wire_o_rrftag_0 :
                            (i_issue_addr == 3) ? wire_o_rrftag_0 :
                            (i_issue_addr == 4) ? wire_o_rrftag_0 :
                            (i_issue_addr == 5) ? wire_o_rrftag_0 :
                            (i_issue_addr == 6) ? wire_o_rrftag_0 : 
                                                  wire_o_rrftag_0 ;
   
    assign o_rrftag_valid_q = (i_issue_addr == 0) ? wire_o_rrftag_valid_0 :
                            (i_issue_addr == 1) ? wire_o_rrftag_valid_1 :
                            (i_issue_addr == 2) ? wire_o_rrftag_valid_2 :
                            (i_issue_addr == 3) ? wire_o_rrftag_valid_3 :
                            (i_issue_addr == 4) ? wire_o_rrftag_valid_4 :
                            (i_issue_addr == 5) ? wire_o_rrftag_valid_5 :
                            (i_issue_addr == 6) ? wire_o_rrftag_valid_6 : 
                                                  wire_o_rrftag_valid_7 ;
   
    assign o_src1_sel     = (i_issue_addr == 0) ? wire_o_src1_sel_0 :
                            (i_issue_addr == 1) ? wire_o_src1_sel_1 :
                            (i_issue_addr == 2) ? wire_o_src1_sel_2 :
                            (i_issue_addr == 3) ? wire_o_src1_sel_3 :
                            (i_issue_addr == 4) ? wire_o_src1_sel_4 :
                            (i_issue_addr == 5) ? wire_o_src1_sel_5 :
                            (i_issue_addr == 6) ? wire_o_src1_sel_6 : 
                                                  wire_o_src1_sel_7 ;
   
    assign o_src2_sel     = (i_issue_addr == 0) ? wire_o_src2_sel_0 :
                            (i_issue_addr == 1) ? wire_o_src2_sel_1 :
                            (i_issue_addr == 2) ? wire_o_src2_sel_2 :
                            (i_issue_addr == 3) ? wire_o_src2_sel_3 :
                            (i_issue_addr == 4) ? wire_o_src2_sel_4 :
                            (i_issue_addr == 5) ? wire_o_src2_sel_5 :
                            (i_issue_addr == 6) ? wire_o_src2_sel_6 : 
                                                  wire_o_src2_sel_7 ;
   
    assign o_alu_op       = (i_issue_addr == 0) ? wire_o_alu_op_0 :
                            (i_issue_addr == 1) ? wire_o_alu_op_1 :
                            (i_issue_addr == 2) ? wire_o_alu_op_2 :
                            (i_issue_addr == 3) ? wire_o_alu_op_3 :
                            (i_issue_addr == 4) ? wire_o_alu_op_4 :
                            (i_issue_addr == 5) ? wire_o_alu_op_5 :
                            (i_issue_addr == 6) ? wire_o_alu_op_6 : 
                                                  wire_o_alu_op_7 ;
   
    assign o_spectag_q      = (i_issue_addr == 0) ? wire_o_spectag_0 :
                            (i_issue_addr == 1) ? wire_o_spectag_1 :
                            (i_issue_addr == 2) ? wire_o_spectag_2 :
                            (i_issue_addr == 3) ? wire_o_spectag_3 :
                            (i_issue_addr == 4) ? wire_o_spectag_4 :
                            (i_issue_addr == 5) ? wire_o_spectag_5 :
                            (i_issue_addr == 6) ? wire_o_spectag_6 : 
                                                  wire_o_spectag_7 ;
endmodule //RS_ALU








module search_begin
#(
    parameter ENTSEL = 2,
    parameter ENTNUM = 4
)
(
    input      [ENTNUM-1:0] in,
    output reg [ENTSEL-1:0] out,
    output reg              en
);

    integer i;
    always @ (*) begin
        out = 0;
        en  = 0;
        for (i = ENTNUM-1; i >= 0 ; i = i-1) begin
            if (in[i]) begin
                out = i;
                en  = 1;
            end
        end
    end
endmodule //search_from_top


module search_end
#(
    parameter ENTSEL = 2,
    parameter ENTNUM = 4
)
(
    input      [ENTNUM-1:0] in,
    output reg [ENTSEL-1:0] out,
    output reg              en
);

    integer i;
    always @ (*) begin
        out = 0;
        en = 0;
        for (i = 0 ; i < ENTNUM ; i = i + 1) begin
            if (in[i]) begin
                out = i;
                en  = 1;
            end
        end
    end
endmodule // search_from_bottom








module alloc_issue_ino
#(
    parameter ENTSEL = 2,
    parameter ENTNUM = 4
)
(
    input                   i_clk,
    input                   i_resetn,
    input  [1:0]            i_reqNum,
    input  [ENTNUM-1:0]     i_busy_vector,
    input  [ENTNUM-1:0]     prbusyvec_next,
    input  [ENTNUM-1:0]     i_ready_vector,
    input                   i_predict_miss,
    input                   exunit_busynext,
    input                   i_stall_dispatch,
    input                   i_flush_dispatch,
    output reg [ENTSEL-1:0] o_allocate_ptr_q,
    output                  o_allocatable,
    output     [ENTSEL-1:0] o_issue_ptr,
    output                  o_issue_valid
);
    wire [ENTSEL-1:0]       allocptr2 = o_allocate_ptr_q + 1;
    wire [ENTSEL-1:0]       b0;
    wire [ENTSEL-1:0]       e0;
    wire [ENTSEL-1:0]       b1;
    wire [ENTSEL-1:0]       e1;
    wire                    notfull;

    wire [ENTSEL-1:0]       ne1;
    wire [ENTSEL-1:0]       nb0;
    wire [ENTSEL-1:0]       nb1;
    wire                    notfull_next;
   
    search_begin #(ENTSEL, ENTNUM) sb1
    (
        .in(i_busy_vector),
        .out(b1),
        .en()
    );
   
    search_end #(ENTSEL, ENTNUM) se1
    (
        .in(i_busy_vector),
        .out(e1),
        .en()
    );

    search_end #(ENTSEL, ENTNUM) se0
    (
        .in(~i_busy_vector),
        .out(e0),
        .en(notfull)
    );

    search_begin #(ENTSEL, ENTNUM) snb1
    (
        .in(prbusyvec_next),
        .out(nb1),
        .en()
    );
   
    search_end #(ENTSEL, ENTNUM) sne1
    (
        .in(prbusyvec_next),
        .out(ne1),
        .en()
    );

    search_begin #(ENTSEL, ENTNUM) snb0
    (
        .in(~prbusyvec_next),
        .out(nb0),
        .en(notfull_next)
    );

    assign o_issue_ptr = ~notfull                        ? o_allocate_ptr_q :
                      ((b1 == 0) && (e1 == ENTNUM-1)) ? (e0+1) : b1;
   
    assign o_issue_valid = i_ready_vector[o_issue_ptr] & ~i_predict_miss & ~exunit_busynext;

    assign o_allocatable = (i_reqNum == 2'h0) ? 1'b1 :
                         (i_reqNum == 2'h1) ? ((~i_busy_vector[o_allocate_ptr_q] ? 1'b1 : 1'b0)) :
                                            ((~i_busy_vector[o_allocate_ptr_q] && ~i_busy_vector[allocptr2]) ? 1'b1 : 1'b0);
   
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_allocate_ptr_q <= 0;
        end else if (i_predict_miss) begin
            o_allocate_ptr_q <= ~notfull_next ? o_allocate_ptr_q : (((nb1 == 0) && (ne1 == ENTNUM-1)) ? nb0 : (ne1+1));
        end else if (~i_stall_dispatch && ~i_flush_dispatch) begin
            o_allocate_ptr_q <= o_allocate_ptr_q + i_reqNum;
        end
    end
endmodule //alloc_issue_ino





module RS_LDST_Entry
(
    input                       i_clk,
    input                       i_resetn,
    
    input                       i_busy,
    input      [31:0]           i_PC,
    input      [31:0]           i_src1_data,
    input      [31:0]           i_src2_data,
    input                       i_src1_ready,
    input                       i_src2_ready,
    input      [31:0]           i_imm,
    input      [`RRF_ADDRW-1:0] i_rename_rd,
    input                       i_rd_valid,
    input      [4:0]            i_spectag,
    input                       i_we,
    
    output reg [31:0]           o_src1_data_q,
    output reg [31:0]           o_src2_data_q,
    output reg                  o_data_ready_q,
    output reg [31:0]           o_PC_q,
    output reg [31:0]           o_imm_q,
    output reg [`RRF_ADDRW-1:0] o_rrftag_q,
    output reg                  o_rrftag_valid_q,
    output reg [4:0]            o_spectag_q,
    
    //
    input      [31:0]           i_exe_result1,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag1,
    input                       i_flush_spec1,
    input      [31:0]           i_exe_result2,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag2,
    input                       i_flush_spec2,
    input      [31:0]           i_exe_result3,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag3,
    input                       i_flush_spec3,
    input      [31:0]           i_exe_result4,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag4,
    input                       i_flush_spec4,
    input      [31:0]           i_exe_result5,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag5,
    input                       i_flush_spec5
);

    wire [31:0] wire_src1_data;
    wire [31:0] wire_src2_data;
    wire        wire_src1_ready;
    wire        wire_src2_ready;
	
    wire wire_data_ready = i_busy & wire_src1_ready & wire_src2_ready;
   
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_PC_q           <= 32'b0;
            o_imm_q          <= 32'b0;
            o_rrftag_q       <= `RRF_ADDRW'b0;
            o_rrftag_valid_q <= 1'b0;
            o_spectag_q      <= 5'b0;

            o_src1_data_q    <= 32'b0;
            o_src2_data_q    <= 32'b0;
            o_data_ready_q   <= 1'b0;
        end else if (i_we) begin
            o_PC_q           <= i_PC;
            o_imm_q          <= i_imm;
            o_rrftag_q       <= i_rename_rd;
            o_rrftag_valid_q <= i_rd_valid;
            o_spectag_q      <= i_spectag;

            o_src1_data_q    <= wire_src1_data;
            o_src2_data_q    <= wire_src2_data;
            o_data_ready_q   <= wire_data_ready;
        end // if (i_we)
    end
   
    RAW_Resolve rs_ldst_raw_resolve1
    (
        .i_mux_data    (i_src1_data    ),
        .i_mux_ready   (i_src1_ready   ),
        .i_exe_result1 (i_exe_result1  ),
        .i_exe_rrftag1 (i_exe_rrftag1  ),
        .i_flush_spec1 (i_flush_spec1  ),
        .i_exe_result2 (i_exe_result2  ),
        .i_exe_rrftag2 (i_exe_rrftag2  ),
        .i_flush_spec2 (i_flush_spec2  ),
        .i_exe_result3 (i_exe_result3  ),
        .i_exe_rrftag3 (i_exe_rrftag3  ),
        .i_flush_spec3 (i_flush_spec3  ),
        .i_exe_result4 (i_exe_result4  ),
        .i_exe_rrftag4 (i_exe_rrftag4  ),
        .i_flush_spec4 (i_flush_spec4  ),
        .i_exe_result5 (i_exe_result5  ),
        .i_exe_rrftag5 (i_exe_rrftag5  ),
        .i_flush_spec5 (i_flush_spec5  ),
        .o_src_data    (wire_src1_data ),
        .o_src_ready   (wire_src1_ready)
    );

    RAW_Resolve rs_ldst_raw_resolve2
    (
        .i_mux_data    (i_src2_data    ),
        .i_mux_ready   (i_src1_ready   ),
        .i_exe_result1 (i_exe_result1  ),
        .i_exe_rrftag1 (i_exe_rrftag1  ),
        .i_flush_spec1 (i_flush_spec1  ),
        .i_exe_result2 (i_exe_result2  ),
        .i_exe_rrftag2 (i_exe_rrftag2  ),
        .i_flush_spec2 (i_flush_spec2  ),
        .i_exe_result3 (i_exe_result3  ),
        .i_exe_rrftag3 (i_exe_rrftag3  ),
        .i_flush_spec3 (i_flush_spec3  ),
        .i_exe_result4 (i_exe_result4  ),
        .i_exe_rrftag4 (i_exe_rrftag4  ),
        .i_flush_spec4 (i_flush_spec4  ),
        .i_exe_result5 (i_exe_result5  ),
        .i_exe_rrftag5 (i_exe_rrftag5  ),
        .i_flush_spec5 (i_flush_spec5  ),
        .o_src_data    (wire_src2_data ),
        .o_src_ready   (wire_src2_ready)
    );
   
endmodule //RS_LDST_Entry


module RS_LdSt
(
    input                       i_clk,
    input                       i_resetn,
	
    output reg [3:0] 			o_busy_vector_q,
    input                       i_predict_miss,
    input                       i_predict_hit,
    input      [4:0]  			i_predict_tag,
    input      [4:0]  			i_specfixtag,
    output     [3:0] 			prbusyvec_next,
    
	//dispatch and issue signal
    input                       i_issue_en,
    input      [1:0] 			i_issue_addr,
    input                       i_dispatch1_en,
    input                       i_dispatch2_en,
    input      [1:0] 			i_dispatch1_addr,
    input      [1:0] 			i_dispatch2_addr,
	
    //input 1
    input      [31:0]         	i_PC_1,
    input      [31:0]        	i_src1_data_1,
    input      [31:0]         	i_src2_data_1,
    input                       i_src1_ready_1,
    input                       i_src2_ready_1,
    input      [31:0]         	i_imm_1,
    input      [`RRF_ADDRW-1:0] i_rename_rd_1,
    input                       i_rd_valid_1,
    input      [4:0]  			i_spectag_1,
    input                       i_spectag_valid_1,
    
	//input 2
    input      [31:0]         	i_PC_2,
    input      [31:0]         	i_src1_data_2,
    input      [31:0]         	i_src2_data_2,
    input                       i_src1_ready_2,
    input                       i_src2_ready_2,
    input      [31:0]           i_imm_2,
    input      [`RRF_ADDRW-1:0] i_rename_rd_2,
    input                       i_rd_valid_2,
    input      [4:0]  			i_spectag_2,
    input                       i_spectag_valid_2,

    //output
    output     [31:0]         	o_src1_data,
    output     [31:0]         	o_src2_data,
    output     [3:0] 			o_data_ready,
    output     [31:0]           o_PC,
    output     [31:0]           o_imm,
    output     [`RRF_ADDRW-1:0] o_rrftag,
    output                      o_rrftag_valid,
    output     [4:0]  			o_spectag,
    output                      o_spectag_valid,
  
   //EXRSLT
    input      [31:0]           i_exe_result1,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag1,
    input                       i_flush_spec1,
    input      [31:0]           i_exe_result2,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag2,
    input                       i_flush_spec2,
    input      [31:0]           i_exe_result3,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag3,
    input                       i_flush_spec3,
    input      [31:0]           i_exe_result4,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag4,
    input                       i_flush_spec4,
    input      [31:0]           i_exe_result5,
    input      [`RRF_ADDRW-1:0] i_exe_rrftag5,
    input                       i_flush_spec5
);

    //_0
    wire [31:0]        wire_o_src1_data_0;
    wire [31:0]        wire_o_src2_data_0;
    wire                    wire_o_data_ready_0;
    wire [31:0]        wire_o_PC_0;
    wire [31:0]        wire_o_imm_0;
    wire [`RRF_ADDRW-1:0]     wire_o_rrftag_0;
    wire                    wire_o_rrftag_valid_0;
    wire [4:0] wire_o_spectag_0;
   //_1
    wire [31:0]        ex_src1_1;
    wire [31:0]        ex_src2_1;
    wire                    ready_1;
    wire [31:0]        pc_1;
    wire [31:0]        imm_1;
    wire [`RRF_ADDRW-1:0]     rrftag_1;
    wire                    dstval_1;
    wire [4:0] spectag_1;
   //_2
    wire [31:0]        ex_src1_2;
    wire [31:0]        ex_src2_2;
    wire                    ready_2;
    wire [31:0]        pc_2;
    wire [31:0]        imm_2;
    wire [`RRF_ADDRW-1:0]     rrftag_2;
    wire                    dstval_2;
    wire [4:0] spectag_2;
   //_3
    wire [31:0]        ex_src1_3;
    wire [31:0]        ex_src2_3;
    wire                    ready_3;
    wire [31:0]        pc_3;
    wire [31:0]        imm_3;
    wire [`RRF_ADDRW-1:0]     rrftag_3;
    wire                    dstval_3;
    wire [4:0] spectag_3;
   
    reg  [3:0] reg_spectag_valid_vector;

    //busy invalidation
    wire [3:0] wire_busy_set_vector = { (spectag_3 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                            (spectag_2 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                            (spectag_1 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                            (wire_o_spectag_0 & i_specfixtag) == 0 ? 1'b1 : 1'b0  };

    wire [3:0] wire_spectag_valid_set_vector = { (spectag_3 == i_predict_tag) ? 1'b0 : 1'b1,
                                                 (spectag_2 == i_predict_tag) ? 1'b0 : 1'b1,
                                                 (spectag_1 == i_predict_tag) ? 1'b0 : 1'b1,
                                                 (wire_o_spectag_0 == i_predict_tag) ? 1'b0 : 1'b1  };

    wire [3:0] wire_spectag_valid_vector_next = (wire_spectag_valid_set_vector & reg_spectag_valid_vector);
    
    assign o_spectag_valid        = i_predict_hit ? wire_spectag_valid_vector_next[i_issue_addr] : reg_spectag_valid_vector[i_issue_addr];
    assign o_data_ready          = {ready_3, ready_2, ready_1, wire_o_data_ready_0};
    assign prbusyvec_next = wire_busy_set_vector & o_busy_vector_q;
   
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_busy_vector_q    <= 0;
            reg_spectag_valid_vector <= 0;
        end else begin
            if (i_predict_miss) begin
                o_busy_vector_q    <= prbusyvec_next;
                reg_spectag_valid_vector <= 0;
            end else if (i_predict_hit) begin
                reg_spectag_valid_vector <= wire_spectag_valid_vector_next;
                if (i_issue_en) begin
                    o_busy_vector_q[i_issue_addr] <= 1'b0;
                end
            end else begin
                if (i_dispatch1_en) begin
                    o_busy_vector_q[i_dispatch1_addr]    <= 1'b1;
                    reg_spectag_valid_vector[i_dispatch1_addr] <= i_spectag_valid_1;
                end
                if (i_dispatch2_en) begin
                    o_busy_vector_q[i_dispatch2_addr]    <= 1'b1;
                    reg_spectag_valid_vector[i_dispatch2_addr] <= i_spectag_valid_2;
                end
                if (i_issue_en) begin
                    o_busy_vector_q[i_issue_addr] <= 1'b0;
                end
            end
        end
    end

    RS_LDST_Entry ent0
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .i_busy(o_busy_vector_q[0]),
        .i_PC((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_PC_1 : i_PC_2),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_imm((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_imm_1 : i_imm_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_spectag_1 : i_spectag_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 0)) || (i_dispatch2_en && (i_dispatch2_addr == 0))),
        .o_src1_data(wire_o_src1_data_0),
        .o_src2_data(wire_o_src2_data_0),
        .o_data_ready(wire_o_data_ready_0),
        .o_PC_q(wire_o_PC_0),
        .o_imm_q(wire_o_imm_0),
        .o_rrftag_q(wire_o_rrftag_0),
        .o_rrftag_valid_q(wire_o_rrftag_valid_0),
        .o_spectag_q(wire_o_spectag_0),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );

    RS_LDST_Entry entry1
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),        
        .i_busy(o_busy_vector_q[1]),
        .i_PC((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_PC_1 : i_PC_2),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_imm((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_imm_1 : i_imm_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_spectag_1 : i_spectag_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 1)) || (i_dispatch2_en && (i_dispatch2_addr == 1))),
        .o_src1_data(ex_src1_1),
        .o_src2_data(ex_src2_1),
        .o_data_ready(ready_1),
        .o_PC_q(pc_1),
        .o_imm_q(imm_1),
        .o_rrftag_q(rrftag_1),
        .o_rrftag_valid_q(dstval_1),
        .o_spectag_q(spectag_1),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );

    RS_LDST_Entry entry2
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),        
        .i_busy(o_busy_vector_q[2]),
        .i_PC((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_PC_1 : i_PC_2),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_imm((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_imm_1 : i_imm_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_spectag_1 : i_spectag_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 2)) || (i_dispatch2_en && (i_dispatch2_addr == 2))),
        .o_src1_data(ex_src1_2),
        .o_src2_data(ex_src2_2),
        .o_data_ready(ready_2),
        .o_PC_q(pc_2),
        .o_imm_q(imm_2),
        .o_rrftag_q(rrftag_2),
        .o_rrftag_valid_q(dstval_2),
        .o_spectag_q(spectag_2),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );

    RS_LDST_Entry ent3
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),        
        .i_busy(o_busy_vector_q[3]),
        .i_PC((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_PC_1 : i_PC_2),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_imm((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_imm_1 : i_imm_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_spectag_1 : i_spectag_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 3)) || (i_dispatch2_en && (i_dispatch2_addr == 3))),
        .o_src1_data(ex_src1_3),
        .o_src2_data(ex_src2_3),
        .o_data_ready(ready_3),
        .o_PC_q(pc_3),
        .o_imm_q(imm_3),
        .o_rrftag_q(rrftag_3),
        .o_rrftag_valid_q(dstval_3),
        .o_spectag_q(spectag_3),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );

   
    assign o_src1_data = (i_issue_addr == 0) ? wire_o_src1_data_0 :
                     (i_issue_addr == 1) ? ex_src1_1 :
                     (i_issue_addr == 2) ? ex_src1_2 : ex_src1_3;
    assign o_src2_data = (i_issue_addr == 0) ? wire_o_src2_data_0 :
                     (i_issue_addr == 1) ? ex_src2_1 :
                     (i_issue_addr == 2) ? ex_src2_2 : ex_src2_3;
    assign o_PC_q      = (i_issue_addr == 0) ? wire_o_PC_0 :
                     (i_issue_addr == 1) ? pc_1 :
                     (i_issue_addr == 2) ? pc_2 : pc_3;
    assign o_imm_q     = (i_issue_addr == 0) ? wire_o_imm_0 :
                     (i_issue_addr == 1) ? imm_1 :
                     (i_issue_addr == 2) ? imm_2 : imm_3;
    assign o_rrftag_q  = (i_issue_addr == 0) ? wire_o_rrftag_0 :
                     (i_issue_addr == 1) ? rrftag_1 :
                     (i_issue_addr == 2) ? rrftag_2 : rrftag_3;
    assign o_rrftag_valid_q  = (i_issue_addr == 0) ? wire_o_rrftag_valid_0 :
                     (i_issue_addr == 1) ? dstval_1 :
                     (i_issue_addr == 2) ? dstval_2 : dstval_3;
    assign o_spectag_q = (i_issue_addr == 0) ? wire_o_spectag_0 :
                     (i_issue_addr == 1) ? spectag_1 :
                     (i_issue_addr == 2) ? spectag_2 : spectag_3; 
endmodule //RS_LdSt





module RS_Branch_Ent
(
    //Memory
    input                          i_clk,
    input                          i_resetn,
    input                          i_busy,
    input      [31:0]         i_PC,
    input      [31:0]         i_src1_data,
    input      [31:0]         i_src2_data,
    input                          i_src1_ready,
    input                          i_src2_ready,
    input      [31:0]         i_imm,
    input      [`RRF_ADDRW-1:0]      i_rename_rd,
    input                          i_rd_valid,
    input      [3:0] i_alu_op,
    input      [4:0]  i_spectag,
    input      [`GSH_BHR_WIDTH-1:0]  wbhr,
    input                          wprcond,
    input      [31:0]         wpraddr,
    input      [6:0]               wopcode,
    input                          i_we,
    output     [31:0]         o_src1_data,
    output     [31:0]         o_src2_data,
    output                         o_data_ready,
    output reg [31:0]         o_PC_q,
    output reg [31:0]         o_imm_q,
    output reg [`RRF_ADDRW-1:0]      o_rrftag_q,
    output reg                     o_rrftag_valid_q,
    output reg [3:0] o_alu_op,
    output reg [4:0]  o_spectag_q,
    output reg [`GSH_BHR_WIDTH-1:0]  bhr,
    output reg                     prcond,
    output reg [31:0]         praddr,
    output reg [6:0]               opcode,
    //EXRSLT
    input      [31:0]         i_exe_result1,
    input      [`RRF_ADDRW-1:0]      i_exe_rrftag1,
    input                          i_flush_spec1,
    input      [31:0]         i_exe_result2,
    input      [`RRF_ADDRW-1:0]      i_exe_rrftag2,
    input                          i_flush_spec2,
    input      [31:0]         i_exe_result3,
    input      [`RRF_ADDRW-1:0]      i_exe_rrftag3,
    input                          i_flush_spec3,
    input      [31:0]         i_exe_result4,
    input      [`RRF_ADDRW-1:0]      i_exe_rrftag4,
    input                          i_flush_spec4,
    input      [31:0]         i_exe_result5,
    input      [`RRF_ADDRW-1:0]      i_exe_rrftag5,
    input                          i_flush_spec5
);

    reg  [31:0] wire_src1_data;
    reg  [31:0] wire_src2_data;
    reg              wire_src1_ready;
    reg              wire_src2_ready;

    wire [31:0] wire_next_src1_data;
    wire [31:0] wire_next_src2_data;   
    wire             wire_next_src1_ready;
    wire             wire_next_src2_ready;
   
    assign o_data_ready = i_busy & wire_src1_ready & wire_src2_ready;
    assign o_src1_data = ~wire_src1_ready & wire_next_src1_ready ? wire_next_src1_data : wire_src1_data;
    assign o_src2_data = ~wire_src2_ready & wire_next_src2_ready ? wire_next_src2_data : wire_src2_data;
   
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_PC_q <= 0;
            o_imm_q <= 0;
            o_rrftag_q <= 0;
            o_rrftag_valid_q <= 0;
            o_alu_op <= 0;
            o_spectag_q <= 0;
            bhr <= 0;
            prcond <= 0;
            praddr <= 0;
            opcode <= 0;
            
            wire_src1_data <= 0;
            wire_src2_data <= 0;
            wire_src1_ready <= 0;
            wire_src2_ready <= 0;
        end else if (i_we) begin
            o_PC_q <= i_PC;
            o_imm_q <= i_imm;
            o_rrftag_q <= i_rename_rd;
            o_rrftag_valid_q <= i_rd_valid;
            o_alu_op <= i_alu_op;
            o_spectag_q <= i_spectag;
            bhr <= wbhr;
            prcond <= wprcond;
            praddr <= wpraddr;
            opcode <= wopcode;

            wire_src1_data <= i_src1_data;
            wire_src2_data <= i_src2_data;
            wire_src1_ready <= i_src1_ready;
            wire_src2_ready <= i_src2_ready;
        end else begin // if (i_we)
            wire_src1_data <= wire_next_src1_data;
            wire_src2_data <= wire_next_src2_data;
            wire_src1_ready <= wire_next_src1_ready;
            wire_src2_ready <= wire_next_src2_ready;
        end
    end
   
    RAW_Resolve srcmng1
    (
        .i_mux_data(wire_src1_data),
        .i_mux_ready(wire_src1_ready),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5),
        .o_src_data(wire_next_src1_data),
        .o_src_ready(wire_next_src1_ready)
    );

    RAW_Resolve srcmng2
    (
        .i_mux_data(wire_src2_data),
        .i_mux_ready(wire_src2_ready),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5),
        .o_src_data(wire_next_src2_data),
        .o_src_ready(wire_next_src2_ready)
    );
   
endmodule //RS_Branch_Ent



module RS_Branch
(
    //System
    input                            i_clk,
    input                            i_resetn,
    output reg [`GSH_PHT_DEPTH-1:0] o_busy_vector_q,
    input                            i_predict_miss,
    input                            i_predict_hit,
    input      [4:0]    i_predict_tag,
    input      [4:0]    i_specfixtag,
    output     [`GSH_PHT_DEPTH-1:0] prbusyvec_next,
    //WriteSignal
    input                            i_issue_en, //Issue 
    input      [`BRANCH_ENT_SEL-1:0] i_issue_addr, //= raddr, clsbsyadr
    input                            i_dispatch1_en, //alloc1
    input                            i_dispatch2_en, //alloc2
    input      [`BRANCH_ENT_SEL-1:0] i_dispatch1_addr, //allocentry1
    input      [`BRANCH_ENT_SEL-1:0] i_dispatch2_addr, //allocentry2
    //WriteSignal1
    input      [31:0]           i_PC_1,
    input      [31:0]           i_src1_data_1,
    input      [31:0]           i_src2_data_1,
    input                            i_src1_ready_1,
    input                            i_src2_ready_1,
    input      [31:0]           i_imm_1,
    input      [`RRF_ADDRW-1:0]        i_rename_rd_1,
    input                            i_rd_valid_1,
    input      [3:0]   i_alu_op_1,
    input      [4:0]    i_spectag_1,
    input                            i_spectag_valid_1,
    input      [`GSH_BHR_WIDTH-1:0]    wbhr_1,
    input                            wprcond_1,
    input      [31:0]           wpraddr_1,
    input      [6:0]                 wopcode_1,

    //WriteSignal2
    input      [31:0]           i_PC_2,
    input      [31:0]           i_src1_data_2,
    input      [31:0]           i_src2_data_2,
    input                            i_src1_ready_2,
    input                            i_src2_ready_2,
    input      [31:0]           i_imm_2,
    input      [`RRF_ADDRW-1:0]        i_rename_rd_2,
    input                            i_rd_valid_2,
    input      [3:0]   walu_op_2,
    input      [4:0]    i_spectag_2,
    input                            i_spectag_valid_2,
    input      [`GSH_BHR_WIDTH-1:0]    wbhr_2,
    input                            wprcond_2,
    input      [31:0]           wpraddr_2,
    input      [6:0]                 wopcode_2,

    //ReadSignal
    output     [31:0]           o_src1_data,
    output     [31:0]           o_src2_data,
    output     [`GSH_PHT_DEPTH-1:0] o_data_ready,
    output     [31:0]           o_PC_q,
    output     [31:0]           o_imm_q,
    output     [`RRF_ADDRW-1:0]        o_rrftag_q,
    output                           o_rrftag_valid_q,
    output     [3:0]   o_alu_op,
    output     [4:0]    o_spectag_q,
    output                           o_spectag_valid,
    output     [`GSH_BHR_WIDTH-1:0]    bhr,
    output                           prcond,
    output     [31:0]           praddr,
    output     [6:0]                 opcode,
  
    //EXRSLT
    input      [31:0]           i_exe_result1,
    input      [`RRF_ADDRW-1:0]        i_exe_rrftag1,
    input                            i_flush_spec1,
    input      [31:0]           i_exe_result2,
    input      [`RRF_ADDRW-1:0]        i_exe_rrftag2,
    input                            i_flush_spec2,
    input      [31:0]           i_exe_result3,
    input      [`RRF_ADDRW-1:0]        i_exe_rrftag3,
    input                            i_flush_spec3,
    input      [31:0]           i_exe_result4,
    input      [`RRF_ADDRW-1:0]        i_exe_rrftag4,
    input                            i_flush_spec4,
    input      [31:0]           i_exe_result5,
    input      [`RRF_ADDRW-1:0]        i_exe_rrftag5,
    input                            i_flush_spec5
   );

    //_0
    wire [31:0]         wire_o_src1_data_0;
    wire [31:0]         wire_o_src2_data_0;
    wire                     wire_o_data_ready_0;
    wire [31:0]         wire_o_PC_0;
    wire [31:0]         wire_o_imm_0;
    wire [`RRF_ADDRW-1:0]      wire_o_rrftag_0;
    wire                     wire_o_rrftag_valid_0;
    wire [3:0] wire_o_alu_op_0;
    wire [4:0]  wire_o_spectag_0;
    wire [`GSH_BHR_WIDTH-1:0]  bhr_0;
    wire                     prcond_0;
    wire [31:0]         praddr_0;
    wire [6:0]               opcode_0;
    //_1
    wire [31:0]         ex_src1_1;
    wire [31:0]         ex_src2_1;
    wire                     ready_1;
    wire [31:0]         pc_1;
    wire [31:0]         imm_1;
    wire [`RRF_ADDRW-1:0]      rrftag_1;
    wire                     dstval_1;
    wire [3:0] alu_op_1;
    wire [4:0]  spectag_1;
    wire [`GSH_BHR_WIDTH-1:0]  bhr_1;
    wire                     prcond_1;
    wire [31:0]         praddr_1;
    wire [6:0]               opcode_1;
    //_2
    wire [31:0]         ex_src1_2;
    wire [31:0]         ex_src2_2;
    wire                     ready_2;
    wire [31:0]         pc_2;
    wire [31:0]         imm_2;
    wire [`RRF_ADDRW-1:0]      rrftag_2;
    wire                     dstval_2;
    wire [3:0] alu_op_2;
    wire [4:0]  spectag_2;
    wire [`GSH_BHR_WIDTH-1:0]  bhr_2;
    wire                     prcond_2;
    wire [31:0]         praddr_2;
    wire [6:0]               opcode_2;
    //_3
    wire [31:0]         ex_src1_3;
    wire [31:0]         ex_src2_3;
    wire                     ready_3;
    wire [31:0]         pc_3;
    wire [31:0]         imm_3;
    wire [`RRF_ADDRW-1:0]      rrftag_3;
    wire                     dstval_3;
    wire [3:0] alu_op_3;
    wire [4:0]  spectag_3;
    wire [`GSH_BHR_WIDTH-1:0]  bhr_3;
    wire                     prcond_3;
    wire [31:0]         praddr_3;
    wire [6:0]               opcode_3;
   
    reg  [`GSH_PHT_DEPTH-1:0] reg_spectag_valid_vector;

    wire [`GSH_PHT_DEPTH-1:0] wire_busy_set_vector = { (spectag_3 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                              (spectag_2 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                              (spectag_1 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                              (wire_o_spectag_0 & i_specfixtag) == 0 ? 1'b1 : 1'b0 };

    wire [`GSH_PHT_DEPTH-1:0] wire_spectag_valid_set_vector = { (spectag_3 == i_predict_tag) ? 1'b0 : 1'b1,
                                                   (spectag_2 == i_predict_tag) ? 1'b0 : 1'b1,
                                                   (spectag_1 == i_predict_tag) ? 1'b0 : 1'b1,
                                                   (wire_o_spectag_0 == i_predict_tag) ? 1'b0 : 1'b1  };

    wire [`GSH_PHT_DEPTH-1:0]  wire_spectag_valid_vector_next = (wire_spectag_valid_set_vector & reg_spectag_valid_vector);

    assign o_spectag_valid        = i_predict_hit ? wire_spectag_valid_vector_next[i_issue_addr] : reg_spectag_valid_vector[i_issue_addr];
    assign o_data_ready          = {ready_3, ready_2, ready_1, wire_o_data_ready_0};
    assign prbusyvec_next = wire_busy_set_vector & o_busy_vector_q;
   
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_busy_vector_q <= 0;
            reg_spectag_valid_vector <= 0;
        end else begin
            if (i_predict_miss) begin
                o_busy_vector_q <= prbusyvec_next;
                reg_spectag_valid_vector <= 0;
            end else if (i_predict_hit) begin
                reg_spectag_valid_vector <= wire_spectag_valid_vector_next;
                if (i_issue_en) begin
                    o_busy_vector_q[i_issue_addr] <= 1'b0;
                end
            end else begin
                if (i_dispatch1_en) begin
                    o_busy_vector_q[i_dispatch1_addr] <= 1'b1;
                    reg_spectag_valid_vector[i_dispatch1_addr] <= i_spectag_valid_1;
                end
                if (i_dispatch2_en) begin
                    o_busy_vector_q[i_dispatch2_addr] <= 1'b1;
                    reg_spectag_valid_vector[i_dispatch2_addr] <= i_spectag_valid_2;
                end
                if (i_issue_en) begin
                    o_busy_vector_q[i_issue_addr] <= 1'b0;
                end
            end
        end
    end

    RS_Branch_Ent ent0
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .i_busy(o_busy_vector_q[0]),
        .i_PC((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_PC_1 : i_PC_2),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_imm((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_imm_1 : i_imm_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_alu_op((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_alu_op_1 : walu_op_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_spectag_1 : i_spectag_2),
        .wbhr((i_dispatch1_en && (i_dispatch1_addr == 0)) ? wbhr_1 : wbhr_2),
        .wpraddr((i_dispatch1_en && (i_dispatch1_addr == 0)) ? wpraddr_1 : wpraddr_2),
        .wprcond((i_dispatch1_en && (i_dispatch1_addr == 0)) ? wprcond_1 : wprcond_2),
        .wopcode((i_dispatch1_en && (i_dispatch1_addr == 0)) ? wopcode_1 : wopcode_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 0)) || (i_dispatch2_en && (i_dispatch2_addr == 0))),
        .o_src1_data(wire_o_src1_data_0),
        .o_src2_data(wire_o_src2_data_0),
        .o_data_ready(wire_o_data_ready_0),
        .o_PC_q(wire_o_PC_0),
        .o_imm_q(wire_o_imm_0),
        .o_rrftag_q(wire_o_rrftag_0),
        .o_rrftag_valid_q(wire_o_rrftag_valid_0),
        .o_alu_op(wire_o_alu_op_0),
        .o_spectag_q(wire_o_spectag_0),
        .bhr(bhr_0),
        .prcond(prcond_0),
        .praddr(praddr_0),
        .opcode(opcode_0),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );

    RS_Branch_Ent entry1
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .i_busy(o_busy_vector_q[1]),
        .i_PC((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_PC_1 : i_PC_2),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_imm((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_imm_1 : i_imm_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_alu_op((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_alu_op_1 : walu_op_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_spectag_1 : i_spectag_2),
        .wbhr((i_dispatch1_en && (i_dispatch1_addr == 1)) ? wbhr_1 : wbhr_2),
        .wpraddr((i_dispatch1_en && (i_dispatch1_addr == 1)) ? wpraddr_1 : wpraddr_2),
        .wprcond((i_dispatch1_en && (i_dispatch1_addr == 1)) ? wprcond_1 : wprcond_2),
        .wopcode((i_dispatch1_en && (i_dispatch1_addr == 1)) ? wopcode_1 : wopcode_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 1)) || (i_dispatch2_en && (i_dispatch2_addr == 1))),
        .o_src1_data(ex_src1_1),
        .o_src2_data(ex_src2_1),
        .o_data_ready(ready_1),
        .o_PC_q(pc_1),
        .o_imm_q(imm_1),
        .o_rrftag_q(rrftag_1),
        .o_rrftag_valid_q(dstval_1),
        .o_alu_op(alu_op_1),
        .o_spectag_q(spectag_1),
        .bhr(bhr_1),
        .prcond(prcond_1),
        .praddr(praddr_1),
        .opcode(opcode_1),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );

    RS_Branch_Ent entry2
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .i_busy(o_busy_vector_q[2]),
        .i_PC((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_PC_1 : i_PC_2),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_imm((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_imm_1 : i_imm_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_alu_op((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_alu_op_1 : walu_op_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 2)) ? i_spectag_1 : i_spectag_2),
        .wbhr((i_dispatch1_en && (i_dispatch1_addr == 2)) ? wbhr_1 : wbhr_2),
        .wpraddr((i_dispatch1_en && (i_dispatch1_addr == 2)) ? wpraddr_1 : wpraddr_2),
        .wprcond((i_dispatch1_en && (i_dispatch1_addr == 2)) ? wprcond_1 : wprcond_2),
        .wopcode((i_dispatch1_en && (i_dispatch1_addr == 2)) ? wopcode_1 : wopcode_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 2)) || (i_dispatch2_en && (i_dispatch2_addr == 2))),
        .o_src1_data(ex_src1_2),
        .o_src2_data(ex_src2_2),
        .o_data_ready(ready_2),
        .o_PC_q(pc_2),
        .o_imm_q(imm_2),
        .o_rrftag_q(rrftag_2),
        .o_rrftag_valid_q(dstval_2),
        .o_alu_op(alu_op_2),
        .o_spectag_q(spectag_2),
        .bhr(bhr_2),
        .prcond(prcond_2),
        .praddr(praddr_2),
        .opcode(opcode_2),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );

    RS_Branch_Ent ent3
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .i_busy(o_busy_vector_q[3]),
        .i_PC((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_PC_1 : i_PC_2),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_imm((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_imm_1 : i_imm_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_alu_op((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_alu_op_1 : walu_op_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 3)) ? i_spectag_1 : i_spectag_2),
        .wbhr((i_dispatch1_en && (i_dispatch1_addr == 3)) ? wbhr_1 : wbhr_2),
        .wpraddr((i_dispatch1_en && (i_dispatch1_addr == 3)) ? wpraddr_1 : wpraddr_2),
        .wprcond((i_dispatch1_en && (i_dispatch1_addr == 3)) ? wprcond_1 : wprcond_2),
        .wopcode((i_dispatch1_en && (i_dispatch1_addr == 3)) ? wopcode_1 : wopcode_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 3)) || (i_dispatch2_en && (i_dispatch2_addr == 3))),
        .o_src1_data(ex_src1_3),
        .o_src2_data(ex_src2_3),
        .o_data_ready(ready_3),
        .o_PC_q(pc_3),
        .o_imm_q(imm_3),
        .o_rrftag_q(rrftag_3),
        .o_rrftag_valid_q(dstval_3),
        .o_alu_op(alu_op_3),
        .o_spectag_q(spectag_3),
        .bhr(bhr_3),
        .prcond(prcond_3),
        .praddr(praddr_3),
        .opcode(opcode_3),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );
   
    assign o_src1_data = (i_issue_addr == 0) ? wire_o_src1_data_0 :
                     (i_issue_addr == 1) ? ex_src1_1 :
                     (i_issue_addr == 2) ? ex_src1_2 : ex_src1_3;
   
    assign o_src2_data = (i_issue_addr == 0) ? wire_o_src2_data_0 :
                     (i_issue_addr == 1) ? ex_src2_1 :
                     (i_issue_addr == 2) ? ex_src2_2 : ex_src2_3;
   
    assign o_PC_q      = (i_issue_addr == 0) ? wire_o_PC_0 :
                     (i_issue_addr == 1) ? pc_1 :
                     (i_issue_addr == 2) ? pc_2 : pc_3;
   
    assign o_imm_q     = (i_issue_addr == 0) ? wire_o_imm_0 :
                     (i_issue_addr == 1) ? imm_1 :
                     (i_issue_addr == 2) ? imm_2 : imm_3;
   
    assign o_rrftag_q  = (i_issue_addr == 0) ? wire_o_rrftag_0 :
                     (i_issue_addr == 1) ? rrftag_1 :
                     (i_issue_addr == 2) ? rrftag_2 : rrftag_3;
   
    assign o_rrftag_valid_q  = (i_issue_addr == 0) ? wire_o_rrftag_valid_0 :
                     (i_issue_addr == 1) ? dstval_1 :
                     (i_issue_addr == 2) ? dstval_2 : dstval_3;

    assign o_alu_op  = (i_issue_addr == 0) ? wire_o_alu_op_0 :
                     (i_issue_addr == 1) ? alu_op_1 :
                     (i_issue_addr == 2) ? alu_op_2 : alu_op_3;

    assign o_spectag_q = (i_issue_addr == 0) ? wire_o_spectag_0 :
                     (i_issue_addr == 1) ? spectag_1 :
                     (i_issue_addr == 2) ? spectag_2 : spectag_3;
   
    assign bhr     = (i_issue_addr == 0) ? bhr_0 :
                     (i_issue_addr == 1) ? bhr_1 :
                     (i_issue_addr == 2) ? bhr_2 : bhr_3;
   
    assign prcond  = (i_issue_addr == 0) ? prcond_0 :
                     (i_issue_addr == 1) ? prcond_1 :
                     (i_issue_addr == 2) ? prcond_2 : prcond_3;
   
    assign praddr  = (i_issue_addr == 0) ? praddr_0 :
                     (i_issue_addr == 1) ? praddr_1 :
                     (i_issue_addr == 2) ? praddr_2 : praddr_3;
   
    assign opcode  = (i_issue_addr == 0) ? opcode_0 :
                     (i_issue_addr == 1) ? opcode_1 :
                     (i_issue_addr == 2) ? opcode_2 : opcode_3;
endmodule // RS_Branch







module RS_Mul_Ent
(
    //Memory
    input                         i_clk,
    input                         i_resetn,
    input                         i_busy,
    input      [31:0]        i_src1_data,
    input      [31:0]        i_src2_data,
    input                         i_src1_ready,
    input                         i_src2_ready,
    input      [`RRF_ADDRW-1:0]     i_rename_rd,
    input                         i_rd_valid,
    input      [4:0] i_spectag,
    input                         wsrc1_signed,
    input                         wsrc2_signed,
    input                         wsel_lohi,
    input                         i_we,
    output     [31:0]        o_src1_data,
    output     [31:0]        o_src2_data,
    output                        o_data_ready,
    output reg [`RRF_ADDRW-1:0]     o_rrftag_q,
    output reg                    o_rrftag_valid_q,
    output reg [4:0] o_spectag_q,
    output reg                    src1_signed,
    output reg                    src2_signed,
    output reg                    sel_lohi,
    //EXRSLT
    input      [31:0]        i_exe_result1,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag1,
    input                         i_flush_spec1,
    input      [31:0]        i_exe_result2,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag2,
    input                         i_flush_spec2,
    input      [31:0]        i_exe_result3,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag3,
    input                         i_flush_spec3,
    input      [31:0]        i_exe_result4,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag4,
    input                         i_flush_spec4,
    input      [31:0]        i_exe_result5,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag5,
    input                         i_flush_spec5
);

    reg  [31:0] wire_src1_data;
    reg  [31:0] wire_src2_data;
    reg              wire_src1_ready;
    reg              wire_src2_ready;

    wire [31:0] wire_next_src1_data;
    wire [31:0] wire_next_src2_data;   
    wire             wire_next_src1_ready;
    wire             wire_next_src2_ready;
   
    assign o_data_ready = i_busy & wire_src1_ready & wire_src2_ready;
    assign o_src1_data = ~wire_src1_ready & wire_next_src1_ready ? wire_next_src1_data : wire_src1_data;
    assign o_src2_data = ~wire_src2_ready & wire_next_src2_ready ? wire_next_src2_data : wire_src2_data;
   
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_rrftag_q <= 0;
            o_rrftag_valid_q <= 0;
            o_spectag_q <= 0;
            src1_signed <= 0;
            src2_signed <= 0;
            sel_lohi <= 0;

            wire_src1_data <= 0;
            wire_src2_data <= 0;
            wire_src1_ready <= 0;
            wire_src2_ready <= 0;
        end else if (i_we) begin
            o_rrftag_q <= i_rename_rd;
            o_rrftag_valid_q <= i_rd_valid;
            o_spectag_q <= i_spectag;
            src1_signed <= wsrc1_signed;
            src2_signed <= wsrc2_signed;
            sel_lohi <= wsel_lohi;

            wire_src1_data <= i_src1_data;
            wire_src2_data <= i_src2_data;
            wire_src1_ready <= i_src1_ready;
            wire_src2_ready <= i_src2_ready;
        end else begin // if (i_we)
            wire_src1_data <= wire_next_src1_data;
            wire_src2_data <= wire_next_src2_data;
            wire_src1_ready <= wire_next_src1_ready;
            wire_src2_ready <= wire_next_src2_ready;
        end
    end
   
    RAW_Resolve srcmng1
    (
        .i_mux_data(wire_src1_data),
        .i_mux_ready(wire_src1_ready),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5),
        .o_src_data(wire_next_src1_data),
        .o_src_ready(wire_next_src1_ready)
    );

    RAW_Resolve srcmng2
    (
        .i_mux_data(wire_src2_data),
        .i_mux_ready(wire_src2_ready),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5),
        .o_src_data(wire_next_src2_data),
        .o_src_ready(wire_next_src2_ready)
    );
   
endmodule //RS_Mul_Ent


module RS_Mul
(
   //System
    input                         i_clk,
    input                         i_resetn,
    output reg [1:0] o_busy_vector_q,
    input                         i_predict_miss,
    input                         i_predict_hit,
    input      [4:0] i_predict_tag,
    input      [4:0] i_specfixtag,
   //WriteSignal
    input                         i_issue_en, //Issue 
    input      [0:0] i_issue_addr, //= raddr, clsbsyadr
    input                         i_dispatch1_en, //alloc1
    input                         i_dispatch2_en, //alloc2
    input      [0:0] i_dispatch1_addr, //allocentry1
    input      [0:0] i_dispatch2_addr, //allocentry2
   //WriteSignal1
    input      [31:0]        i_src1_data_1,
    input      [31:0]        i_src2_data_1,
    input                         i_src1_ready_1,
    input                         i_src2_ready_1,
    input      [`RRF_ADDRW-1:0]     i_rename_rd_1,
    input                         i_rd_valid_1,
    input      [4:0] i_spectag_1,
    input                         i_spectag_valid_1,
    input                         wsrc1_signed_1,
    input                         wsrc2_signed_1,
    input                         wsel_lohi_1,

   //WriteSignal2
    input      [31:0]        i_src1_data_2,
    input      [31:0]        i_src2_data_2,
    input                         i_src1_ready_2,
    input                         i_src2_ready_2,
    input      [`RRF_ADDRW-1:0]     i_rename_rd_2,
    input                         i_rd_valid_2,
    input      [4:0] i_spectag_2,
    input                         i_spectag_valid_2,
    input                         wsrc1_signed_2,
    input                         wsrc2_signed_2,
    input                         wsel_lohi_2,

   //ReadSignal
    output     [31:0]        o_src1_data,
    output     [31:0]        o_src2_data,
    output     [1:0] o_data_ready,
    output     [`RRF_ADDRW-1:0]     o_rrftag_q,
    output                        o_rrftag_valid_q,
    output     [4:0] o_spectag_q,
    output                        o_spectag_valid,
    output                        src1_signed,
    output                        src2_signed,
    output                        sel_lohi,

   //EXRSLT
    input      [31:0]        i_exe_result1,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag1,
    input                         i_flush_spec1,
    input      [31:0]        i_exe_result2,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag2,
    input                         i_flush_spec2,
    input      [31:0]        i_exe_result3,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag3,
    input                         i_flush_spec3,
    input      [31:0]        i_exe_result4,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag4,
    input                         i_flush_spec4,
    input      [31:0]        i_exe_result5,
    input      [`RRF_ADDRW-1:0]     i_exe_rrftag5,
    input                         i_flush_spec5
);

    //_0
    wire [31:0]        wire_o_src1_data_0;
    wire [31:0]        wire_o_src2_data_0;
    wire                    wire_o_data_ready_0;
    wire [`RRF_ADDRW-1:0]     wire_o_rrftag_0;
    wire                    wire_o_rrftag_valid_0;
    wire [4:0] wire_o_spectag_0;
    wire                    src1_signed_0;
    wire                    src2_signed_0;
    wire                    sel_lohi_0;
   
    //_1
    wire [31:0]        ex_src1_1;
    wire [31:0]        ex_src2_1;
    wire                    ready_1;
    wire [`RRF_ADDRW-1:0]     rrftag_1;
    wire                    dstval_1;
    wire [4:0] spectag_1;
    wire                    src1_signed_1;
    wire                    src2_signed_1;
    wire                    sel_lohi_1;

    reg  [1:0] reg_spectag_valid_vector;

    wire [1:0] wire_busy_set_vector = { (spectag_1 & i_specfixtag) == 0 ? 1'b1 : 1'b0,
                                           (wire_o_spectag_0 & i_specfixtag) == 0 ? 1'b1 : 1'b0 };

    wire [1:0] wire_spectag_valid_set_vector = { (spectag_1 == i_predict_tag) ? 1'b0 : 1'b1,
                                                (wire_o_spectag_0 == i_predict_tag) ? 1'b0 : 1'b1  };

    wire [1:0] wire_spectag_valid_vector_next = (wire_spectag_valid_set_vector & reg_spectag_valid_vector);
    assign o_spectag_valid = i_predict_hit ? wire_spectag_valid_vector_next[i_issue_addr] : reg_spectag_valid_vector[i_issue_addr];
    assign o_data_ready   = {ready_1, wire_o_data_ready_0};
   
    always @(posedge i_clk) begin
        if (i_resetn) begin
            o_busy_vector_q    <= 0;
            reg_spectag_valid_vector <= 0;
        end else begin
            if (i_predict_miss) begin
                o_busy_vector_q    <= wire_busy_set_vector & o_busy_vector_q;
                reg_spectag_valid_vector <= 0;
            end else if (i_predict_hit) begin
                reg_spectag_valid_vector <= wire_spectag_valid_vector_next;
                if (i_issue_en) begin
                    o_busy_vector_q[i_issue_addr] <= 1'b0;
                end
            end else begin
                if (i_dispatch1_en) begin
                    o_busy_vector_q[i_dispatch1_addr]    <= 1'b1;
                    reg_spectag_valid_vector[i_dispatch1_addr] <= i_spectag_valid_1;
                end
                if (i_dispatch2_en) begin
                    o_busy_vector_q[i_dispatch2_addr]    <= 1'b1;
                    reg_spectag_valid_vector[i_dispatch2_addr] <= i_spectag_valid_2;
                end
                if (i_issue_en) begin
                    o_busy_vector_q[i_issue_addr] <= 1'b0;
                end
            end
        end
    end

    RS_Mul_Ent ent0
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .i_busy(o_busy_vector_q[0]),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 0)) ? i_spectag_1 : i_spectag_2),
        .wsrc1_signed((i_dispatch1_en && (i_dispatch1_addr == 0)) ? wsrc1_signed_1 : wsrc1_signed_2),
        .wsrc2_signed((i_dispatch1_en && (i_dispatch1_addr == 0)) ? wsrc2_signed_1 : wsrc2_signed_2),
        .wsel_lohi((i_dispatch1_en && (i_dispatch1_addr == 0)) ? wsel_lohi_1 : wsel_lohi_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 0)) || (i_dispatch2_en && (i_dispatch2_addr == 0))),
        .o_src1_data(wire_o_src1_data_0),
        .o_src2_data(wire_o_src2_data_0),
        .o_data_ready(wire_o_data_ready_0),
        .o_rrftag_q(wire_o_rrftag_0),
        .o_rrftag_valid_q(wire_o_rrftag_valid_0),
        .o_spectag_q(wire_o_spectag_0),
        .src1_signed(src1_signed_0),
        .src2_signed(src2_signed_0),
        .sel_lohi(sel_lohi_0),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );

    RS_Mul_Ent entry1
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .i_busy(o_busy_vector_q[1]),
        .i_src1_data((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src1_data_1 : i_src1_data_2),
        .i_src2_data((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src2_data_1 : i_src2_data_2),
        .i_src1_ready((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src1_ready_1 : i_src1_ready_2),
        .i_src2_ready((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_src2_ready_1 : i_src2_ready_2),
        .i_rename_rd((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_rename_rd_1 : i_rename_rd_2),
        .i_rd_valid((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_rd_valid_1 : i_rd_valid_2),
        .i_spectag((i_dispatch1_en && (i_dispatch1_addr == 1)) ? i_spectag_1 : i_spectag_2),
        .wsrc1_signed((i_dispatch1_en && (i_dispatch1_addr == 1)) ? wsrc1_signed_1 : wsrc1_signed_2),
        .wsrc2_signed((i_dispatch1_en && (i_dispatch1_addr == 1)) ? wsrc2_signed_1 : wsrc2_signed_2),
        .wsel_lohi((i_dispatch1_en && (i_dispatch1_addr == 1)) ? wsel_lohi_1 : wsel_lohi_2),
        .i_we((i_dispatch1_en && (i_dispatch1_addr == 1)) || (i_dispatch2_en && (i_dispatch2_addr == 1))),
        .o_src1_data(ex_src1_1),
        .o_src2_data(ex_src2_1),
        .o_data_ready(ready_1),
        .o_rrftag_q(rrftag_1),
        .o_rrftag_valid_q(dstval_1),
        .o_spectag_q(spectag_1),
        .src1_signed(src1_signed_1),
        .src2_signed(src2_signed_1),
        .sel_lohi(sel_lohi_1),
        .i_exe_result1(i_exe_result1),
        .i_exe_rrftag1(i_exe_rrftag1),
        .i_flush_spec1(i_flush_spec1),
        .i_exe_result2(i_exe_result2),
        .i_exe_rrftag2(i_exe_rrftag2),
        .i_flush_spec2(i_flush_spec2),
        .i_exe_result3(i_exe_result3),
        .i_exe_rrftag3(i_exe_rrftag3),
        .i_flush_spec3(i_flush_spec3),
        .i_exe_result4(i_exe_result4),
        .i_exe_rrftag4(i_exe_rrftag4),
        .i_flush_spec4(i_flush_spec4),
        .i_exe_result5(i_exe_result5),
        .i_exe_rrftag5(i_exe_rrftag5),
        .i_flush_spec5(i_flush_spec5)
    );
   
    assign o_src1_data     = (i_issue_addr == 0) ? wire_o_src1_data_0     : ex_src1_1;
   
    assign o_src2_data     = (i_issue_addr == 0) ? wire_o_src2_data_0     : ex_src2_1;

    assign o_rrftag_q      = (i_issue_addr == 0) ? wire_o_rrftag_0      : rrftag_1;
   
    assign o_rrftag_valid_q      = (i_issue_addr == 0) ? wire_o_rrftag_valid_0      : dstval_1;

    assign o_spectag_q     = (i_issue_addr == 0) ? wire_o_spectag_0     : spectag_1;

    assign src1_signed = (i_issue_addr == 0) ? src1_signed_0 : src1_signed_1;

    assign src2_signed = (i_issue_addr == 0) ? src2_signed_0 : src2_signed_1;

    assign sel_lohi    = (i_issue_addr == 0) ? sel_lohi_0    : sel_lohi_1;   
endmodule // RS_Mul