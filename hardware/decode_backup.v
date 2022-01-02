/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Create Date: 2021.12.23
 *  Description: instruction decoder module of rv32Core
 */
`include "configure.h"

module Decoder
(
    input      [31:0]             inst,
    output reg [1:0]   inst_type,
    output     [4:0]          rs1,
    output     [4:0]          rs2,
    output     [4:0]          rd,
    output reg [1:0]  src1_sel,
    output reg [1:0]  src2_sel,
    output reg                         rd_valid,
	       
    output reg                         rs1_valid,
    output reg                         rs2_valid,
    output reg                         ill_inst,
    output reg [3:0]     alu_op,
    output reg [2:0]       rs_type,
//	output reg                         dmem_valid,
//	output reg                         dmem_write,
    output     [2:0] 	               dmem_size,
    output     [2:0]   dmem_type, 
    output reg [1:0]      muldiv_op,
    output reg                         muldiv_rs1_signed,
    output reg                         muldiv_rs2_signed,
    output reg [1:0] muldiv_out_sel
);

    wire [3:0]           srl_or_sra;
    wire [3:0]           add_or_sub;
    wire [2:0]             rs_type_md;
   
    wire [6:0] 		    opcode  = inst[6:0];
    wire [6:0] 		    funct7  = inst[31:25];
    wire [11:0]         funct12 = inst[31:20];
    wire [2:0] 		    funct3  = inst[14:12];
//  reg [1:0]   muldiv_op;
    reg [3:0]  alu_op_arith;
   
    assign rd  = inst[11:7];
    assign rs1 = inst[19:15];
    assign rs2 = inst[24:20];
 
    assign dmem_size = {1'b0,funct3[1:0]};
    assign dmem_type = funct3;
   
    always @(*) begin
        inst_type     = `I_type;
        src1_sel    = `SRC1_RS1;
        src2_sel    = `SRC2_IMM;
        rd_valid       = 1'b0;
        rs1_valid     = 1'b1;
        rs2_valid     = 1'b0;
        ill_inst = 1'b0;
      //dmem_valid     = 1'b0;
      //dmem_write   = 1'b0;
        rs_type       = `RS_ALU;
        alu_op       = `ALU_ADD;
      
        case (opcode)
	        `RV32_LOAD : begin
//              dmem_valid      = 1'b1;
                rd_valid        = 1'b1;
	            rs_type        = `RS_LDST;
//              wb_src_sel_DX = `WB_SRC_MEM;
            end
            `STORE : begin
                rs2_valid      = 1'b1;
                inst_type      = `S_type;
//              dmem_valid      = 1'b1;
 //             dmem_write    = 1'b1;
	            rs_type        = `RS_LDST;
            end
            `BRANCH : begin
                rs2_valid = 1'b1;
                //branch_taken_unkilled = cmp_true;
                src2_sel = `SRC2_RS2;
                case (funct3)
                    `FUNCT3_BEQ  : alu_op = `ALU_SEQ;
                    `FUNCT3_BNE  : alu_op = `ALU_SNE;
                    `FUNCT3_BLT  : alu_op = `ALU_SLT;
                    `FUNCT3_BLTU : alu_op = `ALU_SLTU;
                    `FUNCT3_BGE  : alu_op = `ALU_SGE;
                    `FUNCT3_BGEU : alu_op = `ALU_SGEU;
                    default :     ill_inst = 1'b1;
                endcase //case(funct3)
	            rs_type = `RS_BRANCH;
            end
            `JAL : begin
	          //jal_unkilled  = 1'b1;
                rs1_valid      = 1'b0;
                src1_sel     = `SRC1_PC;
                src2_sel     = `SRC2_FOUR;
                rd_valid        = 1'b1;
	            rs_type        = `RS_JAL;
            end
            `JALR : begin
                ill_inst  = (funct3 != 0);
	   //       jalr_unkilled = 1'b1;
                src1_sel     = `SRC1_PC;
                src2_sel     = `SRC2_FOUR;
                rd_valid        = 1'b1;
	            rs_type        = `RS_JALR;
            end
	        /****************************************************************
            `RV32_MISC_MEM : begin
                case(funct3)
                    `RV32_FUNCT3_FENCE : begin
                        if ((inst[31:28] == 0) && (rs1 == 0) && (reg_to_wr_DX == 0))
                            ; // most fences are no-ops
                        else
                            ill_inst = 1'b1;
                    end
                    `RV32_FUNCT3_FENCE_I : begin
                        if ((inst[31:20] == 0) && (rs1 == 0) && (reg_to_wr_DX == 0))
                            fence_i = 1'b1;
                        else
                            ill_inst = 1'b1;
                    end
                    default : ill_inst = 1'b1;
                endcase //case(funct3)
            end
	        ************************************************************************/
            `RV32_OP_IMM : begin
                alu_op    = alu_op_arith;
                rd_valid    = 1'b1;
            end
            `RV32_OP  : begin
                rs2_valid  = 1'b1;
                src2_sel = `SRC2_RS2;
                alu_op    = alu_op_arith;
                rd_valid    = 1'b1;
                if (funct7 == `RV32_FUNCT7_MUL_DIV) begin
//                  uses_md_unkilled = 1'b1;
	                rs_type = rs_type_md;
//                  wb_src_sel_DX = `WB_SRC_MD;
                end
            end
	        /*****************************************************************************
            `RV32_SYSTEM : begin
                wb_src_sel_DX = `WB_SRC_CSR;
                rd_valid = (funct3 != `RV32_FUNCT3_PRIV);
                case (funct3)
                    `RV32_FUNCT3_PRIV : begin
                        if ((rs1 == 0) && (reg_to_wr_DX == 0)) begin
                            case (funct12)
                                `RV32_FUNCT12_ECALL : ecall = 1'b1;
                                `RV32_FUNCT12_EBREAK : ebreak = 1'b1;
                                `RV32_FUNCT12_ERET : begin
                                    if (prv == 0)
                                        ill_inst = 1'b1;
                                    else
                                        eret_unkilled = 1'b1;
                                end
                                default : ill_inst = 1'b1;
                            endcase //case(funct12)
                        end //if ((rs1 == 0) && (reg_to_wr_DX == 0))
                    end // case: `RV32_FUNCT3_PRIV
                    `RV32_FUNCT3_CSRRW : csr_cmd = (rs1 == 0) ? `CSR_READ : `CSR_WRITE;
                    `RV32_FUNCT3_CSRRS : csr_cmd = (rs1 == 0) ? `CSR_READ : `CSR_SET;
                    `RV32_FUNCT3_CSRRC : csr_cmd = (rs1 == 0) ? `CSR_READ : `CSR_CLEAR;
                    `RV32_FUNCT3_CSRRWI : csr_cmd = (rs1 == 0) ? `CSR_READ : `CSR_WRITE;
                    `RV32_FUNCT3_CSRRSI : csr_cmd = (rs1 == 0) ? `CSR_READ : `CSR_SET;
                    `RV32_FUNCT3_CSRRCI : csr_cmd = (rs1 == 0) ? `CSR_READ : `CSR_CLEAR;
                    default : ill_inst = 1'b1;
                endcase //case(funct3)
            end
	        ************************************************************************************/
            `AUIPC : begin
                rs1_valid  = 1'b0;
                src1_sel = `SRC1_PC;
                inst_type  = `U_type;
                rd_valid    = 1'b1;
            end
            `LUI : begin
                rs1_valid  = 1'b0;
                src1_sel = `SRC1_ZERO;
                inst_type  = `U_type;
                rd_valid    = 1'b1;
            end
            default : begin
                ill_inst = 1'b1;
            end
        endcase //case (opcode)
    end //always @(*)

    assign add_or_sub = ((opcode == `RV32_OP) && (funct7[5])) ? `ALU_SUB : `ALU_ADD;
    assign srl_or_sra = (funct7[5]) ? `ALU_SRA : `ALU_SRL;

    always @(*) begin
        case (funct3)
            `RV32_FUNCT3_ADD_SUB : alu_op_arith = add_or_sub;
            `RV32_FUNCT3_SLL     : alu_op_arith = `ALU_SLL;
            `RV32_FUNCT3_SLT     : alu_op_arith = `ALU_SLT;
            `RV32_FUNCT3_SLTU    : alu_op_arith = `ALU_SLTU;
            `RV32_FUNCT3_XOR     : alu_op_arith = `ALU_XOR;
            `RV32_FUNCT3_SRA_SRL : alu_op_arith = srl_or_sra;
            `RV32_FUNCT3_OR      : alu_op_arith = `ALU_OR;
            `RV32_FUNCT3_AND     : alu_op_arith = `ALU_AND;
            default              : alu_op_arith = `ALU_ADD;
        endcase //case (funct3)
    end // always @ begin


    //assign md_req_valid = uses_md;
    assign rs_type_md = ( (funct3 == `RV32_FUNCT3_MUL)    ||
		                 (funct3 == `RV32_FUNCT3_MULH)   ||
		                 (funct3 == `RV32_FUNCT3_MULHSU) ||
		                 (funct3 == `RV32_FUNCT3_MULHU)
		               ) ? `RS_MUL : `RS_DIV;
   
    always @(*) begin
        muldiv_op = `MUL;
        muldiv_rs1_signed = 0;
        muldiv_rs2_signed = 0;
        muldiv_out_sel = `OUT_LOW;
        case (funct3)
            `RV32_FUNCT3_MUL : begin
            end
            `RV32_FUNCT3_MULH : begin
                muldiv_rs1_signed = 1;
                muldiv_rs2_signed = 1;
                muldiv_out_sel = `OUT_HIGH;
            end
            `RV32_FUNCT3_MULHSU : begin
                muldiv_rs1_signed = 1;
                muldiv_out_sel = `OUT_HIGH;
            end
            `RV32_FUNCT3_MULHU : begin
                muldiv_out_sel = `OUT_HIGH;
            end
            `RV32_FUNCT3_DIV : begin
                muldiv_op = `DIV;
                muldiv_rs1_signed = 1;
                muldiv_rs2_signed = 1;
            end
            `RV32_FUNCT3_DIVU : begin
                muldiv_op = `DIV;
            end
            `RV32_FUNCT3_REM : begin
                muldiv_op = `REM;
                muldiv_rs1_signed = 1;
                muldiv_rs2_signed = 1;
                muldiv_out_sel = `OUT_REM;
            end
            `RV32_FUNCT3_REMU : begin
                muldiv_op = `REM;
                muldiv_out_sel = `OUT_REM;
            end
        endcase//case (funct3)
    end

   
endmodule //Decoder