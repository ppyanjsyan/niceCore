/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Email      : wuhongbin2014@163.com
 *  Create Date: 2021.12.23
 *  Description: instruction decoder stage of rv32Core
 */
`include "configure.h"

module Decoder
(
    input      [31:0] inst,
    
	output     [4:0]  rd,
    output     [4:0]  rs1,
    output     [4:0]  rs2,
	output reg        rd_valid,
    output reg        rs1_valid,
    output reg        rs2_valid,
    
	output reg [2:0]  inst_type,
    output reg [1:0]  src1_sel,
    output reg [1:0]  src2_sel,
    output reg        ill_inst,
    output reg [3:0]  alu_op,
    output reg [2:0]  rs_type,

`ifdef ENABLE_MULDIV
	output reg [1:0]  muldiv_op,
    output reg        muldiv_rs1_signed,
    output reg        muldiv_rs2_signed,
    output reg [1:0]  muldiv_out_sel,
`endif

    output reg        dmem_valid,
    output reg [3:0]  dmem_wrstrb,
	output reg [3:0]  dmem_rdstrb
);
    wire [3:0]        srl_or_sra;
    wire [3:0]        add_or_sub;
    wire [2:0]        rs_type_md;
   
    wire [6:0]        opcode  = inst[6:0];
    wire [6:0]        funct7  = inst[31:25];
    wire [11:0]       funct12 = inst[31:20];
    wire [2:0]        funct3  = inst[14:12];
    reg  [3:0]        alu_op_arith;
   
    assign rd  = inst[11:7];
    assign rs1 = inst[19:15];
    assign rs2 = inst[24:20];
   
    always @(*) begin
        rd_valid    = 1'b0;
        rs1_valid   = 1'b1;
        rs2_valid   = 1'b0;

        inst_type    = `I_type;
        src1_sel    = `SRC1_RS1;
        src2_sel    = `SRC2_IMM;
        ill_inst    = 1'b0;
        alu_op      = `ALU_ADD;
        rs_type     = `RS_ALU;
		
        dmem_valid  = 1'b0;
        dmem_wrstrb = 4'b0;
		dmem_rdstrb = 4'b0;

        (* parallel_case, full_case *)
        case (opcode)
            7'b0110111 : begin //lui
                rd_valid  = 1'b1;
				rs1_valid = 1'b0;
                inst_type  = `U_type;
                src1_sel  = `SRC1_ZERO;
			end
            7'b0010111 : begin //auipc
                rd_valid  = 1'b1;
				rs1_valid = 1'b0;
                inst_type  = `U_type;
				src1_sel  = `SRC1_PC;   
            end
            7'b1101111 : begin //jal
                rd_valid  = 1'b1;
				rs1_valid = 1'b0;
                src1_sel  = `SRC1_PC;
                src2_sel  = `SRC2_FOUR;    
                rs_type   = `RS_JAL;
            end
            7'b1100111 : begin //jalr
                rd_valid  = 1'b1;
                src1_sel  = `SRC1_PC;
                src2_sel  = `SRC2_FOUR;
                ill_inst  = (funct3 != 0);
                rs_type   = `RS_JALR;
            end
            7'b1100011 : begin //branch
                //rd_valid = 1'b0;
				rs2_valid = 1'b1;
                src2_sel  = `SRC2_RS2;
				rs_type   = `RS_BRANCH;
                case (funct3)
                    3'b000  : alu_op   = `ALU_SEQ;  //beq
                    3'b001  : alu_op   = `ALU_SNE;  //bne
                    3'b100  : alu_op   = `ALU_SLT;  //ble
                    3'b101  : alu_op   = `ALU_SGE;  //bge
					3'b110  : alu_op   = `ALU_SLTU; //bltu
                    3'b111  : alu_op   = `ALU_SGEU; //bgeu
                    default : ill_inst = 1'b1;
                endcase    
            end
            7'b0000011 : begin //load
                dmem_valid = 1'b1;
                rd_valid   = 1'b1;
                rs_type    = `RS_LDST;
				(* full_case *)
				case (funct3)
				    3'b000: dmem_rdstrb = 4'b0001; //lb
					3'b001: dmem_rdstrb = 4'b0011; //lh
					3'b010: dmem_rdstrb = 4'b1111; //lw
				endcase
            end
            7'b0100011 : begin //store
                rs2_valid  = 1'b1;
                inst_type   = `S_type;
                dmem_valid = 1'b1;
                rs_type    = `RS_LDST;
				(* full_case *)
				case (funct3)
				    3'b000: dmem_wrstrb = 4'b0001; //sb
					3'b001: dmem_wrstrb = 4'b0011; //sh
					3'b010: dmem_wrstrb = 4'b1111; //sw
				endcase
            end
            7'b0010011 : begin //op-imm: addi/slti/sltiu/xori/ori/andi/slli/srli/srai
                rd_valid   = 1'b1;
                case (funct3)
                    3'b000  : alu_op   = `ALU_ADD;  //addi
                    3'b010  : alu_op   = `ALU_SLT;  //slti
                    3'b011  : alu_op   = `ALU_SLT;  //sltui
                    3'b100  : alu_op   = `ALU_XOR;  //xori
					3'b110  : alu_op   = `ALU_OR;   //ori
                    3'b111  : alu_op   = `ALU_AND;  //andi
					3'b001  : begin
					      if (funct7 == 7'b0000000) begin  //slli
						      alu_op   = `ALU_SLL;
						  end else begin
						      ill_inst = 1'b1;
						  end
					end
					3'b101 : begin
					      if (funct7 == 7'b0000000) begin  //srli
						      alu_op   = `ALU_SRL;
						  end else if (funct7 == 7'b0100000) begin //srai
						      alu_op   = `ALU_SRA;
					      end else
						      ill_inst = 1'b1;
						  end
                    default : ill_inst = 1'b1;
                endcase  
            end
            7'b0110011 : begin //op: add/sub/sll/slt/sltu/xor/srl/sra/or/and/mul/mulh/mulhsu/mulhu/div/divu/rem/remu
                rd_valid   = 1'b1;
				rs2_valid  = 1'b1;
                src2_sel   = `SRC2_RS2;
				if (funct7 == 7'b0000001) begin //RV32M

`ifdef ENABLE_MULDIV
                    rs_type = ( (funct3 == 3'b000) ||  //mul
                                (funct3 == 3'b001) ||  //mulh
                                (funct3 == 3'b010) ||  //mulhsu
                                (funct3 == 3'b011) )?  //mulhu
                                `RS_MUL : `RS_DIV;
`else
					ill_inst = 1'b1;
`endif

                end else begin //if (funct7 == 7'b0000001)
					case (funct3)
						3'b000  : begin
							if (funct7 == 7'b0000000) begin  //add
								alu_op   = `ALU_ADD;
							end else if (funct7 == 7'b0100000) begin  //sub
								alu_op   = `ALU_SUB;
							end else begin
								ill_inst = 1'b1;
							end
						end
						3'b001  : begin
							if (funct7 == 7'b0000000) begin  //sll
								alu_op   = `ALU_SLL;
							end else begin
								ill_inst = 1'b1;
							end
						end
						3'b010  : begin
							if (funct7 == 7'b0000000) begin  //slt
								alu_op   = `ALU_SLT;
							end else begin
								ill_inst = 1'b1;
							end
						end
						3'b011  : begin
							if (funct7 == 7'b0000000) begin  //sltu
								alu_op   = `ALU_SLT;
							end else begin
								ill_inst = 1'b1;
							end
						end
						3'b100  : begin
							if (funct7 == 7'b0000000) begin  //xor
								alu_op   = `ALU_XOR;
							end else begin
								ill_inst = 1'b1;
							end
						end
						3'b101  : begin
							if (funct7 == 7'b0000000) begin  //srl
								alu_op   = `ALU_SRL;
							end else if (funct7 == 7'b0100000) begin  //sra
								alu_op   = `ALU_SRA;
							end else begin
								ill_inst = 1'b1;
							end
						end
						3'b110  : begin
							if (funct7 == 7'b0000000) begin  //or
								alu_op   = `ALU_OR;
							end else begin
								ill_inst = 1'b1;
							end
						end
						3'b111  : begin
							if (funct7 == 7'b0000000) begin  //xor
								alu_op   = `ALU_AND;
							end else begin
								ill_inst = 1'b1;
							end
						end
						default : ill_inst = 1'b1;
					endcase //case (funct3)
				end //!if (funct7 == 7'b0000001)
            end
            default : begin
                ill_inst = 1'b1;
            end
        endcase //case (opcode)
    end //always @(*)


`ifdef ENABLE_MULDIV  
    always @(*) begin
        muldiv_op         = `MUL;
        muldiv_rs1_signed = 1'b0;
        muldiv_rs2_signed = 1'b0;
        muldiv_out_sel    = `OUT_LOW;
        case (funct3)
            3'b000 : begin  //mul
            end
            3'b001 : begin  //mulh
                muldiv_rs1_signed = 1'b1;
                muldiv_rs2_signed = 1'b1;
                muldiv_out_sel    = `OUT_HIGH;
            end
            3'b010 : begin  //mulhsu
                muldiv_rs1_signed = 1'b1;
                muldiv_out_sel    = `OUT_HIGH;
            end
            3'b011 : begin  //mulhu
                muldiv_out_sel    = `OUT_HIGH;
            end
            3'b100 : begin  //div
                muldiv_op         = `DIV;
                muldiv_rs1_signed = 1'b1;
                muldiv_rs2_signed = 1'b1;
            end
            3'b101 : begin  //divu
                muldiv_op         = `DIV;
            end
            3'b110 : begin  //rem
                muldiv_op         = `REM;
                muldiv_rs1_signed = 1;
                muldiv_rs2_signed = 1;
                muldiv_out_sel    = `OUT_REM;
            end
            3'b111 : begin  //remu
                muldiv_op         = `REM;
                muldiv_out_sel    = `OUT_REM;
            end
        endcase//case (funct3)
    end
`endif

endmodule //Decoder








module TAG_Generator
(
    input            i_clk,
    input            i_resetn,
	
    input            branch_valid1,
    input            branch_valid2,
    input            predict_miss,
    input            predict_hit,
    input            enable,        //indicates whether the instructions can be issued
    input      [4:0] tag_fix,
	
    output     [4:0] spec_tag1,
    output     [4:0] spec_tag2,
    output           speculative1,
    output           speculative2,
    output           attachable,
    output reg [4:0] reg_tag
);

    reg [4:0] brdepth;
   
    assign spec_tag1 = (branch_valid1) ? 
                       {reg_tag[3:0], reg_tag[4]} : reg_tag;

    assign spec_tag2 = (branch_valid2) ? 
                       {spec_tag1[3:0], spec_tag1[4]} : spec_tag1;

    assign speculative1 = (brdepth != 0) ? 1'b1 : 1'b0;
    assign speculative2 = ((brdepth != 0) || branch_valid1) ? 1'b1 : 1'b0;
    assign attachable   = (brdepth + branch_valid1 + branch_valid2)>(`GSH_PHT_DEPTH + predict_hit) ? 1'b0 : 1'b1;

    always @(posedge i_clk) begin
        if (~i_resetn) begin
            reg_tag <= 5'b00001;
            brdepth <= 5'b00000;
        end else begin
            reg_tag <= predict_miss ? tag_fix : ~enable ? reg_tag : spec_tag2;
            brdepth <= predict_miss ? 5'b0 :
                       ~enable ? brdepth - predict_hit :
                       brdepth + branch_valid1 + branch_valid2 - predict_hit;
        end
    end

endmodule //TAG_Generator