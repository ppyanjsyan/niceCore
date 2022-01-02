/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Email      : wuhongbin2014@163.com
 *  Create Date: 2021.12.24
 *  Description: the execution stage of rv32Core
 */
`include "configure.h"

module ExeUnit_Alu
(
    input                         i_clk,
    input                         i_resetn,
    input  [31:0]            ex_src1,
    input  [31:0]            ex_src2,
    input  [31:0]            PC,
    input  [31:0]            imm,
    input                         dstval,
    input  [1:0] src_a,
    input  [1:0] src_b,
    input  [3:0]    alu_op,
    input  [4:0]     spectag,
    input                         specbit,
    input                         issue,
    input                         predict_miss,
    input  [4:0]     spectagfix,
    output [31:0]            result,
    output                        rrf_we,
    output                        rob_we, //set finish
    output                        kill_speculative
);

    wire [31:0] alusrc1;
    wire [31:0] alusrc2;

    reg                  busy;

    assign rob_we           = busy;
    assign rrf_we           = busy & dstval;
    assign kill_speculative = ((spectag & spectagfix) != 0) && specbit && predict_miss;
   
    always @(posedge i_clk) begin
        if (~i_resetn) begin
            busy <= 0;
        end else begin
            busy <= issue;
        end
    end
   
    src_a_mux samx
    (
        .src1_sel(src_a),
        .PC(PC),
        .rs1(ex_src1),
        .alu_src_a(alusrc1)
    );

    src_b_mux sbmx
    (
        .src2_sel(src_b),
        .imm(imm),
        .rs2(ex_src2),
        .alu_src_b(alusrc2)
    );

    alu alice
    (
        .op(alu_op),
        .in1(alusrc1),
        .in2(alusrc2),
        .out(result)
    );
endmodule // ExeUnit_Alu


module src_a_mux
(
    input      [1:0] src1_sel,
    input      [31:0]        PC,
    input      [31:0]        rs1,
    output reg [31:0]        alu_src_a
);
    always @(*) begin
        case (src1_sel)
            `SRC1_RS1 : alu_src_a = rs1;
            `SRC1_PC  : alu_src_a = PC;
            default    : alu_src_a = 0;
        endcase // case (src1_sel)
   end
endmodule // src_a_mux


module src_b_mux
(
    input      [1:0] src2_sel,
    input      [31:0]            imm,
    input      [31:0]            rs2,
    output reg [31:0]            alu_src_b
);
    always @(*) begin
        case (src2_sel)
            `SRC2_RS2  : alu_src_b = rs2;
            `SRC2_IMM  : alu_src_b = imm;
            `SRC2_FOUR : alu_src_b = 4;
            default     : alu_src_b = 0;
        endcase // case (src2_sel)
    end
endmodule // src_b_mux





module alu
(
    input      [3:0] op,
    input      [`XPR_LEN-1:0]      in1,
    input      [`XPR_LEN-1:0]      in2,
    output reg [`XPR_LEN-1:0]      out
);

    wire [`SHAMT_WIDTH-1:0] shamt;

    assign shamt = in2[`SHAMT_WIDTH-1:0];

    always @(*) begin
        case (op)
            `ALU_ADD  : out = in1 + in2;
            `ALU_SLL  : out = in1 << shamt;
            `ALU_XOR  : out = in1 ^ in2;
            `ALU_OR   : out = in1 | in2;
            `ALU_AND  : out = in1 & in2;
            `ALU_SRL  : out = in1 >> shamt;
            `ALU_SEQ  : out = {31'b0, in1 == in2};
            `ALU_SNE  : out = {31'b0, in1 != in2};
            `ALU_SUB  : out = in1 - in2;
            `ALU_SRA  : out = $signed(in1) >>> shamt;
            `ALU_SLT  : out = {31'b0, $signed(in1) < $signed(in2)};
            `ALU_SGE  : out = {31'b0, $signed(in1) >= $signed(in2)};
            `ALU_SLTU : out = {31'b0, in1 < in2};
            `ALU_SGEU : out = {31'b0, in1 >= in2};
            default      : out = 0;
        endcase // case op
    end
endmodule // alu









module ExeUnit_LdSt
(
    input                     i_clk,
    input                     i_resetn,
    input  [31:0]        ex_src1,
    input  [31:0]        ex_src2,
    input  [31:0]        PC,
    input  [31:0]        imm,
    input                     dstval,
    input  [4:0] spectag,
    input                     specbit,
    input                     issue,
    input                     predict_miss,
    input  [4:0] spectagfix,
    input  [`RRF_ADDRW-1:0]     rrftag,
    output [31:0]        result,
    output                    rrf_we,
    output                    rob_we, //set finish
    output [`RRF_ADDRW-1:0]     wrrftag,
    output                    kill_speculative,
    output                    busy_next,
    //Signal StoreBuf
    output                    stfin,
    //Store
    output                    memoccupy_ld,
    input                     fullsb,
    output [31:0]        storedata,
    output [31:0]        storeaddr,
    //Load
    input                     hitsb,
    output [31:0]        ldaddr,
    input  [31:0]        lddatasb,
    input  [31:0]        lddatamem
);

    reg                    busy;
    wire                   clearbusy;
    wire [31:0]       effaddr;
    wire                   killspec1;
   
    //LATCH
    reg                    dstval_latch;
    reg [`RRF_ADDRW-1:0]     rrftag_latch;
    reg                    specbit_latch;
    reg [4:0] spectag_latch;
    reg [31:0]        lddatasb_latch;
    reg                    hitsb_latch;
    reg                    insnvalid_latch;

    assign clearbusy        = (killspec1 || dstval || (~dstval && ~fullsb)) ? 1'b1 : 1'b0;
    assign killspec1        = ((spectag & spectagfix) != 0) && specbit && predict_miss;
    assign kill_speculative = ((spectag_latch & spectagfix) != 0) && specbit_latch && predict_miss;
    assign result           = hitsb_latch ? lddatasb_latch : lddatamem;
    assign rrf_we           = dstval_latch & insnvalid_latch;
    assign rob_we           = insnvalid_latch;
    assign wrrftag          = rrftag_latch;
    assign busy_next        = clearbusy ? 1'b0 : busy;
    assign memoccupy_ld     = ~killspec1 & busy & dstval;
    assign storedata        = ex_src2;
    assign storeaddr        = effaddr;
    assign ldaddr           = effaddr;
    assign effaddr          = ex_src1 + imm;
   
    always @(posedge i_clk) begin
        if (i_resetn | killspec1 | ~busy | (~dstval & fullsb)) begin
            dstval_latch    <= 0;
            rrftag_latch    <= 0;
            specbit_latch   <= 0;
            spectag_latch   <= 0;
            lddatasb_latch  <= 0;
            hitsb_latch     <= 0;
            insnvalid_latch <= 0;
        end else begin
            dstval_latch    <= dstval;
            rrftag_latch    <= rrftag;
            specbit_latch   <= specbit;
            spectag_latch   <= spectag;
            lddatasb_latch  <= lddatasb;
            hitsb_latch     <= hitsb;
            insnvalid_latch <= ~killspec1 & ( (busy & dstval) | (busy & ~dstval & ~fullsb) );
        end
    end // always @(posedge i_clk)

    always @(posedge i_clk) begin
        if (i_resetn | killspec1) begin
            busy <= 0;
        end else begin
            busy <= issue | busy_next;
        end
    end
endmodule // ExeUnit_LdSt





module ExeUnit_Mul
(
    input                     i_clk,
    input                     i_resetn,
    input  [31:0]        ex_src1,
    input  [31:0]        ex_src2,
    input                     dstval,
    input  [4:0] spectag,
    input                     specbit,
    input                     src1_signed,
    input                     src2_signed,
    input                     sel_lohi,
    input                     issue,
    input                     predict_miss,
    input  [4:0] spectagfix,
    output [31:0]        result,
    output                    rrf_we,
    output                    rob_we, //set finish
    output                    kill_speculative
);

    reg  busy;
   
    assign rob_we           = busy;
    assign rrf_we           = busy & dstval;
    assign kill_speculative = ((spectag & spectagfix) != 0) && specbit && predict_miss;
   
    always @(posedge i_clk) begin
        if (~i_resetn) begin
            busy <= 0;
        end else begin
            busy <= issue;
        end
    end
   
    multiplier bob
    (
        .src1(ex_src1),
        .src2(ex_src2),
        .src1_signed(src1_signed),
        .src2_signed(src2_signed),
        .sel_lohi(sel_lohi),
        .result(result)
    );
   
endmodule // ExeUnit_Mul


module mux_4x1
(
    input  [1:0]             sel,
    input  [2*31:0]     dat0,
    input  [2*31:0]     dat1,
    input  [2*31:0]     dat2,
    input  [2*31:0]     dat3,
    output reg [2*31:0] out
);
    always @(*) begin
        case(sel)
            0: begin
                out = dat0;
            end
            1: begin
                out = dat1;
            end
            2: begin
                out = dat2;
            end
            3: begin
                out = dat3;
            end
        endcase
    end
endmodule // mux_4x1

// sel_lohi = muldiv_out_sel[0]
module multiplier
(
    input  signed [31:0] src1,
    input  signed [31:0] src2,
    input                     src1_signed,
    input                     src2_signed,
    input                     sel_lohi,
    output        [31:0] result
);

    wire signed [32:0]     src1_unsign = {1'b0, src1};
    wire signed [32:0]     src2_unsign = {1'b0, src2};

    wire signed [2*31:0] res_ss = src1 * src2;
    wire signed [2*31:0] res_su = src1 * src2_unsign;
    wire signed [2*31:0] res_us = src1_unsign * src2;
    wire signed [2*31:0] res_uu = src1_unsign * src2_unsign;

    wire [2*31:0]  res;

    mux_4x1 mxres
	(
        .sel({src1_signed, src2_signed}),
        .dat0(res_uu),
        .dat1(res_us),
        .dat2(res_su),
        .dat3(res_ss),
        .out(res)
    );
   
    assign result = sel_lohi ? res[32+:32] : res[31:0];
   
endmodule // multiplier


module ExeUnit_Branch
(
    input                      i_clk,
    input                      i_resetn,
    input  [31:0]         ex_src1,
    input  [31:0]         ex_src2,
    input  [31:0]         PC,
    input  [31:0]         imm,
    input                      dstval,
    input  [3:0] alu_op,
    input  [4:0]  spectag,
    input                      specbit,
    input  [31:0]         praddr,
    input  [6:0]               opcode,
    input                      issue,
    output [31:0]         result,
    output                     rrf_we,
    output                     rob_we, //set finish
    output                     predict_hit,
    output                     predict_miss,
    output [31:0]         jmpaddr,
    output [31:0]         jmpaddr_taken,
    output                     brcond,
    output [4:0]  tag_fix
   );

    reg              busy;
   
    wire [31:0] comprslt;
    wire             addrmatch = (jmpaddr == praddr) ? 1'b1 : 1'b0;

   
    assign rob_we        = busy;
    assign rrf_we        = busy & dstval;
    assign result        = PC + 4;
    assign predict_hit     = busy & addrmatch;
    assign predict_miss        = busy & ~addrmatch;
    assign jmpaddr       = brcond ? jmpaddr_taken : (PC + 4);
    assign jmpaddr_taken = (((opcode == `JALR) ? ex_src1 : PC) + imm);
   
    assign brcond        = ((opcode == `JAL) || (opcode == `JALR)) ? 1'b1 : comprslt[0];
    assign tag_fix     = {spectag[0], spectag[4:1]};
   
    always @(posedge i_clk) begin
        if (~i_resetn) begin
            busy <= 0;
        end else begin
            busy <= issue;
        end
    end
          
    alu comparator
    (
        .op(alu_op),
        .in1(ex_src1),
        .in2(ex_src2),
        .out(comprslt)
    );
endmodule // ExeUnit_Branch


module tag_decoder
(
    input      [4:0] in,
    output reg [2:0]              out
);

    always @ (*) begin
        out = 0;
        case (in)
            5'b00001: out = 0;
            5'b00010: out = 1;
            5'b00100: out = 2;
            5'b01000: out = 3;
            5'b10000: out = 4;
            default : out = 0;
        endcase // case (in)
    end
endmodule // tag_decoder

module miss_prediction_fix_table
(
    input                         i_clk,
    input                         i_resetn,
    output reg [4:0] mpft_valid,
    input      [4:0] value_addr,
    output     [4:0] mpft_value,
    input                         predict_miss,
    input                         predict_hit,
    input      [4:0] prsuccess_tag,
    input      [4:0] setspec1_tag, //inst1_spectag
    input                         setspec1_en, //inst1_isbranch & ~inst1_inv
    input      [4:0] setspec2_tag,
    input                         setspec2_en
);

    reg [4:0]        value0;
    reg [4:0]        value1;
    reg [4:0]        value2;
    reg [4:0]        value3;
    reg [4:0]        value4;

    wire [2:0]                    val_idx;
   
    tag_decoder td(
        .in(value_addr),
        .out(val_idx)
    );
   
   
    assign mpft_value = { value4[val_idx], value3[val_idx], value2[val_idx], value1[val_idx], value0[val_idx] };

    wire [4:0] wdecdata = mpft_valid | (setspec1_en ? setspec1_tag : 0);

    wire [4:0] value0_wdec = ( ~setspec1_tag[0] || ~setspec1_en ? 5'b0 :
                                           ( setspec1_tag | ((setspec1_tag == 5'b00001) ? mpft_valid : 5'b0) ) ) |
                                          ( ~setspec2_tag[0] || ~setspec2_en ? 5'b0 :
                                           ( setspec2_tag | ((setspec2_tag == 5'b00001) ? wdecdata   : 5'b0) ) );
   
    wire [4:0] value1_wdec = (~setspec1_tag[1] || ~setspec1_en ? 5'b0 :
                                           (setspec1_tag | ((setspec1_tag == 5'b00010) ? mpft_valid : 5'b0))) |
                                          (~setspec2_tag[1] || ~setspec2_en ? 5'b0 :
                                           (setspec2_tag | ((setspec2_tag == 5'b00010) ? wdecdata : 5'b0)));
   
    wire [4:0] value2_wdec = (~setspec1_tag[2] || ~setspec1_en ? 5'b0 :
                                           (setspec1_tag | ((setspec1_tag == 5'b00100) ? mpft_valid : 5'b0))) |
                                          (~setspec2_tag[2] || ~setspec2_en ? 5'b0 :
                                           (setspec2_tag | ((setspec2_tag == 5'b00100) ? wdecdata : 5'b0)));
   
    wire [4:0] value3_wdec = (~setspec1_tag[3] || ~setspec1_en ? 5'b0 :
                                           (setspec1_tag | ((setspec1_tag == 5'b01000) ? mpft_valid : 5'b0))) |
                                          (~setspec2_tag[3] || ~setspec2_en ? 5'b0 :
                                           (setspec2_tag | ((setspec2_tag == 5'b01000) ? wdecdata : 5'b0)));
   
    wire [4:0] value4_wdec = (~setspec1_tag[4] || ~setspec1_en ? 5'b0 :
                                           (setspec1_tag | ((setspec1_tag == 5'b10000) ? mpft_valid : 5'b0))) |
                                          (~setspec2_tag[4] || ~setspec2_en ? 5'b0 :
                                           (setspec2_tag | ((setspec2_tag == 5'b10000) ? wdecdata : 5'b0)));
   
    wire [4:0] value0_wprs = (prsuccess_tag[0] ? 5'b0 : ~prsuccess_tag);
    wire [4:0] value1_wprs = (prsuccess_tag[1] ? 5'b0 : ~prsuccess_tag);
    wire [4:0] value2_wprs = (prsuccess_tag[2] ? 5'b0 : ~prsuccess_tag);
    wire [4:0] value3_wprs = (prsuccess_tag[3] ? 5'b0 : ~prsuccess_tag);
    wire [4:0] value4_wprs = (prsuccess_tag[4] ? 5'b0 : ~prsuccess_tag);
   
    always @(posedge i_clk) begin
        if (i_resetn | predict_miss) begin
            mpft_valid <= 0;
        end else if (predict_hit) begin
            mpft_valid <= mpft_valid & ~prsuccess_tag;
        end else begin
            mpft_valid <= mpft_valid | 
                         (setspec1_en ? setspec1_tag : 0) |
                         (setspec2_en ? setspec2_tag : 0);
        end
    end

    always @(posedge i_clk) begin
        if (i_resetn | predict_miss) begin
            value0 <= 0;
            value1 <= 0;
            value2 <= 0;
            value3 <= 0;
            value4 <= 0;
        end else begin
            value0 <= predict_hit ? (value0 & value0_wprs) : (value0 | value0_wdec);
            value1 <= predict_hit ? (value1 & value1_wprs) : (value1 | value1_wdec);
            value2 <= predict_hit ? (value2 & value2_wprs) : (value2 | value2_wdec);
            value3 <= predict_hit ? (value3 & value3_wprs) : (value3 | value3_wdec);
            value4 <= predict_hit ? (value4 & value4_wprs) : (value4 | value4_wdec);
        end
    end
   
endmodule // miss_prediction_fix_table