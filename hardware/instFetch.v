/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Create Date: 2021.12.23
 *  Description: instruction fetch stage of rv32Core
 */
`include "configure.h"

module instFetch
(
    input                       i_clk,
    input                       i_resetn,
    input  [31:0]               PC,
    output                      predict_cond,
    output [31:0]               nextPC,
    input                       inst_ready,
    input  [127:0]              rinst,
    output [31:0]               inst1,
    output [31:0]               inst2,
    output                      invalid2,
    
    input                       btbpht_we,
    input  [31:0]               btbpht_pc,
    input  [31:0]               btb_jmpdst,
    input                       pht_wcond,
    input  [4:0]                mpft_valid,//Miss Prediction Fix Table
    input  [`GSH_BHR_WIDTH-1:0] pht_bhr,
    input                       predict_miss,
    input                       predict_hit,
    input  [4:0]                predict_tag,
    output [`GSH_BHR_WIDTH-1:0] bhr,
    input  [4:0]                spectagnow,
    
);
    wire        hit;
    wire [31:0] predict_PC;
    
    assign      nextPC  = (hit && predict_cond) ? predict_PC : (invalid2 ? PC + 4 : PC + 8);    
    assign      invalid = PC[2];
   
    always @(*) begin
        case(PC[3:2])
            2'b00: begin
                inst1 = inst_ready ? rinst[31:0]  : 32'b0;
                inst2 = inst_ready ? rinst[63:32] : 32'b0;
            end
            2'b01: begin
                inst1 = inst_ready ? rinst[63:32] : 32'b0;
                inst2 = 32'b0;
            end
            2'b10: begin
                inst1 = inst_ready ? rinst[95:64] : 32'b0;
                inst2 = inst_ready ? rinst[127:96]: 32'b0;
            end
            2'b11: begin
                inst1 = inst_ready ? rinst[127:96]: 32'b0;
                inst2 = 32'b0;
            end
        endcase //case(PC[3:2])
    end

    BTB btb
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .pc(pc),
        .hit(hit),
        .jmpaddr(predict_PC),
        .we(btbpht_we),
        .jmpsrc(btbpht_pc),
        .jmpdst(btb_jmpdst),
        .invalid2(invalid2)
    );

    GShare_Predictor gshare_predictor
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
        .pc(pc),
        .hit_bht(hit),
        .predict_cond(predict_cond),
        .we(btbpht_we),
        .wcond(pht_wcond),
        .went(btbpht_pc[2+:`GSH_BHR_WIDTH] ^ pht_bhr),
        .mpft_valid(mpft_valid),
        .predict_miss(predict_miss),
        .predict_hit(predict_hit),
        .predict_tag(predict_tag),
        .bhr_master(bhr),
        .spectagnow(spectagnow)
    );
   
endmodule //instFetch




module BTB( //Branch Target Buffer
    input         i_clk,
    input         i_resetn,
    input  [31:0] PC,
    output        hit,
    output [31:0] jmpaddr,
    input         we,
    input  [31:0] jmpsrc,
    input  [31:0] jmpdst,
    input         invalid2
);

    reg  [`BTB_DEPTH-1:0] valid;    //valid table

    wire [31:0]           bia_data; //the PC of branch instruction
    wire [`BTB_ADDRW-1:0] waddr = jmpsrc[3+:`BTB_ADDRW];
    wire [31:0]           PC2   = PC + 4;
   
    wire                  hit1  = ((bia_data == PC)  &&  valid[PC[3+:`BTB_ADDRW]])             ? 1'b1 : 1'b0;
    wire                  hit2  = ((bia_data == PC2) && ~invalid2 && valid[pc[3+:`BTB_ADDRW]]) ? 1'b1 : 1'b0;
    assign                hit   = hit1 | hit2;
        
    always @(posedge i_clk) begin
        if(~i_resetn) begin
            valid <= `BTB_DEPTH'b0;
        end else begin
            if(we) begin
                valid[waddr] <= 1'b1;
            end
        end
    end
   
    BTB_RAM bia //branch instruction address table
    (
        .i_clk(i_clk),
        .raddr(PC[3+:`BTB_ADDRW]),
        .rdata(bia_data),
        .we(we),
        .waddr(waddr),
        .wdata(jmpsrc)
    );

    BTB_RAM bta //branch target address table
    (
        .i_clk(i_clk),
        .raddr(PC[3+:`BTB_ADDRW]),
        .rdata(jmpaddr),
        .we(we),
        .waddr(waddr),
        .wdata(jmpdst)        
    );
        
endmodule //BTB


module BTB_RAM 
(
    input                       i_clk,
    input      [`BTB_ADDRW-1:0] raddr,
    output reg [31:0]           rdata,
    input                       we,
    input      [`BTB_ADDRW-1:0] waddr,
    input      [31:0]           wdata,
);

    reg [31:0] btbmem [0:`BTB_DEPTH-1];

    always @(posedge i_clk) begin
        rdata <= btbmem[raddr];
        if(we)
            btbmem[waddr] <= wdata;
    end
endmodule //BTB_RAM




//PRTAG is tag of predict_miss/success branch instruction's
module GShare_Predictor
(
    input                           i_clk,
    input                           i_resetn,
    input      [31:0]               PC,
    input                           hit_bht,
    output                          predict_cond,
    input                           we,
    input                           wcond,
    input      [`GSH_PHT_ADDRW-1:0] went,
    input      [4:0] mpft_valid,
    input                           predict_miss,
    input                           predict_hit,
    input      [4:0]                predict_tag,
    output reg [`GSH_BHR_WIDTH-1:0] bhr_master,
    input      [4:0]                spectagnow
);

    //5 branch history registers
    reg  [`GSH_BHR_WIDTH-1:0] bhr0;
    reg  [`GSH_BHR_WIDTH-1:0] bhr1;
    reg  [`GSH_BHR_WIDTH-1:0] bhr2;
    reg  [`GSH_BHR_WIDTH-1:0] bhr3;
    reg  [`GSH_BHR_WIDTH-1:0] bhr4;
    reg  [`GSH_BHR_WIDTH-1:0] bhr_fix;
    wire [1:0]                rif;
    wire [1:0]                rex;
    wire [1:0]                wex;
    wire [2:0]                wex_calc;
   
    always @(*) begin
        case (predict_tag)
            5'b00001 : bhr_fix = bhr0;
            5'b00010 : bhr_fix = bhr1;
            5'b00100 : bhr_fix = bhr2;
            5'b01000 : bhr_fix = bhr3;
            5'b10000 : bhr_fix = bhr4;
            default  : bhr_fix = `GSH_BHR_WIDTH'b0;
        endcase //case(predict_tag)
    end
   
   
    always @(posedge i_clk) begin
        if (~i_resetn) begin
            bhr0 <= 0;
            bhr1 <= 0;
            bhr2 <= 0;
            bhr3 <= 0;
            bhr4 <= 0;
            bhr_master <= 0;
        end else if (predict_miss) begin
            bhr0 <= bhr_fix;
            bhr1 <= bhr_fix;
            bhr2 <= bhr_fix;
            bhr3 <= bhr_fix;
            bhr4 <= bhr_fix;
            bhr_master <= bhr_fix;
        end else if (predict_hit) begin
            bhr0 <= (predict_tag == 5'b00001) ? {bhr_master[`GSH_BHR_WIDTH-2:0], predict_cond} : bhr0; 
            bhr1 <= (predict_tag == 5'b00010) ? {bhr_master[`GSH_BHR_WIDTH-2:0], predict_cond} : bhr1;
            bhr2 <= (predict_tag == 5'b00100) ? {bhr_master[`GSH_BHR_WIDTH-2:0], predict_cond} : bhr2;
            bhr3 <= (predict_tag == 5'b01000) ? {bhr_master[`GSH_BHR_WIDTH-2:0], predict_cond} : bhr3;
            bhr4 <= (predict_tag == 5'b10000) ? {bhr_master[`GSH_BHR_WIDTH-2:0], predict_cond} : bhr4;
        end else if (hit_bht) begin
            if (we & mpft_valid[0]) begin
                bhr0       <= {bhr0[`GSH_BHR_WIDTH-2:0], predict_cond};
            end
            if (we & mpft_valid[1]) begin
                bhr1       <= {bhr1[`GSH_BHR_WIDTH-2:0], predict_cond};
            end
            if (we & mpft_valid[2]) begin
                bhr2       <= {bhr2[`GSH_BHR_WIDTH-2:0], predict_cond};
            end
            if (we & mpft_valid[3]) begin
                bhr3       <= {bhr3[`GSH_BHR_WIDTH-2:0], predict_cond};
            end
            if (we & mpft_valid[4]) begin
                bhr4       <= {bhr4[`GSH_BHR_WIDTH-2:0], predict_cond};
            end
            if (we) begin
                bhr_master <= {bhr_master[`GSH_BHR_WIDTH-2:0], predict_cond};
            end
        end
    end
   
    assign predict_cond = (hit_bht && (rif > 2'b01)) ? 1'b1 : 1'b0;
    assign wex_calc     = {1'b0, rex} + (wcond ? 3'b001 : 3'b111);
    assign wex          = ((rex == 2'b00) && ~wcond) ? 2'b00 : ((rex == 2'b11) && wcond) ? 2'b11 : wex_calc[1:0];

    PHT pht
    (
        .i_clk      (i_clk),
        .raddr_if (PC[2+:`GSH_BHR_WIDTH] ^ bhr_master),
        .rdata_if (rif),
        .raddr_ex (went),
        .rdata_ex (rex),
        .we_ex    (we),
        .waddr_ex (went),
        .wdata_ex (wex)        
    );
endmodule //GShare_Predictor

module PHT
(
    input                       i_clk,
    input  [`GSH_PHT_ADDRW-1:0] raddr_if,
    output [1:0]                rdata_if,
    input  [`GSH_PHT_ADDRW-1:0] raddr_ex,
    output [1:0]                rdata_ex,
    input                       we_ex,
    input  [`GSH_PHT_ADDRW-1:0] waddr_ex,
    input  [1:0]             	wdata_ex    
);
   
    PHT_RAM pht0
    (
        .clka   (i_clk),
        .wea    (1'b0),
        .addra  (raddr_if),
        .rdataa (rdata_if),
        .wdataa (),        
        .clkb   (i_clk),
        .web    (we_ex),
        .addrb  (waddr_ex),
        .rdatab (),
        .wdatab (wdata_ex)
    );
   
    PHT_RAM pht1
    (
        .clka   (i_clk),
        .wea    (1'b0),
        .addra  (raddr_ex),
        .rdataa (rdata_ex),
        .wdataa (),
        .clkb   (i_clk),
        .web    (we_ex),
        .addrb  (waddr_ex),
        .rdatab (),
        .wdatab (wdata_ex)
    );
   
endmodule //PHT


module PHT_RAM
(
    input                       clka,
    input                       wea,
    input      [`PHT_ADDRW-1:0] addra,
    output reg [1:0]            rdataa,
    input      [1:0]            wdataa,
    
    input                       clkb,
    input                       web
    input      [`PHT_ADDRW-1:0] addrb,
    output reg [1:0]            rdatab,
    input      [1:0]            wdatab    
);

    reg [1:0] phtmem [0:`PHT_DEPTH-1];
   
    always @(posedge clka) begin
        rdataa <= phtmem[addra];
        if (wea) begin
            phtmem[addra] <= wdataa;
        end
    end

    always @(posedge clkb) begin
        rdatab <= phtmem[addrb];
        if (web) begin
            phtmem[addrb] <= wdatab;
        end
    end
    
endmodule //PHT_RAM



