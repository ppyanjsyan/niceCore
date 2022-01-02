/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Create Date: 2021.12.22
 *  Description: rv32CoreTop module of rv32Core
 */
`include "configure.h"

module rv32CoreTop
(
    input i_clk,
    input i_resetn
);
	wire                       pc_valid,
    wire [31:0]           PC,
	wire                       inst_ready,
    wire [32*4-1:0] rinst,
	
    wire                       data_valid,
	wire                       we,
	wire [31:0]           addr,
    wire [31:0]           wdata,
	wire                       data_ready,
    wire [31:0]           rdata

    rv32Core nicecore
    (
        .i_clk(i_clk),
        .i_resetn(i_resetn),
		.pc_valid(pc_valid),
        .PC(PC),
		.inst_ready(inst_ready),
        .rinst(rinst),
		.data_valid(data_valid),
		.we(we),
		.addr(addr),
        .wdata(wdata),
        .data_ready(data_ready),		
        .rdata(rdata)
    );

    instMEM instmem
	(
		.i_clk(i_clk),
		.i_resetn(i_resetn),
		.valid(pc_valid),
		.raddr(PC[`IMEM_ADDRW-1+4:4]),
		.ready(inst_ready),
		.rinst(rinst)
    );
    
    dataMEM datamem
	(
		.i_clk(i_clk),
		.i_resetn(i_resetn),
		.valid(data_valid),
		.we(we),
		.addr(addr[`DMEM_ADDRW-1:0]),
		.wdata(wdata),
		.ready(data_ready),
		.rdata(rdata)
    );
	
);
endmodule //rv32CoreTop
