//`define ENABLE_MULDIV      /* Uncomment to support RV32M decoding. */

//program address
`define PROGADDR_RESET  32'h00000000
`define PROGADDR_IRQ    32'h00000010

//instruction memory，the instruction memory capacity = 32bit*4*1024 = 16KB
`define IMEM_DEPTH 1024
`define IMEM_ADDRW 10
`define INST_MEM_INIT_FILE ""

`define ARF_INIT_ZERO
`define RRF_INIT_ZERO

//data memory, the data memory capacity = 32bit*4096 = 16KB
`define DMEM_DEPTH 4096
`define DMEM_ADDRW 12
`define DATA_MEM_INIT_FILE ""

//BTB
`define BTB_DEPTH 512
`define BTB_ADDRW 9

//PHT
`define PHT_DEPTH 1024
`define PHT_ADDRW 10
`define GSH_PHT_DEPTH 1024
`define GSH_PHT_ADDRW 10
`define GSH_BHR_WIDTH 10

//TagGenerator
//`define SPECTAG_WIDTH 1+`BRANCH_ENT_NUM
`define SPECTAG_WIDTH 5
//`define BRDEPTH_LEN 5
`define BRDEPTH_LEN 5


//Decoder
`define RS_ALU 1
`define RS_BRANCH 2
`define RS_JAL 2
`define RS_JALR 2
`define RS_MUL 3
`define RS_DIV 3
`define RS_LDST 4


//Re-Order Buffer
`define ROB_DEPTH 64
`define ROB_ADDRW 6

//Renaming Register File
`define RRF_DEPTH 64
`define RRF_ADDRW 6




//Immediate type
`define R_type 3'd1
`define I_type 3'd2
`define S_type 3'd3
`define B_type 3'd4
`define U_type 3'd5
`define J_type 3'd6

//src1
`define SRC1_RS1  2'd0
`define SRC1_PC   2'd1
`define SRC1_ZERO 2'd2

//src2
`define SRC2_RS2  2'd0
`define SRC2_IMM  2'd1
`define SRC2_FOUR 2'd2
`define SRC2_ZERO 2'd3

`define MEM_TYPE_WIDTH 3
`define MEM_TYPE_LB  `MEM_TYPE_WIDTH'd0
`define MEM_TYPE_LH  `MEM_TYPE_WIDTH'd1
`define MEM_TYPE_LW  `MEM_TYPE_WIDTH'd2
`define MEM_TYPE_LD  `MEM_TYPE_WIDTH'd3
`define MEM_TYPE_LBU `MEM_TYPE_WIDTH'd4
`define MEM_TYPE_LHU `MEM_TYPE_WIDTH'd5
`define MEM_TYPE_LWU `MEM_TYPE_WIDTH'd6

`define MEM_TYPE_SB  `MEM_TYPE_WIDTH'd0
`define MEM_TYPE_SH  `MEM_TYPE_WIDTH'd1
`define MEM_TYPE_SW  `MEM_TYPE_WIDTH'd2
`define MEM_TYPE_SD  `MEM_TYPE_WIDTH'd3

//RV32M
`define MUL 2'd0
`define DIV 2'd1
`define REM 2'd2
`define OUT_LOW  2'd0
`define OUT_HIGH 2'd1
`define OUT_REM  2'd2



//ALU operation
`define ALU_ADD  4'd0
`define ALU_SLL  4'd1
`define ALU_XOR  4'd4
`define ALU_OR   4'd6
`define ALU_AND  4'd7
`define ALU_SRL  4'd5
`define ALU_SEQ  4'd8
`define ALU_SNE  4'd9
`define ALU_SUB  4'd10
`define ALU_SRA  4'd11
`define ALU_SLT  4'd12
`define ALU_SGE  4'd13
`define ALU_SLTU 4'd14
`define ALU_SGEU 4'd15





// Width-related constants
`define INST_WIDTH     32
`define REG_ADDR_WIDTH  5
`define XPR_LEN        32
`define DOUBLE_XPR_LEN 64
`define LOG2_XPR_LEN    5
`define SHAMT_WIDTH     5

`define RV_NOP `INST_WIDTH'b0010011





// PRIV FUNCT12 encodings

`define RV32_FUNCT12_ECALL  12'b000000000000
`define RV32_FUNCT12_EBREAK 12'b000000000001
`define RV32_FUNCT12_ERET   12'b000100000000

// RV32M encodings
`define RV32_FUNCT7_MUL_DIV 7'd1

`define RV32_FUNCT3_MUL    3'd0
`define RV32_FUNCT3_MULH   3'd1
`define RV32_FUNCT3_MULHSU 3'd2
`define RV32_FUNCT3_MULHU  3'd3
`define RV32_FUNCT3_DIV    3'd4
`define RV32_FUNCT3_DIVU   3'd5
`define RV32_FUNCT3_REM    3'd6
`define RV32_FUNCT3_REMU   3'd7



















/**************************************************************************************************/
/* Many-core processor project Arch Lab.                                               TOKYO TECH */
/**************************************************************************************************/
//`default_nettype none
/**************************************************************************************************/

/* Clock Frequency Definition                                                                     */
/* Clock Freq = (System Clock Freq) * (DCM_CLKFX_MULTIPLY) / (DCM_CLKFX_DIVIDE)                   */
/**************************************************************************************************/
`define SYSTEM_CLOCK_FREQ   200    // Atlys -> 100 MHz, Nexys4 -> 100 MHz, Virtex7 -> 200 MHz

`define DCM_CLKIN_PERIOD    5.000  // Atlys -> 10.000 ns
`define DCM_CLKFX_MULTIPLY  3      // CLKFX_MULTIPLY must be 2~32
`define DCM_CLKFX_DIVIDE    25     // CLKFX_DIVIDE   must be 1~32

`define MMCM_CLKIN1_PERIOD  5.000  // Nexys4 -> 10.000 ns, Virtex7 -> 5.000 ns
`define MMCM_VCO_MULTIPLY   8      // for VCO, 800-1600
`define MMCM_VCO_DIVIDE     2
`define MMCM_CLKOUT0_DIVIDE 16     // for user clock, 50  MHz
`define MMCM_CLKOUT1_DIVIDE 4      // for dram clock, 200 MHz


/* UART Definition                                                                                */
/**************************************************************************************************/
`define SERIAL_WCNT  'd50       // 24MHz/1.5Mbaud, parameter for UartRx and UartTx
`define APP_SIZE        8*1024  // application program load size in byte (64*16=1024KB)
`define SCD_SIZE       64*1024  // scheduling program  load size in byte (64* 1=  64KB)
`define IMG_SIZE     1088*1024  // full image file     load size in byte (64*17=1088KB)


/**************************************************************************************************/
//`default_nettype wire
/**************************************************************************************************/
