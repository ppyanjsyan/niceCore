
`define XLEN 32


//instruction memory


//Register File
`define REG_SEL 5
//`define REG_NUM 2**5
`define REG_NUM 32

//Instruction
`define IMM_TYPE_WIDTH 2
`define I_type 2'd0
`define S_type 2'd1
`define U_type 2'd2
`define J_type 2'd3

//Important Wire
`define DATA_LEN 32
`define INSN_LEN 32
`define ADDR_LEN 32
`define ISSUE_NUM 2
`define ENTRY_POINT `ADDR_LEN'h0
//`define REQDATA_LEN 2

//Decoder
`define RS_ENT_SEL 3
`define RS_ALU 1
`define RS_BRANCH 2
`define RS_JAL `RS_BRANCH
`define RS_JALR `RS_BRANCH
`define RS_MUL 3
`define RS_DIV 3
`define RS_LDST 4

//RS
`define ALU_ENT_SEL 3
`define ALU_ENT_NUM 8
`define BRANCH_ENT_SEL 2
`define GSH_PHT_DEPTH 4
`define LDST_ENT_SEL 2
`define LDST_ENT_NUM 4
//`define LDST_ENT_SEL 3
//`define LDST_ENT_NUM 8
`define MUL_ENT_SEL 1
`define MUL_ENT_NUM 2

//STOREBUFFER
`define STBUF_ENT_SEL 5
`define STBUF_ENT_NUM 32

//BTB
`define BTB_ADDRW 9
`define BTB_DEPTH 512
//`define BTB_DEPTH 2**`BTB_ADDRW
//`define BTB_TAG_LEN `ADDR_LEN-3-`BTB_ADDRW
`define BTB_TAG_LEN 20

//Gshare
`define GSH_BHR_WIDTH 10
`define GSH_PHT_ADDRW 10
`define GSH_PHT_DEPTH 1024
//`define GSH_PHT_DEPTH 2**`GSH_PHT_ADDRW

//TagGenerator

//`define SPECTAG_WIDTH 1+`GSH_PHT_DEPTH
`define SPECTAG_WIDTH 5
//`define BRDEPTH_LEN 5
`define BRDEPTH_LEN 5

//Re-Order Buffer
`define ROB_ADDRW 6
//`define ROB_DEPTH 2**`ROB_ADDRW
`define ROB_DEPTH 64
`define RRF_ADDRW `ROB_ADDRW
`define RRF_DEPTH `ROB_DEPTH

//src_a
`define SRC_A_SEL_WIDTH 2
`define SRC1_RS1  `SRC_A_SEL_WIDTH'd0
`define SRC1_PC   `SRC_A_SEL_WIDTH'd1
`define SRC1_ZERO `SRC_A_SEL_WIDTH'd2

//src_b
`define SRC_B_SEL_WIDTH 2
`define SRC2_RS2  `SRC_B_SEL_WIDTH'd0
`define SRC2_IMM  `SRC_B_SEL_WIDTH'd1
`define SRC2_FOUR `SRC_B_SEL_WIDTH'd2
`define SRC2_ZERO `SRC_B_SEL_WIDTH'd3

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

`define MD_OP_WIDTH 2
`define MUL `MD_OP_WIDTH'd0
`define DIV `MD_OP_WIDTH'd1
`define REM `MD_OP_WIDTH'd2

`define MD_OUT_SEL_WIDTH 2
`define OUT_LOW  `MD_OUT_SEL_WIDTH'd0
`define OUT_HIGH  `MD_OUT_SEL_WIDTH'd1
`define OUT_REM `MD_OUT_SEL_WIDTH'd2

