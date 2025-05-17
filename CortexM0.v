module CortexM0 (
	input	wire		      CLK,
	input	wire	        RESET_N, // reset when negative
	
	// For instruction memory
	output wire	    		IREQ,
	output wire [31:0]	IADDR,
	input	wire  [31:0]	INSTR,

	// For data memory
	output wire	    		DREQ,
	output wire	[31:0]	DADDR,
	output wire	  			DRW,
	output wire	[ 1:0]	DSIZE,
	input	wire	[31:0]	DIN,
	output wire	[31:0]	DOUT
);


REGFILE REGFILE (
  .CLK(clk_i),
  .nRST(RESET_N),
  .WEN1(wb_rf_wr_a_en),
  .WA1(wb_rf_wr_a_addr), 
  .DI1(wb_wr_data), 
  .WEN2(wb_rf_wr_b_en),
  .WA2(wb_rf_wr_b_addr),
  .DI2(wb_pc_addr),
  .RA0(id_rf_rd_a_addr),
  .RA1(id_rf_rd_b_addr),
  .RA2(id_rf_rd_c_addr),
  .DOUT0(rf_reg_a_data),
  .DOUT1(rf_reg_b_data),
  .DOUT2(rf_reg_c_data)
);



// your code here

//------------------------------------------------------------
// FETCH ↔ MEM-stage interface (PC-related control)
//------------------------------------------------------------
wire [31:0] addr_branch;   // From EXE (ALU) → IF.  Target PC computed for
                           // “PC = PC + imm” or conditional branch/jump.  
                           // Always half-word aligned → bit 0 == 0.

wire [31:0] addr_reg;      // From EXE (reg C) → IF.  Register-indirect jump
                           // target (e.g. BX/BLX Rs).  Any 32-bit value,
                           // but in Thumb state LSB must be 1; IF masks it.

wire [1:0]  addr_sel;      // From MEM → IF.  PC-mux select:
                           // 00 = sequential PC+2
                           // 01 = addr_branch
                           // 10 = addr_reg
                           // 11 = reserved (treated as sequential)

//------------------------------------------------------------
// Outputs of IF stage
//------------------------------------------------------------
wire [15:0] if_instr_reg;  // Current half-word in IF/ID pipeline reg.
                           // 16-bit Thumb opcode (little-endian view).

wire [15:0] if_instr_noreg;// Next half-word straight from memory
                           // (needed for 32-bit-decode look-ahead).

wire [31:0] if_pc_addr;    // Address of if_instr_reg (word-aligned, bit0=0).

//------------------------------------------------------------
// Register-file read ports (combinational)
//------------------------------------------------------------
wire [31:0] rf_reg_a_data; // Rn   value read in ID
wire [31:0] rf_reg_b_data; // Rm   value read in ID
wire [31:0] rf_reg_c_data; // optional “Rc”/stack data (e.g. STR/LDR)

//------------------------------------------------------------
// Forwarding control (ID → EXE)
//------------------------------------------------------------
wire [1:0]  fwd_mux_a;     // 00 = use reg-file value  
                           // 01 = forward MEM/WB  
                           // 10 = forward EXE/MEM  
                           // 11 = reserved
wire [1:0]  fwd_mux_b;     // same encoding as fwd_mux_a

//------------------------------------------------------------
// ID-stage outputs (latched in ID/EXE reg)
//------------------------------------------------------------
wire [3:0] id_rf_rd_a_addr;// Source A register index  (0-15)
wire [3:0] id_rf_rd_b_addr;// Source B register index
wire [3:0] id_rf_rd_c_addr;// Extra/source C (e.g. base for STR/LDR)
wire [3:0] id_rf_wr_a_addr;// Dest A  register index
wire [3:0] id_rf_wr_b_addr;// Dest B  (LR for BL, etc.)
wire        id_rf_wr_a_en; // 1 → write RF dest A in WB
wire        id_rf_wr_b_en; // 1 → write RF dest B in WB
wire [31:0] id_imm;        // Sign/zero-extended immediate operand
wire [31:0] id_rf_reg_a_data; // bypassed source values
wire [31:0] id_rf_reg_b_data;
wire [31:0] id_rf_reg_c_data;
wire [3:0]  id_br_type;    // Condition code for B<cond> 0000…1110
wire [1:0]  id_br_en;      // 00 = none, 01 = jump ALU, 11 = jump REG
wire [31:0] id_pc_addr;    // PC of current instr (for ADD PC, imm etc.)
wire [4:0]  id_alu_op;     // Encodings defined in `ALU_*` macros
wire [1:0]  id_alu_a_sel;  // 00 = Reg A, 01 = PC, others reserved
wire [1:0]  id_alu_b_sel;  // 00 = Reg B, 01 = Imm, 10 = PC
wire        id_mem_write;  // 1 → data-mem write (STR,…)
wire        id_mem_cs;     // 0 = mem active, 1 = chip-select inactive
wire [1:0]  id_mem_be;     // 00 = byte, 01 = half, 10 = word
wire        id_mem_signed; // 1 → sign-extend load result
wire        id_wb_sel;     // 0 = ALU → RF, 1 = MEM → RF
wire        id_apsr_wr_en; // Update Z/N/C/V flags
wire        id_Z,id_N,id_C,id_V; // Current APSR flags forwarded

//------------------------------------------------------------
// EXE-stage outputs (latched in EXE/MEM reg)
//------------------------------------------------------------
wire [15:0] exe_instr;      // Opcode in EXE

wire        exe_rf_wr_a_en; // Dest A write-enable (propagated)
wire [3:0]  exe_rf_wr_a_addr;
wire        exe_rf_wr_b_en;
wire [3:0]  exe_rf_wr_b_addr;

wire [31:0] exe_alu_result; // ALU result or computed memory address
wire [3:0]  exe_br_type;
wire [1:0]  exe_br_en;
wire [31:0] exe_reg_c_data; // Store-data for STR* instructions
wire [31:0] exe_pc_addr;    // PC value forwarded for WB-to-LR etc.
wire        exe_wb_sel;     // Same encoding as id_wb_sel
wire        exe_mem_write;
wire        exe_mem_cs;
wire [1:0]  exe_mem_be;
wire        exe_mem_signed;
wire        exe_apsr_wr_en;
wire        exe_Z, exe_N, exe_C, exe_V; // Updated flags

//------------------------------------------------------------
// MEM-stage outputs (latched in MEM/WB reg)
//------------------------------------------------------------
wire [15:0] mem_instr;      // Opcode in MEM

wire        mem_rf_wr_a_en;
wire [3:0]  mem_rf_wr_a_addr;
wire        mem_rf_wr_b_en;
wire [3:0]  mem_rf_wr_b_addr;

wire [31:0] mem_alu_result; // Pass-through of exe_alu_result
wire [31:0] mem_rdata;      // Data loaded from memory (after sign-ext.)
wire [31:0] mem_pc_addr;    // PC for possible PC-writeback
wire [1:0]  mem_pc_sel;     // Same encoding as addr_sel
wire        mem_wb_sel;     // Select between mem_rdata / alu_result
wire        mem_apsr_wr_en;
wire        mem_Z, mem_N, mem_C, mem_V;

//------------------------------------------------------------
// WB-stage outputs (final write-back to RF)
//------------------------------------------------------------
wire [15:0] wb_instr;       // Opcode in WB

wire [31:0] wb_wr_data;     // Data actually written to register file
wire        wb_rf_wr_a_en;
wire [3:0]  wb_rf_wr_a_addr;
wire [31:0] wb_pc_addr;     // Value written to R15 when BL/POP PC etc.
wire        wb_rf_wr_b_en;
wire [3:0]  wb_rf_wr_b_addr;
wire        wb_apsr_wr_en;
wire        wb_Z, wb_N, wb_C, wb_V;

//------------------------------------------------------------
// Global control / housekeeping
//------------------------------------------------------------
reg  [2:0] clk_prescaler;   // Generates slow “clk_i” = CLK/4 for core
wire [15:0] instr_word;     // Half-word selected from 32-bit IMEM data
wire        clk_i;          // Prescaled core clock (clk_prescaler[1])
wire        reset_i;        // Active-high reset (== ~RESET_N)
wire        stall;          // Load-use hazard signal (stall IF & ID)
wire        flush;          // Pipeline flush on taken branch/jump


`define   ALU_AND  5'b00000
`define   ALU_EOR  5'b00001
`define   ALU_LSL  5'b00010
`define   ALU_LSR  5'b00011
`define   ALU_ASR  5'b00100
`define   ALU_ADC  5'b00101
`define   ALU_SBC  5'b00110
`define   ALU_ROR  5'b00111
`define   ALU_TST  5'b01000
`define   ALU_RSB  5'b01001
`define   ALU_CMP  5'b01010
`define   ALU_CMN  5'b01011
`define   ALU_ORR  5'b01100
`define   ALU_MUL  5'b01101
`define   ALU_BIC  5'b01110
`define   ALU_MVN  5'b01111
`define   ALU_ADD  5'b10000
`define   ALU_MOVA 5'b10001
`define   ALU_MOVB 5'b10010
`define   ALU_SXTH 5'b10011
`define   ALU_SXTB 5'b10100 
`define   ALU_REV 5'b10101 
`define   ALU_REV16  5'b10110 
`define   ALU_REVSH 5'b10111 
`define   ALU_SUB   5'b11000
`define   ALU_UXTH  5'b11001 
`define   ALU_UXTB  5'b11010


`define   ALU_A_SEL_REG  2'b00
`define   ALU_A_SEL_PC  2'b01
`define   ALU_B_SEL_REG  2'b00
`define   ALU_B_SEL_IMM  2'b01
`define   ALU_B_SEL_PC  2'b10

`define   ALU_NO_CARRY  1'b0
`define   ALU_USE_CARRY  1'b1

`define   DEC_IMM  6'b00xxxx
`define   DEC_LSL_IMM  5'b000xx
`define   DEC_MOV_REG_T2  5'b00000
`define   DEC_LSR_IMM  5'b001xx
`define   DEC_ASR_IMM  5'b010xx
`define   DEC_ADD_IREG  5'b01100
`define   DEC_SUB_IREG  5'b01101
`define   DEC_ADD_IMM3  5'b01110
`define   DEC_SUB_IMM3  5'b01111
`define   DEC_MOV_IMM  5'b100xx
`define   DEC_CMP_IMM  5'b101xx
`define   DEC_ADD_IMM8  5'b110xx
`define   DEC_SUB_IMM8  5'b111xx

`define  DEC_DATA_PROC  6'b010000
`define  DEC_AND_REG  4'b0
`define  DEC_EOR_REG  4'b0001
`define  DEC_LSL_REG  4'b0010
`define  DEC_LSR_REG  4'b0011
`define  DEC_ASR_REG  4'b0100
`define  DEC_ADC_REG  4'b0101
`define  DEC_SBC_REG  4'b0110
`define  DEC_ROR_REG  4'b0111
`define  DEC_TST_REG  4'b1000
`define  DEC_RSB_REG  4'b1001
`define  DEC_CMP_REG  4'b1010
`define  DEC_CMN_REG  4'b1011
`define  DEC_ORR_REG  4'b1100
`define  DEC_MUL_REG  4'b1101
`define  DEC_BIC_REG  4'b1110
`define  DEC_MVN_REG  4'b1111

`define  DEC_DS_BEX  6'b010001
`define DEC_ADD_REG  4'b00xx
`define DEC_CMP_1  4'b0101
`define DEC_CMP_2  4'b011x
`define DEC_MOV_REG  4'b10xx
`define DEC_BX  4'b110x
`define DEC_BLX  4'b111x

`define DEC_LDR_LIT  6'b01001x

`define  DEC_LS_1   6'b0101xx
`define DEC_STR_REG  3'b000
`define DEC_STRH_REG  3'b001
`define DEC_STRB_REG  3'b010
`define DEC_LDRSB_REG  3'b011
`define DEC_LDR_REG  3'b100
`define DEC_LDRH_REG  3'b101
`define DEC_LDRB_REG  3'b110
`define DEC_LDRSH_REG  3'b111

`define  DEC_LS_2   6'b011xxx
`define DEC_STR_IMM  4'b00xx
`define DEC_LDR_IMM  4'b01xx
`define DEC_STRB_IMM  4'b10xx
`define DEC_LDRB_IMM  4'b11xx

`define  DEC_LS_3   6'b100xxx
`define DEC_STRH_IMM  4'b00xx
`define DEC_LDRH_IMM  4'b01xx
`define DEC_STR_SP  4'b10xx
`define DEC_LDR_SP  4'b11xx


`define  DEC_PC_REL  6'b10100x

`define  DEC_SP_REL  6'b10101x

`define  DEC_MISC_16  6'b1011xx
`define DEC_ADD_SPIMM  7'b00000xx
`define DEC_SUB_SPIMM  7'b00001xx
`define DEC_SXTH  7'b001000x
`define DEC_SXTB  7'b001001x
`define DEC_UXTH  7'b001010x
`define DEC_UXTB  7'b001011x
`define DEC_PUSH  7'b010xxxx
`define DEC_CPS  7'b0110011 //UNUSED
`define DEC_REV  7'b101000x
`define DEC_REV16  7'b101001x
`define DEC_REVSH  7'b101011x
`define DEC_POP  7'b110xxxx
`define DEC_BKPT  7'b1110xxx //UNUSED
`define DEC_HINT  7'b1111xxx //UNUSED

`define  DEC_STM  6'b11000x

`define  DEC_LDM  6'b11001x


`define DEC_B_COND  6'b1101xx
`define B_EQ  4'b0000
`define B_NE  4'b0001
`define B_CS  4'b0010
`define B_CC  4'b0011
`define B_MI  4'b0100
`define B_PL  4'b0101
`define B_VS  4'b0110
`define B_VC  4'b0111
`define B_HI  4'b1000
`define B_LS  4'b1001
`define B_GE  4'b1010
`define B_LT  4'b1011
`define B_GT  4'b1100
`define B_LE  4'b1101
`define B_AL  4'b1110


`define  DEC_B_UC   6'b11100x



`define  DEC_32B_1   6'b11101x
`define  DEC_32B_2   6'b11110x
`define DEC_BL  3'b1x1
`define  DEC_32B_3   6'b11111x 

`define SREG_NO_CHANGE  1'b0
`define SREG_WRITE  1'b1

`define BR_DIS  1'b0
`define BR_EN  1'b1

`define PC_RESET_VECTOR 32'h000000D0
`define PC_SEL_NEXT 2'b00
`define PC_SEL_BRANCH 2'b01
`define PC_SEL_REG  2'b10

`define BR_NO_BRANCH 2'b00
`define BR_JUMP_ALU 2'b01
`define BR_JUMP_REG 2'b11

`define WB_SEL_ALU  1'b0
`define WB_SEL_MEM  1'b1

`define WB_NO_WRITE  1'b0
`define WB_WRITE  1'b1

`define MEM_CSN_ACTIVE  1'b0
`define MEM_CSN_INACTIVE  1'b1
`define MEM_WE_READ  1'b0
`define MEM_WE_WRITE  1'b1

`define MEM_BE_BYTE  2'b00
`define MEM_BE_HALFWORD  2'b01
`define MEM_BE_WORD  2'b10

`define FWD_MUX_REG 2'b00
`define FWD_MUX_EXE_MEM 2'b10
`define FWD_MUX_MEM_WB 2'b01




//======================= Forwarding Unit =======================
ForwardUnit fwd (
    // ─── Inputs ───────────────────────────────────────────────
    .exe_mem_rd_i   (exe_rf_wr_a_addr),
    .mem_wb_rd_i    (mem_rf_wr_a_addr),
    .id_exe_rs1_i   (id_rf_rd_a_addr),
    .id_exe_rs2_i   (id_rf_rd_b_addr),
    .exe_wb_a_en_i  (exe_rf_wr_a_en),
    .mem_wb_a_en_i  (mem_rf_wr_a_en),

    // ─── Outputs ──────────────────────────────────────────────
    .fwd_mux_a_o    (fwd_mux_a),
    .fwd_mux_b_o    (fwd_mux_b)
);


//################### PIPELINE STAGES ###################

//Instruction halfword selector
assign instr_word = IADDR[1] ? INSTR[31:16] : INSTR[15:0];
assign clk_i = clk_prescaler[1];
assign reset_i = RESET_N;

/*
  Stall and flush signals for pipeline control, clock prescaler
*/
assign stall = (((exe_mem_cs == `MEM_CSN_ACTIVE) && (exe_mem_write == `MEM_WE_READ)) &&  (exe_rf_wr_a_en == `WB_WRITE) && ((exe_rf_wr_a_addr == id_rf_rd_a_addr) || (exe_rf_wr_a_addr == id_rf_rd_b_addr)));

assign flush = ((mem_pc_sel == `PC_SEL_BRANCH) || (mem_pc_sel == `PC_SEL_REG)); 

always @(posedge CLK or negedge RESET_N) begin
  if(~RESET_N) clk_prescaler <= 3'b0;
  else clk_prescaler <= clk_prescaler + 3'b001;
end

//==================== Instruction-Fetch Stage ===================
InstructionFetchStage IF (
    // ─── Inputs ───────────────────────────────────────────────
    .clk_i          (clk_i),
    .reset_i        (reset_i),
    .stall_i        (stall),
    .flush_i        (flush),
    .instr_mem_i    (instr_word),
    .addr_branch_i  (exe_alu_result),
    .addr_reg_i     (exe_reg_c_data),
    .addr_sel_i     (mem_pc_sel),

    // ─── Outputs ──────────────────────────────────────────────
    .instr_reg_o    (if_instr_reg),
    .instr_noreg_o  (if_instr_noreg),
    .pc_addr_o      (if_pc_addr),
    .imem_addr_o    (IADDR),
    .imem_req_o     (IREQ)
);

//=================== Instruction-Decode Stage ===================
InstructionDecodeStage ID (
    // ─── Inputs ───────────────────────────────────────────────
    .clk_i          (clk_i),
    .reset_i        (reset_i),
    .stall_i        (stall),
    .flush_i        (flush),
    .instr_i        (if_instr_reg),
    .instr_noreg_i  (if_instr_noreg),
    .pc_addr_i      (if_pc_addr),
    .apsr_wr_en_i   (wb_apsr_wr_en),
    .Z_new_i        (wb_Z),
    .N_new_i        (wb_N),
    .C_new_i        (wb_C),
    .V_new_i        (wb_V),
    .reg_a_data_i   (rf_reg_a_data),
    .reg_b_data_i   (rf_reg_b_data),
    .reg_c_data_i   (rf_reg_c_data),

    // ─── Outputs ──────────────────────────────────────────────
    .rf_rd_a_addr_o (id_rf_rd_a_addr),
    .rf_rd_b_addr_o (id_rf_rd_b_addr),
    .rf_rd_c_addr_o (id_rf_rd_c_addr),
    .rf_wr_a_en_o   (id_rf_wr_a_en),
    .rf_wr_a_addr_o (id_rf_wr_a_addr),
    .rf_wr_b_en_o   (id_rf_wr_b_en),
    .rf_wr_b_addr_o (id_rf_wr_b_addr),
    .imm_o          (id_imm),
    .instr_o        (exe_instr),
    .reg_a_data_o   (id_rf_reg_a_data),
    .reg_b_data_o   (id_rf_reg_b_data),
    .reg_c_data_o   (id_rf_reg_c_data),
    .br_en_o        (id_br_en),
    .br_type_o      (id_br_type),
    .pc_addr_o      (id_pc_addr),
    .alu_op_o       (id_alu_op),
    .alu_a_sel_o    (id_alu_a_sel),
    .alu_b_sel_o    (id_alu_b_sel),
    .mem_write_o    (id_mem_write),
    .mem_cs_o       (id_mem_cs),
    .mem_be_o       (id_mem_be),
    .mem_signed_o   (id_mem_signed),
    .wb_sel_o       (id_wb_sel),
    .apsr_wr_en_o   (id_apsr_wr_en),
    .Z_o            (id_Z),
    .N_o            (id_N),
    .C_o            (id_C),
    .V_o            (id_V)
);


//====================== Execute Stage ===========================
ExecuteStage EXE (
    // ─── Inputs ───────────────────────────────────────────────
    .clk_i              (clk_i),
    .reset_i            (reset_i),
    .stall_i            (stall),
    .flush_i            (flush),
    .rf_rd_a_addr_i     (id_rf_rd_a_addr),
    .rf_rd_b_addr_i     (id_rf_rd_b_addr),
    .rf_rd_c_addr_i     (id_rf_rd_c_addr),
    .rf_wr_a_en_i       (id_rf_wr_a_en),
    .rf_wr_a_addr_i     (id_rf_wr_a_addr),
    .rf_wr_b_en_i       (id_rf_wr_b_en),
    .rf_wr_b_addr_i     (id_rf_wr_b_addr),
    .imm_i              (id_imm),
    .instr_i            (exe_instr),
    .reg_a_data_i       (rf_reg_a_data),
    .reg_b_data_i       (rf_reg_b_data),
    .reg_c_data_i       (rf_reg_c_data),
    .br_en_i            (id_br_en),
    .br_type_i          (id_br_type),
    .pc_addr_i          (if_pc_addr),
    .alu_op_i           (id_alu_op),
    .alu_a_sel_i        (id_alu_a_sel),
    .alu_b_sel_i        (id_alu_b_sel),
    .mem_write_i        (id_mem_write),
    .mem_cs_i           (id_mem_cs),
    .mem_be_i           (id_mem_be),
    .mem_signed_i       (id_mem_signed),
    .wb_sel_i           (id_wb_sel),
    .apsr_wr_en_i       (id_apsr_wr_en),
    .Z_i                (id_Z),
    .N_i                (id_N),
    .C_i                (id_C),
    .V_i                (id_V),
    .fwd_mux_a_i        (fwd_mux_a),
    .fwd_mux_b_i        (fwd_mux_b),
    .fwd_exe_mem_data_i (exe_alu_result),
    .fwd_mem_wb_data_i  (mem_alu_result),

    // ─── Outputs ──────────────────────────────────────────────
    .instr_o            (mem_instr),
    .rf_wr_a_en_o       (exe_rf_wr_a_en),
    .rf_wr_a_addr_o     (exe_rf_wr_a_addr),
    .rf_wr_b_en_o       (exe_rf_wr_b_en),
    .rf_wr_b_addr_o     (exe_rf_wr_b_addr),
    .alu_result_o       (exe_alu_result),
    .br_en_o            (exe_br_en),
    .br_type_o          (exe_br_type),
    .reg_c_data_o       (exe_reg_c_data),
    .wb_sel_o           (exe_wb_sel),
    .pc_addr_o          (exe_pc_addr),
    .mem_write_o        (exe_mem_write),
    .mem_cs_o           (exe_mem_cs),
    .mem_be_o           (exe_mem_be),
    .mem_signed_o       (exe_mem_signed),
    .apsr_wr_en_o       (exe_apsr_wr_en),
    .Z_o                (exe_Z),
    .N_o                (exe_N),
    .C_o                (exe_C),
    .V_o                (exe_V)
);


//====================== Memory Stage ============================
MemoryStage MEM (
    // ─── Inputs ───────────────────────────────────────────────
    .clk_i          (clk_i),
    .reset_i        (reset_i),
    .stall_i        (stall),
    .rf_wr_a_en_i   (exe_rf_wr_a_en),
    .rf_wr_a_addr_i (exe_rf_wr_a_addr),
    .rf_wr_b_en_i   (exe_rf_wr_b_en),
    .rf_wr_b_addr_i (exe_rf_wr_b_addr),
    .alu_result_i   (exe_alu_result),
    .br_en_i        (exe_br_en),
    .br_type_i      (exe_br_type),
    .reg_c_data_i   (exe_reg_c_data),
    .wb_sel_i       (exe_wb_sel),
    .instr_i        (mem_instr),
    .pc_addr_i      (exe_pc_addr),
    .mem_write_i    (exe_mem_write),
    .mem_cs_i       (exe_mem_cs),
    .mem_be_i       (exe_mem_be),
    .mem_signed_i   (exe_mem_signed),
    .mem_rdata_i    (DIN),
    .apsr_wr_en_i   (exe_apsr_wr_en),
    .Z_i            (exe_Z),
    .N_i            (exe_N),
    .C_i            (exe_C),
    .V_i            (exe_V),

    // ─── Outputs ──────────────────────────────────────────────
    .mem_write_o    (DRW),
    .mem_cs_o       (DREQ),
    .mem_be_o       (DSIZE),
    .mem_wdata_o    (DOUT),
    .mem_rdata_o    (mem_rdata),
    .mem_addr_o     (DADDR),
    .rf_wr_a_en_o   (mem_rf_wr_a_en),
    .rf_wr_a_addr_o (mem_rf_wr_a_addr),
    .rf_wr_b_en_o   (mem_rf_wr_b_en),
    .rf_wr_b_addr_o (mem_rf_wr_b_addr),
    .alu_result_o   (mem_alu_result),
    .pc_sel_o       (mem_pc_sel),
    .wb_sel_o       (mem_wb_sel),
    .pc_addr_o      (mem_pc_addr),
    .apsr_wr_en_o   (mem_apsr_wr_en),
    .Z_o            (mem_Z),
    .N_o            (mem_N),
    .C_o            (mem_C),
    .V_o            (mem_V)
);

//====================== Write-Back Stage ========================
WriteBackStage WB (
    // ─── Inputs ───────────────────────────────────────────────
    .clk_i          (clk_i),
    .reset_i        (reset_i),
    .rf_wr_a_en_i   (mem_rf_wr_a_en),
    .rf_wr_a_addr_i (mem_rf_wr_a_addr),
    .rf_wr_b_en_i   (mem_rf_wr_b_en),
    .rf_wr_b_addr_i (mem_rf_wr_b_addr),
    .mem_rdata_i    (mem_rdata),
    .alu_result_i   (mem_alu_result),
    .pc_addr_i      (mem_pc_addr),
    .wb_sel_i       (mem_wb_sel),
    .apsr_wr_en_i   (mem_apsr_wr_en),
    .Z_i            (mem_Z),
    .N_i            (mem_N),
    .C_i            (mem_C),
    .V_i            (mem_V),
    .instr_i        (mem_instr),

    // ─── Outputs ──────────────────────────────────────────────
    .wr_data_o      (wb_wr_data),
    .rf_wr_a_en_o   (wb_rf_wr_a_en),
    .rf_wr_a_addr_o (wb_rf_wr_a_addr),
    .rf_wr_b_en_o   (wb_rf_wr_b_en),
    .rf_wr_b_addr_o (wb_rf_wr_b_addr),
    .apsr_wr_en_o   (wb_apsr_wr_en),
    .pc_addr_o      (wb_pc_addr),
    .Z_o            (wb_Z),
    .N_o            (wb_N),
    .C_o            (wb_C),
    .V_o            (wb_V)
);

endmodule

// your code here (for other modules)


//Instruction fetch pipeline stage
module InstructionFetchStage(

  input             clk_i,
  input             reset_i,
  input             stall_i,
  input             flush_i,

  input [15:0]      instr_mem_i,
  input [31:0]      addr_branch_i,
  input [31:0]      addr_reg_i,
  input [1:0]       addr_sel_i,

  output reg [15:0] instr_reg_o,  //Latched instruction in pipeline register  
  output [15:0]     instr_noreg_o, //Non-latched instruction(straight from memory, next instr)
  output reg [31:0] pc_addr_o,
  output [31:0]     imem_addr_o,
  output            imem_req_o
);

reg  [31:0] pc_mux_addr;
wire [31:0] pc_current_addr;
wire [31:0] pc_branch_aux;


ProgramCounter PC(

  .clk_i(clk_i),
  .reset_i(reset_i),
  .stall_i(stall_i),
  .new_addr_i(pc_mux_addr),
  .current_addr_o(pc_current_addr)

);


assign instr_noreg_o = (~reset_i || flush_i) ? 16'b0 : instr_mem_i;
assign imem_req_o = (reset_i && ~flush_i);
assign imem_addr_o = pc_current_addr;
assign pc_branch_aux = addr_branch_i + 2;
//PC address select multiplexer
always @(*) begin
  case(addr_sel_i) 
    `PC_SEL_NEXT : pc_mux_addr = pc_current_addr + 2;
    `PC_SEL_BRANCH : pc_mux_addr = {pc_branch_aux[31:1], 1'b0} ;
    `PC_SEL_REG : pc_mux_addr = addr_reg_i;
    default : pc_mux_addr = pc_current_addr + 2;
  endcase
end

always @(posedge clk_i or negedge reset_i or posedge flush_i)
begin
    if(~reset_i || flush_i) begin
      instr_reg_o <= 16'b0;
      pc_addr_o <= 32'b0;
    end
    else if(~stall_i) begin
      instr_reg_o <= instr_mem_i;
      pc_addr_o <= pc_current_addr;
    end
end


endmodule

//Instruction decode pipeline stage
module InstructionDecodeStage(
  input             clk_i,
  input             reset_i,
  input             stall_i,
  input             flush_i,

  input  [15:0]     instr_i,      //Latched instruction in pipeline register 
  input  [15:0]     instr_noreg_i, //Non-latched instruction(straight from memory, next instr)
  input  [31:0]     pc_addr_i,
  input             apsr_wr_en_i,
  input             Z_new_i,
  input             N_new_i,
  input             C_new_i,
  input             V_new_i,
  input [31:0]      reg_a_data_i,
  input [31:0]      reg_b_data_i,
  input [31:0]      reg_c_data_i,

  output reg [15:0] instr_o,
  output reg [3:0]  rf_rd_a_addr_o,
  output reg [3:0]  rf_rd_b_addr_o,
  output reg [3:0]  rf_rd_c_addr_o,
  output reg        rf_wr_a_en_o,
  output reg [3:0]  rf_wr_a_addr_o,
  output reg        rf_wr_b_en_o,
  output reg [3:0]  rf_wr_b_addr_o,
  output reg [31:0] imm_o,
  output reg [31:0] reg_a_data_o,
  output reg [31:0] reg_b_data_o,
  output reg [31:0] reg_c_data_o,
  output reg [3:0]  br_type_o,
  output reg [1:0]  br_en_o,  
  output reg [31:0] pc_addr_o,
  output reg [4:0]  alu_op_o,
  output reg [1:0]  alu_a_sel_o,
  output reg [1:0]  alu_b_sel_o,
  output reg        mem_write_o,
  output reg        mem_cs_o,
  output reg [1:0]  mem_be_o,
  output reg        mem_signed_o,
  output reg        wb_sel_o,
  output reg        apsr_wr_en_o,
  output reg        Z_o,
  output reg        N_o,
  output reg        C_o,
  output reg        V_o

);

wire [3:0]  rf_rd_a_addr_w;
wire [3:0]  rf_rd_b_addr_w;
wire [3:0]  rf_rd_c_addr_w;
wire        rf_wr_a_en_w;
wire [3:0]  rf_wr_a_addr_w;
wire        rf_wr_b_en_w;
wire [3:0]  rf_wr_b_addr_w;
wire [31:0] imm_w;
wire [31:0] reg_a_data_w;
wire [31:0] reg_b_data_w;
wire [31:0] reg_c_data_w;
wire [3:0]  br_type_w;
wire [1:0]  br_en_w;
wire [4:0]  alu_op_w;
wire [1:0]  alu_a_sel_w;
wire [1:0]  alu_b_sel_w;
wire        mem_write_w;
wire        mem_cs_w;
wire [1:0]  mem_be_w;
wire        mem_signed_w;
wire        wb_sel_w;
wire        apsr_wr_en_w;
wire        Z_w;
wire        N_w;
wire        C_w;
wire        V_w;



StatusRegister APSR(

  .clk_i(clk_i),
  .reset_i(reset_i),
  .new_carry_i(C_new_i),
  .new_overflow_i(V_new_i),
  .new_negative_i(N_new_i),
  .new_zero_i(Z_new_i),
  .write_en_i(apsr_wr_en_i),
  .carry_o(C_w),
  .overflow_o(V_w),
  .negative_o(N_w),
  .zero_o(Z_w)
);

Decoder DEC(

  .clk_i(clk_i),
  .reset_i(reset_i),
  .instr_i(instr_i),
  .instr_noreg_i(instr_noreg_i),
  .immediate_o(imm_w),
  .rd_a_addr_o(rf_rd_a_addr_w),
  .rd_b_addr_o(rf_rd_b_addr_w),
  .rd_c_addr_o(rf_rd_c_addr_w),
  .wr_a_addr_o(rf_wr_a_addr_w),
  .wr_b_addr_o(rf_wr_b_addr_w),
  .wr_a_en_o(rf_wr_a_en_w),
  .wr_b_en_o(rf_wr_b_en_w),
  .wr_wb_sel_o(wb_sel_w),
  .a_sel_o(alu_a_sel_w), //Reg or PC
  .b_sel_o(alu_b_sel_w), //Reg or Imm or PC
  .alu_op_sel_o(alu_op_w),
  .sreg_we_o(apsr_wr_en_w),
  .br_en_o(br_en_w),
  .br_type_o(br_type_w),
  .dmem_csn_o(mem_cs_w),
  .dmem_wr_en_o(mem_write_w),
  .dmem_be_o(mem_be_w),
  .dmem_signed_o(mem_signed_w)
  
);


always @(posedge clk_i or negedge reset_i or posedge flush_i)
begin
  if(~reset_i || flush_i) begin
    instr_o <= 16'b0;
    rf_rd_a_addr_o <= 4'b0;
    rf_rd_b_addr_o <= 4'b0;
    rf_rd_c_addr_o <= 4'b0;
    rf_wr_a_en_o <= 1'b0;
    rf_wr_a_addr_o <= 4'b0;
    rf_wr_b_en_o <= 1'b0;
    rf_wr_b_addr_o <= 4'b0;
    imm_o <= 32'b0;
    reg_a_data_o <= 32'b0;
    reg_b_data_o <= 32'b0;
    reg_c_data_o <= 32'b0;
    br_en_o <= 2'b0;
    br_type_o <= 4'b0000;
    pc_addr_o <= 32'b0;
    alu_op_o <= 5'b0;
    alu_a_sel_o <= 2'b0;
    alu_b_sel_o <= 2'b0;
    mem_write_o <= 1'b0;
    mem_cs_o <= `MEM_CSN_ACTIVE;
    mem_be_o <= 2'b0;
    mem_signed_o <= 1'b0;
    wb_sel_o <= 1'b0;
    apsr_wr_en_o <= 1'b0;
    Z_o <= 1'b0;
    N_o <= 1'b0;
    C_o <= 1'b0;
    V_o <= 1'b0;

  end
  else if(~stall_i) begin

    instr_o <= instr_i;
    rf_rd_a_addr_o <= rf_rd_a_addr_w;
    rf_rd_b_addr_o <= rf_rd_b_addr_w;
    rf_rd_c_addr_o <= rf_rd_c_addr_w;
    rf_wr_a_en_o <= rf_wr_a_en_w;
    rf_wr_a_addr_o <= rf_wr_a_addr_w;
    rf_wr_b_en_o <= rf_wr_b_en_w;
    rf_wr_b_addr_o <= rf_wr_b_addr_w;
    imm_o <= imm_w;
    reg_a_data_o <= reg_a_data_i;
    reg_b_data_o <= reg_b_data_i;
    reg_c_data_o <= reg_c_data_i;
    br_en_o <= br_en_w;
    br_type_o <= br_type_w;
    pc_addr_o <= pc_addr_i;
    alu_op_o <= alu_op_w;
    alu_a_sel_o <= alu_a_sel_w;
    alu_b_sel_o <= alu_b_sel_w;
    mem_write_o <= mem_write_w;
    mem_cs_o <= mem_cs_w; 
    mem_be_o <= mem_be_w;
    mem_signed_o <= mem_signed_w;
    wb_sel_o <= wb_sel_w;
    apsr_wr_en_o <= apsr_wr_en_w;
    Z_o <= Z_w;
    N_o <= N_w;
    C_o <= C_w;
    V_o <= V_w;
  end
end

endmodule

//Execute pipeline stage
module ExecuteStage(
  
  input             clk_i,
  input             reset_i,
  input             stall_i,
  input             flush_i,

  input [15:0]      instr_i,
  input [3:0]       rf_rd_a_addr_i, //Not sure if needed here
  input [3:0]       rf_rd_b_addr_i, //Not sure if needed here
  input [3:0]       rf_rd_c_addr_i, //Not sure if needed here
  input             rf_wr_a_en_i,
  input [3:0]       rf_wr_a_addr_i,
  input             rf_wr_b_en_i,
  input [3:0]       rf_wr_b_addr_i,
  input [31:0]      imm_i,
  input [31:0]      reg_a_data_i,
  input [31:0]      reg_b_data_i,
  input [31:0]      reg_c_data_i,
  input [3:0]       br_type_i,
  input [1:0]       br_en_i,
  input [31:0]      pc_addr_i,
  input [4:0]       alu_op_i,
  input [1:0]       alu_a_sel_i,
  input [1:0]       alu_b_sel_i,
  input             mem_write_i,
  input             mem_cs_i,
  input [1:0]       mem_be_i,
  input             mem_signed_i,
  input             wb_sel_i,
  input             apsr_wr_en_i,
  input             Z_i,
  input             N_i,
  input             C_i,
  input             V_i,
  input [1:0]       fwd_mux_a_i,
  input [1:0]       fwd_mux_b_i,
  input [31:0]      fwd_exe_mem_data_i,
  input [31:0]      fwd_mem_wb_data_i,

  output reg [15:0] instr_o,
  output reg        rf_wr_a_en_o,
  output reg [3:0]  rf_wr_a_addr_o,
  output reg        rf_wr_b_en_o,
  output reg [3:0]  rf_wr_b_addr_o,
  output reg [31:0] alu_result_o,
  output reg [3:0]  br_type_o,
  output reg [1:0]  br_en_o,
  output reg [31:0] reg_c_data_o,
  output reg [31:0] pc_addr_o,
  output reg        wb_sel_o,
  output reg        mem_write_o,
  output reg        mem_cs_o,
  output reg [1:0]  mem_be_o,
  output reg        mem_signed_o,
  output reg        apsr_wr_en_o,
  output reg        Z_o,
  output reg        N_o,
  output reg        C_o,
  output reg        V_o

);

  wire        rf_wr_a_en_w;
  wire [3:0]  rf_wr_a_addr_w;
  wire        rf_wr_b_en_w;
  wire [3:0]  rf_wr_b_addr_w;
  wire [31:0] alu_result_w;
  wire [3:0]  br_type_w;
  wire [1:0]  br_en_w;
  wire [31:0] reg_c_data_w;
  wire        wb_sel_w;
  wire        mem_write_w;
  wire        mem_cs_w;
  wire [1:0]  mem_be_w;
  wire        apsr_wr_en_w;
  wire        Z_w;
  wire        N_w;
  wire        C_w;
  wire        V_w;

  reg [31:0] alu_a_w;
  reg [31:0] alu_b_w;

  reg [31:0] fwd_mux_a_data;
  reg [31:0] fwd_mux_b_data;

  assign rf_wr_a_en_w =  rf_wr_a_en_i;
  assign rf_wr_a_addr_w = rf_wr_a_addr_i;
  assign rf_wr_b_en_w = rf_wr_b_en_i;
  assign rf_wr_b_addr_w = rf_wr_b_addr_i;
  assign br_en_w = br_en_i;
  assign br_type_w = br_type_i;
  assign reg_c_data_w = reg_c_data_i;
  assign wb_sel_w = wb_sel_i;
  assign mem_write_w = mem_write_i; 
  assign mem_cs_w = mem_cs_i;
  assign mem_be_w = mem_be_i;
  assign apsr_wr_en_w = apsr_wr_en_i;




  //Forwarding multiplexers
  always @(*) begin

    case(fwd_mux_a_i)
      `FWD_MUX_REG : fwd_mux_a_data = reg_a_data_i;
      `FWD_MUX_EXE_MEM : fwd_mux_a_data = fwd_exe_mem_data_i;
      `FWD_MUX_MEM_WB : fwd_mux_a_data = fwd_mem_wb_data_i;
      default : fwd_mux_a_data = 32'b0;
    endcase

    case(fwd_mux_b_i) 
      `FWD_MUX_REG : fwd_mux_b_data = reg_b_data_i;
      `FWD_MUX_EXE_MEM : fwd_mux_b_data = fwd_exe_mem_data_i;
      `FWD_MUX_MEM_WB : fwd_mux_b_data = fwd_mem_wb_data_i;
      default : fwd_mux_b_data = 32'b0;
    endcase
  
  //Data multiplexers
    if(alu_a_sel_i == `ALU_A_SEL_REG) alu_a_w = fwd_mux_a_data;
    else alu_a_w = pc_addr_i;

    case(alu_b_sel_i)
      `ALU_B_SEL_REG : alu_b_w = fwd_mux_b_data;
      `ALU_B_SEL_IMM : alu_b_w = imm_i;
      `ALU_B_SEL_PC : alu_b_w = pc_addr_i;
      default : alu_b_w = 32'b0;
    endcase  
  end

  ArithmeticLogicUnit ALU(
  .alu_op_i(alu_op_i),
  .a_i(alu_a_w),
  .b_i(alu_b_w),
  .carry_i(C_i),
  .overflow_i(V_i),
  .negative_i(N_i),
  .zero_i(Z_i),
  .result_o(alu_result_w),
  .carry_o(C_w),
  .overflow_o(V_w),
  .negative_o(N_w),
  .zero_o(Z_w)
);

always @(posedge clk_i or negedge reset_i /*or posedge flush_i*/) begin
  if(~reset_i /*|| flush_i*/) begin
    instr_o <= 16'b0;
    rf_wr_a_en_o <= 1'b0;
    rf_wr_a_addr_o <= 4'b0;
    rf_wr_b_en_o <= 1'b0;
    rf_wr_b_addr_o <= 4'b0;
    alu_result_o <= 32'b0;
    br_type_o <= 4'b0;
    br_en_o <= 2'b0;
    reg_c_data_o <= 32'b0;
    wb_sel_o <= 1'b0;
    mem_write_o <= 1'b0;
    mem_cs_o <= `MEM_CSN_INACTIVE;
    mem_be_o <= 2'b0;
    mem_signed_o <= 1'b0;
    pc_addr_o <= 32'b0;
    apsr_wr_en_o <= 1'b0;
    Z_o <= 1'b0;
    N_o <= 1'b0;
    C_o <= 1'b0;
    V_o <= 1'b0;
  end
  else /*if(~stall_i)*/ begin
    instr_o <= instr_i;
    rf_wr_a_en_o <= rf_wr_a_en_w;
    rf_wr_a_addr_o <= rf_wr_a_addr_w;
    rf_wr_b_en_o <= rf_wr_b_en_w;
    rf_wr_b_addr_o <= rf_wr_b_addr_w;
    alu_result_o <= alu_result_w;
    br_type_o <= br_type_w;
    br_en_o <= br_en_w;
    reg_c_data_o <= reg_c_data_w;
    wb_sel_o <= wb_sel_w;
    mem_write_o <= mem_write_w;
    mem_cs_o <= mem_cs_w;
    mem_be_o <= mem_be_w;
    mem_signed_o <= mem_signed_i;
    pc_addr_o <= pc_addr_i;
    apsr_wr_en_o <= apsr_wr_en_w;
    Z_o <= Z_w;
    N_o <= N_w;
    C_o <= C_w;
    V_o <= V_w;
  end
end

endmodule

//Memory access pipeline stage
module MemoryStage(

  input             clk_i,
  input             reset_i,
  input             stall_i,
  //input           flush_i,

  input [15:0]      instr_i,
  input             rf_wr_a_en_i,
  input [3:0]       rf_wr_a_addr_i,
  input             rf_wr_b_en_i,
  input [3:0]       rf_wr_b_addr_i,
  input [31:0]      alu_result_i,
  input [3:0]       br_type_i,
  input [1:0]       br_en_i,
  input [31:0]      reg_c_data_i,
  input             wb_sel_i,
  input             mem_write_i,
  input             mem_cs_i,
  input [1:0]       mem_be_i,
  input             mem_signed_i,
  input [31:0]      mem_rdata_i,
  input [31:0]      pc_addr_i,
  input             apsr_wr_en_i,
  input             Z_i,
  input             N_i,
  input             C_i,
  input             V_i,

  output reg [15:0] instr_o,
  output            mem_write_o,
  output            mem_cs_o,
  output  [1:0]     mem_be_o,
  output  [31:0]    mem_wdata_o,
  output reg [31:0]    mem_rdata_o,
  output  [31:0]    mem_addr_o,
  output reg        rf_wr_a_en_o,
  output reg [3:0]  rf_wr_a_addr_o,
  output reg        rf_wr_b_en_o,
  output reg [3:0]  rf_wr_b_addr_o,
  output reg [31:0] alu_result_o,
  output reg [1:0]  pc_sel_o,
  output reg        wb_sel_o,
  output reg [31:0] pc_addr_o,
  output reg        apsr_wr_en_o,
  output reg        Z_o,
  output reg        N_o,
  output reg        C_o,
  output reg        V_o

);


  wire [31:0] mem_wdata_w;
  wire [31:0] mem_addr_w;
  wire        rf_wr_a_en_w;
  wire [3:0]  rf_wr_a_addr_w;
  wire        rf_wr_b_en_w;
  wire [3:0]  rf_wr_b_addr_w;
  wire [31:0] alu_result_w;
  wire        wb_sel_w;
  wire        apsr_wr_en_w;
  wire        Z_w;
  wire        N_w;
  wire        C_w;
  wire        V_w;

  assign mem_write_o = mem_write_i;
  assign mem_cs_o = mem_cs_i;
  assign mem_be_o = mem_be_i;
  assign mem_wdata_o = reg_c_data_i;
  assign mem_addr_o = alu_result_i;
  assign rf_wr_a_en_w = rf_wr_a_en_i;
  assign rf_wr_a_addr_w = rf_wr_a_addr_i;
  assign rf_wr_b_en_w = rf_wr_b_en_i;
  assign rf_wr_b_addr_w = rf_wr_b_addr_i;
  assign alu_result_w = alu_result_i;
  assign wb_sel_w = wb_sel_i;
  assign apsr_wr_en_w = apsr_wr_en_i;
  assign Z_w = Z_i;
  assign N_w = N_i;
  assign C_w = C_i;
  assign V_w = V_i;

//Sign extension load control
always @(*) begin
  if(mem_be_i == `MEM_BE_BYTE) begin
    mem_rdata_o = mem_signed_i ? {{24{mem_rdata_i[7]}}, mem_rdata_i[7:0]} : {{24{1'b0}}, mem_rdata_i[7:0]}; 
  end
  else if(mem_be_i == `MEM_BE_HALFWORD) begin
    mem_rdata_o = mem_signed_i ? {{16{mem_rdata_i[15]}}, mem_rdata_i[15:0]} : {{16{1'b0}}, mem_rdata_i[15:0]}; 
  end
  else begin
    mem_rdata_o = mem_rdata_i;
  end
end
//Branch decision logic
always @(*) begin
    if(br_en_i == `BR_JUMP_ALU) begin
      case(br_type_i)
        `B_EQ : pc_sel_o = (Z_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;            
        `B_NE : pc_sel_o = (~Z_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_CS : pc_sel_o = (C_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_CC : pc_sel_o = (~C_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_MI : pc_sel_o = (N_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_PL : pc_sel_o = (~N_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_VS : pc_sel_o = (V_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_VC : pc_sel_o = (~V_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_HI : pc_sel_o = (C_o && ~Z_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_LS : pc_sel_o = (~C_o || Z_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT; 
        `B_GE : pc_sel_o = (N_o == V_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_LT : pc_sel_o = (N_o != V_o) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_GT : pc_sel_o = (~Z_o && (N_o == V_o)) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_LE : pc_sel_o = (Z_o || (N_o != V_o)) ? `PC_SEL_BRANCH : `PC_SEL_NEXT;
        `B_AL : pc_sel_o = `PC_SEL_BRANCH; 
        default : pc_sel_o = `PC_SEL_NEXT;
      endcase
    end
    else if(br_en_i == `BR_JUMP_REG) pc_sel_o = `PC_SEL_REG;
    else pc_sel_o = `PC_SEL_NEXT;
end

always @(posedge clk_i or negedge reset_i) begin
  if(~reset_i) begin
    instr_o <= 16'b0;
    rf_wr_a_en_o <= 1'b0;
    rf_wr_a_addr_o <= 4'b0;
    rf_wr_b_en_o <= 1'b0;
    rf_wr_b_addr_o <= 4'b0;
    alu_result_o <= 32'b0;
    wb_sel_o <= 1'b0;
    pc_addr_o <= 32'b0;
    apsr_wr_en_o <= 1'b0;
    Z_o <= 1'b0;
    N_o <= 1'b0;
    C_o <= 1'b0;
    V_o <= 1'b0;
  end
  else if(~stall_i) begin
    instr_o <= instr_i;
    rf_wr_a_en_o <= rf_wr_a_en_w;
    rf_wr_a_addr_o <= rf_wr_a_addr_w;
    rf_wr_b_en_o <= rf_wr_b_en_w;
    rf_wr_b_addr_o <= rf_wr_b_addr_w;
    alu_result_o <= alu_result_w;
    wb_sel_o <= wb_sel_w;
    pc_addr_o <= pc_addr_i;
    apsr_wr_en_o <= apsr_wr_en_w;
    Z_o <= Z_w;
    N_o <= N_w;
    C_o <= C_w;
    V_o <= V_w;
  end
end

endmodule

//Writeback pipeline stage
module WriteBackStage(

  input         clk_i,
  input         reset_i,
  //input         stall_i,
  //input         flush_i,
  input         rf_wr_a_en_i,
  input [3:0]   rf_wr_a_addr_i,
  input         rf_wr_b_en_i,
  input [3:0]   rf_wr_b_addr_i,
  input [31:0]  mem_rdata_i,
  input [31:0]  alu_result_i,
  input         wb_sel_i,
  input         apsr_wr_en_i,
  input         Z_i,
  input         N_i,
  input         C_i,
  input         V_i,
  input [15:0]  instr_i,
  input [31:0]  pc_addr_i,

  output [31:0] wr_data_o,
  output [31:0] pc_addr_o,

  output        rf_wr_a_en_o,
  output [3:0]  rf_wr_a_addr_o,
  output        rf_wr_b_en_o,
  output [3:0]  rf_wr_b_addr_o,
  output        apsr_wr_en_o,
  output        Z_o,
  output        N_o,
  output        C_o,
  output        V_o
);
//Writeback multiplexer
  assign wr_data_o = (wb_sel_i == `WB_SEL_MEM) ? mem_rdata_i : alu_result_i;
  assign rf_wr_a_en_o = rf_wr_a_en_i;
  assign rf_wr_a_addr_o = rf_wr_a_addr_i;
  assign rf_wr_b_en_o = rf_wr_b_en_i;
  assign rf_wr_b_addr_o = rf_wr_b_addr_i;
  assign apsr_wr_en_o = apsr_wr_en_i;
  assign Z_o = Z_i;
  assign N_o = N_i;
  assign C_o = C_i; 
  assign V_o = V_i;
  assign pc_addr_o = pc_addr_i;

endmodule


module ForwardUnit(

input   [3:0]   exe_mem_rd_i,
input   [3:0]   mem_wb_rd_i,
input   [3:0]   id_exe_rs1_i,
input   [3:0]   id_exe_rs2_i,

input           exe_wb_a_en_i,
input           mem_wb_a_en_i,

output  reg[1:0]   fwd_mux_a_o,
output  reg [1:0]   fwd_mux_b_o

);


always @(*) begin
  if((exe_wb_a_en_i == `WB_WRITE) && (exe_mem_rd_i == id_exe_rs1_i)) fwd_mux_a_o = `FWD_MUX_EXE_MEM;
  else if((mem_wb_a_en_i == `WB_WRITE) && !(exe_wb_a_en_i && (exe_mem_rd_i == id_exe_rs1_i)) && (mem_wb_rd_i == id_exe_rs1_i)) fwd_mux_a_o = `FWD_MUX_MEM_WB;
  else fwd_mux_a_o = `FWD_MUX_REG;


  if((exe_wb_a_en_i == `WB_WRITE) && (exe_mem_rd_i == id_exe_rs2_i)) fwd_mux_b_o = `FWD_MUX_EXE_MEM;
  else if((mem_wb_a_en_i == `WB_WRITE) && !(exe_wb_a_en_i && (exe_mem_rd_i == id_exe_rs2_i)) && (mem_wb_rd_i == id_exe_rs2_i)) fwd_mux_b_o = `FWD_MUX_MEM_WB;
  else fwd_mux_b_o = `FWD_MUX_REG;
end

endmodule

//Program counter module
module ProgramCounter(

  //Common inputs
  input           clk_i,
  input           reset_i,
  input           stall_i,
  //Address ports
  input   [31:0]  new_addr_i,
  output [31:0]  current_addr_o

);

reg [31:0]  pc_addr;

always @(posedge clk_i or negedge reset_i) begin
  if(!reset_i) pc_addr <= `PC_RESET_VECTOR;
  else if(~stall_i) pc_addr <= new_addr_i;
end

assign current_addr_o = pc_addr;

endmodule

//Status register module(APSR)
module StatusRegister(

  //Common inputs
  input   clk_i,
  input   reset_i,

  //New flag values
  input   new_carry_i,
  input   new_overflow_i,
  input   new_negative_i,
  input   new_zero_i,

  input   write_en_i,

  //Flag register outputs
  output carry_o,
  output overflow_o,
  output negative_o,
  output zero_o
);

reg C;
reg	V; 
reg	N;
reg	Z;

always @(posedge clk_i or negedge reset_i) begin
  if(!reset_i)  begin
  	C <= 1'b0;
  	V <= 1'b0; 
    N <= 1'b0; 
    Z <= 1'b0;
  end
  else if(write_en_i) begin
    C <= new_carry_i;
    V <= new_overflow_i;
    N <= new_negative_i;
    Z <= new_zero_i;
  end
  
end

assign carry_o = C;
assign overflow_o = V;
assign negative_o = N;
assign zero_o = Z;

endmodule

//Decoder module
module Decoder(

  //Common inputs
  input           clk_i,
  input           reset_i,

  //Instruction  
  input   [15:0]  instr_i,
  input   [15:0]  instr_noreg_i,
  

  output reg [31:0]   immediate_o,

  //Register file control signals
  output reg [3:0]    rd_a_addr_o,
  output reg [3:0]    rd_b_addr_o,
  output reg [3:0]    rd_c_addr_o,
  output reg [3:0]    wr_a_addr_o,
  output reg [3:0]    wr_b_addr_o,
  output reg          wr_a_en_o,
  output reg          wr_b_en_o,
  output reg          wr_wb_sel_o,

  //ALU control signals
  output  reg [1:0]   a_sel_o, //Reg or PC
  output  reg [1:0]   b_sel_o, //Reg or Imm or PC 
  output  reg [4:0]   alu_op_sel_o,

  //Status register control signals
  output  reg         sreg_we_o,
  //PC control signals
  output  reg [3:0]   br_type_o,
  output  reg [1:0]   br_en_o,

  //Data memory control signals(SRAM PORT 1)
  output  reg         dmem_csn_o,
  output  reg         dmem_wr_en_o,
  output  reg [1:0]   dmem_be_o,
  output  reg         dmem_signed_o



);
  always @(*) begin
     casex(instr_i[15:10])

      `DEC_IMM : begin // 6'b00xxxx
        rd_c_addr_o = 4'b0;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE; // No writeback
        wr_wb_sel_o = `WB_SEL_ALU; // ALU result
        a_sel_o = `ALU_A_SEL_REG; // Register
        sreg_we_o = `SREG_WRITE; // Write to status register
        br_type_o = 4'b0;// No type of branch
        br_en_o = `BR_NO_BRANCH; // No branch
        dmem_csn_o = `MEM_CSN_INACTIVE; // No memory access
        dmem_wr_en_o = `MEM_WE_READ; // Read from memory 
        dmem_be_o = `MEM_BE_WORD; // Word access 
        dmem_signed_o = 1'b0;// No sign extension
        casex(instr_i[13:9])

          `DEC_LSL_IMM : begin //5'b000xx
				    if(instr_i[10:6] == 5'b00000) begin
				  //MOV REG T2
              immediate_o = 0; // immediate value
              rd_a_addr_o = instr_i[5:3]; // Read address Rm
              rd_b_addr_o = 4'b0; // no use  register
              wr_a_addr_o = instr_i[2:0]; // Write address Rd
              wr_a_en_o = `WB_WRITE; // Writeback
              b_sel_o = `ALU_B_SEL_IMM; // Immediate value for b source
              alu_op_sel_o = `ALU_MOVA;     // MOV instruction
				    end
				  else begin
				//LSL IMM
              immediate_o = instr_i[10:6]; // immediate value
              rd_a_addr_o = instr_i[5:3]; // Read address Rm
              rd_b_addr_o = 4'b0; // no use register
              wr_a_addr_o = instr_i[2:0]; // Write address Rd
              wr_a_en_o = `WB_WRITE; // Writeback
              b_sel_o = `ALU_B_SEL_IMM; // Immediate value for b source
              alu_op_sel_o = `ALU_LSL; // LSL instruction
				    end
          end

          `DEC_LSR_IMM : begin
            immediate_o = instr_i[10:6]; // immediate value
            rd_a_addr_o = instr_i[5:3]; // Read address Rm
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0]; // Write address Rd
            wr_a_en_o = `WB_WRITE; // Writeback
            b_sel_o = `ALU_B_SEL_IMM; // Immediate value for b source
            alu_op_sel_o = `ALU_LSR; // LSR instruction
          end
          `DEC_ASR_IMM : begin
            immediate_o = instr_i[10:6]; // immediate value
            rd_a_addr_o = instr_i[5:3]; // Read address Rm
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0]; // Write address Rd
            wr_a_en_o = `WB_WRITE; // Writeback
            b_sel_o = `ALU_B_SEL_IMM; // Immediate value for b source
            alu_op_sel_o = `ALU_ASR; // ASR instruction
          end
          `DEC_ADD_IREG : begin
            immediate_o = 11'b0; 
            rd_a_addr_o = instr_i[8:6]; // Read address Rm
            rd_b_addr_o = instr_i[5:3]; // Read address Rn
            wr_a_addr_o = instr_i[2:0]; // Write address Rd
            wr_a_en_o = `WB_WRITE; // Writeback
            b_sel_o = `ALU_B_SEL_REG; // Register value for b source
            alu_op_sel_o = `ALU_ADD; // ADD instruction
          end
          `DEC_SUB_IREG : begin
            immediate_o = 32'b0;
            rd_a_addr_o = instr_i[8:6];
            rd_b_addr_o = instr_i[5:3];
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            b_sel_o = `ALU_B_SEL_REG;
            alu_op_sel_o = `ALU_SUB;
          end
          `DEC_ADD_IMM3 : begin
            immediate_o = instr_i[8:6];
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_ADD;
          end
          `DEC_SUB_IMM3 : begin
            immediate_o = instr_i[8:6];
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_SUB;
          end
          `DEC_MOV_IMM : begin
            immediate_o = instr_i[7:0]; // immediate value
            rd_a_addr_o = 4'b0; // no use  register
            rd_b_addr_o = 4'b0; // no use  register
            wr_a_addr_o = instr_i[10:8]; // Write address Rd
            wr_a_en_o = `WB_WRITE; // Writeback
            b_sel_o = `ALU_B_SEL_IMM; // Immediate value for b source
            alu_op_sel_o = `ALU_MOVB;   // MOV instruction
          end
          `DEC_CMP_IMM : begin
            immediate_o = instr_i[7:0]; // immediate value
            rd_a_addr_o = instr_i[10:8]; // Read address Rm
            rd_b_addr_o = 4'b0; // no use  register
            wr_a_addr_o = 4'b0; // no use  register
            wr_a_en_o = `WB_NO_WRITE;   // No writeback
            b_sel_o = `ALU_B_SEL_IMM;   // Immediate value for b source
            alu_op_sel_o = `ALU_CMP;     // CMP instruction 
          end
          `DEC_ADD_IMM8 : begin // 5'b110xx
            immediate_o = instr_i[7:0]; // immediate value
            rd_a_addr_o = instr_i[10:8]; // Read address Rnn
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[10:8]; // Write address Rd
            wr_a_en_o = `WB_WRITE; // Writeback
            b_sel_o = `ALU_B_SEL_IMM; // Immediate value for b source
            alu_op_sel_o = `ALU_ADD;  
          end
          `DEC_SUB_IMM8 : begin
            immediate_o = instr_i[7:0];
            rd_a_addr_o = instr_i[10:8];
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = instr_i[10:8];
            wr_a_en_o = `WB_WRITE;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_SUB; 
          end
          default : begin
            immediate_o = 32'b0;
            rd_a_addr_o = 4'b0;
            rd_b_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = `WB_NO_WRITE;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_MOVB;            
          end 
        endcase
      end

      `DEC_DATA_PROC : begin

        immediate_o = 32'b0;
        rd_a_addr_o = {1'b0, instr_i[5:3]};
        rd_b_addr_o = {1'b0, instr_i[2:0]};
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = {1'b0, instr_i[2:0]};
        wr_a_en_o = `WB_WRITE;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE; 
        wr_wb_sel_o = `WB_SEL_ALU;
        a_sel_o = `ALU_A_SEL_REG;
        b_sel_o = `ALU_B_SEL_REG;
        alu_op_sel_o = {1'b0, instr_i[9:6]};  
        sreg_we_o = `SREG_WRITE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_INACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;

      end

      `DEC_DS_BEX : begin

        immediate_o = 32'b0;
        wr_wb_sel_o = `WB_SEL_ALU;
        sreg_we_o = `SREG_WRITE;
        dmem_csn_o = `MEM_CSN_INACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;
        casex(instr_i[9:6])

          `DEC_ADD_REG : begin
            rd_a_addr_o = {instr_i[7], instr_i[2:0]};
            rd_b_addr_o = instr_i[6:3];
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = {instr_i[7], instr_i[2:0]};
            wr_a_en_o = `WB_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = (rd_a_addr_o == 15) ? `ALU_A_SEL_PC : `ALU_A_SEL_REG;
            b_sel_o = (rd_b_addr_o == 15) ? `ALU_B_SEL_PC : `ALU_B_SEL_REG;        
            alu_op_sel_o = `ALU_ADD;
  
      
            br_en_o = (wr_a_addr_o == 15) ? `BR_JUMP_ALU : `BR_NO_BRANCH;
            br_type_o = (wr_a_addr_o == 15) ? `B_AL : 4'h0;
          end

          `DEC_CMP_1 : begin
            rd_a_addr_o = instr_i[5:3];
            rd_b_addr_o = instr_i[2:0];
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = `WB_NO_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_REG;        
            alu_op_sel_o = `ALU_CMP;
  
      
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;      
          end

          `DEC_CMP_2 : begin
            rd_a_addr_o = instr_i[6:3];
            rd_b_addr_o = {instr_i[7], instr_i[2:0]};
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            wr_a_en_o = `WB_NO_WRITE;
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_REG;        
            alu_op_sel_o = `ALU_CMP;
  
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;        
          end

          `DEC_MOV_REG : begin
            rd_a_addr_o = instr_i[6:3];
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = {instr_i[7], instr_i[2:0]};
            wr_a_en_o = `WB_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;        
            alu_op_sel_o = `ALU_MOVA;
  
      
            br_en_o = (wr_a_addr_o == 15) ? `BR_JUMP_ALU : `BR_NO_BRANCH;
            br_type_o = (wr_a_addr_o == 15) ? `B_AL : 4'h0;          
          end

          `DEC_BX : begin
            rd_a_addr_o = instr_i[6:3];
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = {instr_i[7], instr_i[2:0]};
            wr_a_en_o = `WB_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;        
            alu_op_sel_o = `ALU_ADD;
  
      
            br_type_o = `B_AL;
            br_en_o = `BR_JUMP_ALU;                  
          end

          `DEC_BLX : begin
            rd_a_addr_o = 4'b0;
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = instr_i[6:3];
            wr_a_addr_o = 4'b0; 
            wr_a_en_o = `WB_NO_WRITE;
            wr_b_addr_o = 4'b1110;//LR
            wr_b_en_o = `WB_WRITE;            
            a_sel_o = `ALU_A_SEL_PC;
            b_sel_o = `ALU_B_SEL_IMM;        
            alu_op_sel_o = `ALU_ADD;
  
      
            br_type_o = 4'b0;
            br_en_o = `BR_JUMP_REG;          
          end

          default : begin
            rd_a_addr_o = 0;
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 0;
            wr_a_en_o = `WB_NO_WRITE;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE;            
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;        
            alu_op_sel_o = `ALU_MOVA;
  
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH; 
          end

        endcase
        
      end

      `DEC_LDR_LIT : begin
        immediate_o = {instr_i[7:0], 2'b00};
        rd_a_addr_o = 4'b0;
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = {1'b0, instr_i[10:8]};
        wr_a_en_o = `WB_WRITE;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE;
        wr_wb_sel_o = `WB_SEL_MEM; //Write data from memory
        a_sel_o = `ALU_A_SEL_PC;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_ADD; 
        sreg_we_o = `SREG_WRITE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_ACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;      
      end

      `DEC_LS_1 :
		begin
		
		    rd_a_addr_o = instr_i[8:6];
        rd_b_addr_o = instr_i[5:3];
        rd_c_addr_o = instr_i[2:0];
        wr_a_addr_o = instr_i[2:0];
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE;
        wr_wb_sel_o = `WB_SEL_MEM;

        immediate_o = 32'b0;
        a_sel_o = `ALU_A_SEL_REG;
        b_sel_o = `ALU_B_SEL_REG;
        alu_op_sel_o = `ALU_ADD; 
        sreg_we_o = `SREG_WRITE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_ACTIVE;
        
        casex(instr_i[11:9])


          
          `DEC_STR_REG : begin
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_WRITE;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;           
          end
          `DEC_STRH_REG : begin
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_WRITE;
            dmem_be_o = `MEM_BE_HALFWORD; 
            dmem_signed_o = 1'b0;             
          end
         `DEC_STRB_REG : begin
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_WRITE;
            dmem_be_o = `MEM_BE_BYTE;
            dmem_signed_o = 1'b0;               
          end
          `DEC_LDRSB_REG : begin
            //Signed
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_BYTE;
            dmem_signed_o = 1'b1;             
          end
          `DEC_LDR_REG : begin
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;              
          end
          `DEC_LDRH_REG : begin
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_HALFWORD;
            dmem_signed_o = 1'b0;           
          end
          `DEC_LDRB_REG : begin
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_BYTE;
            dmem_signed_o = 1'b0;              
          end
          `DEC_LDRSH_REG : begin
            //Signed
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_HALFWORD;
            dmem_signed_o = 1'b1;  
          end

          default : begin
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;  
          end

        endcase
      end

      `DEC_LS_2 : begin
        rd_a_addr_o = instr_i[5:3];
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = instr_i[2:0];
        wr_b_addr_o = 4'b0;
        wr_wb_sel_o = `WB_SEL_MEM;
        wr_b_en_o = `WB_NO_WRITE; 

        
        a_sel_o = `ALU_A_SEL_REG;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_MOVA; 
        sreg_we_o = `SREG_WRITE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_ACTIVE;
        dmem_signed_o = 1'b0;

        casex(instr_i[12:9])
          `DEC_STR_IMM : begin
            immediate_o = {{25{1'b0}}, instr_i[10:6], 2'b00};
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_WRITE;
            dmem_be_o = `MEM_BE_WORD;            
          end
          `DEC_LDR_IMM : begin
            immediate_o = {{25{1'b0}}, instr_i[10:6], 2'b00};
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;            
          end
          `DEC_STRB_IMM : begin
            immediate_o = {{27{1'b0}}, instr_i[10:6]};
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_WRITE;
            dmem_be_o = `MEM_BE_BYTE;              
          end
          `DEC_LDRB_IMM : begin
            immediate_o = {{27{1'b0}}, instr_i[10:6]};
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_BYTE;              
          end
          default : begin
            immediate_o = 32'b0;
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;              
          end
        endcase
      end

      `DEC_LS_3 : begin
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = instr_i[2:0];
        wr_b_addr_o = 4'b0;
        wr_wb_sel_o = `WB_SEL_MEM;
        wr_b_en_o = `WB_NO_WRITE;
        
        a_sel_o = `ALU_A_SEL_REG;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_MOVA; 
        sreg_we_o = `SREG_WRITE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_ACTIVE;
        dmem_signed_o = 1'b0;
       
        casex(instr_i[12:9])
          `DEC_STRH_IMM : begin
            rd_a_addr_o = instr_i[5:3];
            immediate_o = {{26{1'b0}}, instr_i[10:6], 1'b0};
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_WRITE;
            dmem_be_o = `MEM_BE_HALFWORD;             
          end
          `DEC_LDRH_IMM : begin
            rd_a_addr_o = instr_i[5:3];
            immediate_o = {{26{1'b0}}, instr_i[10:6], 1'b0};
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_HALFWORD;              
          end
          `DEC_STR_SP : begin
            rd_a_addr_o = 4'b1101; //R13 - SP
            immediate_o = {{22{1'b0}}, instr_i[7:0], 2'b00};
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_WRITE;
            dmem_be_o = `MEM_BE_HALFWORD;               
          end
          `DEC_LDR_SP : begin
            rd_a_addr_o = 4'b1101; //R13 - SP
            immediate_o = {{22{1'b0}}, instr_i[7:0], 2'b00};
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;             
          end
          default : begin
            rd_a_addr_o = 4'b0;
            immediate_o = 32'b0;
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;                   
          end
        endcase
      end

      `DEC_PC_REL : begin
        immediate_o = instr_i[7:0];
        rd_a_addr_o = 4'b0;
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = {1'b0, instr_i[10:8]};
        wr_a_en_o = `WB_WRITE;
        wr_wb_sel_o = `WB_SEL_ALU;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE; 
        a_sel_o = `ALU_A_SEL_PC;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_ADD; 
        sreg_we_o = `SREG_NO_CHANGE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_INACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;         
      end

      `DEC_SP_REL : begin
        immediate_o = instr_i[7:0];
        rd_a_addr_o = 4'b1101; //R13 - SP 
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = {1'b0, instr_i[10:8]};
        wr_a_en_o = `WB_WRITE;
        wr_wb_sel_o = `WB_SEL_ALU;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE; 
        a_sel_o = `ALU_A_SEL_REG;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_ADD; 
        sreg_we_o = `SREG_NO_CHANGE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_INACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;         
      end

      `DEC_MISC_16 : begin
     
        casex(instr_i[11:5])
          `DEC_ADD_SPIMM : begin
            immediate_o = instr_i[6:0];
            rd_a_addr_o = 4'b1101; //R13 - SP 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b1101;
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_ADD; 

            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;  
          end
          `DEC_SUB_SPIMM : begin
            immediate_o = instr_i[6:0];
            rd_a_addr_o = 4'b1101; //R13 - SP 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b1101;
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_SUB; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
           
          end

          `DEC_SXTH : begin
            immediate_o = 0;
            rd_a_addr_o = instr_i[5:3]; 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_SXTH; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
          end

          `DEC_SXTB : begin
            immediate_o = 0;
            rd_a_addr_o = instr_i[5:3]; 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_SXTB; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
          end

          `DEC_UXTH : begin
            immediate_o = 0;
            rd_a_addr_o = instr_i[5:3]; 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_UXTH; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
          end

          `DEC_UXTB : begin
            immediate_o = 0;
            rd_a_addr_o = instr_i[5:3]; 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_UXTB; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
          end

          `DEC_PUSH : begin
            
            rd_a_addr_o = 4'b1101; //SP
            rd_b_addr_o = instr_i[2:0];
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE;
            wr_wb_sel_o = `WB_SEL_MEM;

            immediate_o = 32'b0;
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_REG;
            alu_op_sel_o = `ALU_MOVA; 
            sreg_we_o = `SREG_WRITE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_ACTIVE;
            wr_a_en_o = `WB_NO_WRITE;
            dmem_wr_en_o = `MEM_WE_WRITE;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;           
          end

          `DEC_REV : begin
            immediate_o = 0;
            rd_a_addr_o = instr_i[5:3]; 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_REV; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
          end

          `DEC_REV16 : begin
            immediate_o = 0;
            rd_a_addr_o = instr_i[5:3]; 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_REV16; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
          end

          `DEC_REVSH : begin
            immediate_o = 0;
            rd_a_addr_o = instr_i[5:3]; 
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_a_en_o = `WB_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_REVSH; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
          end

          `DEC_POP : begin
            
            rd_a_addr_o = 4'b1101; //SP
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = instr_i[2:0];
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE;
            wr_wb_sel_o = `WB_SEL_MEM;

            immediate_o = 32'b0;
            a_sel_o = `ALU_A_SEL_REG;
            b_sel_o = `ALU_B_SEL_REG;
            alu_op_sel_o = `ALU_ADD; 
            sreg_we_o = `SREG_WRITE;
            br_type_o = 4'b0;
            br_en_o = `BR_NO_BRANCH;
            dmem_csn_o = `MEM_CSN_ACTIVE;
            wr_a_en_o = `WB_WRITE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;     
          end

            default : begin
              immediate_o = 0;
              rd_a_addr_o = 4'b1101; //R13 - SP 
              rd_b_addr_o = 4'b0;
              rd_c_addr_o = 4'b0;
              wr_a_addr_o = 4'b1101;
              wr_a_en_o = `WB_NO_WRITE;
              wr_wb_sel_o = `WB_SEL_ALU;
              wr_b_addr_o = 4'b0;
              wr_b_en_o = `WB_NO_WRITE; 
              a_sel_o = `ALU_A_SEL_REG;
              b_sel_o = `ALU_B_SEL_IMM;
              alu_op_sel_o = 0; 
    
              sreg_we_o = `SREG_NO_CHANGE;
              br_type_o = 4'b0;
              br_en_o = `BR_NO_BRANCH;
              dmem_csn_o = `MEM_CSN_INACTIVE;
              dmem_wr_en_o = `MEM_WE_READ;
              dmem_be_o = `MEM_BE_WORD;
              dmem_signed_o = 1'b0;
            end
        endcase

      end

      `DEC_STM : begin
        /*
          JUST FOR TEST
        */

        immediate_o = instr_i[10:0];
        rd_a_addr_o = 4'b0;
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = 4'b0;
        wr_a_en_o = `WB_NO_WRITE;
        wr_wb_sel_o = `WB_SEL_ALU;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE; 
        a_sel_o = `ALU_A_SEL_PC;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_ADC; 
        sreg_we_o = `SREG_NO_CHANGE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_INACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;

      end

      `DEC_LDM : begin
        /*
          JUST FOR TEST
        */

        immediate_o = instr_i[10:0];
        rd_a_addr_o = 4'b0;
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = 4'b0;
        wr_a_en_o = `WB_NO_WRITE;
        wr_wb_sel_o = `WB_SEL_ALU;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE; 
        a_sel_o = `ALU_A_SEL_PC;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_ADC; 
        sreg_we_o = `SREG_NO_CHANGE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_INACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;      
      end

      `DEC_B_COND : begin
            immediate_o = {{23{instr_i[7]}} , instr_i[7:0], 1'b0};
            rd_a_addr_o = 4'b0;
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = `WB_NO_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_PC;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_ADD; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;
            br_type_o = instr_i[11:8];
            br_en_o = `BR_JUMP_ALU;
           
      end

      `DEC_B_UC : begin
            immediate_o = {{20{instr_i[10]}}, instr_i[10:0], 1'b0};
            rd_a_addr_o = 4'b0;
            rd_b_addr_o = 4'b0;
            rd_c_addr_o = 4'b0;
            wr_a_addr_o = 4'b0;
            wr_a_en_o = `WB_NO_WRITE;
            wr_wb_sel_o = `WB_SEL_ALU;
            wr_b_addr_o = 4'b0;
            wr_b_en_o = `WB_NO_WRITE; 
            a_sel_o = `ALU_A_SEL_PC;
            b_sel_o = `ALU_B_SEL_IMM;
            alu_op_sel_o = `ALU_ADD; 
  
            sreg_we_o = `SREG_NO_CHANGE;
            br_type_o = `B_AL;
            br_en_o = `BR_JUMP_ALU;
            dmem_csn_o = `MEM_CSN_INACTIVE;
            dmem_wr_en_o = `MEM_WE_READ;
            dmem_be_o = `MEM_BE_WORD;
            dmem_signed_o = 1'b0;      
      end

      `DEC_32B_2 : begin
         
        immediate_o = {{7{instr_i[10]}} , instr_i[10], ~(instr_noreg_i[13] ^ instr_i[10]), ~(instr_noreg_i[11] ^ instr_i[10]), instr_i[9:0], instr_noreg_i[10:0], 1'b0};
        rd_a_addr_o = 4'b0;
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = 4'b0;
        wr_a_en_o = `WB_NO_WRITE;
        wr_wb_sel_o = `WB_SEL_ALU;
        wr_b_addr_o = 4'b1110;//LR
        wr_b_en_o = `WB_WRITE; 
        a_sel_o = `ALU_A_SEL_PC;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_ADD; 
        sreg_we_o = `SREG_NO_CHANGE;
        br_type_o = `B_AL;
        br_en_o = `BR_JUMP_ALU;
        dmem_csn_o = `MEM_CSN_INACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;        
      end

      default : begin
        immediate_o = instr_i[10:0];
        rd_a_addr_o = 4'b0;
        rd_b_addr_o = 4'b0;
        rd_c_addr_o = 4'b0;
        wr_a_addr_o = 4'b0;
        wr_a_en_o = `WB_NO_WRITE;
        wr_wb_sel_o = `WB_SEL_ALU;
        wr_b_addr_o = 4'b0;
        wr_b_en_o = `WB_NO_WRITE; 
        a_sel_o = `ALU_A_SEL_PC;
        b_sel_o = `ALU_B_SEL_IMM;
        alu_op_sel_o = `ALU_MOVA; 
        sreg_we_o = `SREG_NO_CHANGE;
        br_type_o = 4'b0;
        br_en_o = `BR_NO_BRANCH;
        dmem_csn_o = `MEM_CSN_INACTIVE;
        dmem_wr_en_o = `MEM_WE_READ;
        dmem_be_o = `MEM_BE_WORD;
        dmem_signed_o = 1'b0;
                
      end
    endcase
  end




endmodule

//Prototype module for multiple access unit - not finished
module MultipleAccessUnit(
  input               clk_i,
  input               reset_i,
  input               enable_i,
  input      [13:0]   reglist_i,
  input      [31:0]   addr_i,
  input      [3:0]    reg_base_i,
  input               store_nload_i,
  input               normal_nstack_i,

  output reg [31:0]   addr_offset_o,
  output reg          imm_mux_o,
  output reg [15:0]   instr_inject_o,
  output reg          instr_mux_o,
  output reg          if_stall_o,
  output reg          ready_o
);
  reg [13:0] reglist_reg;
  reg        running;
  reg [3:0]  counter;
  

 
  

/*
  always @(*) begin
    //Check for stack too
    if(store_nload == 1) begin
      instr_inject_o = reglist_i[counter] ? ( normal_nstack_i ? {//store : push) : 0;
    end 
    else begin
      instr_inject_o = reglist_i[counter] ? ( normal_nstack_i ? //load : pop) : 0;
    end 

  end
*/
  always @(posedge clk_i or negedge reset_i) begin
    if(~reset_i) begin
      ready_o <= 1'b0;
      running <= 1'b0;
      reglist_reg <= 0;
      counter <= 14;
      if_stall_o <= 1'b0;
    end
    else if(enable_i) begin 
      running <= 1'b1;
      counter <= 14;
      if_stall_o <= 1'b1;
      reglist_reg <= reglist_i;
    end
    else if(running) begin
      counter <= counter - 1;
      if(counter == 0) begin 
        ready_o <= 1'b1;
        running <= 1'b0;
        if_stall_o <= 1'b0;
      end
    end
  end
endmodule

//ALU module 
module ArithmeticLogicUnit(
  
  input   [4:0]   alu_op_i,
  input   [31:0]  a_i,
  input   [31:0]  b_i,

  //Flags in
  input           carry_i,
  input           overflow_i,
  input           negative_i,
  input           zero_i,

  //Result and flags out
  output reg [31:0]  result_o,
  output reg         carry_o,
  output reg         overflow_o,
  output reg         negative_o,
  output reg         zero_o

);


`define   ALU_AND  5'b00000
`define   ALU_EOR  5'b00001
`define   ALU_LSL  5'b00010
`define   ALU_LSR  5'b00011
`define   ALU_ASR  5'b00100
`define   ALU_ADC  5'b00101
`define   ALU_SBC  5'b00110
`define   ALU_ROR  5'b00111
`define   ALU_TST  5'b01000
`define   ALU_RSB  5'b01001
`define   ALU_CMP  5'b01010
`define   ALU_CMN  5'b01011
`define   ALU_ORR  5'b01100
`define   ALU_MUL  5'b01101
`define   ALU_BIC  5'b01110
`define   ALU_MVN  5'b01111
`define   ALU_ADD  5'b10000
`define   ALU_MOVA 5'b10001
`define   ALU_MOVB 5'b10010
`define   ALU_SXTH 5'b10011
`define   ALU_SXTB 5'b10100 
`define   ALU_REV 5'b10101 
`define   ALU_REV16  5'b10110 
`define   ALU_REVSH 5'b10111 

always @(*) begin
    
    case(alu_op_i)

      `ALU_AND : begin
        result_o = a_i & b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;  
      end

      `ALU_EOR : begin
        result_o = a_i ^ b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;        
      end

      `ALU_LSL : begin
        {carry_o, result_o} = a_i << b_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;    
      end

      `ALU_LSR : begin
        result_o = a_i >> b_i;
        carry_o =  a_i[b_i-1];
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;         
      end

      `ALU_ASR : begin
        result_o = a_i >>> b_i;
        carry_o =  a_i[b_i-1]; 
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;          
      end

      `ALU_ADC : begin
        {carry_o, result_o} = a_i + b_i + carry_i;
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;  
      end

      `ALU_SBC : begin
        {carry_o, result_o} = a_i + ~b_i + carry_i; //carry 
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;          
      end

      `ALU_ROR : begin
        result_o = (a_i << b_i[4:0])| (a_i >> 32-(b_i[4:0]));
        carry_o =  result_o[31];
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;        
      end

      `ALU_TST : begin
        result_o = a_i & b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;  
      end
      
      `ALU_RSB : begin
        {carry_o, result_o} = b_i + ~a_i + 1; //carry 
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;             
      end

      `ALU_CMP : begin
        {carry_o, result_o} = a_i + ~b_i + 1; //carry 
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;            
      end

      `ALU_CMN : begin
        {carry_o, result_o} = a_i + b_i;
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;  
      end

      `ALU_ORR : begin
        result_o = a_i | b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;       
      end

      `ALU_MUL : begin
        result_o = a_i * b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;   
      end

      `ALU_BIC : begin
        result_o = a_i & ~b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;         
      end

      `ALU_MVN : begin
        result_o = ~b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;           
      end
      `ALU_ADD : begin
        {carry_o, result_o} = a_i + b_i;
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;    
      end

      `ALU_SUB : begin
        {carry_o, result_o} = a_i + ~b_i + 1; //carry 
        overflow_o = a_i[31] ^ b_i[31] ^ result_o[31] ^ carry_o;
        negative_o = result_o[31];
        zero_o = ~|result_o;            
      end

      `ALU_MOVA : begin
        result_o = a_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;      
      end

      `ALU_MOVB : begin
        result_o = b_i;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;      
      end

      `ALU_SXTH : begin
        result_o = {{16{a_i[15]}}, a_i[15:0]};
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;      
      end

      `ALU_SXTB : begin
        result_o = {{24{a_i[7]}}, a_i[7:0]};
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;      
      end

      `ALU_UXTH : begin
        result_o = {{16{1'b0}}, a_i[15:0]};
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;    
      end

      `ALU_UXTB : begin
        result_o = {{24{1'b0}}, a_i[7:0]};
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;      
      end

      `ALU_REV : begin
        result_o = {a_i[7:0], a_i[15:8], a_i[23:16], a_i[31:24]};
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;      
      end

      `ALU_REV16 : begin
        result_o = {a_i[23:16], a_i[31:24], a_i[7:0], a_i[15:8]};
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;      
      end

      `ALU_REVSH : begin
        result_o = {{16{a_i[7]}}, a_i[7:0], a_i[15:8]};
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = result_o[31];
        zero_o = ~|result_o;      
      end

      default : begin
        result_o = 32'h0;
        carry_o = carry_i;
        overflow_o = overflow_i;
        negative_o = negative_i;
        zero_o = zero_i;          
      end
    endcase

end

endmodule

