//`timescale 1ns/1ps

module TB ();

	parameter CLK_PER = 4;
	parameter NUM_CLK = 10000;


	// --------------------------------------------
	// Wires and Regs
	// --------------------------------------------
	reg				CLK;
	reg				RESET_N;

	wire			IREQ;
	wire	[31:0]	IADDR;
	wire	[31:0]	INSTR;

	wire			DREQ;
	wire	[31:0]	DADDR;
	wire			DWE;
	wire	[1:0]	DSIZE;
	wire	[31:0]	DIN;
	wire	[31:0]	DOUT;

	reg		[3:0]	DBE;

	// --------------------------------------------
	// Reg for ALU_TEST
	// --------------------------------------------
	reg		[15:0]	ALU_CLK;
	reg 	[31:0]	ALU_TEST_MEM [0:145];


	always #(CLK_PER/2) CLK = ~CLK;
	always #(CLK_PER) ALU_CLK = ALU_CLK+1;

	initial begin
		CLK = 1'b0;
		RESET_N = 1'b0;
		ALU_CLK = 1'b0;

		#(CLK_PER/4);
		
		#(CLK_PER*4);
			RESET_N = 1;
	end

	
	CortexM0 CortexM0 (
		.CLK(CLK),
		.RESET_N(RESET_N),
		
		// For instruction memory
		.IREQ(IREQ),
		.IADDR(IADDR),
		.INSTR(INSTR),

		// For data memory
		.DREQ(DREQ),
		.DADDR(DADDR),
		.DRW(DWE),		// read/write
		.DSIZE(DSIZE),	// Data memory access size 
		.DIN(DIN),
		.DOUT(DOUT)
	);

	SRAM MEM (
		.CLK (CLK),
		.CSN1 (1'b0),			// always chip select
		.ADDR1 (IADDR[13:2]),
		.WE1 (1'b0),			// only read operation
		.BE1 (4'b1111),			// word access
		.DI1 (),				// not used
		.DO1 (INSTR),			// read data

		.CSN2 (~DREQ),
		.ADDR2 (DADDR[13:2]),
		.WE2 (DWE),
		.BE2 (DBE),
		.DI2 (DOUT),
		.DO2 (DIN)
	);

	always @ (posedge CLK) begin
		if (DWE && DREQ==1'b1)
    		$display("%t  STORE  @%h  = %h", $time, DADDR, DOUT); 
	end


	always @* begin
		casex( {DSIZE, DADDR[1:0]} )
			{2'b00, 2'b00}	:	DBE = 4'b0001;
			{2'b00, 2'b01}	:	DBE = 4'b0010;
			{2'b00, 2'b10}	:	DBE = 4'b0100;
			{2'b00, 2'b11}	:	DBE = 4'b1000;
			{2'b01, 2'b00}	:	DBE = 4'b0011;
			{2'b01, 2'b10}	:	DBE = 4'b1100;
			{2'b10, 2'b00}	:	DBE = 4'b1111;
		endcase
	
	
	// --------------------------------------------
	// for ALU_TEST (can be ignored)
	// --------------------------------------------
	// Rping data when store

		if (ALU_CLK == 16'd9500) begin
			$readmemh("ALU_test_data.hex", ALU_TEST_MEM);
		end
		if (ALU_CLK == 16'd9550) begin
			if (ALU_CLK == 16'd9550) begin
				$display("\n--- ALU_TEST_MEM DUMP @ cycle %0d ---", ALU_CLK);
				for (integer i = 0; i <= 145; i = i + 1) begin
					$display("  [%0d] = %0d", i, ALU_TEST_MEM[i]);
				end
			end
			


			if (ALU_TEST_MEM[132] == 32'h0000_0001)
				$display("ADD_REG_test passed");
			else
				$display("ADD_REG_test failed");

			if (ALU_TEST_MEM[133] == 32'h0000_0001)
				$display("ADD_IMM_test passed");
			else
				$display("ADD_IMM_test failed");

			if (ALU_TEST_MEM[134] == 32'hfffffffd)
				$display("SUB_REG_test passed");
			else
				$display("SUB_REG_test failed");

			if (ALU_TEST_MEM[135] == 32'hfffffffd)
				$display("SUB_IMM_test passed");
			else
				$display("SUB_IMM_test failed");

			if (ALU_TEST_MEM[136] == 32'hfffffffe)
				$display("MUL_test passed");
			else
				$display("MUL_test failed");

			if (ALU_TEST_MEM[137] == 32'hffffffff)
				$display("ASR_IMM_test passed");
			else
				$display("ASR_IMM_test failed");

			if (ALU_TEST_MEM[138] == 32'hffffffff)
				$display("ASR_REG_test passed");
			else
				$display("ASR_REG_test failed");

			if (ALU_TEST_MEM[139] == 32'hfffffffc)
				$display("LSL_REG_test passed");
			else
				$display("LSL_REG_test failed");

			if (ALU_TEST_MEM[140] == 32'hfffffffc)
				$display("LSR_IMM_test passed");
			else
				$display("LSR_IMM_test failed");

			if (ALU_TEST_MEM[141] == 32'h00000002)
				$display("AND_test passed");
			else
				$display("AND_test failed");

			if (ALU_TEST_MEM[142] == 32'hfffffffd)
				$display("XOR_test passed");
			else
				$display("XOR_test failed");

			if (ALU_TEST_MEM[143] == 32'hffffffff)
				$display("OR_test passed");
			else
				$display("OR_test failed");

			if (ALU_TEST_MEM[144] == 32'hfffffffd)
				$display("NOT_test passed");
			else
				$display("NOT_test failed");

			if (ALU_TEST_MEM[145] == 32'h0000_0001)
				$display("RSB_test passed");
			else
				$display("RSB_test failed");
		end
	// --------------------------------------------
	end

	// --------------------------------------------
	// Load test vector to inst and data memory
	// --------------------------------------------
	// Caution : Assumption : input file has hex data like below. 
	//			 input file : M[0x03]M[0x02]M[0x01]M[0x00]
	//                        M[0x07]M[0x06]M[0x05]M[0x04]
	//									... 
	//           If the first 4 bytes in input file is 1234_5678
	//           then, the loaded value is mem[0x0000] = 0x1234_5678 (LSB)

	// // ROMDATA = initial memory data
	// defparam TB.MEM.ROMDATA = "ALU_TEST_inst.hex";
	// // MEMDATA = additional file for ALU_TEST (can be ignored)
  	// defparam TB.MEM.MEMDATA = "ALU_TEST_data.hex";


	// --------------------------------------------
	// For Dump variables
	// --------------------------------------------

	initial begin
		$dumpfile("myfile.dmp");
		$dumpvars;
	end

	initial begin
		#(CLK_PER * NUM_CLK); $finish;
	end


endmodule

