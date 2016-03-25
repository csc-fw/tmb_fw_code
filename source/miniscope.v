`timescale 1ns / 1ps
//`define DEBUG_MINISCOPE 1
//------------------------------------------------------------------------------------------------------------------
//	Mini-Scope
//		Records pre-trigger + alct*clct matching waveforms
//		Uses CFEB|RPC fifo addressing
//		Reads out into DMB data stream
//------------------------------------------------------------------------------------------------------------------
//	04/29/09 Initial
//	04/30/09 Mod port names to match caller
//	06/25/10 Add write address offset to look back from L1A time
//	08/16/10 Port to ise 12
//	10/06/10 Add virtex 6 ram option
//	10/07/10 Remove unused virtex 6 outputs
//------------------------------------------------------------------------------------------------------------------
	module miniscope
	(
// Clock
	clock,

// DMB Readout FIFO RAM Ports
	fifo_wen,
	fifo_wadr_mini,
	fifo_radr_mini,
	fifo_wdata_mini,
	fifo_rdata_mini,

// Status
	mini_tbins_test,
	parity_err_mini,

// Sump
	mini_sump
	);

//------------------------------------------------------------------------------------------------------------------
// Constants:
//------------------------------------------------------------------------------------------------------------------
// Raw hits RAM parameters
	`include "firmware_version.v"
	parameter RAM_DEPTH		= 2048;					// Storage bx depth
	parameter RAM_ADRB		= 11;					// Address width=log2(ram_depth)
	parameter RAM_WIDTH		= 8;					// Data width

//------------------------------------------------------------------------------------------------------------------
// IO:
//------------------------------------------------------------------------------------------------------------------
// Clock
	input						clock;				// TMB 40MHz main

// DMB Readout FIFO RAM Ports
	input						fifo_wen;			// 1=Write enable FIFO RAM
	input	[RAM_ADRB-1:0]		fifo_wadr_mini;		// FIFO RAM write address
	input	[RAM_ADRB-1:0]		fifo_radr_mini;		// FIFO RAM read address
	input	[RAM_WIDTH*2-1:0]	fifo_wdata_mini;	// FIFO RAM write data
	output	[RAM_WIDTH*2-1:0]	fifo_rdata_mini;	// FIFO RAM read  data

// Status Ports
	input						mini_tbins_test;	// Sets data=address for testing
	output	[1:0]				parity_err_mini;	// Miniscope RAM parity error detected

// Sump
	output						mini_sump;			// Unused signals

//------------------------------------------------------------------------------------------------------------------
//	Test pattern mux
//------------------------------------------------------------------------------------------------------------------
// Multiplex miniscope data with data=address text pattern
	wire [RAM_WIDTH-1:0] fifo_wdata [1:0];
	wire [15:0]	test_wdata;
	reg test_ff = 0;

	always @(posedge clock) begin
	test_ff <= mini_tbins_test;
	end

	assign test_wdata[15:0] = {5'h00,fifo_wadr_mini[10:0]};

	assign fifo_wdata[0] = (test_ff) ? test_wdata[7:0]  : fifo_wdata_mini[7:0];
	assign fifo_wdata[1] = (test_ff) ? test_wdata[15:8] : fifo_wdata_mini[15:8];

//------------------------------------------------------------------------------------------------------------------
// Data storage:
//------------------------------------------------------------------------------------------------------------------
// Calculate parity for  RAM write data
	wire [1:0] parity_wr;
	wire [1:0] parity_rd;

	assign parity_wr[0] = ~(^ fifo_wdata[0][RAM_WIDTH-1:0]);
	assign parity_wr[1] = ~(^ fifo_wdata[1][RAM_WIDTH-1:0]);

// Store mini scope dat in FIFO RAM, 8 bits wide + 1 parity x 2048 tbins deep, write port A, read port B
	wire [RAM_WIDTH-1:0] fifo_rdata [1:0];

`ifdef VIRTEX2
	initial $display("miniscope: generating Virtex2 RAMB16_S9_S9");
	wire [1:0] dopa;
	genvar i;
	generate
	for (i=0; i<=1; i=i+1) begin: mini_v2

	RAMB16_S9_S9 #(										// Virtex2
	.WRITE_MODE_A		("READ_FIRST"),					// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.WRITE_MODE_B		("READ_FIRST"),					// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.SIM_COLLISION_CHECK("ALL")							// "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
	) rawhits_ram (
	.WEA				(fifo_wen),						// Port A Write Enable Input
	.ENA				(1'b1),							// Port A RAM Enable Input
	.SSRA				(1'b0),							// Port A Synchronous Set/Reset Input
	.CLKA				(clock),						// Port A Clock
	.ADDRA				(fifo_wadr_mini[10:0]),			// Port A 11-bit Address Input
	.DIA				(fifo_wdata[i]),				// Port A 8-bit Data Input
	.DIPA				(parity_wr[i]),					// Port A 1-bit parity Input
	.DOA				(),								// Port A 8-bit Data Output
	.DOPA				(dopa[i]),						// Port A 1-bit Parity Output

	.WEB				(1'b0),							// Port B Write Enable Input
	.ENB				(1'b1),							// Port B RAM Enable Input
	.SSRB				(1'b0),							// Port B Synchronous Set/Reset Input
	.CLKB				(clock),						// Port B Clock
	.ADDRB				(fifo_radr_mini[10:0]),			// Port B 11-bit Address Input
	.DIB				(8'h00),						// Port B 8-bit Data Input
	.DIPB				(),								// Port-B 1-bit parity Input
	.DOB				(fifo_rdata[i]),				// Port B 8-bit Data Output
	.DOPB				(parity_rd[i])					// Port B 1-bit Parity Output
   );
  	end
	endgenerate

	assign mini_sump = (|dopa);							// Port A Virtex2 requires parity-out if parity-in is used

`elsif VIRTEX6
	initial $display("miniscope: generating Virtex6 RAMB18E1");
	wire [8:0] dum [1:0];
	assign mini_sump = 0;								// Virtex6 does not require parity-out if parity-in is used

	genvar i;
	generate
	for (i=0; i<=1; i=i+1) begin: mini_v6

	RAMB18E1 #(											// Virtex6
	.RAM_MODE			("TDP"),						// SDP or TDP
 	.READ_WIDTH_A		(0),							// 0,1,2,4,9,18,36 Read/write width per port
	.READ_WIDTH_B		(9),							// 0,1,2,4,9,18
	.WRITE_WIDTH_A		(9),							// 0,1,2,4,9,18
	.WRITE_WIDTH_B		(0),							// 0,1,2,4,9,18,36
	.WRITE_MODE_A		("READ_FIRST"),					// Must be same for both ports in SDP mode: WRITE_FIRST, READ_FIRST, or NO_CHANGE)
	.WRITE_MODE_B		("READ_FIRST"),
	.SIM_COLLISION_CHECK("ALL")							// ALL, WARNING_ONLY, GENERATE_X_ONLY or NONE)
	) uram (
	.WEA				({2{fifo_wen}}),				//  2-bit A port write enable input
	.ENARDEN			(1'b1),							//  1-bit A port enable/Read enable input
	.RSTRAMARSTRAM		(1'b0),							//  1-bit A port set/reset input
	.RSTREGARSTREG		(1'b0),							//  1-bit A port register set/reset input
	.REGCEAREGCE		(1'b0),							//  1-bit A port register enable/Register enable input
	.CLKARDCLK			(clock),						//  1-bit A port clock/Read clock input
	.ADDRARDADDR		({fifo_wadr_mini,3'h7}),		// 14-bit A port address/Read address input
	.DIADI				({8'h00,fifo_wdata[i]}),		// 16-bit A port data/LSB data input
	.DIPADIP			({1'b0,parity_wr[i]}),			//  2-bit A port parity/LSB parity input
	.DOADO				(),								// 16-bit A port data/LSB data output
	.DOPADOP			(),								//  2-bit A port parity/LSB parity output

	.WEBWE				(),								//  4-bit B port write enable/Write enable input
	.ENBWREN			(1'b1),							//  1-bit B port enable/Write enable input
	.REGCEB				(1'b0),							//  1-bit B port register enable input
	.RSTRAMB			(1'b0),							//  1-bit B port set/reset input
	.RSTREGB			(1'b0),							//  1-bit B port register set/reset input
	.CLKBWRCLK			(clock),						//  1-bit B port clock/Write clock input
	.ADDRBWRADDR		({fifo_radr_mini,3'h7}),		// 14-bit B port address/Write address input
	.DIBDI				(),								// 16-bit B port data/MSB data input
	.DIPBDIP			(),								//  2-bit B port parity/MSB parity input
	.DOBDO				({dum[i][7:0],fifo_rdata[i]}),	// 16-bit B port data/MSB data output
	.DOPBDOP			({dum[i][8],parity_rd[i]})		//  2-bit B port parity/MSB parity output
	);
	end
	endgenerate

`else
	initial begin
	$display ("miniscope: Virtex Undefined. Halting.");
	$finish
	end
`endif

// Map read data arrays
	assign fifo_rdata_mini[7:0]  = fifo_rdata[0];
	assign fifo_rdata_mini[15:8] = fifo_rdata[1];

// Compare read parity to write parity
	wire [1:0] parity_expect;

	assign parity_expect[0] = ~(^ fifo_rdata[0][RAM_WIDTH-1:0]);
	assign parity_expect[1] = ~(^ fifo_rdata[1][RAM_WIDTH-1:0]);

	assign parity_err_mini[1:0] = ~(parity_rd ~^ parity_expect);	// ~^ is bitwise equivalence operator

//------------------------------------------------------------------------------------------------------------------
	endmodule
//------------------------------------------------------------------------------------------------------------------
