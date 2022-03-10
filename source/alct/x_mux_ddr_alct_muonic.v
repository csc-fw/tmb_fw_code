`timescale 1ns / 1ps
//-------------------------------------------------------------------------------------------------------------------
// Virtex2: 2-to-1 Multiplexer converts 40MHz data to 80MHz
//-------------------------------------------------------------------------------------------------------------------
//	12/03/01 Initial
//	03/03/02 Replaced library FFs with behavioral FFs
//	01/29/03 Added async set to blank /mpc output at startup
//	09/19/06 Mod for xst
//	10/10/06 Convert to ddr for virtex2
//	01/22/08 NB all virtex2 ddr outputs power up as 0 during GSR, the init attribute can not be applied
//	03/20/09 ISE 10.1i ready
//	03/20/09 Add sync stage and clock for fpga fabric, separate iob clock
//	03/23/09 Reinstate din2nd holding stage
//	03/23/09 Move holding stage to iob clock domain
//	03/24/09 Add buffer ffs before iobs
//	03/26/09 Add then remove iob clock buffers between fabric FFs and IOB FFs, didn't improve window
//	03/30/09 Add inter-stage FFs with programmable clock phase
//	05/28/09 Add skew and delay constraints to FF paths
//	06/03/09 Turn off constraints to see if goodspots improves
//	06/12/09 Change to lac commutator for interstage
//	06/15/09 Add 1st|2nd swap for digital phase shifter
//	06/16/09 Remove digital phase shifter
//	08/05/09 Move timing constraints to ucf, remove async clear, add sync clear to IOB ffs
//	07/14/10 Mod default width for sim, add display
//-------------------------------------------------------------------------------------------------------------------
	module x_mux_ddr_alct_muonic
	(
	clock,
	clock_lac,
	clock_2x,
	clock_iob,
	clock_en,
	posneg,
	clr,
	din1st,
	din2nd,
	dout
	);

// Generic
	parameter WIDTH = 8;
	initial	$display("x_mux_ddr_alct_muonic: WIDTH=%d",WIDTH);

// Ports
	input				clock;				// 40MHz TMB main clock
	input				clock_lac;			// 40MHz logic accessible clock
	input				clock_2x;			// 80MHz commutator clock
	input				clock_iob;			// ALCT rx  40 MHz clock
	input				clock_en;			// Clock enable
	input				posneg;				// Select inter-stage clock 0 or 180 degrees
	input				clr;				// Sync clear
	input	[WIDTH-1:0]	din1st;				// Input data 1st-in-time
	input	[WIDTH-1:0]	din2nd;				// Input data 2nd-in-time
	output	[WIDTH-1:0]	dout;				// Output data multiplexed 2-to-1

// Latch fpga fabric inputs in main clock time domain
	reg  [2*WIDTH-1:0] din_ff = 0;
	wire [2*WIDTH-1:0] din;

	assign din = {din2nd,din1st};

	always @(posedge clock) begin
	din_ff <= din;
	end

// Interstage clock latches on rising or falling edge of main clock using clock_2x 
	reg	isen=0;

	always @(posedge clock_2x)begin
	isen <= clock_lac ^ posneg;
	end

// Latch fpga fabric inputs in an inter-stage time domain
	reg  [2*WIDTH-1:0] din_is_ff = 0;

	always @(posedge clock_2x) begin
	if (isen) din_is_ff <= din_ff;
	end

// Hold 2nd-in-time in IOB clock time domain, ucf LOCs these near DDR IOBs
	reg  [WIDTH-1:0] din2nd_iobff_hold = 0;
	wire [WIDTH-1:0] din1st_iobff;
	wire [WIDTH-1:0] din2nd_iobff;

	assign din1st_iobff = din_is_ff[WIDTH-1:0];
	assign din2nd_iobff = din_is_ff[2*WIDTH-1:WIDTH];

	always @(posedge clock_iob) begin
	din2nd_iobff_hold <= din2nd_iobff;
	end

// Generate array of output IOB DDRs, xst can not infer ddr outputs, alas
	genvar i;
	generate
	for (i=0; i<=WIDTH-1; i=i+1) begin: ddr_gen
	OFDDRRSE u0 (
	.C0		( clock_iob),			// In	0   degree clock
	.C1		(~clock_iob),			// In	180 degree clock
	.CE		(clock_en),				// In	Clock enable
	.R		(clr),					// In	Synchronous reset
	.S		(1'b0),					// In	Synchronous preset
	.D0		(din1st_iobff[i]),		// In	Posedge data
	.D1		(din2nd_iobff_hold[i]),	// In	Negedge data
	.Q		(dout[i]));				// Out	Data output (connect directly to top-level port)
	end
	endgenerate

//-------------------------------------------------------------------------------------------------------------------
	endmodule
//-------------------------------------------------------------------------------------------------------------------
