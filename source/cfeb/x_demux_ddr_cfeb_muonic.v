`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
// Virtex2: 1-to-2 De-multiplexer converts 80MHz data to 40MHz - CFEB Version requires IOB=true in ucf for rx ffs
//------------------------------------------------------------------------------------------------------------------
// 12/03/01 Initial
// 01/29/02 Added aclr input
// 03/03/02 Replaced library FFs with behavioral FFs
// 06/04/04 Add async set for MPC receiver
// 09/19/06 Mod for xst
// 10/06/06 Convert to DDR
// 10/09/06 Optimized for reduced latecy, pushes 1st cycle timing in the cable
// 05/27/09 Change to alct receive data muonic timing to float ALCT board in clock-space
// 05/28/09 Add timing constraints in hdl,interstage FFs, and sync FFs
// 05/29/09 Relax delay constraints to 2ns
// 06/03/09 Turn off constraints to see if goodspots improves
// 06/12/09 Change to lac commutator for interstage
// 06/15/09 Add 1st|2nd swap for digital phase shifter
// 06/16/09 Remove digital phase shifter
// 06/25/09 Clock phase mux can now span a full cycle, so dont need 1st|2nd swap
// 07/09/09 Mod for cfeb muonic, some ddrs move from IOB FFs to fabric due to 1-clock-per-iob-pair limit
// 07/10/09 Add programmable interstage delay to compensate for osu cable delay mistakes
// 07/22/09 Remove posneg
// 08/05/09 Remove iob async clear, add final stage sync clear, push srl delay through final sync stage
// 08/07/09 Push srl delay back to before final sync stage, was causing pattern finder fail timing
// 08/10/09 Add ff buffer for delay stage address
// 08/21/09 Add posneg
// 07/23/10 Add width display
//------------------------------------------------------------------------------------------------------------------
	module x_demux_ddr_cfeb_muonic
	(
	clock,
	clock_iob,
	posneg,
	delay_is,
	clr,
	din,
	dout1st,
	dout2nd
	);

// Generic
	parameter WIDTH = 16;
	initial	$display("x_demux_ddr_cfeb_muonic: WIDTH=%d",WIDTH);

// Ports
	input				clock;			// 40MHz TMB main clock
	input				clock_iob;		// 40MHZ iob ddr clock
	input				posneg;			// Select inter-stage clock 0 or 180 degrees
	input	[3:0]		delay_is;		// Interstage delay
	input				clr;			// Sync clear
	input	[WIDTH-1:0]	din;			// 80MHz ddr data
	output	[WIDTH-1:0]	dout1st;		// Data de-multiplexed 1st in time
	output	[WIDTH-1:0]	dout2nd;		// Data de-multiplexed 2nd in time

// Latch 80 MHz multiplexed inputs in DDR IOB FFs in the clock_iob time domain
	reg		[WIDTH-1:0]	din1st=0;		// Manually set iob=true in ucf
	reg		[WIDTH-1:0]	din2nd=0;		// Manually set iob=true in ucf

	always @(negedge clock_iob) begin	// Latch 1st-in-time on falling edge
	din1st <= din;
	end
	always @(posedge clock_iob) begin	// Latch 2nd-in-time on rising edge
	din2nd <= din;
	end

// Delay 1st-in-time by 1/2 cycle to align with 2nd-in-time, in the clock_iob time domain
	reg	 [WIDTH-1:0] din1st_ff=0;

	always @(posedge clock_iob) begin
	din1st_ff <= din1st;
	end

// Buffer local copy of posneg for fanout
	reg	posneg_ff=0;

	always @(posedge clock)begin
	posneg_ff <= posneg;
	end

// Latch demux data in inter-stage on falling edge of main clock
	reg [WIDTH-1:0]	din1st_is_neg=0;
	reg [WIDTH-1:0]	din2nd_is_neg=0;

	always @(negedge clock) begin
	din1st_is_neg <= din1st_ff;
	din2nd_is_neg <= din2nd;
	end

// Multiplex inter-stage data with direct IOB data, posneg=0 uses direct IOB
	wire [WIDTH-1:0] din1st_mux;
	wire [WIDTH-1:0] din2nd_mux;

	assign din1st_mux = (posneg_ff) ? din1st_is_neg : din1st_ff;
	assign din2nd_mux = (posneg_ff) ? din2nd_is_neg : din2nd;

// Delay data n-bx to compensate for osu cable error
	wire [WIDTH-1:0] din1st_srl, din1st_dly;
	wire [WIDTH-1:0] din2nd_srl, din2nd_dly;
	reg  [3:0]       dly=0;
	reg              dly_is_0=0;
	
	always @(posedge clock) begin
	dly      <=  delay_is-4'd1;		// Pointer to clct SRL data accounts for SLR 1bx latency
	dly_is_0 <= (delay_is == 0);	// Use direct input if delay is 0 beco 1st SRL output has 1bx overhead
	end

	srl16e_bbl #(WIDTH) ucfebdly1st (.clock(clock),.ce(1'b1),.adr(dly),.d(din1st_mux),.q(din1st_srl));
	srl16e_bbl #(WIDTH) ucfebdly2nd (.clock(clock),.ce(1'b1),.adr(dly),.d(din2nd_mux),.q(din2nd_srl));

	assign din1st_dly = (dly_is_0) ? din1st_mux : din1st_srl;
	assign din2nd_dly = (dly_is_0) ? din2nd_mux : din2nd_srl;

// Synchronize demux data in main clock time domain
	reg [WIDTH-1:0]	dout1st=0;
	reg [WIDTH-1:0]	dout2nd=0;

	always @(posedge clock) begin
	if (clr) begin
	dout1st <= 0;
	dout2nd <= 0;
	end
	else begin
	dout1st <= din1st_dly;
	dout2nd <= din2nd_dly;
	end
	end

//------------------------------------------------------------------------------------------------------------------
	endmodule
//------------------------------------------------------------------------------------------------------------------
