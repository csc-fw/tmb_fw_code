`timescale 1ns / 1ps
//`define DEBUG_CFEB 1
//-------------------------------------- bufferless raw hits version ------------------------------------------------
// Process 1 CFEB:
//		Input	8 DiStrips x 6 CSC Layers
//		Output	6x32=192 triad decoder pulses
//------------------------------------------------------------------------------------------------------------------
//	01/29/02 Initial
//	02/05/02 Changed ladj and radj to cfeb n+1 cfeb n-1 in Plane OR
//	02/06/02 Added inj_febsel and fifo_feb_sel for multiplexing with other CFEBs
//	02/26/02 Added FF to raw hits RAM outputs
//	02/27/02 Replaced library FFs and counters with behavioral code
//	02/28/02 Added FF to injector RAM outputs
//	02/28/02 Changed RAM INIT format to work with 2 lines
//	03/02/02 Changed tri-state RAM mux outputs to normal drivers, mux is now in clct_fifo
//	03/07/02 Changed x_demux reset to aclr, converted to behavioral version
//	03/15/02 Tri-stated inj_rdata
//	03/26/02 Set RAM pattern defaults to normal CLCT
//	04/02/02 Added 2nd demux test point
//	09/11/02 New triad decoder with distrip output and new CLCT patterns
//	09/20/02 Put back old triad decoder, crate DiStrips with ORs to correct for layer stagger
//	09/26/02 New algorithm finds pattern envelopes and priority enocodes only on number layers hit
//	09/30/02 Added cell retention to pattern finder module
//	10/28/02 XST mods
//	11/04/02 Change to triad_decode_v7, programmable persistence, sm reset, no parameter list
//	11/04/02 Triad decoder gets pgmlut to set all pattern lut inputs =1 for programming
//	11/05/02 State machine resets now async with auto power-up SRL16
//	11/06/02 Fnd4.2 mods
//	11/07/02 Fix distrip OR
//	11/12/02 2nd muon logic
//	11/13/02 Add () to count1s, speeds it up 10%
//	11/19/02 Convert pipeline delays from FF to SRL16
//	11/19/02 Revert to best 1 muon, 2 muon logic won't fit in XCV1000E, OR key hits for pre-trigger
//	11/21/02 Last stage selects between best distrip and 1/2-strip
//	11/22/02 Add 1/2-strip id bit to pattern number, add adjacent cfeb hit flags
//	12/20/02 Separated pattern width parameters for distrip bit
//	12/20/02 Remove last FF stage for speed
//	12/26/02 Change inj_rdata from tristate to mux
//	12/27/02 Change to triad_decode, allows 1-clock persistence
//	01/02/03 Remove pipe delay on pre-trigger outputs, speeds up pre-trig out by 1 clock
//	01/02/03 Add programmable strip mask for adject cfeb hits
//	05/05/03 Fix hs ds selection rule
//	05/06/03 Use hs_thresh_ff in hs ds selection instead of hs_thresh
//	05/06/03 FF final stage for speed, otherwise get only 38MHz
//	05/12/03 Add hsds tag to pretrigger output
//	03/15/04 Convert 80MHz inputs to ddr
//	04/20/04 Revert all DDR to 80MHz
//	06/07/04 Change to x_demux_v2 which has aset for mpc
//	05/19/06 Add cfeb_en to block triggers for disabled cfebs
//	05/24/06 Add ceb_en resets triad decoders to allow raw hits readout of disabled cfebs without triggering
//	08/02/06 Add layer trigger
//	09/11/06 Mods for xst compile
//	09/12/06 Optimize for xst
//	09/14/06 Local persist subtraction, convert to virtex2 rams
//	10/04/06 Mod triad decoder instantiation to use generate
//	10/04/06 Replace for-loops with while-loops to remove xst warnings re unused integers
//	10/10/06 Replace 80mhz demux with 40mhz ddr
//	10/16/06 Temporarily revert triad persistence to 6-1=5 for software compatibility
//	10/16/06 Unrevert, so persitence is now 6=6
//	11/29/06 Remove scope debug signals
//	04/27/07 Remove rx sync stage, shifts rx clock 12.5ns
//	07/02/07 Revert to key layer 2
//	07/30/07 Convert to bufferless raw hits ram, add debug state machine ascii display
//	08/02/07 Extend pattern injector to 1k tbins, add programmable firing length
//	02/01/08 Add parity to raw hits rams for seu detection
//	02/05/08 Replace inferred raw hits ram with instantiated ram, xst fails to build dual port parity block rams
//	04/22/08 Add triad test point at raw hits RAM input
//	11/17/08 Invert parity so all 0s data has parity=1
//	11/17/08 Change raw hits ram access to read-first so parity output is always valid
//	11/18/08 Add non-staggered injector pattern for ME1A/B
//	04/23/09 Mod for ise 10.1i
//	06/18/09 Add cfeb muonic timing
//	06/25/09 Muonic timing now spans a full clock cycle
//	06/29/09 Remove digital phase shifters for cfebs, certain cfeb IOBs can not have 2 clock domains
//	07/10/09 Return digital phase shifters for cfebs, mod ucf to move 5 IOB DDRs to fabric
//	07/22/09 Remove clock_vme global net to make room for cfeb digital phase shifter gbufs
//	08/05/09 Remove posneg, push cfeb_rx delay through final sync ff
//	08/07/09 Revert to 10mhz vme clock
//	08/11/09 Replace clock_vme with clock
//	08/21/09 Add posneg
//	09/03/09 Change cfeb_delay_is to cfeb_rxd_int_delay
//	12/11/09 Add bad cfeb bit checking
//	01/06/09 Restructure bad bit masks into 1d arrays
//	01/11/10 Move bad bits check downstream of pattern injector
//	01/13/10 Add single bx bad bit detection mode
//	01/14/10 Move bad bits check to triad_s1
//	03/05/10 Move hot channel + bad bits blocking ahead of raw hits ram, a big mistake, but poobah insists
//	03/07/10 Add masked cfebs to blocked list
//	06/30/10 Mod injector RAM for alct and l1a bits
//	07/07/10 Revert to discrete ren, wen
//	07/23/10 Replace DDR sub-module
//	08/06/10 Port to ise 12
//	08/09/10 Add init to pass_ff to power up in pass state
//	08/19/10 Replace * with &
//	08/25/10 Replace async resets with reg init
//	10/18/10 Add virtex 6 RAM option
//-------------------------------------------------------------------------------------------------------------------
	module cfeb
	(
// Clock
	clock,
	clock_cfeb_rxd,
	cfeb_rxd_posneg,
	cfeb_rxd_int_delay,

// Global Reset
	global_reset,
	ttc_resync,

// CFEBs
	cfeb_rx,
	mask_all,

// Injector
	inj_febsel,
	inject,
	inj_last_tbin,
	inj_wen,
	inj_rwadr,
	inj_wdata,
	inj_ren,
	inj_rdata,
	inj_ramout,
	inj_ramout_pulse,

// Raw Hits FIFO RAM
	fifo_wen,
	fifo_wadr,
	fifo_radr,
	fifo_sel,
	fifo_rdata,

// Hot Channel Mask
	ly0_hcm,
	ly1_hcm,
	ly2_hcm,
	ly3_hcm,
	ly4_hcm,
	ly5_hcm,

// Bad CFEB rx bit detection
	cfeb_badbits_reset,
	cfeb_badbits_block,
	cfeb_badbits_nbx,
	cfeb_badbits_found,
	cfeb_blockedbits,

	ly0_badbits,
	ly1_badbits,
	ly2_badbits,
	ly3_badbits,
	ly4_badbits,
	ly5_badbits,

// Triad Decoder
	triad_persist,
	triad_clr,
	triad_skip,
	ly0hs,
	ly1hs,
	ly2hs,
	ly3hs,
	ly4hs,
	ly5hs,
	nhits_per_cfeb,
    layers_withhits_per_cfeb,

// Status
	demux_tp_1st,
	demux_tp_2nd,
	triad_tp,
	parity_err_cfeb,
	cfeb_sump

// Debug
`ifdef DEBUG_CFEB
	,inj_sm_dsp
	,parity_wr
	,parity_rd
	,parity_expect
	,pass_ff

	,fifo_rdata_lyr0
	,fifo_rdata_lyr1
	,fifo_rdata_lyr2
	,fifo_rdata_lyr3
	,fifo_rdata_lyr4
	,fifo_rdata_lyr5
`endif
	);
//------------------------------------------------------------------------------------------------------------------
// Bus Widths
//------------------------------------------------------------------------------------------------------------------
	parameter MXLY			=	6;			// Number Layers in CSC
	parameter MXMUX			=	24;			// Number of multiplexed CFEB bits
	parameter MXTR			=	MXMUX*2;	// Number of Triad bits per CFEB
	parameter MXDS			=	8;			// Number of DiStrips per layer
	parameter MXHS			=	32;			// Number of 1/2-Strips per layer
	parameter MXKEY			=	MXHS;		// Number of Key 1/2-strips
	parameter MXKEYB		=	5;			// Number of key bits

// Raw hits RAM parameters
	parameter RAM_DEPTH		= 2048;			// Storage bx depth
	parameter RAM_ADRB		= 11;			// Address width=log2(ram_depth)
	parameter RAM_WIDTH		= 8;			// Data width
	
//------------------------------------------------------------------------------------------------------------------
// CFEB Ports
//------------------------------------------------------------------------------------------------------------------
// Clock
	input					clock;				// 40MHz TMB system clock
	input					clock_cfeb_rxd;		// 40MHz iob ddr clock
	input					cfeb_rxd_posneg;	// CFEB cfeb-to-tmb inter-stage clock select 0 or 180 degrees
	input	[3:0]			cfeb_rxd_int_delay;	// Interstage delay, integer bx

// Global reset
	input					global_reset;		// 1=Reset everything
	input					ttc_resync;			// 1=Reset everything

// CFEBs
	input	[MXMUX-1:0]		cfeb_rx;			// Multiplexed LVDS inputs from CFEB
	input					mask_all;			// 1=Enable, 0=Turn off all inputs

// Injector
	input					inj_febsel;			// 1=Enable RAM write
	input					inject;				// 1=Start pattern injector
	input	[11:0]			inj_last_tbin;		// Last tbin, may wrap past 1024 ram adr
	input	[2:0]			inj_wen;			// 1=Write enable injector RAM
	input	[9:0]			inj_rwadr;			// Injector RAM read/write address
	input	[17:0]			inj_wdata;			// Injector RAM write data
	input	[2:0]			inj_ren;			// Injector RAM select
	output	[17:0]			inj_rdata;			// Injector RAM read data
	output	[5:0]			inj_ramout;			// Injector RAM read data for ALCT and L1A
	output					inj_ramout_pulse;	// Injector RAM is injecting

// Raw Hits FIFO RAM
	input					fifo_wen;			// 1=Write enable FIFO RAM
	input	[RAM_ADRB-1:0]	fifo_wadr;			// FIFO RAM write address
	input	[RAM_ADRB-1:0]	fifo_radr;			// FIFO RAM read tbin address
	input	[2:0]			fifo_sel;			// FIFO RAM read layer address 0-5
	output	[RAM_WIDTH-1:0]	fifo_rdata;			// FIFO RAM read data

// Hot Channel Mask
	input	[MXDS-1:0]		ly0_hcm;			// 1=enable DiStrip
	input	[MXDS-1:0]		ly1_hcm;			// 1=enable DiStrip
	input	[MXDS-1:0]		ly2_hcm;			// 1=enable DiStrip
	input	[MXDS-1:0]		ly3_hcm;			// 1=enable DiStrip
	input	[MXDS-1:0]		ly4_hcm;			// 1=enable DiStrip
	input	[MXDS-1:0]		ly5_hcm;			// 1=enable DiStrip

// Bad CFEB rx bit detection
	input					cfeb_badbits_reset;	// Reset bad cfeb bits FFs
	input					cfeb_badbits_block;	// Allow bad bits to block triads
	input	[15:0]			cfeb_badbits_nbx;	// Cycles a bad bit must be continuously high
	output					cfeb_badbits_found;	// This CFEB has at least 1 bad bit
	output	[MXDS*MXLY-1:0]	cfeb_blockedbits;	// 1=CFEB rx bit blocked by hcm or went bad, packed

	output	[MXDS-1:0]		ly0_badbits;		// 1=CFEB rx bit went bad
	output	[MXDS-1:0]		ly1_badbits;		// 1=CFEB rx bit went bad
	output	[MXDS-1:0]		ly2_badbits;		// 1=CFEB rx bit went bad
	output	[MXDS-1:0]		ly3_badbits;		// 1=CFEB rx bit went bad
	output	[MXDS-1:0]		ly4_badbits;		// 1=CFEB rx bit went bad
	output	[MXDS-1:0]		ly5_badbits;		// 1=CFEB rx bit went bad

// Triad Decoder
	input	[3:0]			triad_persist;		// Triad 1/2-strip persistence
	input					triad_clr;			// Triad one-shot clear
	output					triad_skip;			// Triads skipped
	output	[MXHS-1:0]		ly0hs;
	output	[MXHS-1:0]		ly1hs;
	output	[MXHS-1:0]		ly2hs;
	output	[MXHS-1:0]		ly3hs;
	output	[MXHS-1:0]		ly4hs;
	output	[MXHS-1:0]		ly5hs;
	output [5:0] nhits_per_cfeb;
    output [MXLY-1:0] layers_withhits_per_cfeb;

// Status
	output					demux_tp_1st;		// Demultiplexer test point first-in-time
	output					demux_tp_2nd;		// Demultiplexer test point second-in-time
	output					triad_tp;			// Triad test point at raw hits RAM input
	output	[MXLY-1:0]		parity_err_cfeb;	// Raw hits RAM parity error detected
	output					cfeb_sump;			// Unused signals wot must be connected

// Debug
`ifdef DEBUG_CFEB
	output	[71:0]			inj_sm_dsp;			// Injector state machine ascii display
	output	[MXLY-1:0]		parity_wr;
	output	[MXLY-1:0]		parity_rd;
	output	[MXLY-1:0]		parity_expect;
	output					pass_ff;

	output	[MXDS-1+1:0]	fifo_rdata_lyr0;
	output	[MXDS-1+1:0]	fifo_rdata_lyr1;
	output	[MXDS-1+1:0]	fifo_rdata_lyr2;
	output	[MXDS-1+1:0]	fifo_rdata_lyr3;
	output	[MXDS-1+1:0]	fifo_rdata_lyr4;
	output	[MXDS-1+1:0]	fifo_rdata_lyr5;
`endif

//-------------------------------------------------------------------------------------------------------------------
// Load global definitions
//-------------------------------------------------------------------------------------------------------------------
	`include "source/tmb_virtex2_fw_version.v"
	`ifdef CSC_TYPE_A initial $display ("CSC_TYPE_A=%H",`CSC_TYPE_A); `endif	// Normal   CSC
	`ifdef CSC_TYPE_B initial $display ("CSC_TYPE_B=%H",`CSC_TYPE_B); `endif	// Reversed CSC
	`ifdef CSC_TYPE_C initial $display ("CSC_TYPE_C=%H",`CSC_TYPE_C); `endif	// Normal	ME1B reversed ME1A
	`ifdef CSC_TYPE_D initial $display ("CSC_TYPE_D=%H",`CSC_TYPE_D); `endif	// Reversed ME1B normal   ME1A

	`ifdef CSC_TYPE_A `define CFEB_INJECT_STAGGER 08'hAB
	`endif
	`ifdef CSC_TYPE_B `define CFEB_INJECT_STAGGER 08'hAB
	`endif

	`ifdef  CFEB_INJECT_STAGGER initial $display ("CFEB Pattern injector layer staggering is ON");  `endif
	`ifndef CFEB_INJECT_STAGGER initial $display ("CFEB Pattern injector layer staggering is OFF"); `endif

//-------------------------------------------------------------------------------------------------------------------
// State machine power-up reset + global reset
//-------------------------------------------------------------------------------------------------------------------
	wire [3:0]	pdly   = 1;		// Power-up reset delay
	reg			ready  = 0;
	reg			tready = 0;

	SRL16E upup (.CLK(clock),.CE(!power_up),.D(1'b1),.A0(pdly[0]),.A1(pdly[1]),.A2(pdly[2]),.A3(pdly[3]),.Q(power_up));

	always @(posedge clock) begin
	ready  <= power_up && !(global_reset || ttc_resync);
	tready <= power_up && !(global_reset || triad_clr  || ttc_resync);
	end

	wire reset  = !ready;	// injector state machine reset
	wire treset = !tready;	// triad decoder state machine reset

//-------------------------------------------------------------------------------------------------------------------
// Stage bx0: Demultiplex 80MHz CFEB data stream into comparator triads.
//			  Map triads by CSC layer and DiStrip.
//-------------------------------------------------------------------------------------------------------------------
// Latch 80 MHz multiplexed inputs in FDCE IOB FFs, 80MHz 1st in time is aligned with 40MHz falling edge
// triad_ff MSBs contain second in time, LSBs first in time
	wire	[MXTR-1:0]	triad_ff;

	x_demux_ddr_cfeb_muonic #(MXMUX) ux_demux_cfeb (
	.clock		(clock),						// In	40MHz TMB main clock
	.clock_iob	(clock_cfeb_rxd),				// 40MHz iob ddr clock
	.posneg		(cfeb_rxd_posneg),				// Select inter-stage clock 0 or 180 degrees
	.delay_is	(cfeb_rxd_int_delay[3:0]),		// Interstage delay
	.clr		(~mask_all),					// In	Sync clear
	.din		(cfeb_rx[MXMUX-1:0]),			// In	80MHz data from CFEB
	.dout1st	(triad_ff[(MXTR/2)-1:0]),		// Out	Data de-multiplexed 1st in time
	.dout2nd	(triad_ff[(MXTR-1):MXTR/2]));	// Out	Data de-multiplexed 2nd in time

// Map CFEB Signal names into Triad names, use BFA150/CED243 for simulator check
//	Signal	Pins	1st Cy 	2nd Cy	Triad_ff
//	rx0		1+	2-	Ly0Tr0	Ly3Tr0	0	24	
//	rx1		49+	50-	Ly0Tr1	Ly3Tr1	1	25
//	rx2		3+	4-	Ly0Tr2	Ly3Tr2	2	26
//	rx3		47+	48-	Ly0Tr3	Ly3Tr3	3	27
//	rx4		5+	6-	Ly5Tr0	Ly4Tr0	4	28
//	rx5		45+	46-	Ly5Tr1	Ly4Tr1	5	29
//	rx6		7+	8-	Ly5Tr2	Ly4Tr2	6	30
//	rx7		43+	44-	Ly5Tr3	Ly4Tr3	7	31
//	rx8		9+	10-	Ly1Tr0	Ly2Tr0	8	32
//	rx9		41+	42-	Ly1Tr1	Ly2Tr1	9	33
//	rx10	11+	12-	Ly1Tr2	Ly2Tr2	10	34
//	rx11	39+	40-	Ly1Tr3	Ly2Tr3	11	35
//	rx12	13+	14-	Ly0Tr4	Ly3Tr4	12	36
//	rx13	37+	38-	Ly0Tr5	Ly3Tr5	13	37
//	rx14	15+	16-	Ly0Tr6	Ly3Tr6	14	38
//	rx15	35+	36-	Ly0Tr7	Ly3Tr7	15	39
//	rx16	17+	18-	Ly5Tr4	Ly4Tr4	16	40
//	rx17	33+	34-	Ly5Tr5	Ly4Tr5	17	41
//	rx18	19+	20-	Ly5Tr6	Ly4Tr6	18	42
//	rx19	31+	32-	Ly5Tr7	Ly4Tr7	19	43
//	rx20	21+	22-	Ly1Tr4	Ly2Tr4	20	44
//	rx21	29+	30-	Ly1Tr5	Ly2Tr5	21	45
//	rx22	23+	24-	Ly1Tr6	Ly2Tr6	22	46
//	rx23	27+	28-	Ly1Tr7	Ly2Tr7	23	47

	wire [MXDS-1:0]	triad_s0 [MXLY-1:0];

	assign triad_s0[0][0]	=	triad_ff[0];	// Layer 0
	assign triad_s0[0][1]	=	triad_ff[1];	
	assign triad_s0[0][2]	=	triad_ff[2];	
	assign triad_s0[0][3]	=	triad_ff[3];
	assign triad_s0[0][4]	=	triad_ff[12];	
	assign triad_s0[0][5]	=	triad_ff[13];	
	assign triad_s0[0][6]	=	triad_ff[14];	
	assign triad_s0[0][7]	=	triad_ff[15];

	assign triad_s0[1][0]	=	triad_ff[8];	// Layer 1
	assign triad_s0[1][1]	=	triad_ff[9];	
	assign triad_s0[1][2]	=	triad_ff[10];	
	assign triad_s0[1][3]	=	triad_ff[11];
	assign triad_s0[1][4]	=	triad_ff[20];	
	assign triad_s0[1][5]	=	triad_ff[21];	
	assign triad_s0[1][6]	=	triad_ff[22];	
	assign triad_s0[1][7]	=	triad_ff[23];

	assign triad_s0[2][0]	=	triad_ff[32];	// Layer 2
	assign triad_s0[2][1]	=	triad_ff[33];	
	assign triad_s0[2][2]	=	triad_ff[34];	
	assign triad_s0[2][3]	=	triad_ff[35];
	assign triad_s0[2][4]	=	triad_ff[44];	
	assign triad_s0[2][5]	=	triad_ff[45];	
	assign triad_s0[2][6]	=	triad_ff[46];	
	assign triad_s0[2][7]	=	triad_ff[47];

	assign triad_s0[3][0]	=	triad_ff[24];	// Layer 3
	assign triad_s0[3][1]	=	triad_ff[25];	
	assign triad_s0[3][2]	=	triad_ff[26];	
	assign triad_s0[3][3]	=	triad_ff[27];
	assign triad_s0[3][4]	=	triad_ff[36];	
	assign triad_s0[3][5]	=	triad_ff[37];	
	assign triad_s0[3][6]	=	triad_ff[38];	
	assign triad_s0[3][7]	=	triad_ff[39];

	assign triad_s0[4][0]	=	triad_ff[28];	// Layer 4
	assign triad_s0[4][1]	=	triad_ff[29];	
	assign triad_s0[4][2]	=	triad_ff[30];	
	assign triad_s0[4][3]	=	triad_ff[31];
	assign triad_s0[4][4]	=	triad_ff[40];	
	assign triad_s0[4][5]	=	triad_ff[41];	
	assign triad_s0[4][6]	=	triad_ff[42];	
	assign triad_s0[4][7]	=	triad_ff[43];

	assign triad_s0[5][0]	=	triad_ff[4];	// Layer 5
	assign triad_s0[5][1]	=	triad_ff[5];	
	assign triad_s0[5][2]	=	triad_ff[6];	
	assign triad_s0[5][3]	=	triad_ff[7];
	assign triad_s0[5][4]	=	triad_ff[16];	
	assign triad_s0[5][5]	=	triad_ff[17];	
	assign triad_s0[5][6]	=	triad_ff[18];	
	assign triad_s0[5][7]	=	triad_ff[19];

// De-multiplexer test points: rx0 pins 1+2- Ly0Tr0	Ly3Tr0
	reg demux_tp_1st = 0;
	reg demux_tp_2nd = 0;

	always @(posedge clock) begin
	demux_tp_1st <= triad_s0[0][0];	// Layer 0 ds 0	1st in time
	demux_tp_2nd <= triad_s0[3][0];	// Layer 3 ds 0 2nd in time
	end

//-------------------------------------------------------------------------------------------------------------------
// Stage 1:	Pattern Injector
//			Injects an arbitrary test pattern into the Triad data stream.
//			Mask_all in the previous stage turns off CFEB inputs.
//			Stores raw hits in RAM
//
// Injector powers up with a preset pattern
//	Key on Ly2:	a05b06c05d06e05f06:  a straight 6-hit pattern on key 1/2-strip 05 starting in time bin 0
//				axxb27c26d27e26f27:  a straight 5-hit pattern on key 1/2-strip 26 starting in time bin 0
//
//	Key on Ly3:	a04b05c04d05e04f05:  a straight 6-hit pattern on key 1/2-strip 05 starting in time bin 0
//				axxb26c25d26e25f26:  a straight 5-hit pattern on key 1/2-strip 26 starting in time bin 0
//
//
//	DiStrip			0           1           2           3           4           5           6           7
//	Strip			0     1     0     1     0     1     0     1     0     1     0     1     0     1     0     1  
//	HStrip		 	0  1  2  3  0  1  2  3  0  1  2  3  0  1  2  3  0  1  2  3  0  1  2  3  0  1  2  3  0  1  2  3
//	1/2 Strip		00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
//
//            	  a5 b6 c5 d6 e5 f6
//	DiStrip  1:	  1  1  1  1  1  1
//	Strip  0,1:	  0  1  0  1  0  1
//	HStrip 6,5:   1  0  1  0  1  0
//
// 10/08/01 Initial
// 10/11/01 Converted from CLB to RAM. CLB version used 198 Slices, 195 FFs, RAM version uses xx FFs and 3 BlockRAMs
// 01/29/02 Ported to CFEB
// 11/07/02 Converted to key layer 3, because I was forced to =:-(
// 07/02/07 Reverted  to key layer 2, because I was forced to =:-)
//-------------------------------------------------------------------------------------------------------------------
// Injector State Machine Declarations
	reg [1:0] inj_sm;		// synthesis attribute safe_implementation of inj_sm is yes;
	parameter pass		= 0;
	parameter injecting	= 1;

// Injector State Machine
	wire inj_tbin_cnt_done;

	initial inj_sm = pass;

	always @(posedge clock) begin
	if   (reset)	                   inj_sm <= pass;
	else begin
	case (inj_sm)
	pass:		if (inject           ) inj_sm <= injecting;
	injecting:	if (inj_tbin_cnt_done) inj_sm <= pass;
	default                            inj_sm <= pass;
	endcase
	end
	end

// Injector Time Bin Counter
	reg  [11:0] inj_tbin_cnt=0;	// Counter runs 0-4095
	wire [9:0]  inj_tbin_adr;	// Injector adr runs 0-1023

	always @(posedge clock) begin
	if		(inj_sm==pass     ) inj_tbin_cnt <= 0;					// Sync  load
	else if	(inj_sm==injecting) inj_tbin_cnt <= inj_tbin_cnt+1'b1;	// Sync  count
	end

	assign inj_tbin_cnt_done = (inj_tbin_cnt==inj_last_tbin);		// Counter may wrap past 1024 ram adr limit
	assign inj_tbin_adr[9:0] = inj_tbin_cnt[9:0];					// injector ram address confined to 0-1023

// Pass state FF delays output mux 1 cycle
	reg pass_ff=1;

	always @(posedge clock) begin
	if (reset) pass_ff <= 1'b1;
	else       pass_ff <= (inj_sm == pass);
	end

// Injector RAM: 3 RAMs each 2 layers x 8 triads wide x 1024 tbins deep
// Port A: rw 18-bits via VME
// Port B: r  18-bits via injector SM
	wire [17:0]		inj_rdataa  [2:0];
	wire [1:0]		inj_ramoutb [2:0];
	wire [MXDS-1:0] triad_inj   [MXLY-1:0];

	initial $display("cfeb: generating Virtex2 RAMB16_S18_S18 ram.inj");

	genvar i;
	generate
	for (i=0; i<=2; i=i+1) begin: ram
	RAMB16_S18_S18 #(
	.WRITE_MODE_A		 ("READ_FIRST"),					// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.WRITE_MODE_B		 ("READ_FIRST"),					// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.SIM_COLLISION_CHECK ("ALL")							// "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
	) inj (
	.WEA				(inj_wen[i] & inj_febsel),			// Port A Write Enable Input
	.ENA				(1'b1),								// Port A RAM Enable Input
	.SSRA				(1'b0),								// Port A Synchronous Set/Reset Input
	.CLKA				(clock),							// Port A Clock
	.ADDRA				(inj_rwadr[9:0]),					// Port A 10-bit Address Input
	.DIA				(inj_wdata[15:0]),					// Port A 16-bit Data Input
	.DIPA				(inj_wdata[17:16]),					// Port A 2-bit parity Input
	.DOA				(inj_rdataa[i][15:0]),				// Port A 16-bit Data Output
	.DOPA				(inj_rdataa[i][17:16]),				// Port A 2-bit Parity Output

	.WEB				(1'b0),								// Port B Write Enable Input
	.ENB				(1'b1),								// Port B RAM Enable Input
	.SSRB				(1'b0),								// Port B Synchronous Set/Reset Input
	.CLKB				(clock),							// Port B Clock
	.ADDRB				(inj_tbin_adr[9:0]),				// Port B 10-bit Address Input
	.DIB				({16{1'b0}}),						// Port B 16-bit Data Input
	.DIPB				(2'b00),							// Port B 2-bit parity Input
	.DOB				({triad_inj[2*i+1],triad_inj[2*i]}),// Port B 16-bit Data Output
	.DOPB				(inj_ramoutb[i][1:0]));				// Port B 2-bit Parity Output
	end
	endgenerate

`ifdef CFEB_INJECT_STAGGER	// Normal Staggered CSC
// Initialize Injector RAMs, INIT values contain preset test pattern, 2 layers x 16 tbins per line
// Key layer 2: 6 hits on key 5 + 5 hits on key 26, 565656 staggered CSC
// Tbin                               FFFFEEEEDDDDCCCCBBBBAAAA9999888877776666555544443333222211110000;
	defparam ram[0].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000400242004202;
	defparam ram[0].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;
	defparam ram[1].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000400242404242;
	defparam ram[1].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;
	defparam ram[2].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000400242404242;
	defparam ram[2].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;

`else						// ME1A/B Non-Staggered CSC
// Initialize Injector RAMs, INIT values contain preset test pattern, 2 layers x 16 tbins per line
// Key layer 2: 6 hits on key 5 + 5 hits on key 26, 55555 non-staggered CSC
// Tbin                               FFFFEEEEDDDDCCCCBBBBAAAA9999888877776666555544443333222211110000;
	defparam ram[0].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000400242000202;
	defparam ram[0].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;
	defparam ram[1].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000400242400202;
	defparam ram[1].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;
	defparam ram[2].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000400242400202;
	defparam ram[2].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;
`endif

// Injector test pattern
// Tbin                               FFFFEEEEDDDDCCCCBBBBAAAA9999888877776666555544443333222211110000;
//	defparam ram[0].inj.INIT_00 =256'hA515A414A313A212A111A010A909A808A707A606A505A404A303A202A101ABCD;
//	defparam ram[0].inj.INIT_01 =256'hB131B030B929B828B727B626B525B424B323B222B121B020B919B818B717B616;
//	defparam ram[1].inj.INIT_00 =256'hC515C414C313C212C111C010C909C808C707C606C505C404C303C202C101DEF0;
//	defparam ram[1].inj.INIT_01 =256'hD131D030D929D828D727D626D525D424D323D222D121D020D919D818D717D616;
//	defparam ram[2].inj.INIT_00 =256'hE515E414E313E212E111E010E909E808E707E606E505E404E303E202E101EBBA;
//	defparam ram[2].inj.INIT_01 =256'hF131F030F929F828F727F626F525F424F323F222F121F020F919F818F717F616;

// Key layer 3: 6 hits on key 5 + 5 hits on key 26
// Tbin                               FFFFEEEEDDDDCCCCBBBBAAAA9999888877776666555544443333222211110000;
//	defparam ram[0].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000024040004242;
//	defparam ram[0].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;
//	defparam ram[1].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000024040004242;
//	defparam ram[1].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;
//	defparam ram[2].inj.INIT_00 =256'h0000000000000000000000000000000000000000000000000000024040004242;
//	defparam ram[2].inj.INIT_01 =256'h0000000000000000000000000000000000000000000000000000000000000000;

// Multiplex Injector RAM output data, tri-state output if CFEB is not selected
	reg [17:0] inj_rdata;

	always @(inj_rdataa[0] or inj_ren) begin
	case (inj_ren[2:0])
	3'b001:	inj_rdata <= inj_rdataa[0];
	3'b010:	inj_rdata <= inj_rdataa[1];
	3'b100:	inj_rdata <= inj_rdataa[2];
	default	inj_rdata <= inj_rdataa[0];
	endcase
	end

	assign inj_ramout[1:0] = inj_ramoutb[0][1:0];
	assign inj_ramout[3:2] = inj_ramoutb[1][1:0];
	assign inj_ramout[5:4] = inj_ramoutb[2][1:0];

	assign inj_ramout_pulse	= !pass_ff;

// Multiplex Triads from previous stage with Injector RAM data, output to next stage
	wire [MXDS-1:0] triad_s1 [MXLY-1:0];

	assign triad_s1[0] = (pass_ff) ? triad_s0[0] : triad_inj[0];
	assign triad_s1[1] = (pass_ff) ? triad_s0[1] : triad_inj[1];
	assign triad_s1[2] = (pass_ff) ? triad_s0[2] : triad_inj[2];
	assign triad_s1[3] = (pass_ff) ? triad_s0[3] : triad_inj[3];
	assign triad_s1[4] = (pass_ff) ? triad_s0[4] : triad_inj[4];
	assign triad_s1[5] = (pass_ff) ? triad_s0[5] : triad_inj[5];

	assign triad_tp	= triad_s1[2][1];	// Triad 1 hs4567 layer 2 test point for internal scope

//-------------------------------------------------------------------------------------------------------------------
// Stage 2: Check for CFEB bits stuck at logic 1 for too long + Apply hot channel mask
//-------------------------------------------------------------------------------------------------------------------
// FF buffer control inputs
	reg [15:0]	cfeb_badbits_nbx_minus1 = 16'hFFFF;
	reg			cfeb_badbits_block_ena  = 0;
	reg			single_bx_mode = 0;

	always @(posedge clock) begin
	cfeb_badbits_block_ena	<= cfeb_badbits_block;
	cfeb_badbits_nbx_minus1	<= cfeb_badbits_nbx-1'b1;
	single_bx_mode			<= cfeb_badbits_nbx==1;
	end

// Periodic check pulse counter
	reg [15:0] check_cnt=16'h000F;

	wire check_cnt_ena = (check_cnt < cfeb_badbits_nbx_minus1);
	
	always @(posedge clock) begin
	if      (cfeb_badbits_reset) check_cnt <= 0;
	else if (check_cnt_ena     ) check_cnt <= check_cnt+1'b1;
	else                         check_cnt <= 0;
	end

	wire check_pulse = (check_cnt==0);

// Check CFEB bits with high-too-long state machine
	wire [MXDS-1:0] badbits [MXLY-1:0];

	genvar ids;
	genvar ily;
	generate
	for (ids=0; ids<MXDS; ids=ids+1) begin: ckbitds
	for (ily=0; ily<MXLY; ily=ily+1) begin: ckbitly

	cfeb_bit_check ucfeb_bit_check (
	.clock			(clock),				// 40MHz main clock
	.reset			(cfeb_badbits_reset),	// Clear stuck bit FFs
	.check_pulse	(check_pulse),			// Periodic checking
	.single_bx_mode	(single_bx_mode),		// Check for single bx pulses		
	.bit_in			(triad_s1[ily][ids]),	// Bit to check
	.bit_bad		(badbits[ily][ids]));	// Bit went bad flag

	end
	end
	endgenerate

// Summary badbits for this CFEB
	reg cfeb_badbits_found=0;

	wire cfeb_badbits_or =
	(|badbits[0][7:0])|
	(|badbits[1][7:0])|
	(|badbits[2][7:0])|
	(|badbits[3][7:0])|
	(|badbits[4][7:0])|
	(|badbits[5][7:0]);

	always @(posedge clock) begin
	cfeb_badbits_found <= cfeb_badbits_or;
	end

// Blocked triad bits list, 1=blocked 0=ok to tuse
	reg [MXDS-1:0] blockedbits [MXLY-1:0];

	always @(posedge clock) begin
	blockedbits[0] <= ~ly0_hcm | (badbits[0] & {MXDS {cfeb_badbits_block_ena}});
	blockedbits[1] <= ~ly1_hcm | (badbits[1] & {MXDS {cfeb_badbits_block_ena}});
	blockedbits[2] <= ~ly2_hcm | (badbits[2] & {MXDS {cfeb_badbits_block_ena}});
	blockedbits[3] <= ~ly3_hcm | (badbits[3] & {MXDS {cfeb_badbits_block_ena}});
	blockedbits[4] <= ~ly4_hcm | (badbits[4] & {MXDS {cfeb_badbits_block_ena}});
	blockedbits[5] <= ~ly5_hcm | (badbits[5] & {MXDS {cfeb_badbits_block_ena}});
	end

// Apply Hot Channel Mask to block Errant DiStrips: 1=enable DiStrip, not blocking hstrips, they share a triad start bit
	wire [MXDS-1:0] triad_s2 [MXLY-1:0];	// Masked triads

	assign triad_s2[0] = triad_s1[0] & ~blockedbits[0];
	assign triad_s2[1] = triad_s1[1] & ~blockedbits[1];
	assign triad_s2[2] = triad_s1[2] & ~blockedbits[2];
	assign triad_s2[3] = triad_s1[3] & ~blockedbits[3];
	assign triad_s2[4] = triad_s1[4] & ~blockedbits[4];
	assign triad_s2[5] = triad_s1[5] & ~blockedbits[5];

// Map 2D arrays to 1D for VME
	assign ly0_badbits = badbits[0];
	assign ly1_badbits = badbits[1];
	assign ly2_badbits = badbits[2];
	assign ly3_badbits = badbits[3];
	assign ly4_badbits = badbits[4];
	assign ly5_badbits = badbits[5];

// Map to 1D 48bits for readout machine, mark all blocked if mask_all=0
	assign cfeb_blockedbits = (mask_all) ? {blockedbits[5],blockedbits[4],blockedbits[3],blockedbits[2],blockedbits[1],blockedbits[0]}
	                                     : 48'hFFFFFFFFFFFF;
//-------------------------------------------------------------------------------------------------------------------
// Raw hits RAM storage
//-------------------------------------------------------------------------------------------------------------------
// Calculate parity for raw hits RAM write data
	wire [MXLY-1:0] parity_wr;
	wire [MXLY-1:0] parity_rd;

	assign parity_wr[0] = ~(^ triad_s2[0][MXDS-1:0]);
	assign parity_wr[1] = ~(^ triad_s2[1][MXDS-1:0]);
	assign parity_wr[2] = ~(^ triad_s2[2][MXDS-1:0]);
	assign parity_wr[3] = ~(^ triad_s2[3][MXDS-1:0]);
	assign parity_wr[4] = ~(^ triad_s2[4][MXDS-1:0]);
	assign parity_wr[5] = ~(^ triad_s2[5][MXDS-1:0]);

// Raw hits RAM writes incoming hits into port A, reads out to DMB via port B
	wire [MXDS-1:0] fifo_rdata_ly [MXLY-1:0];

	initial $display("cfeb: generating Virtex2 RAMB16_S9_S9 raw.rawhits_ram");
	wire [MXLY-1:0] dopa;

	generate
	for (ily=0; ily<=MXLY-1; ily=ily+1) begin: raw
	RAMB16_S9_S9 #(
	.WRITE_MODE_A		("READ_FIRST"),			// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.WRITE_MODE_B		("READ_FIRST"),			// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.SIM_COLLISION_CHECK("ALL")					// "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
	) rawhits_ram (
	.WEA				(fifo_wen),				// Port A Write Enable Input
	.ENA				(1'b1),					// Port A RAM Enable Input
	.SSRA				(1'b0),					// Port A Synchronous Set/Reset Input
	.CLKA				(clock),				// Port A Clock
	.ADDRA				(fifo_wadr[10:0]),		// Port A 11-bit Address Input
	.DIA				(triad_s2[ily]),		// Port A 8-bit Data Input
	.DIPA				(parity_wr[ily]),		// Port A 1-bit parity Input
	.DOA				(),						// Port A 8-bit Data Output
	.DOPA				(dopa[ily]),			// Port A 1-bit Parity Output

	.WEB				(1'b0),					// Port B Write Enable Input
	.ENB				(1'b1),					// Port B RAM Enable Input
	.SSRB				(1'b0),					// Port B Synchronous Set/Reset Input
	.CLKB				(clock),				// Port B Clock
	.ADDRB				(fifo_radr),			// Port B 11-bit Address Input
	.DIB				(8'h00),				// Port B 8-bit Data Input
	.DIPB				(),						// Port-B 1-bit parity Input
	.DOB				(fifo_rdata_ly[ily]),	// Port B 8-bit Data Output
	.DOPB				(parity_rd[ily])		// Port B 1-bit Parity Output
   );
	end
	endgenerate

// Compare read parity to write parity
	wire [MXLY-1:0] parity_expect;

	assign parity_expect[0] = ~(^ fifo_rdata_ly[0]);
	assign parity_expect[1] = ~(^ fifo_rdata_ly[1]);
	assign parity_expect[2] = ~(^ fifo_rdata_ly[2]);
	assign parity_expect[3] = ~(^ fifo_rdata_ly[3]);
	assign parity_expect[4] = ~(^ fifo_rdata_ly[4]);
	assign parity_expect[5] = ~(^ fifo_rdata_ly[5]);

	assign parity_err_cfeb[5:0] =  ~(parity_rd ~^ parity_expect);	// ~^ is bitwise equivalence operator

// Multiplex Raw Hits FIFO RAM output data
	assign fifo_rdata = fifo_rdata_ly[fifo_sel];

//-------------------------------------------------------------------------------------------------------------------
// Stage 3:	Triad Decoder
//			Decodes Triads into DiStrips, Strips, 1/2-Strips.
//			Digital One-shots stretch 1/2-Strip pulses for pattern finding.
//			Hot channel mask applied to Triad DiStrips after storage, but before Triad decoder
//-------------------------------------------------------------------------------------------------------------------
// Local buffer Triad Decoder controls
	reg			persist1 = 0;
	reg	[3:0]	persist  = 0;

	always @(posedge clock) begin
	persist	 <=	 triad_persist-1'b1;
	persist1 <=	(triad_persist==1 || triad_persist==0);
	end

// Instantiate mxly*mxds = 48 triad decoders
	wire [MXDS-1:0] tskip [MXLY-1:0];	// Skipped triads
	wire [MXHS-1:0] hs    [MXLY-1:0];	// Decoded 1/2-strip pulses

	generate
	for (ily=0; ily<=MXLY-1; ily=ily+1) begin: ily_loop
	for (ids=0; ids<=MXDS-1; ids=ids+1) begin: ids_loop
	triad_decode utriad(clock,treset,persist,persist1,triad_s2[ily][ids],hs[ily][3+ids*4:ids*4],tskip[ily][ids]);
	end
	end
	endgenerate

	assign triad_skip = (|tskip[0]) | (|tskip[1]) | (|tskip[2]) | (|tskip[3]) | (|tskip[4]) | (|tskip[5]);

	reg [5:0] nhits_s0 = 6'b0;
    reg [MXLY-1:0] layers_with_hit_s0 = 0;
    
    assign nhits_per_cfeb = nhits_s0;
    assign layers_withhits_per_cfeb = layers_with_hit_s0;

// Expand 2d arrays for transmission to next module
	assign ly0hs = hs[0];
	assign ly1hs = hs[1];
	assign ly2hs = hs[2];
	assign ly3hs = hs[3];
	assign ly4hs = hs[4];
	assign ly5hs = hs[5];

// Unused signals
	assign cfeb_sump = | dopa;

//------------------------------------------------------------------------------------------------------------------
// Debug
//------------------------------------------------------------------------------------------------------------------
`ifdef DEBUG_CFEB
// Injector State Machine ASCII display
	reg[71:0] inj_sm_dsp;
	always @* begin
	case (inj_sm)
	pass:		inj_sm_dsp <= "pass     ";
	injecting:	inj_sm_dsp <= "injecting";
	default		inj_sm_dsp <= "pass     ";
	endcase
	end

// Raw hits RAM outputs
	assign fifo_rdata_lyr0 = fifo_rdata_ly[0];
	assign fifo_rdata_lyr1 = fifo_rdata_ly[1];
	assign fifo_rdata_lyr2 = fifo_rdata_ly[2];
	assign fifo_rdata_lyr3 = fifo_rdata_ly[3];
	assign fifo_rdata_lyr4 = fifo_rdata_ly[4];
	assign fifo_rdata_lyr5 = fifo_rdata_ly[5];
`endif

//------------------------------------------------------------------------------------------------------------------
	endmodule
//------------------------------------------------------------------------------------------------------------------
