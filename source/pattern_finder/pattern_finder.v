`timescale 1ns / 1ps
//`define DEBUG_PATTERN_FINDER		// Turn on debug mode
//-------------------------------------------------------------------------------------------------------------------
// Conditional compile flags, normally set by global defines. Override here for standalone debugging
//-------------------------------------------------------------------------------------------------------------------
//	`define CSC_TYPE_A			04'hA		// Normal   CSC
//	`define CSC_TYPE_B			04'hB		// Reversed CSC
//	`define CSC_TYPE_C			04'hC		// Normal   ME1B reversed ME1A
//	`define CSC_TYPE_D			04'hD		// Reversed ME1B:normal   ME1A
//-------------------------------------------------------------------------------------------------------------------
// CLCT Pattern Finder
// Algorithm: 9-Pattern Front-end 80 MHz pattern-unit duplexing
//
// Process 5 CFEBs:
//		Input	32 1/2-Strips x 6 CSC Layers x 5 CFEBs
//		Output	Best 2 of 160 CLCTs
//			   +DMB pre-trigger signals
//------------------------------------------------------------------------------------------------------------------
//	01/31/07 Initial
//	02/14/07 Add 1st clct logic
//	02/20/07 Port from pattern6 back-end version
//	02/26/07 Phase-align pattern unit pipeline
//	02/27/07 Reposition pipleline for speed, change to xor LUT for sel, FF version was slower
//	02/28/07 Add 2nd CLCT
//	03/02/07 Add pid_thresh_pretrig and dmb_thresh_pretrig
//	03/07/07 Add layer-or barrel delay
//	03/12/07 Ifdef temporary 1st stage FFs and DCM
//	05/08/07 Change pattern numbers 1-9 to 0-8 so lsb now implies bend direction
//	05/09/07 Remove debug code, shift layer trigger output 1bx earlier to match pre-trig
//	05/16/07 Mod a/b mulitpler to force initial state and move from lut to 80mhz ff
//	05/22/07 Reduce to just one vme clct separation parameter
//	06/13/07 Replace a/b mux flipflops with logic accessible clock signal 
//	06/14/07 Add serial fanout for a/b mux select signal
//	06/15/07 Incorporate layer mode as pattern 1, shift clct patterns IDs to the range 2-10
//	06/18/07 Automatic adjacent mask generation, add layer mode to pattern output
//	06/19/07 Tune layer delay for clct mux
//	06/20/07 One more tune session, remove debug code
//	06/28/07 Shift key layer to ly2, flip patterns top-to-bottom, left-to-right, old ly0 becomes new ly5
//	07/02/07 Mod pattern_unit.v to match prior ly3 result, reduces fpga usage from 93% to 90%
//	07/03/07 Mod pid_thresh_pretrig AND logic
//	07/05/07 Fix adjcfeb logic, extend masks to span full cfeb, increase adjcfeb_dist to 6 bits
//	07/09/08 Rename thresholds
//	07/30/08 Add debug mode, add thresh FF inits
//	07/31/08 Expand reset to purge pattern finder pipelines
//	08/01/08 Change csc_stagger to be an output, add non-stagger mode as a debug option
//	08/20/08 Return to programmable csc stagger, add programmable me1a/b hs reversal
//	08/21/08 Revert once again to ifdef csc stagger, was too slow
//	08/23/08 Add reverse option for normal cscs
//	08/28/08 Replace condtional compile switches with csc type code
//	04/23/09 Mod for ise 10.1i
//	08/20/09 Add register balancing
//	08/21/09 Take out register balancing, ise8.2 does not need it
//	02/04/10 Reverse type b layers
//	02/10/10 Add type b active feb case
//	03/19/10 Mod busy hs delimiters for me1a me1b cscs to separate cfeb4
//	08/11/10 Port to ise 12
//	08/12/10 Fix clct separation ram adr truncation
//	08/12/10 Add virtex6 ifdef to pattern_unit, replace stop with finish
//	08/13/10 Change dcm feedback to 1x, ise 12 does not allow 2x
//	08/13/10 Change sm to non-blocking operators, shorten ascii sm vector
//	08/16/10 Change sm to start up in purge mode
//	08/30/10 Mod integer length for types c,d
//	11/24/10 Replace inferred clct separation ram with library instance, finally ditch that clct_sep_ram_init.dat file
//	12/01/10 Remove leftover sep ram array
//-------------------------------------------------------------------------------------------------------------------
	module pattern_finder
	(
// Clock Ports
	clock,
	clock_lac,
	clock_2x,
	global_reset,

`ifndef DEBUG_PATTERN_FINDER
// CFEB Ports
	cfeb0_ly0hs, cfeb0_ly1hs, cfeb0_ly2hs, cfeb0_ly3hs, cfeb0_ly4hs, cfeb0_ly5hs,
	cfeb1_ly0hs, cfeb1_ly1hs, cfeb1_ly2hs, cfeb1_ly3hs, cfeb1_ly4hs, cfeb1_ly5hs,
	cfeb2_ly0hs, cfeb2_ly1hs, cfeb2_ly2hs, cfeb2_ly3hs, cfeb2_ly4hs, cfeb2_ly5hs,
	cfeb3_ly0hs, cfeb3_ly1hs, cfeb3_ly2hs, cfeb3_ly3hs, cfeb3_ly4hs, cfeb3_ly5hs,
	cfeb4_ly0hs, cfeb4_ly1hs, cfeb4_ly2hs, cfeb4_ly3hs, cfeb4_ly4hs, cfeb4_ly5hs,
`else
// CFEB Ports, debug
	tmb_clock0,
	cfeb0_ly0hst, cfeb0_ly1hst, cfeb0_ly2hst, cfeb0_ly3hst, cfeb0_ly4hst, cfeb0_ly5hst,
	cfeb1_ly0hst, cfeb1_ly1hst, cfeb1_ly2hst, cfeb1_ly3hst, cfeb1_ly4hst, cfeb1_ly5hst,
	cfeb2_ly0hst, cfeb2_ly1hst, cfeb2_ly2hst, cfeb2_ly3hst, cfeb2_ly4hst, cfeb2_ly5hst,
	cfeb3_ly0hst, cfeb3_ly1hst, cfeb3_ly2hst, cfeb3_ly3hst, cfeb3_ly4hst, cfeb3_ly5hst,
	cfeb4_ly0hst, cfeb4_ly1hst, cfeb4_ly2hst, cfeb4_ly3hst, cfeb4_ly4hst, cfeb4_ly5hst,
`endif

// CSC Orientation Ports
	csc_type,
	csc_me1ab,
	stagger_hs_csc,
	reverse_hs_csc,
	reverse_hs_me1a,
	reverse_hs_me1b,

// PreTrigger Ports
	layer_trig_en,
	lyr_thresh_pretrig,
	hit_thresh_pretrig,
	pid_thresh_pretrig,
	dmb_thresh_pretrig,
	cfeb_en,
	adjcfeb_dist,
	clct_blanking,

	cfeb_hit,
	cfeb_active,

	cfeb_layer_trig,
	cfeb_layer_or,
	cfeb_nlayers_hit,

// 2nd CLCT separation RAM Ports
	clct_sep_src,
	clct_sep_vme,
	clct_sep_ram_we,
	clct_sep_ram_adr,
	clct_sep_ram_wdata,
	clct_sep_ram_rdata,

// CLCT Pattern-finder results
	hs_hit_1st,
	hs_pid_1st,
	hs_key_1st,

	hs_hit_2nd,
	hs_pid_2nd,
	hs_key_2nd,
	hs_bsy_2nd,

	hs_layer_trig,
	hs_nlayers_hit,
	hs_layer_or

// Debug
`ifdef DEBUG_PATTERN_FINDER
	,purge_sm_dsp
	,reset
	,lock

	,lyr_thresh_pretrig_ff
	,hit_thresh_pretrig_ff
	,pid_thresh_pretrig_ff
	,dmb_thresh_pretrig_ff
	,cfeb_en_ff
	,layer_trig_en_ff

	,busy_min
	,busy_max
	,busy_key
	,clct0_is_on_me1a
`endif
	);

//-------------------------------------------------------------------------------------------------------------------
// Constants
//-------------------------------------------------------------------------------------------------------------------
	parameter MXCFEB		= 	5;				// Number of CFEBs on CSC
	parameter MXLY			=	6;				// Number Layers in CSC
	parameter MXDS			=	8;				// Number of DiStrips per layer on 1 CFEB
	parameter MXDSX			=	MXCFEB*MXDS;	// Number of DiStrips per layer on 5 CFEBs
	parameter MXHS			=	32;				// Number of 1/2-Strips per layer on 1 CFEB
	parameter MXHSX			=	MXCFEB*MXHS;	// Number of 1/2-Strips per layer on 5 CFEBs
	parameter MXKEY			=	MXHS;			// Number of key 1/2-strips on 1 CFEB
	parameter MXKEYB		=	5;				// Number of 1/2-strip key bits on 1 CFEB
	parameter MXKEYX		=	MXHSX;			// Number of key 1/2-strips on 5 CFEBs
	parameter MXKEYBX		=	8;				// Number of 1/2-strip key bits on 5 CFEBs

	parameter MXPIDB		=	4;				// Pattern ID bits
	parameter MXHITB		=	3;				// Hits on pattern bits
	parameter MXPATB		=	3+4;			// Pattern bits

//-------------------------------------------------------------------------------------------------------------------
// Ports
//-------------------------------------------------------------------------------------------------------------------
`ifndef DEBUG_PATTERN_FINDER
// Clock Ports
	input					clock;				// 40MHz TMB main clock
	input					clock_lac;			// 40MHz logic accessible clock
	input					clock_2x;			// 80MHz commutator clock
	input					global_reset;		// 1=Reset everything

// CFEB Ports									// Triad decoder 1/2-strip pulses
	input	[MXHS-1:0]		cfeb0_ly0hs, cfeb0_ly1hs, cfeb0_ly2hs, cfeb0_ly3hs, cfeb0_ly4hs, cfeb0_ly5hs;
	input	[MXHS-1:0]		cfeb1_ly0hs, cfeb1_ly1hs, cfeb1_ly2hs, cfeb1_ly3hs, cfeb1_ly4hs, cfeb1_ly5hs;
	input	[MXHS-1:0]		cfeb2_ly0hs, cfeb2_ly1hs, cfeb2_ly2hs, cfeb2_ly3hs, cfeb2_ly4hs, cfeb2_ly5hs;
	input	[MXHS-1:0]		cfeb3_ly0hs, cfeb3_ly1hs, cfeb3_ly2hs, cfeb3_ly3hs, cfeb3_ly4hs, cfeb3_ly5hs;
	input	[MXHS-1:0]		cfeb4_ly0hs, cfeb4_ly1hs, cfeb4_ly2hs, cfeb4_ly3hs, cfeb4_ly4hs, cfeb4_ly5hs;
`else									
// Clock Ports, debug
	output					clock;				// 40MHz TMB main clock
	output					clock_lac;			// 40MHz logic accessible clock
	output					clock_2x;			// 80MHz commutator clock
	input					global_reset;		// 1=Reset everything
	input					tmb_clock0;

// CFEB Ports, debug							// Triad decoder 1/2-strip pulses, FF buffered for sim
	input	[MXHS-1:0]		cfeb0_ly0hst, cfeb0_ly1hst, cfeb0_ly2hst, cfeb0_ly3hst, cfeb0_ly4hst, cfeb0_ly5hst;
	input	[MXHS-1:0]		cfeb1_ly0hst, cfeb1_ly1hst, cfeb1_ly2hst, cfeb1_ly3hst, cfeb1_ly4hst, cfeb1_ly5hst;
	input	[MXHS-1:0]		cfeb2_ly0hst, cfeb2_ly1hst, cfeb2_ly2hst, cfeb2_ly3hst, cfeb2_ly4hst, cfeb2_ly5hst;
	input	[MXHS-1:0]		cfeb3_ly0hst, cfeb3_ly1hst, cfeb3_ly2hst, cfeb3_ly3hst, cfeb3_ly4hst, cfeb3_ly5hst;
	input	[MXHS-1:0]		cfeb4_ly0hst, cfeb4_ly1hst, cfeb4_ly2hst, cfeb4_ly3hst, cfeb4_ly4hst, cfeb4_ly5hst;
`endif

// CSC Orientation Ports
	output	[3:0]			csc_type;			// Firmware compile type
	output					csc_me1ab;			// 1=ME1A or ME1B CSC type
	output					stagger_hs_csc;		// 1=Staggered CSC non-me1, 0=non-staggered me1
	output					reverse_hs_csc;		// 1=Reverse staggered CSC, non-me1
	output					reverse_hs_me1a;	// 1=reverse me1a hstrips prior to pattern sorting
	output					reverse_hs_me1b;	// 1=reverse me1b hstrips prior to pattern sorting

// PreTrigger Ports
	input					layer_trig_en;		// 1=Enable layer trigger mode
	input	[MXHITB-1:0]	lyr_thresh_pretrig;	// Layers hit pre-trigger threshold
	input	[MXHITB-1:0]	hit_thresh_pretrig;	// Hits on pattern template pre-trigger threshold
	input	[MXPIDB-1:0]	pid_thresh_pretrig;	// Pattern shape ID pre-trigger threshold
	input	[MXHITB-1:0]	dmb_thresh_pretrig;	// Hits on pattern template DMB active-feb threshold
	input	[MXCFEB-1:0]	cfeb_en;			// 1=Enable cfeb for pre-triggering
	input	[MXKEYB-1+1:0]	adjcfeb_dist;		// Distance from key to cfeb boundary for marking adjacent cfeb as hit
	input					clct_blanking;		// 1=Blank clct outputs if zero hits

	output	[MXCFEB-1:0]	cfeb_hit;			// This CFEB has a pattern over pre-trigger threshold
	output	[MXCFEB-1:0]	cfeb_active;		// CFEBs marked for DMB readout

	output					cfeb_layer_trig;	// Layer pretrigger
	output	[MXLY-1:0]		cfeb_layer_or;		// OR of hstrips on each layer
	output	[MXHITB-1:0]	cfeb_nlayers_hit;	// Number of CSC layers hit

// 2nd CLCT separation RAM Ports
	input					clct_sep_src;		// CLCT separation source 1=vme, 0=ram
	input	[7:0]			clct_sep_vme;		// CLCT separation from vme
	input					clct_sep_ram_we;	// CLCT separation RAM write enable
	input	[3:0]			clct_sep_ram_adr;	// CLCT separation RAM rw address VME
	input	[15:0]			clct_sep_ram_wdata;	// CLCT separation RAM write data VME
	output	[15:0]			clct_sep_ram_rdata;	// CLCT separation RAM read  data VME

// CLCT Pattern-finder results
	output	[MXHITB-1:0]	hs_hit_1st;			// 1st CLCT pattern hits
	output	[MXPIDB-1:0]	hs_pid_1st;			// 1st CLCT pattern ID
	output	[MXKEYBX-1:0]	hs_key_1st;			// 1st CLCT key 1/2-strip

	output	[MXHITB-1:0]	hs_hit_2nd;			// 2nd CLCT pattern hits
	output	[MXPIDB-1:0]	hs_pid_2nd;			// 2nd CLCT pattern ID
	output	[MXKEYBX-1:0]	hs_key_2nd;			// 2nd CLCT key 1/2-strip
	output					hs_bsy_2nd;			// 2nd CLCT busy, logic error indicator

	output					hs_layer_trig;		// Layer triggered
	output	[MXHITB-1:0]	hs_nlayers_hit;		// Number of layers hit
	output	[MXLY-1:0]		hs_layer_or;		// Layer OR
	
// Debug
`ifdef DEBUG_PATTERN_FINDER
	output	[39:0]			purge_sm_dsp;
	output					reset;
	output					lock;

	output	[MXHITB-1:0]	lyr_thresh_pretrig_ff;
	output	[MXHITB-1:0]	hit_thresh_pretrig_ff;
	output	[MXPIDB-1:0]	pid_thresh_pretrig_ff;
	output	[MXHITB-1:0]	dmb_thresh_pretrig_ff;
	output	[MXCFEB-1:0]	cfeb_en_ff;
	output					layer_trig_en_ff;

	output	[MXKEYBX-1:0]	busy_min;
	output	[MXKEYBX-1:0]	busy_max;
	output	[MXHSX-1:0]		busy_key;
	output					clct0_is_on_me1a;

`endif
//-------------------------------------------------------------------------------------------------------------------
// Load global definitions
//-------------------------------------------------------------------------------------------------------------------
	`include "source/tmb_virtex2_fw_version.v"
	`ifdef CSC_TYPE_A initial $display ("CSC_TYPE_A=%H",`CSC_TYPE_A); `endif	// Normal   CSC
	`ifdef CSC_TYPE_B initial $display ("CSC_TYPE_B=%H",`CSC_TYPE_B); `endif	// Reversed CSC
	`ifdef CSC_TYPE_C initial $display ("CSC_TYPE_C=%H",`CSC_TYPE_C); `endif	// Normal	ME1B reversed ME1A
	`ifdef CSC_TYPE_D initial $display ("CSC_TYPE_D=%H",`CSC_TYPE_D); `endif	// Reversed ME1B normal   ME1A
	`ifdef VIRTEX2    initial $display ("VIRTEX2   =%H",`VIRTEX2   ); `endif	// Virtex 2 Mezzanine card

//-------------------------------------------------------------------------------------------------------------------
// Debug mode, FF aligns inputs, and has local DLL to generate 2x clock and lac clock
//-------------------------------------------------------------------------------------------------------------------
`ifdef DEBUG_PATTERN_FINDER
// Flip-flop align hs inputs
	reg	[MXHS-1:0] cfeb0_ly0hs, cfeb0_ly1hs, cfeb0_ly2hs, cfeb0_ly3hs, cfeb0_ly4hs, cfeb0_ly5hs;
	reg	[MXHS-1:0] cfeb1_ly0hs, cfeb1_ly1hs, cfeb1_ly2hs, cfeb1_ly3hs, cfeb1_ly4hs, cfeb1_ly5hs;
	reg	[MXHS-1:0] cfeb2_ly0hs, cfeb2_ly1hs, cfeb2_ly2hs, cfeb2_ly3hs, cfeb2_ly4hs, cfeb2_ly5hs;
	reg	[MXHS-1:0] cfeb3_ly0hs, cfeb3_ly1hs, cfeb3_ly2hs, cfeb3_ly3hs, cfeb3_ly4hs, cfeb3_ly5hs;
	reg	[MXHS-1:0] cfeb4_ly0hs, cfeb4_ly1hs, cfeb4_ly2hs, cfeb4_ly3hs, cfeb4_ly4hs, cfeb4_ly5hs;

	wire clock;
	always @(posedge clock) begin
	{cfeb0_ly5hs,cfeb0_ly4hs,cfeb0_ly3hs,cfeb0_ly2hs,cfeb0_ly1hs,cfeb0_ly0hs}<={cfeb0_ly5hst,cfeb0_ly4hst,cfeb0_ly3hst,cfeb0_ly2hst,cfeb0_ly1hst,cfeb0_ly0hst};
	{cfeb1_ly5hs,cfeb1_ly4hs,cfeb1_ly3hs,cfeb1_ly2hs,cfeb1_ly1hs,cfeb1_ly0hs}<={cfeb1_ly5hst,cfeb1_ly4hst,cfeb1_ly3hst,cfeb1_ly2hst,cfeb1_ly1hst,cfeb1_ly0hst};
	{cfeb2_ly5hs,cfeb2_ly4hs,cfeb2_ly3hs,cfeb2_ly2hs,cfeb2_ly1hs,cfeb2_ly0hs}<={cfeb2_ly5hst,cfeb2_ly4hst,cfeb2_ly3hst,cfeb2_ly2hst,cfeb2_ly1hst,cfeb2_ly0hst};
	{cfeb3_ly5hs,cfeb3_ly4hs,cfeb3_ly3hs,cfeb3_ly2hs,cfeb3_ly1hs,cfeb3_ly0hs}<={cfeb3_ly5hst,cfeb3_ly4hst,cfeb3_ly3hst,cfeb3_ly2hst,cfeb3_ly1hst,cfeb3_ly0hst};
	{cfeb4_ly5hs,cfeb4_ly4hs,cfeb4_ly3hs,cfeb4_ly2hs,cfeb4_ly1hs,cfeb4_ly0hs}<={cfeb4_ly5hst,cfeb4_ly4hst,cfeb4_ly3hst,cfeb4_ly2hst,cfeb4_ly1hst,cfeb4_ly0hst};
	end

// Global clock input buffers
	IBUFG uibufg4p  (.I(tmb_clock0  ),.O(tmb_clock0_ibufg));	// synthesis attribute LOC of uibufg4p   is "AF18"
	BUFG ugbuftmb1x	(.I(clock_dcm   ),.O(clock           ));	// synthesis attribute LOC of ugbuftmb1x is "BUFGMUX0P"
	BUFG ugbuftmb2x	(.I(clock_2x_dcm),.O(clock_2x        ));	// synthesis attribute LOC of ugbuftmb2x is "BUFGMUX2P"
	
// Main TMB DLL generates clocks at 1x=40MHz, 2x=80MHz, and 1/4 =10MHz
	DCM udcmtmb (
	.CLKIN		(tmb_clock0_ibufg),
	.CLKFB		(clock),
	.RST		(1'b0),
	.DSSEN		(1'b0),
	.PSINCDEC	(1'b0),
	.PSEN		(1'b0),
	.PSCLK		(1'b0),
	.CLK0		(clock_dcm),
	.CLK90		(clock_dcm_90),
	.CLK180		(),
	.CLK270		(),
	.CLK2X		(clock_2x_dcm),
	.CLK2X180	(),
	.CLKDV		(),
	.CLKFX		(),
	.CLKFX180	(),
	.LOCKED		(lock),
	.STATUS		(),
	.PSDONE		());
	defparam udcmtmb.STARTUP_WAIT = "TRUE";
	defparam udcmtmb.CLK_FEEDBACK = "1X";

// Logic Accessible clock
   FDRSE #(.INIT(1'b0)) u0 (// Initial value of register
	.Q	(clock_lac),		// Data output
	.C	(clock_2x),			// Clock input
	.CE	(1'b1),				// Clock enable input
	.D	(!clock_dcm_90),	// Data input
	.R	(1'b0),				// Synchronous reset input
	.S	(1'b0));			// Synchronous set input

// ME1A signal is not used for debug versions of type a or type b
	`ifdef CSC_TYPE_A wire clct0_is_on_me1a=0; `endif
	`ifdef CSC_TYPE_B wire clct0_is_on_me1a=0; `endif

 `endif
//-------------------------------------------------------------------------------------------------------------------
// Stage 4A1: Power  up, reset, and purge
//-------------------------------------------------------------------------------------------------------------------
	reg  ready = 0;
	wire reset = !ready;

	always @(posedge clock) begin
	ready <= !global_reset;
	end

// Pipeline purge blanks pattern finder until pipes are cleared
	reg [1:0] purge_sm;		// synthesis attribute safe_implementation of purge_sm is yes;
	parameter	pass	= 0;
	parameter	purge	= 1;

	reg	[2:0] purge_cnt = 0;

	always @(posedge clock) begin
	if      (reset)				purge_cnt <= 0;
	else if (purge_sm==purge)	purge_cnt <= purge_cnt+1'b1;
	else						purge_cnt <= 0;
	end

	wire purge_done = (purge_cnt==7);
	wire purging    = (purge_sm==purge) || reset;

// Pipeline purge state machine
	initial purge_sm = purge;

	always @(posedge clock) begin
	if (reset)              purge_sm <= purge;
	else begin
	case (purge_sm)
	pass:					purge_sm <= pass;
	purge:	if (purge_done)	purge_sm <= pass;
	endcase
	end
	end

`ifdef DEBUG_PATTERN_FINDER
	reg[39:0] purge_sm_dsp;
	always @* begin
	case (purge_sm)
	pass:	purge_sm_dsp <= "pass ";
	purge:	purge_sm_dsp <= "purge";
	default	purge_sm_dsp <= "error";
	endcase
	end
`endif
//-------------------------------------------------------------------------------------------------------------------
// Local copy of number-planes-hit pretrigger threshold powers up with high threshold to block spurious patterns
//-------------------------------------------------------------------------------------------------------------------
	reg [MXHITB-1:0] lyr_thresh_pretrig_ff	= 3'h7;		// Layers hit pre-trigger threshold
	reg [MXHITB-1:0] hit_thresh_pretrig_ff	= 3'h7;		// Hits on pattern template pre-trigger threshold
	reg [MXPIDB-1:0] pid_thresh_pretrig_ff	= 4'hF;		// Pattern shape ID pre-trigger threshold
	reg [MXHITB-1:0] dmb_thresh_pretrig_ff	= 3'h7;		// Hits on pattern template DMB active-feb threshold
	reg [MXCFEB-1:0] cfeb_en_ff				= 5'h00;	// CFEB enabled for pre-triggering
	reg				 layer_trig_en_ff		= 1'b0;		// Layer trigger mode enabled

	always @(posedge clock) begin
	if(purging) begin						// Transient power-up values
	lyr_thresh_pretrig_ff	<= 3'h7;
	hit_thresh_pretrig_ff 	<= 3'h7;
	pid_thresh_pretrig_ff 	<= 4'hF;
	dmb_thresh_pretrig_ff 	<= 3'h7;
	cfeb_en_ff				<= 5'h00;
	layer_trig_en_ff		<= 1'b0;
	end
	else begin								// Subsequent VME values
	lyr_thresh_pretrig_ff	<= lyr_thresh_pretrig;
	hit_thresh_pretrig_ff 	<= hit_thresh_pretrig;
	pid_thresh_pretrig_ff 	<= pid_thresh_pretrig;
	dmb_thresh_pretrig_ff 	<= dmb_thresh_pretrig;
	cfeb_en_ff				<= cfeb_en;
	layer_trig_en_ff		<= layer_trig_en;
	end
	end

// Generate mask for marking adjacent cfeb as hit if nearby keys are over thresh
	reg [MXHS-1:0] adjcfeb_mask_nm1;	// Adjacent CFEB active feb flag mask
	reg [MXHS-1:0] adjcfeb_mask_np1;

	genvar ihs;
	generate
	for (ihs=0; ihs<=31; ihs=ihs+1) begin: genmask
	always @(posedge clock) begin
	adjcfeb_mask_nm1[ihs]	 <= (ihs<adjcfeb_dist);
	adjcfeb_mask_np1[31-ihs] <= (ihs<adjcfeb_dist);
	end
	end
	endgenerate

//-------------------------------------------------------------------------------------------------------------------
// Stage 4A1: CSC_TYPE_A Normal CSC
//-------------------------------------------------------------------------------------------------------------------
`ifdef CSC_TYPE_A
`define	STAGGER_HS_CSC 01'h1
`define CSC_TYPE_A_or_CSC_TYPE_B 01'h1
`define CSC_TYPE_A_or_CSC_TYPE_C 01'h1

	wire [MXHS*5-1:0] me1234_ly0hs; 
	wire [MXHS*5-1:0] me1234_ly1hs; 
	wire [MXHS*5-1:0] me1234_ly2hs; 
	wire [MXHS*5-1:0] me1234_ly3hs; 
	wire [MXHS*5-1:0] me1234_ly4hs; 
	wire [MXHS*5-1:0] me1234_ly5hs; 

// Orientation flags
	assign csc_type        = 4'hA;		// Firmware compile type
	assign csc_me1ab	   = 0;			// 1= ME1A or ME1B CSC
	assign stagger_hs_csc  = 1;			// 1=Staggered CSC non-me1, 0=non-staggered me1
	assign reverse_hs_csc  = 0;			// 1=Reversed  CSC non-me1
	assign reverse_hs_me1a = 0;			// 1=reverse me1a hstrips prior to pattern sorting
	assign reverse_hs_me1b = 0;			// 1=reverse me1b hstrips prior to pattern sorting
	initial $display ("CSC_TYPE_A instantiated");

// Normal CSC cfebs
	assign me1234_ly0hs = {cfeb4_ly0hs, cfeb3_ly0hs, cfeb2_ly0hs, cfeb1_ly0hs, cfeb0_ly0hs};
	assign me1234_ly1hs = {cfeb4_ly1hs, cfeb3_ly1hs, cfeb2_ly1hs, cfeb1_ly1hs, cfeb0_ly1hs};
	assign me1234_ly2hs = {cfeb4_ly2hs, cfeb3_ly2hs, cfeb2_ly2hs, cfeb1_ly2hs, cfeb0_ly2hs};
	assign me1234_ly3hs = {cfeb4_ly3hs, cfeb3_ly3hs, cfeb2_ly3hs, cfeb1_ly3hs, cfeb0_ly3hs};
	assign me1234_ly4hs = {cfeb4_ly4hs, cfeb3_ly4hs, cfeb2_ly4hs, cfeb1_ly4hs, cfeb0_ly4hs};
	assign me1234_ly5hs = {cfeb4_ly5hs, cfeb3_ly5hs, cfeb2_ly5hs, cfeb1_ly5hs, cfeb0_ly5hs};

//-------------------------------------------------------------------------------------------------------------------
// Stage 4A2: CSC_TYPE_B Reversed CSC
//-------------------------------------------------------------------------------------------------------------------
`elsif  CSC_TYPE_B
`define	STAGGER_HS_CSC 01'h1
`define CSC_TYPE_A_or_CSC_TYPE_B 01'h1

	wire [MXHS*5-1:0] me1234_ly0hs; 
	wire [MXHS*5-1:0] me1234_ly1hs; 
	wire [MXHS*5-1:0] me1234_ly2hs; 
	wire [MXHS*5-1:0] me1234_ly3hs; 
	wire [MXHS*5-1:0] me1234_ly4hs; 
	wire [MXHS*5-1:0] me1234_ly5hs;

// Orientation flags
	assign csc_type        = 4'hB;		// Firmware compile type
	assign csc_me1ab	   = 0;			// 1= ME1A or ME1B CSC
	assign stagger_hs_csc  = 1;			// 1=Staggered CSC non-me1
	assign reverse_hs_csc  = 1;			// 1=Reversed  CSC non-me1
	assign reverse_hs_me1a = 0;			// 1=reverse me1a hstrips prior to pattern sorting
	assign reverse_hs_me1b = 0;			// 1=reverse me1b hstrips prior to pattern sorting
	initial $display ("CSC_TYPE_B instantiated");

// Generate hs reversal map for all cfebs
	wire [MXHS-1:0] cfeb0_ly0hsr, cfeb0_ly1hsr, cfeb0_ly2hsr, cfeb0_ly3hsr, cfeb0_ly4hsr, cfeb0_ly5hsr;
	wire [MXHS-1:0] cfeb1_ly0hsr, cfeb1_ly1hsr, cfeb1_ly2hsr, cfeb1_ly3hsr, cfeb1_ly4hsr, cfeb1_ly5hsr;
	wire [MXHS-1:0] cfeb2_ly0hsr, cfeb2_ly1hsr, cfeb2_ly2hsr, cfeb2_ly3hsr, cfeb2_ly4hsr, cfeb2_ly5hsr;
	wire [MXHS-1:0] cfeb3_ly0hsr, cfeb3_ly1hsr, cfeb3_ly2hsr, cfeb3_ly3hsr, cfeb3_ly4hsr, cfeb3_ly5hsr;
	wire [MXHS-1:0] cfeb4_ly0hsr, cfeb4_ly1hsr, cfeb4_ly2hsr, cfeb4_ly3hsr, cfeb4_ly4hsr, cfeb4_ly5hsr;

	generate
	for (ihs=0; ihs<=MXHS-1; ihs=ihs+1) begin: hsrev
	assign cfeb0_ly0hsr[ihs]=cfeb0_ly0hs[(MXHS-1)-ihs];
	assign cfeb0_ly1hsr[ihs]=cfeb0_ly1hs[(MXHS-1)-ihs];
	assign cfeb0_ly2hsr[ihs]=cfeb0_ly2hs[(MXHS-1)-ihs];
	assign cfeb0_ly3hsr[ihs]=cfeb0_ly3hs[(MXHS-1)-ihs];
	assign cfeb0_ly4hsr[ihs]=cfeb0_ly4hs[(MXHS-1)-ihs];
	assign cfeb0_ly5hsr[ihs]=cfeb0_ly5hs[(MXHS-1)-ihs];

	assign cfeb1_ly0hsr[ihs]=cfeb1_ly0hs[(MXHS-1)-ihs];
	assign cfeb1_ly1hsr[ihs]=cfeb1_ly1hs[(MXHS-1)-ihs];
	assign cfeb1_ly2hsr[ihs]=cfeb1_ly2hs[(MXHS-1)-ihs];
	assign cfeb1_ly3hsr[ihs]=cfeb1_ly3hs[(MXHS-1)-ihs];
	assign cfeb1_ly4hsr[ihs]=cfeb1_ly4hs[(MXHS-1)-ihs];
	assign cfeb1_ly5hsr[ihs]=cfeb1_ly5hs[(MXHS-1)-ihs];

	assign cfeb2_ly0hsr[ihs]=cfeb2_ly0hs[(MXHS-1)-ihs];
	assign cfeb2_ly1hsr[ihs]=cfeb2_ly1hs[(MXHS-1)-ihs];
	assign cfeb2_ly2hsr[ihs]=cfeb2_ly2hs[(MXHS-1)-ihs];
	assign cfeb2_ly3hsr[ihs]=cfeb2_ly3hs[(MXHS-1)-ihs];
	assign cfeb2_ly4hsr[ihs]=cfeb2_ly4hs[(MXHS-1)-ihs];
	assign cfeb2_ly5hsr[ihs]=cfeb2_ly5hs[(MXHS-1)-ihs];

	assign cfeb3_ly0hsr[ihs]=cfeb3_ly0hs[(MXHS-1)-ihs];
	assign cfeb3_ly1hsr[ihs]=cfeb3_ly1hs[(MXHS-1)-ihs];
	assign cfeb3_ly2hsr[ihs]=cfeb3_ly2hs[(MXHS-1)-ihs];
	assign cfeb3_ly3hsr[ihs]=cfeb3_ly3hs[(MXHS-1)-ihs];
	assign cfeb3_ly4hsr[ihs]=cfeb3_ly4hs[(MXHS-1)-ihs];
	assign cfeb3_ly5hsr[ihs]=cfeb3_ly5hs[(MXHS-1)-ihs];

	assign cfeb4_ly0hsr[ihs]=cfeb4_ly0hs[(MXHS-1)-ihs];
	assign cfeb4_ly1hsr[ihs]=cfeb4_ly1hs[(MXHS-1)-ihs];
	assign cfeb4_ly2hsr[ihs]=cfeb4_ly2hs[(MXHS-1)-ihs];
	assign cfeb4_ly3hsr[ihs]=cfeb4_ly3hs[(MXHS-1)-ihs];
	assign cfeb4_ly4hsr[ihs]=cfeb4_ly4hs[(MXHS-1)-ihs];
	assign cfeb4_ly5hsr[ihs]=cfeb4_ly5hs[(MXHS-1)-ihs];
	end
	endgenerate

// Reverse all CFEBs and reverse layers
	assign me1234_ly5hs = {cfeb0_ly0hsr, cfeb1_ly0hsr, cfeb2_ly0hsr, cfeb3_ly0hsr, cfeb4_ly0hsr};
	assign me1234_ly4hs = {cfeb0_ly1hsr, cfeb1_ly1hsr, cfeb2_ly1hsr, cfeb3_ly1hsr, cfeb4_ly1hsr};
	assign me1234_ly3hs = {cfeb0_ly2hsr, cfeb1_ly2hsr, cfeb2_ly2hsr, cfeb3_ly2hsr, cfeb4_ly2hsr};
	assign me1234_ly2hs = {cfeb0_ly3hsr, cfeb1_ly3hsr, cfeb2_ly3hsr, cfeb3_ly3hsr, cfeb4_ly3hsr};
	assign me1234_ly1hs = {cfeb0_ly4hsr, cfeb1_ly4hsr, cfeb2_ly4hsr, cfeb3_ly4hsr, cfeb4_ly4hsr};
	assign me1234_ly0hs = {cfeb0_ly5hsr, cfeb1_ly5hsr, cfeb2_ly5hsr, cfeb3_ly5hsr, cfeb4_ly5hsr};

//-------------------------------------------------------------------------------------------------------------------
// Stage 4A3: CSC_TYPE_C Normal ME1B reversed ME1A
//-------------------------------------------------------------------------------------------------------------------
`elsif CSC_TYPE_C
`define CSC_TYPE_A_or_CSC_TYPE_C 01'h1
`define CSC_TYPE_C_or_CSC_TYPE_D 01'h1

	wire [MXHS*1-1:0] me1a_ly0hs;
	wire [MXHS*1-1:0] me1a_ly1hs;
	wire [MXHS*1-1:0] me1a_ly2hs;
	wire [MXHS*1-1:0] me1a_ly3hs;
	wire [MXHS*1-1:0] me1a_ly4hs;
	wire [MXHS*1-1:0] me1a_ly5hs;

	wire [MXHS*4-1:0] me1b_ly0hs;
	wire [MXHS*4-1:0] me1b_ly1hs;
	wire [MXHS*4-1:0] me1b_ly2hs;
	wire [MXHS*4-1:0] me1b_ly3hs;
	wire [MXHS*4-1:0] me1b_ly4hs;
	wire [MXHS*4-1:0] me1b_ly5hs;
	
// Orientation flags
	assign csc_type        = 4'hC;		// Firmware compile type
	assign csc_me1ab	   = 1;			// 1= ME1A or ME1B CSC
	assign stagger_hs_csc  = 0;			// 1=Staggered CSC non-me1
	assign reverse_hs_csc  = 0;			// 1=Reversed  CSC non-me1
	assign reverse_hs_me1a = 1;			// 1=reverse me1a hstrips prior to pattern sorting
	assign reverse_hs_me1b = 0;			// 1=reverse me1b hstrips prior to pattern sorting
	initial $display ("CSC_TYPE_C instantiated");

// Generate hs reversal map for ME1A
	wire [MXHS-1:0] cfeb4_ly0hsr, cfeb4_ly1hsr, cfeb4_ly2hsr, cfeb4_ly3hsr, cfeb4_ly4hsr, cfeb4_ly5hsr;

	generate
	for (ihs=0; ihs<=MXHS-1; ihs=ihs+1) begin: hsrev
	assign cfeb4_ly0hsr[ihs]=cfeb4_ly0hs[(MXHS-1)-ihs];
	assign cfeb4_ly1hsr[ihs]=cfeb4_ly1hs[(MXHS-1)-ihs];
	assign cfeb4_ly2hsr[ihs]=cfeb4_ly2hs[(MXHS-1)-ihs];
	assign cfeb4_ly3hsr[ihs]=cfeb4_ly3hs[(MXHS-1)-ihs];
	assign cfeb4_ly4hsr[ihs]=cfeb4_ly4hs[(MXHS-1)-ihs];
	assign cfeb4_ly5hsr[ihs]=cfeb4_ly5hs[(MXHS-1)-ihs];
	end
	endgenerate

// Reversed ME1A cfebs
	assign me1a_ly0hs = cfeb4_ly0hsr;
	assign me1a_ly1hs = cfeb4_ly1hsr;
	assign me1a_ly2hs = cfeb4_ly2hsr;
	assign me1a_ly3hs = cfeb4_ly3hsr;
	assign me1a_ly4hs = cfeb4_ly4hsr;
	assign me1a_ly5hs = cfeb4_ly5hsr;
	
// Normal ME1B cfebs
	assign me1b_ly0hs = {cfeb3_ly0hs, cfeb2_ly0hs, cfeb1_ly0hs, cfeb0_ly0hs};
	assign me1b_ly1hs = {cfeb3_ly1hs, cfeb2_ly1hs, cfeb1_ly1hs, cfeb0_ly1hs};
	assign me1b_ly2hs = {cfeb3_ly2hs, cfeb2_ly2hs, cfeb1_ly2hs, cfeb0_ly2hs};
	assign me1b_ly3hs = {cfeb3_ly3hs, cfeb2_ly3hs, cfeb1_ly3hs, cfeb0_ly3hs};
	assign me1b_ly4hs = {cfeb3_ly4hs, cfeb2_ly4hs, cfeb1_ly4hs, cfeb0_ly4hs};
	assign me1b_ly5hs = {cfeb3_ly5hs, cfeb2_ly5hs, cfeb1_ly5hs, cfeb0_ly5hs};

//-------------------------------------------------------------------------------------------------------------------
// Stage 4A4: CSC_TYPE_D Normal ME1A reversed ME1B
//-------------------------------------------------------------------------------------------------------------------
`elsif CSC_TYPE_D
`define CSC_TYPE_C_or_CSC_TYPE_D 01'h1

	wire [MXHS*1-1:0] me1a_ly0hs;
	wire [MXHS*1-1:0] me1a_ly1hs;
	wire [MXHS*1-1:0] me1a_ly2hs;
	wire [MXHS*1-1:0] me1a_ly3hs;
	wire [MXHS*1-1:0] me1a_ly4hs;
	wire [MXHS*1-1:0] me1a_ly5hs;

	wire [MXHS*4-1:0] me1b_ly0hs;
	wire [MXHS*4-1:0] me1b_ly1hs;
	wire [MXHS*4-1:0] me1b_ly2hs;
	wire [MXHS*4-1:0] me1b_ly3hs;
	wire [MXHS*4-1:0] me1b_ly4hs;
	wire [MXHS*4-1:0] me1b_ly5hs;
	
// Orientation flags
	assign csc_type        = 4'hD;		// Firmware compile type
	assign csc_me1ab	   = 1;			// 1= ME1A or ME1B CSC
	assign stagger_hs_csc  = 0;			// 1=Staggered CSC non-me1
	assign reverse_hs_csc  = 0;			// 1=Reversed  CSC non-me1
	assign reverse_hs_me1a = 0;			// 1=reverse me1a hstrips prior to pattern sorting
	assign reverse_hs_me1b = 1;			// 1=reverse me1b hstrips prior to pattern sorting
	initial $display ("CSC_TYPE_D instantiated");

// Generate hs reversal map for ME1B
	wire [MXHS-1:0] cfeb0_ly0hsr, cfeb0_ly1hsr, cfeb0_ly2hsr, cfeb0_ly3hsr, cfeb0_ly4hsr, cfeb0_ly5hsr;
	wire [MXHS-1:0] cfeb1_ly0hsr, cfeb1_ly1hsr, cfeb1_ly2hsr, cfeb1_ly3hsr, cfeb1_ly4hsr, cfeb1_ly5hsr;
	wire [MXHS-1:0] cfeb2_ly0hsr, cfeb2_ly1hsr, cfeb2_ly2hsr, cfeb2_ly3hsr, cfeb2_ly4hsr, cfeb2_ly5hsr;
	wire [MXHS-1:0] cfeb3_ly0hsr, cfeb3_ly1hsr, cfeb3_ly2hsr, cfeb3_ly3hsr, cfeb3_ly4hsr, cfeb3_ly5hsr;

	generate
	for (ihs=0; ihs<=MXHS-1; ihs=ihs+1) begin: hsrev
	assign cfeb0_ly0hsr[ihs]=cfeb0_ly0hs[(MXHS-1)-ihs];
	assign cfeb0_ly1hsr[ihs]=cfeb0_ly1hs[(MXHS-1)-ihs];
	assign cfeb0_ly2hsr[ihs]=cfeb0_ly2hs[(MXHS-1)-ihs];
	assign cfeb0_ly3hsr[ihs]=cfeb0_ly3hs[(MXHS-1)-ihs];
	assign cfeb0_ly4hsr[ihs]=cfeb0_ly4hs[(MXHS-1)-ihs];
	assign cfeb0_ly5hsr[ihs]=cfeb0_ly5hs[(MXHS-1)-ihs];

	assign cfeb1_ly0hsr[ihs]=cfeb1_ly0hs[(MXHS-1)-ihs];
	assign cfeb1_ly1hsr[ihs]=cfeb1_ly1hs[(MXHS-1)-ihs];
	assign cfeb1_ly2hsr[ihs]=cfeb1_ly2hs[(MXHS-1)-ihs];
	assign cfeb1_ly3hsr[ihs]=cfeb1_ly3hs[(MXHS-1)-ihs];
	assign cfeb1_ly4hsr[ihs]=cfeb1_ly4hs[(MXHS-1)-ihs];
	assign cfeb1_ly5hsr[ihs]=cfeb1_ly5hs[(MXHS-1)-ihs];

	assign cfeb2_ly0hsr[ihs]=cfeb2_ly0hs[(MXHS-1)-ihs];
	assign cfeb2_ly1hsr[ihs]=cfeb2_ly1hs[(MXHS-1)-ihs];
	assign cfeb2_ly2hsr[ihs]=cfeb2_ly2hs[(MXHS-1)-ihs];
	assign cfeb2_ly3hsr[ihs]=cfeb2_ly3hs[(MXHS-1)-ihs];
	assign cfeb2_ly4hsr[ihs]=cfeb2_ly4hs[(MXHS-1)-ihs];
	assign cfeb2_ly5hsr[ihs]=cfeb2_ly5hs[(MXHS-1)-ihs];

	assign cfeb3_ly0hsr[ihs]=cfeb3_ly0hs[(MXHS-1)-ihs];
	assign cfeb3_ly1hsr[ihs]=cfeb3_ly1hs[(MXHS-1)-ihs];
	assign cfeb3_ly2hsr[ihs]=cfeb3_ly2hs[(MXHS-1)-ihs];
	assign cfeb3_ly3hsr[ihs]=cfeb3_ly3hs[(MXHS-1)-ihs];
	assign cfeb3_ly4hsr[ihs]=cfeb3_ly4hs[(MXHS-1)-ihs];
	assign cfeb3_ly5hsr[ihs]=cfeb3_ly5hs[(MXHS-1)-ihs];
	end
	endgenerate

// Normal ME1A cfebs
	assign me1a_ly0hs = cfeb4_ly0hs;
	assign me1a_ly1hs = cfeb4_ly1hs;
	assign me1a_ly2hs = cfeb4_ly2hs;
	assign me1a_ly3hs = cfeb4_ly3hs;
	assign me1a_ly4hs = cfeb4_ly4hs;
	assign me1a_ly5hs = cfeb4_ly5hs;
	
// Reversed ME1B cfebs
	assign me1b_ly0hs = {cfeb0_ly0hsr, cfeb1_ly0hsr, cfeb2_ly0hsr, cfeb3_ly0hsr};
	assign me1b_ly1hs = {cfeb0_ly1hsr, cfeb1_ly1hsr, cfeb2_ly1hsr, cfeb3_ly1hsr};
	assign me1b_ly2hs = {cfeb0_ly2hsr, cfeb1_ly2hsr, cfeb2_ly2hsr, cfeb3_ly2hsr};
	assign me1b_ly3hs = {cfeb0_ly3hsr, cfeb1_ly3hsr, cfeb2_ly3hsr, cfeb3_ly3hsr};
	assign me1b_ly4hs = {cfeb0_ly4hsr, cfeb1_ly4hsr, cfeb2_ly4hsr, cfeb3_ly4hsr};
	assign me1b_ly5hs = {cfeb0_ly5hsr, cfeb1_ly5hsr, cfeb2_ly5hsr, cfeb3_ly5hsr};

//-------------------------------------------------------------------------------------------------------------------
// Stage 4A5: CSC_TYPE_X Undefined
//-------------------------------------------------------------------------------------------------------------------
`else
	initial $display ("CSC_TYPE Undefined. Halting.");
	$finish
`endif

//-------------------------------------------------------------------------------------------------------------------
// Stage 4B: Correct for CSC layer stagger: 565656 is a straight track, becomes 555555 on key layer 2
//
//	ly0hs:   -2 -1 | 00 01 02 03 04 05 06 ... 152 153 154 155 156 157 158 159 | 160 no shift
//	ly1hs:   -1 00 | 01 02 03 04 05 06 07 ... 153 154 155 156 157 158 159 160 | 161 
//	ly2hs:   -2 -1 | 00 01 02 03 04 05 06 ... 152 153 154 155 156 157 158 159 | 160 no shift, key layer
//	ly3hs:   -1 00 | 01 02 03 04 05 06 07 ... 153 154 155 156 157 158 159 160 | 161 
//	ly4hs:   -2 -1 | 00 01 02 03 04 05 06 ... 152 153 154 155 156 157 158 159 | 160 no shift
//	ly5hs:   -1 00 | 01 02 03 04 05 06 07 ... 153 154 155 156 157 158 159 160 | 161 
//
//-------------------------------------------------------------------------------------------------------------------
// Staggered layers
//-------------------------------------------------------------------------------------------------------------------
	parameter j=1;								// Shift negative array indexes positive

`ifdef STAGGER_HS_CSC
	wire [MXHSX-1+j:-0+j] ly0hs;
	wire [MXHSX-1+j:-1+j] ly1hs;
	wire [MXHSX-1+j:-0+j] ly2hs;				// key layer 2
	wire [MXHSX-1+j:-1+j] ly3hs;
	wire [MXHSX-1+j:-0+j] ly4hs;
	wire [MXHSX-1+j:-1+j] ly5hs;

	assign ly0hs = {      me1234_ly0hs};		// Stagger correction
	assign ly1hs = {1'b0, me1234_ly1hs};
	assign ly2hs = {      me1234_ly2hs};
	assign ly3hs = {1'b0, me1234_ly3hs};
	assign ly4hs = {      me1234_ly4hs};
	assign ly5hs = {1'b0, me1234_ly5hs};

//-------------------------------------------------------------------------------------------------------------------
// Non-staggered layers
//-------------------------------------------------------------------------------------------------------------------
`else
	wire [MXHSX-1:0] ly0hs;
	wire [MXHSX-1:0] ly1hs;
	wire [MXHSX-1:0] ly2hs;						// key layer 2
	wire [MXHSX-1:0] ly3hs;
	wire [MXHSX-1:0] ly4hs;
	wire [MXHSX-1:0] ly5hs;

	assign ly0hs = {me1a_ly0hs, me1b_ly0hs};	// No stagger correction
	assign ly1hs = {me1a_ly1hs, me1b_ly1hs};
	assign ly2hs = {me1a_ly2hs, me1b_ly2hs};
	assign ly3hs = {me1a_ly3hs, me1b_ly3hs};
	assign ly4hs = {me1a_ly4hs, me1b_ly4hs};
	assign ly5hs = {me1a_ly5hs, me1b_ly5hs};
`endif

//-------------------------------------------------------------------------------------------------------------------
// Stage 4C:  Layer-trigger mode
//-------------------------------------------------------------------------------------------------------------------
// Layer Trigger Mode, delay 1bx for FF
	reg [MXLY-1:0] layer_or_s0;

	always @(posedge clock) begin
	layer_or_s0[0] = |{cfeb4_ly0hs, cfeb3_ly0hs, cfeb2_ly0hs, cfeb1_ly0hs, cfeb0_ly0hs};
	layer_or_s0[1] = |{cfeb4_ly1hs, cfeb3_ly1hs, cfeb2_ly1hs, cfeb1_ly1hs, cfeb0_ly1hs};
	layer_or_s0[2] = |{cfeb4_ly2hs, cfeb3_ly2hs, cfeb2_ly2hs, cfeb1_ly2hs, cfeb0_ly2hs};
	layer_or_s0[3] = |{cfeb4_ly3hs, cfeb3_ly3hs, cfeb2_ly3hs, cfeb1_ly3hs, cfeb0_ly3hs};
	layer_or_s0[4] = |{cfeb4_ly4hs, cfeb3_ly4hs, cfeb2_ly4hs, cfeb1_ly4hs, cfeb0_ly4hs};
	layer_or_s0[5] = |{cfeb4_ly5hs, cfeb3_ly5hs, cfeb2_ly5hs, cfeb1_ly5hs, cfeb0_ly5hs};
	end

// Sum number of layers hit into a binary pattern number
	wire [MXHITB-1:0] nlayers_hit_s0;
	wire layer_trig_s0;
	
	assign nlayers_hit_s0 = count1s(layer_or_s0[5:0]);
	assign layer_trig_s0  = (nlayers_hit_s0 >= lyr_thresh_pretrig_ff);

	function [2:0]	count1s;
	input	 [5:0]	inp;
	count1s = (inp[5] + inp[4]) + (inp[3] + inp[2]) + (inp[1] + inp[0]);
	endfunction

// Delay 1bx more to coincide with pretrigger
	parameter dlya = 4'd0;
	srl16e_bbl #(1)      udlya0 (.clock(clock),.ce(1'b1),.adr(dlya),.d(layer_trig_s0 ),.q(cfeb_layer_trig ));
	srl16e_bbl #(MXHITB) udlya1 (.clock(clock),.ce(1'b1),.adr(dlya),.d(nlayers_hit_s0),.q(cfeb_nlayers_hit));
	srl16e_bbl #(MXLY)   udlya2 (.clock(clock),.ce(1'b1),.adr(dlya),.d(layer_or_s0   ),.q(cfeb_layer_or   ));

// Delay 4bx to latch in time with 1st and 2nd clct, need to FF these again to align
	wire [MXLY-1:0]		hs_layer_or_dly;
	wire [MXHITB-1:0]	hs_nlayers_hit_dly;

	parameter dlyb = 4'd3;
	srl16e_bbl #(1)      udlyb0 (.clock(clock),.ce(1'b1),.adr(dlyb),.d(layer_trig_s0 ),.q(hs_layer_latch    ));
	srl16e_bbl #(MXHITB) udlyb1 (.clock(clock),.ce(1'b1),.adr(dlyb),.d(nlayers_hit_s0),.q(hs_nlayers_hit_dly));
	srl16e_bbl #(1)      udlyb2 (.clock(clock),.ce(1'b1),.adr(dlyb),.d(layer_trig_s0 ),.q(hs_layer_trig_dly ));
	srl16e_bbl #(MXLY)   udlyb3 (.clock(clock),.ce(1'b1),.adr(dlyb),.d(layer_or_s0   ),.q(hs_layer_or_dly   ));

//-------------------------------------------------------------------------------------------------------------------
// Stage 4D: 1/2-Strip Pattern Finder
//			 Finds number of hits in pattern templates for each key 1/2-strip.
//
//			hs	0123456789A
//	ly0[10:0]	xxxxxkxxxxx    5+1+5 =11
//	ly1[ 7:3]	   xxkxx       2+1+2 = 5
//	ly2[ 5:5]	     k         0+1+0 = 1
//	ly3[ 7:3]	   xxkxx       2+1+2 = 5
//	ly4[ 9:1]	 xxxxkxxxx     4+1+4 = 9
//	ly5[10:0]	xxxxxkxxxxx    5+1+5 =11
//                                                                   11111111 11111
//              nnnnn            77777777 88888     77777 88888888   55555555 66666
//          hs  54321 01234567   23456789 01234     56789 01234567   23456789 01234
//	ly0[10:0]	00000|aaaaaaaa...aaaaaaaa|bbbbb     aaaaa|bbbbbbbb...bbbbbbbb|00000
//	ly1[ 7:3]	   0s|aaaaaaaa...aaaaaaaa|bb           aa|bbbbbbbb...bbbbbbb0|00
//	ly2[ 5:5]	     |aaaaaaaa...aaaaaaaa|               |bbbbbbbb...bbbbbbbb|
//	ly3[ 7:3]	   0s|aaaaaaaa...aaaaaaaa|bb           aa|bbbbbbbb...bbbbbbb0|00
//	ly4[ 9:1]	 0000|aaaaaaaa...aaaaaaaa|bbbb       aaaa|bbbbbbbb...bbbbbbbb|0000
//	ly5[10:0]	0000s|aaaaaaaa...aaaaaaaa|bbbbb     aaaaa|bbbbbbbb...bbbbbbb0|00000
//
//-------------------------------------------------------------------------------------------------------------------
// Replicate logic accessible clock, serial logic takes 6 cycles to propagate, parallel logic is 10mhz slower
	reg	[MXLY-1:0] sel;	//xsynthesis attribute equivalent_register_removal of sel is "no";

	always @(posedge clock_2x)begin
	sel[MXLY-1:0] <= {~sel[MXLY-2:0],~clock_lac};
	end

//-------------------------------------------------------------------------------------------------------------------
// Staggered layers
//-------------------------------------------------------------------------------------------------------------------
`ifdef STAGGER_HS_CSC

// Create hs arrays with 0s padded at left and right csc edges
	parameter k=5;		// Shift negative array indexes positive

	wire [MXHSX/2-1+5+k:-5+k] ly0hs_pad_a, ly0hs_pad_b, ly0hs_pad;
	wire [MXHSX/2-1+2+k:-2+k] ly1hs_pad_a, ly1hs_pad_b, ly1hs_pad;
	wire [MXHSX/2-1+0+k: 0+k] ly2hs_pad_a, ly2hs_pad_b, ly2hs_pad;
	wire [MXHSX/2-1+2+k:-2+k] ly3hs_pad_a, ly3hs_pad_b, ly3hs_pad;
	wire [MXHSX/2-1+4+k:-4+k] ly4hs_pad_a, ly4hs_pad_b, ly4hs_pad;
	wire [MXHSX/2-1+5+k:-5+k] ly5hs_pad_a, ly5hs_pad_b, ly5hs_pad;

// Pad 0s beyond csc edges: Left 1/2 of CSC
	assign ly0hs_pad_a = {ly0hs[84+j:80+j], ly0hs[79+j:j],              5'b00000};
	assign ly1hs_pad_a = {ly1hs[81+j:80+j], ly1hs[79+j:j], ly1hs[-1+j], 1'b0};
	assign ly2hs_pad_a = {                  ly2hs[79+j:j]};
	assign ly3hs_pad_a = {ly3hs[81+j:80+j], ly3hs[79+j:j], ly3hs[-1+j], 1'b0};
	assign ly4hs_pad_a = {ly4hs[83+j:80+j], ly4hs[79+j:j],              4'b0000};
	assign ly5hs_pad_a = {ly5hs[84+j:80+j], ly5hs[79+j:j], ly5hs[-1+j], 4'b0000};

// Pad 0s beyond csc edges: Right 1/2 of CSC
	assign ly0hs_pad_b = {5'b00000, ly0hs[159+j:80+j], ly0hs[79+j:75+j]};
	assign ly1hs_pad_b = {2'b00,    ly1hs[159+j:80+j], ly1hs[79+j:78+j]};
	assign ly2hs_pad_b = {          ly2hs[159+j:80+j]};
	assign ly3hs_pad_b = {2'b00,    ly3hs[159+j:80+j], ly3hs[79+j:78+j]};
	assign ly4hs_pad_b = {4'b0000,  ly4hs[159+j:80+j], ly4hs[79+j:76+j]};
	assign ly5hs_pad_b = {5'b00000, ly5hs[159+j:80+j], ly5hs[79+j:75+j]};

// Select Left then Right 1/2 of CSC at 80MHz
	assign ly0hs_pad = (sel[0]) ? ly0hs_pad_a : ly0hs_pad_b;
	assign ly1hs_pad = (sel[1]) ? ly1hs_pad_a : ly1hs_pad_b;
	assign ly2hs_pad = (sel[2]) ? ly2hs_pad_a : ly2hs_pad_b;
	assign ly3hs_pad = (sel[3]) ? ly3hs_pad_a : ly3hs_pad_b;
	assign ly4hs_pad = (sel[4]) ? ly4hs_pad_a : ly4hs_pad_b;
	assign ly5hs_pad = (sel[5]) ? ly5hs_pad_a : ly5hs_pad_b;

// Find pattern hits for each 1/2-strip key
	wire [MXHITB-1:0] hs_hit [MXHSX/2-1:0];
	wire [MXPIDB-1:0] hs_pid [MXHSX/2-1:0];

	generate
	for (ihs=0; ihs<=MXHSX/2-1; ihs=ihs+1) begin: patgen
	pattern_unit upat (
	.clock_2x	(clock_2x),
	.ly0		(ly0hs_pad[ihs+5+k:ihs-5+k]),
	.ly1		(ly1hs_pad[ihs+2+k:ihs-2+k]),
	.ly2		(ly2hs_pad[ihs+0+k:ihs-0+k]),	//key on ly2
	.ly3		(ly3hs_pad[ihs+2+k:ihs-2+k]),
	.ly4		(ly4hs_pad[ihs+4+k:ihs-4+k]),
	.ly5		(ly5hs_pad[ihs+5+k:ihs-5+k]),
	.pat_nhits	(hs_hit[ihs]),
	.pat_id		(hs_pid[ihs]));
	end
	endgenerate

// Store 1/2-cycle pattern unit results
	reg	[MXHITB-1:0]	hs_hit_s0a	[MXHSX/2-1:0];
	reg	[MXPIDB-1:0]	hs_pid_s0a	[MXHSX/2-1:0];
	reg	[MXHITB-1:0]	hs_hit_s0b	[MXHSX/2-1:0];
	reg	[MXPIDB-1:0]	hs_pid_s0b	[MXHSX/2-1:0];

	generate
	for (ihs=0; ihs<=MXHSX/2-1; ihs=ihs+1) begin: store_ab
	always @(posedge clock) begin
	hs_hit_s0a[ihs] <= hs_hit[ihs];		// store result a on rising edge
	hs_pid_s0a[ihs] <= hs_pid[ihs];
	end
	always @(negedge clock) begin
	hs_hit_s0b[ihs]	<= hs_hit[ihs];		// store result b on falling edge	
	hs_pid_s0b[ihs]	<= hs_pid[ihs];
	end
	end
	endgenerate

// s0 latch: realign with main clock
	reg	[MXHITB-1:0]	hs_hit_s0	[MXHSX-1:0];
	reg	[MXPIDB-1:0]	hs_pid_s0	[MXHSX-1:0];

	generate
	for (ihs=0; ihs<=MXHSX/2-1; ihs=ihs+1) begin: store_s0
	always @(posedge clock) begin
	hs_hit_s0[ihs]		<= hs_hit_s0a[ihs];
	hs_pid_s0[ihs]		<= hs_pid_s0a[ihs];
	hs_hit_s0[ihs+80]	<= hs_hit_s0b[ihs];
	hs_pid_s0[ihs+80]	<= hs_pid_s0b[ihs];
	end
	end
	endgenerate

// pre-s0 latch signals for pre-trigger speed
	wire [MXHITB-1:0] hs_hit_pre_s0 [MXHSX-1:0];
	wire [MXPIDB-1:0] hs_pid_pre_s0 [MXHSX-1:0];

	generate
	for (ihs=0; ihs<=MXHSX/2-1; ihs=ihs+1) begin: build_pad_a_b
	assign hs_hit_pre_s0[ihs]		= hs_hit_s0a[ihs];
	assign hs_pid_pre_s0[ihs]		= hs_pid_s0a[ihs];
	assign hs_hit_pre_s0[ihs+80]	= hs_hit_s0b[ihs];
	assign hs_pid_pre_s0[ihs+80]	= hs_pid_s0b[ihs];
	end
	endgenerate

//-------------------------------------------------------------------------------------------------------------------
// Non-Staggered layers
//-------------------------------------------------------------------------------------------------------------------
`else

// Create hs arrays with 0s padded at left and right csc edges
	parameter k		 = 5;		// Shift negative array indexes positive
	parameter MXHSXA = 32;		// Number of hs on ME1A
	parameter MXHSXB = 128;		// Number of hs on ME1B

	wire [MXHSXA/2-1+5+k:0-5+k] ly0hs_pad_me1a, ly0hs_pad_me1a_a, ly0hs_pad_me1a_b;
	wire [MXHSXA/2-1+2+k:0-2+k] ly1hs_pad_me1a, ly1hs_pad_me1a_a, ly1hs_pad_me1a_b;
	wire [MXHSXA/2-1+0+k:0-0+k] ly2hs_pad_me1a, ly2hs_pad_me1a_a, ly2hs_pad_me1a_b;
	wire [MXHSXA/2-1+2+k:0-2+k] ly3hs_pad_me1a, ly3hs_pad_me1a_a, ly3hs_pad_me1a_b;
	wire [MXHSXA/2-1+4+k:0-4+k] ly4hs_pad_me1a, ly4hs_pad_me1a_a, ly4hs_pad_me1a_b;
	wire [MXHSXA/2-1+5+k:0-5+k] ly5hs_pad_me1a, ly5hs_pad_me1a_a, ly5hs_pad_me1a_b;

	wire [MXHSXB/2-1+5+k:0-5+k] ly0hs_pad_me1b, ly0hs_pad_me1b_a, ly0hs_pad_me1b_b;
	wire [MXHSXB/2-1+2+k:0-2+k] ly1hs_pad_me1b, ly1hs_pad_me1b_a, ly1hs_pad_me1b_b;
	wire [MXHSXB/2-1+0+k:0-0+k] ly2hs_pad_me1b, ly2hs_pad_me1b_a, ly2hs_pad_me1b_b;
	wire [MXHSXB/2-1+2+k:0-2+k] ly3hs_pad_me1b, ly3hs_pad_me1b_a, ly3hs_pad_me1b_b;
	wire [MXHSXB/2-1+4+k:0-4+k] ly4hs_pad_me1b, ly4hs_pad_me1b_a, ly4hs_pad_me1b_b;
	wire [MXHSXB/2-1+5+k:0-5+k] ly5hs_pad_me1b, ly5hs_pad_me1b_a, ly5hs_pad_me1b_b;

// Pad 0s beyond csc edges  ME1A hs128-159, isolate it from ME1B
	assign ly0hs_pad_me1a_a = { ly0hs[148:144], ly0hs[143:128], 5'b00000       };
	assign ly1hs_pad_me1a_a = { ly0hs[145:144], ly1hs[143:128], 2'b00          };
	assign ly2hs_pad_me1a_a = {                 ly2hs[143:128]                 };
	assign ly3hs_pad_me1a_a = { ly0hs[145:144], ly3hs[143:128], 2'b00          };
	assign ly4hs_pad_me1a_a = { ly0hs[147:144], ly4hs[143:128], 4'b0000        };
	assign ly5hs_pad_me1a_a = { ly0hs[148:144], ly5hs[143:128], 5'b00000       };

	assign ly0hs_pad_me1a_b = { 5'b00000,       ly0hs[159:144], ly0hs[143:139] };
	assign ly1hs_pad_me1a_b = {    2'b00,       ly1hs[159:144], ly0hs[143:142] };
	assign ly2hs_pad_me1a_b = {                 ly2hs[159:144]                 };
	assign ly3hs_pad_me1a_b = {    2'b00,       ly3hs[159:144], ly0hs[143:142] };
	assign ly4hs_pad_me1a_b = {  4'b0000,       ly4hs[159:144], ly0hs[143:140] };
	assign ly5hs_pad_me1a_b = { 5'b00000,       ly5hs[159:144], ly0hs[143:139] };

// Pad 0s beyond csc edges  ME1B hs0-127, isolate it from ME1A
	assign ly0hs_pad_me1b_a = { ly0hs[68:64],   ly0hs[63:0],    5'b00000       };
	assign ly1hs_pad_me1b_a = { ly0hs[65:64],   ly1hs[63:0],    2'b00          };
	assign ly2hs_pad_me1b_a = {                 ly2hs[63:0]                    };
	assign ly3hs_pad_me1b_a = { ly0hs[65:64],   ly3hs[63:0],    2'b00          };
	assign ly4hs_pad_me1b_a = { ly0hs[67:64],   ly4hs[63:0],    4'b0000        };
	assign ly5hs_pad_me1b_a = { ly0hs[68:64],   ly5hs[63:0],    5'b00000       };

	assign ly0hs_pad_me1b_b = { 5'b00000,       ly0hs[127:64],  ly0hs[63:59]   };
	assign ly1hs_pad_me1b_b = {    2'b00,       ly1hs[127:64],  ly0hs[63:62]   };
	assign ly2hs_pad_me1b_b = {                 ly2hs[127:64]                  };
	assign ly3hs_pad_me1b_b = {    2'b00,       ly3hs[127:64],  ly0hs[63:62]   };
	assign ly4hs_pad_me1b_b = {  4'b0000,       ly4hs[127:64],  ly0hs[63:60]   };
	assign ly5hs_pad_me1b_b = { 5'b00000,       ly5hs[127:64],  ly0hs[63:59]   };
	
// Select Left then Right 1/2 of CSC at 80MHz
	assign ly0hs_pad_me1a = (sel[0]) ? ly0hs_pad_me1a_a : ly0hs_pad_me1a_b;
	assign ly1hs_pad_me1a = (sel[1]) ? ly1hs_pad_me1a_a : ly1hs_pad_me1a_b;
	assign ly2hs_pad_me1a = (sel[2]) ? ly2hs_pad_me1a_a : ly2hs_pad_me1a_b;
	assign ly3hs_pad_me1a = (sel[3]) ? ly3hs_pad_me1a_a : ly3hs_pad_me1a_b;
	assign ly4hs_pad_me1a = (sel[4]) ? ly4hs_pad_me1a_a : ly4hs_pad_me1a_b;
	assign ly5hs_pad_me1a = (sel[5]) ? ly5hs_pad_me1a_a : ly5hs_pad_me1a_b;

	assign ly0hs_pad_me1b = (sel[0]) ? ly0hs_pad_me1b_a : ly0hs_pad_me1b_b;
	assign ly1hs_pad_me1b = (sel[1]) ? ly1hs_pad_me1b_a : ly1hs_pad_me1b_b;
	assign ly2hs_pad_me1b = (sel[2]) ? ly2hs_pad_me1b_a : ly2hs_pad_me1b_b;
	assign ly3hs_pad_me1b = (sel[3]) ? ly3hs_pad_me1b_a : ly3hs_pad_me1b_b;
	assign ly4hs_pad_me1b = (sel[4]) ? ly4hs_pad_me1b_a : ly4hs_pad_me1b_b;
	assign ly5hs_pad_me1b = (sel[5]) ? ly5hs_pad_me1b_a : ly5hs_pad_me1b_b;

// Find pattern hits for each 1/2-strip key
	wire [MXHITB-1:0] hs_hit_me1a [MXHSXA/2-1:0];
	wire [MXPIDB-1:0] hs_pid_me1a [MXHSXA/2-1:0];

	wire [MXHITB-1:0] hs_hit_me1b [MXHSXB/2-1:0];
	wire [MXPIDB-1:0] hs_pid_me1b [MXHSXB/2-1:0];

	generate
	for (ihs=0; ihs<=15; ihs=ihs+1) begin: patgen_me1a
	pattern_unit upat_me1a (
	.clock_2x	(clock_2x),
	.ly0		(ly0hs_pad_me1a[ihs+5+k:ihs-5+k]),
	.ly1		(ly1hs_pad_me1a[ihs+2+k:ihs-2+k]),
	.ly2		(ly2hs_pad_me1a[ihs+0+k:ihs-0+k]),	//key on ly2
	.ly3		(ly3hs_pad_me1a[ihs+2+k:ihs-2+k]),
	.ly4		(ly4hs_pad_me1a[ihs+4+k:ihs-4+k]),
	.ly5		(ly5hs_pad_me1a[ihs+5+k:ihs-5+k]),
	.pat_nhits	(hs_hit_me1a[ihs]),
	.pat_id		(hs_pid_me1a[ihs]));
	end
	endgenerate

	generate
	for (ihs=0; ihs<=63; ihs=ihs+1) begin: patgen_me1b
	pattern_unit upat_me1b (
	.clock_2x	(clock_2x),
	.ly0		(ly0hs_pad_me1b[ihs+5+k:ihs-5+k]),
	.ly1		(ly1hs_pad_me1b[ihs+2+k:ihs-2+k]),
	.ly2		(ly2hs_pad_me1b[ihs+0+k:ihs-0+k]),	//key on ly2
	.ly3		(ly3hs_pad_me1b[ihs+2+k:ihs-2+k]),
	.ly4		(ly4hs_pad_me1b[ihs+4+k:ihs-4+k]),
	.ly5		(ly5hs_pad_me1b[ihs+5+k:ihs-5+k]),
	.pat_nhits	(hs_hit_me1b[ihs]),
	.pat_id		(hs_pid_me1b[ihs]));
	end
	endgenerate

// Store 1/2-cycle pattern unit results
	reg	[MXHITB-1:0] hs_hit_me1a_s0a	[MXHSXA/2-1:0];
	reg	[MXPIDB-1:0] hs_pid_me1a_s0a	[MXHSXA/2-1:0];
	reg	[MXHITB-1:0] hs_hit_me1a_s0b	[MXHSXA/2-1:0];
	reg	[MXPIDB-1:0] hs_pid_me1a_s0b	[MXHSXA/2-1:0];

	reg	[MXHITB-1:0] hs_hit_me1b_s0a	[MXHSXB/2-1:0];
	reg	[MXPIDB-1:0] hs_pid_me1b_s0a	[MXHSXB/2-1:0];
	reg	[MXHITB-1:0] hs_hit_me1b_s0b	[MXHSXB/2-1:0];
	reg	[MXPIDB-1:0] hs_pid_me1b_s0b	[MXHSXB/2-1:0];

	generate
	for (ihs=0; ihs<=15; ihs=ihs+1) begin: store_me1a_ab
	always @(posedge clock) begin
	hs_hit_me1a_s0a[ihs] <= hs_hit_me1a[ihs];	// store result a on rising edge
	hs_pid_me1a_s0a[ihs] <= hs_pid_me1a[ihs];
	end
	always @(negedge clock) begin
	hs_hit_me1a_s0b[ihs] <= hs_hit_me1a[ihs];	// store result b on falling edge	
	hs_pid_me1a_s0b[ihs] <= hs_pid_me1a[ihs];
	end
	end
	endgenerate

	generate
	for (ihs=0; ihs<=63; ihs=ihs+1) begin: store_me1b_ab
	always @(posedge clock) begin
	hs_hit_me1b_s0a[ihs] <= hs_hit_me1b[ihs];	// store result a on rising edge
	hs_pid_me1b_s0a[ihs] <= hs_pid_me1b[ihs];
	end
	always @(negedge clock) begin
	hs_hit_me1b_s0b[ihs] <= hs_hit_me1b[ihs];	// store result b on falling edge
	hs_pid_me1b_s0b[ihs] <= hs_pid_me1b[ihs];
	end
	end
	endgenerate

// S0 latch: realign with main clock
	reg	[MXHITB-1:0]	hs_hit_s0	[MXHSX-1:0];
	reg	[MXPIDB-1:0]	hs_pid_s0	[MXHSX-1:0];

	generate
	for (ihs=0; ihs<=15; ihs=ihs+1) begin: store_me1a_s0
	always @(posedge clock) begin
	hs_hit_s0[ihs+128]		<= hs_hit_me1a_s0a[ihs];	// me1a hs 128-159
	hs_pid_s0[ihs+128]		<= hs_pid_me1a_s0a[ihs];
	hs_hit_s0[ihs+128+16]	<= hs_hit_me1a_s0b[ihs];
	hs_pid_s0[ihs+128+16]	<= hs_pid_me1a_s0b[ihs];
	end
	end
	endgenerate

	generate
	for (ihs=0; ihs<=63; ihs=ihs+1) begin: store_me1b_s0
	always @(posedge clock) begin
	hs_hit_s0[ihs]			<= hs_hit_me1b_s0a[ihs];	// me1b hs 0-127
	hs_pid_s0[ihs]			<= hs_pid_me1b_s0a[ihs];
	hs_hit_s0[ihs+64]		<= hs_hit_me1b_s0b[ihs];
	hs_pid_s0[ihs+64]		<= hs_pid_me1b_s0b[ihs];
	end
	end
	endgenerate

// pre-s0 latch signals for pre-trigger speed
	wire [MXHITB-1:0] hs_hit_pre_s0 [MXHSX-1:0];
	wire [MXPIDB-1:0] hs_pid_pre_s0 [MXHSX-1:0];

	generate
	for (ihs=0; ihs<=15; ihs=ihs+1) begin: build_pad_me1a_ab
	assign hs_hit_pre_s0[ihs+128]		= hs_hit_me1a_s0a[ihs];		// me1a hs 128-159
	assign hs_pid_pre_s0[ihs+128]		= hs_pid_me1a_s0a[ihs];
	assign hs_hit_pre_s0[ihs+128+16]	= hs_hit_me1a_s0b[ihs];
	assign hs_pid_pre_s0[ihs+128+16]	= hs_pid_me1a_s0b[ihs];
	end
	endgenerate

	generate
	for (ihs=0; ihs<=63; ihs=ihs+1) begin: build_pad_me1b_ab		// me1b hs 0-127
	assign hs_hit_pre_s0[ihs]			= hs_hit_me1b_s0a[ihs];
	assign hs_pid_pre_s0[ihs]			= hs_pid_me1b_s0a[ihs];
	assign hs_hit_pre_s0[ihs+64]		= hs_hit_me1b_s0b[ihs];
	assign hs_pid_pre_s0[ihs+64]		= hs_pid_me1b_s0b[ihs];
	end
	endgenerate
`endif

//-------------------------------------------------------------------------------------------------------------------
// Convert S0 pattern IDs and hits into sortable pattern numbers, [6:4]=nhits, [3:0]=pattern id
//-------------------------------------------------------------------------------------------------------------------
	wire [MXPATB-1:0] hs_pat_s0 [MXHSX-1:0];

	generate
	for (ihs=0; ihs<=MXHSX-1; ihs=ihs+1) begin: patcat
	assign hs_pat_s0[ihs] = {hs_hit_s0[ihs],hs_pid_s0[ihs]};
	end
	endgenerate

//-------------------------------------------------------------------------------------------------------------------
// Stage 5A: Pre-Trigger Look-ahead
// 			 Set active FEB bit ASAP if any pattern is over threshold. 
//			 It comes out before the priority encoder result
//-------------------------------------------------------------------------------------------------------------------
// Flag keys with pattern hits over threshold, use fast-out hit numbers before s0 latch
	reg	[MXHS-1:0] hs_key_hit0, hs_key_pid0, hs_key_dmb0;
	reg	[MXHS-1:0] hs_key_hit1, hs_key_pid1, hs_key_dmb1;
	reg	[MXHS-1:0] hs_key_hit2, hs_key_pid2, hs_key_dmb2;
	reg	[MXHS-1:0] hs_key_hit3, hs_key_pid3, hs_key_dmb3;
	reg	[MXHS-1:0] hs_key_hit4, hs_key_pid4, hs_key_dmb4;

// Flag keys with pattern hits over threshold, use fast-out hit numbers before s0 latch
	`ifdef CSC_TYPE_A_or_CSC_TYPE_C initial $display ("CSC_TYPE_A_or_CSC_TYPE_C is defined"); `endif
	generate
	for (ihs=0; ihs<=MXHS-1; ihs=ihs+1) begin: thrg
	always @(posedge clock) begin: thrff
`ifdef CSC_TYPE_A_or_CSC_TYPE_C	// Unreversed CSC or unreversed ME1B
	hs_key_hit0[ihs] = (hs_hit_pre_s0[ihs+MXHS*0] >= hit_thresh_pretrig_ff);	// Normal CSC
	hs_key_hit1[ihs] = (hs_hit_pre_s0[ihs+MXHS*1] >= hit_thresh_pretrig_ff);
	hs_key_hit2[ihs] = (hs_hit_pre_s0[ihs+MXHS*2] >= hit_thresh_pretrig_ff);
	hs_key_hit3[ihs] = (hs_hit_pre_s0[ihs+MXHS*3] >= hit_thresh_pretrig_ff);
	hs_key_hit4[ihs] = (hs_hit_pre_s0[ihs+MXHS*4] >= hit_thresh_pretrig_ff);

	hs_key_pid0[ihs] = (hs_pid_pre_s0[ihs+MXHS*0] >= pid_thresh_pretrig_ff);
	hs_key_pid1[ihs] = (hs_pid_pre_s0[ihs+MXHS*1] >= pid_thresh_pretrig_ff);
	hs_key_pid2[ihs] = (hs_pid_pre_s0[ihs+MXHS*2] >= pid_thresh_pretrig_ff);
	hs_key_pid3[ihs] = (hs_pid_pre_s0[ihs+MXHS*3] >= pid_thresh_pretrig_ff);
	hs_key_pid4[ihs] = (hs_pid_pre_s0[ihs+MXHS*4] >= pid_thresh_pretrig_ff);

	hs_key_dmb0[ihs] = (hs_hit_pre_s0[ihs+MXHS*0] >= dmb_thresh_pretrig_ff);
	hs_key_dmb1[ihs] = (hs_hit_pre_s0[ihs+MXHS*1] >= dmb_thresh_pretrig_ff);
	hs_key_dmb2[ihs] = (hs_hit_pre_s0[ihs+MXHS*2] >= dmb_thresh_pretrig_ff);
	hs_key_dmb3[ihs] = (hs_hit_pre_s0[ihs+MXHS*3] >= dmb_thresh_pretrig_ff);
	hs_key_dmb4[ihs] = (hs_hit_pre_s0[ihs+MXHS*4] >= dmb_thresh_pretrig_ff);

`elsif CSC_TYPE_B				// Reversed CSC
	hs_key_hit0[ihs] = (hs_hit_pre_s0[MXHS*5-1-ihs] >= hit_thresh_pretrig_ff);	// Reversed CSC
	hs_key_hit1[ihs] = (hs_hit_pre_s0[MXHS*4-1-ihs] >= hit_thresh_pretrig_ff);
	hs_key_hit2[ihs] = (hs_hit_pre_s0[MXHS*3-1-ihs] >= hit_thresh_pretrig_ff);
	hs_key_hit3[ihs] = (hs_hit_pre_s0[MXHS*2-1-ihs] >= hit_thresh_pretrig_ff);
	hs_key_hit4[ihs] = (hs_hit_pre_s0[MXHS*1-1-ihs] >= hit_thresh_pretrig_ff);

	hs_key_pid0[ihs] = (hs_pid_pre_s0[MXHS*5-1-ihs] >= pid_thresh_pretrig_ff);
	hs_key_pid1[ihs] = (hs_pid_pre_s0[MXHS*4-1-ihs] >= pid_thresh_pretrig_ff);
	hs_key_pid2[ihs] = (hs_pid_pre_s0[MXHS*3-1-ihs] >= pid_thresh_pretrig_ff);
	hs_key_pid3[ihs] = (hs_pid_pre_s0[MXHS*2-1-ihs] >= pid_thresh_pretrig_ff);
	hs_key_pid4[ihs] = (hs_pid_pre_s0[MXHS*1-1-ihs] >= pid_thresh_pretrig_ff);

	hs_key_dmb0[ihs] = (hs_hit_pre_s0[MXHS*5-1-ihs] >= dmb_thresh_pretrig_ff);
	hs_key_dmb1[ihs] = (hs_hit_pre_s0[MXHS*4-1-ihs] >= dmb_thresh_pretrig_ff);
	hs_key_dmb2[ihs] = (hs_hit_pre_s0[MXHS*3-1-ihs] >= dmb_thresh_pretrig_ff);
	hs_key_dmb3[ihs] = (hs_hit_pre_s0[MXHS*2-1-ihs] >= dmb_thresh_pretrig_ff);
	hs_key_dmb4[ihs] = (hs_hit_pre_s0[MXHS*1-1-ihs] >= dmb_thresh_pretrig_ff);

`else							// Reversed ME1B
	hs_key_hit0[ihs] = (hs_hit_pre_s0[MXHS*4-1-ihs] >= hit_thresh_pretrig_ff);	// Reversed ME1B, not reversed ME1A
	hs_key_hit1[ihs] = (hs_hit_pre_s0[MXHS*3-1-ihs] >= hit_thresh_pretrig_ff);
	hs_key_hit2[ihs] = (hs_hit_pre_s0[MXHS*2-1-ihs] >= hit_thresh_pretrig_ff);
	hs_key_hit3[ihs] = (hs_hit_pre_s0[MXHS*1-1-ihs] >= hit_thresh_pretrig_ff);
	hs_key_hit4[ihs] = (hs_hit_pre_s0[ihs+MXHS*4]   >= hit_thresh_pretrig_ff);

	hs_key_pid0[ihs] = (hs_pid_pre_s0[MXHS*4-1-ihs] >= pid_thresh_pretrig_ff);
	hs_key_pid1[ihs] = (hs_pid_pre_s0[MXHS*3-1-ihs] >= pid_thresh_pretrig_ff);
	hs_key_pid2[ihs] = (hs_pid_pre_s0[MXHS*2-1-ihs] >= pid_thresh_pretrig_ff);
	hs_key_pid3[ihs] = (hs_pid_pre_s0[MXHS*1-1-ihs] >= pid_thresh_pretrig_ff);
	hs_key_pid4[ihs] = (hs_pid_pre_s0[ihs+MXHS*4]   >= pid_thresh_pretrig_ff);

	hs_key_dmb0[ihs] = (hs_hit_pre_s0[MXHS*4-1-ihs] >= dmb_thresh_pretrig_ff);
	hs_key_dmb1[ihs] = (hs_hit_pre_s0[MXHS*3-1-ihs] >= dmb_thresh_pretrig_ff);
	hs_key_dmb2[ihs] = (hs_hit_pre_s0[MXHS*2-1-ihs] >= dmb_thresh_pretrig_ff);
	hs_key_dmb3[ihs] = (hs_hit_pre_s0[MXHS*1-1-ihs] >= dmb_thresh_pretrig_ff);
	hs_key_dmb4[ihs] = (hs_hit_pre_s0[ihs+MXHS*4]   >= dmb_thresh_pretrig_ff);
`endif
	end
	end
	endgenerate

// Output active FEB signal, and adjacent FEBs if hit is near board boundary
	//wire [4:1] cfebnm1_hit;	// Adjacent CFEB-1 has a pattern over threshold
	//wire [3:0] cfebnp1_hit;	// Adjacent CFEB+1 has a pattern over threshold
//Tao, Fixed the active cfeb flag bug here!!!  replace cfeb_hit by cfeb_dmb for active cfeb flag!
	wire [4:1] cfebnm1_dmb;	// Adjacent CFEB-1 has a pattern over threshold
	wire [3:0] cfebnp1_dmb;	// Adjacent CFEB+1 has a pattern over threshold
    wire [MXCFEB - 1: 0] cfeb_dmb; // This CFEB has a pattern over DMB-trigger threshold    

	wire [MXHS-1:0] hs_key_hitpid0 = hs_key_hit0 & hs_key_pid0;	// hits on key satify both hit and pid thresholds
	wire [MXHS-1:0] hs_key_hitpid1 = hs_key_hit1 & hs_key_pid1;
	wire [MXHS-1:0] hs_key_hitpid2 = hs_key_hit2 & hs_key_pid2;
	wire [MXHS-1:0] hs_key_hitpid3 = hs_key_hit3 & hs_key_pid3;
	wire [MXHS-1:0] hs_key_hitpid4 = hs_key_hit4 & hs_key_pid4;

	wire [MXHS-1:0] hs_key_dmbpid0 = hs_key_dmb0 & hs_key_pid0;	// hits on key satify both hit and pid thresholds
	wire [MXHS-1:0] hs_key_dmbpid1 = hs_key_dmb1 & hs_key_pid1;
	wire [MXHS-1:0] hs_key_dmbpid2 = hs_key_dmb2 & hs_key_pid2;
	wire [MXHS-1:0] hs_key_dmbpid3 = hs_key_dmb3 & hs_key_pid3;
	wire [MXHS-1:0] hs_key_dmbpid4 = hs_key_dmb4 & hs_key_pid4;

	wire cfeb_layer_trigger = cfeb_layer_trig && layer_trig_en_ff;

	assign cfeb_hit[0] = ((|hs_key_hitpid0) || cfeb_layer_trigger) && cfeb_en_ff[0];
	assign cfeb_hit[1] = ((|hs_key_hitpid1) || cfeb_layer_trigger) && cfeb_en_ff[1];
	assign cfeb_hit[2] = ((|hs_key_hitpid2) || cfeb_layer_trigger) && cfeb_en_ff[2];
	assign cfeb_hit[3] = ((|hs_key_hitpid3) || cfeb_layer_trigger) && cfeb_en_ff[3];
	assign cfeb_hit[4] = ((|hs_key_hitpid4) || cfeb_layer_trigger) && cfeb_en_ff[4];

	//assign cfebnm1_hit[1]	= | (hs_key_hitpid1 & adjcfeb_mask_nm1);
	//assign cfebnm1_hit[2]	= | (hs_key_hitpid2 & adjcfeb_mask_nm1);
	//assign cfebnm1_hit[3]	= | (hs_key_hitpid3 & adjcfeb_mask_nm1);
	//assign cfebnm1_hit[4]	=(| (hs_key_hitpid4 & adjcfeb_mask_nm1)) && !csc_me1ab;	// Turn off adjacency for me1ab

	//assign cfebnp1_hit[0]	= | (hs_key_hitpid0 & adjcfeb_mask_np1);
	//assign cfebnp1_hit[1]	= | (hs_key_hitpid1 & adjcfeb_mask_np1);
	//assign cfebnp1_hit[2]	= | (hs_key_hitpid2 & adjcfeb_mask_np1);
	//assign cfebnp1_hit[3]	=(| (hs_key_hitpid3 & adjcfeb_mask_np1)) && !csc_me1ab;	// Turn off adjacency for me1ab

    //// Output active FEB signal, and adjacent FEBs if hit is near board boundary
	//assign cfeb_active[0]	=	(cfeb_hit[0] ||                   cfebnm1_hit[1] || (| hs_key_dmb0)) && cfeb_en_ff[0];
	//assign cfeb_active[1]	=	(cfeb_hit[1] || cfebnp1_hit[0] || cfebnm1_hit[2] || (| hs_key_dmb1)) && cfeb_en_ff[1];
	//assign cfeb_active[2]	=	(cfeb_hit[2] || cfebnp1_hit[1] || cfebnm1_hit[3] || (| hs_key_dmb2)) && cfeb_en_ff[2];
	//assign cfeb_active[3]	=	(cfeb_hit[3] || cfebnp1_hit[2] || cfebnm1_hit[4] || (| hs_key_dmb3)) && cfeb_en_ff[3];
	//assign cfeb_active[4]	=	(cfeb_hit[4] || cfebnp1_hit[3]                   || (| hs_key_dmb4)) && cfeb_en_ff[4];
	assign cfeb_dmb[0] = ((|hs_key_dmbpid0) || cfeb_layer_trigger) && cfeb_en_ff[0];
	assign cfeb_dmb[1] = ((|hs_key_dmbpid1) || cfeb_layer_trigger) && cfeb_en_ff[1];
	assign cfeb_dmb[2] = ((|hs_key_dmbpid2) || cfeb_layer_trigger) && cfeb_en_ff[2];
	assign cfeb_dmb[3] = ((|hs_key_dmbpid3) || cfeb_layer_trigger) && cfeb_en_ff[3];
	assign cfeb_dmb[4] = ((|hs_key_dmbpid4) || cfeb_layer_trigger) && cfeb_en_ff[4];

	assign cfebnm1_dmb[1]	= | (hs_key_dmbpid1 & adjcfeb_mask_nm1);
	assign cfebnm1_dmb[2]	= | (hs_key_dmbpid2 & adjcfeb_mask_nm1);
	assign cfebnm1_dmb[3]	= | (hs_key_dmbpid3 & adjcfeb_mask_nm1);
	assign cfebnm1_dmb[4]	=(| (hs_key_dmbpid4 & adjcfeb_mask_nm1)) && !csc_me1ab;	// Turn off adjacency for me1ab

	assign cfebnp1_dmb[0]	= | (hs_key_dmbpid0 & adjcfeb_mask_np1);
	assign cfebnp1_dmb[1]	= | (hs_key_dmbpid1 & adjcfeb_mask_np1);
	assign cfebnp1_dmb[2]	= | (hs_key_dmbpid2 & adjcfeb_mask_np1);
	assign cfebnp1_dmb[3]	=(| (hs_key_dmbpid3 & adjcfeb_mask_np1)) && !csc_me1ab;	// Turn off adjacency for me1ab

// Output active FEB signal, and adjacent FEBs if hit is near board boundary
	assign cfeb_active[0]	=	(cfeb_dmb[0] ||                   cfebnm1_dmb[1] || (| hs_key_dmb0)) && cfeb_en_ff[0];
	assign cfeb_active[1]	=	(cfeb_dmb[1] || cfebnp1_dmb[0] || cfebnm1_dmb[2] || (| hs_key_dmb1)) && cfeb_en_ff[1];
	assign cfeb_active[2]	=	(cfeb_dmb[2] || cfebnp1_dmb[1] || cfebnm1_dmb[3] || (| hs_key_dmb2)) && cfeb_en_ff[2];
	assign cfeb_active[3]	=	(cfeb_dmb[3] || cfebnp1_dmb[2] || cfebnm1_dmb[4] || (| hs_key_dmb3)) && cfeb_en_ff[3];
	assign cfeb_active[4]	=	(cfeb_dmb[4] || cfebnp1_dmb[3]                   || (| hs_key_dmb4)) && cfeb_en_ff[4];


//-------------------------------------------------------------------------------------------------------------------
// Stage 5B: 1/2-Strip Priority Encoder
// 			 Select the 1st best pattern from 160 Key 1/2-Strips
//-------------------------------------------------------------------------------------------------------------------
// Best 5 of 160 1/2-strip patterns
	wire [MXPATB-1:0] hs_pat_s1	[4:0];
	wire [MXKEYB-1:0] hs_key_s1	[4:0];	// partial key for 1 of 32

	genvar i;
	generate
	for (i=0; i<=4; i=i+1) begin: hs_gen
	best_1of32 ubest1of32_1st
	(
	clock,
	hs_pat_s0[i*32+ 0], hs_pat_s0[i*32+ 1], hs_pat_s0[i*32+ 2], hs_pat_s0[i*32+ 3], hs_pat_s0[i*32+ 4], hs_pat_s0[i*32+ 5], hs_pat_s0[i*32+ 6], hs_pat_s0[i*32+ 7],
	hs_pat_s0[i*32+ 8], hs_pat_s0[i*32+ 9], hs_pat_s0[i*32+10], hs_pat_s0[i*32+11], hs_pat_s0[i*32+12], hs_pat_s0[i*32+13], hs_pat_s0[i*32+14], hs_pat_s0[i*32+15],
	hs_pat_s0[i*32+16], hs_pat_s0[i*32+17], hs_pat_s0[i*32+18], hs_pat_s0[i*32+19], hs_pat_s0[i*32+20], hs_pat_s0[i*32+21], hs_pat_s0[i*32+22], hs_pat_s0[i*32+23],
	hs_pat_s0[i*32+24], hs_pat_s0[i*32+25], hs_pat_s0[i*32+26], hs_pat_s0[i*32+27], hs_pat_s0[i*32+28], hs_pat_s0[i*32+29], hs_pat_s0[i*32+30], hs_pat_s0[i*32+31],
	hs_pat_s1[i],
	hs_key_s1[i]
	);
	end
	endgenerate

// Best 1 of 5 1/2-strip patterns
	wire [MXPATB-1:0]	hs_pat_s2;
	wire [MXKEYBX-1:0]	hs_key_s2;		// full key for 1 of 160

	best_1of5 ubest1of5_1st
	(
	hs_pat_s1[0], hs_pat_s1[1], hs_pat_s1[2], hs_pat_s1[3], hs_pat_s1[4],
	hs_key_s1[0], hs_key_s1[1], hs_key_s1[2], hs_key_s1[3], hs_key_s1[4],
	hs_pat_s2,
	hs_key_s2
	);

// Latch final hs pattern data for 1st CLCT
	reg	[MXPATB-1:0]	hs_pat_1st_nodly;
	reg	[MXKEYBX-1:0]	hs_key_1st_nodly;
	
	always @(posedge clock) begin
	hs_pat_1st_nodly <= hs_pat_s2;
	hs_key_1st_nodly <= hs_key_s2;
	end

//-------------------------------------------------------------------------------------------------------------------
// Stage 6A: Delay 1st CLCT to output at same time as 2nd CLCT
//-------------------------------------------------------------------------------------------------------------------
	wire [MXPATB-1:0]  hs_pat_1st_dly;
	wire [MXKEYBX-1:0] hs_key_1st_dly;
	wire [MXHITB-1:0]  hs_hit_1st_dly;

	parameter cdly = 4'd0;

	srl16e_bbl #(MXPATB ) upatbbl (.clock(clock),.ce(1'b1),.adr(cdly),.d(hs_pat_1st_nodly),.q(hs_pat_1st_dly));
	srl16e_bbl #(MXKEYBX) ukeybbl (.clock(clock),.ce(1'b1),.adr(cdly),.d(hs_key_1st_nodly),.q(hs_key_1st_dly));

// Final 1st CLCT flipflop
	reg  [MXPIDB-1:0]	hs_pid_1st;
	reg  [MXHITB-1:0]	hs_hit_1st;
	reg  [MXKEYBX-1:0]	hs_key_1st;

	assign hs_hit_1st_dly = hs_pat_1st_dly[MXPATB-1:MXPIDB];
	wire   blank_1st	  = ((hs_hit_1st_dly==0) && (clct_blanking==1)) || purging;
	wire   lyr_trig_1st	  = (hs_layer_latch && layer_trig_en_ff);

	always @(posedge clock) begin
	if (blank_1st) begin							// blank 1st clct
	hs_pid_1st	<= 0;
	hs_hit_1st	<= 0;
	hs_key_1st	<= 0;
	end
	else if (lyr_trig_1st) begin					// layer-trigger mode
	hs_pid_1st	<= 1;								// Pattern id=1 for layer triggers
	hs_hit_1st	<= hs_nlayers_hit_dly;				// Insert number of layers hit
	hs_key_1st	<= 0;								// Dummy key
	end
	else begin										// else assert final 1st clct
	hs_key_1st <= hs_key_1st_dly;
	hs_pid_1st <= hs_pat_1st_dly[MXPIDB-1:0];
	hs_hit_1st <= hs_pat_1st_dly[MXPATB-1:MXPIDB];
	end
	end

// FF layer-mode status
	reg hs_layer_trig;
	reg [MXLY-1:0]		hs_layer_or;
	reg [MXHITB-1:0]	hs_nlayers_hit;

	always @(posedge clock) begin
	hs_layer_trig	<= hs_layer_trig_dly;
	hs_layer_or		<= hs_layer_or_dly;
	hs_nlayers_hit	<= hs_nlayers_hit_dly;
	end

//-------------------------------------------------------------------------------------------------------------------
// Stage 6B: Mark key 1/2-strips near the 1st CLCT key as busy to exclude them from 2nd CLCT priority encoding
//-------------------------------------------------------------------------------------------------------------------
// Dual-Port RAM with Asynchronous Read: look up busy key region for excluding 2nd clct, port A=VME r/w, port B=readonly
	wire [3:0]  adra;					// Port A address, set by VME register
	wire [3:0]  adrb;					// Port B address, set by pattern ID number 0 to 9
	wire [15:0] rdataa;					// Port A read  data, read by VME register
	wire [15:0] rdatab;					// Port B read  data, reads out pspan,nspan for this pattern ID number 
	wire [15:0] wdataa;					// Port A write data, written by VME register, there is no portb wdatab

	assign wea    =	clct_sep_ram_we;
	assign adra   =	clct_sep_ram_adr;
	assign wdataa = clct_sep_ram_wdata;

	assign clct_sep_ram_rdata = rdataa;
	assign adrb[3:0]          = hs_pat_s2[MXPIDB-1:0];	// Pattern ID points to nspan,pspan values for this bend angle

// Instantiate 16adr x 16bit dual port RAM
// Port A: write/read via VME
// Port B: readonly pattern ID lookup
// Initial RAM contents   FFEEDDCCBBAA99887766554433221100
	parameter nsep = 128'h0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A;
	parameter psep = 128'h0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A0A;
	generate
	for (i=0; i<=15; i=i+1) begin: sepram
	RAM16X1D uram16x1d
	(
	.WCLK	(clock),		// Port A	Write clock input
	.WE		(wea),			// Port A	Write enable input
	.A0		(adra[0]),		// Port A	R/W address[0] input bit
	.A1		(adra[1]),		// Port A	R/W address[1] input bit
	.A2		(adra[2]),		// Port A	R/W address[2] input bit
	.A3		(adra[3]),		// Port A	R/W address[3] input bit
	.D		(wdataa[i]),	// Port A	Write 1-bit data input
	.SPO	(rdataa[i]),	// Port A	R/W 1-bit data output for A0-A3

	.DPRA0	(adrb[0]),		// Port B	Read address[0] input bit
	.DPRA1	(adrb[1]),		// Port B	Read address[1] input bit
	.DPRA2	(adrb[2]),		// Port B	Read address[2] input bit
	.DPRA3	(adrb[03]),		// Port B	Read address[3] input bit
	.DPO	(rdatab[i])		// Port B	Read-only 1-bit data output for DPRA
	);

	if (i<=7) begin: gena defparam sepram[i].uram16x1d.INIT = {nsep[i+120],nsep[i+112],nsep[i+104],nsep[i+96],nsep[i+88],nsep[i+80],nsep[i+72],nsep[i+64],nsep[i+56],nsep[i+48],nsep[i+40],nsep[i+32],nsep[i+24],nsep[i+16],nsep[i+8],nsep[i-0]}; end
	else 	  begin: genb defparam sepram[i].uram16x1d.INIT = {psep[i+112],psep[i+104],psep[i+96 ],psep[i+88],psep[i+80],psep[i+72],psep[i+64],psep[i+56],psep[i+48],psep[i+40],psep[i+32],psep[i+24],psep[i+16],psep[i+8 ],psep[i+0],psep[i-8]}; end
	end
	endgenerate

// Extract busy key spans from RAM data
	wire [7:0] nspan_ram;
	wire [7:0] pspan_ram;

	assign nspan_ram = rdatab[ 7:0];
	assign pspan_ram = rdatab[15:8];

// Multiplex with single-parameter busy key span from vme
 	reg [7:0] nspan;
	reg [7:0] pspan;

	always @(posedge clock) begin
	nspan <= (clct_sep_src) ? clct_sep_vme : nspan_ram;
	pspan <= (clct_sep_src) ? clct_sep_vme : pspan_ram;
	end

// CSC Type A or B delimiters for excluding 2nd clct span hs0-159
	reg [MXKEYBX-1:0] busy_min;
	reg [MXKEYBX-1:0] busy_max;

	`ifdef CSC_TYPE_A_or_CSC_TYPE_B

	always @* begin
	busy_max <= (hs_key_s2 <= 159-pspan) ? hs_key_s2+pspan : 8'd159;	// Limit busy list to range 0-159
	busy_min <= (hs_key_s2 >= nspan    ) ? hs_key_s2-nspan : 8'd0;
	end

// CSC Type C or D delimiters for excluding 2nd clct span ME1B hs0-127  ME1A hs128-159
	`elsif CSC_TYPE_C_or_CSC_TYPE_D

	wire clct0_is_on_me1a = hs_key_s2[MXKEYBX-1];
	
	always @* begin
	if (clct0_is_on_me1a) begin	// CLCT0 is on ME1A cfeb4, limit blanking region to 128-159
	busy_max <= (hs_key_s2 <= 159-pspan) ? hs_key_s2+pspan : 8'd159;
	busy_min <= (hs_key_s2 >= 128+nspan) ? hs_key_s2-nspan : 8'd128;
	end
	else begin					// CLCT0 is on ME1B cfeb0-cfeb3, limit blanking region to 0-127
	busy_max <= (hs_key_s2 <= 127-pspan) ? hs_key_s2+pspan : 8'd127;
	busy_min <= (hs_key_s2 >=     nspan) ? hs_key_s2-nspan : 8'd0;
	end
	end

// CSC Type missing
	`else
	initial $display ("CSC_TYPE undefined for 2nd clct delimiters in pattern_finder.v: Halting");
	$finish
	`endif

// Latch busy key 1/2-strips for excluding 2nd clct
	reg [MXHSX-1:0] busy_key;

	genvar ikey;
	generate
	for (ikey=0; ikey<=MXHSX-1; ikey=ikey+1) begin: bloop
	always @(posedge clock) begin
	busy_key[ikey] <= (ikey>=busy_min)&&(ikey<=busy_max);
	end
	end
	endgenerate

//-------------------------------------------------------------------------------------------------------------------
// Stage 7A: 1/2-Strip Priority Encoder
// 			Find 2nd best of 160 patterns, excluding busy region around 1st best key
//-------------------------------------------------------------------------------------------------------------------
// Delay 1st CLCT pattern numbers to align in time with 1st CLCT busy keys
	wire [MXPATB-1:0] hs_pat_s3 [MXHSX-1:0];

	parameter pdly = 4'd1;

	genvar ibit;
	generate
	for (ikey=0; ikey<=MXHSX-1;  ikey=ikey+1) begin: key_loop
	for (ibit=0; ibit<=MXPATB-1; ibit=ibit+1) begin: bit_loop
	SRL16E u0 (.CLK(clock),.CE(1'b1),.D(hs_pat_s0[ikey][ibit]),.A0(pdly[0]),.A1(pdly[1]),.A2(pdly[2]),.A3(pdly[3]),.Q(hs_pat_s3[ikey][ibit]));
	end
	end
	endgenerate

// Best 5 of 160 1/2-strip patterns
	wire [MXPATB-1:0] hs_pat_s4 [4:0];
	wire [MXKEYB-1:0] hs_key_s4 [4:0];	// partial key for 1 of 32
	wire [4:0] 		  hs_bsy_s4;
	
	generate
	for (i=0; i<=4; i=i+1) begin: hs_2nd_gen
	best_1of32_busy ubest1of32_2nd
	(
	clock,
	hs_pat_s3[i*32+ 0], hs_pat_s3[i*32+ 1], hs_pat_s3[i*32+ 2], hs_pat_s3[i*32+ 3], hs_pat_s3[i*32+ 4], hs_pat_s3[i*32+ 5], hs_pat_s3[i*32+ 6], hs_pat_s3[i*32+ 7],
	hs_pat_s3[i*32+ 8], hs_pat_s3[i*32+ 9], hs_pat_s3[i*32+10], hs_pat_s3[i*32+11], hs_pat_s3[i*32+12], hs_pat_s3[i*32+13], hs_pat_s3[i*32+14], hs_pat_s3[i*32+15],
	hs_pat_s3[i*32+16], hs_pat_s3[i*32+17], hs_pat_s3[i*32+18], hs_pat_s3[i*32+19], hs_pat_s3[i*32+20], hs_pat_s3[i*32+21], hs_pat_s3[i*32+22], hs_pat_s3[i*32+23],
	hs_pat_s3[i*32+24], hs_pat_s3[i*32+25], hs_pat_s3[i*32+26], hs_pat_s3[i*32+27], hs_pat_s3[i*32+28], hs_pat_s3[i*32+29], hs_pat_s3[i*32+30], hs_pat_s3[i*32+31],
	busy_key[i*32+31:i*32],

	hs_pat_s4[i],
	hs_key_s4[i],
	hs_bsy_s4[i]
	);
	end
	endgenerate

// Best 1 of 5 1/2-strip patterns
	wire [MXPATB-1:0]	hs_pat_s5;
	wire [MXKEYBX-1:0]	hs_key_s5;		// full key for 1 of 160
	wire [MXHITB-1:0]	hs_hit_s5;
	wire				hs_bsy_s5;

	best_1of5_busy ubest1of5_2nd
	(
	hs_pat_s4[0], hs_pat_s4[1], hs_pat_s4[2], hs_pat_s4[3], hs_pat_s4[4],
	hs_key_s4[0], hs_key_s4[1], hs_key_s4[2], hs_key_s4[3], hs_key_s4[4],
	hs_bsy_s4[0], hs_bsy_s4[1], hs_bsy_s4[2], hs_bsy_s4[3], hs_bsy_s4[4],
	hs_pat_s5,
	hs_key_s5,
	hs_bsy_s5
	);

// Latch final 2nd CLCT
	reg	 [MXPIDB-1:0]	hs_pid_2nd;
	reg	 [MXHITB-1:0]	hs_hit_2nd;
	reg	 [MXKEYBX-1:0]	hs_key_2nd;
	reg					hs_bsy_2nd;

	assign hs_hit_s5    = hs_pat_s5[MXPATB-1:MXPIDB];
	wire   blank_2nd    = ((hs_hit_s5==0) && (clct_blanking==1)) || purging;
	wire   lyr_trig_2nd = (hs_layer_latch && layer_trig_en_ff);

	always @(posedge clock) begin
	if (blank_2nd) begin
	hs_pid_2nd	<= 0;
	hs_hit_2nd	<= 0;
	hs_key_2nd	<= 0;
	hs_bsy_2nd	<= hs_bsy_s5;
	end
	else if (lyr_trig_2nd) begin				// layer-trigger mode
	hs_pid_2nd	<= 0;
	hs_hit_2nd	<= 0;
	hs_key_2nd	<= 0;
	hs_bsy_2nd	<= hs_bsy_s5;
	end
	else begin									// else assert final 2nd clct
	hs_pid_2nd	<= hs_pat_s5[MXPIDB-1:0];
	hs_hit_2nd	<= hs_pat_s5[MXPATB-1:MXPIDB];
	hs_key_2nd	<= hs_key_s5;
	hs_bsy_2nd	<= hs_bsy_s5;
	end
	end

//-------------------------------------------------------------------------------------------------------------------
	endmodule
//-------------------------------------------------------------------------------------------------------------------
