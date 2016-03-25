`timescale 1ns / 1ps
//`define DEBUG_RPC 1
//------------------------------------------------------------------------------------------------------------------
//	RPC
//		Receives 80MHz data from RPC
//		Demultiplexes, stores in RAM, and sends data to TMB
//------------------------------------------------------------------------------------------------------------------
//	03/13/02 Initial
//	10/06/03 TMB2003 mods
//	03/12/04 Add RPC demux, RAT controls, and RPC raw hits RAM
//	03/16/04 Correct bxn 
//	04/20/04 Revert DDR to 80MHz
//	04/26/04 Add programmable delay
//	04/26/04 Add injector and raw hits RAM for DMB readout
//	04/28/04 Re-order rpc raw hits into link-board groups
//	04/30/04 Add scope outputs
//	05/05/04 Remove RAMB16, they will not initialize for some reason
//	05/06/04 New injector default pattern, add rat sync pulse
//	05/10/04 Widen bxn offset
//	05/12/04 Separate RPC units for readout, allows sparsification
//	05/13/04 More mods
//	05/17/04 RAM address counter now preloaded and unbuffered for speed
//	05/18/04 Debugs removed, rat injector delay moved to sequencer
//	05/20/04 Take fifo_busy down one clock earlier for sequencer, narrow rpc_tbinbxn to 4 bits
//	06/07/04 Change to x_demux_v2 which has aset for mpc
//	06/09/05 TMB2005 Change rpc_rxalt[1:0] to rpc_dsn,rpc_smbrx
//	08/28/06 Add 3d3444 signal multiplexer
//	09/11/06 Mods for xst
//	09/18/06 Convert to Virtex 2 RAMs
//	10/05/06 Replace for-loops with while-loops for xst
//	10/09/06 Convert to ddr receivers
//	04/04/07 Reduce to 2 RPCs
//	05/09/07 Update srl16e_bbl port names
//	07/16/07 Mod RAMs, add ascii display for state machine
//	07/26/07 Reduce rams, delay injector_go_rpc to match cfeb raw hits
//	08/02/07 Extend pattern injector to 1k tbins, add programmable firing length
//	08/06/07 Replace raw hits rams with inferred units
//	09/19/07 Remove raw hits delay
//	09/26/07 Separate rpc fifo outputs to match new fifo controller
//	10/05/07 Trim mux case variables
//	02/01/08 Add parity to raw hits rams for seu detection
//	02/05/08 Replace inferred raw hits ram with instantiated ram, xst fails to build dual port parity block rams
//	11/18/08 Change raw hits ram access to read-first so parity output is always valid
//	11/26/08 Mod read first on raw hits rams, keep write first on injector rams
//	04/24/09 Add recovery to state machine
//	05/08/09 Add pre-trigger marker to rpc ram
//	05/11/09 Add data=address test mode
//	05/11/09 Remove 4 RPC references, replace DDR receiver to store only 1st-in-time data\
//	07/22/09 Remove clock_vme global net to make room for cfeb digital phase shifter gbufs
//	08/07/09 Revert to 10mhz vme clock
//	08/12/09 Remove clock_vme again
//------------------------------------------------------------------------------------------------------------------
	module rpc
	(
// RAT Module Signals
	clock,
	global_reset,		
	rpc_rx,			
	rpc_smbrx,
	rpc_tx,

// RAT Control
	rpc_sync,
	rpc_posneg,
	rpc_loop_tmb,
	rpc_free_tx0,
	smb_data_rat,

// RAT 3D3444 Delay Signals
	dddr_clock,
	dddr_adr_latch,
	dddr_serial_in,
	dddr_busy,

// RAT Serial Number
	rat_sn_out,
	rat_dsn_en,

// RPC Injector Ports
	mask_all,
	injector_go_rpc,
	injector_go_rat,
	inj_last_tbin,
	rpc_tbins_test,
	inj_wen,
	inj_rwadr,
	inj_wdata,
	inj_ren,
	inj_rdata,

// RPC Raw Hits Delay
	rpc0_delay,
	rpc1_delay,
	
// Raw Hits FIFO RAM Ports
	fifo_wen,
	fifo_wadr,
	fifo_radr,
	fifo_sel,
	fifo0_rdata,
	fifo1_rdata,

// RPC Scope Ports
	scp_rpc0_bxn,
	scp_rpc1_bxn,
	scp_rpc0_nhits,
	scp_rpc1_nhits,

// RPC Raw hits VME Readback Ports
	rpc_bank,
	rpc_rdata,
	rpc_rbxn,

// RPC Hot Channel Mask Ports
	rpc0_hcm,
	rpc1_hcm,

// RPC Bxn Offset
	rpc_bxn_offset,
	rpc0_bxn_diff,
	rpc1_bxn_diff,

// Status
	clct_pretrig,
	parity_err_rpc,

// Sump
	rpc_sump
	
// Debug
`ifdef DEBUG_RPC
	,inj_sm_dsp
`endif
	);
//------------------------------------------------------------------------------------------------------------------
// Constants:
//------------------------------------------------------------------------------------------------------------------
	parameter MXRPC				=	2;			// Number RPCs
	parameter MXRPCB			=	1;			// Number RPC ID bits
	parameter MXRPCPAD			=	16;			// Number RPC pads per link board
	parameter MXRPCDB			=	19;			// Number RPC bits per link board
	parameter MXRPCRX			=	38;			// Number RPC bits per phase from RAT module
	parameter MXTBIN			=	5;			// Time bin address width
	parameter MXBUF				=	8;			// Number of buffers
	parameter MXBUFB			=	3;			// Buffer address width 
	parameter READ_ADR_OFFSET	=	5;			// Number clocks from first address to pretrigger adr latch, trial 03/01/03

// Raw hits RAM parameters
	parameter RAM_DEPTH		= 2048;				// Storage bx depth
	parameter RAM_ADRB		= 11;				// Address width=log2(ram_depth)
	parameter RAM_WIDTH		= 8;				// Data width

//------------------------------------------------------------------------------------------------------------------
// IO:
//------------------------------------------------------------------------------------------------------------------
// RAT Module Ports
	input					clock;				// TMB 40MHz main
	input					global_reset;		// TMB global reset
	input	[MXRPCRX-1:0]	rpc_rx;				// RPC data inputs		
	input					rpc_smbrx;			// RPC SMB receive data
	output	[3:0]			rpc_tx;				// RPC control output

// RAT Control Ports
	input					rpc_sync;			// Sync mode
	input					rpc_posneg;			// Clock phase
	input					rpc_loop_tmb;		// Loop mode (loops in RAT Spartan)
	input					rpc_free_tx0;		// Unassigned
	output					smb_data_rat;		// RAT smb_data

// RAT 3D3444 Delay Signals
	input					dddr_clock;			// DDDR clock			/ rpc_sync
	input					dddr_adr_latch;		// DDDR address latch	/ rpc_posneg
	input					dddr_serial_in;		// DDDR serial in		/ rpc_loop_tmb
	input					dddr_busy;			// DDDR busy			/ rpc_free_tx0

// RAT Serial Number Ports
	input					rat_sn_out;			// RAT serial number, out = rpc_posneg
	input					rat_dsn_en;			// RAT dsn enable

// RPC Injector Ports
	input					mask_all;			// 1=Enable, 0=Turn off all RPC inputs
	input					injector_go_rpc;	// 1=Start RPC pattern injector
	input					injector_go_rat;	// 1=Start RAT pattern injector
	input	[11:0]			inj_last_tbin;		// Last tbin, may wrap past 1024 ram adr
	input					rpc_tbins_test;		// Set write_data=address
	input	[MXRPC-1:0]		inj_wen;			// 1=Write enable injector RAM
	input	[9:0]			inj_rwadr;			// Injector RAM read/write address
	input	[MXRPCDB-1:0]	inj_wdata;			// Injector RAM write data
	input	[MXRPC-1:0]		inj_ren;			// 1=Read enable Injector RAM
	output	[MXRPCDB-1:0]	inj_rdata;			// Injector RAM read data

// RPC Raw Hits Delay
	input	[3:0]			rpc0_delay;			// RPC0 raw hits delay
	input	[3:0]			rpc1_delay;			// RPC1 raw hits delay

// Raw Hits FIFO RAM Ports
	input					fifo_wen;			// 1=Write enable FIFO RAM
	input	[RAM_ADRB-1:0]	fifo_wadr;			// FIFO RAM write address
	input	[RAM_ADRB-1:0]	fifo_radr;			// FIFO RAM read address
	input	[0:0]			fifo_sel;			// FIFO RAM select bank 0-1

	output	[RAM_WIDTH-1+4:0]fifo0_rdata;		// FIFO RAM read data
	output	[RAM_WIDTH-1+4:0]fifo1_rdata;		// FIFO RAM read data

// RPC Scope Ports
	output	[2:0]			scp_rpc0_bxn;		// RPC0 bunch crossing number
	output	[2:0]			scp_rpc1_bxn;		// RPC1 bunch crossing number
	output	[3:0]			scp_rpc0_nhits;		// RPC0 number of pads hit
	output	[3:0]			scp_rpc1_nhits;		// RPC1 number of pads hit

// RPC Raw hits VME Readback Ports
	input	[MXRPCB-1:0]	rpc_bank;			// RPC bank address
	output	[15:0]			rpc_rdata;			// RPC RAM read data
	output	[2:0]			rpc_rbxn;			// RPC RAM read bxn

// RPC Hot Channel Mask Ports
	input	[MXRPCPAD-1:0]	rpc0_hcm;			// 1=enable RPC pad
	input	[MXRPCPAD-1:0]	rpc1_hcm;			// 1=enable RPC pad

// RPC Bxn Offset Ports
	input	[3:0]			rpc_bxn_offset;		// RPC bunch crossing offset
	output	[3:0]			rpc0_bxn_diff;		// RPC - offset
	output	[3:0]			rpc1_bxn_diff;		// RPC - offset

// Status Ports
	input					clct_pretrig;		// Pre-trigger marker at (clct_sm==pretrig)
	output	[4:0]			parity_err_rpc;		// Raw hits RAM parity error detected

// Sump
	output					rpc_sump;			// Unused signals

// Debug
`ifdef DEBUG_RPC
	output	[71:0]			inj_sm_dsp;			// Injector state machine ascii display
`endif

//------------------------------------------------------------------------------------------------------------------
// RAT Module Control:
//------------------------------------------------------------------------------------------------------------------	
// Buffer RAT Module control signals
	reg  [3:0]	rpc_tx_std=0;		// Normal mode
	wire [3:0]	rpc_tx_dly;			// 3D3444 writing mode
	reg			smb_data_rat=0;
	reg			test=0;

	always @(posedge clock) begin
	rpc_tx_std[0] 	<= rpc_sync | injector_go_rat;
	rpc_tx_std[1] 	<= (rpc_posneg  || (rat_sn_out && rat_dsn_en)) & !rpc_loop_tmb;
	rpc_tx_std[2] 	<= rpc_loop_tmb;
	rpc_tx_std[3] 	<= rpc_free_tx0;
	smb_data_rat	<= rpc_smbrx;
	test			<= rpc_tbins_test;
	end

	assign rpc_tx_dly[0] = dddr_clock;			// rpc_sync
	assign rpc_tx_dly[1] = dddr_adr_latch;		// rpc_posneg
	assign rpc_tx_dly[2] = dddr_serial_in;		// rpc_loop_tmb
	assign rpc_tx_dly[3] = dddr_busy;			// rpc_free_tx0

	assign rpc_tx[3:0] = (dddr_busy) ? (rpc_tx_dly[3:0]) : (rpc_tx_std[3:0]); // multiplexer engages to write RAT 3D3444 chip

//------------------------------------------------------------------------------------------------------------------
// RPC Demultiplexer:
//------------------------------------------------------------------------------------------------------------------
// All-channels mask FF, 1=enable RPC bits, 0=clear in IOB
	reg rpc_clr;
	
	always @(posedge clock or posedge global_reset) begin
	if (global_reset)	rpc_clr = 1'b1;
	else				rpc_clr = !mask_all;
	end

	wire sm_reset=global_reset;

// Demux RPC 80MHz data from RAT module in DDR IOB FFs, rpc0,1 arrive 1st phase, future rpc2,3 arrive 2nd phase
	reg	[MXRPCRX-1:0] din1st=0;

	wire aset = 1'b0;
	wire aclr = rpc_clr;

	always @(posedge clock or posedge aset or posedge aclr) begin	// Latch 1st-in-time on rising edge
	if		(aclr)	din1st = {MXRPCRX{1'b0}};		// async clear
	else if	(aset)	din1st = {MXRPCRX{1'b1}};		// async set
	else			din1st = rpc_rx[MXRPCRX-1:0];	// sync  store
	end

// Delay RPC raw hits
	wire [18:0] rpc0_rx_srl, rpc0_rx_dly;
	wire [18:0] rpc1_rx_srl, rpc1_rx_dly;

	reg  [3:0] rpc0_srl_adr    = 0;
	reg  [3:0] rpc1_srl_adr    = 0;
	reg		   rpc0_delay_is_0 = 0;
	reg		   rpc1_delay_is_0 = 0;

	always @(posedge clock) begin
	rpc0_srl_adr 	<= (rpc0_delay - 1);
	rpc1_srl_adr 	<= (rpc1_delay - 1);
	rpc0_delay_is_0	<= (rpc0_delay == 0);	// Use direct input if SRL address is 0, 1st SRL output has 1bx overhead
	rpc1_delay_is_0	<= (rpc1_delay == 0);
	end

	wire [MXRPCRX-1:0] rpc_rxdemux = din1st;

	srl16e_bbl #(19) urpc0dly (.clock(clock),.ce(1'b1),.adr(rpc0_srl_adr),.d(rpc_rxdemux[18: 0]),.q(rpc0_rx_srl));
	srl16e_bbl #(19) urpc1dly (.clock(clock),.ce(1'b1),.adr(rpc1_srl_adr),.d(rpc_rxdemux[37:19]),.q(rpc1_rx_srl));

	assign rpc0_rx_dly = (rpc0_delay_is_0) ? rpc_rxdemux[18: 0] : rpc0_rx_srl;
	assign rpc1_rx_dly = (rpc1_delay_is_0) ? rpc_rxdemux[37:19] : rpc1_rx_srl;

// Pack demux RPC data into 2d array
	wire [18:0]	rpc_rxd [1:0];

	assign rpc_rxd[0][18:0] = rpc0_rx_dly[18:0];
	assign rpc_rxd[1][18:0] = rpc1_rx_dly[18:0];

//------------------------------------------------------------------------------------------------------------------
// RPC Raw Hits Injector:
//------------------------------------------------------------------------------------------------------------------
/// Injector State Machine Declarations
	reg [1:0] inj_sm;		// synthesis attribute safe_implementation of inj_sm is "yes";
	parameter pass		= 0;
	parameter injecting	= 1;

// Injector State Machine
	wire inj_tbin_cnt_done;

	always @(posedge clock or posedge sm_reset) begin

	if(sm_reset)inj_sm = pass;
	else begin
	case (inj_sm)

	pass:
	 if (injector_go_rpc)
	 inj_sm =	injecting;

	injecting:
	 if (inj_tbin_cnt_done)
	 inj_sm	=	pass;
	
	default
	 inj_sm	=	pass;
	endcase
	end
	end

// Injector Time Bin Counter
	reg  [11:0] inj_tbin_cnt;	// Counter runs 0-4095
	wire [9:0]  inj_tbin_adr;	// Injector adr runs 0-1023

	always @(posedge clock) begin
	if		(inj_sm==pass     ) inj_tbin_cnt = 0;				// Sync  load
	else if	(inj_sm==injecting) inj_tbin_cnt = inj_tbin_cnt+1;	// Sync  count
	end

	assign inj_tbin_cnt_done = (inj_tbin_cnt==inj_last_tbin);	// Counter may wrap past 1024 ram adr limit
	assign inj_tbin_adr[9:0] = inj_tbin_cnt[9:0];				// injector ram address confined to 0-1023

// Pass state FF delays output mux 1 cycle
	reg pass_ff;

	always @(posedge clock or posedge sm_reset) begin
	if(sm_reset) pass_ff = 1'b1;
	else
	pass_ff = (inj_sm == pass);
	end

// Injector RAM: RPC Pads, 16 bits x 1024 tbins deep PortA: read/write via VME, PortB: read via injector SM
	wire [18:0] rpc_inj	   [1:0];
	wire [18:0]	inj_rdataa [1:0];

	genvar i;
	generate
	for (i=0; i<=1; i=i+1) begin: ram
	RAMB16_S18_S18 #(
	.WRITE_MODE_A		 ("WRITE_FIRST"),		// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.WRITE_MODE_B		 ("WRITE_FIRST"),		// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.SIM_COLLISION_CHECK ("GENERATE_X_ONLY")	// "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
	) uinjpads (
		.WEA	(inj_wen[i]),					// Port A Write Enable Input
		.ENA	(1'b1),							// Port A RAM Enable Input
		.SSRA	(1'b0),							// Port A Synchronous Set/Reset Input
		.CLKA	(clock),						// Port A Clock
		.ADDRA	(inj_rwadr[9:0]),				// Port A 10-bit Address Input
		.DIA	(inj_wdata[15:0]),				// Port A 16-bit Data Input
		.DIPA	(2'b00),						// Port A 2-bit parity Input
		.DOA	(inj_rdataa[i][15:0]),			// Port A 16-bit Data Output
		.DOPA	(),								// Port A 2-bit Parity Output

		.WEB	(1'b0),							// Port B Write Enable Input
		.ENB	(1'b1),							// Port B RAM Enable Input
		.SSRB	(1'b0),							// Port B Synchronous Set/Reset Input
		.CLKB	(clock),						// Port B Clock
		.ADDRB	(inj_tbin_adr[9:0]),			// Port B 10-bit Address Input
		.DIB	({16{1'b0}}),					// Port B 16-bit Data Input
		.DIPB	(2'b00),						// Port B 2-bit parity Input
		.DOB	(rpc_inj[i][15:0]),				// Port B 16-bit Data Output
		.DOPB	());
	end
	endgenerate

// Initialize Pad Injector RAMs, INIT values contain preset test pattern
// Tbin                         	       FFFFEEEEDDDDCCCCBBBBAAAA9999888877776666555544443333222211110000;
	defparam ram[0].uinjpads.INIT_00 =256'h000F000E000D000C000B000A00090008000700060005000400030002EFABABCD;
	defparam ram[1].uinjpads.INIT_00 =256'h432f432E432D432C432B432A432943284327432643254324432343224321CCCC;

// Injector RAM: RPC BXN, 16 bits x 1024 tbins deep PortA: read/write via VME, PortB: read via injector SM
	wire [7:0]	injbxn_rdataa;
	wire [7:0]	bxn_inj [1:0];

	wire wen_bxn = |(inj_wen);
	wire bank01	= (inj_wen[1] | inj_ren[1]);		// bank=0 for inj0, bank=1 for inj1

	RAMB16_S9_S18 #(
	.WRITE_MODE_A		 ("WRITE_FIRST"),			// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.WRITE_MODE_B		 ("WRITE_FIRST"),			// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.SIM_COLLISION_CHECK ("GENERATE_X_ONLY")		// "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
	) uinjbxn (
		.WEA	(wen_bxn),							// Port A Write Enable Input
		.ENA	(1'b1),								// Port A RAM Enable Input
		.SSRA	(1'b0),								// Port A Synchronous Set/Reset Input
		.CLKA	(clock),							// Port A Clock
		.ADDRA	({inj_rwadr[9:0],bank01}),			// Port A 11-bit Address Input
		.DIA	({5'b00000,inj_wdata[18:16]}),		// Port A 8-bit Data Input
		.DIPA	(1'b0),								// Port A 1-bit parity Input
		.DOA	(injbxn_rdataa[7:0]),				// Port A 8-bit Data Output
		.DOPA	(),									// Port A 1-bit Parity Output

		.WEB	(1'b0),								// Port B Write Enable Input
		.ENB	(1'b1),								// Port B RAM Enable Input
		.SSRB	(1'b0),								// Port B Synchronous Set/Reset Input
		.CLKB	(clock),							// Port B Clock
		.ADDRB	(inj_tbin_adr[9:0]),				// Port B 10-bit Address Input
		.DIB	({16{1'b0}}),						// Port B 16-bit Data Input
		.DIPB	(2'b00),							// Port B 2-bit parity Input
		.DOB	({bxn_inj[1][7:0],bxn_inj[0][7:0]}),// Port B 16-bit Data Output
		.DOPB	());

// Initialize BXN Injector RAM, INIT values contain preset test pattern
	defparam uinjbxn.INIT_00 =256'h432f432E432D432C432B432A432943284327432643254324432343224321ABCD;
	defparam uinjbxn.INIT_01 =256'h432f432E432D432C432B432A432943284327432643254324432343224321DDDD;

// Multiplex Injector RAM output data
	reg [18:0] inj_rdata;

	assign inj_rdataa[0][18:16] = injbxn_rdataa[2:0];
	assign inj_rdataa[1][18:16] = injbxn_rdataa[2:0];
	
	always @(inj_rdataa[0]or inj_ren) begin
	case (inj_ren[1:0])
	2'b01:	inj_rdata <= inj_rdataa[0];
	2'b10:	inj_rdata <= inj_rdataa[1];
	default	inj_rdata <= inj_rdataa[0];
	endcase
	end
	
// Multiplex RPC hits with injector RAM data
	wire [MXRPCDB-1:0] rpc_raw [1:0];

	assign rpc_inj[0][18:16] = bxn_inj[0][2:0];
	assign rpc_inj[1][18:16] = bxn_inj[1][2:0];

	assign rpc_raw[0][18:0] = (pass_ff) ? rpc_rxd[0][18:0] : rpc_inj[0][18:0];
	assign rpc_raw[1][18:0] = (pass_ff) ? rpc_rxd[1][18:0] : rpc_inj[1][18:0];

//------------------------------------------------------------------------------------------------------------------
// RPC Raw Hits Storage:
//------------------------------------------------------------------------------------------------------------------
// Store RPC Pads and BXNs in 2D array for raw hits ram generator
	wire [RAM_WIDTH-1:0] fifo_wdata [4:0];

// Multplex RPC data with data=address test mode, insert clct pre-trigger flag
	wire [15:0] tdata = fifo_wadr;	// data=address, leading 0s padded
	wire flag=clct_pretrig;			// clct pre-trigger marker

	assign fifo_wdata[0] = (test) ? tdata[ 7:0] : rpc_raw[0][7:0];								// RPC 0 lsbs
	assign fifo_wdata[1] = (test) ? tdata[15:8] : rpc_raw[0][15:8];								// RPC 0 msbs
	assign fifo_wdata[2] = (test) ? tdata[ 7:0] : rpc_raw[1][7:0];								// RPC 1 lsbs
	assign fifo_wdata[3] = (test) ? tdata[15:8] : rpc_raw[1][15:8];								// RPC 1 msbs
	assign fifo_wdata[4] = (test) ? tdata[ 7:0] : {flag,rpc_raw[1][18:16],flag,rpc_raw[0][18:16]};// RPC 1, rpc0 3-bit bxn

// Calculate parity for raw hits RAM write data
	wire [4:0] parity_wr;
	wire [4:0] parity_rd;

	assign parity_wr[0] = ~(^ fifo_wdata[0][RAM_WIDTH-1:0]);
	assign parity_wr[1] = ~(^ fifo_wdata[1][RAM_WIDTH-1:0]);
	assign parity_wr[2] = ~(^ fifo_wdata[2][RAM_WIDTH-1:0]);
	assign parity_wr[3] = ~(^ fifo_wdata[3][RAM_WIDTH-1:0]);
	assign parity_wr[4] = ~(^ fifo_wdata[4][RAM_WIDTH-1:0]);

// Store RPC Pads and BXNs in Raw Hits FIFO RAM, 8 bits wide + 1 parity x 2048 tbins deep, write port A, read port B
	wire [RAM_WIDTH-1:0] fifo_rdata [4:0];
	wire [4:0] dopa;

	generate
	for (i=0; i<=4; i=i+1) begin: raw
	RAMB16_S9_S9 #(
	.WRITE_MODE_A			("READ_FIRST"),		// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.WRITE_MODE_B			("READ_FIRST"),		// WRITE_FIRST, READ_FIRST or NO_CHANGE
	.SIM_COLLISION_CHECK	("GENERATE_X_ONLY")	// "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
	) rawhits_ram (
	.WEA			(fifo_wen),					// Port A Write Enable Input
	.ENA			(1'b1),						// Port A RAM Enable Input
	.SSRA			(1'b0),						// Port A Synchronous Set/Reset Input
	.CLKA			(clock),					// Port A Clock
	.ADDRA			(fifo_wadr),				// Port A 11-bit Address Input
	.DIA			(fifo_wdata[i]),			// Port A 8-bit Data Input
	.DIPA			(parity_wr[i]),				// Port A 1-bit parity Input
	.DOA			(),							// Port A 8-bit Data Output
	.DOPA			(dopa[i]),					// Port A 1-bit Parity Output

	.WEB			(1'b0),						// Port B Write Enable Input
	.ENB			(1'b1),						// Port B RAM Enable Input
	.SSRB			(1'b0),						// Port B Synchronous Set/Reset Input
	.CLKB			(clock),					// Port B Clock
	.ADDRB			(fifo_radr),				// Port B 11-bit Address Input
	.DIB			(8'h00),					// Port B 8-bit Data Input
	.DIPB			(),							// Port-B 1-bit parity Input
	.DOB			(fifo_rdata[i]),			// Port B 8-bit Data Output
	.DOPB			(parity_rd[i])				// Port B 1-bit Parity Output
   );
	end
	endgenerate

// Map read data arrays
	wire [RAM_WIDTH-1:0] fifo_rdata_rpc0a = fifo_rdata[0];
	wire [RAM_WIDTH-1:0] fifo_rdata_rpc0b = fifo_rdata[1];
	wire [RAM_WIDTH-1:0] fifo_rdata_rpc1a = fifo_rdata[2];
	wire [RAM_WIDTH-1:0] fifo_rdata_rpc1b = fifo_rdata[3];
	wire [RAM_WIDTH-1:0] fifo_rdata_bxn   = fifo_rdata[4];

// Compare read parity to write parity
	wire [4:0] parity_expect;

	assign parity_expect[0] = ~(^ fifo_rdata[0][RAM_WIDTH-1:0]);
	assign parity_expect[1] = ~(^ fifo_rdata[1][RAM_WIDTH-1:0]);
	assign parity_expect[2] = ~(^ fifo_rdata[2][RAM_WIDTH-1:0]);
	assign parity_expect[3] = ~(^ fifo_rdata[3][RAM_WIDTH-1:0]);
	assign parity_expect[4] = ~(^ fifo_rdata[4][RAM_WIDTH-1:0]);

	assign parity_err_rpc[4:0] =  ~(parity_rd ~^ parity_expect);	// ~^ is bitwise equivalence operator

// Multiplex Raw Hits FIFO RAM output data
	reg [RAM_WIDTH-1+4:0] fifo0_rdata;
	reg	[RAM_WIDTH-1+4:0] fifo1_rdata;

	wire [2:0] rpc0_bxn  = fifo_rdata_bxn[3:0];
	wire [2:0] rpc1_bxn  = fifo_rdata_bxn[7:4];

	wire       rpc0_flag = fifo_rdata_bxn[3];
	wire       rpc1_flag = fifo_rdata_bxn[7];

	always @* begin
	case (fifo_sel[0:0])
	1'h0:	fifo0_rdata <= {rpc0_flag,rpc0_bxn,fifo_rdata_rpc0a};	// slice 0 rpc0 flag, bxn[2:0],pads[7:0]
	1'h1:	fifo0_rdata <= {rpc0_flag,rpc0_bxn,fifo_rdata_rpc0b};	// slice 1 rpc0 flag, bxn[2:0],pads[15:8]
	endcase
	end

	always @* begin
	case (fifo_sel[0:0])
	1'h0:	fifo1_rdata <= {rpc1_flag,rpc1_bxn,fifo_rdata_rpc1a};	// slice 0 rpc1 flag, bxn[2:0],pads[7:0]
	1'h1:	fifo1_rdata <= {rpc1_flag,rpc1_bxn,fifo_rdata_rpc1b};	// slice 1 rpc1 flag, bxn[2:0],pads[15:8]
	endcase
	end

//------------------------------------------------------------------------------------------------------------------
// RPC raw hits VME readback:
//------------------------------------------------------------------------------------------------------------------
// Buffer RPC data for local processing, raw hits ram gets unbuffered data
	reg [MXRPCDB-1:0] rpc_rxdff [1:0];

	always @(posedge clock) begin
	rpc_rxdff[0] <= rpc_raw[0];
	rpc_rxdff[1] <= rpc_raw[1];
	end

// Latch RPC data for VME readback, used for 80MHz demux synchronization
	reg [MXRPCDB-1:0] rdata;
	
	always @(posedge clock) begin
	case (rpc_bank)
	1'h0:	rdata[18:0] <= rpc_rxdff[0][18:0];
	1'h1:	rdata[18:0] <= rpc_rxdff[1][18:0];
	endcase
	end

	assign rpc_rdata[15:0]	= rdata[15:0];
	assign rpc_rbxn[2:0]	= rdata[18:16];

//------------------------------------------------------------------------------------------------------------------
// RPC Hits Processing:
//------------------------------------------------------------------------------------------------------------------
// Apply Hot Channel Mask to block errant RPC pads: 1=enable pad
	wire [MXRPCPAD-1:0]	rpc_rxm [1:0];

	assign rpc_rxm[0] = rpc_rxdff[0] & rpc0_hcm;
	assign rpc_rxm[1] = rpc_rxdff[1] & rpc1_hcm;
	
// Count pad hits
	reg [3:0] rpc_nhits [1:0];

	always @(posedge clock) begin
	rpc_nhits[0] <= rcount1s(rpc_rxm[0]);
	rpc_nhits[1] <= rcount1s(rpc_rxm[1]);
	end
	
// Prodcedural function, sums number 1-bits into a binary pattern number
	function [3:0]	rcount1s;
	input	 [15:0]	rinp;

	rcount1s =	((rinp[ 3]+rinp[ 2])+(rinp[ 1]+rinp[ 0]))+
				((rinp[ 7]+rinp[ 6])+(rinp[ 5]+rinp[ 4]))+
				((rinp[11]+rinp[10])+(rinp[ 9]+rinp[ 8]))+
				((rinp[15]+rinp[14])+(rinp[13]+rinp[12]));
	endfunction

// BXN offsets
	reg [3:0] rpc0_bxn_diff;
	reg [3:0] rpc1_bxn_diff;

	always @(posedge clock) begin
	rpc0_bxn_diff = rpc_rxdff[0][18:16] - rpc_bxn_offset;
	rpc1_bxn_diff = rpc_rxdff[1][18:16] - rpc_bxn_offset;
	end

//------------------------------------------------------------------------------------------------------------------
// RPC raw hits scope signals:
//------------------------------------------------------------------------------------------------------------------
	assign scp_rpc0_bxn[2:0] = rpc_rxdff[0][18:16];
	assign scp_rpc1_bxn[2:0] = rpc_rxdff[1][18:16];

	assign scp_rpc0_nhits[3:0] = rpc_nhits[0][3:0];
	assign scp_rpc1_nhits[3:0] = rpc_nhits[1][3:0];

// Sump
	assign rpc_sump =		
	(|injbxn_rdataa[7:3]) | (|dopa)	|
	(|bxn_inj[0][7:3])    | (|bxn_inj[1][7:3])
	;

//------------------------------------------------------------------------------------------------------------------
// Debug
//------------------------------------------------------------------------------------------------------------------
`ifdef DEBUG_RPC
// Injector State Machine ASCII display
	reg[71:0] inj_sm_dsp;
	always @* begin
	case (inj_sm)
	pass:		inj_sm_dsp <= "pass     ";
	injecting:	inj_sm_dsp <= "injecting";
	default		inj_sm_dsp <= "pass     ";
	endcase
	end
`endif

//------------------------------------------------------------------------------------------------------------------
	endmodule
//------------------------------------------------------------------------------------------------------------------
