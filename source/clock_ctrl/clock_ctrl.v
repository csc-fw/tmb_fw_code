`timescale 1ns / 1ps
//`define DEBUG_CLOCK 1
//---------------------------------------------------------------------------------------------------------------------
//	Virtex2 Clock DLLs
//---------------------------------------------------------------------------------------------------------------------
//	09/30/03 Initial for Virtex2
//	12/11/03 Add global reset
//	09/12/06 Mods for xst
//	09/13/06 Rename rpc_rxalt1 to rpc_dsn
//	09/15/06 Bitgen demands clk_feeback set to 2x
//	10/10/06 Change bufg to bufgmux for virtex2
//	10/10/06 Remove clock_2x, clock_alct_2x
//	05/10/07 Put clock_2x back for new pattern finder mux
//	06/13/07 Add multiplexer output signal for pattern finder
//	06/14/07 Change logic accessible clock input to dcm 90 degree signal
//	05/14/08 Mod global_reset to latch off once DLL locks
//	05/23/08 Mod global_reset to pulse when DLL re-locks, latch lock loss for sync_err
//	06/02/08 Add enable for global_reset on lock-loss
//	03/30/09 Add programmable 1/2 cycle shift for alct inter-stage
//	03/31/09 Tried 3/4 cycle shift
//	04/02/09 No improvement, so reverted to 1/2-cycle
//	05/27/09 Add tmb_rx_clock for incoming alct data with muonic timing
//	06/05/09 Remove non-muonic clock mux to see if it improves good spots
//	06/10/09 Convert to digital phase shifting for alct txd and rxd clocks, totally awesome
//	06/11/09 Change psclk to run continuously from tmb main, add phaser reset
//	06/12/09 Remove interstage clock muxes, do it with lac instead, add dcm reset for phaser
//	06/12/09 Change to mux method for txd rxd because phase amplification idea didnt work
//	06/15/09 Change to -32 to +31 phase range
//	06/16/09 Remove digital phase shifter
//	06/18/09 Add cfeb muonic dcm
//	06/24/09 Put back alct digital phase shifters, add cfeb digital phase shifters
//	06/25/09 Switch from global buffers to FFs for quadrant shifting
//	06/29/09 Remove digital phase shifters for cfebs, certain cfeb IOBs can not have 2 clock domains
//	07/10/09 Return digital phase shifters for cfebs, mod ucf to move 5 IOB DDRs to fabric
//	07/16/09 Add global buffers to phase shifter clocks, constrain dcm and gbuf locs
//	07/17/09 Reassign global buffer and dcm locs per dcm_locator Monte Carlo
//	07/21/09 New gbuf and dcm locs, remove gbufs for alct, add gbufs for cfebs, remove vme gbuf
//	07/22/09 Remove clock_vme global net to make room for cfeb digital phase shifter gbufs
//	08/04/09 Add location constraints in ucf for phase shifter mux components
//	08/04/09 Replace digital phase shifter multiplexer code with submodules
//	08/07/09 Relax digital phase shifter quadrant selects maxdelay
//	08/07/09 Remove gbufs for cfeb phase shifted clocks, put vme clock back in
//	08/10/09 Add ffs in clock_mux to transfer dc quadrant selects into 160mhz time domain
//	08/10/09 Swap 2x and vme global buffers to match older versions
//	08/10/09 Add ffs to transfer dc quadrant selects into 40mhz time domain before sending to clock_mux
//	09/11/09 Add quadrant update linked to fire stobe
//	09/18/09 Relocate dcms to shorten clock paths
//	04/27/10 Allow ttc_resync to clear clock_lock_lost
//	05/12/10 Move clock_lock_lost ff to sync_err module, remove ttc_resync
//	11/08/10 Rename module for virtex 4/6
//	11/18/10 Convert phase shifter VME signals to arrays
//	11/18/10 Conform dps order to match VME register addresses
//	12/01/10 Change bufg instance names, somehow xst sees both virtex2 and virtex6 meta comments
//	12/02/10 Remove virtex2 bufg locs
//	12/09/10 Move bufg and dcm LOCs to ucf, xst fails to attach attributes to generated instances :(
//------------------------------------------------------------------------------------------------------------------
	module clock_ctrl
	(
// Clock inputs
	tmb_clock0,
	tmb_clock0d,
	tmb_clock1,
	alct_rxclock,
	alct_rxclockd,
	mpc_clock,
	dcc_clock,
	rpc_sig,

// Main clock outputs
	clock,
	clock_2x,
	clock_vme,
	clock_lac,

// Phase delayed clocks
	clock_alct_rxd,
	clock_alct_txd,
	clock_cfeb0_rxd,
	clock_cfeb1_rxd,
	clock_cfeb2_rxd,
	clock_cfeb3_rxd,
	clock_cfeb4_rxd,

// Global reset
	global_reset_en,
	global_reset,
	clock_lock_lost_err,

// Clock DCM lock status
	lock_tmb_clock0,
	lock_tmb_clock0d,
	lock_alct_rxclockd,
	lock_mpc_clock,
	lock_dcc_clock,
	lock_rpc_rxalt1,
	lock_tmb_clock1,
	lock_alct_rxclock,
	clock_ctrl_sump,

// Phaser VME control/status ports
	dps_fire,
	dps_reset,
	dps_busy,
	dps_lock,

	dps0_phase,
	dps1_phase,
	dps2_phase,
	dps3_phase,
	dps4_phase,
	dps5_phase,
	dps6_phase,

	dps0_sm_vec,
	dps1_sm_vec,
	dps2_sm_vec,
	dps3_sm_vec,
	dps4_sm_vec,
	dps5_sm_vec,
	dps6_sm_vec

// Debug
`ifdef DEBUG_CLOCK
	,dcm_reset
	,psen_alct_rxd
	,psincdec_alct_rxd
	,psdone_alct_rxd
`endif
	);

//------------------------------------------------------------------------------------------------------------------
// Ports
//------------------------------------------------------------------------------------------------------------------
// Clock inputs
	input			tmb_clock0;				// 40MHz clock bypasses 3D3444 and loads Mez PROMs, chip bottom
	input			tmb_clock0d;			// 40MHz clock bypasses 3D3444 and loads Mez PROMs, chip top
	input			tmb_clock1;				// 40MHz clock with 3D3444 delay
	input			alct_rxclock;			// 40MHz ALCT receive data clock with 3D3444 delay, chip bottom
	input			alct_rxclockd;			// 40MHz ALCT receive data clock with 3D3444 delay, chip top
	input			mpc_clock;				// 40MHz MPC clock
	input			dcc_clock;				// 40MHz Duty cycle corrected clock with 3D3444 delay
	input			rpc_sig;				// 40MHz Unused

// Main clock outputs
	output			clock;					// 40MHz global TMB clock 1x
	output			clock_2x;				// 80MHz global TMB clock 2x
	output			clock_vme;				// 10MHz global VME clock 1/4x
	output			clock_lac;				// 40MHz logic accessible clock

// Phase delayed clocks
	output			clock_alct_rxd;			// 40MHz ALCT receive  data clock 1x
	output			clock_alct_txd;			// 40MHz ALCT transmit data clock 1x
	output			clock_cfeb0_rxd;		// 40MHz CFEB receive  data clock 1x
	output			clock_cfeb1_rxd;		// 40MHz CFEB receive  data clock 1x
	output			clock_cfeb2_rxd;		// 40MHz CFEB receive  data clock 1x
	output			clock_cfeb3_rxd;		// 40MHz CFEB receive  data clock 1x
	output			clock_cfeb4_rxd;		// 40MHz CFEB receive  data clock 1x

// Global reset
	input			global_reset_en;		// Enable global reset on lock_lost
	output			global_reset;			// Global reset, asserted until main DLL locks
	output			clock_lock_lost_err;	// 40MHz main clock lost lock FF

// Clock DCM lock status
	output			lock_tmb_clock0;		// DCM lock status
	output			lock_tmb_clock0d;		// DCM lock status
	output			lock_alct_rxclockd;		// DCM lock status
	output			lock_mpc_clock;			// DCM lock status
	output			lock_dcc_clock;			// DCM lock status
	output			lock_rpc_rxalt1;		// DCM lock status
	output			lock_tmb_clock1;		// DCM lock status
	output			lock_alct_rxclock;		// DCM lock status
	output			clock_ctrl_sump;		// Clock section unused signals

// Phaser VME control/status ports
	input	[6:0]	dps_fire;				// Set new phase
	input	[6:0]	dps_reset;				// VME Reset current phase
	output	[6:0]	dps_busy;				// Phase shifter busy
	output	[6:0]	dps_lock;				// DCM lock status

	input	[7:0]	dps0_phase;				// Phase to set, 0-255
	input	[7:0]	dps1_phase;				// Phase to set, 0-255
	input	[7:0]	dps2_phase;				// Phase to set, 0-255
	input	[7:0]	dps3_phase;				// Phase to set, 0-255
	input	[7:0]	dps4_phase;				// Phase to set, 0-255
	input	[7:0]	dps5_phase;				// Phase to set, 0-255
	input	[7:0]	dps6_phase;				// Phase to set, 0-255

	output	[2:0]	dps0_sm_vec;			// Phase shifter machine state
	output	[2:0]	dps1_sm_vec;			// Phase shifter machine state
	output	[2:0]	dps2_sm_vec;			// Phase shifter machine state
	output	[2:0]	dps3_sm_vec;			// Phase shifter machine state
	output	[2:0]	dps4_sm_vec;			// Phase shifter machine state
	output	[2:0]	dps5_sm_vec;			// Phase shifter machine state
	output	[2:0]	dps6_sm_vec;			// Phase shifter machine state

// Debug
`ifdef DEBUG_CLOCK
	input			dcm_reset;				// DCM reset input for simulation, assert for at least 3 cycles
	output			psen_alct_rxd;			// Dps phase shift enable
	output			psincdec_alct_rxd;		// Dps phase increment/decrement
	output			psdone_alct_rxd;		// Dps done
`else
	wire			dcm_reset = 1'b0;		// DCM reset hard-wired to 0 normally
`endif

//-------------------------------------------------------------------------------------------------------------------
// Clock DCM Assignments
//
//	Signal			Pin		Pad		DCM		BUFG	3D3444 Channel	Usage
//	-------------	---		------	----	------	--------------	--------------------------------
//	alct_rxclock	AK17	GCLK0P					Chip 0 Ch 1		ALCT receive, chip bottom
//	alct_rxclockd	H17		GCLK1P					Chip 0 Ch 1		ALCT receive, chip top
//	dcc_clock		AG17	GCLK2P					Chip 1 Ch 2		Duty cycle corrected
//	mpc_clock		E17		GCLK3P					Chip 1 Ch 1		MPC
//	tmb_clock0		AF18	GCLK4P					None			TMB main, chip bottom
//	tmb_clock0d		E19		GCLK5P					None			TMB main, chip top
//	tmb_clock1		AK19	GCLK6P					Chip 1 Ch 0		TMB delayed
//	rpc_dsn			K18		GCLK7P					None			RPC rpc_dsn shared with rat 3d3444 verify
//
//-------------------------------------------------------------------------------------------------------------------
//	Virtex2 DCM global clock buffer rules:
// 
//	A DCM may drive four global clock buffers on the edge (top or bottom) where
//	it exists using the optimal dedicated routing resources.
//	AND
//	A DCM may only drive one of buffer of an exclusive site pair:
//	Top:
//	  (BUFGMUX7P, BUFGMUX3P)
//	  (BUFGMUX6S, BUFGMUX2S)
//	  (BUFGMUX5P, BUFGMUX1P)
//	  (BUFGMUX4S, BUFGMUX0S)
//	Bottom:
//	  (BUFGMUX7S, BUFGMUX3S)
//	  (BUFGMUX6P, BUFGMUX2P)
//	  (BUFGMUX5S, BUFGMUX1S)
//	  (BUFGMUX4P, BUFGMUX0P)
//------------------------------------------------------------------------------------------------------------------
// Global clock input buffers
//------------------------------------------------------------------------------------------------------------------
	IBUFG uibufg_0p (.I(alct_rxclock ), .O(alct_rxclock_ibufg ));	// synthesis attribute LOC of uibufg_0p is "AK17";
	IBUFG uibufg_1p (.I(alct_rxclockd), .O(alct_rxclockd_ibufg));	// synthesis attribute LOC of uibufg_1p is "H17";	
	IBUFG uibufg_2p (.I(dcc_clock    ), .O(dcc_clock_ibufg    ));	// synthesis attribute LOC of uibufg_2p is "AG17";
	IBUFG uibufg_3p (.I(mpc_clock    ), .O(mpc_clock_ibufg    ));	// synthesis attribute LOC of uibufg_3p is "E17";
	IBUFG uibufg_4p (.I(tmb_clock0   ), .O(tmb_clock0_ibufg   ));	// synthesis attribute LOC of uibufg_4p is "AF18";
	IBUFG uibufg_5p (.I(tmb_clock0d  ), .O(tmb_clock0d_ibufg  ));	// synthesis attribute LOC of uibufg_5p is "E19";
	IBUFG uibufg_6p (.I(tmb_clock1   ), .O(tmb_clock1_ibufg   ));	// synthesis attribute LOC of uibufg_6p is "AK19";
//	IBUFG uibufg_7p (.I(rpc_sig      ), .O(rpc_sig_ibufg      ));	//xsynthesis attribute LOC of uibufg_7p is "K18";

//------------------------------------------------------------------------------------------------------------------
// Main TMB DLL global clock output buffers
//------------------------------------------------------------------------------------------------------------------
// Main DLL global buffers FPGA bottom edge
	BUFG ubufg_dll_1x  (.I(clock_dcm    ), .O(clock    ));			// synthesis attribute LOC of ubufg_dll_1x  is "BUFGMUX0P"
	BUFG ubufg_dll_vme (.I(clock_vme_dcm), .O(clock_vme));			// synthesis attribute LOC of ubufg_dll_vme is "BUFGMUX1S"
	BUFG ubufg_dll_2x  (.I(clock_2x_dcm ), .O(clock_2x ));			// synthesis attribute LOC of ubufg_dll_2x  is "BUFGMUX2P"

// Phaser DLL feedback and fanout buffers: FPGA bottom edge, nb attributes fail to attach to generated instances, ucf has real assigments
	//xsynthesis attribute LOC of dps[0].ubufg_fb is "BUFGMUX6P";	// alct_rxd
	//xsynthesis attribute LOC of dps[1].ubufg_fb is "BUFGMUX7S";	// alct_txd

// Phaser DLL feedback and fanout buffers: FPGA top edge
	//xsynthesis attribute LOC of dps[2].ubufg_fb is "BUFGMUX3P";	// cfeb0_rxd
	//xsynthesis attribute LOC of dps[3].ubufg_fb is "BUFGMUX4S";	// cfeb1_rxd
	//xsynthesis attribute LOC of dps[4].ubufg_fb is "BUFGMUX5P";	// cfeb2_rxd
	//xsynthesis attribute LOC of dps[5].ubufg_fb is "BUFGMUX6S";	// cfeb3_rxd
	//xsynthesis attribute LOC of dps[6].ubufg_fb is "BUFGMUX7P";	// cfeb4_rxd

// DCM locations bottom edge
	//xsynthesis attribute LOC of udcm_main        is "DCM_X2Y0";
	//xsynthesis attribute LOC of dps[0].udcm_dps  is "DCM_X1Y0";	// alct_rxd
	//xsynthesis attribute LOC of dps[1].udcm_dps  is "DCM_X0Y0";	// alct_txd

// DCM locations top edge
	//xsynthesis attribute LOC of dps[2].udcm_dps  is "DCM_X5Y1";	// cfeb0_rxd 
	//xsynthesis attribute LOC of dps[3].udcm_dps  is "DCM_X4Y1";	// cfeb1_rxd 
	//xsynthesis attribute LOC of dps[4].udcm_dps  is "DCM_X3Y1";	// cfeb2_rxd 
	//xsynthesis attribute LOC of dps[5].udcm_dps  is "DCM_X1Y1";	// cfeb3_rxd 
	//xsynthesis attribute LOC of dps[6].udcm_dps  is "DCM_X0Y1";	// cfeb4_rxd 

//------------------------------------------------------------------------------------------------------------------
// Main TMB DLL generates clocks at 1x=40MHz, 2x=80MHz, 1/4x=10MHz
//------------------------------------------------------------------------------------------------------------------
	DCM # (
	.CLKDV_DIVIDE		(4.0), 					// Divide by: 1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,6.5,7.0,7.5,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0 or 16.0
	.CLKIN_PERIOD		(25.0),					// Specify period of input clock in ns
	.CLKOUT_PHASE_SHIFT	("NONE"),				// Specify phase shift of NONE, FIXED or VARIABLE
	.CLK_FEEDBACK		("2X"),					// Specify clock feedback of NONE, 1X or 2X
	.PHASE_SHIFT		(0),					// Amount of fixed phase shift from -255 to 255
	.STARTUP_WAIT		("TRUE")				// Delay configuration DONE until DCM LOCK, TRUE/FALSE

	) udcm_main (

	.CLKIN				(tmb_clock0_ibufg),		// In	Clock input (from IBUFG, BUFG or DCM)
	.CLKFB				(clock_2x),				// In	DCM clock feedback
	.RST				(dcm_reset),			// In	DCM asynchronous reset
	.LOCKED				(lock_tmb_clock0),		// Out	DCM LOCK status
	.STATUS				(),						// Out	8-bit DCM status [0]=phase shift overflow,[1]=clkin stopped,[2]=clkfx stopped
	.DSSEN				(),						// In	Not used
	.CLK0				(clock_dcm),			// Out	0   degree DCM CLK
	.CLK90				(clock_dcm_90),			// Out	90 degree DCM CLK
	.CLK180				(),						// Out	180 degree DCM CLK
	.CLK270				(),						// Out	270 degree DCM CLK
	.CLK2X				(clock_2x_dcm),			// Out	2X DCM CLK
	.CLK2X180			(),						// Out	2X, 180 degree DCM CLK
	.CLKDV				(clock_vme_dcm),		// Out	Divided DCM CLK   (CLKDV_DIVIDE)
	.CLKFX				(),						// Out	DCM CLK synthesis (M/D)
	.CLKFX180			(),						// Out	180 degree CLK synthesis
	.PSCLK				(1'b0),					// In	Dynamic phase adjust clock
	.PSEN				(1'b0),					// In	Dynamic phase adjust enable
	.PSINCDEC			(1'b0),					// In	Dynamic phase adjust increment/decrement
	.PSDONE				()						// Out	Dynamic phase adjust done
	);

//------------------------------------------------------------------------------------------------------------------
// Logic Accessible Clock copy of 40MHz main clock, synchronized to clock_2x global net
//------------------------------------------------------------------------------------------------------------------
	FDRSE #(.INIT(1'b0)) ulac (
	.Q	(clock_lac),							// Out	Data
	.C	(clock_2x),								// In	Clock
	.CE	(lock_tmb_clock0),						// In	Clock enable
	.D	(!clock_dcm_90),						// In	Data
	.R	(1'b0),									// In	Synchronous reset
	.S	(1'b0));								// In	Synchronous set

//------------------------------------------------------------------------------------------------------------------
// Generate Digital Phase Shifters
//------------------------------------------------------------------------------------------------------------------
// Map ports for generated instances
	wire [6:0]	dps_clock_fbi;
	wire [6:0]	dps_clock;
	wire [6:0]	dps_clock_4x;

	wire [6:0]	dps_clk0;
	wire [6:0]	dps_clk90;
	wire [6:0]	dps_clk180;
	wire [6:0]	dps_clk270;

	wire [6:0]	dps_psen;
	wire [6:0]	dps_psincdec;
	wire [6:0]	dps_psdone;
	wire [6:0]	dps_update_quad;

	wire [7:0]	dps_phase  [6:0];
	wire [2:0]	dps_sm_vec [6:0];
	
	assign dps_phase[0][7:0] = dps0_phase[7:0];
	assign dps_phase[1][7:0] = dps1_phase[7:0];
	assign dps_phase[2][7:0] = dps2_phase[7:0];
	assign dps_phase[3][7:0] = dps3_phase[7:0];
	assign dps_phase[4][7:0] = dps4_phase[7:0];
	assign dps_phase[5][7:0] = dps5_phase[7:0];
	assign dps_phase[6][7:0] = dps6_phase[7:0];

	assign dps0_sm_vec[2:0]  = dps_sm_vec[0][2:0];
	assign dps1_sm_vec[2:0]  = dps_sm_vec[1][2:0];
	assign dps2_sm_vec[2:0]  = dps_sm_vec[2][2:0];
	assign dps3_sm_vec[2:0]  = dps_sm_vec[3][2:0];
	assign dps4_sm_vec[2:0]  = dps_sm_vec[4][2:0];
	assign dps5_sm_vec[2:0]  = dps_sm_vec[5][2:0];
	assign dps6_sm_vec[2:0]  = dps_sm_vec[6][2:0];

	assign clock_alct_rxd    = dps_clock[0];
	assign clock_alct_txd    = dps_clock[1];
	assign clock_cfeb0_rxd   = dps_clock[2];
	assign clock_cfeb1_rxd   = dps_clock[3];
	assign clock_cfeb2_rxd   = dps_clock[4];
	assign clock_cfeb3_rxd   = dps_clock[5];
	assign clock_cfeb4_rxd   = dps_clock[6];

// DLL phase shifters
	reg [6:0] hcycle_gated = 0;
	reg [6:0] qcycle_gated = 0;
	reg [6:0] hcycle_ff    = 0;
	reg [6:0] qcycle_ff    = 0;

	genvar i;
	generate
	for (i=0; i<=6; i=i+1) begin: dps

	BUFG ubufg_fb (.I(dps_clk0[i]),.O(dps_clock_fbi[i]));

	DCM # (
	.CLKDV_DIVIDE		(4.0), 					// Divide by: 1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,6.5,7.0,7.5,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0 or 16.0
	.CLKFX_DIVIDE		(1),					// Can be any integer from 1 to 32
	.CLKFX_MULTIPLY		(4),					// Can be any integer from 2 to 32
 	.CLKIN_PERIOD		(25.0),					// Specify period of input clock in ns
	.CLKOUT_PHASE_SHIFT	("VARIABLE"),			// Specify phase shift of NONE, FIXED or VARIABLE
	.CLK_FEEDBACK		("1X"),					// Specify clock feedback of NONE, 1X or 2X
	.PHASE_SHIFT		(0),					// Amount of fixed phase shift from -255 to 255
	.STARTUP_WAIT		("FALSE")				// Delay configuration DONE until DCM LOCK, TRUE/FALSE

	) udcm_dps (

	.CLKIN				(clock),				// In	Clock input (from IBUFG, BUFG or DCM)
	.CLKFB				(dps_clock_fbi[i]),		// In	DCM clock feedback
	.RST				(dps_reset[i]),			// In	DCM asynchronous reset
	.LOCKED				(dps_lock[i]),			// Out	DCM LOCK status
	.STATUS				(),						// Out	8-bit DCM status [0]=phase shift overflow,[1]=clkin stopped,[2]=clkfx stopped
	.DSSEN				(),						// In	Not used
	.CLK0				(dps_clk0[i]),			// Out	0   degree DCM CLK
	.CLK90				(dps_clk90[i]),			// Out	90 degree DCM CLK
	.CLK180				(dps_clk180[i]),		// Out	180 degree DCM CLK
	.CLK270				(dps_clk270[i]),		// Out	270 degree DCM CLK
	.CLK2X				(),						// Out	2X DCM CLK
	.CLK2X180			(),						// Out	2X, 180 degree DCM CLK
	.CLKDV				(),						// Out	Divided DCM CLK   (CLKDV_DIVIDE)
	.CLKFX				(dps_clock_4x[i]),		// Out	DCM CLK synthesis (M/D)
	.CLKFX180			(),						// Out	180 degree CLK synthesis
	.PSCLK				(clock),				// In	Dynamic phase adjust clock
	.PSEN				(dps_psen[i]),			// In	Dynamic phase adjust enable
	.PSINCDEC			(dps_psincdec[i]),		// In	Dynamic phase adjust increment/decrement
	.PSDONE				(dps_psdone[i])			// Out	Dynamic phase adjust done
	);

// Update quadrant select latches when phasers fire
	always @(posedge clock) begin
	if (dps_update_quad[i]) begin
	hcycle_gated[i] <= dps_phase[i][7];
	qcycle_gated[i] <= dps_phase[i][6];
	end
	end

// Local copy of quadrant selects pushed into clock_mux slices
	always @(posedge clock) begin
	hcycle_ff[i] <= hcycle_gated[i];
	qcycle_ff[i] <= qcycle_gated[i];
	end

// Programmable 1/4, 1/2, 3/4 cycle phase shifts, location constraints are in ucf
	clock_mux uclock_mux
	(
	.clk4x			(dps_clock_4x[i]),			// In	4x clock
	.hcycle			(hcycle_ff[i]),				// In	Half    cycle select
	.qcycle			(qcycle_ff[i]),				// In	Quarter cycle select
	.clk0			(dps_clk0[i]),				// In	Clock 000 degrees
	.clk90			(dps_clk90[i]),				// In	Clock 090 degrees
	.clk180			(dps_clk180[i]),			// In	Clock 180 degrees
	.clk270			(dps_clk270[i]),			// In	Clock 270 degrees
	.clk			(dps_clock[i])				// Out	Quadrant shifted clock
	);

// Digital phase shifter state machine
	phaser uphaser 
	(
	.clock			(clock),					// In	40MHz global TMB clock 1x
	.global_reset	(global_reset),				// In	Global reset, asserted until main DLL locks
	.lock_tmb		(lock_tmb_clock0),			// In	Lock state for TMB main clock DLL
	.lock_dcm		(dps_lock[i]),				// In	Lock state for this DCM
	.psen			(dps_psen[i]),				// Out	Dps phase shift enable
	.psincdec		(dps_psincdec[i]),			// Out	Dps phase increment/decrement
	.psdone			(dps_psdone[i]),			// In	Dps done
	.fire			(dps_fire[i]),				// In	VME Set new phase
	.reset			(dps_reset[i]),				// In	Reset current phase
	.phase			(dps_phase[i][5:0]),		// In	VME Phase to set, 0-31
	.busy			(dps_busy[i]),				// Out	VME Phase shifter busy
	.dps_sm_vec		(dps_sm_vec[i][2:0]),		// Out	VME Phase shifter machine state
	.update_quad	(dps_update_quad[i])		// Out	Update quadrant select FFs
	);
	end
	endgenerate

//------------------------------------------------------------------------------------------------------------------
// Global reset for fpga-wide state machines, asserted until main DLL locks
//------------------------------------------------------------------------------------------------------------------
	reg global_reset    = 1;
	reg startup_done	= 0;

	wire first_lock = (lock_tmb_clock0 || startup_done);

	always @(posedge clock) begin
	startup_done	<=	first_lock;												// Latches  on 1st dll lock
	global_reset	<= !first_lock || (!lock_tmb_clock0 && global_reset_en);	// Re-fires on lock lost
	end

	assign clock_lock_lost_err = (!global_reset && !lock_tmb_clock0);			// Latches  on lock lost in sync module

//------------------------------------------------------------------------------------------------------------------
// Pseudo-locked status for unused clock inputs, DDR FFs q0+q1 should add to 1 if clock is running
//------------------------------------------------------------------------------------------------------------------
	reg	[1:0]	tmb_clock0d_q	= 0;
	reg	[1:0]	alct_rxclockd_q	= 0;
	reg	[1:0]	mpc_clock_q		= 0;
	reg	[1:0]	dcc_clock_q		= 0;
	reg	[1:0]	rpc_rxalt1_q	= 0;
	reg [1:0]	tmb_clock1_q	= 0;
	reg [1:0]	alct_rxclock_q	= 0;

	always @(posedge clock)	begin
	tmb_clock0d_q[0]	<= tmb_clock0d_ibufg && lock_tmb_clock0;	// Force ibuf insertion
	alct_rxclockd_q[0]	<= alct_rxclockd_ibufg;
	mpc_clock_q[0]		<= mpc_clock_ibufg;
	dcc_clock_q[0]		<= dcc_clock_ibufg;
	rpc_rxalt1_q[0]		<= rpc_sig;
	tmb_clock1_q[0]		<= tmb_clock1_ibufg;
	alct_rxclock_q[0]	<= alct_rxclock_ibufg;
	end

	always @(negedge clock) begin
	tmb_clock0d_q[1]	<= tmb_clock0d_ibufg && lock_tmb_clock0;	// Force ibuf insertion
	alct_rxclockd_q[1]	<= alct_rxclockd_ibufg;
	mpc_clock_q[1]		<= mpc_clock_ibufg;
	dcc_clock_q[1]		<= dcc_clock_ibufg;
	rpc_rxalt1_q[1]		<= rpc_sig;
	tmb_clock1_q[1]		<= tmb_clock1_ibufg;
	alct_rxclock_q[1]	<= alct_rxclock_ibufg;
	end

	assign lock_tmb_clock0d		= ^ tmb_clock0d_q[1:0];
	assign lock_alct_rxclockd	= ^ alct_rxclockd_q[1:0];
	assign lock_mpc_clock		= ^ mpc_clock_q[1:0];
	assign lock_dcc_clock		= ^ dcc_clock_q[1:0];
	assign lock_rpc_rxalt1		= ^ rpc_rxalt1_q[1:0];
	assign lock_tmb_clock1		= ^ tmb_clock1_q[1:0];
	assign lock_alct_rxclock	= ^ alct_rxclock_q[1:0];

// Sump
	assign clock_ctrl_sump = 0;

//---------------------------------------------------------------------------------------------------------------------
	endmodule
//---------------------------------------------------------------------------------------------------------------------
