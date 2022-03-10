`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
// Virtex2: Clock DCM quadrant multiplexer for digital phase shifters
//------------------------------------------------------------------------------------------------------------------
//	08/04/09 Initial
//	08/05/09 Replace timing constraints with location constraints in ucf
//	08/06/09 Add local ffs to boost quadrant selects into 160mhz time domain
//	08/07/09 Relax quadrant selects maxdelay, add skew limit to clk out
//	08/10/09 Add local ffs on quadrant selects in 40mhz time domain prior to 160mhz boost
//	08/10/09 Move local ffs out of this module, caused par to fail in full tmb, but test subdesign is ok
//	08/12/09 Add fillers to occupy the unused slice in clb
//	08/14/09 Add clock property to 4x
//	11/08/10 Add bypass for virtex6, this module is not used
//	11/30/10 Conform virtex2|6 selection
//-----------------------------------------------------------------------------------------------------------------
	module clock_mux
	(
	clk4x,
	hcycle,
	qcycle,
	clk0,
	clk90,
	clk180,
	clk270,
	clk
	);

// Ports
	input	clk4x;		// 4x clock
	input	hcycle;		// Half    cycle quadrant select
	input	qcycle;		// Quarter cycle quadrant select
	input	clk0;		// Clock 000 degrees
	input	clk90;		// Clock 090 degrees
	input	clk180;		// Clock 180 degrees
	input	clk270;		// Clock 270 degrees
	output	clk;		// Quadrant shifted clock

// Transfer DC quadrant select to 160MHz clock domain, and fill unused slice in clb
	(*SHREG_EXTRACT="NO"*) reg [1:0] quadrant   = 0;
	(*SHREG_EXTRACT="NO"*) reg [1:0] transfer_a = 0;
	(*SHREG_EXTRACT="NO"*) reg [1:0] transfer_b = 0;
	
	always @(posedge clk4x) begin
	transfer_a <= {hcycle, qcycle};
	transfer_b <= transfer_a;
	quadrant   <= transfer_b;
	end

// Clock multiplexer latches 160MHz clock on falling edge, selects phase quadrant
	(*CLOCK_SIGNAL="YES", MAXSKEW=  "1ns" *) reg  clk=0;
	(*CLOCK_SIGNAL="YES", MAXDELAY= "2ns" *) wire clk4x;

	always @(negedge clk4x) begin
	case (quadrant)
	2'b00:	clk <= clk0;
	2'b01:	clk <= clk90;
	2'b10:	clk <= clk180;
	2'b11:	clk <= clk270;
	endcase
	end

//------------------------------------------------------------------------------------------------------------------
	endmodule
//------------------------------------------------------------------------------------------------------------------
