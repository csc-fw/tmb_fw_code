`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------------
//	Finds:	  Best matching pattern template and number of layers hit on that pattern for 1 key 1/2-strip
//	Returns:  Best matching pattern template ID, and number of hits on the pattern
//
//	12/19/06 Initial
//	12/22/06 Change comparison direction
//	01/05/07 Change to combined 1/2-strip and distrip method
//	01/09/07 Replace count1s adders with LUT version
//	01/10/07 Narrow layer inputs to exclude unused 1/2-strips
//	01/16/07 Change from 15 to 9 patterns
//	01/18/07 Mod pattern 5 OR
//	02/22/07 Add pipleline latch
//	02/27/07 Reposition pipleline for max speed, min area
//	05/08/07 Change pattern numbers 1-9 to 0-8 so lsb now implies bend direction
//	05/23/07 Mod pattern 3 ly5 to mirror pattern 2
//	06/08/07 Remove pipeline stage
//	06/12/07 Had to revert to pipeline stage, could only achieve 30MHz otherwise
//	06/15/07 Incorporate layer mode as pattern 1, shift clct patterns IDs to the range 2-10
//	06/28/07 Shift key layer to ly2, flip patterns top-to-bottom, old ly0 becomes new ly5, left bends become right
//	07/02/07 Flip pat[i][5:0] to pat[i][0:5] to match prior ly3 result, reduces fpga usage from 93% to 90%
//	08/11/10 Port to ise 12
//	08/12/10 Replace LUT version of count1s because xst 12.2 inferred read-only ram instead of luts
//------------------------------------------------------------------------------------------------------------------------
	module pattern_unit
	(
// Input ports
	clock_2x,
	ly0,
	ly1,
	ly2,
	ly3,
	ly4,
	ly5,

// Output ports
	pat_nhits,
	pat_id
	);

//------------------------------------------------------------------------------------------------------------------------
// Generic
//------------------------------------------------------------------------------------------------------------------------
	parameter MXLY		= 6;			// Number of CSC layers
	parameter MXHITB	= 3;			// Hits on pattern bits
	parameter MXPID		=11;			// Number of patterns
	parameter MXPIDB	= 4;			// Pattern ID bits, lsb=bend direction

//------------------------------------------------------------------------------------------------------------------------
// Ports
//------------------------------------------------------------------------------------------------------------------------
// Inputs
	input					clock_2x;	// Pipeline clock
	input	[10:0]			ly0;
	input	[ 7:3]			ly1;
	input	[ 5:5]			ly2;		// Key layer 2
	input	[ 7:3]			ly3;
	input	[ 9:1]			ly4;
	input	[10:0]			ly5;		// 1/2-strips 1 layer 1 cell

// Outputs
	output	[MXHITB-1:0]	pat_nhits;	// Number layers hit for highest pattern
	output	[MXPIDB-1:0]	pat_id;		// Highest pattern found

//------------------------------------------------------------------------------------------------------------------------
// Finds best 1-of-9 1/2-strip patterns for 1 key 1/2-strip 
// Returns pattern number 2-10 and number of layers hit on that pattern 0-6.
// Pattern LSB = bend direction
// Hit pattern LUTs for 1 layer: - = don't care, xx= one hit or the other or both
//
// Pattern Templates:
//
// Pattern       id=2        id=3        id=4        id=5        id=6        id=7        id=8        id=9        idA
// Bend dir      bd=0        bd=1        bd=0        bd=1        bd=0        bd=1        bd=0        bd=1        bd=0
//               |           |           |           |           |           |           |           |           |
// ly0      --------xxx xxx-------- -------xxx- -xxx------- ------xxx-- --xxx------ -----xxx--- ---xxx----- ----xxx----
// ly1      ------xx--- ---xx------ ------xx--- ---xx------ -----xx---- ----xx----- -----xx---- ----xx----- -----x-----
// ly2 key  -----x----- -----x----- -----x----- -----x----- -----x----- -----x----- -----x----- -----x----- -----x-----
// ly3      ---xxx----- -----xxx--- ---xx------ ------xx--- ----xx----- -----xx---- ----xx----- -----xx---- -----x-----
// ly4      -xxx------- -------xxx- -xxx------- -------xxx- ---xx------ ------xx--- ---xxx----- -----xxx--- ----xxx----
// ly5      xxx-------- --------xxx -xxx------- -------xxx- --xxx------ ------xxx-- ---xxx----- -----xxx--- ----xxx----
//               |           |           |           |           |           |           |           |           |
// Extent   0123456789A 0123456789A 0123456789A 0123456789A 0123456789A 0123456789A 0123456789A 0123456789A 0123456789A
// Avg.bend - 8.0 hs    + 8.0 hs    -6.0 hs     +6.0 hs     -4.0 hs     +4.0 hs     -2.0 hs     +2.0 hs      0.0 hs
// Min.bend -10.0 hs    + 6.0 hs    -8.0 hs     +4.0 hs     -6.0 hs     +2.0 hs     -4.0 hs      0.0 hs     -1.0 hs
// Max.bend - 6.0 hs    +10.0 hs    -4.0 hs     +8.0 hs     -2.0 hs     +6.0 hs      0.0 hs     +4.0 hs     +1.0 hs
//------------------------------------------------------------------------------------------------------------------------
//	wire [MXLY-1:0] pat [MXPID-1:2];	// Ordering 5:0 uses 130 LUTs, but fpga usage is 93%
	wire [0:MXLY-1] pat [MXPID-1:2];	// Ordering 0:5 uses 132 LUTs, and fpga usage is 90%, matches ly3 key result
	parameter A=10;

// Pattern A										       0123456789A
	assign pat[A][0] = ly0[4]|ly0[5]|ly0[6];		// ly0 ----xxx----
	assign pat[A][1] =        ly1[5];				// ly1 -----x-----
	assign pat[A][2] =        ly2[5];				// ly2 -----x-----
	assign pat[A][3] =        ly3[5];				// ly3 -----x-----
	assign pat[A][4] = ly4[4]|ly4[5]|ly4[6];		// ly4 ----xxx---
	assign pat[A][5] = ly5[4]|ly5[5]|ly5[6];		// ly5 ----xxx---

// Pattern 9												       0123456789A
	assign pat[9][0] = ly0[3]|ly0[4]|ly0[5];				// ly0 ---xxx-----
	assign pat[9][1] =        ly1[4]|ly1[5];				// ly1 ----xx-----
	assign pat[9][2] =               ly2[5];				// ly2 -----x-----
	assign pat[9][3] =               ly3[5]|ly3[6];			// ly3 -----xx----
	assign pat[9][4] =               ly4[5]|ly4[6]|ly4[7];	// ly4 -----xxx---
	assign pat[9][5] =               ly5[5]|ly5[6]|ly5[7];	// ly5 -----xxx---
	
// Pattern 8												       0123456789A
	assign pat[8][0] =               ly0[5]|ly0[6]|ly0[7];	// ly0 -----xxx---
	assign pat[8][1] =               ly1[5]|ly1[6];			// ly1 -----xx----
	assign pat[8][2] =               ly2[5];				// ly2 -----x-----
	assign pat[8][3] =        ly3[4]|ly3[5];				// ly3 ----xx-----
	assign pat[8][4] = ly4[3]|ly4[4]|ly4[5];				// ly4 ---xxx-----
	assign pat[8][5] = ly5[3]|ly5[4]|ly5[5];				// ly5 ---xxx-----

// Pattern 7																       0123456789A
	assign pat[7][0] = ly0[2]|ly0[3]|ly0[4];								// ly0 --xxx------
	assign pat[7][1] =               ly1[4]|ly1[5];							// ly1 ----xx-----
	assign pat[7][2] =                      ly2[5];							// ly2 -----x-----
	assign pat[7][3] =                      ly3[5]|ly3[6];					// ly3 -----xx----
	assign pat[7][4] =                             ly4[6]|ly4[7];			// ly4 ------xx---
	assign pat[7][5] =                             ly5[6]|ly5[7]|ly5[8];	// ly5 ------xxx--

// Pattern 6																       0123456789A
	assign pat[6][0] =                             ly0[6]|ly0[7]|ly0[8];	// ly0 ------xxx--
	assign pat[6][1] =                      ly1[5]|ly1[6];					// ly1 -----xx----
	assign pat[6][2] =                      ly2[5];							// ly2 -----x-----
	assign pat[6][3] =               ly3[4]|ly3[5];							// ly3 ----xx-----
	assign pat[6][4] =        ly4[3]|ly4[4];								// ly4 ---xx------
	assign pat[6][5] = ly5[2]|ly5[3]|ly5[4];								// ly5 --xxx------

// Pattern 5																			       0123456789A
	assign pat[5][0] = ly0[1]|ly0[2]|ly0[3];											// ly0 -xxx-------
	assign pat[5][1] =               ly1[3]|ly1[4];										// ly1 ---xx------
	assign pat[5][2] =                             ly2[5];								// ly2 -----x-----
	assign pat[5][3] =                                    ly3[6]|ly3[7];				// ly3 ------xx---
	assign pat[5][4] =                                           ly4[7]|ly4[8]|ly4[9];	// ly4 -------xxx-
	assign pat[5][5] =                                           ly5[7]|ly5[8]|ly5[9];	// ly5 -------xxx-
	
// Pattern 4																			       0123456789A
	assign pat[4][0] =                                            ly0[7]|ly0[8]|ly0[9];	// ly0 -------xxx-
	assign pat[4][1] =                                     ly1[6]|ly1[7];				// ly1 ------xx---
	assign pat[4][2] =                              ly2[5];								// ly2 -----x-----
	assign pat[4][3] =               ly3[3]|ly3[4];										// ly3 ---xx------
	assign pat[4][4] = ly4[1]|ly4[2]|ly4[3];											// ly4 -xxx-------
	assign pat[4][5] = ly5[1]|ly5[2]|ly5[3];											// ly5 -xxx-------

// Pattern 3																							       0123456789A
	assign pat[3][0] = ly0[0]|ly0[1]|ly0[2];															// ly0 xxx--------
	assign pat[3][1] =                      ly1[3]|ly1[4];												// ly1 ---xx------
	assign pat[3][2] =                                    ly2[5];										// ly2 -----x-----
	assign pat[3][3] =                                    ly3[5]|ly3[6]|ly3[7];							// ly3 -----xxx---
	assign pat[3][4] =                                                  ly4[7]|ly4[8]|ly4[9];			// ly4 -------xxx-
	assign pat[3][5] =                                                         ly5[8]|ly5[9]|ly5[A];	// ly5 --------xxx

// Pattern 2																							       0123456789A
	assign pat[2][0] =                                                         ly0[8]|ly0[9]|ly0[A];	// ly0 --------xxx
	assign pat[2][1] =                                           ly1[6]|ly1[7];							// ly1 ------xx---
	assign pat[2][2] =                                    ly2[5];										// ly2 -----x-----
	assign pat[2][3] =                      ly3[3]|ly3[4]|ly3[5];										// ly3 ---xxx-----
	assign pat[2][4] =        ly4[1]|ly4[2]|ly4[3];														// ly4 -xxx-------
	assign pat[2][5] = ly5[0]|ly5[1]|ly5[2];															// ly5 xxx--------

// Count number of layers hit for each pattern
	wire [MXHITB-1:0] nhits [MXPID-1:2];

	genvar i;
	generate
	for (i=2; i<=MXPID-1; i=i+1) begin: gencount
	assign nhits[i] = count1s(pat[i]);
	end
	endgenerate

// Best 1 of 8 Priority Encoder, perfers higher pattern number if hits are equal
	reg  [MXHITB-1:0] nhits_s0 [4:0];
	wire [MXHITB-1:0] nhits_s1 [2:0];
	wire [MXHITB-1:0] nhits_s2 [1:0];
	wire [MXHITB-1:0] nhits_s3 [0:0];

	reg  [3:0] pid_s0;	// pid_s0[4] is a constant	
	wire [1:0] pid_s1 [2:0];
	wire [2:0] pid_s2 [1:0];
	wire [3:0] pid_s3 [0:0];

// 9 to 5, pipleline FF, latch A on 40MHz falling edge, B on rising edge
	always @(posedge clock_2x) begin
	{nhits_s0[4]          } =                         {nhits[A]     };
	{nhits_s0[3],pid_s0[3]} = (nhits[8] > nhits[9]) ? {nhits[8],1'b0} : {nhits[9],1'b1};
	{nhits_s0[2],pid_s0[2]} = (nhits[6] > nhits[7]) ? {nhits[6],1'b0} : {nhits[7],1'b1};
	{nhits_s0[1],pid_s0[1]} = (nhits[4] > nhits[5]) ? {nhits[4],1'b0} : {nhits[5],1'b1};
	{nhits_s0[0],pid_s0[0]} = (nhits[2] > nhits[3]) ? {nhits[2],1'b0} : {nhits[3],1'b1};
	end
	wire pid_s0_4 = 1'b0;

// 5 to 3
	assign {nhits_s1[2],pid_s1[2]} =                               {nhits_s0[4],{1'b0,pid_s0_4 }};
	assign {nhits_s1[1],pid_s1[1]} = (nhits_s0[2] > nhits_s0[3]) ? {nhits_s0[2],{1'b0,pid_s0[2]}} : {nhits_s0[3],{1'b1,pid_s0[3]}};
	assign {nhits_s1[0],pid_s1[0]} = (nhits_s0[0] > nhits_s0[1]) ? {nhits_s0[0],{1'b0,pid_s0[0]}} : {nhits_s0[1],{1'b1,pid_s0[1]}};

// 3 to 2
	assign {nhits_s2[1],pid_s2[1]} =                               {nhits_s1[2],{1'b0,pid_s1[2]}};
	assign {nhits_s2[0],pid_s2[0]} = (nhits_s1[0] > nhits_s1[1]) ? {nhits_s1[0],{1'b0,pid_s1[0]}} : {nhits_s1[1],{1'b1,pid_s1[1]}};

// 2 to 1
	assign {nhits_s3[0],pid_s3[0]} = (nhits_s2[0] > nhits_s2[1]) ? {nhits_s2[0],{1'b0,pid_s2[0]}} : {nhits_s2[1],{1'b1,pid_s2[1]}};

// Add 2 to pid to shift to range 2-10
	assign pat_nhits = nhits_s3[0];
	assign pat_id	 = pid_s3[0]+4'd2;

//------------------------------------------------------------------------------------------------------------------------
// Prodcedural function to sum number of layers hit into a binary value - LUT version
// Returns 	count1s = (inp[5]+inp[4]+inp[3])+(inp[2]+inp[1]+inp[0]);
//
// 7 LUT 7.564nsec
// 01/09/2007 Initial
//------------------------------------------------------------------------------------------------------------------------
	function [2:0]	count1s;
	input	 [5:0]	inp;
	reg		 [3:0]	lut;
	reg		 [2:0]	rom;

	begin
	case (inp[2:0])			// inp[2:0] sum lsb
	3'b000:	lut[0] = 0;		// 0
	3'b001:	lut[0] = 1;		// 1
	3'b010:	lut[0] = 1;		// 1
	3'b011:	lut[0] = 0;		// 2
	3'b100:	lut[0] = 1;		// 1
	3'b101:	lut[0] = 0;		// 2
	3'b110:	lut[0] = 0;		// 2
	3'b111:	lut[0] = 1;		// 3
	endcase

	case (inp[2:0])			// inp[2:0] sum msb
	3'b000:	lut[1] = 0;		// 0
	3'b001:	lut[1] = 0;		// 1
	3'b010:	lut[1] = 0;		// 1
	3'b011:	lut[1] = 1;		// 2
	3'b100:	lut[1] = 0;		// 1
	3'b101:	lut[1] = 1;		// 2
	3'b110:	lut[1] = 1;		// 2
	3'b111:	lut[1] = 1;		// 3
	endcase

	case (inp[5:3])			// inp[5:3] sum lsb
	3'b000:	lut[2] = 0;		// 0
	3'b001:	lut[2] = 1;		// 1
	3'b010:	lut[2] = 1;		// 1
	3'b011:	lut[2] = 0;		// 2
	3'b100:	lut[2] = 1;		// 1
	3'b101:	lut[2] = 0;		// 2
	3'b110:	lut[2] = 0;		// 2
	3'b111:	lut[2] = 1;		// 3
	endcase

	case (inp[5:3])			// inp[5:3] sum msb
	3'b000:	lut[3] = 0;		// 0
	3'b001:	lut[3] = 0;		// 1
	3'b010:	lut[3] = 0;		// 1
	3'b011:	lut[3] = 1;		// 2
	3'b100:	lut[3] = 0;		// 1
	3'b101:	lut[3] = 1;		// 2
	3'b110:	lut[3] = 1;		// 2
	3'b111:	lut[3] = 1;		// 3
	endcase

	case (lut[3:0])			// sum lut[3:2]+lut[1:0] bit 0
	4'b0000: rom[0] = 0;	// 0 000
	4'b0001: rom[0] = 1;	// 1 001
	4'b0010: rom[0] = 0;	// 2 010
	4'b0011: rom[0] = 1;	// 3 011
	4'b0100: rom[0] = 1;	// 1 001
	4'b0101: rom[0] = 0;	// 2 010
	4'b0110: rom[0] = 1;	// 3 011
	4'b0111: rom[0] = 0;	// 4 100
	4'b1000: rom[0] = 0;	// 2 010
	4'b1001: rom[0] = 1;	// 3 011
	4'b1010: rom[0] = 0;	// 4 100
	4'b1011: rom[0] = 1;	// 5 100
	4'b1100: rom[0] = 1;	// 3 011
	4'b1101: rom[0] = 0;	// 4 100
	4'b1110: rom[0] = 1;	// 5 101
	4'b1111: rom[0] = 0;	// 6 110
	endcase

	case (lut[3:0])			// sum lut[3:2]+lut[1:0] bit 1
	4'b0000: rom[1] = 0;	// 0 000
	4'b0001: rom[1] = 0;	// 1 001
	4'b0010: rom[1] = 1;	// 2 010
	4'b0011: rom[1] = 1;	// 3 011
	4'b0100: rom[1] = 0;	// 1 001
	4'b0101: rom[1] = 1;	// 2 010
	4'b0110: rom[1] = 1;	// 3 011
	4'b0111: rom[1] = 0;	// 4 100
	4'b1000: rom[1] = 1;	// 2 010
	4'b1001: rom[1] = 1;	// 3 011
	4'b1010: rom[1] = 0;	// 4 100
	4'b1011: rom[1] = 0;	// 5 101
	4'b1100: rom[1] = 1;	// 3 011
	4'b1101: rom[1] = 0;	// 4 100
	4'b1110: rom[1] = 0;	// 5 101
	4'b1111: rom[1] = 1;	// 6 110
	endcase

	case (lut[3:0])			// sum lut[3:2]+lut[1:0] bit 2
	4'b0000: rom[2] = 0;	// 0 000
	4'b0001: rom[2] = 0;	// 1 001
	4'b0010: rom[2] = 0;	// 2 010
	4'b0011: rom[2] = 0;	// 3 011
	4'b0100: rom[2] = 0;	// 1 001
	4'b0101: rom[2] = 0;	// 2 010
	4'b0110: rom[2] = 0;	// 3 011
	4'b0111: rom[2] = 1;	// 4 100
	4'b1000: rom[2] = 0;	// 2 010
	4'b1001: rom[2] = 0;	// 3 011
	4'b1010: rom[2] = 1;	// 4 100
	4'b1011: rom[2] = 1;	// 5 101
	4'b1100: rom[2] = 0;	// 3 011
	4'b1101: rom[2] = 1;	// 4 100
	4'b1110: rom[2] = 1;	// 5 101
	4'b1111: rom[2] = 1;	// 6 110
	endcase

	count1s=rom;
	end
	endfunction

//------------------------------------------------------------------------------------------------------------------------
	endmodule
//------------------------------------------------------------------------------------------------------------------------
