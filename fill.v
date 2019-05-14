// Part 2 skeleton

module fill
	(
		PS2_CLK,
		PS2_DAT,
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		KEY, SW,							// On Board Keys
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,		//	VGA Blue[9:0]
		HEX0,
		HEX1,
		HEX2,
		HEX3,
		HEX4,
		HEX5,
		LEDR
		);
		
	output      [9:0] LEDR;
	output		[6:0]	HEX0;
	output		[6:0]	HEX1;
	output		[6:0]	HEX2;
	output		[6:0]	HEX3;
	output		[6:0]	HEX4;
	output		[6:0]	HEX5;
	inout			PS2_CLK;
	inout			PS2_DAT;
	input			CLOCK_50;				//	50 MHz
	input	[3:0]	KEY;
	input   [9:0]   SW;					
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
	output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
	output	[7:0]	VGA_B;   				//	VGA Blue[7:0]
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.

	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	
	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			// Signals for the DAC to drive the monitor.
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
	
	wire clear;
	
	assign writeEn = 1;
	assign clear = ~KEY[1];
	
	updatePixel inst(CLOCK_50, PS2_CLK, PS2_DAT, resetn, clear, dir, x, y, colour, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, LEDR);
	
	//moveCursor movcursor_inst(CLOCK_50, resetn, go, SW[2:0], x, y);
	
endmodule

module updatePixel(
		input clk, 
		inout ps2_clk, 
		inout ps2_dat,  
		input resetn, 
		input clear, 
		input [2:0]dir, 
		output [7:0]x, 
		output [6:0]y, 
		output [2:0]colourOut,
		output		[6:0]	hex0,
		output		[6:0]	hex1,
		output		[6:0]	hex2,
		output		[6:0]	hex3,
		output		[6:0]	hex4,
		output		[6:0]	hex5,
		output      [9:0] ledr
		);
		
		
	reg [7:0] initialMouseX;
	reg [6:0] initialMouseY;
	wire [7:0] mouseX;
	wire [7:0] mouseY;
	wire [7:0] mz;
	wire [3:0]mStatus;
	wire set;
	wire drawMode;
	wire [2:0] colourIn;
	
	reg [2:0] colourOutReg;
	reg [7:0] xCounter;
	reg [6:0] yCounter;
	initial xCounter = 159;
	initial yCounter = 119;
	wire atMouse;
	
	wire inRectReigon;
	reg enable;
	
	reg[1:0] counter;
	initial counter = 1;
	
	wire [14:0] address;
	wire wren;
	wire[2:0] qColour;
	wire[2:0] data_in;
	
	reg drawRect;
	
	assign address[0] = xCounter[0];
	assign address[1] = xCounter[1];
	assign address[2] = xCounter[2];
	assign address[3] = xCounter[3];
	assign address[4] = xCounter[4];
	assign address[5] = xCounter[5];
	assign address[6] = xCounter[6];
	assign address[7] = xCounter[7];
	assign address[8] = yCounter[0];
	assign address[9] = yCounter[1];
	assign address[10] = yCounter[2];
	assign address[11] = yCounter[3];
	assign address[12] = yCounter[4];
	assign address[13] = yCounter[5];
	assign address[14] = yCounter[6];
	
	
	wire base;
	wire height;
	
	assign base = mouseX - initialMouseX;
	assign height = mouseY - initialMouseY;
	
	assign ledr[3:0] = mStatus;
	assign colourIn = mz;
	
	assign atMouse = xCounter == mouseX && yCounter == mouseY;
	
	assign set = mStatus[0];
	assign drawMode = mStatus[1];
	
	assign wren = (atMouse & set & drawMode == 0) | (drawMode == 1 & inRectRegion ) | clear;
	assign data_in = clear ? 0 : colourIn;
	assign inRectRegion = (((xCounter >= initialMouseX) & (xCounter <= mouseX)) | ((xCounter <= initialMouseX) & (xCounter >= mouseX))) & (((yCounter >= initialMouseY) & (yCounter <= mouseY)) | ((yCounter <= initialMouseY) & (yCounter >= mouseY)));
	
	//moveCursor inst1(clk, resetn, go, dir, mouseX, mouseY);
	
	mouseMove inst(~resetn, enable, ps2_clk, ps2_dat, mStatus, mouseX, mouseY, mz);
	PixelMemory pixmem_inst(address, clk, data_in, wren, qColour);
	
	always@(*) begin
		if (drawMode == 0) colourOutReg = atMouse ? colourIn : qColour;
		else if (drawMode == 1) begin
			if (inRectRegion)
				colourOutReg = colourIn;
			else colourOutReg = qColour;
		end
	end
	
	always @(posedge clk) begin
		if (counter == 0) begin
			enable <= 1;
			counter <= 1;
		end else begin
			enable <= 0;
			counter <= counter - 1;
		end
		if (~drawRect & set & drawMode == 0) begin
			initialMouseX <= mouseX;
			initialMouseY <= mouseY;
			drawRect <= 1;
		end else if (~set) drawRect <= 0;
	end
	
	always @(posedge enable) begin
		if (xCounter > 0) xCounter <= xCounter - 1;
		else begin
			xCounter <= 159;
			if (yCounter > 0) yCounter <= yCounter - 1;
			else yCounter <= 119;
		end
	end
	
	assign colourOut = colourOutReg;
	assign x = xCounter;
	assign y = yCounter;
	
	
	
	Hexadecimal_To_Seven_Segment inst1(mouseX[3:0], hex0);
	Hexadecimal_To_Seven_Segment inst2(mouseX[7:4], hex1);
	Hexadecimal_To_Seven_Segment inst3(mouseY[3:0], hex2);
	Hexadecimal_To_Seven_Segment inst4(mouseY[7:4], hex3);
	Hexadecimal_To_Seven_Segment inst5(mz[3:0], hex4);
	Hexadecimal_To_Seven_Segment inst6(mz[7:4], hex5);
	
endmodule

/*
module moveCursor(input clk, input resetn, input go, input [2:0]dir, output reg [7:0]x, output reg [6:0]y);
	reg [23:0]clkCounter;
	initial clkCounter = 12500000 - 1;
	reg enable;
	
	always @(posedge clk) begin
		if (clkCounter == 0) begin
			enable <= 1;
			clkCounter <= 12500000 - 1;
		end else begin
			clkCounter <= clkCounter - 1;
			enable <= 0;
		end
	end
	
	always @(posedge enable) begin
		if (go) begin
			case (dir)
				0: begin
						if (x < 159) x <= x + 1;
					end
				1: begin
						if (x < 159 & y < 119) begin
						x <= x + 1;
						y <= y + 1;
						end
					end
				2: begin
						if (y < 119) y <= y + 1;
					end
				3: begin
						if (x > 0 & y < 159) begin
						x <= x - 1;
						y <= y + 1;
						end
					end
				4: begin
						if (x > 0) x <= x - 1;
					end
				5: begin
						if (x > 0 & y > 0) begin
						x <= x - 1;
						y <= y - 1;
						end
					end
				6: begin
						if (y > 0) y <= y - 1;
					end
				7: begin
						if (x < 159 & y > 0) begin
						y <= y - 1;
						x <= x + 1;
						end
					end
			endcase
		end
	end
	
	
	
endmodule*/

