//`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:42:10 18/02/2014 
// Design Name: 
// Module Name:    MouseTransceiver 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 	Large ammounts of code were already provided, however
//									that also greatly restricts my coding options
//
//////////////////////////////////////////////////////////////////////////////////
/*
Seven Segement Display values: MouseX, MouseY
*/

module MouseTester(input CLOCK_50, input [0:0]KEY, inout PS2_CLK, inout PS2_DAT, 
output		[6:0]	HEX0,
output		[6:0]	HEX1,
output		[6:0]	HEX2,
output		[6:0]	HEX3,
output		[6:0]	HEX4,
output		[6:0]	HEX5,
output      [9:0] LEDR);

wire [7:0]mx;
wire [7:0]my;
wire [7:0]mz;
wire [3:0]mStatus;

MouseTransceiver inst(~KEY[0], CLOCK_50, PS2_CLK, PS2_DAT, mStatus, mx, my, mz);

Hexadecimal_To_Seven_Segment inst1(mx[3:0], HEX0);
Hexadecimal_To_Seven_Segment inst2(mx[7:4], HEX1);
Hexadecimal_To_Seven_Segment inst3(my[3:0], HEX2);
Hexadecimal_To_Seven_Segment inst4(my[7:4], HEX3);
Hexadecimal_To_Seven_Segment inst5(mz[3:0], HEX4);
Hexadecimal_To_Seven_Segment inst6(mz[7:4], HEX5);
assign LEDR[3:0] = mStatus;


endmodule

module mouseMove(
	input 			reset,
	input 			clk,
	inout 			ps2_clk,
	inout 			ps2_dat,
	output [3:0] 	mStatus,
	output [7:0] 	mx,
	output [7:0] 	my,
	output [7:0]	mz);
	
	wire [7:0] mouseY;
	
	MouseTransceiver inst(reset, clk, ps2_clk, ps2_dat, mStatus, mx, mouseY, mz);
	
	assign my = 119 - mouseY;
	
endmodule


module MouseTransceiver(
	//Standard Inputs
	input 			RESET,
	input 			CLK,
	//IO - Mouse side
	inout 			CLK_MOUSE,
	inout 			DATA_MOUSE,
	// Mouse data information
	output reg [3:0] 	MouseStatus,		// needed to add reg status to do non-blocking assign
	output reg [7:0] 	MouseX,
	output reg [7:0] 	MouseY,
	output reg [7:0]	MouseZ
);


	// X, Y Limits of Mouse Position e.g. VGA Screen with 160 x 120 resolution
	parameter [7:0] MouseLimitX = 160;
	parameter [7:0] MouseLimitY = 120;
	parameter [7:0] MouseLimitZ = 7;
	
	/////////////////////////////////////////////////////////////////////
	//TriState Signals
	//Clk
	reg 	ClkMouseIn;
	wire 	ClkMouseOutEnTrans;
	
	//Data
	wire 	DataMouseIn;
	wire 	DataMouseOutTrans;
	wire 	DataMouseOutEnTrans;
	
	//Clk Output - can be driven by host or device
	assign CLK_MOUSE = ClkMouseOutEnTrans ? 1'b0 : 1'bz;
	
	//Clk Input
	assign DataMouseIn = DATA_MOUSE;
	
	//Clk Output - can be driven by host or device
	assign DATA_MOUSE = DataMouseOutEnTrans ? DataMouseOutTrans : 1'bz;

	/////////////////////////////////////////////////////////////////////
	//This section filters the incoming Mouse clock to make sure that
	//it is stable before data is latched by either transmitter
	//or receiver modules
	reg [7:0] MouseClkFilter;
	
	always@(posedge CLK) begin
		if(RESET)
			ClkMouseIn <= 1'b0;
		else 
			begin
				//A simple shift register
				MouseClkFilter[7:1] <= MouseClkFilter[6:0];
				MouseClkFilter[0] <= CLK_MOUSE;
				
				//falling edge
				if(ClkMouseIn & (MouseClkFilter == 8'h00))			
					ClkMouseIn <= 1'b0;
				
				//rising edge
				else if(~ClkMouseIn & (MouseClkFilter == 8'hFF))
					ClkMouseIn <= 1'b1;
			end
	end
	///////////////////////////////////////////////////////
	//Instantiate the Transmitter module
	wire 			SendByteToMouse;
	wire 			ByteSentToMouse;
	wire [7:0] 	ByteToSendToMouse;
	MouseTransmitter T(
					//Standard Inputs
					.RESET (RESET),
					.CLK(CLK),
					//Mouse IO - CLK
					.CLK_MOUSE_IN(ClkMouseIn),
					.CLK_MOUSE_OUT_EN(ClkMouseOutEnTrans),
					//Mouse IO - DATA
					.DATA_MOUSE_IN(DataMouseIn),
					.DATA_MOUSE_OUT(DataMouseOutTrans),
					.DATA_MOUSE_OUT_EN (DataMouseOutEnTrans),
					//Control
					.SEND_BYTE(SendByteToMouse),
					.BYTE_TO_SEND(ByteToSendToMouse),
					.BYTE_SENT(ByteSentToMouse)
	);
	
	
	///////////////////////////////////////////////////////
	//Instantiate the Receiver module
	wire 					ReadEnable;
	wire [7:0] 				ByteRead;
	wire [1:0] 				ByteErrorCode;
	wire 					ByteReady;
	MouseReceiver R(
							//Standard Inputs
							.RESET(RESET),
							.CLK(CLK),
							//Mouse IO - CLK
							.CLK_MOUSE_IN(ClkMouseIn),
							//Mouse IO - DATA
							.DATA_MOUSE_IN(DataMouseIn),
							//Control
							.READ_ENABLE (ReadEnable),
							.BYTE_READ(ByteRead),
							.BYTE_ERROR_CODE(ByteErrorCode),
							.BYTE_READY(ByteReady)
	);

	///////////////////////////////////////////////////////
	//Instantiate the Master State Machine module
	wire [7:0] 				MouseStatusRaw;
	wire [7:0] 				MouseDxRaw;
	wire [7:0] 				MouseDyRaw;
	wire [7:0]				MouseDzRaw;
	wire 					SendInterrupt;
	MouseMasterSM MSM(
								//Standard Inputs
								.RESET(RESET),
								.CLK(CLK),
								//Transmitter Interface
								.SEND_BYTE(SendByteToMouse),
								.BYTE_TO_SEND(ByteToSendToMouse),
								.BYTE_SENT(ByteSentToMouse),
								//Receiver Interface
								.READ_ENABLE (ReadEnable),
								.BYTE_READ(ByteRead),
								.BYTE_ERROR_CODE(ByteErrorCode),
								.BYTE_READY(ByteReady),
								//Data Registers
								.MOUSE_STATUS(MouseStatusRaw),
								.MOUSE_DX(MouseDxRaw),
								.MOUSE_DY(MouseDyRaw),
								.MOUSE_DZ(MouseDzRaw),
								.SEND_INTERRUPT(SendInterrupt)
	);

/*	SevenSegmentDisplay SevenSeg(
								.CLK(CLK),
								// Anode values
								.AN0(AN0),
								.AN1(AN1),
								.AN2(AN2),
								.AN3(AN3),
								// Cathode Values
								.CA(CA),
								.CB(CB),
								.CC(CC),
								.CD(CD),
								.CE(CE),
								.CF(CF),
								.CG(CG),
								.CDP(CDP),
								// Actual display values
								.DIGIT1(MouseX[7:4]),		
								.DIGIT2(MouseX[3:0]),		
								.DIGIT3(MouseZ[7:4]),		
								.DIGIT4(MouseZ[3:0])			
	);*/
	
	//Pre-processing - handling of overflow and signs.
	//More importantly, this keeps tabs on the actual X/Y
	//location of the mouse.
	wire signed [8:0] MouseDx;
	wire signed [8:0] MouseDy;
	wire signed [8:0] MouseNewX;
	wire signed [8:0] MouseNewY;
	wire signed [8:0] MouseDz;
	wire signed [8:0] MouseNewZ;
	
	//DX and DY are modified to take account of overflow and direction
	assign MouseDx = (MouseStatusRaw[6]) ? (MouseStatusRaw[4] ? {MouseStatusRaw[4],8'h00} : {MouseStatusRaw[4],8'hFF} ) : {MouseStatusRaw[4],MouseDxRaw[7:0]};
	/*
	………………..
	FILL IN THIS AREA
	……………….
	
	*/
	// assign the proper expression to MouseDy
	assign MouseDy = (MouseStatusRaw[7]) ? (MouseStatusRaw[5] ? {MouseStatusRaw[5],8'h00} : {MouseStatusRaw[5],8'hFF} ) : {MouseStatusRaw[5],MouseDyRaw[7:0]};
	/* Below is pseudocode describing this assign statement
		
		if MouseStatusRaw[7]{													// movement overflow
			if MouseStatusRaw[5]{												// negative movement (sign bit)
				MouseDy = concat(MouseStatusRaw[5],8'h00)					//	if overflow dy && neg dy -> set PosY = 0x100 = -256
			}
			else{
				MouseDy = concat(MouseStatusRaw[5],8'hFF)					// if overflow dy && pos dy -> set PosY = 0x0FF = 255
			}
		}
		else{
				MouseDy = concat(MouseStatusRaw[5],MouseDyRaw[7:0])	// if not overflow, accept incoming dy value in 2's comp (hence prepended sign bit)
		}
	
	*/
	// calculate new mouse position
	assign MouseDz = {MouseDzRaw[7],MouseDzRaw[7:0]};
	
	
	assign MouseNewX = {1'b0,MouseX} + MouseDx;
	assign MouseNewY = {1'b0,MouseY} + MouseDy;
	assign MouseNewZ = {1'b0,MouseZ} + MouseDz;

	always@(posedge CLK)
		begin
			if(RESET) 
				begin
					MouseStatus 	<= 0;
					MouseX 			<= MouseLimitX/2;
					MouseY 			<= MouseLimitY/2;
					MouseZ 			<= MouseLimitZ/2;
				end 
			else if (SendInterrupt) 
				begin
				
					//Status is stripped of all unnecessary info
					MouseStatus 	<= MouseStatusRaw[3:0];
				
					//X is modified based on DX with limits on max and min
					if(MouseNewX < 0)
						MouseX 		<= 0;
					else if(MouseNewX > (MouseLimitX-1))
						MouseX 		<= MouseLimitX-1;
					else
						MouseX 		<= MouseNewX[7:0];
					/*
					………………..
					FILL IN THIS AREA
					……………….
					*/
					//Y is modified based on DY with limits on max and min
					if(MouseNewY < 0)
						MouseY 		<= 0;
					else if(MouseNewY > (MouseLimitY-1))
						MouseY 		<= MouseLimitY-1;
					else
						MouseY 		<= MouseNewY[7:0];
						
					//Z is modified based on DZ with limits on max and min
					//Z wraps around, as this is a scroll wheel
					if(MouseNewZ < 0)
						MouseY 		<= MouseLimitZ;
					else if(MouseNewZ > MouseLimitZ)
						MouseZ 		<= 0;
					else
						MouseZ 		<= MouseNewZ[7:0];				
				end
		end


endmodule

