// decoder_10b_8b_BRAM.v
// Diandian Chen, 04/28/2018, 7/13/2018
//
// Description:
// It implements the 10b/8b decoder using BRAMs. There are two decoders for running disparity = +1 and
// -1 respectively. Due to the 1 clk delay of BRAMs, there's a register to latch the need for flipping
// running disparity (disparity_polarity_reg), helping running_disparity to mux final data_out.
// The initialization table for decoder BRAMs are derived from encoding tables that map 8-bit unencoded
// symbols into 10-bit encoded symbols (A Python script is used to get this decoder table from encoder
// table, related files are included). For each entry in the decoder BRAM, there are 2 more bits other
// than 8-bit decoded symbol, one indicating whether there's a valid match for current 10-bit address,
// the other indicating whether current match is a control symbol/data symbol.
//
// Attention: 
// In this design, "m" and "p" stand for "minus" and "plus", but "m" and "p" variables and RD registers
// DON'T represent running disparity of current 10-bit encoded number. Instead, they indicate the running
// disparity when corresponding 8-bit number is encoded into a 10-bit number at TX side. 
// e.g. For the 10-bit encoded number 0111011100, there're more 1s than 0s, which means when encoding
// the original number at TX side, the running disparity should be -1 so that after encoding it into
// 0111011100, the running disparity becomes +1 in the next cycle. Therefore, when decoding 0111011100,
// we should look into the decoder for RD = -1 instead of +1

module decoder_10b_8b(
	input clk,
	input resetn,
	input valid_in,						// indicates that there's transmission going on on physical lines
	input [9:0] data_in,				// 10-bit symbol from physical lines
	output [7:0] data_out,				// decoded symbol
	output reg valid_out,				// data_out is valid after the first COM has been detected
	output control_data,				// 1: current symbol is for control   0: current symbol is for data
	output receiver_error_detected		// running disparity error
	);

	localparam COM_M	= 10'b0101_111100;			// K.28.5 
	localparam COM_P	= 10'b1010_000011;
	
	wire [7:0] decoded_p, decoded_m;				// decoded symbols from two different BRAMs
	wire error_m, error_p;							// indicates whether there's a match error in corresponding BRAM
	wire control_data_p, control_data_m;			// 1: K   0: D
	wire [9:0] bram_read_p, bram_read_m;			// 10-bit symbol from corresponding BRAM

	wire disparity_polarity;						// if 0, RD needs to go from -1 to +1 or +1 to -1
	reg disparity_polarity_reg;						// register for disparity_polarity
	reg running_disparity;							// 1 for RD = +1 and 0 for RD = -1. See "Attention" on the top

	// bram_read is the 10-bit data read from one BRAM entry. Each 10-bit entry in BRAM is formed by:
	// bit 9: Error indicator. A 1 indicates that such 10-bit address has no valid interpreted 8-bit number.
	// bit 8: K/D indicator. A 1 indicates data bits current entry form a control symbol. 0 for data symbol.
	// bits 7-0: 8-bit symbol in current entry.
	assign decoded_p = bram_read_p[7:0];
	assign error_p = bram_read_p[9];
	assign control_data_p = bram_read_p[8];

	assign decoded_m = bram_read_m[7:0];
	assign error_m = bram_read_m[9];
	assign control_data_m = bram_read_m[8];

// Task 1: Generate appropriate signal to indicate whether RD should flip: 0 - flip, 1 - stay the same
	assign disparity_polarity = ^data_in[9:0];	   // There can only be 3 situations for a valid 10-bit symbol: six 1s, five 1s and four 1s.
											// If five 1s, RD remains the same; if six/four 1s, RD needs to be flipped (either from
											// -1 to +1 or from +1 to -1).
// ----------------------------------------------------------------------------------------------------
	// based on current running_disparity, one 10-bit symbol is chosen between two BRAMS
	assign data_out = running_disparity? decoded_p : decoded_m;
	assign receiver_error_detected = running_disparity? error_p : error_m;
	assign control_data = running_disparity? control_data_p : control_data_m;

	always @(posedge clk, negedge resetn)
	if (!resetn)
	begin
		running_disparity <= 0;
		valid_out <= 0;
	end
	else
	begin
		if (valid_in)
		begin
		    disparity_polarity_reg <= disparity_polarity;
			if (!valid_out) 	// Before the first valid symbol (which should be a COM symbol) is detected on physical line, valid_out is 0.
// Task 2: Compelte assignments to initiate RD based on polarity of the first COM detected
			begin
				if (data_in == COM_P)
				begin
					running_disparity <=  1 ;			// RD should start with +1 as the first detected symbol is COM_plus
					valid_out <=  1  ;
				end
				else if (data_in == COM_M)
				begin
					running_disparity <= 0 ;			// RD should start with -1 as the first detected symbol is COM_minus
					valid_out <= 1   ;
				end
			end
// ----------------------------------------------------------------------------------------------------
			else
			begin
				if (!disparity_polarity_reg)		// if there're four/six 1s in the latest input symbol, flip RD
					running_disparity <= ~running_disparity;
			end
		end
		else
			valid_out <= 0;
	end

	// The following two BRAM instantiations can be directly recognized by VIVADO synthesizer and get translated into BRAM resources.
	// Actually for artix-7, it's kind of efficient to implement the decoder LUT by BRAM, as the smallest BRAM unit is 18Kb, which is
	// 1024 locations deep and 18 bits wide. For our 10b-to-8b decoding, addressing BRAM with 10 bits can just fit in one 18Kb BRAM.

	pcie_phy_10b8b_bram #(
		// .MEM_INIT_FILE("C:/Vivado_Projects/PCIe_UART_DU/minus.txt"),
		.MEM_INIT_FILE("minus.data"),
		.ADDR_WIDTH(10),
		.DOUT_WIDTH(10),
		.RADIX(16)
	) decode_10b8brdminus (
    	.clk(clk), 
    	.wea(1'b0), 
    	.addra(), 
    	.dina(), 
    	.addrb(data_in), 
    	.doutb(bram_read_m)
    	);

	pcie_phy_10b8b_bram #(
		// .MEM_INIT_FILE("C:/Vivado_Projects/PCIe_UART_DU/plus.txt"),
		.MEM_INIT_FILE("plus.data"),
		.ADDR_WIDTH(10),
		.DOUT_WIDTH(10),
        .RADIX(16)
	) decode_10b8brdplus (
    	.clk(clk), 
    	.wea(1'b0), 
    	.addra(), 
    	.dina(), 
    	.addrb(data_in), 
    	.doutb(bram_read_p)
    	);


	// For more information, in VIVADO, go to tools -> language templates
	
	// BRAM_SINGLE_MACRO #(
	// 	.BRAM_SIZE("18Kb"), // Target BRAM, "18Kb" or "36Kb" 
	// 	.DEVICE("7SERIES"), // Target Device: "7SERIES" 
	// 	.DO_REG(0), // Optional output register (0 or 1)
	// 	.INIT(18'h000000000), // Initial values on output port
	// 	.INIT_FILE ("C:/Vivado_Projects/PCIe_UART_DU/minus.txt"),
	// 	.WRITE_WIDTH(10), // Valid values are 1-72 (37-72 only valid when BRAM_SIZE="36Kb")
	// 	.READ_WIDTH(10),  // Valid values are 1-72 (37-72 only valid when BRAM_SIZE="36Kb")
	// 	.SRVAL(18'h000000000), // Set/Reset value for port output
	// 	.WRITE_MODE("NO_CHANGE") // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE" 
	// ) BRAM_minus (
	// 	.DO(bram_read_m),       // Output data, width defined by READ_WIDTH parameter
	// 	.ADDR(data_in),   // Input address, width defined by read/write port depth
	// 	.CLK(clk),     // 1-bit input clock
	// 	.DI(10'b0),       // Input data port, width defined by WRITE_WIDTH parameter
	// 	.EN(1'b1),       // 1-bit input RAM enable
	// 	.REGCE(1'b0), // 1-bit input output register enable
	// 	.RST(~resetn),     // 1-bit input reset
	// 	.WE(2'b0)        // Input write enable, width defined by write port depth
	// 	);

	// BRAM_SINGLE_MACRO #(
	// 	.BRAM_SIZE("18Kb"), // Target BRAM, "18Kb" or "36Kb" 
	// 	.DEVICE("7SERIES"), // Target Device: "7SERIES" 
	// 	.DO_REG(0), // Optional output register (0 or 1)
	// 	.INIT(18'h000000000), // Initial values on output port
	// 	.INIT_FILE ("C:/Vivado_Projects/PCIe_UART_DU/plus.txt"),
	// 	.WRITE_WIDTH(10), // Valid values are 1-72 (37-72 only valid when BRAM_SIZE="36Kb")
	// 	.READ_WIDTH(10),  // Valid values are 1-72 (37-72 only valid when BRAM_SIZE="36Kb")
	// 	.SRVAL(18'h000000000), // Set/Reset value for port output
	// 	.WRITE_MODE("NO_CHANGE") // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE" 
	// ) BRAM_plus (
	// 	.DO(bram_read_p),       // Output data, width defined by READ_WIDTH parameter
	// 	.ADDR(data_in),   // Input address, width defined by read/write port depth
	// 	.CLK(clk),     // 1-bit input clock
	// 	.DI(10'b0),       // Input data port, width defined by WRITE_WIDTH parameter
	// 	.EN(1'b1),       // 1-bit input RAM enable
	// 	.REGCE(1'b0), // 1-bit input output register enable
	// 	.RST(~resetn),     // 1-bit input reset
	// 	.WE(2'b0)        // Input write enable, width defined by write port depth
	// 	);

endmodule