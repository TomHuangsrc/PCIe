// PHY_rx_lane.v
// Description:
// This is the first part of PHY receiver who's responsible for receiving data from physical lines, decoding and
// deskewing. It starts with receiving data bits from transmission lines. Then by shifting in data_bit into a
// 10-bit shift register data_in_shift, the lane keeps monitoring symbols in this shift register and once a COM
// has been detected for the first time, logic for dividing high frequency clock by 10 is activated and a short
// pulse will be kept generated every 10 cycles of high frequency clock with help of Xilinx clocking buffer primitive
// BUFGCE (this is how clocking gating is commonly done on FPGA). After the slow clock is activated, decoder will
// keep taking back-to-back 10-bit symbols from this data_in_shift. Also, you can try to do this by generating a
// 50% duty cycle slow clock and use an intermediate 10-bit buffer (will be explained in ppt). After a 10-bit symbol
// is decoded, it enters elastic buffer and then deskew FIFO, and finally valid symbol will be provided to LTSSM or
// receiver buffer.
//
// Reference: Intel patent: LANE TO LANE DESKEWING VIA NON-DATA SYMBOL PROCESSING FOR A SERIAL POINT TO POINT LINK
// Link: https://patents.google.com/patent/US20050141661A1/en

module PHY_rx_lane(
    input clk_sys,
	input clk_w_x10,				// shoule be a revocered clock in real implementation, 10x frequency of local clock
	input clk_r_local,				// local clock used in Elastic buffer and everything after
	input rstn_asyn,				// asynchronous reset signal
	input w_en,						// indicates that there's valid transmission going on on physical transmission lines
	input data_bit,					// 1-bit data from physical lines for this lane
//	input deskew_en,				// indicate that deskew logic is enabled
	input [2:0] window_cnt,     	// counter for "count 4" mechanism
	input FIFO_all_not_empty,		// FIFOs of all lanes are not empty, so they can be read
	output deskew_fifo_not_empty,	// tell top module FIFO of this lane is not empty, used to generate FIFO_all_not_empty
	output [7:0] lane_data,			// 8-bit data from deskew FIFO of this lane
	output TS_obtained				// indicates that this lane has a COM of a Training Set detected
	);

	localparam COM_M	= 10'b0101_111100;			// K.28.5 
	localparam COM_P	= 10'b1010_000011; 

	wire symbol_lock_detected;		// a COM is detected in current symbol
	reg symbol_lock_reg;			// a COM has been detected 

	wire clk_w_slow;				// gated slow clock, functioning as write clock for EB
	wire clk_w_slow_en;				// enable signal for gating logic
	reg [9:0] data_in_shift;		// 10-bit shift register at the entrance of this lane
	reg [3:0] clk_divider;			// counter of clock dividing logic

	wire [7:0] decoded_data;		// 8-bit decoded symbol
	wire decoder_valid;				// current symbol from decoder is valid
	wire control_data, receiver_error_detected;			// more information for current decoded symbol

	wire EB_w_en;					// EB write enable signal
	reg EB_r_en_s, EB_r_en_ss;		// double synchronizing SL to enable reading of EB (write clk domain to read)
	wire [7:0] EB_r_data;			// data read from EB
	wire SOS_exiting_EB;			// indicates that current COM read from EB is start of a SKP ordered set
	wire com_exiting_EB;			// indicates that current data read from EB is a COM

// Task 1: Generate symbol_lock_detected to indicate a COM is detected in current 10-bit shift register
	assign symbol_lock_detected = (data_in_shift == COM_M) || (data_in_shift == COM_P);	 // a COM is detected
// ----------------------------------------------------------------------------------------------------

	assign clk_w_slow_en = (clk_divider==9) || symbol_lock_detected;	// enable slow clock at every valid 10-bit symbol edge
	assign EB_w_en = decoder_valid && !receiver_error_detected;			// write into EB when data from decoder is valid

	// activate slow clock generation when COM has been detected
// Task 2: Fill in correct clk signal for this block
	always @(posedge clk_w_x10, negedge rstn_asyn)
// ----------------------------------------------------------------------------------------------------
	if (!rstn_asyn)
	begin
		data_in_shift <= 0;
		symbol_lock_reg <= 0;
		clk_divider <= 0;
	end
	else if (w_en)
	begin
// Task 3: Complete 10-bit symbol shift in shift register
		data_in_shift <= {data_bit,data_in_shift[9:1]};
// ----------------------------------------------------------------------------------------------------
		if (symbol_lock_detected && !symbol_lock_reg)
		begin
			symbol_lock_reg <= 1;
		end
		// clock divider is activated 1 clk later after COM detected, so enable clk_w_slow when clk_divider==9 instead of 0
		if (symbol_lock_reg)		
		begin
			if (clk_divider == 9)
				clk_divider <= 0;
			else
				clk_divider <= clk_divider + 1;
		end
	end

	// do CDC from write clock of EB to read clock. because symbol lock signal functions as EB write enable (check into decoder
	// how decoder_valid is generated for reference), we do double synchronization from symbol lock signal to generate EB read
	// enable signal, and keep EB written and read since then. because there're about 2 read clk delay after write enable first
	// goes valid and then read enable, we fill EB entries upon reset by half full - 2 slots
// Task 4: Fill in correct clk signal for this block
	always @(posedge clk_r_local, negedge rstn_asyn)
// ----------------------------------------------------------------------------------------------------
	if (!rstn_asyn)
	begin
		EB_r_en_s <= 0;
		EB_r_en_ss <= 0;
	end
	else
	begin
		EB_r_en_s <= symbol_lock_reg;
		EB_r_en_ss <= EB_r_en_s;
	end

	// need to consider about the possibility that as clk_w_slow is derived from clk_w_x10, 
	// posedge of clk_w_slow is a little later than that of clk_w_x10, which has the potential
	// to cause hold violation when using clk_w_slow to latch the whole symbol in data_in_shift

	// Xilinx primitive module, clocking buffer with enable, commonly used for clocking gating on board
	// https://www.xilinx.com/support/documentation/user_guides/ug572-ultrascale-clocking.pdf, page 29
	// On the datasheet, pay attention to synthesis attribute SYNC/ASYNC (SYNC by default)
	BUFGCE bufgce_clk_w_slow 
		(      
			.I		(clk_w_x10),					// high frequency recovered clock
			.CE		(clk_w_slow_en),				// 1: input clk enabled through    0: output suppressed to 0
			.O		(clk_w_slow)    				// gated output clock
		);

	decoder_10b_8b decoder
		(
			.clk                     (clk_w_slow),				// derived low frequency clock from recovered clock
			.resetn                  (rstn_asyn),
			.valid_in                (w_en),					// transmission valid on physical line
			.data_in                 (data_in_shift),			// 10-bit shift register providing 10-bit input symbol
			.data_out                (decoded_data),			// 8-bit decoded symbol
			.valid_out               (decoder_valid),			// indicates validity of decoded symbols
			.control_data            (control_data),			// currently not used
			.receiver_error_detected (receiver_error_detected)	// currently not handled
		);

	elastic_buffer #(
			.POINTER_WIDTH(3)						// EB depth is determined by this parameter: 2**POINTER_WIDTH
		) elastic_buffer (
		    .clk_sys         (clk_sys),				// to serve as a sampling clock for the ILA
			.clk_w_slow		 (clk_w_slow),			// write clock (derived from high frequency recovered clock)
			.clk_r_local     (clk_r_local),			// read clock (local clock)
			.resetn          (rstn_asyn),
			.w_en            (EB_w_en),				// write enable, derived from symbol lock signal
			.w_data          (decoded_data),		// decoded data from decoder
			.r_en            (EB_r_en_ss),			// read enable, derived from symbol lock signal
			.r_data          (EB_r_data),			// data from EB entries
			.SOS_exiting	 (SOS_exiting_EB),		// inform deskew FIFO that current symbol read is the COM of an SOS (Skip Ordered Set)
			.com_exiting 	 (com_exiting_EB),		// inform deskew FIFO that current symbol read is a COM
			.TS_obtained	 (TS_obtained)			// indicates at least one training set has been detected, help with "count 4" mechanism
		);

	deskew_fifo deskew_fifo
		(
		    .clk_sys         (clk_sys),				    // to serve as a sampling clock for the ILA
			.clk_r_local            (clk_r_local),				// single clock FIFO
			.rstn           (rstn_asyn),	
			.window_cnt      (window_cnt),				// enable the FIFO to be written and read
			.SOS_ahead		(SOS_exiting_EB),			// current input symbol is the COM of an SOS
			.com_ahead		(com_exiting_EB),			// current input symbol is a COM
			.w_data         (EB_r_data),				// current input symbol from EB
			.r_en           (FIFO_all_not_empty),		// FIFOs of all lanes are not empty - current entry can be read
			.r_data         (lane_data),				// data symbol of current entry to be read
			.fifo_not_empty (deskew_fifo_not_empty)		// indicates this FIFO is not empty - help generate FIFO_all_not_empty
		);

endmodule // PHY_rx_lane
