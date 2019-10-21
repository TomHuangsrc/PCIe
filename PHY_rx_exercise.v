// PHY_rx.v
// Diandian Chen, 05/17/2018, 7/13/2018
//
// Description:
// This is the top module for PHY rx. It has two lanes and one unstriping buffer, interfacing with physical transmission
// lines, LTSSM and DLL. This is where "count 4" mechanism happens. Find corresponding ppt for reference.
//
// Brief explanation of "count 4":
// "4" is because the maximum lane-to-lane skew defined in spec is 4. In our design, EB begins to be written and read right
// after symbol lock has been obtained for this lane, but we have to let data streams of different lanes enter corresponding
// deskew FIFO at an appropriate time, as we want all FIFOs to offer data of the same group (say symbol 0, 1, 2, 3... are
// identically put on different lanes at TX side, and lane 0 gets its symbol lock when it gets 4 but lane 1 gets its symbol
// lock when it gets 15. Then what we want is something like, instead of one first filled with 4 and the other 15, but both
// deskew FIFOs are first filled by 15 and then 16, 17... to make sure that whatever read from both FIFOs at the same time
// is always aligned as how they were meant to be sent on TX side). To realize this, when all lanes have their respective
// symbol lock and all lanes are ready to send out data, we wait until the "4 clocks skew window" passes and then start to
// let data enter deskew FIFO. Find more details in ppt.

module PHY_rx(
    input clk_sys,
	input clk_w_x10a,				// recovered clock from lane 0 (here we simulate with a side band clock signal)
	input clk_w_x10b,				// recovered clock from lane 1 (here we simulate with a side band clock signal)
	input clk_r_local,				// local clock of nearly the same frequency as recovered clock divided by 10
	input rstn_asyn,
	input w_en,						// indidcates that there's valid transmission going on on physical lines
	input throttle,					// throttling signal given by DLL, we assume Transaction Layer never stops receiving so it's always 0
	input data_bit0,				// data bit put on lane 0 transmission line
	input data_bit1,				// data bit put on lane 1 transmission line
	output [15:0] data_to_DLL,		// grouped 2 byte data sent to DLL
	output [2:0] boundary,			// boundary info for corresponding 2 byte data
	output type_TLP,				// indicate packet type of current packet to DLL
	output [7:0] lane0_data,		// one byte data from deskew FIFO of lane 0, sent to LTSSM
	output lane0_valid,				// validity of lane 0 data
	output [7:0] lane1_data,		// one byte data from deskew FIFO of lane 0, sent to LTSSM
	output lane1_valid				// validity of lane 1 data
	);

	localparam COM = 8'b10111100;	// K.28.5
	localparam SKP = 8'b00011100;	// K.28.0

	wire FIFO_all_not_empty;		// when FIFOs of all lanes are not empty, data in FIFOs are valid to be read
	reg [2:0] window_cnt;			// counter for "count 4" mechanism

	wire TS_obtained0, TS_obtained1;						// indicates corresponding lane has detected an input Training Set
	wire TS_obtained_all;									// all lanes have obtained a TS, so start "count 4" mechanism
	// wire deskew_en;											// after "count 4", deskew FIFO can begin to catch COM and get filled up from then on
	wire deskew_fifo_not_empty0, deskew_fifo_not_empty1;	// indicates corresponding lane has a filled FIFO

	assign TS_obtained_all = TS_obtained0 && TS_obtained1;
	//assign deskew_en = window_cnt[2];						// after "count 4", deskew logic start to work
	assign FIFO_all_not_empty = deskew_fifo_not_empty0 && deskew_fifo_not_empty1;

	assign lane0_valid = FIFO_all_not_empty;				// all FIFOs are not empty - data of all lanes are valid
	assign lane1_valid = FIFO_all_not_empty;

	always @(posedge clk_r_local, negedge rstn_asyn)
	if (!rstn_asyn)
	begin
		window_cnt <= 1;									// enable signal for this counter is a registered value that has 1 clk delay from combinational logic "all saw COM"
	end														// so start with 1
	else
// Task 1: Fill in appropriate condition for "count 4" mechanism to work
	begin
		if (TS_obtained_all && window_cnt < 4)				// if all lanes have their symbol lock and TS detected, and "count 4" isn't yet finished
			window_cnt <= window_cnt + 1;					// count one more
	end
// ----------------------------------------------------------------------------------------------------

	// instantiations of two lanes
	PHY_rx_lane inst_PHY_rx_lane0
		(
		    .clk_sys                    (clk_sys),
			.clk_w_x10             		(clk_w_x10a),				// recovered high frequency clock
			.clk_r_local           		(clk_r_local),				// local clock
			.rstn_asyn             		(rstn_asyn),
			.w_en                  		(w_en),						// transmission valid on physical line
			.data_bit              		(data_bit0),				// input bit from physical line
			.window_cnt       			(window_cnt),				// enable logic of deskew FIFO of this lane
			.FIFO_all_not_empty    		(FIFO_all_not_empty),		// enable the FIFO to be read when all FIFOs are not empty
			.deskew_fifo_not_empty 		(deskew_fifo_not_empty0),	// indicates FIFO of this lane is not empty	
			.lane_data			   		(lane0_data),				// data read from this lane
			.TS_obtained				(TS_obtained0)				// indicates this lane has a TS detected
		);

	PHY_rx_lane inst_PHY_rx_lane1
		(
		    .clk_sys                    (clk_sys),
			.clk_w_x10             		(clk_w_x10b),
			.clk_r_local           		(clk_r_local),
			.rstn_asyn             		(rstn_asyn),
			.w_en                  		(w_en),
			.data_bit              		(data_bit1),
			.window_cnt       			(window_cnt),
			.FIFO_all_not_empty    		(FIFO_all_not_empty),
			.deskew_fifo_not_empty 		(deskew_fifo_not_empty1),
			.lane_data			   		(lane1_data),
			.TS_obtained				(TS_obtained1)	
		);

	// instantiation of unstripipng buffer
	rx_buffer inst_rx_buffer
		(
			.clk         (clk_r_local),				// local clock
			.resetn      (rstn_asyn),
			.throttle    (throttle),				// throttling signal from DLL. we assume our Transaction Layer is always accepting data and this signal is never activated
			.valid_in    (FIFO_all_not_empty),		// only possible to write when data read from FIFO of two lanes are valid - two FIFOs are both not empty
			.lane_data0  (lane0_data),				// data byte from lane 0
			.lane_data1  (lane1_data),				// data byte from lane 1
			.data_to_DLL (data_to_DLL),				// grouped 2-byte data to DLL
			.boundary    (boundary),				// corresponding boundary info of current 2 bytes data
			.type_TLP    (type_TLP)					// corresponding packet type info of current 2 bytes data
		);

endmodule