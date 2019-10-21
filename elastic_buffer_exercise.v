// elastic_buffer.v
// Designed by: Diandian Chen 
// Date: 04/28/2018, 7/14/2018
// Module description and design plan reviewed by: Gandhi Puvvada

// Description:
// Elastic buffer (EB) is implemented here using a register array. 
// The goal of the elastic buffer is to maintain its depth at half-full,
// so that it can accommodate for tendencies to overflow or underflow 
// because of the 600 ppm difference in Tx and Rx local clocks. 

// While we say that the EB should start initially half-filled in the "prime method" 
// in reality, upon reset, the buffer is set to "half-full minus two" state 
// by setting the rd_ptr to 0 and wr_ptr to "half-depth minus 2". 
// Initial contents in all entries are set to SKP (skip symbol), because SKP will be 
// thrown away later by deskew FIFO. Deskew FIFO always ignores SKP symbols. 
// So the first "half-depth - 2" amount of entries read out are all the dummy SKP symbols 
// that won't do any harm. 

// Elastic buffer is a two-clock-FIFO and, as such, pointers are exchanged 
// across clock domain in gray code. 
// As pointers are double-synchronized, there's always a "2-clock delay (lab)" 
// for one domain to know the other domain's pointer.
// And, unlike in the case of a FIFO between a producer and a consumer, here, 
// on every clock of writer's clock, a symbol is written into the EB and 
// on every clock of the reader's clock a symbol is read. 
// So, we reduce the depth (fill count) obtained by doing WP - RP_SS, by a 2.
//                  fillcount_w = w_ptr - (r_ptr_b_ss + 2) 
// 
// In our design, we chose to make the elastic buffer an 8-deep Fifo, 
// but you know that the depth of the elastic buffer should be very carefully determined 
// based on the maximum symbol shift in EB expected
// based on the 600 ppm difference in the clocks and also maximum size of the TLP supported in the design.
// Reference: https://www.mindshare.com/files/resources/mindshare_pcie_elastic_buffer.pdf 
// The above reference however fails to realize the following corner case (discovered by Bo, Gandhi, and Diandian).
// The corner case is a result of the uncertainty issues associated with the read pointer
// double-synchronization, when it (the rp) is led to the writer side of EB. 
// The writer-side of the EB may receive either the latest read pointer or slightly obsolete read pointer.
// The obsolete read pointer is the one which is one-less than the latest read pointer value.
// Hence fill-count can be slightly more than what it should be. Suppose the correct fill-count is 3 but 
// the writer-side evaluated it as 4 (because of the uncertainty) and did not add a SKP 
// (though it was going through a SOS and it has an opportunity to add a SKP to the EB at that time if it had 
// known that the actual fill count is 3 and not 4). Right after this SOS, a long TLP of 4096 length started.
// This would result in (4096+28)/1666 = 2.475 symbol shift accumulation meaning the fill count which was at 3
// could have potentially shrunk to 3 - 2.475 symbols = 0.525 symbols. 
// If fill-count is less than one but more than zero (as is the case above), i t means that the EB becomes empty
//  momentarily  but the writer manages to write  a symbol before the reader tries to read the next symbol.
// So it looks like we narrowly missed reading garbage from the EB. We (Bo, Gandhi, and Diandian) think that 
// it is too close for comfort. This corner case assumes that the transmitter is slower by 600 ppm.

// Another corner case can be thought of for the other case when the transmitter is 600 ppm higher in frequency
// and the EB started showing tendencies of overflowing and we are trying to delete one or two SKPs at a time. 
// Let us assume that the real depth of the EB is 4 and then the corner case depicted in the above reference came 
// where 3.4 symbol shift has accumulated before we had a chance to delete SKPs to correct the situation.
//      Reproduced from the reference:
//          [{(4096 + 28) + 1538} / 1666] = 3.4
// So, fill count reached would be 4 + 3.4 = 7.4. Well, this is not yet 8, but this means the FIFO becomes 
// full on alternative clocks and 
// before the writer over-writes the oldest element in the EB, luckily (somewhere in the middle of the symbol time)
// the reader just whisked it away! So, that oldest element, that was about to be slaughtered by the incoming symbol,
// was luckily saved. We (Bo, Gandhi, and Diandian) think that it is too close for comfort. 
// So we conclude that we should increase the EB size slightly (to 16, as 16 is the next power of two)  
// to safely cater for a maximum TLP size of 4K.

// In our design we actually made the two local clocks, 10 MHz and 9.8 MHz (much higher than 600 ppm) 
// but we scheduled a SOS every 80 decimal symbols. Note that 80 is more than 49 (49 = 98/2). 
// We wanted to show correction by adding or deleting 2 SKPs sometimes by slowing down the SOS scheduling.  So, in our design, 
// the two EBs at one end of the link are working with writer's clock of 10 MHz and reader's clock of 9.8 MHz,
// whereas the other two EBs at the other end of the link are working with writer's clock of 9.8 MHz and reader's clock of 9.8 MHz. 
// This substantial clock difference and frequent SOS scheduling made it possible to see our SOSs and EBs in action
// without needing us to scroll through a very long waveform.

// Extract from pcie_tx_data_serializer.v
/* 				// number of symbols after which SKP ordered set is scheduled dependent on TX and RX clock ppm
				if(phy_tx_char_cnt >= 8'h50)
					begin
							phy_tx_char_cnt <= 8'b00000000;
							phy_tx_skp_sched <= phy_tx_skp_sched + 1;
					end
 */


// If interested, you can play around with different PPM, SOS scheduling duration, elastic buffer depth, 
// and thresholds for inserting/deleting SKP symbols. It's ideal to have a depth of 2's
// power (like 8 or 16) for the EB, as it'll be much more convenient and efficient to do pointer 
// incrementing and depth calculation. 
// It's highly recommended to get familiar enough with elastic buffer and SKP ordered set mechanism, as it's actually
// used in many transmission protocols and has the potential to be catchy to interviewers.
//
// Attention:
// Because we have the assumption that tx keeps scheduling SKP ordered sets as expected, we never need to worry
// about overflow/underflow as any symbol shifted can be compensated soon enough, so we don't really need to
// check for full or empty for this buffer. And as the decision to insert/remove SKP symbols is always made
// on the write side, it's enough to only have rd_ptr getting converted and sampled to enter write domain.

module elastic_buffer
#(
	// These parameters were meant to help paramiterize this module by simply following the given equation for
	// depth of this elastic buffer. But since our project is quite a simplified version of the protocol, the
	// actual key parameter here is only BUFFER_DEPTH, and the other two can be simply ignored.
	// parameter MAX_PAYLOAD_SIZE 	= 128,
	// parameter MAX_SHIFT_ROUNDED	= $rtoi($ceil((MAX_PAYLOAD_SIZE+1566)/1666.0)),
	// parameter BUFFER_DEPTH		= (MAX_SHIFT_ROUNDED>2)? MAX_SHIFT_ROUNDED*2 : 4	// minimum depth set to 4
	parameter POINTER_WIDTH = 3		// buffer depth is 2**POINTER_WIDTH
									// Actually n+1 wide pointers are NOT necessary here, but we apply this idea to conveniently
									// generate full/empty signals for monitoring purpose
)(
	input clk_sys,
	input clk_w_slow,			// derived from recovered x10 clk, by dividing the recovered clk by 10
	input clk_r_local,			// local clk for read
	input resetn,				
	input w_en,					// enables writing into this EB, derived from symbol lock signal of this lane 
	input [7:0] w_data,			// input data from decoder
	input r_en,					// enables reading from this EB, arrives about 2 read clocks later than w_en
	output [7:0] r_data,		// data to be read in current entry
	output SOS_exiting,			// a SKP Ordered Set is exiting EB
	output com_exiting,			// a COM symbol is exiting EB
	output reg TS_obtained		// there's a non-SKP ordered set detected when reading EB, so data can get ready to enter deskew FIFO
);	

	localparam BUFFER_DEPTH		= 2**POINTER_WIDTH;

	localparam COM	= 8'b10111100;			// K.28.5
	localparam SKP	= 8'b00011100;			// K.28.0
	
	// function integer log2;							// ceiling log2 function for the compiler
	// 	input integer value;
	// 	begin
	// 		value = value - 1;
	// 		for (log2=0; value>0; log2=log2+1)
	// 			value = value >> 1;
	// 	end
	// endfunction

	function [POINTER_WIDTH:0] b_to_g;			// synthesizable binary-to-gray function
		input [POINTER_WIDTH:0] data_in_b;
		integer i;
		begin
			b_to_g[POINTER_WIDTH] = data_in_b[POINTER_WIDTH];
		  	for (i=POINTER_WIDTH-1; i>=0; i=i-1)
		  	begin
				b_to_g[i] = (data_in_b[i] != data_in_b[i+1]);
			end
		end
	endfunction

	function [POINTER_WIDTH:0] g_to_b;			// synthesizable gray-to-binary function
		input [POINTER_WIDTH:0] data_in_g;
		integer i;
		begin
			g_to_b[POINTER_WIDTH] = data_in_g[POINTER_WIDTH];
			for (i=POINTER_WIDTH-1; i>=0; i=i-1)
			begin
				g_to_b[i] = (data_in_g[i] != g_to_b[i+1]);
			end
		end	
	endfunction

	reg [7:0] e_buffer [0:BUFFER_DEPTH-1];			// elastic buffer implemented as register array

	reg [POINTER_WIDTH:0] w_ptr, r_ptr;				// original pointers in their respective domain
	reg [POINTER_WIDTH:0] r_ptr_p1;					// always equals to r_ptr+1, for looking into the next symbol when r_ptr pointing to a COM
	reg [POINTER_WIDTH:0] r_ptr_g_s, r_ptr_g_ss;	// sampled gray code of original read pointer
	wire [POINTER_WIDTH:0] r_ptr_g, r_ptr_b_ss;		// converted gray code of original read pointer, converted binary code of sampled read pointer

	wire [POINTER_WIDTH:0] fillcount_w = w_ptr-(r_ptr_b_ss+2);	// elastic buffer won't be full, so actually don't need the fillcount to represent full count
																// PS: we have 2 clk delay between r_ptr and r_ptr_b_ss, and r_ptr always increments every
																// clock in EB, so we decrease fillcount_w by 2 more to get a closer value to (w_ptr - r_ptr).
																// you can compare r_ptr_b_ss and r_ptr in simulation waveforms
	wire [POINTER_WIDTH:0] r_ptr_p1_next = r_ptr_p1 + 1;		// value of r_ptr + 2, or r_ptr_p1 + 1, for updating r_ptr_p1
	wire [POINTER_WIDTH:0] w_ptr_p1 = w_ptr + 1;				// value of write pointer + 1
	wire [POINTER_WIDTH:0] w_ptr_p2 = w_ptr + 2;				// value of write pointer + 2, used when 1 or 2 SKP symbols are determined to be inserted
	wire [POINTER_WIDTH:0] w_ptr_p3 = w_ptr + 3;				// value of write pointer + 3, used when 2 SKP symbols are determined to be inserted

	reg com_detected, skp_detected;			// to form up a state machine that records SOS detection history and makes decision to insert/remove SKP
	wire com_at_r_ptr, skp_at_r_ptr_p1;		// indicating a SKP ordered set at exit of EB when both are high

	integer i;								// for reseting contents of EB

	ila_EB ila_EB
	 (
	 .clk(clk_sys),
	 .probe0(clk_w_slow),
	 .probe1(fillcount_w),
	 .probe2(com_detected),
	 .probe3(skp_detected),
	 .probe4(w_ptr),
	 .probe5(r_ptr),
	 .probe6(r_ptr_b_ss)
	 );
// Task 1: Fill in correct signals to complete b-to-g/g-to-b conversion
	assign r_ptr_g = b_to_g(r_ptr);				    // converts binary read pointer into gray code	
	assign r_ptr_b_ss = g_to_b(r_ptr_g_ss);			// converts gray code read pointer into binary
// ----------------------------------------------------------------------------------------------------
	assign com_at_r_ptr = (e_buffer[r_ptr[POINTER_WIDTH-1:0]]==COM);			// checking whether current exiting entry is a COM
	assign skp_at_r_ptr_p1 = (e_buffer[r_ptr_p1[POINTER_WIDTH-1:0]]==SKP);		// checking whether the next entry to exit is a SKP
	assign SOS_exiting = com_at_r_ptr && skp_at_r_ptr_p1;						// an exiting COM followed by SKP means an exiting SOS
	assign com_exiting = com_at_r_ptr;											// a COM is exiting

	assign r_data = e_buffer[r_ptr[POINTER_WIDTH-1:0]];			// data read out is always pointed to by r_ptr

	// --------------------------------------------------------------------------------------------------------------------------------------------
	// following gray coding and double synchronization of w_ptr and full/empty signals are actually NOT necessary and are just for monitoring
	reg [POINTER_WIDTH:0] w_ptr_g_s, w_ptr_g_ss;	// sampled gray code of original read pointer
	wire [POINTER_WIDTH:0] w_ptr_g, w_ptr_b_ss;		// converted gray code of original read pointer, converted binary code of sampled read pointer
	wire [POINTER_WIDTH:0] fillcount_r = (w_ptr_b_ss+2)-r_ptr;		// elastic buffer won't be full, so don't need the fillcount to represent full count
	wire empty = (w_ptr_b_ss==r_ptr);
	wire full = fillcount_w[POINTER_WIDTH];
	assign w_ptr_g = b_to_g(w_ptr);
	assign w_ptr_b_ss = g_to_b(w_ptr_g_ss);
	// --------------------------------------------------------------------------------------------------------------------------------------------

// Task 2: Fill correct clk signal for this block
	always @(posedge clk_w_slow, negedge resetn)
// ----------------------------------------------------------------------------------------------------
	if (!resetn)
	begin
		w_ptr <= BUFFER_DEPTH/2 - 2;		// fillcount initialized to be half-full - 2, because there're 2 clk delay between w_en and r_en
											// due to generating r_en for EB by double synchronizing symbol_lock_reg
		r_ptr_g_s <= 0;
		r_ptr_g_ss <= 0;
		com_detected <= 0;
		skp_detected <= 0;
		for (i=0; i<BUFFER_DEPTH; i=i+1)
			e_buffer[i] <= SKP;				// contents in buffer initialized to be SKP (dummy)
	end
	else
	begin
		r_ptr_g_s <= r_ptr_g;
		r_ptr_g_ss <= r_ptr_g_s;			// double-synchronization
		if (w_en)
		begin
			case ({skp_detected, com_detected})
				2'b00:
				// keep monitoring whether the incoming symbol is start of an ordered set
				begin
					if (w_data == COM)
						com_detected <= 1;
					w_ptr <= w_ptr_p1;
					e_buffer[w_ptr[POINTER_WIDTH-1:0]] <= w_data;
				end
				2'b01:
				// after the first COM, see if the 2nd symbol is SKP to check if this is a SKP ordered set. if not, clear com_detected
				begin
					if (w_data == SKP)
						skp_detected <= 1;
					else
						com_detected <= 0;
					w_ptr <= w_ptr_p1;
					e_buffer[w_ptr[POINTER_WIDTH-1:0]] <= w_data;
				end
				2'b11:
				// this is a SKP ordered set. decide whether to insert or remove SKP symbols or simply write as normal
				begin
					com_detected <= 0;
					skp_detected <= 0;			// will be set back to 1 if it's decided that 2 SKP symbols need to be removed
					if (fillcount_w < BUFFER_DEPTH/2 - 1)		// insert 2 more SKPs
// Task 3: Complete the process of "inserting 2 more SKP symbols"
					begin
						e_buffer[w_ptr[POINTER_WIDTH-1:0]] <= w_data;
						e_buffer[w_ptr_p1[POINTER_WIDTH-1:0]] <= SKP;
						e_buffer[w_ptr_p2[POINTER_WIDTH-1:0]] <= SKP;
						w_ptr <= w_ptr_p3;
					end
// ----------------------------------------------------------------------------------------------------
					if (fillcount_w == BUFFER_DEPTH/2 - 1)	// insert 1 more SKP
					begin
						e_buffer[w_ptr[POINTER_WIDTH-1:0]] <= w_data;
						e_buffer[w_ptr_p1[POINTER_WIDTH-1:0]] <= SKP;
						w_ptr <= w_ptr_p2;
					end
					if (fillcount_w == BUFFER_DEPTH/2)		// normal write
					begin
						e_buffer[w_ptr[POINTER_WIDTH-1:0]] <= w_data;
						w_ptr <= w_ptr_p1;
					end
					if (fillcount_w == BUFFER_DEPTH/2 + 1)	// remove 1 SKP
					begin
						w_ptr <= w_ptr;			
					end	
					if (fillcount_w > BUFFER_DEPTH/2 + 1)		// remove 2 SKPs, to be finished in next state
					begin
						w_ptr <= w_ptr;
						skp_detected <= 1;		// set back to 1 to move to state 2'b10
					end
				end
				2'b10:		
				// use the state {skp_detected, com_detected} == 2'b10 to remove one more SKP
				begin
					w_ptr <= w_ptr;
					skp_detected <= 0;
				end
				default: {skp_detected, com_detected} <= 0;
			endcase
		end
	end

// Task 4: Fill correct clk signal for this block
	always @(posedge clk_r_local, negedge resetn)			// simple read pointer incrementation in read domain
// ----------------------------------------------------------------------------------------------------
	if (!resetn)
	begin
		r_ptr <= 0;			// reset to 0 to help initialize the buffer to half-full - 2
		r_ptr_p1 <= 1;
		TS_obtained <= 0;	// no Training Set has entered EB at first

		// following double synchronization of w_ptr is actually NOT necessary
		w_ptr_g_s <= 0;
		w_ptr_g_ss <= 0;
	end
	else
	begin
		w_ptr_g_s <= w_ptr_g;
		w_ptr_g_ss <= w_ptr_g_s;
		if (r_en)
		begin
			r_ptr <= r_ptr_p1;
			r_ptr_p1 <= r_ptr_p1_next;
		end
		if (!TS_obtained)	// when a non-SOS ordered set has been detected, validate this signal to help "count 4" mechanism
			if (com_at_r_ptr && !skp_at_r_ptr_p1)
				TS_obtained <= 1;
	end

endmodule // elastic_buffer