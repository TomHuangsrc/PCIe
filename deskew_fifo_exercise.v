// deskew_fifo.v
// Diandian Chen, 05/16/2018, 7/13/2018
//
// Description:
// It implements the deskew FIFO. Deskew FIFO comes after EB. 
// This FIFO looks into data symbols from EB and throws away SoS (Skip Ordered Sets).
// It generates fifo_not_empty signal to upper module. 
// Until upper module tells this Deskew Fifo that Deskew Fifos of all lanes are not empty,
// neither the front-of-the-line location gets read nor the read-pointer gets incremented.
// After the deskew Fifo, data from different lanes are aligned and can be directly 
// read by later blocks.

module deskew_fifo
#(
	parameter FIFO_DEPTH_LOG2 = 2		// used to configure the depth of this FIFO
)(
	input clk_sys,				// to serve as a sampling clock for the ILA
	input clk_r_local,					// local clock of this port
	input rstn,
//	input deskew_en,			// can start to detect incoming COM and begin to deskew when "count 4" finished
	input [2:0] window_cnt,     // counter for "count 4" mechanism
	input SOS_ahead,			// indicating symbols coming out from EB are SKP Ordered Set
	input com_ahead,			// indicating symbol coming out from EB is a COM
	input [7:0] w_data,			// input data from EB
	input r_en,					// read enable signal, high only when all FIFOs are not empty
	output [7:0] r_data,		// data read from this FIFO
	output fifo_not_empty		// indicates that this FIFO is not empty. help generating FIFO_all_not_empty
);	

	localparam SKP	= 8'b00011100;			// K.28.0

	reg [7:0] fifo [0:2**FIFO_DEPTH_LOG2-1];	// depth of FIFO is 2**FIFO_DEPTH_LOG2
	reg [FIFO_DEPTH_LOG2:0] w_ptr, r_ptr;		// n+1 read/write pointers of the FIFO
	reg deskew_begin;							// FIFO only begins to be written after "count 4" is finished and then a COM has been detected
	integer i;

	ila_deskew ila_deskew
	 (
		 .clk(clk_sys),
		 .probe0(clk_r_local),
		 .probe1(window_cnt),
		 .probe2(SOS_ahead),
		 .probe3(com_ahead),
		 .probe4(w_data),
		 .probe5(r_en),
		 .probe6(r_data),
		 .probe7(fifo_not_empty)
	 );	
	
	assign deskew_en = window_cnt[2];		// after "count 4", deskew logic may start to work (we also check com_ahead)
// Task 1: Complete assignments to read data from deskew fifo
	assign r_data = fifo[r_ptr[1:0]];	// data to be read is always pointed to by r_ptr
// Task 2: Generate the signal to indicate fifo is not empty
	assign fifo_not_empty = (w_ptr == r_ptr) ? 0 : 1 ;	// actually NOT necessary to generate "full" if maximum lane-to-lane skew is well defined

	// -------------------------------------------------------------------------
	// following signals for fillcount and full are just for monitoring purpose
	wire [FIFO_DEPTH_LOG2:0] fillcount = w_ptr - r_ptr;
	wire empty = !fifo_not_empty;
	wire full = fillcount[FIFO_DEPTH_LOG2];
	// -------------------------------------------------------------------------

	always @(posedge clk_r_local, negedge rstn)
	if (!rstn)
	begin
		w_ptr <= 0;
		r_ptr <= 0;
		deskew_begin <= 0;
		for (i=0; i<2**FIFO_DEPTH_LOG2; i=i+1)
			fifo[i] <= 0;
	end
	else
	begin
		if (deskew_en && com_ahead)						// after "count 4" and a COM has been detected, start the whole logic
			deskew_begin <= 1;
		if (deskew_begin || (deskew_en&&com_ahead))		// trigger "examine and write" below when and after deskew_begin activated
		begin
			// Task 3: if input data is "COM of an SOS" or "SKP", don't write
            if (w_data != SKP && !SOS_ahead)
			begin
				fifo[w_ptr[FIFO_DEPTH_LOG2-1:0]] <= w_data;
				w_ptr <= w_ptr + 1;
			end
		end
		if (r_en)					// read is enabled when all Deskew FIFOs are not empty (i.e. when none of the Deskew Fifos is empty).
		begin
			r_ptr <= r_ptr + 1;
		end
	end

endmodule