`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
 
// Design Name: 
// Module Name:    pcie_phy_8b10b 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 8 bit to 10 bit encoding 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module pcie_phy_8b10b(clk,
								rst,
								datain_tx_unencoded,
								control_tx,
								dataout_tx_encoded
    );

input clk, rst;
input [7:0]datain_tx_unencoded;			// 8 bit character to be encoded at transmitter
input control_tx;					 //  D/K# for incoming character at transmitter

output reg [9:0]dataout_tx_encoded;		// 10 bit encoded symbol at transmitter

reg running_disparity;						// running disparity -- 7/7/2018 previously it was named prev_running_disparity!
wire [9:0] dataout_tx_encoded_int_rdplus, dataout_tx_encoded_int_rdminus;	// 10 bit encoded data for +1 and -1 running disparity


pcie_phy_10b8b_bram #(
    .ADDR_WIDTH(9),
    .MEM_INIT_FILE("nplus.data")
) encode_8b10brdplus (
    .clk(clk), 
    .wea(1'b0), 
    .addra(9'h000), 
    .dina(10'h000), 
    .addrb({control_tx,datain_tx_unencoded}), 
    .doutb(dataout_tx_encoded_int_rdplus));

pcie_phy_10b8b_bram #(
    .ADDR_WIDTH(9),
    .MEM_INIT_FILE("nminus.data")
) encode_8b10brdminus (
    .clk(clk), 
    .wea(1'b0), 
    .addra(9'h000), 
    .dina(10'h000), 
    .addrb({control_tx,datain_tx_unencoded}), 
    .doutb(dataout_tx_encoded_int_rdminus));


always @(posedge clk, posedge rst)
begin

    if(rst)
    begin
	// running disparity : 0 ( -1) and running disparity :1 (1). Initialize to -1
        running_disparity <= 1'b0;
    end
    
    else
    begin
         
	// TASK 1: Include missing condition in if statement and complete running disparity assignments)

 	// Equal number of 1’s and 0’s in 10 bit encoded data, do not affect running disparity.
	// 10b encoding ensures one of the following three scenarios
	// 	1) Number of 1’s = Number pf 0’s ( disparity = 0)
	// 	2) Number of 1’s is two more than Number of 0’s (disparity = +2)
	//	3) Number of 0’s is two more than Number of 1’s (disparity = -2)
	// Note: 1) Running disparity is different from disparity of 10 bit encoded data
	//	 2) 10 bit 0’s is used to indicate electrical idle on the lanes. Running disparity must not change in this case

         if( (^dataout_tx_encoded[9:0]) ||(dataout_tx_encoded == 10'b0000000000))
            begin
               running_disparity <= running_disparity;
            end
         else
             begin
               running_disparity <= ~running_disparity; 
             end

   	// END TASK 1
    end
end


always @(*)
begin

    if(running_disparity == 1'b0)
        begin // running disparity is -1

                    dataout_tx_encoded = dataout_tx_encoded_int_rdminus;
        end	  
					
    else // running disparity is 1
         begin
 
                     dataout_tx_encoded = dataout_tx_encoded_int_rdplus;
         end     
end



endmodule
