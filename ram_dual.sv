module ram_dual
(
	input [7:0] din,
	input [11:0] rd_addr, wr_addr,
	input wr, rd, wr_clk, rd_clk,
	//output reg [7:0] dout
	output [7:0] dout
);

	// Declare the RAM variable
	reg [7:0] ram[4095:0];
	
	always @ (posedge wr_clk)
	begin
		// Write
		if (wr)
			ram[wr_addr] <= din;
	end
	
	/*always @ (posedge rd_clk)
	begin
		// Read 
		if (rd)
			dout <= ram[rd_addr];
	end*/
	
	
	assign dout = (rd)?ram[rd_addr]:8'hE1;
	//assign dout = ram[rd_addr];
	
	
endmodule
