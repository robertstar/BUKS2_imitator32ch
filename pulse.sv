module pulse(
					input 		clk,	//25Mhz
					input 		fd,   //5kHz
					output reg 	out
);

reg [3:0] state_p;

initial begin
	state_p<=4'd0;
end


//25Mhz
always@(posedge clk) begin
	case(state_p)
		4'd0: begin
			if(fd==1'b1) begin
				out		<=1'b1;
				state_p	<=4'd1;
			end
		end
		
		4'd1: begin
			out		<=1'b0;
			state_p	<=4'd2;
		end
		
		4'd2: begin
			if(fd==1'b0) begin
				state_p	<=4'd0;
			end
		end
	
		default: state_p<=4'd0;
	endcase
end


endmodule
