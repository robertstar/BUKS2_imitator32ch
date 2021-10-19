module fifo2 # (parameter wbits = 8, words = 4096)
(
	input 				    		clk,
	input 				    		rst,
	input 				    		rd,
	input 				    		wr,
	input 			[wbits-1:0] din,
	output reg		[wbits-1:0]	dout,
	output logic 		    		empty,
	output logic 			 		full,
	output			[15:0] 		cnt
);

int i;
reg [15:0] tail;
reg [15:0] head;
reg [15:0] count;
reg [wbits-1:0] FIFO [words-1:0]; 

assign cnt = count;

initial begin
	tail<=16'd0;
	head<=16'd0;
	count<=16'd0;
end

always @(posedge clk) begin
	//if (rst == 1) begin
	if (!rd) begin
		dout <= 8'd0;
	end
	else begin
		dout <= FIFO[tail];
	end
end

always @(posedge clk) begin
	if (rst == 1'b0) begin
		if (wr == 1'b1 && full == 1'b0)
			FIFO[head] <= din;
	end
end

always @(posedge clk) begin
	if (rst == 1'b1) begin
		head <= 0;
	end
	else begin
		if (wr == 1'b1 && full == 1'b0) begin
			head <= head + 1; //Write
		end
	end
end

always @(posedge clk) begin
	if (rst == 1'b1) begin
		tail <= 0;
	end
	else begin
		if (rd == 1'b1 && empty == 1'b0) begin
			tail <= tail + 1; //Read
		end
	end
end

always @(posedge clk) begin
	if (rst == 1'b1) begin
		count <= 0;
	end
	else begin
		case ({rd, wr})
			2'b00: count <= count;
			2'b01://Write
				if (!full)
					count <= count + 1;
			2'b10://Read
				if (!empty)
					count <= count - 1;
			2'b11://Write & Read
				count <= count;
		endcase
	end
end

always @(count) begin
	if (count == 0)
		empty = 1'b1;
	else
		empty = 1'b0;
	end

always @(count) begin
	if (count < words)
		full = 1'b0;
	else
		full = 1'b1;
end

endmodule
