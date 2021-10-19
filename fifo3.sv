module fifo3 # (parameter wbits = 8, words = 4096)
(
	input 				    		rst,
	input 				    		rd_clk,
	input 				    		wr_clk,
	input 				    		rd_en,
	input 				    		wr_en,
	input 			[wbits-1:0] din,
	output reg		[wbits-1:0]	dout,
	output logic 		    		empty,
	output logic 			 		full,
	output			[15:0] 		cnt,
	
	output			[15:0] 		tail_cnt,
	output			[15:0] 		head_cnt
);

int i;
reg [11:0] tail;
reg [11:0] head;
reg [15:0] count;
reg [wbits-1:0] FIFO [words-1:0]; 

reg [15:0] wr_cnt;
reg [15:0] rd_cnt;
reg [15:0] cnt1;
reg [15:0] cnt2;

assign cnt = count;
assign tail_cnt = tail;
assign head_cnt = head;

initial begin
	tail<=12'd0;
	head<=12'd0;
	count<=16'd0;
	rd_cnt<=16'd0;
	wr_cnt<=16'd0;
	cnt1<=16'd0;
	cnt2<=16'd0;
end


always @(posedge rd_clk) begin
	if (!rd_en) begin
		dout <= 8'd0;
	end
	else begin
		dout <= FIFO[tail];
	end
end

always @(posedge rd_clk) begin
	if (rst == 1'b1) begin
		tail <= 0;
	end
	else begin
		if (rd_en == 1'b1 && empty == 1'b0) begin
			tail <= tail + 1; //Read
		end
	end
end	
	
always @(posedge wr_clk) begin
	if (rst == 1'b0) begin
		if (wr_en == 1'b1 && full == 1'b0)
			FIFO[head] <= din;
	end
end	
	
always @(posedge wr_clk) begin
	if (rst == 1'b1) begin
		head <= 0;
	end
	else begin
		if (wr_en == 1'b1 && full == 1'b0) begin
			head <= head + 1; //Write
		end
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

always @(*) begin  //pointer difference is evaluated for both clock edges 
	if(head > tail)
		count <= head - tail;
	else if(tail > head)
		count <= (16'd4096-(tail)+head);
	else 
		count <=0;
end	
	

endmodule
