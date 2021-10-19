module fifo # (parameter wbits = 8, words = 4096)( 
	input 						clk, 
	input 						rst,
	input 						rd, 
	input 						wr, 
	input 	  	[wbits-1:0] dataIn, 
	output reg 	[wbits-1:0] dataOut, 
	output 						empty, 
	output 						full,
	output		[15:0]		cnt
); 


reg [15:0]  Count = 0; 
reg [wbits-1:0] FIFO [words-1:0]; 
//reg [wbits-1:0] FIFO [words-1:0]; 
reg [15:0]  readCounter; 
reg [15:0]  writeCounter;

initial begin
	readCounter	 =16'd0;
	writeCounter =16'd0;
end
				

assign empty = (Count==0)?1'b1:1'b0; 
assign full  = (Count==words)?1'b1:1'b0; 
assign cnt   = Count;

always@(posedge clk) begin 

	if (rst) begin 
		readCounter  =16'd0;
		writeCounter =16'd0;
	end 

	else if (rd ==1'b1 && Count!=0) begin 
		dataOut  = FIFO[readCounter]; 
		readCounter = readCounter+1; 
	end 

	else if (wr==1'b1 && Count<words) begin
		FIFO[writeCounter]  = dataIn; 
		writeCounter  = writeCounter+1; 
	end 
	//else; 


	if (writeCounter==words) 
		writeCounter=16'd0;
	else if (readCounter==words) 
		readCounter=16'd0;
	//else;

	if (readCounter > writeCounter) begin 
		Count=readCounter-writeCounter; 
	end 

	else if (writeCounter > readCounter) 
		Count=writeCounter-readCounter; 
	//else;
end 

endmodule


