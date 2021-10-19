`timescale 1ns/1ps

module fifo_asy #(parameter   DATA_WIDTH    = 8,
										ADDRESS_WIDTH = 12,
										FIFO_DEPTH    = (1 << ADDRESS_WIDTH))
(
	//Reading port
	output reg  [DATA_WIDTH-1:0]        dout, 
	output reg                          empty,
	input wire                          rd,
	input wire                          rd_clk, 

	//Writing port.	 
	input wire  [DATA_WIDTH-1:0]        din,  
	output reg                          full,
	input wire                          wr,
	input wire                          wr_clk,
	
	output reg [ADDRESS_WIDTH :0] 		cnt,

	input wire                          rst);

	/////Internal connections & variables//////
	reg   [DATA_WIDTH-1:0]              Mem [FIFO_DEPTH-1:0];
	wire  [ADDRESS_WIDTH-1:0]           pNextWordToWrite, pNextWordToRead;
	wire                                EqualAddresses;
	wire                                NextWriteAddressEn, NextReadAddressEn;
	wire                                Set_Status, Rst_Status;
	reg                                 Status;
	wire                                PresetFull, PresetEmpty;

	wire  [ADDRESS_WIDTH-1:0] 				wr_cnt;
	wire  [ADDRESS_WIDTH-1:0] 				rd_cnt;
	
	gray2bin gray2bin_wr( .G(pNextWordToWrite), .bin(wr_cnt) );
	gray2bin gray2bin_rd( .G(pNextWordToRead),  .bin(rd_cnt) );

	
	
	always @(*) begin  //pointer difference is evaluated for both clock edges 
		if(wr_cnt > rd_cnt)
			cnt <= wr_cnt - rd_cnt;
		else if(rd_cnt > wr_cnt)
			cnt <= rd_cnt - wr_cnt;
		else 
			cnt <=0; 
	end
	
	
    
	//////////////Code///////////////
	//Data ports logic:
	//(Uses a dual-port RAM).
	//'Data_out' logic:
	always @ (posedge rd_clk) begin
		/*if(rst)
			cnt <= 0;*/
		if(rd & !empty) begin
			dout <= Mem[pNextWordToRead];
			//cnt  <= cnt - 1;
		end
	end	
            
	//'Data_in' logic:
	always @ (posedge wr_clk) begin
		/*if(rst)
			cnt <= 0;*/
		if(wr & !full)
			Mem[pNextWordToWrite] <= din;
			//cnt <= cnt + 1;
	end

	 //Fifo addresses support logic: 
    //'Next Addresses' enable logic:
    assign NextWriteAddressEn = wr & ~full;
    assign NextReadAddressEn  = rd  & ~empty;
           
    //Addreses (Gray counters) logic:
    GrayCounter GrayCounter_pWr
       (.GrayCount_out(pNextWordToWrite),
        .Enable_in(NextWriteAddressEn),
        .Clear_in(rst),
        .Clk(wr_clk)
       );
       
    GrayCounter GrayCounter_pRd
       (.GrayCount_out(pNextWordToRead),
        .Enable_in(NextReadAddressEn),
        .Clear_in(rst),
        .Clk(rd_clk)
       );
     

    //'EqualAddresses' logic:
    assign EqualAddresses = (pNextWordToWrite == pNextWordToRead);

    //'Quadrant selectors' logic:
    assign Set_Status = (pNextWordToWrite[ADDRESS_WIDTH-2] ~^ pNextWordToRead[ADDRESS_WIDTH-1]) &
                        (pNextWordToWrite[ADDRESS_WIDTH-1]  ^ pNextWordToRead[ADDRESS_WIDTH-2]);
                            
    assign Rst_Status = (pNextWordToWrite[ADDRESS_WIDTH-2]  ^ pNextWordToRead[ADDRESS_WIDTH-1]) &
                        (pNextWordToWrite[ADDRESS_WIDTH-1] ~^ pNextWordToRead[ADDRESS_WIDTH-2]);
                         
    //'Status' latch logic:
    always @ (Set_Status, Rst_Status, rst) //D Latch w/ Asynchronous Clear & Preset.
        if (Rst_Status | rst)
            Status = 0;  //Going 'Empty'.
        else if (Set_Status)
            Status = 1;  //Going 'Full'.
            
    //'Full_out' logic for the writing port:
    assign PresetFull = Status & EqualAddresses;  //'Full' Fifo.
    
    always @ (posedge wr_clk, posedge PresetFull) //D Flip-Flop w/ Asynchronous Preset.
        if (PresetFull)
            full <= 1;
        else
            full <= 0;
            
    //'Empty_out' logic for the reading port:
    assign PresetEmpty = ~Status & EqualAddresses;  //'Empty' Fifo.
    
    always @ (posedge rd_clk, posedge PresetEmpty)  //D Flip-Flop w/ Asynchronous Preset.
        if (PresetEmpty)
            empty <= 1;
        else
            empty <= 0;
            
endmodule
