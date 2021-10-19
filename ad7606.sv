module ad7606
(
	input        			clk,           //25Mhz
	input        			fd,            //25kHz for test
	input        			rst_n,	

	input  		      	ad_douta,   	//ADC 1
	input  		        	ad_doutb, 		//ADC 2

	output reg 				ad_cs1, 
	output reg 				ad_cs2,   

	output reg   			ad_reset,           
	output reg    			ad_cnv,	        	        
	output reg          	ad_sclk,			//serial clock

	//ADC1
	output reg	[15:0]  	ad_ch1,             
	output reg	[15:0] 	ad_ch2,              
	output reg	[15:0] 	ad_ch3,           
	output reg	[15:0] 	ad_ch4,             
	output reg	[15:0] 	ad_ch5,              
	output reg	[15:0] 	ad_ch6,             
	output reg	[15:0] 	ad_ch7,             
	output reg	[15:0] 	ad_ch8,
	
	//ADC2
	output reg	[15:0]  	ad_ch9,             
	output reg	[15:0] 	ad_ch10,              
	output reg	[15:0] 	ad_ch11,           
	output reg	[15:0] 	ad_ch12,             
	output reg	[15:0] 	ad_ch13,              
	output reg	[15:0] 	ad_ch14,             
	output reg	[15:0] 	ad_ch15,             
	output reg	[15:0] 	ad_ch16,
	
	//ADC3
	output reg	[15:0]  	ad_ch17,             
	output reg	[15:0] 	ad_ch18,              
	output reg	[15:0] 	ad_ch19,           
	output reg	[15:0] 	ad_ch20,             
	output reg	[15:0] 	ad_ch21,              
	output reg	[15:0] 	ad_ch22,             
	output reg	[15:0] 	ad_ch23,             
	output reg	[15:0] 	ad_ch24,
	
	//ADC4
	output reg	[15:0]  	ad_ch25,             
	output reg	[15:0] 	ad_ch26,              
	output reg	[15:0] 	ad_ch27,           
	output reg	[15:0] 	ad_ch28,             
	output reg	[15:0] 	ad_ch29,              
	output reg	[15:0] 	ad_ch30,             
	output reg	[15:0] 	ad_ch31,             
	output reg	[15:0] 	ad_ch32,

	output reg				ad_done,
	output      [3:0]   	state_s,
	
	output					temp1_s,
	output					temp2_s,
	output					temp3_s,
	
	//output      [7:0]   	i_s
	output      [15:0]   counter1_s	
	
	);


reg [7:0] 	i;
reg [7:0] 	bit_cnt;
reg [3:0] 	state;
reg [15:0]  cnt;
reg [127:0] adc1_t;	//Temp reg for ADC1
reg [127:0] adc2_t;	//Temp reg for ADC2
reg [127:0] adc3_t;	//Temp reg for ADC3
reg [127:0] adc4_t;	//Temp reg for ADC4

reg [7:0] 	tim_cs2;
reg [7:0] 	tim_5us;

reg [15:0] 	counter1;

reg temp1; 
reg temp2; 
reg temp3; 

parameter 	IDLE	    =4'd0;
parameter 	CONV	  	 =4'd1;
parameter 	WAIT_5us  =4'd2;
parameter 	READ_ADC  =4'd3;
parameter 	READ_DONE =4'd4;
parameter 	WAIT_FD_N =4'd5;

initial begin
	i				<=8'd0;
	cnt			<=16'd0;
	ad_cs1		<=1'b1;	//normal 1
	ad_cs2		<=1'b1;	//normal 1
	ad_reset		<=1'b0;
	ad_cnv      <=1'b1;	//normal 1
	ad_sclk		<=1'b1;	//normal 1
	tim_5us		<=8'd0;
	bit_cnt		<=8'd0;
	ad_done		<=1'b0;
	adc1_t      <=128'd0;
	adc2_t      <=128'd0;
	adc3_t      <=128'd0;
	adc4_t      <=128'd0;
	state			<=4'd0;
	tim_cs2		<=8'd0;
	
	temp1<=0;
	temp2<=0;
	temp3<=0;
	
	counter1		<=16'd0;
	
	ad_ch1 <=0;             
	ad_ch2 <=0;             
	ad_ch3 <=0;          
	ad_ch4 <=0;             
	ad_ch5 <=0;              
	ad_ch6 <=0;             
	ad_ch7 <=0;             
	ad_ch8 <=0;
	ad_ch9 <=0;            
	ad_ch10<=0;              
	ad_ch11<=0;          
	ad_ch12<=0;             
	ad_ch13<=0;             
	ad_ch14<=0;            
	ad_ch15<=0;             
	ad_ch16<=0;
end


assign state_s = state;
assign temp1_s = temp1;
assign temp2_s = temp2;
assign temp3_s = temp3;
assign counter1_s = counter1;

//always@(posedge clk or posedge ad_reset) begin
always@(posedge clk) begin
	
	//sync clock
	temp1 <= fd;
	temp2 <= temp1;
	temp3 <= temp2;
	
//	if (ad_reset) begin 
//		state     <=4'd0; 
//		ad_cs1    <=1'b1;
//		ad_cs2    <=1'b1;
//		ad_sclk   <=1'b1; 
//		ad_cnv    <=1'b1;
//		counter1	 <=16'd0;
//	end
//	else begin
	
		case(state)
		
			4'd0: begin	//IDLE
				i			<=8'd0;	
				state		<=4'd0;
				ad_cs1	<=1'b1;
				ad_cs2	<=1'b1;
				ad_sclk	<=1'b1; 
				ad_cnv	<=1'b1;
				adc1_t  	<=128'd0;
				adc2_t  	<=128'd0;
				adc3_t  	<=128'd0;
				adc4_t  	<=128'd0;
				tim_cs2	<=8'd0;
				
				if(temp3==1'b1) begin
					state<=4'd1;
				end
			end
		
			4'd1: begin //CONV            
				if(i<9) begin                   
					i<=i+1'b1;
					ad_cnv<=1'b0; 
				end
				else begin		 
					ad_cnv<=1'b1;  
					state<=4'd2;	
				end
			end
			
			4'd2: begin //BUSY & CS1
				if(tim_5us<8'd125) begin
					tim_5us <=tim_5us+1'b1;
					if(tim_5us==8'd100)
						ad_cs1 <=1'b0;  
				end
				else begin
					tim_5us <=8'd0;
					state <=4'd3;
				end
			end
			
			4'd3: begin //READ ADC1 & ADC2                       
				if(i==6) begin	//4
					ad_sclk	<=1'b1;
					i<=0;	
					if(bit_cnt<8'd127) begin	 
						adc1_t	<={adc1_t[126:0],ad_douta};
						adc2_t	<={adc2_t[126:0],ad_doutb};
						bit_cnt	<=bit_cnt+1'b1;
					end
					else begin
						adc1_t	<={adc1_t[126:0],ad_douta};
						adc2_t	<={adc2_t[126:0],ad_doutb};
						bit_cnt	<=8'd0;
						state		<=4'd4;
					end
				end
				else begin
					i<=i+1'b1;
					if(i==4)//2
						ad_sclk<=1'b0;	
				end			
			end
			
			4'd4: begin	//SAVE DATA
				ad_sclk	<=1'b1;	 
				ad_cs1	<=1'b1;
				
				//ADC1
				ad_ch1	<=adc1_t[127:112];
				ad_ch2	<=adc1_t[111:96];
				ad_ch3	<=adc1_t[95:80];
				ad_ch4	<=adc1_t[79:64];
				ad_ch5	<=adc1_t[63:48];
				ad_ch6	<=adc1_t[47:32];
				ad_ch7	<=adc1_t[31:16];
				ad_ch8	<=adc1_t[15:0];
				
				//ADC2
				ad_ch9	<=adc2_t[127:112];
				ad_ch10	<=adc2_t[111:96];
				ad_ch11	<=adc2_t[95:80];
				ad_ch12	<=adc2_t[79:64];
				ad_ch13	<=adc2_t[63:48];
				ad_ch14	<=adc2_t[47:32];
				ad_ch15	<=adc2_t[31:16];
				ad_ch16	<=adc2_t[15:0];
				
				//ad_done	<=1'b1;
				state		<=4'd5;
				i			<=8'd0;
				bit_cnt	<=8'd0;	
			end
			
			//CS2
			4'd5: begin
				ad_cs2 <=1'b0; 
				if(tim_cs2 < 8'd25) begin
					tim_cs2 <=tim_cs2+1'b1;
				end
				else begin
					tim_cs2 <=8'd0;
					state <=4'd6;
				end
			end
			
			4'd6: begin //READ ADC3 & ADC4                       
				if(i==6) begin	//4
					ad_sclk	<=1'b1;
					i<=0;	
					if(bit_cnt<8'd127) begin	 
						adc3_t	<={adc3_t[126:0],ad_douta};
						adc4_t	<={adc4_t[126:0],ad_doutb};
						bit_cnt	<=bit_cnt+1'b1;
					end
					else begin
						adc3_t	<={adc3_t[126:0],ad_douta};
						adc4_t	<={adc4_t[126:0],ad_doutb};
						bit_cnt	<=8'd0;
						state		<=4'd7;
					end
				end
				else begin
					i<=i+1'b1;
					if(i==4)//2
						ad_sclk<=1'b0;	
				end			
			end
			
			4'd7: begin	//SAVE DATA
				ad_sclk	<=1'b1;	 
				ad_cs2	<=1'b1;
				
				//ADC3
				ad_ch17	<=adc3_t[127:112];
				ad_ch18	<=adc3_t[111:96];
				ad_ch19	<=adc3_t[95:80];
				ad_ch20	<=adc3_t[79:64];
				ad_ch21	<=adc3_t[63:48];
				ad_ch22	<=adc3_t[47:32];
				ad_ch23	<=adc3_t[31:16];
				ad_ch24	<=adc3_t[15:0];
				
				//ADC4
				ad_ch25	<=adc4_t[127:112];
				ad_ch26	<=adc4_t[111:96];
				ad_ch27	<=adc4_t[95:80];
				ad_ch28	<=adc4_t[79:64];
				ad_ch29	<=adc4_t[63:48];
				ad_ch30	<=adc4_t[47:32];
				ad_ch31	<=adc4_t[31:16];
				ad_ch32	<=adc4_t[15:0];
				
				ad_done	<=1'b1;
				state		<=4'd8;
			end

			4'd8: begin
				ad_done	<=1'b0;
				if(temp3==1'b0) begin
					state<=4'd0;
				end
			end
		
			default: state<=4'd0;
		endcase
	//end
end

endmodule		
		
		
		
		
//			4'd0: begin
//				ad_cs1	<=1'b1;
//				ad_cs2	<=1'b1;
//				ad_sclk	<=1'b1; 
//				ad_cnv	<=1'b1;
//				adc1_t  	<=128'd0;
//				i			<=8'd0;	
//
//				if(temp3==1'b1) begin
//					state<=4'd1;
//				end
//			end
//			
//			4'd1: begin
//				if(i<8'd9) begin                   
//					i<=i+1'b1;
//					ad_cnv<=1'b0; 
//				end
//				else begin		 
//					ad_cnv<=1'b1;  
//					state<=4'd2;	
//				end
//			end
//			
//			4'd2: begin
//				if(tim_5us<8'd124) begin
//					tim_5us    <=tim_5us+1'b1;
//					if(tim_5us==8'd100)
//						ad_cs1 	  <=1'b0;  
//				end
//				else begin
//					tim_5us    <=8'd0;
//					state      <=4'd3;
//				end
//			end	
//			
//			4'd3: begin //READ_ADC                       
//				if(i==4) begin	//4
//					ad_sclk	<=1'b1;
//					i<=0;	
//					if(bit_cnt<8'd127) begin	 
//						adc1_t	<={adc1_t[126:0],ad_douta};
//						bit_cnt	<=bit_cnt+1'b1;
//					end
//					else begin
//						adc1_t	<={adc1_t[126:0],ad_douta};
//						bit_cnt	<=8'd0;
//						state		<=4'd4;
//					end
//				end
//				else begin
//					i			<=i+1'b1;
//					if(i==2)
//						ad_sclk	<=1'b0;	
//				end			
//			end
//			
//			4'd4: begin 
//				ad_sclk	<=1'b1;	 
//				ad_cs1	<=1'b1;
//				ad_ch1	<=adc1_t[127:112];
//				ad_ch2	<=adc1_t[111:96];
//				ad_ch3	<=adc1_t[95:80];
//				ad_ch4	<=adc1_t[79:64];
//				ad_ch5	<=adc1_t[63:48];
//				ad_ch6	<=adc1_t[47:32];
//				ad_ch7	<=adc1_t[31:16];
//				ad_ch8	<=adc1_t[15:0];
//				ad_done	<=1'b1;
//				state		<=4'd5;
//			end
//			
//			4'd5: begin
//				ad_done	<=1'b0;
//				//if(fd==1'b0) begin
//				if(temp3==1'b0) begin
//					state<=4'd0;
//				end
//			end
			
//			default: state<=4'd0;
//		endcase
//	end
//end
//
//endmodule

