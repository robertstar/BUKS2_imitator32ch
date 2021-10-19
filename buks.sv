module buks(
					input 				clk_i,
					
					input 				crs_dv_a,
					input 				crs_dv_b,
										
					input 				rx_dv_a,
					input 				rx_dv_b,
					
					input 				rx_clk_a,				
					input 				rx_clk_b,
					
					input 				tx_clk_a,
					input 				tx_clk_b,
					
					//input 				eth_phy_clk,
					
					input wire  [3:0] rxd_a,
					input wire  [3:0] rxd_b,
					
					output reg 			eth_phy_rst_n,
					
					output reg			pwrd_down_a,
					output reg			pwrd_down_b,
					
					output 				tx_en_a,
					output 				tx_en_b,
					
					output wire [3:0]	txd_a,
					output wire [3:0]	txd_b,
	
	
					
					/*input					ad_douta,
					output 				ad_cs,               
					output 				ad_reset,           
					output 				ad_convstab,		
					output 				ad_sclk,	
					output				fd_clk	*/
					
					
					//AD7606
					output 				AD_CNV,
					output 				AD_RST,
					output 				AD_SCLK,
					output 				AD_CS1,	//ADC1 & ADC2
					output 				AD_CS2,	//ADC3 & ADC4
					input					AD_DOUT1,
					input					AD_DOUT2,
					input					AD_FD,
					
					//ETH address
					input wire  [4:0] IP_ADDR
);


wire clk_50Mhz;

pll pll_inst(
		.inclk0(clk_i),
		.c0(clk_50Mhz)
);

reg [31:0] 	cnt_rst;
reg [3:0]  	alg_rst;
reg [47:0] 	eth_dest;
reg [47:0] 	eth_src;
reg [15:0] 	eth_type;
reg [47:0] 	sender_mac;
reg [31:0] 	sender_ip;
reg [31:0] 	target_ip;

reg			adc_ok;
reg [3:0]	adc_state;
reg [15:0]	byte_cnt;
reg [31:0]	adc_timer;
reg [7:0] 	adc_data;

reg [31:0]	fd_tim;
reg			fd;
wire			puls;

reg [31:0]  IP_local;


reg [15:0]ad_ch1;            
reg [15:0]ad_ch2;              
reg [15:0]ad_ch3;           
reg [15:0]ad_ch4;            
reg [15:0]ad_ch5;            
reg [15:0]ad_ch6;            
reg [15:0]ad_ch7;             
reg [15:0]ad_ch8;
reg [15:0]ad_ch9;            
reg [15:0]ad_ch10;              
reg [15:0]ad_ch11;           
reg [15:0]ad_ch12;            
reg [15:0]ad_ch13;            
reg [15:0]ad_ch14;            
reg [15:0]ad_ch15;             
reg [15:0]ad_ch16;
reg [15:0]ad_ch17;              
reg [15:0]ad_ch18;           
reg [15:0]ad_ch19;            
reg [15:0]ad_ch20;            
reg [15:0]ad_ch21;            
reg [15:0]ad_ch22;             
reg [15:0]ad_ch23;
reg [15:0]ad_ch24;            
reg [15:0]ad_ch25;            
reg [15:0]ad_ch26;             
reg [15:0]ad_ch27;
reg [15:0]ad_ch28;            
reg [15:0]ad_ch29;            
reg [15:0]ad_ch30;             
reg [15:0]ad_ch31;
reg [15:0]ad_ch32;


reg 		 ad_done;

reg [15:0]test_signal;
reg [7:0] test_cnt;
reg [31:0]pkt_cnt;


//for imitate cordic
localparam An = 32000/1.647;
reg [15:0] Xin, Yin;

//32 chanells
reg [31:0] angle_51Hz;
reg [31:0] angle_61Hz;
reg [31:0] angle_71Hz;
reg [31:0] angle_81Hz;
reg [31:0] angle_91Hz;
reg [31:0] angle_101Hz;
reg [31:0] angle_201Hz;
reg [31:0] angle_301Hz;

/*
reg [31:0] angle_401Hz;
reg [31:0] angle_501Hz;
reg [31:0] angle_601Hz;
reg [31:0] angle_701Hz;
reg [31:0] angle_801Hz;
reg [31:0] angle_901Hz;
reg [31:0] angle_951Hz;
reg [31:0] angle_1001Hz;*/


initial begin
	eth_phy_rst_n<=1'b1;
	cnt_rst<=32'd0;
	alg_rst<=4'd0;
	adc_ok<=1'b0;
	adc_state<=4'd0;
	byte_cnt<=16'd0;
	adc_timer<=32'd0;
	pwrd_down_a<=1'b1;//Normal
	pwrd_down_b<=1'b1;//Normal
	
	fd<=1'b0;
	fd_tim<=32'd0;
	
	test_signal<=16'h4000;
	test_cnt<=8'd0;
	pkt_cnt<=32'd0;
	
	IP_local<={8'd192, 8'd168, 8'd4, 8'd100};
	//IP_local<={8'd192, 8'd168, 8'd10, 8'd100};
	
	//cordic
	Xin<=An;
	Yin<=0;
	angle_51Hz	<=32'd0;
	angle_61Hz	<=32'd0;
	angle_71Hz	<=32'd0;
	angle_81Hz	<=32'd0;
	angle_91Hz	<=32'd0;
	angle_101Hz	<=32'd0;
	angle_201Hz	<=32'd0;
	angle_301Hz	<=32'd0;
	
	/*angle_401Hz	<=32'd0;
	angle_501Hz	<=32'd0;
	angle_601Hz	<=32'd0;
	angle_701Hz	<=32'd0;
	angle_801Hz	<=32'd0;
	angle_901Hz	<=32'd0;
	angle_951Hz	<=32'd0;
	angle_1001Hz<=32'd0;*/
end

//assign fd_clk = fd;
assign ad_done_s = ad_done;

//for test fd adc
always@(posedge tx_clk_b) begin
	if(fd_tim < 32'd499) begin
		fd_tim<=fd_tim+1'b1;
	end
	else begin
		fd_tim<=32'd0;
		fd<=~fd;//25kHz
	end
end

always@(posedge fd) begin
	test_cnt<=test_cnt+1'b1;
	if(test_cnt==8'd5)begin
		test_signal[15]<=~test_signal[15];//invert sign
		test_cnt<=8'd0;
	end
end

always@(posedge AD_FD) begin
	Xin <= 32000/1.647;
	angle_51Hz	<=angle_51Hz   + 32'd21907217;
	angle_61Hz	<=angle_61Hz   + 32'd26202749;
	angle_71Hz	<=angle_71Hz   + 32'd30498282;
	angle_81Hz	<=angle_81Hz   + 32'd34793814;
	angle_91Hz	<=angle_91Hz   + 32'd39089347;
	angle_101Hz	<=angle_101Hz  + 32'd43384880;
	angle_201Hz	<=angle_201Hz  + 32'd86340206;
	angle_301Hz	<=angle_301Hz  + 32'd129295532;
	
	/*angle_401Hz	<=angle_401Hz  + 32'd172250859;
	angle_501Hz	<=angle_501Hz  + 32'd215206185;
	angle_601Hz	<=angle_601Hz  + 32'd258161512;
	angle_701Hz	<=angle_701Hz  + 32'd301116838;
	angle_801Hz	<=angle_801Hz  + 32'd344072164;
	angle_901Hz	<=angle_901Hz  + 32'd387027491;
	angle_51Hz	<=angle_951Hz  + 32'd408505154;
	angle_1001Hz<=angle_1001Hz + 32'd429982817;*/
end


wire [15:0] test_51Hz;
cordic cordic_51Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_51Hz),
	.sin(test_51Hz),
	.cos()
);

wire [15:0] test_61Hz;
cordic cordic_61Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_61Hz),
	.sin(test_61Hz),
	.cos()
);

wire [15:0] test_71Hz;
cordic cordic_71Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_71Hz),
	.sin(test_71Hz),
	.cos()
);

wire [15:0] test_81Hz;
cordic cordic_81Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_81Hz),
	.sin(test_81Hz),
	.cos()
);

wire [15:0] test_91Hz;
cordic cordic_91Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_91Hz),
	.sin(test_91Hz),
	.cos()
);

wire [15:0] test_101Hz;
cordic cordic_101Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_101Hz),
	.sin(test_101Hz),
	.cos()
);

wire [15:0] test_201Hz;
cordic cordic_201Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_201Hz),
	.sin(test_201Hz),
	.cos()
);

wire [15:0] test_301Hz;
cordic cordic_301Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_301Hz),
	.sin(test_301Hz),
	.cos()
);

/*
wire [15:0] test_401Hz;
cordic cordic_401Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_401Hz),
	.sin(test_401Hz),
	.cos()
);

wire [15:0] test_501Hz;
cordic cordic_501Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_501Hz),
	.sin(test_501Hz),
	.cos()
);

wire [15:0] test_601Hz;
cordic cordic_601Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_601Hz),
	.sin(test_601Hz),
	.cos()
);

wire [15:0] test_701Hz;
cordic cordic_701Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_701Hz),
	.sin(test_701Hz),
	.cos()
);

wire [15:0] test_801Hz;
cordic cordic_801Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_801Hz),
	.sin(test_801Hz),
	.cos()
);

wire [15:0] test_901Hz;
cordic cordic_901Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_901Hz),
	.sin(test_901Hz),
	.cos()
);

wire [15:0] test_951Hz;
cordic cordic_951Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_951Hz),
	.sin(test_951Hz),
	.cos()
);

wire [15:0] test_1001Hz;
cordic cordic_1001Hz
(
	.clk(AD_FD),
	.x_start(Xin),
	.y_start(Yin),
	.angle(angle_1001Hz),
	.sin(test_1001Hz),
	.cos()
);*/



//ADC AD7606 module

ad7606 adc1
(
	.clk					(tx_clk_b),    //25Mhz
	//.fd					(fd),          //25kHz for test
	.fd					(AD_FD),			//FD
	.rst_n				(1'b1),			//reset
		
	.ad_douta			(AD_DOUT1), 	//ADC1 serial in
	.ad_doutb			(AD_DOUT2),		//ADC2 serial in
	
	.ad_cs1				(AD_CS1),     
	.ad_cs2				(AD_CS2),
	
	.ad_reset			(AD_RST),           
	.ad_cnv				(AD_CNV),		//convert start
	.ad_sclk				(AD_SCLK),		//serial clock
	
	.ad_ch1				(ad_ch1),             
	.ad_ch2				(ad_ch2),              
	.ad_ch3				(ad_ch3),           
	.ad_ch4				(ad_ch4),             
	.ad_ch5				(ad_ch5),              
	.ad_ch6				(ad_ch6),             
	.ad_ch7				(ad_ch7),             
	.ad_ch8				(ad_ch8),
	.ad_ch9				(ad_ch9),             
	.ad_ch10				(ad_ch10),              
	.ad_ch11				(ad_ch11),           
	.ad_ch12				(ad_ch12),             
	.ad_ch13				(ad_ch13),              
	.ad_ch14				(ad_ch14),             
	.ad_ch15				(ad_ch15),             
	.ad_ch16				(ad_ch16),
	.ad_ch17				(ad_ch17),              
	.ad_ch18				(ad_ch18),           
	.ad_ch19				(ad_ch19),             
	.ad_ch20				(ad_ch20),              
	.ad_ch21				(ad_ch21),             
	.ad_ch22				(ad_ch22),             
	.ad_ch23				(ad_ch23),
	.ad_ch24				(ad_ch24),              
	.ad_ch25				(ad_ch25),             
	.ad_ch26				(ad_ch26),             
	.ad_ch27				(ad_ch27),
	.ad_ch28				(ad_ch28),              
	.ad_ch29				(ad_ch29),             
	.ad_ch30				(ad_ch30),             
	.ad_ch31				(ad_ch31),
	.ad_ch32				(ad_ch32),

	.ad_done				(ad_done)
);

//send adc data to mac
always@(posedge tx_clk_b) begin
	
	//Check IP address set/unset jumpers
	if(~IP_ADDR==5'd0) begin
		IP_local<={8'd192, 8'd168, 8'd4, 8'd100};
		//IP_local<={8'd192, 8'd168, 8'd10, 8'd100};
	end
	else begin
		IP_local<={8'd192, 8'd168, 8'd4, 3'd0, ~IP_ADDR};
		//IP_local<={8'd192, 8'd168, 8'd10, 3'd0, ~IP_ADDR};
	end
	

	case(adc_state)
		4'd0: begin
			byte_cnt		<=16'd0;
			adc_ok		<=1'b0;
			if(ad_done) begin
				adc_state	<=4'd1; //send adc data
				//pkt_cnt		<=pkt_cnt+1'b1;
			end
		end
		
		4'd1: begin
			case(byte_cnt)
			
				/****************************************************************************/
				//Packet cnt 4bytes
//				16'd0: begin
//					adc_ok	<=1'b1;
//					byte_cnt	<=byte_cnt+1'b1;	
//					adc_data	<=pkt_cnt[7:0];
//				end
//				
//				16'd1: begin
//					byte_cnt	<=byte_cnt+1'b1;	
//					adc_data	<=pkt_cnt[15:8];
//				end
//				
//				16'd2: begin
//					byte_cnt	<=byte_cnt+1'b1;	
//					adc_data	<=pkt_cnt[23:16];
//				end
//				
//				16'd3: begin
//					byte_cnt	<=byte_cnt+1'b1;	
//					adc_data	<=pkt_cnt[31:24];
//				end

				/****************************************************************************/
				//ADC
				//Chanel1
				16'd0: begin
					adc_ok	<=1'b1;
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch1[7:0];
					adc_data	<=test_51Hz[7:0];
				end
				
				16'd1: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch1[15:8];
					adc_data	<=test_51Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel2
				16'd2: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch2[7:0];
					adc_data	<=test_61Hz[7:0];
				end
				
				16'd3: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch2[15:8];
					adc_data	<=test_61Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel3
				16'd4: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch3[7:0];
					adc_data	<=test_71Hz[7:0];
				end
				
				16'd5: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch3[15:8];
					adc_data	<=test_71Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel4
				16'd6: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch4[7:0];
					adc_data	<=test_81Hz[7:0];
				end
				
				16'd7: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch4[15:8];
					adc_data	<=test_81Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel5
				16'd8: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch5[7:0];
					adc_data	<=test_91Hz[7:0];
				end
				
				16'd9: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch5[15:8];
					adc_data	<=test_91Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel6
				16'd10: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch6[7:0];
					adc_data	<=test_101Hz[7:0];
				end
				
				16'd11: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch6[15:8];
					adc_data	<=test_101Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel7
				16'd12: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch7[7:0];
					adc_data	<=test_201Hz[7:0];
				end
				
				16'd13: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch7[15:8];
					adc_data	<=test_201Hz[15:8];
				end

				/****************************************************************************/
				//Chanel8
				16'd14: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch8[7:0];
					adc_data	<=test_301Hz[7:0];
				end
				
				16'd15: begin
					//byte_cnt	<=16'd0;	
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch8[15:8];
					adc_data	<=test_301Hz[15:8];
				end
				
				/****************************************************************************/
				//Comments
				//02.10.2019 ADD 24 chanels
				
				/****************************************************************************/
				//Chanel9
				16'd16: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch9[7:0];
					adc_data	<=test_51Hz[7:0];
				end
				
				16'd17: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch9[15:8];
					adc_data	<=test_51Hz[15:8];
				end
				
				
				/****************************************************************************/
				//Chanel10
				16'd18: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch10[7:0];
					adc_data	<=test_61Hz[7:0];
				end
				
				16'd19: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch10[15:8];
					adc_data	<=test_61Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel11
				16'd20: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch11[7:0];
					adc_data	<=test_301Hz[7:0];
				end
				
				16'd21: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch11[15:8];
					adc_data	<=test_301Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel12
				16'd22: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch12[7:0];
					adc_data	<=test_201Hz[7:0];
				end
				
				16'd23: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch12[15:8];
					adc_data	<=test_201Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel13
				16'd24: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch13[7:0];
					adc_data	<=test_101Hz[7:0];
				end
				
				16'd25: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch13[15:8];
					adc_data	<=test_101Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel14
				16'd26: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch14[7:0];
					adc_data	<=test_91Hz[7:0];
				end
				
				16'd27: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch14[15:8];
					adc_data	<=test_91Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel15
				16'd28: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch15[7:0];
					adc_data	<=test_81Hz[7:0];
				end
				
				16'd29: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch15[15:8];
					adc_data	<=test_81Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel16
				16'd30: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch16[7:0];
					adc_data	<=test_71Hz[7:0];
				end
				
				16'd31: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch16[15:8];
					adc_data	<=test_71Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel17
				16'd32: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch17[7:0];
					adc_data	<=test_61Hz[7:0];
				end
				
				16'd33: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch17[15:8];
					adc_data	<=test_61Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel18
				16'd34: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch18[7:0];
					adc_data	<=test_201Hz[7:0];
				end
				
				16'd35: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch18[15:8];
					adc_data	<=test_201Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel19
				16'd36: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch19[7:0];
					adc_data	<=test_101Hz[7:0];
				end
				
				16'd37: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch19[15:8];
					adc_data	<=test_101Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel20
				16'd38: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch20[7:0];
					adc_data	<=test_51Hz[7:0];
				end
				
				16'd39: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch20[15:8];
					adc_data	<=test_51Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel21
				16'd40: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch21[7:0];
					adc_data	<=test_71Hz[7:0];
				end
				
				16'd41: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch21[15:8];
					adc_data	<=test_71Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel22
				16'd42: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch22[7:0];
					adc_data	<=test_91Hz[7:0];
				end
				
				16'd43: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch22[15:8];
					adc_data	<=test_91Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel23
				16'd44: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch23[7:0];
					adc_data	<=test_301Hz[7:0];
				end
				
				16'd45: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch23[15:8];
					adc_data	<=test_301Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel24
				16'd46: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch24[7:0];
					adc_data	<=test_101Hz[7:0];
				end
				
				16'd47: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch24[15:8];
					adc_data	<=test_101Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel25
				16'd48: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch25[7:0];
					adc_data	<=test_201Hz[7:0];
				end
				
				16'd49: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch25[15:8];
					adc_data	<=test_201Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel26
				16'd50: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch26[7:0];
					adc_data	<=test_61Hz[7:0];
				end
				
				16'd51: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch26[15:8];
					adc_data	<=test_61Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel27
				16'd52: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch27[7:0];
					adc_data	<=test_81Hz[7:0];
				end
				
				16'd53: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch27[15:8];
					adc_data	<=test_81Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel28
				16'd54: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch28[7:0];
					adc_data	<=test_301Hz[7:0];
				end
				
				16'd55: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch28[15:8];
					adc_data	<=test_301Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel29
				16'd56: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch29[7:0];
					adc_data	<=test_61Hz[7:0];
				end
				
				16'd57: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch29[15:8];
					adc_data	<=test_61Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel30
				16'd58: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch30[7:0];
					adc_data	<=test_101Hz[7:0];
				end
				
				16'd59: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch30[15:8];
					adc_data	<=test_101Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel31
				16'd60: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch31[7:0];
					adc_data	<=test_201Hz[7:0];
				end
				
				16'd61: begin
					byte_cnt	<=byte_cnt+1'b1;
					//adc_data	<=ad_ch31[15:8];
					adc_data	<=test_201Hz[15:8];
				end
				
				/****************************************************************************/
				//Chanel32
				16'd62: begin
					byte_cnt	<=byte_cnt+1'b1;	
					//adc_data	<=ad_ch32[7:0];
					adc_data	<=test_51Hz[7:0];
				end
				
				16'd63: begin
					byte_cnt	<=0;
					//adc_data	<=ad_ch32[15:8];
					adc_data	<=test_51Hz[15:8];
					adc_state<=4'd0;
				end
				
				default: byte_cnt<=16'd0;
			endcase
		end

		default: adc_state<=4'd0;
	endcase
end



double_mac double_mac_inst(
	.reset_n		 		(1'b1),
											
//	.eth1_rx_clk 		(rx_clk_a),
//	.eth1_tx_clk 		(tx_clk_a),
//	.eth1_rxd    		(rxd_a),
//	.eth1_rxdv    		(rx_dv_a),
//	.eth1_txd			(txd_a),
//	.eth1_tx_en			(tx_en_a),
//	.eth1_local_ip		({8'd192, 8'd168, 8'd4, 8'd2}),
//	.eth1_local_mac	(48'h000a35000102),
//	.eth1_local_port	(16'd25700),
//	
//	.eth2_rx_clk 		(rx_clk_b),
//	.eth2_tx_clk 		(tx_clk_b),
//	.eth2_rxd    		(rxd_b),
//	.eth2_rxdv    		(rx_dv_b),
//	.eth2_txd			(txd_b),
//	.eth2_tx_en			(tx_en_b),
//	.eth2_local_ip		({8'd192, 8'd168, 8'd4, 8'd3}),
//	.eth2_local_mac	(48'h000a35000103),
//	.eth2_local_port	(16'd25700),

	.eth1_rx_clk 		(rx_clk_b),
	.eth1_tx_clk 		(tx_clk_b),
	.eth1_rxd    		(rxd_b),
	.eth1_rxdv    		(rx_dv_b),
	.eth1_txd			(txd_b),
	.eth1_tx_en			(tx_en_b),
//	.eth1_local_ip		({8'd192, 8'd168, 8'd4, 8'd2}),
//	.eth1_local_mac	(48'h000a35000102),
//	.eth1_local_port	(16'd25700),
	
	.eth1_local_ip		(IP_local),
	.eth1_local_mac	({40'h000a350001, 3'd0, ~IP_ADDR}),
	.eth1_local_port	(16'd25700),
	

	
	.eth2_rx_clk 		(rx_clk_a),
	.eth2_tx_clk 		(tx_clk_a),
	.eth2_rxd    		(rxd_a),
	.eth2_rxdv    		(rx_dv_a),
	.eth2_txd			(txd_a),
	.eth2_tx_en			(tx_en_a),
// .eth2_local_ip		({8'd192, 8'd168, 8'd4, 8'd3}),
// .eth2_local_mac	(48'h000a35000103),
// .eth2_local_port	(16'd25700),
	

	.adc_ok				(adc_ok),
	.adc_byte			(adc_data)
);

always@(posedge clk_50Mhz) begin
	//Reset PHY alg state
	case(alg_rst)
		4'd0: begin
			eth_phy_rst_n<=1'b0;//Reset PHY
			if(cnt_rst<=32'h01312D00)begin
				cnt_rst<=cnt_rst+1'b1;
			end
			else begin
				alg_rst<=4'd1;
			end
		end
	
		4'd1: begin
			eth_phy_rst_n<=1'b1;//Enable PHY
		end
	endcase
end
endmodule



//25Mhz Imitation ADC Data 5khz
//always@(posedge tx_clk_a) begin
//	case(adc_state)
//		4'd0: begin
//			if(byte_cnt<16'd1408) begin
//				adc_ok	<=1'b1;
//				byte_cnt	<=byte_cnt+1'b1;	
//				
//				//SYNC
//				if(byte_cnt==0)
//					adc_data	<= 8'h1A;
//				else if(byte_cnt==1)
//					adc_data	<= 8'hCF;
//				else if(byte_cnt==2)
//					adc_data	<= 8'hFC;
//				else if(byte_cnt==3)
//					adc_data	<= 8'h1D;
//				else//DATA
//					adc_data	<= byte_cnt;
//					
//					
//			end
//			else begin
//				adc_ok<=1'b0;
//				adc_state<=4'd2;
//				byte_cnt	<=16'd0;
//			end
//		end
//		
//		
//		4'd2: begin
//			if(adc_timer < 32'd110000) begin
//				adc_timer<=adc_timer+1'b1;
//			end
//			else begin
//				adc_state<=4'd0;
//				adc_timer<=32'd0;
//			end
//		end
//	
//		default: adc_state<=4'd0;
//	endcase
//end

/*
//for test
pulse pulse_test
(
	.clk(tx_clk_a),	//25Mhz
	.fd(fd),   			//25kHz
	.out(puls)
);*/





		/*4'd0: begin //Fill Data
			if(byte_cnt<16'd1407) begin
				adc_data[byte_cnt]	<= byte_cnt[7:0];
				byte_cnt					<= byte_cnt+1'b1;
			end
			else begin
				adc_data[byte_cnt]	<= byte_cnt[7:0];
				adc_state				<= 4'd1;
				adc_ok					<= 1'b1;//Enable flag ADC data
			end
		end
		
		4'd1: begin //Delay for 5kHz
			adc_ok<= 1'b0;						//Disable flag ADC data
			
			if(adc_timer < 32'd3591) begin
				adc_timer<=adc_timer+1'b1;
			end
			else begin
				adc_state	<= 4'd0;
				byte_cnt		<= 16'd0;
			end
		end*/