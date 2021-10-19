module udp_ip_stack(
	input               		reset_n,
	
	//*****************************************************//
	//*Ethernet 1														*//
	input               		eth1_rx_clk,
	input               		eth1_tx_clk,
	
	input       [3:0]   		eth1_rxd,
	input               		eth1_rxdv,
	
	output reg  [3:0]   		eth1_txd,
	output reg          		eth1_tx_en,

	input  wire [31:0]  		eth1_local_ip,
	input  wire [47:0]  		eth1_local_mac,
	input  wire [15:0]  		eth1_local_port,

	//*****************************************************//
	//*Ethernet 2														*//
	input               		eth2_rx_clk,
	input               		eth2_tx_clk,
	
	input       [3:0]   		eth2_rxd,
	input               		eth2_rxdv,
	
	output reg  [3:0]   		eth2_txd,
	output reg          		eth2_tx_en,

	input  wire [31:0]  		eth2_local_ip,
	input  wire [47:0]  		eth2_local_mac,
	input  wire [15:0]  		eth2_local_port,

	//*****************************************************//
	//*ADC Data 														*//
	input							adc_ok,
	input  		[7:0]   		adc_byte,

	
	
	
	//*****************************************************//
	//*For Debug 														*//

	output			arp,
	output			icmp,
	output			udp,


	output 			fifo_adc_rd,
	output 			fifo_adc_wr,
	output 			fifo_adc_empty,
	output 			fifo_adc_full,
	output [15:0]	fifo_adc_cnt,
	output [7:0]	fifo_adc_dout,
	
	output 			fifo_arp_rd,
	output 			fifo_arp_wr,
	output 			fifo_arp_empty,
	output 			fifo_arp_full,
	output [15:0]	fifo_arp_cnt,
	output [7:0]	fifo_arp_dout,
	
	output 			fifo_icmp_rd,
	output 			fifo_icmp_wr,
	output 			fifo_icmp_empty,
	output 			fifo_icmp_full,
	output [15:0]	fifo_icmp_cnt,
	output [7:0]	fifo_icmp_din,
	output [7:0]	fifo_icmp_dout,
	output [3:0]	alg_pre_eth1_icmp
);


reg byte_sig_t, byte_sig;
reg byte_rxdv_t, byte_rxdv;

reg [7:0]	i,j;
reg [15:0]	n,m;

reg [15:0]	rx_n, rx_m;
reg [15:0] 	dbg_byte_cnt;


reg		  	eth1_arp_ok;
reg			eth1_udp_ok;
reg		  	eth1_icmp_ok;

reg [15:0]  eth1_icmp_cnt;
reg [31:0]  eth1_chk_buf;



reg [7:0]  	rx_state;
reg [15:0] 	rx_byte_cnt;
reg [7:0]  	rx_alg_state;

reg [7:0]  	tx_state;  
reg [15:0] 	tx_byte_cnt;
reg [7:0]  	tx_alg_state;

reg [7:0]  	tx_arp_data  [71:0]; 
reg [7:0]  	tx_icmp_data [109:0]; //8byte Preambule + 14byte Eth + 20byte IPv4 + 64byte ICMP (64 for Linux, 40 for Windows)
reg [7:0]   tx_adc_data  [50:0];

reg [7:0]	icmp24;




reg [111:0] ETH_layer;
reg [47:0]  ETH_MAC_dst;
reg [47:0]  ETH_MAC_src;
reg [15:0]  ETH_type;

reg [159:0] IPv4_layer;
reg [7:0]   IPv4_vh;
reg [7:0]   IPv4_dscp;
reg [15:0]  IPv4_tlen;
reg [15:0]  IPv4_id;
reg [15:0]  IPv4_flags;
reg [7:0]   IPv4_ttl;
reg [7:0]	IPv4_type;
reg [15:0]  IPv4_crc;
reg [31:0]	IPv4_IP_src;
reg [31:0]	IPv4_IP_dst;						

reg [223:0] ARP_layer;
reg [47:0]  ARP_MAC_src;
reg [31:0]  ARP_IP_src;
reg [31:0]  ARP_IP_dst;

reg [511:0] ICMP_layer;
reg [15:0]  ICMP_tlen;

reg [63:0]  UDP_layer;	//8 bytes
reg [79:0]  UDP_data;	//10 bytes
reg [31:0]  UDP_fcs;		//4 bytes
reg [15:0]  UDP_tlen;

reg [15:0]  UDP_SP;
reg [15:0]	UDP_DP;
reg [15:0]	UDP_data_len;


parameter	rx_IDLE			=8'd0,
				rx_PREAMBULE	=8'd1,
            rx_ETH_layer	=8'd2,
				rx_IPv4_layer  =8'd3,
				rx_ARP_layer	=8'd4,
				rx_ICMP_layer	=8'd5,
				rx_UDP_layer	=8'd6,
				rx_UDP_Data 	=8'd7,
				rx_ETH_TRL		=8'd8,
				rx_ETH_FCS		=8'd9;
				
parameter 	tx_IDLE			=8'd0,
				tx_ADC		 	=8'd1,	//Data from ADC
				tx_ARP		 	=8'd2,	//Answer for request packet from Eth1
				tx_UDP		 	=8'd3,
				tx_ICMP		 	=8'd4,
				
				tx_FCS	      =8'd5,
				tx_IGP		   =8'd6;
							
initial begin
	rx_state<=rx_IDLE;
	tx_state<=tx_IDLE;
	
	eth1_arp_ok<=1'b0;
	eth1_udp_ok<=1'b0;
	eth1_icmp_ok<=1'b0;
	eth1_icmp_cnt<=16'd0;
	
	rx_byte_cnt<=16'd0;
	rx_alg_state<=8'd0;
	
	tx_byte_cnt<=16'd0;
	tx_alg_state<=8'd0;
	
	dbg_byte_cnt<=16'd0;
end


//assign rx_state_dbg = rx_state;
//assign UDP_layer_dbg = UDP_layer;



//CRC32 Module
wire	[31:0] crcnext;
wire	[31:0] fcs;
reg	crcrst;
reg	crcen;

//input 4-bit data
crc crc_inst(
	.clk(eth1_tx_clk),
	.reset(crcrst),
	.enable(crcen),
	.data(txd),
	.crc(fcs),
	.crc_next(crcnext)
);

//4bit->byte
reg [7:0] mybyte;
reg sig;
always @(posedge rx_clk) begin
	if(rxdv)	begin
		mybyte<={rxd,mybyte[7:4]};
		sig<=~sig;
	end
	else begin
		mybyte<=mybyte;
		sig<=1'b0;
	end
end

//4bit->byte
reg [7:0] datain;
always @(posedge rx_clk) begin
	if(sig&&rxdv) begin
		datain<={rxd,mybyte[7:4]};
	end
	else if(!rxdv)
		datain<=8'd0;
end

//generate byte rxdv signal
always @(posedge rx_clk) begin
	byte_sig_t<=sig;
	byte_sig<=byte_sig_t;
	byte_rxdv_t<=rxdv;
	byte_rxdv<=byte_rxdv_t;	
end

//***********************************************************************************************************************
//RECEIVE PACKET
always@(posedge rx_clk) begin
	case(rx_state)
		rx_IDLE:       begin
			eth1_arp_ok<=1'b0;
			eth1_udp_ok<=1'b0;
			eth1_icmp_ok<=1'b0;
			
			rx_alg_state<=8'd0;
			rx_byte_cnt<=16'd0;
			
			if(byte_rxdv && !byte_sig)  begin
				if(datain==8'h55)
					rx_state<=rx_PREAMBULE; 
			end
		end
		
		rx_PREAMBULE:  begin
			if(byte_rxdv && !byte_sig) begin				  
				case (rx_alg_state)
					8'd0: begin
						if(datain!=8'h55) begin 
							rx_state<=rx_IDLE;
						end
						else begin 
							if(rx_byte_cnt<16'd5)
								rx_byte_cnt<=rx_byte_cnt+1'b1;
							else begin
								rx_byte_cnt<=16'd0;
								rx_alg_state<=8'd1;
							end
						end
					end
					
					8'd1: begin
						if(datain==8'hd5) begin
							rx_state<=rx_ETH_layer;
							rx_alg_state<=8'd0;
						end
						else
							rx_state<=rx_IDLE;
					end
				endcase
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
		
		rx_ETH_layer:  begin
			if(byte_rxdv && !byte_sig) begin
				if(rx_byte_cnt<16'd13) begin
					ETH_layer	<={ETH_layer[103:0],datain};
					rx_byte_cnt	<=rx_byte_cnt+1'b1;
				end
				else begin	
					ETH_layer	<={ETH_layer[103:0],datain};
					ETH_MAC_dst	<=ETH_layer[103:56];
					ETH_MAC_src	<=ETH_layer[55:8];
					ETH_type		<={ETH_layer[7:0],datain};
					
					case ({ETH_layer[7:0],datain}) 							//Ethernet Type Protocol
						16'h0800: begin rx_state<=rx_IPv4_layer; end 	//Receive IPv4
						16'h0806: begin rx_state<=rx_ARP_layer;  end		//Receive ARP
						default:  begin rx_state<=rx_IDLE; 		  end
					endcase
					rx_byte_cnt <=16'd0;
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
		
		rx_IPv4_layer: begin
			if(byte_rxdv && !byte_sig) begin
				if(rx_byte_cnt<16'd19) begin
					IPv4_layer  <={IPv4_layer[151:0],datain[7:0]};
					rx_byte_cnt <=rx_byte_cnt+1'b1;
				end
				else begin
					IPv4_layer	<={IPv4_layer[151:0],datain[7:0]};

					IPv4_vh		<=IPv4_layer[151:144];
					IPv4_dscp	<=IPv4_layer[143:136];
					IPv4_tlen	<=IPv4_layer[135:120];
					IPv4_id		<=IPv4_layer[119:104];
					IPv4_flags  <=IPv4_layer[103:88];
					IPv4_ttl		<=IPv4_layer[87:80];
					IPv4_type	<=IPv4_layer[79:72];
					IPv4_crc		<=IPv4_layer[71:56];
					IPv4_IP_src	<=IPv4_layer[55:24];
					IPv4_IP_dst	<={IPv4_layer[23:0],datain[7:0]};
					
					case (IPv4_layer[79:72])									//IPv4 Type Protocol
						8'd01: begin 												//ICMP
							ICMP_tlen <=IPv4_layer[135:120] - 16'd20; 
							rx_state  <=rx_ICMP_layer; end	
						8'd17: begin 												//UDP
							UDP_tlen  <=IPv4_layer[135:120] - 16'd20; 
							rx_state  <=rx_UDP_layer;  end
						default: rx_state<=rx_IDLE;
					endcase
					rx_byte_cnt <=16'd0;
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
		
		rx_ARP_layer:  begin
			if(byte_rxdv && !byte_sig) begin
				if(rx_byte_cnt<16'd27) begin
					ARP_layer	<={ARP_layer[215:0],datain};
					rx_byte_cnt	<=rx_byte_cnt+1'b1;
				end
				else begin
					ARP_MAC_src	<=ARP_layer[151:104];
					ARP_IP_src	<=ARP_layer[103:72];	
					ARP_IP_dst	<={ARP_layer[23:0],datain};	
					
					if(LOCAL_IP == {ARP_layer[23:0],datain}) begin
						eth1_arp_ok<=1'b1; //Receive ARP Request
					end
					rx_byte_cnt	<=16'd0;
					rx_state<=rx_IDLE;
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
		
		rx_ICMP_layer: begin
			if(byte_rxdv && !byte_sig) begin
				if(rx_byte_cnt<ICMP_tlen) begin
					ICMP_layer	<={ICMP_layer[503:0],datain};
					rx_byte_cnt	<=rx_byte_cnt+1'b1;
				end
				else begin
					ICMP_layer	<={ICMP_layer[503:0],datain};
					
					if(LOCAL_IP == IPv4_IP_dst) begin
						eth1_icmp_ok<=1'b1; //Receive ICMP
					end
					rx_byte_cnt	<=16'd0;
					rx_state<=rx_IDLE;
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end

		rx_UDP_layer:  begin
			if(byte_rxdv && !byte_sig) begin
				if(rx_byte_cnt < 16'd7) begin 							//8 byte UDP Layer
					UDP_layer	<={UDP_layer[55:0],datain};
					rx_byte_cnt	<=rx_byte_cnt+1'b1;
				end
				else begin
					UDP_layer	 <={UDP_layer[55:0],datain};
					UDP_SP		 <=UDP_layer[55:40];
					UDP_DP		 <=UDP_layer[39:24];
					UDP_data_len <=(UDP_layer[23:8]-16'd8);	
					rx_byte_cnt	 <=16'd0;
					if( ({8'd192, 8'd168, 8'd4, 8'd255} == IPv4_IP_dst) && ((UDP_layer[23:8]-16'd8) == 16'd10) ) // Broadcast IP and 10 bytes DATA available?
						rx_state<=rx_UDP_Data;
					else
						rx_state<=rx_IDLE;
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
		
		rx_UDP_Data:   begin
			if(byte_rxdv && !byte_sig) begin
				if(rx_byte_cnt < (UDP_data_len-1'b1)) begin
					UDP_data		<={UDP_data[71:0],datain};
					rx_byte_cnt	<=rx_byte_cnt+1'b1;
				end
				else begin
					UDP_data		<={UDP_data[71:0],datain};
					rx_state		<=rx_ETH_TRL;
					rx_byte_cnt	<=16'd0;
				end	
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
		
		rx_ETH_TRL:    begin //Receive ethernet Trailer 8bytes: 0x00
			if(byte_rxdv && !byte_sig) begin
				if(rx_byte_cnt < 16'd7) begin
					rx_byte_cnt	<=rx_byte_cnt+1'b1;
				end
				else begin
					rx_state		<=rx_ETH_FCS;
					rx_byte_cnt	<=16'd0;
				end
			end
		end
		
		rx_ETH_FCS:    begin //Receive ethernet frame check sequence for retransmit to Ethernet Port 2
			if(byte_rxdv && !byte_sig) begin
				if(rx_byte_cnt < 16'd3) begin
					UDP_fcs		<={UDP_fcs[23:0],datain};
					rx_byte_cnt	<=rx_byte_cnt+1'b1;
				end
				else begin
					UDP_fcs		<={UDP_fcs[23:0],datain};
					eth1_udp_ok	<=1'b1;
					rx_byte_cnt	<=16'd0;
					rx_state		<=rx_IDLE;
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end

		default: rx_state<=rx_IDLE;  
	endcase
end










//***********************************************************************************************************************
//For test
reg [3:0] 	wr,rd; //pointer in current tx data
reg [3:0] 	adc_wr;
reg [3:0] 	igp_cnt;



reg [3:0] 	tx_mem_state;
reg [3:0] 	prepare_pkt;

reg [15:0] 	tx_i, tx_j;
reg [15:0] 	mem_i, mem_j;
reg [15:0] 	pre_i, pre_j;

reg [31:0] 	timer;

reg [3:0]  	eth_state;

reg [3:0]	adc_pkt;
reg [3:0]	mac_pkt;
reg [3:0]	eth_pkt;

reg [15:0]	adc_n, adc_m;
reg [15:0]  ADC_IPv4_id;

reg [15:0]  adc_cnt;

reg  	[3:0]	pre_state;
reg  	[7:0] temp;


//********************************************************
reg	[3:0] pre_eth1_arp;
reg  	[3:0] pre_eth1_udp;
reg	[3:0] pre_eth1_icmp;

reg 	[15:0]pre_eth1_arp_bcnt;
reg 	[15:0]pre_eth1_icmp_bcnt;

reg 	[15:0]eth1_icmp_i;
reg 	[15:0]eth1_icmp_j;
reg 	[15:0]eth1_icmp_len;


reg 			fifo_eth1_adc_rd;
reg 			fifo_eth1_adc_wr;
wire 	[7:0] fifo_eth1_adc_dout;
wire 			fifo_eth1_adc_empty;
wire 			fifo_eth1_adc_full;
wire 	[15:0]fifo_eth1_adc_cnt;

reg			fifo_eth1_arp_rd;
reg			fifo_eth1_arp_wr;
reg	[7:0]	fifo_eth1_arp_din;
wire	[7:0]	fifo_eth1_arp_dout;
wire			fifo_eth1_arp_empty;
wire			fifo_eth1_arp_full;
wire	[15:0]fifo_eth1_arp_cnt;

reg			fifo_eth1_icmp_rd;
reg			fifo_eth1_icmp_wr;
reg	[7:0]	fifo_eth1_icmp_din;
wire	[7:0]	fifo_eth1_icmp_dout;
wire			fifo_eth1_icmp_empty;
wire			fifo_eth1_icmp_full;
wire	[15:0]fifo_eth1_icmp_cnt;

initial begin
	wr<=0;
	rd<=0;
	adc_wr<=0;
	igp_cnt<=0;
	
	pre_state<=4'd0;
	
	tx_mem_state<=4'd0;
	prepare_pkt<=4'd0;
	
	tx_i<=16'd0;
	tx_j<=16'd0;
	
	mem_i<=16'd0;
	mem_j<=16'd0;
	
	pre_i<=16'd0;
	pre_j<=16'd0;
	
	timer<=32'd0;
	
	eth_state<=4'd0;
	
	adc_pkt<=4'd0;
	mac_pkt<=4'd0;
	eth_pkt<=4'd0;
	
	temp<=8'd0;
		
	crcen<=1'b0;
	crcrst<=1'b1;
	ADC_IPv4_id<=16'd0;
	adc_cnt<=16'd50;
	
	pre_eth1_arp  <=4'd0;
	pre_eth1_udp  <=4'd0;
	pre_eth1_icmp <=4'd0;
	
	pre_eth1_arp_bcnt<=16'd0;
	pre_eth1_icmp_bcnt<=16'd0;
	
	fifo_eth1_adc_rd <=1'b0;
	fifo_eth1_adc_wr <=1'b0;
	
	fifo_eth1_arp_wr  <=1'b0;
	fifo_eth1_icmp_wr <=1'b0;
	
	
end

//********************************************************
//for debug
assign fifo_adc_rd 	 = fifo_eth1_adc_rd; 
assign fifo_adc_wr 	 = adc_pkti_ok;
assign fifo_adc_empty = fifo_eth1_adc_empty;
assign fifo_adc_full  = fifo_eth1_adc_full;
assign fifo_adc_cnt   = fifo_eth1_adc_cnt;
assign fifo_adc_dout  = fifo_eth1_adc_dout;

assign fifo_arp_rd 	 = fifo_eth1_arp_rd; 
assign fifo_arp_wr 	 = fifo_eth1_arp_wr;
assign fifo_arp_empty = fifo_eth1_arp_empty;
assign fifo_arp_full  = fifo_eth1_arp_full;
assign fifo_arp_cnt   = fifo_eth1_arp_cnt;
assign fifo_arp_dout  = fifo_eth1_arp_dout;

assign fifo_icmp_wr 	  = fifo_eth1_icmp_wr;
assign fifo_icmp_din   = fifo_eth1_icmp_din;
assign fifo_icmp_rd 	  = fifo_eth1_icmp_rd; 
assign fifo_icmp_dout  = fifo_eth1_icmp_dout;
assign fifo_icmp_cnt   = fifo_eth1_icmp_cnt;
assign fifo_icmp_full  = fifo_eth1_icmp_full;
assign fifo_icmp_empty = fifo_eth1_icmp_empty;

assign alg_pre_eth1_icmp = pre_eth1_icmp;

assign arp			= eth1_arp_ok;
assign udp			= eth1_udp_ok;
assign icmp			= eth1_icmp_ok;



	

//********************************************************//
//*Put ADC data in FIFO for transmission						*//
//********************************************************//
fifo2 # (8,4096) fifo_eth1_adc
(
	.clk		(tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth1_adc_rd & !fifo_eth1_adc_empty),
	.wr		(adc_pkti_ok      & !fifo_eth1_adc_full),
	.din		(adc_pkti),
	.dout		(fifo_eth1_adc_dout),
	.empty	(fifo_eth1_adc_empty),
	.full		(fifo_eth1_adc_full),
	.cnt		(fifo_eth1_adc_cnt)
);

//********************************************************//
//*Put ARP Packet in FIFO for transmission						*//
//********************************************************//
fifo2 # (8,512) fifo_eth1_arp
(
	.clk		(tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth1_arp_rd & !fifo_eth1_arp_empty),
	.wr		(fifo_eth1_arp_wr & !fifo_eth1_arp_full),
	.din		(fifo_eth1_arp_din),
	.dout		(fifo_eth1_arp_dout),
	.empty	(fifo_eth1_arp_empty),
	.full		(fifo_eth1_arp_full),
	.cnt		(fifo_eth1_arp_cnt)
);

//********************************************************//
//*Put ICMP Packet in FIFO for transmission			*//
//********************************************************//
fifo2 # (8,512) fifo_eth1_icmp
(
	.clk		(tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth1_icmp_rd & !fifo_eth1_icmp_empty),
	.wr		(fifo_eth1_icmp_wr & !fifo_eth1_icmp_full),
	.din		(fifo_eth1_icmp_din),
	.dout		(fifo_eth1_icmp_dout),
	.empty	(fifo_eth1_icmp_empty),
	.full		(fifo_eth1_icmp_full),
	.cnt		(fifo_eth1_icmp_cnt)
);



//***********************************************************************************************************************
//PREPARE $ TRANSMIT PACKET	
always@(posedge tx_clk) begin

	//*******************************************************//
	//Prepare ARP packet from Eth1, and put in FIFO			  *//
	//*******************************************************//
	case(pre_eth1_arp)
		4'd0: begin //Idle
			fifo_eth1_arp_wr<=1'b0;
			pre_eth1_arp_bcnt<=16'd0;
			
			if(eth1_arp_ok)
				pre_eth1_arp<=4'd1;
		end
		
		4'd1: begin //Fill
			tx_arp_data[0]  <=8'h55;               //PREAMBULE  
			tx_arp_data[1]  <=8'h55;
			tx_arp_data[2]  <=8'h55;
			tx_arp_data[3]  <=8'h55;
			tx_arp_data[4]  <=8'h55;
			tx_arp_data[5]  <=8'h55;
			tx_arp_data[6]  <=8'h55;
			tx_arp_data[7]	 <=8'hD5; 
			
			tx_arp_data[8]  <=ARP_MAC_src[47:40];	//Dest MAC  
			tx_arp_data[9]  <=ARP_MAC_src[39:32];
			tx_arp_data[10] <=ARP_MAC_src[31:24];
			tx_arp_data[11] <=ARP_MAC_src[23:16];
			tx_arp_data[12] <=ARP_MAC_src[15:8];
			tx_arp_data[13] <=ARP_MAC_src[7:0];	
			
			tx_arp_data[14] <=LOCAL_MAC[47:40];		//Src MAC
			tx_arp_data[15] <=LOCAL_MAC[39:32];
			tx_arp_data[16] <=LOCAL_MAC[31:24];
			tx_arp_data[17] <=LOCAL_MAC[23:16];
			tx_arp_data[18] <=LOCAL_MAC[15:8];
			tx_arp_data[19] <=LOCAL_MAC[7:0];	
			
			tx_arp_data[20] <=ETH_type[15:8]; 		//TYPE 
			tx_arp_data[21] <=ETH_type[7:0];
			tx_arp_data[22] <=8'h00;					//HType
			tx_arp_data[23] <=8'h01;
			tx_arp_data[24] <=8'h08;					//IPv4
			tx_arp_data[25] <=8'h00;
			tx_arp_data[26] <=8'h06;					//Hsz
			tx_arp_data[27] <=8'h04;					//Psz
			tx_arp_data[28] <=8'h00;					//Opcode 0x0002 reply
			tx_arp_data[29] <=8'h02;
			tx_arp_data[30] <=LOCAL_MAC[47:40];		//My MAC  
			tx_arp_data[31] <=LOCAL_MAC[39:32];
			tx_arp_data[32] <=LOCAL_MAC[31:24];
			tx_arp_data[33] <=LOCAL_MAC[23:16];
			tx_arp_data[34] <=LOCAL_MAC[15:8];
			tx_arp_data[35] <=LOCAL_MAC[7:0];
			tx_arp_data[36] <=LOCAL_IP[31:24];		//My IP   
			tx_arp_data[37] <=LOCAL_IP[23:16];
			tx_arp_data[38] <=LOCAL_IP[15:8];
			tx_arp_data[39] <=LOCAL_IP[7:0];
			tx_arp_data[40] <=ARP_MAC_src[47:40];	//Target MAC 
			tx_arp_data[41] <=ARP_MAC_src[39:32];
			tx_arp_data[42] <=ARP_MAC_src[31:24];
			tx_arp_data[43] <=ARP_MAC_src[23:16];
			tx_arp_data[44] <=ARP_MAC_src[15:8];
			tx_arp_data[45] <=ARP_MAC_src[7:0];	
			tx_arp_data[46] <=ARP_IP_src[31:24];	//Target IP   
			tx_arp_data[47] <=ARP_IP_src[23:16];
			tx_arp_data[48] <=ARP_IP_src[15:8];
			tx_arp_data[49] <=ARP_IP_src[7:0];
			pre_eth1_arp<=4'd2;
		end

		4'd2: begin //Put it FIFO
			if(pre_eth1_arp_bcnt<16'd49) begin
				fifo_eth1_arp_wr<=1'b1;
				fifo_eth1_arp_din<=tx_arp_data[pre_eth1_arp_bcnt];
				pre_eth1_arp_bcnt<=pre_eth1_arp_bcnt+1'b1;
			end
			else begin
				fifo_eth1_arp_din<=tx_arp_data[pre_eth1_arp_bcnt];
				pre_eth1_arp<=4'd0;
			end
		end

		default: pre_eth1_arp<=4'd0;
	endcase
	
	//*******************************************************//
	//Prepare ICMP packet from Eth1, and put in FIFO		  *//
	//*******************************************************//
	case(pre_eth1_icmp)
		4'd0: begin //Idle
			eth1_icmp_i<=16'd0;
			eth1_icmp_j<=16'd0;
			fifo_eth1_icmp_wr<=1'b0;

			if(eth1_icmp_ok)
				pre_eth1_icmp<=4'd1;
		end
		
		4'd1: begin //Fill
			eth1_icmp_cnt<=eth1_icmp_cnt+1'b1;
			//PREAMBULE  
			tx_icmp_data[0]  <=8'h55;					
			tx_icmp_data[1]  <=8'h55;
			tx_icmp_data[2]  <=8'h55;
			tx_icmp_data[3]  <=8'h55;
			tx_icmp_data[4]  <=8'h55;
			tx_icmp_data[5]  <=8'h55;
			tx_icmp_data[6]  <=8'h55;
			tx_icmp_data[7]  <=8'hD5; 
			//ETH
			tx_icmp_data[8]  <=ETH_MAC_src[47:40];	//Dest MAC  
			tx_icmp_data[9]  <=ETH_MAC_src[39:32];
			tx_icmp_data[10] <=ETH_MAC_src[31:24];
			tx_icmp_data[11] <=ETH_MAC_src[23:16];
			tx_icmp_data[12] <=ETH_MAC_src[15:8];
			tx_icmp_data[13] <=ETH_MAC_src[7:0];	
			tx_icmp_data[14] <=LOCAL_MAC[47:40];	//Src MAC
			tx_icmp_data[15] <=LOCAL_MAC[39:32];
			tx_icmp_data[16] <=LOCAL_MAC[31:24];
			tx_icmp_data[17] <=LOCAL_MAC[23:16];
			tx_icmp_data[18] <=LOCAL_MAC[15:8];
			tx_icmp_data[19] <=LOCAL_MAC[7:0];	
			tx_icmp_data[20] <=ETH_type[15:8];		//TYPE 
			tx_icmp_data[21] <=ETH_type[7:0];
			//IPv4
			tx_icmp_data[22] <=8'h45;					//Version number: 4; Header length: 20; 
			tx_icmp_data[23] <=8'h00;
			tx_icmp_data[24] <=IPv4_tlen[15:8];		//IPv4 total length
			tx_icmp_data[25] <=IPv4_tlen[7:0];
			tx_icmp_data[26] <=eth1_icmp_cnt[15:8];//Package serial number
			tx_icmp_data[27] <=eth1_icmp_cnt[7:0];
			tx_icmp_data[28] <=8'h00;					//Fragment offset
			tx_icmp_data[29] <=8'h00;
			tx_icmp_data[30] <=IPv4_ttl;				//TTL 64 8'h40;
			tx_icmp_data[31] <=8'h01;					//Protocol: 01(ICMP)
			tx_icmp_data[32] <=8'h00;					//Header checksum
			tx_icmp_data[33] <=8'h00;
			tx_icmp_data[34] <=LOCAL_IP[31:24];		//Source IP   
			tx_icmp_data[35] <=LOCAL_IP[23:16];
			tx_icmp_data[36] <=LOCAL_IP[15:8];
			tx_icmp_data[37] <=LOCAL_IP[7:0];				
			tx_icmp_data[38] <=IPv4_IP_src[31:24];	//Dest IP
			tx_icmp_data[39] <=IPv4_IP_src[23:16];
			tx_icmp_data[40] <=IPv4_IP_src[15:8];
			tx_icmp_data[41] <=IPv4_IP_src[7:0];
			
			case(ICMP_tlen)
//				16'd40: begin //ICMP packet for Windows OS
//					tx_icmp_data[42]  <=8'h00;						//TYPE:0 (Echo (ping) reply)
//					tx_icmp_data[43]  <=8'h00;						//CODE:0
//					tx_icmp_data[44]  <=8'h00;						//CHECKSUM
//					tx_icmp_data[45]  <=8'h00;
//					tx_icmp_data[46]  <=ICMP_layer[295:288]; 	//Id BE
//					tx_icmp_data[47]  <=ICMP_layer[287:280];	//Id LE
//					tx_icmp_data[48]  <=ICMP_layer[279:272];	//SN BE
//					tx_icmp_data[49]  <=ICMP_layer[271:264];	//SN LE
//					
//					//Fill data
//					for(eth1_icmp_i=50, eth1_icmp_j=0; eth1_icmp_i<82; eth1_icmp_i=eth1_icmp_i+1, eth1_icmp_j=eth1_icmp_j+1) begin
//						tx_icmp_data[eth1_icmp_i] <=ICMP_layer[263-(eth1_icmp_j*8)-:8];
//					end
//				end
				
				16'd64: begin //ICMP packet for Linux OS
					tx_icmp_data[42]  <=8'h00;						//TYPE:0 (Echo (ping) reply)
					tx_icmp_data[43]  <=8'h00;						//CODE:0
					tx_icmp_data[44]  <=8'h00;						//CHECKSUM
					tx_icmp_data[45]  <=8'h00;
					
					//Fill data
					for(int eth1_icmp_i=46, eth1_icmp_j=0; eth1_icmp_i<106; eth1_icmp_i=eth1_icmp_i+1, eth1_icmp_j=eth1_icmp_j+1) begin
						tx_icmp_data[eth1_icmp_i] <=ICMP_layer[487-(eth1_icmp_j*8)-:8];
					end
				end
				
				default: pre_eth1_icmp<=4'd0;
			endcase
			pre_eth1_icmp<=4'd2;
		end
		
		4'd2: begin //Generate CRC for IPv4
			if(eth1_icmp_i==0) begin											
				eth1_chk_buf <={tx_icmp_data[24],tx_icmp_data[25]} + {tx_icmp_data[22],tx_icmp_data[23]} +
									{tx_icmp_data[28],tx_icmp_data[29]} + {tx_icmp_data[26],tx_icmp_data[27]} +
									{tx_icmp_data[32],tx_icmp_data[33]} + {tx_icmp_data[30],tx_icmp_data[31]} +
									{tx_icmp_data[36],tx_icmp_data[37]} + {tx_icmp_data[34],tx_icmp_data[35]} +
									{tx_icmp_data[40],tx_icmp_data[41]} + {tx_icmp_data[38],tx_icmp_data[39]};
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else if(eth1_icmp_i==1) begin
				eth1_chk_buf[15:0]<=eth1_chk_buf[31:16]+eth1_chk_buf[15:0];
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else begin
				tx_icmp_data[32]<=~eth1_chk_buf[15:8];
				tx_icmp_data[33]<=~eth1_chk_buf[7:0];
				eth1_icmp_i<=0;
				eth1_icmp_j<=0;
				pre_eth1_icmp<=4'd3;
			end
		end
		
		4'd3: begin //Generate CRC ICMP
			if(eth1_icmp_i==0) begin
				case(ICMP_tlen)
//					16'd40: begin //for Windows OS
//						eth1_chk_buf <={tx_icmp_data[44],tx_icmp_data[45]}   + {tx_icmp_data[42],tx_icmp_data[43]} +
//											{tx_icmp_data[48],tx_icmp_data[49]}   + {tx_icmp_data[46],tx_icmp_data[47]} +
//											{tx_icmp_data[52],tx_icmp_data[53]}   + {tx_icmp_data[50],tx_icmp_data[51]} +
//											{tx_icmp_data[56],tx_icmp_data[57]}   + {tx_icmp_data[54],tx_icmp_data[55]} +
//											{tx_icmp_data[60],tx_icmp_data[61]}   + {tx_icmp_data[58],tx_icmp_data[59]} +	
//											{tx_icmp_data[64],tx_icmp_data[65]}   + {tx_icmp_data[62],tx_icmp_data[63]} +
//											{tx_icmp_data[68],tx_icmp_data[69]}   + {tx_icmp_data[66],tx_icmp_data[67]} +
//											{tx_icmp_data[72],tx_icmp_data[73]}   + {tx_icmp_data[70],tx_icmp_data[71]} +
//											{tx_icmp_data[76],tx_icmp_data[77]}   + {tx_icmp_data[74],tx_icmp_data[75]} +
//											{tx_icmp_data[80],tx_icmp_data[81]}   + {tx_icmp_data[78],tx_icmp_data[79]} ;
//					end
					
					16'd64: begin //for Linux OS
						eth1_chk_buf <={tx_icmp_data[44],tx_icmp_data[45]}   + {tx_icmp_data[42],tx_icmp_data[43]} +
											{tx_icmp_data[48],tx_icmp_data[49]}   + {tx_icmp_data[46],tx_icmp_data[47]} +
											{tx_icmp_data[52],tx_icmp_data[53]}   + {tx_icmp_data[50],tx_icmp_data[51]} +
											{tx_icmp_data[56],tx_icmp_data[57]}   + {tx_icmp_data[54],tx_icmp_data[55]} +
											{tx_icmp_data[60],tx_icmp_data[61]}   + {tx_icmp_data[58],tx_icmp_data[59]} +	
											{tx_icmp_data[64],tx_icmp_data[65]}   + {tx_icmp_data[62],tx_icmp_data[63]} +
											{tx_icmp_data[68],tx_icmp_data[69]}   + {tx_icmp_data[66],tx_icmp_data[67]} +
											{tx_icmp_data[72],tx_icmp_data[73]}   + {tx_icmp_data[70],tx_icmp_data[71]} +
											{tx_icmp_data[76],tx_icmp_data[77]}   + {tx_icmp_data[74],tx_icmp_data[75]} +
											{tx_icmp_data[80],tx_icmp_data[81]}   + {tx_icmp_data[78],tx_icmp_data[79]} +
											{tx_icmp_data[84],tx_icmp_data[85]}   + {tx_icmp_data[82],tx_icmp_data[83]} +
											{tx_icmp_data[88],tx_icmp_data[89]}   + {tx_icmp_data[86],tx_icmp_data[87]} +
											{tx_icmp_data[92],tx_icmp_data[93]}   + {tx_icmp_data[90],tx_icmp_data[91]} +
											{tx_icmp_data[96],tx_icmp_data[97]}   + {tx_icmp_data[94],tx_icmp_data[95]} +
											{tx_icmp_data[100],tx_icmp_data[101]} + {tx_icmp_data[98],tx_icmp_data[99]} +
											{tx_icmp_data[104],tx_icmp_data[105]} + {tx_icmp_data[102],tx_icmp_data[103]};	
					end
					
					default: pre_eth1_icmp<=4'd0;
				endcase
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else if(eth1_icmp_i==1) begin
				eth1_chk_buf[15:0]<=eth1_chk_buf[31:16]+eth1_chk_buf[15:0];
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else begin
				tx_icmp_data[44]<=~eth1_chk_buf[15:8];
				tx_icmp_data[45]<=~eth1_chk_buf[7:0];
				eth1_icmp_i<=0;
				eth1_icmp_j<=0;
				case(ICMP_tlen)
					16'd64:  pre_eth1_icmp<=4'd4;
					default: pre_eth1_icmp<=4'd0;
				endcase
			end
		end
		
		4'd4: begin //Put it FIFO
			
			//Put in FIFO 2byte ICMP length of packet (Linux or Windows)
//			if(eth1_icmp_i==16'd0) begin
//				eth1_icmp_i<=eth1_icmp_i+1'b1;
//				fifo_eth1_icmp_wr	<=1'b1;
//				fifo_eth1_icmp_din<=ICMP_tlen[15:8];
//			end
//			else if(eth1_icmp_i==16'd1) begin
//				eth1_icmp_i<=eth1_icmp_i+1'b1;
//				fifo_eth1_icmp_din<=ICMP_tlen[7:0];
//			end
			
			//Put in FIFO ICMP data
			if(eth1_icmp_i<16'd105) begin //104
				fifo_eth1_icmp_wr	<=1'b1;
				fifo_eth1_icmp_din<=tx_icmp_data[eth1_icmp_i];
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else begin//105
				fifo_eth1_icmp_din<=tx_icmp_data[eth1_icmp_i];
				pre_eth1_icmp<=4'd0;
			end
			
		end
		
		default: pre_eth1_icmp<=4'd0;
	endcase
	
	
	
	//*******************************************************//
	//*Send packet from FIFO's										  *//
	//*******************************************************//
	case(tx_state)
		tx_IDLE: begin
			
			//Checking fifo's for transmission
			if(fifo_eth1_adc_cnt>=16'd1408) begin	//512
				tx_state<=tx_ADC;
			end
			else if(fifo_eth1_arp_cnt>=16'd49) begin
				tx_state<=tx_ARP;
			end
			else if(fifo_eth1_icmp_cnt>=16'd105) begin
				tx_state<=tx_ICMP;
			end
			

			crcen<=0;
			crcrst<=1;
			
			tx_en<=0;
			tx_i<=0;
			tx_j<=0;
			pre_i<=0; 
			pre_j<=0;
			igp_cnt<=4'd0;
			pre_state<=4'd0;
			
			temp<=8'd0;
		end
		
		tx_ADC:  begin
			case(pre_state)
				4'd0: begin	//Prepare packet
					ADC_IPv4_id<=ADC_IPv4_id+1'b1;	//Increment ID
				
					tx_adc_data[0]  <=8'h55;	//PREAMBULE  
					tx_adc_data[1]  <=8'h55;
					tx_adc_data[2]  <=8'h55;
					tx_adc_data[3]  <=8'h55;
					tx_adc_data[4]  <=8'h55;
					tx_adc_data[5]  <=8'h55;
					tx_adc_data[6]  <=8'h55;
					tx_adc_data[7]	 <=8'hD5; 
					
					tx_adc_data[8]  <=8'hFF;	//Dest MAC  
					tx_adc_data[9]  <=8'hFF;
					tx_adc_data[10] <=8'hFF;
					tx_adc_data[11] <=8'hFF;
					tx_adc_data[12] <=8'hFF;
					tx_adc_data[13] <=8'hFF;	

					tx_adc_data[14] <=LOCAL_MAC[47:40];		//Src MAC
					tx_adc_data[15] <=LOCAL_MAC[39:32];
					tx_adc_data[16] <=LOCAL_MAC[31:24];
					tx_adc_data[17] <=LOCAL_MAC[23:16];
					tx_adc_data[18] <=LOCAL_MAC[15:8];
					tx_adc_data[19] <=LOCAL_MAC[7:0];	

					tx_adc_data[20] <=8'h08;				//TYPE 
					tx_adc_data[21] <=8'h00;
					tx_adc_data[22] <=8'h45;
					tx_adc_data[23] <=8'h00;
					
					tx_adc_data[24] <=8'h05;				//Len 20 + 8 + 512 = 1436 -> 059C
					tx_adc_data[25] <=8'h9C;	
					
					tx_adc_data[26] <=ADC_IPv4_id[15:8];
					tx_adc_data[27] <=ADC_IPv4_id[7:0];
					tx_adc_data[28] <=8'h40;				//Flags
					tx_adc_data[29] <=8'h00;
					tx_adc_data[30] <=8'h40;				//TTL
					tx_adc_data[31] <=8'h11;				//UDP
					tx_adc_data[32] <=8'h00;				//CRC
					tx_adc_data[33] <=8'h00;
					tx_adc_data[34] <=LOCAL_IP[31:24];	//My IP   
					tx_adc_data[35] <=LOCAL_IP[23:16];
					tx_adc_data[36] <=LOCAL_IP[15:8];
					tx_adc_data[37] <=LOCAL_IP[7:0];
					tx_adc_data[38] <=8'd192;				//Dst IP
					tx_adc_data[39] <=8'd168;
					tx_adc_data[40] <=8'd4;
					tx_adc_data[41] <=8'd1;
					
					tx_adc_data[42] <=8'hBB;	//SP
					tx_adc_data[43] <=8'h80;
					tx_adc_data[44] <=8'hBB;	//DP
					tx_adc_data[45] <=8'h80;
					
					tx_adc_data[46] <=8'h05;	//Length = 8 + 1408
					tx_adc_data[47] <=8'h88;
					
					tx_adc_data[48] <=8'h00;	//CRC
					tx_adc_data[49] <=8'h00;
					
					pre_state<=4'd1;
				end
				
				4'd1: begin //Send packet
					
					tx_en<=1;
					//CRC
					if(tx_j<=7) begin
						crcen<=0;  	//Disable module FCS
						crcrst<=1; 	//Reset value FCS 
					end
					else begin
						crcen<=1;	//Enable module FCS
						crcrst<=0; 
					end
					
					//****************************************************************
					//End of Packet
					if(tx_j==16'd1457) begin	//561
						
						//LSB part
						if(tx_i==0) begin

							if(pre_j>49)begin
								tx_i<=tx_i+1'b1;
								fifo_eth1_adc_rd	<=1'b1;
								txd[3:0] 	<= fifo_eth1_adc_dout[3:0];
								temp			<= fifo_eth1_adc_dout;	
							end
							else begin
								tx_i<=tx_i+1'b1;
								txd[3:0] 	<=tx_adc_data[pre_j][3:0];
							end
						end
						
						//MSB part
						else if(tx_i==1) begin
							if(pre_j>49)begin
								tx_i<=0;
								fifo_eth1_adc_rd	<=1'b0;
								txd[3:0] <= temp[7:4];
								tx_state	<=tx_FCS;
							end
							else begin
								tx_i<=0;
								pre_j<=pre_j+1'b1;
								txd[3:0] <= tx_adc_data[pre_j][7:4];
							end
						end
					end
					
					
					
					//****************************************************************
					//Start of Packet
					else begin
						//LSB part
						if(tx_i==0) begin
						
							if(pre_j>49)begin
								tx_i<=tx_i+1'b1;
								fifo_eth1_adc_rd	<=1'b1;
								txd[3:0] 	<= fifo_eth1_adc_dout[3:0];
								temp			<= fifo_eth1_adc_dout;
							end
							else begin
								tx_i<=tx_i+1'b1;
								txd[3:0] <= tx_adc_data[pre_j][3:0];
								
								case(pre_j)
									16'd49: fifo_eth1_adc_rd	<=1'b1;
								endcase
							end
							
						end
						
						//MSB part
						else if(tx_i==1) begin
							if(pre_j>49)begin
								tx_i<=0;		
								fifo_eth1_adc_rd	<=1'b0;
								txd[3:0] 	<= temp[7:4];
								tx_j<=tx_j+1'b1;
							end
							else begin
								tx_i<=0;
								txd[3:0] <= tx_adc_data[pre_j][7:4];
								pre_j<=pre_j+1'b1;
								tx_j<=tx_j+1'b1;
								case(pre_j)
									16'd49: fifo_eth1_adc_rd	<=1'b0;
								endcase
							end
						end
					end
					
				end
				
				default: pre_state<=4'd0;
			endcase
		end
		
		tx_ARP:  begin
			case(pre_state)
				4'd0: begin
					fifo_eth1_arp_rd	<=1'b1;
					pre_state<=4'd1;
				end
				
				4'd1: begin
					fifo_eth1_arp_rd	<=1'b10;
					pre_state<=4'd2;
				end
				
				4'd2: begin
					tx_en<=1;
					//CRC
					if(tx_j<=7) begin
						crcen<=0;  	//Disable module FCS
						crcrst<=1; 	//Reset value FCS 
					end
					else begin
						crcen<=1;	//Enable module FCS
						crcrst<=0; 
					end
					
					//****************************************************************
					//End of Packet
					if(tx_j==16'd49) begin//49
						if(tx_i==0) begin
							tx_i<=tx_i+1'b1;
							txd[3:0] <= fifo_eth1_arp_dout[3:0];
							temp		<= fifo_eth1_arp_dout;	
						end
						else if(tx_i==1) begin
							tx_i<=0;
							txd[3:0] <= temp[7:4];
							tx_state <=tx_FCS;
						end
					end
					
					//Start of Packet
					else begin
						if(tx_i==0) begin
							tx_i<=tx_i+1'b1;
							fifo_eth1_arp_rd	<=1'b1;
							txd[3:0] 			<= fifo_eth1_arp_dout[3:0];
							temp					<= fifo_eth1_arp_dout;	
							 
						end
						else if(tx_i==1) begin
							tx_i<=0;
							tx_j<=tx_j+1'b1;
							fifo_eth1_arp_rd	<=1'b0;
							txd[3:0] 			<= temp[7:4];
						end
					end
				end
				
				default: pre_state<=4'd0;
			endcase
		end
		
		tx_ICMP: begin
			case(pre_state)
				//DUMMY
				4'd0: begin
					//eth1_icmp_len <=16'd0;
					fifo_eth1_icmp_rd	<=1'b1;
					pre_state<=4'd1;
				end
				
				//DUMMY
				4'd1: begin
					fifo_eth1_icmp_rd	<=1'b10;
					pre_state<=4'd3;
				end
				
				//Get length of ICMP packet
//				4'd2: begin
//					if(tx_i==0) begin
//						tx_i<=tx_i+1'b1;
//						fifo_eth1_icmp_rd	<=1'b1;
//						eth1_icmp_len 		<={fifo_eth1_icmp_dout,8'd0};
//					end
//					else if(tx_i==1) begin
//						tx_i<=tx_i+1'b1;
//						fifo_eth1_icmp_rd	<=1'b0;
//					end
//					else if(tx_i==2) begin
//						tx_i<=tx_i+1'b1;
//						fifo_eth1_icmp_rd	<=1'b1;
//						eth1_icmp_len 		<=fifo_eth1_icmp_dout;
//					end
//					else if(tx_i==3) begin
//						fifo_eth1_icmp_rd	<=1'b0;
//						tx_i<=0;
//						tx_j<=0;
//						
//						if(eth1_icmp_len>0)
//							pre_state<=4'd3;
//						else
//							tx_state<=tx_IDLE;
//					end
//				end
				
				//Send ICMP
				4'd3: begin
					tx_en<=1;
					//CRC
					if(tx_j<=7) begin
						crcen<=0;  	//Disable module FCS
						crcrst<=1; 	//Reset value FCS 
					end
					else begin
						crcen<=1;	//Enable module FCS
						crcrst<=0; 
					end
					
					//****************************************************************
					//End of Packet
					if(tx_j==16'd105) begin
						if(tx_i==0) begin
							tx_i<=tx_i+1'b1;
							txd[3:0] <= fifo_eth1_icmp_dout[3:0];
							temp		<= fifo_eth1_icmp_dout;	
						end
						else if(tx_i==1) begin
							tx_i<=0;
							tx_j<=0;
							txd[3:0] <= temp[7:4];
							tx_state <= tx_FCS;
//							case(eth1_icmp_len)
//								16'd40:  tx_state <=tx_FCS;
//								16'd64:  begin 
//									pre_state<=4'd4;
//									icmp24<=8'h20;
//								end
//								default: tx_state <=tx_IDLE;
//							endcase
						end
					end
					
					//Start of Packet
					else begin
						if(tx_i==0) begin
							tx_i<=tx_i+1'b1;
							fifo_eth1_icmp_rd	<=1'b1;
							txd[3:0] 			<= fifo_eth1_icmp_dout[3:0];
							temp					<= fifo_eth1_icmp_dout;	
							 
						end
						else if(tx_i==1) begin
							tx_i<=0;
							tx_j<=tx_j+1'b1;
							fifo_eth1_icmp_rd	<=1'b0;
							txd[3:0] 			<= temp[7:4];
						end
					end
					
				end
				
				//Send ICMP data 24 bytes reconstruct
//				4'd4: begin
//					//End of Packet
//					if(tx_j==23) begin
//						if(tx_i==0) begin
//							tx_i<=tx_i+1'b1;
//							txd[3:0] <= icmp24[3:0];	
//						end
//						else if(tx_i==1) begin
//							tx_i<=0;
//							txd[3:0] <= icmp24[7:4];
//							tx_state <=tx_FCS;
//						end
//					end
//					
//					//Start of Packet
//					else begin
//						if(tx_i==0) begin
//							tx_i<=tx_i+1'b1;
//							txd[3:0] 			<= icmp24[3:0]; 
//						end
//						else if(tx_i==1) begin
//							tx_i<=0;
//							tx_j<=tx_j+1'b1;
//							txd[3:0] 			<= icmp24[7:4];
//							icmp24<=icmp24+1'b1;
//						end
//					end
//				end
				
				default:tx_state<=tx_IDLE;
			endcase	
		end

		tx_FCS:  begin 
			crcen<=1'b0;
			if(tx_i==0)  begin
				txd[3:0] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31]};
				tx_i<=tx_i+1;
			end
			else if(tx_i==1) begin
				txd[3:0]<={~fcs[24], ~fcs[25], ~fcs[26], ~fcs[27]};
				tx_i<=tx_i+1;
			end
			else if(tx_i==2) begin
				txd[3:0]<={~fcs[20], ~fcs[21], ~fcs[22], ~fcs[23]};
				tx_i<=tx_i+1;
			end
			else if(tx_i==3) begin
				txd[3:0]<={~fcs[16], ~fcs[17], ~fcs[18], ~fcs[19]};
				tx_i<=tx_i+1;
			end
			else if(tx_i==4) begin
				txd[3:0]<={~fcs[12], ~fcs[13], ~fcs[14], ~fcs[15]};
				tx_i<=tx_i+1;
			end
			else if(tx_i==5) begin
				txd[3:0]<={~fcs[8], ~fcs[9], ~fcs[10], ~fcs[11]};
				tx_i<=tx_i+1;
			end
			else if(tx_i==6) begin
				txd[3:0]<={~fcs[4], ~fcs[5], ~fcs[6], ~fcs[7]};
				tx_i<=tx_i+1;
			end
			else if(tx_i==7) begin
				txd[3:0]<={~fcs[0], ~fcs[1], ~fcs[2], ~fcs[3]};
				tx_i<=tx_i+1;
			end
			else  begin
				tx_en<=0;
				txd<=4'h00;
				tx_i<=0;
				tx_j<=0;
				tx_state<=tx_IGP;
			end
		end

		tx_IGP:  begin
			tx_en<=1'b0;
			txd<=4'h00;
			if(igp_cnt<4'd12)
				igp_cnt<=igp_cnt+1'b1;
			else
				tx_state<=tx_IDLE;
		end
		
		default: tx_state<=tx_IDLE;
	endcase		
		
	
end
endmodule
	
	
	
	
					
/*
fifo # (8,4096) fifo_adc 
( 	
	.clk(tx_clk), 
	.rst(1'b0),
	.rd(fifo_adc_rd & !fifo_adc_empty), 
	.wr(adc_pkti_ok & !fifo_adc_full),			//enable write adc data to fifo 
	.dataIn(adc_pkti),								//ADC data [7:0] 
	.dataOut(fifo_adc_out), 
	.empty(fifo_adc_empty), 
	.full(fifo_adc_full),
	.cnt(fifo_adc_cnt)	
);*/
	
//typedef struct {
//	logic	[3:0]  fcs;						//Attach fcs for packet?
//	logic	[3:0]  txen;					//Packet enable to transmit
//	logic [15:0] len[3:0];				//length of packet
//	logic [7:0]  data[1499:0][3:0];  //data
//}fifo_t;
//
//fifo_t fifo;

//	case(adc_pkt)
//	
//		ADC_IDLE: begin
//			tx_en<=0;
//			tx_i<=0;
//			tx_j<=0;
//			
//			if(fifo_adc_cnt>=16'd512) begin
//				adc_pkt<=ADC_PRE;	//go to read fifo
//			end
//		end
//		
//		ADC_PRE: begin
//			tx_en<=1;
//			
//			if(tx_j==16'd512) begin
//				if(tx_i==0) begin
//					fifo_adc_rd<=1'b1;
//					txd[3:0] <= fifo_adc_out[3:0];
//					tx_i<=tx_i+1'b1;
//				end
//				else if(tx_i==1) begin
//					fifo_adc_rd<=1'b0;
//					txd[3:0] <= fifo_adc_out[7:4];
//					adc_pkt<=ADC_IDLE;
//					tx_i<=0;
//				end
//			end
//			
//			else begin
//				if(tx_i==0) begin
//					fifo_adc_rd<=1'b1;
//					txd[3:0] <= fifo_adc_out[3:0];
//					tx_i<=tx_i+1'b1;
//				end
//				else if(tx_i==1) begin
//					fifo_adc_rd<=1'b0;
//					txd[3:0] <= fifo_adc_out[7:4];
//					tx_i<=16'd0;
//					tx_j<=tx_j+1'b1;
//				end
//			end
//		end
//		
//		default: adc_pkt<=ADC_IDLE;
//	endcase			
	
	
	
//			if(adc_pkti_ok)begin
				
				//Fill ADC Data
//				if(adc_cnt<16'd704) begin
//					tx_adc_data[adc_cnt]  <= adc_pkti[7:0];
//					adc_cnt<=adc_cnt+1'b1;
//				end
//				else begin
//					adc_cnt<= 16'd0;
					
					
					
					
//					ADC_IPv4_id<=ADC_IPv4_id+1'b1;	//Increment ID
//				
//					tx_adc_data[0]  <=8'h55;	//PREAMBULE  
//					tx_adc_data[1]  <=8'h55;
//					tx_adc_data[2]  <=8'h55;
//					tx_adc_data[3]  <=8'h55;
//					tx_adc_data[4]  <=8'h55;
//					tx_adc_data[5]  <=8'h55;
//					tx_adc_data[6]  <=8'h55;
//					tx_adc_data[7]	 <=8'hD5; 
//					
//					tx_adc_data[8]  <=8'hFF;	//Dest MAC  
//					tx_adc_data[9]  <=8'hFF;
//					tx_adc_data[10] <=8'hFF;
//					tx_adc_data[11] <=8'hFF;
//					tx_adc_data[12] <=8'hFF;
//					tx_adc_data[13] <=8'hFF;	
//
//					tx_adc_data[14] <=LOCAL_MAC[47:40];		//Src MAC
//					tx_adc_data[15] <=LOCAL_MAC[39:32];
//					tx_adc_data[16] <=LOCAL_MAC[31:24];
//					tx_adc_data[17] <=LOCAL_MAC[23:16];
//					tx_adc_data[18] <=LOCAL_MAC[15:8];
//					tx_adc_data[19] <=LOCAL_MAC[7:0];	
//
//					tx_adc_data[20] <=8'h08;				//TYPE 
//					tx_adc_data[21] <=8'h00;
//					tx_adc_data[22] <=8'h45;
//					tx_adc_data[23] <=8'h00;
//					tx_adc_data[24] <=8'h05;				//Len 20 + 8 + 1408 = 1436 -> 059C
//					tx_adc_data[25] <=8'h9C;	
//					tx_adc_data[26] <=ADC_IPv4_id[15:8];
//					tx_adc_data[27] <=ADC_IPv4_id[7:0];
//					tx_adc_data[28] <=8'h40;				//Flags
//					tx_adc_data[29] <=8'h00;
//					tx_adc_data[30] <=8'h40;				//TTL
//					tx_adc_data[31] <=8'h11;				//UDP
//					tx_adc_data[32] <=8'h00;				//CRC
//					tx_adc_data[33] <=8'h00;
//					tx_adc_data[34] <=LOCAL_IP[31:24];	//My IP   
//					tx_adc_data[35] <=LOCAL_IP[23:16];
//					tx_adc_data[36] <=LOCAL_IP[15:8];
//					tx_adc_data[37] <=LOCAL_IP[7:0];
//					tx_adc_data[38] <=8'd192;				//Dst IP
//					tx_adc_data[39] <=8'd168;
//					tx_adc_data[40] <=8'd4;
//					tx_adc_data[41] <=8'd1;
//					
//					tx_adc_data[42] <=8'hBB;	//SP
//					tx_adc_data[43] <=8'h80;
//					tx_adc_data[44] <=8'hBB;	//DP
//					tx_adc_data[45] <=8'h80;
//					tx_adc_data[46] <=8'h05;	//Length
//					tx_adc_data[47] <=8'h88;
//					tx_adc_data[48] <=8'h00;	//CRC
//					tx_adc_data[49] <=8'h00;
					
					
					
//					if(!fifo.len[0])
//						adc_wr<=0;
//					else if(!fifo.len[1]) 
//						adc_wr<=1;
//					else if(!fifo.len[2]) 
//						adc_wr<=2;
//					else if(!fifo.len[3]) 
//						adc_wr<=3;
//					else
//						adc_pkt<=ADC_IDLE;
						
						
					//adc_pkt<=ADC_PRE;	
					//adc_n<=16'd0;
				//end

				/*
				//DATA
				for(adc_n=50, adc_m=0; adc_n<1458; adc_n=adc_n+1, adc_m=adc_m+1) begin
					tx_adc_data[adc_n] <= adc_pkti[adc_m];
				end
				*/
//			end
//		end
//		
//		ADC_PRE: begin
		
		
		
		
		
		
//			fifo.fcs[adc_wr] <=1;								//Enable FCS
//			fifo.len[adc_wr] <=1457;							//Set len of packet
//			for(adc_n=0; adc_n<1458; adc_n=adc_n+1) begin
//				fifo.data[adc_n][wr] <=tx_adc_data[adc_n];	//Fill fifo
//			end
//			adc_pkt<=ADC_IDLE;
//			fifo.txen[adc_wr] <=1;//Enable to transmit

			/*if(adc_n<16'd704) begin
				fifo.data[adc_n][adc_wr] <=tx_adc_data[adc_n];
				adc_n<=adc_n+1'b1;
			end
			else begin
				adc_pkt<=ADC_IDLE;
				fifo.txen[adc_wr] <=1;//Enable to transmit
			end*/
			
//		end
		
//		default: adc_pkt<=ADC_IDLE;
//	endcase
		
	
	
//*******************************************************//
//*Send packet from fifo										  *//
//*******************************************************//

//	case(tx_state)
//		tx_IDLE: begin
//			crcen<=0;
//			crcrst<=1;
//			tx_en<=0;
//			
//			tx_i<=0;
//			tx_j<=0;
//			igp_cnt<=4'd0;
//		end
//		
//		tx_PKT:  begin
//			tx_en<=1;
//		end
//		
//		tx_FCS:  begin 
//			crcen<=1'b0;
//			if(tx_i==0)  begin
//				txd[3:0] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==1) begin
//				txd[3:0]<={~fcs[24], ~fcs[25], ~fcs[26], ~fcs[27]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==2) begin
//				txd[3:0]<={~fcs[20], ~fcs[21], ~fcs[22], ~fcs[23]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==3) begin
//				txd[3:0]<={~fcs[16], ~fcs[17], ~fcs[18], ~fcs[19]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==4) begin
//				txd[3:0]<={~fcs[12], ~fcs[13], ~fcs[14], ~fcs[15]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==5) begin
//				txd[3:0]<={~fcs[8], ~fcs[9], ~fcs[10], ~fcs[11]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==6) begin
//				txd[3:0]<={~fcs[4], ~fcs[5], ~fcs[6], ~fcs[7]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==7) begin
//				txd[3:0]<={~fcs[0], ~fcs[1], ~fcs[2], ~fcs[3]};
//				tx_i<=tx_i+1;
//			end
//			else  begin
//				tx_en<=0;
//				txd<=4'h00;
//				tx_i<=0;
//				tx_j<=0;
//				tx_state<=tx_IGP;
//			end
//		end
//
//		tx_IGP:  begin
//			tx_en<=1'b0;
//			txd<=4'h00;
//			if(igp_cnt<4'd12)
//				igp_cnt<=igp_cnt+1'b1;
//			else
//				tx_state<=tx_IDLE;
//		end
//		
//		default: tx_state<=tx_IDLE;
//	endcase
	
	
//*******************************************************//
//*Send packet from fifo										  *//
//*******************************************************//
//	case(tx_state)
//		tx_IDLE: begin
//
//			if( (fifo.len[0] > 0) & (fifo.txen[0]) ) begin
//				rd<=0;
//				tx_state<=tx_PKT;
//			end
//			else if( (fifo.len[1] > 0) & (fifo.txen[1]) ) begin
//				rd<=1;
//				tx_state<=tx_PKT;
//			end
//			else if( (fifo.len[2] > 0) & (fifo.txen[2]) ) begin
//				rd<=2;
//				tx_state<=tx_PKT;
//			end
//			else if( (fifo.len[3] > 0) & (fifo.txen[3]) ) begin
//				rd<=3;
//				tx_state<=tx_PKT;
//			end
//			
//			crcen<=0;
//			crcrst<=1;
//			
//			tx_en<=0;
//			tx_i<=0;
//			tx_j<=0;
//			igp_cnt<=4'd0;
//		end
//		
//		tx_PKT:  begin
//			tx_en<=1;
//			
//			//CRC [enable or disable] for packet
//			if(fifo.fcs[rd]) begin 
//				if(tx_j<=7) begin
//						crcen<=0;  	//Disable module FCS
//						crcrst<=1; 	//Reset value FCS 
//					end
//					else begin
//						crcen<=1;	//Enable module FCS
//						crcrst<=0; 
//					end
//			end
//			
//			if(tx_j==fifo.len[rd]) begin
//				if(tx_i==0) begin
//					txd[3:0] <= fifo.data[tx_j][rd][3:0];
//					tx_i<=tx_i+1'b1;
//				end
//				else if(tx_i==1) begin
//					txd[3:0] <= fifo.data[tx_j][rd][7:4];
//					fifo.len[rd]<=16'd0;	//clear len
//					fifo.txen[rd]<=1'b0;	//clear en
//					if(fifo.fcs[rd]) begin
//						fifo.fcs[rd] <= 0;
//						tx_state<=tx_FCS;
//					end
//					else
//						tx_state<=tx_IGP;
//					tx_i<=0;
//				end
//			end
//			
//			else begin
//				if(tx_i==0) begin
//					 txd[3:0] <= fifo.data[tx_j][rd][3:0];
//					 tx_i<=tx_i+1'b1;
//				end
//				else if(tx_i==1) begin
//					 txd[3:0] <= fifo.data[tx_j][rd][7:4];
//					 tx_i<=16'd0;
//					 tx_j<=tx_j+1'b1;
//				end
//			end
//		end
//		
//		tx_FCS:  begin 
//			crcen<=1'b0;
//			if(tx_i==0)  begin
//				txd[3:0] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==1) begin
//				txd[3:0]<={~fcs[24], ~fcs[25], ~fcs[26], ~fcs[27]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==2) begin
//				txd[3:0]<={~fcs[20], ~fcs[21], ~fcs[22], ~fcs[23]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==3) begin
//				txd[3:0]<={~fcs[16], ~fcs[17], ~fcs[18], ~fcs[19]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==4) begin
//				txd[3:0]<={~fcs[12], ~fcs[13], ~fcs[14], ~fcs[15]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==5) begin
//				txd[3:0]<={~fcs[8], ~fcs[9], ~fcs[10], ~fcs[11]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==6) begin
//				txd[3:0]<={~fcs[4], ~fcs[5], ~fcs[6], ~fcs[7]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==7) begin
//				txd[3:0]<={~fcs[0], ~fcs[1], ~fcs[2], ~fcs[3]};
//				tx_i<=tx_i+1;
//			end
//			else  begin
//				tx_en<=0;
//				txd<=4'h00;
//				tx_i<=0;
//				tx_j<=0;
//				tx_state<=tx_IGP;
//			end
//		end
//
//		tx_IGP:  begin
//			tx_en<=1'b0;
//			txd<=4'h00;
//			if(igp_cnt<4'd12)
//				igp_cnt<=igp_cnt+1'b1;
//			else
//				tx_state<=tx_IDLE;
//		end
//		
//		default: tx_state<=tx_IDLE;
//	endcase	
	
	
	
	
	
	
	//********************************************************//
	//*Prepare ETH packet and put it in fifo for transmission*//
	//********************************************************//
//	case(eth_pkt)
//		ETH_IDLE: begin
//			pre_i<=0;
//			pre_j<=0;
//			wr<=0;
//			m<=0;
//			n<=0;
//			eth_state<=4'd0;
//			
//			eth_pkto_ok  <= 1'b0;
//			eth_pkto_len <= 16'd0;
//
//			if(arp_ok)
//				eth_pkt<=ETH_ARP;
//			else if(icmp_ok)
//				eth_pkt<=ETH_ICMP;
//			else if(udp_ok)
//				eth_pkt<=ETH_UDP;
//		end
//		
//		ETH_ARP:  begin
//			case(eth_state)	
//				4'd0: begin //Prepare ARP
//					tx_arp_data[0]  <=8'h55;               //PREAMBULE  
//					tx_arp_data[1]  <=8'h55;
//					tx_arp_data[2]  <=8'h55;
//					tx_arp_data[3]  <=8'h55;
//					tx_arp_data[4]  <=8'h55;
//					tx_arp_data[5]  <=8'h55;
//					tx_arp_data[6]  <=8'h55;
//					tx_arp_data[7]	 <=8'hD5; 
//					
//					tx_arp_data[8]  <=ARP_MAC_src[47:40];	//Dest MAC  
//					tx_arp_data[9]  <=ARP_MAC_src[39:32];
//					tx_arp_data[10] <=ARP_MAC_src[31:24];
//					tx_arp_data[11] <=ARP_MAC_src[23:16];
//					tx_arp_data[12] <=ARP_MAC_src[15:8];
//					tx_arp_data[13] <=ARP_MAC_src[7:0];	
//					
//					tx_arp_data[14] <=LOCAL_MAC[47:40];		//Src MAC
//					tx_arp_data[15] <=LOCAL_MAC[39:32];
//					tx_arp_data[16] <=LOCAL_MAC[31:24];
//					tx_arp_data[17] <=LOCAL_MAC[23:16];
//					tx_arp_data[18] <=LOCAL_MAC[15:8];
//					tx_arp_data[19] <=LOCAL_MAC[7:0];	
//					
//					tx_arp_data[20] <=ETH_type[15:8]; 		//TYPE 
//					tx_arp_data[21] <=ETH_type[7:0];
//					tx_arp_data[22] <=8'h00;					//HType
//					tx_arp_data[23] <=8'h01;
//					tx_arp_data[24] <=8'h08;					//IPv4
//					tx_arp_data[25] <=8'h00;
//					tx_arp_data[26] <=8'h06;					//Hsz
//					tx_arp_data[27] <=8'h04;					//Psz
//					tx_arp_data[28] <=8'h00;					//Opcode 0x0002 reply
//					tx_arp_data[29] <=8'h02;
//					tx_arp_data[30] <=LOCAL_MAC[47:40];		//My MAC  
//					tx_arp_data[31] <=LOCAL_MAC[39:32];
//					tx_arp_data[32] <=LOCAL_MAC[31:24];
//					tx_arp_data[33] <=LOCAL_MAC[23:16];
//					tx_arp_data[34] <=LOCAL_MAC[15:8];
//					tx_arp_data[35] <=LOCAL_MAC[7:0];
//					tx_arp_data[36] <=LOCAL_IP[31:24];		//My IP   
//					tx_arp_data[37] <=LOCAL_IP[23:16];
//					tx_arp_data[38] <=LOCAL_IP[15:8];
//					tx_arp_data[39] <=LOCAL_IP[7:0];
//					tx_arp_data[40] <=ARP_MAC_src[47:40];	//Target MAC 
//					tx_arp_data[41] <=ARP_MAC_src[39:32];
//					tx_arp_data[42] <=ARP_MAC_src[31:24];
//					tx_arp_data[43] <=ARP_MAC_src[23:16];
//					tx_arp_data[44] <=ARP_MAC_src[15:8];
//					tx_arp_data[45] <=ARP_MAC_src[7:0];	
//					tx_arp_data[46] <=ARP_IP_src[31:24];	//Target IP   
//					tx_arp_data[47] <=ARP_IP_src[23:16];
//					tx_arp_data[48] <=ARP_IP_src[15:8];
//					tx_arp_data[49] <=ARP_IP_src[7:0];
//
//					if(!fifo.len[0])
//						wr<=0;
//					else if(!fifo.len[1]) 
//						wr<=1;
//					else if(!fifo.len[2]) 
//						wr<=2;
//					else if(!fifo.len[3]) 
//						wr<=3;
//					else
//						eth_pkt<=PRE_IDLE;
//					
//					eth_state<=4'd1;
//					pre_i<=0;
//					pre_j<=0; 
//				end
//	
//				4'd1: begin //Put packet to FIFO	
//					fifo.fcs[wr] <=1;								//Enable FCS
//					fifo.txen[wr] <=1;							//Enable to transmit
//					fifo.len[wr] <=49;							//Set len of packet
//					
//					for(n=0; n<50; n=n+1) begin
//						fifo.data[n][wr] <=tx_arp_data[n];	//Fill fifo
//					end
//					
//					eth_pkt<=PRE_IDLE;
//				end
//				
//				default: eth_state<=0;
//			endcase
//		end
//		
//		ETH_ICMP: begin
//			case(eth_state)
//				4'd0: begin //Prepare ICMP
//					icmp_cnt<=icmp_cnt+1'b1;
//					//PREAMBULE  
//					tx_icmp_data[0]  <=8'h55;					
//					tx_icmp_data[1]  <=8'h55;
//					tx_icmp_data[2]  <=8'h55;
//					tx_icmp_data[3]  <=8'h55;
//					tx_icmp_data[4]  <=8'h55;
//					tx_icmp_data[5]  <=8'h55;
//					tx_icmp_data[6]  <=8'h55;
//					tx_icmp_data[7]  <=8'hD5; 
//					//ETH
//					tx_icmp_data[8]  <=ETH_MAC_src[47:40];	//Dest MAC  
//					tx_icmp_data[9]  <=ETH_MAC_src[39:32];
//					tx_icmp_data[10] <=ETH_MAC_src[31:24];
//					tx_icmp_data[11] <=ETH_MAC_src[23:16];
//					tx_icmp_data[12] <=ETH_MAC_src[15:8];
//					tx_icmp_data[13] <=ETH_MAC_src[7:0];	
//					tx_icmp_data[14] <=LOCAL_MAC[47:40];	//Src MAC
//					tx_icmp_data[15] <=LOCAL_MAC[39:32];
//					tx_icmp_data[16] <=LOCAL_MAC[31:24];
//					tx_icmp_data[17] <=LOCAL_MAC[23:16];
//					tx_icmp_data[18] <=LOCAL_MAC[15:8];
//					tx_icmp_data[19] <=LOCAL_MAC[7:0];	
//					tx_icmp_data[20] <=ETH_type[15:8];		//TYPE 
//					tx_icmp_data[21] <=ETH_type[7:0];
//					//IPv4
//					tx_icmp_data[22] <=8'h45;					//Version number: 4; Header length: 20; 
//					tx_icmp_data[23] <=8'h00;
//					tx_icmp_data[24] <=IPv4_tlen[15:8];		//IPv4 total length
//					tx_icmp_data[25] <=IPv4_tlen[7:0];
//					tx_icmp_data[26] <=icmp_cnt[15:8];		//Package serial number
//					tx_icmp_data[27] <=icmp_cnt[7:0];
//					tx_icmp_data[28] <=8'h00;					//Fragment offset
//					tx_icmp_data[29] <=8'h00;
//					tx_icmp_data[30] <=IPv4_ttl;				//TTL 64 8'h40;
//					tx_icmp_data[31] <=8'h01;					//Protocol: 01(ICMP)
//					tx_icmp_data[32] <=8'h00;					//Header checksum
//					tx_icmp_data[33] <=8'h00;
//					tx_icmp_data[34] <=LOCAL_IP[31:24];		//Source IP   
//					tx_icmp_data[35] <=LOCAL_IP[23:16];
//					tx_icmp_data[36] <=LOCAL_IP[15:8];
//					tx_icmp_data[37] <=LOCAL_IP[7:0];				
//					tx_icmp_data[38] <=IPv4_IP_src[31:24];	//Dest IP
//					tx_icmp_data[39] <=IPv4_IP_src[23:16];
//					tx_icmp_data[40] <=IPv4_IP_src[15:8];
//					tx_icmp_data[41] <=IPv4_IP_src[7:0];
//					
//					case(ICMP_tlen)
//						16'd40: begin //ICMP packet for Windows OS
//							tx_icmp_data[42]  <=8'h00;						//TYPE:0 (Echo (ping) reply)
//							tx_icmp_data[43]  <=8'h00;						//CODE:0
//							tx_icmp_data[44]  <=8'h00;						//CHECKSUM
//							tx_icmp_data[45]  <=8'h00;
//							tx_icmp_data[46]  <=ICMP_layer[295:288]; 	//Id BE
//							tx_icmp_data[47]  <=ICMP_layer[287:280];	//Id LE
//							tx_icmp_data[48]  <=ICMP_layer[279:272];	//SN BE
//							tx_icmp_data[49]  <=ICMP_layer[271:264];	//SN LE
//							
//							//Fill data
//							for(n=50, m=0; n<82; n=n+1, m=m+1) begin
//								tx_icmp_data[n] <=ICMP_layer[263-(m*8)-:8];
//							end
//						end
//						
//						16'd64: begin //ICMP packet for Linux OS
//							tx_icmp_data[42]  <=8'h00;						//TYPE:0 (Echo (ping) reply)
//							tx_icmp_data[43]  <=8'h00;						//CODE:0
//							tx_icmp_data[44]  <=8'h00;						//CHECKSUM
//							tx_icmp_data[45]  <=8'h00;
//							
//							//Fill data
//							for(n=46, m=0; n<106; n=n+1, m=m+1) begin
//								tx_icmp_data[n] <=ICMP_layer[487-(m*8)-:8];
//							end
//						end
//						
//						default: eth_pkt<=PRE_IDLE;
//					endcase
//					
//					if(!fifo.len[0])
//						wr<=0;
//					else if(!fifo.len[1]) 
//						wr<=1;
//					else if(!fifo.len[2]) 
//						wr<=2;
//					else if(!fifo.len[3]) 
//						wr<=3;
//					else
//						eth_pkt<=PRE_IDLE;
//					
//					pre_i<=0;
//					pre_j<=0;
//					eth_state<=4'd1;
//				end
//				
//				4'd1: begin //Generate CRC for IPv4 header
//					if(pre_i==0) begin											
//						check_buffer <={tx_icmp_data[24],tx_icmp_data[25]} + {tx_icmp_data[22],tx_icmp_data[23]} +
//											{tx_icmp_data[28],tx_icmp_data[29]} + {tx_icmp_data[26],tx_icmp_data[27]} +
//											{tx_icmp_data[32],tx_icmp_data[33]} + {tx_icmp_data[30],tx_icmp_data[31]} +
//											{tx_icmp_data[36],tx_icmp_data[37]} + {tx_icmp_data[34],tx_icmp_data[35]} +
//											{tx_icmp_data[40],tx_icmp_data[41]} + {tx_icmp_data[38],tx_icmp_data[39]};
//						pre_i<=pre_i+1'b1;
//					end
//					else if(pre_i==1) begin
//						check_buffer[15:0]<=check_buffer[31:16]+check_buffer[15:0];
//						pre_i<=pre_i+1'b1;
//					end
//					else begin
//						tx_icmp_data[32]<=~check_buffer[15:8];
//						tx_icmp_data[33]<=~check_buffer[7:0];
//						pre_i<=0;
//						pre_j<=0;
//						eth_state<=4'd2;
//					end
//				end
//				
//				4'd2: begin //Generate CRC ICMP header
//					if(pre_i==0) begin
//						case(ICMP_tlen)
//							16'd40: begin //for Windows OS
//								check_buffer <={tx_icmp_data[44],tx_icmp_data[45]}   + {tx_icmp_data[42],tx_icmp_data[43]} +
//													{tx_icmp_data[48],tx_icmp_data[49]}   + {tx_icmp_data[46],tx_icmp_data[47]} +
//													{tx_icmp_data[52],tx_icmp_data[53]}   + {tx_icmp_data[50],tx_icmp_data[51]} +
//													{tx_icmp_data[56],tx_icmp_data[57]}   + {tx_icmp_data[54],tx_icmp_data[55]} +
//													{tx_icmp_data[60],tx_icmp_data[61]}   + {tx_icmp_data[58],tx_icmp_data[59]} +	
//													{tx_icmp_data[64],tx_icmp_data[65]}   + {tx_icmp_data[62],tx_icmp_data[63]} +
//													{tx_icmp_data[68],tx_icmp_data[69]}   + {tx_icmp_data[66],tx_icmp_data[67]} +
//													{tx_icmp_data[72],tx_icmp_data[73]}   + {tx_icmp_data[70],tx_icmp_data[71]} +
//													{tx_icmp_data[76],tx_icmp_data[77]}   + {tx_icmp_data[74],tx_icmp_data[75]} +
//													{tx_icmp_data[80],tx_icmp_data[81]}   + {tx_icmp_data[78],tx_icmp_data[79]} ;
//							end
//							
//							16'd64: begin //for Linux OS
//								check_buffer <={tx_icmp_data[44],tx_icmp_data[45]}   + {tx_icmp_data[42],tx_icmp_data[43]} +
//													{tx_icmp_data[48],tx_icmp_data[49]}   + {tx_icmp_data[46],tx_icmp_data[47]} +
//													{tx_icmp_data[52],tx_icmp_data[53]}   + {tx_icmp_data[50],tx_icmp_data[51]} +
//													{tx_icmp_data[56],tx_icmp_data[57]}   + {tx_icmp_data[54],tx_icmp_data[55]} +
//													{tx_icmp_data[60],tx_icmp_data[61]}   + {tx_icmp_data[58],tx_icmp_data[59]} +	
//													{tx_icmp_data[64],tx_icmp_data[65]}   + {tx_icmp_data[62],tx_icmp_data[63]} +
//													{tx_icmp_data[68],tx_icmp_data[69]}   + {tx_icmp_data[66],tx_icmp_data[67]} +
//													{tx_icmp_data[72],tx_icmp_data[73]}   + {tx_icmp_data[70],tx_icmp_data[71]} +
//													{tx_icmp_data[76],tx_icmp_data[77]}   + {tx_icmp_data[74],tx_icmp_data[75]} +
//													{tx_icmp_data[80],tx_icmp_data[81]}   + {tx_icmp_data[78],tx_icmp_data[79]} +
//													{tx_icmp_data[84],tx_icmp_data[85]}   + {tx_icmp_data[82],tx_icmp_data[83]} +
//													{tx_icmp_data[88],tx_icmp_data[89]}   + {tx_icmp_data[86],tx_icmp_data[87]} +
//													{tx_icmp_data[92],tx_icmp_data[93]}   + {tx_icmp_data[90],tx_icmp_data[91]} +
//													{tx_icmp_data[96],tx_icmp_data[97]}   + {tx_icmp_data[94],tx_icmp_data[95]} +
//													{tx_icmp_data[100],tx_icmp_data[101]} + {tx_icmp_data[98],tx_icmp_data[99]} +
//													{tx_icmp_data[104],tx_icmp_data[105]} + {tx_icmp_data[102],tx_icmp_data[103]};	
//							end
//							
//							default: eth_pkt<=PRE_IDLE;
//						endcase
//					
//						pre_i<=pre_i+1'b1;
//					end
//					else if(pre_i==1) begin
//						check_buffer[15:0]<=check_buffer[31:16]+check_buffer[15:0];
//						pre_i<=pre_i+1'b1;
//					end
//					else begin
//						tx_icmp_data[44]<=~check_buffer[15:8];
//						tx_icmp_data[45]<=~check_buffer[7:0];
//						pre_i<=0;
//						pre_j<=0;
//						eth_state<=4'd3;
//					end
//				end
//				
//				
//				
//				4'd3: begin //Put packet to FIFO	
//					case(ICMP_tlen)
//						16'd40: begin 										//For Windows OS
//							fifo.fcs[wr] <=1;								//Enable FCS
//							fifo.txen[wr] <=1;							//Enable to transmit
//							fifo.len[wr] <=81;							//Set len of packet
//							
//							for(n=0; n<82; n=n+1) begin
//								fifo.data[n][wr] <=tx_icmp_data[n];	//Fill fifo
//							end
//						end
//						
//						16'd64: begin 										//For Linux OS
//							fifo.fcs[wr] <=1;								//Enable FCS
//							fifo.txen[wr] <=1;							//Enable to transmit
//							fifo.len[wr] <=105;							//Set len of packet
//							
//							for(n=0; n<106; n=n+1) begin
//								fifo.data[n][wr] <=tx_icmp_data[n];	//Fill fifo
//							end
//						end
//						
//						default: prepare_pkt<=PRE_IDLE;
//					endcase
//					eth_pkt<=PRE_IDLE;
//				end
//				
//				default: eth_state<=0;
//			endcase
//		end
//		
//		ETH_UDP:  begin //Prepare UDP packet for retransmit in to MAC2 and for answer to MAC1
//			case(eth_state)
//				4'd0: begin
//					pre_i<=0;
//					pre_j<=0;
//					
//					//Resend broadcast UDP packet
//					//PREAMBULE 					
//					/*eth_pkto[0] <=8'h55;                
//					eth_pkto[1] <=8'h55;
//					eth_pkto[2] <=8'h55;
//					eth_pkto[3] <=8'h55;
//					eth_pkto[4] <=8'h55;
//					eth_pkto[5] <=8'h55;
//					eth_pkto[6] <=8'h55;
//					eth_pkto[7]	<=8'hD5; 
//					
//					//Fill Packet ETH_layer
//					for(rx_n=8, rx_m=0; rx_n<22; rx_n=rx_n+1, rx_m=rx_m+1) begin
//						eth_pkto[rx_n] <= ETH_layer[111-(rx_m*8)-:8];
//					end
//					
//					//Fill Packet IPv4_layer
//					for(rx_n=22, rx_m=0; rx_n<42; rx_n=rx_n+1, rx_m=rx_m+1) begin
//						eth_pkto[rx_n] <= IPv4_layer[159-(rx_m*8)-:8];
//					end
//					
//					//Fill Packet UDP_layer
//					for(rx_n=42, rx_m=0; rx_n<50; rx_n=rx_n+1, rx_m=rx_m+1) begin
//						eth_pkto[rx_n] <= UDP_layer[63-(rx_m*8)-:8];
//					end
//					
//					//Fill Packet UDP_data
//					for(rx_n=50, rx_m=0; rx_n<60; rx_n=rx_n+1, rx_m=rx_m+1) begin
//						eth_pkto[rx_n] <= UDP_data[79-(rx_m*8)-:8];
//					end
//					
//					//Fill Packet UDP_fcs
//					for(rx_n=60, rx_m=0; rx_n<64; rx_n=rx_n+1, rx_m=rx_m+1) begin
//						eth_pkto[rx_n] <= UDP_fcs[31-(rx_m*8)-:8];
//					end
//					
//					eth_pkto_ok	 <=1'b1;
//					eth_pkto_len <=16'd64;
//					eth_pkt<=PRE_IDLE;*/
//					
//					
//					
//					
//					
//					//pre_state<=2;
//					//dbg_byte_cnt<=16'd0;
//
//					/*pre_state<=1;
//					
//					if(!fifo.len[0])
//						wr<=4'd0;
//					else if(!fifo.len[1]) 
//						wr<=4'd1;
//					else if(!fifo.len[2]) 
//						wr<=4'd2;
//					else if(!fifo.len[3]) 
//						wr<=4'd3;
//					else*/
//						//prepare_pkt<=PRE_IDLE;
//						
//					eth_pkt<=PRE_IDLE;
//				end
//				
//				//for debug
//				/*4'd2: begin
//					pkt_out_en	<=1'b1;
//					pkt_out_len <=16'd64;
//
//					if(dbg_byte_cnt == 63) begin
//						pkt_out_dbg <= pkt_out[dbg_byte_cnt];
//						eth_state<=3;
//					end
//					else begin
//						pkt_out_dbg <= pkt_out[dbg_byte_cnt];
//						dbg_byte_cnt<=dbg_byte_cnt+1'b1;
//					end
//				end
//				
//				4'd3: begin
//					eth_state<=0;
//					dbg_byte_cnt<=16'd0;
//					pkt_out_en	<=1'b0;
//					pkt_out_len <=16'd0;
//					prepare_pkt<=PRE_IDLE;
//				end*/
//				
//				
//				4'd1: begin
//					fifo.fcs[wr] <=1;					//Enable FCS
//					fifo.len[wr] <=32;				//Set len of packet
//					for(n=0; n<74; n=n+1) begin
//						fifo.data[n][wr] <=8'd0;	//fill fifo
//					end
//					eth_pkt<=PRE_IDLE;
//				end
//				
//				default: eth_state<=0;
//			endcase
//		end
//		
//		
//		
//		default: eth_pkt<=ETH_IDLE;
//	endcase

	
	
	
//	//*******************************************************//
//	//*Send packet from fifo										  *//
//	//*******************************************************//
//	case(tx_state)
//		tx_IDLE: begin
//
//			if( (fifo.len[0] > 0) & (fifo.txen[0]) ) begin
//				rd<=0;
//				tx_state<=tx_PKT;
//			end
//			else if( (fifo.len[1] > 0) & (fifo.txen[1]) ) begin
//				rd<=1;
//				tx_state<=tx_PKT;
//			end
//			else if( (fifo.len[2] > 0) & (fifo.txen[2]) ) begin
//				rd<=2;
//				tx_state<=tx_PKT;
//			end
//			else if( (fifo.len[3] > 0) & (fifo.txen[3]) ) begin
//				rd<=3;
//				tx_state<=tx_PKT;
//			end
//			
//			crcen<=0;
//			crcrst<=1;
//			
//			tx_en<=0;
//			tx_i<=0;
//			tx_j<=0;
//			igp_cnt<=4'd0;
//		end
//		
//		tx_PKT:  begin
//			tx_en<=1;
//			
//			//CRC [enable or disable] for packet
//			if(fifo.fcs[rd]) begin 
//				if(tx_j<=7) begin
//						crcen<=0;  	//Disable module FCS
//						crcrst<=1; 	//Reset value FCS 
//					end
//					else begin
//						crcen<=1;	//Enable module FCS
//						crcrst<=0; 
//					end
//			end
//			
//			if(tx_j==fifo.len[rd]) begin
//				if(tx_i==0) begin
//					txd[3:0] <= fifo.data[tx_j][rd][3:0];
//					tx_i<=tx_i+1'b1;
//				end
//				else if(tx_i==1) begin
//					txd[3:0] <= fifo.data[tx_j][rd][7:4];
//					fifo.len[rd]<=16'd0;	//clear len
//					fifo.txen[rd]<=1'b0;	//clear en
//					if(fifo.fcs[rd]) begin
//						fifo.fcs[rd] <= 0;
//						tx_state<=tx_FCS;
//					end
//					else
//						tx_state<=tx_IGP;
//					tx_i<=0;
//				end
//			end
//			
//			else begin
//				if(tx_i==0) begin
//					 txd[3:0] <= fifo.data[tx_j][rd][3:0];
//					 tx_i<=tx_i+1'b1;
//				end
//				else if(tx_i==1) begin
//					 txd[3:0] <= fifo.data[tx_j][rd][7:4];
//					 tx_i<=16'd0;
//					 tx_j<=tx_j+1'b1;
//				end
//			end
//		end
//		
//		tx_FCS:  begin 
//			crcen<=1'b0;
//			if(tx_i==0)  begin
//				txd[3:0] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==1) begin
//				txd[3:0]<={~fcs[24], ~fcs[25], ~fcs[26], ~fcs[27]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==2) begin
//				txd[3:0]<={~fcs[20], ~fcs[21], ~fcs[22], ~fcs[23]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==3) begin
//				txd[3:0]<={~fcs[16], ~fcs[17], ~fcs[18], ~fcs[19]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==4) begin
//				txd[3:0]<={~fcs[12], ~fcs[13], ~fcs[14], ~fcs[15]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==5) begin
//				txd[3:0]<={~fcs[8], ~fcs[9], ~fcs[10], ~fcs[11]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==6) begin
//				txd[3:0]<={~fcs[4], ~fcs[5], ~fcs[6], ~fcs[7]};
//				tx_i<=tx_i+1;
//			end
//			else if(tx_i==7) begin
//				txd[3:0]<={~fcs[0], ~fcs[1], ~fcs[2], ~fcs[3]};
//				tx_i<=tx_i+1;
//			end
//			else  begin
//				tx_en<=0;
//				txd<=4'h00;
//				tx_i<=0;
//				tx_j<=0;
//				tx_state<=tx_IGP;
//			end
//		end
//
//		tx_IGP:  begin
//			tx_en<=1'b0;
//			txd<=4'h00;
//			if(igp_cnt<4'd12)
//				igp_cnt<=igp_cnt+1'b1;
//			else
//				tx_state<=tx_IDLE;
//		end
//		
//		default: tx_state<=tx_IDLE;
//	endcase
//end
//
//endmodule


		
		
		
		
		
		
		
/*
				4'd3: begin //Prepare for FCS of packet
					if(pre_j<=7) begin
						crcen<=0;  	//disable crc
						crcrst<=1; 	//reset crc 
					end
					else begin
						crcen<=1;	//enable crc
						crcrst<=0; 
					end
					
					//*****************************************
					case(ICMP_tlen)
						16'd40: begin //for Windows OS
							if(pre_j==81) begin
								tx_data[7:0] <= tx_icmp_data[pre_j][7:0];
								pre_state<=4'd4;
							end
							else begin
								tx_data[7:0] <= tx_icmp_data[pre_j][7:0];
								pre_j<=pre_j+1'b1;
							end
						end
						
						16'd64: begin //for Linux OS
							if(pre_j==105) begin
								tx_data[7:0] <= tx_icmp_data[pre_j][7:0];
								pre_state<=4'd4;
							end
							else begin
								tx_data[7:0] <= tx_icmp_data[pre_j][7:0];
								pre_j<=pre_j+1'b1;
							end
						end
						
						default: prepare_pkt<=PRE_IDLE;
					endcase	
				end
				
				4'd4: begin //Disable FCS module
					pre_i<=0;
					pre_j<=0;
					crcen<=1'b0;		
					pre_state<=4'd5;
				end
				
				4'd5: begin //Attach Frame check sequence
					case(ICMP_tlen)
						16'd40: begin //for Windows OS
							tx_icmp_data[82] <= {~fcs[24], ~fcs[25], ~fcs[26], ~fcs[27],   ~fcs[28], ~fcs[29], ~fcs[30], ~fcs[31]};
							tx_icmp_data[83] <= {~fcs[16], ~fcs[17], ~fcs[18], ~fcs[19],   ~fcs[20], ~fcs[21], ~fcs[22], ~fcs[23]};
							tx_icmp_data[84] <= {~fcs[8],  ~fcs[9],  ~fcs[10], ~fcs[11],   ~fcs[12], ~fcs[13], ~fcs[14], ~fcs[15]};
							tx_icmp_data[85] <= {~fcs[0],  ~fcs[1],  ~fcs[2],  ~fcs[3],    ~fcs[4],  ~fcs[5],  ~fcs[6],  ~fcs[7] };
						end
						
						16'd64: begin //for Linux OS
							tx_icmp_data[106] <= {~fcs[24], ~fcs[25], ~fcs[26], ~fcs[27],   ~fcs[28], ~fcs[29], ~fcs[30], ~fcs[31]};
							tx_icmp_data[107] <= {~fcs[16], ~fcs[17], ~fcs[18], ~fcs[19],   ~fcs[20], ~fcs[21], ~fcs[22], ~fcs[23]};
							tx_icmp_data[108] <= {~fcs[8],  ~fcs[9],  ~fcs[10], ~fcs[11],   ~fcs[12], ~fcs[13], ~fcs[14], ~fcs[15]};
							tx_icmp_data[109] <= {~fcs[0],  ~fcs[1],  ~fcs[2],  ~fcs[3],    ~fcs[4],  ~fcs[5],  ~fcs[6],  ~fcs[7] };
						end
						
						default: prepare_pkt<=PRE_IDLE;
					endcase
					
					if(!fifo.lock[0])
						wr<=4'd0;
					else if(!fifo.lock[1]) 
						wr<=4'd1;
					else if(!fifo.lock[2]) 
						wr<=4'd2;
					else if(!fifo.lock[3]) 
						wr<=4'd3;
					else
						prepare_pkt<=PRE_IDLE;
						
					pre_i<=0;
					pre_j<=0;
					pre_state<=4'd6;
				end*/		
		
		
		
		
	
	
	
	//*******************************************************//
	//*Send packet from fifo										  *//
	//*******************************************************//
	/*case(tx_state)
		
		tx_IDLE: begin
			tx_en<=0;
			mem_i<=0;
			mem_j<=0;
			
			tx_arp_data[0]  <=8'h55;               //PREAMBULE  
			tx_arp_data[1]  <=8'h55;
			tx_arp_data[2]  <=8'h55;
			tx_arp_data[3]  <=8'h55;
			tx_arp_data[4]  <=8'h55;
			tx_arp_data[5]  <=8'h55;
			tx_arp_data[6]  <=8'h55;
			tx_arp_data[7]	 <=8'hD5; 

			tx_arp_data[8]  <=ARP_MAC_src[47:40];	//Dest MAC  
			tx_arp_data[9]  <=ARP_MAC_src[39:32];
			tx_arp_data[10] <=ARP_MAC_src[31:24];
			tx_arp_data[11] <=ARP_MAC_src[23:16];
			tx_arp_data[12] <=ARP_MAC_src[15:8];
			tx_arp_data[13] <=ARP_MAC_src[7:0];	

			tx_arp_data[14] <=LOCAL_MAC[47:40];		//Src MAC
			tx_arp_data[15] <=LOCAL_MAC[39:32];
			tx_arp_data[16] <=LOCAL_MAC[31:24];
			tx_arp_data[17] <=LOCAL_MAC[23:16];
			tx_arp_data[18] <=LOCAL_MAC[15:8];
			tx_arp_data[19] <=LOCAL_MAC[7:0];
			tx_arp_data[20] <=ETH_type[15:8]; 		//TYPE 
			tx_arp_data[21] <=ETH_type[7:0];
			tx_arp_data[22] <=8'h00;					//HType
			tx_arp_data[23] <=8'h01;
			tx_arp_data[24] <=8'h08;					//IPv4
			tx_arp_data[25] <=8'h00;
			tx_arp_data[26] <=8'h06;					//Hsz
			tx_arp_data[27] <=8'h04;					//Psz
			tx_arp_data[28] <=8'h00;					//Opcode 0x0002 reply
			tx_arp_data[29] <=8'h02;
			tx_arp_data[30] <=LOCAL_MAC[47:40];		//My MAC  
			tx_arp_data[31] <=LOCAL_MAC[39:32];
			tx_arp_data[32] <=LOCAL_MAC[31:24];
			tx_arp_data[33] <=LOCAL_MAC[23:16];
			tx_arp_data[34] <=LOCAL_MAC[15:8];
			tx_arp_data[35] <=LOCAL_MAC[7:0];
			tx_arp_data[36] <=LOCAL_IP[31:24];		//My IP   
			tx_arp_data[37] <=LOCAL_IP[23:16];
			tx_arp_data[38] <=LOCAL_IP[15:8];
			tx_arp_data[39] <=LOCAL_IP[7:0];
			tx_arp_data[40] <=ARP_MAC_src[47:40];	//Target MAC 
			tx_arp_data[41] <=ARP_MAC_src[39:32];
			tx_arp_data[42] <=ARP_MAC_src[31:24];
			tx_arp_data[43] <=ARP_MAC_src[23:16];
			tx_arp_data[44] <=ARP_MAC_src[15:8];
			tx_arp_data[45] <=ARP_MAC_src[7:0];	
			tx_arp_data[46] <=ARP_IP_src[31:24];	//Target IP   
			tx_arp_data[47] <=ARP_IP_src[23:16];
			tx_arp_data[48] <=ARP_IP_src[15:8];
			tx_arp_data[49] <=ARP_IP_src[7:0];
			tx_arp_data[50] <=8'd00; 					//Padding
			tx_arp_data[51] <=8'd00;
			tx_arp_data[52] <=8'd00;
			tx_arp_data[53] <=8'd00;
			tx_arp_data[54] <=8'd00;
			tx_arp_data[55] <=8'd00;
			tx_arp_data[56] <=8'd00;
			tx_arp_data[57] <=8'd00;
			tx_arp_data[58] <=8'd00;
			tx_arp_data[59] <=8'd00;
			tx_arp_data[60] <=8'd00;
			tx_arp_data[61] <=8'd00;
			tx_arp_data[62] <=8'd00;
			tx_arp_data[63] <=8'd00;
			tx_arp_data[64] <=8'd00;
			tx_arp_data[65] <=8'd00;
			tx_arp_data[66] <=8'd00;
			tx_arp_data[67] <=8'd00;	
			
			if(arp_ok) begin
				tx_state<=tx_ARP;
				
				for(n=0; n<68; n=n+1) begin
					fifo.data[n][0] <=tx_arp_data[n];	//fill fifo
				end
				fifo.lock[0]<=1;		//set lock for packet
				fifo.len[0] <=67;		//set len  of  packet
			end
				
		end
		
		
		
		
		tx_ARP: begin
			tx_en<=1;
			if(mem_j==fifo.len[0]) begin
				if(mem_i==0) begin
					txd[3:0] <= fifo.data[mem_j][0][3:0];
					mem_i<=mem_i+1;
				end
				else if(mem_i==1) begin
					txd[3:0] <= fifo.data[mem_j][0][7:4];
					fifo.lock[0]<=1'b0;	//unset lock
					fifo.len[0]<=16'd0;	//clear len
					
					mem_i<=0;
					mem_j<=0;
					tx_state<=tx_IDLE;
				end
			end
			
			else begin
				if(mem_i==0) begin
					 txd[3:0] <= fifo.data[mem_j][0][3:0];
					 mem_i<=mem_i+1;
				end
				else if(mem_i==1) begin
					 txd[3:0] <= fifo.data[mem_j][0][7:4];
					 mem_i<=0;
					 mem_j<=mem_j+1;
				end
			end
			
			
		end
		
		default: tx_state<=tx_IDLE;
	endcase*/



	
	//Prepare packet and put it in memory for transmission
	/*case(tx_state)
		
		tx_IDLE: begin
			tx_alg_state<=8'd0;
			tx_byte_cnt<=16'd0;
			mem_i<=0;
			mem_j<=0;
			
			if(arp_ok)
				tx_state<=tx_ARP;
		end
		
		tx_ARP: begin
			case(tx_alg_state)	
				8'd0: begin //Prepare ARP
					tx_arp_data[0]  <=8'h55;               //PREAMBULE  
					tx_arp_data[1]  <=8'h55;
					tx_arp_data[2]  <=8'h55;
					tx_arp_data[3]  <=8'h55;
					tx_arp_data[4]  <=8'h55;
					tx_arp_data[5]  <=8'h55;
					tx_arp_data[6]  <=8'h55;
					tx_arp_data[7]	 <=8'hD5; 
					
					tx_arp_data[8]  <=ARP_MAC_src[47:40];	//Dest MAC  
					tx_arp_data[9]  <=ARP_MAC_src[39:32];
					tx_arp_data[10] <=ARP_MAC_src[31:24];
					tx_arp_data[11] <=ARP_MAC_src[23:16];
					tx_arp_data[12] <=ARP_MAC_src[15:8];
					tx_arp_data[13] <=ARP_MAC_src[7:0];	
					
					tx_arp_data[14] <=LOCAL_MAC[47:40];		//Src MAC
					tx_arp_data[15] <=LOCAL_MAC[39:32];
					tx_arp_data[16] <=LOCAL_MAC[31:24];
					tx_arp_data[17] <=LOCAL_MAC[23:16];
					tx_arp_data[18] <=LOCAL_MAC[15:8];
					tx_arp_data[19] <=LOCAL_MAC[7:0];	
					
					tx_arp_data[20] <=ETH_type[15:8]; 		//TYPE 
					tx_arp_data[21] <=ETH_type[7:0];
					tx_arp_data[22] <=8'h00;					//HType
					tx_arp_data[23] <=8'h01;
					tx_arp_data[24] <=8'h08;					//IPv4
					tx_arp_data[25] <=8'h00;
					tx_arp_data[26] <=8'h06;					//Hsz
					tx_arp_data[27] <=8'h04;					//Psz
					tx_arp_data[28] <=8'h00;					//Opcode 0x0002 reply
					tx_arp_data[29] <=8'h02;
					tx_arp_data[30] <=LOCAL_MAC[47:40];		//My MAC  
					tx_arp_data[31] <=LOCAL_MAC[39:32];
					tx_arp_data[32] <=LOCAL_MAC[31:24];
					tx_arp_data[33] <=LOCAL_MAC[23:16];
					tx_arp_data[34] <=LOCAL_MAC[15:8];
					tx_arp_data[35] <=LOCAL_MAC[7:0];
					tx_arp_data[36] <=LOCAL_IP[31:24];		//My IP   
					tx_arp_data[37] <=LOCAL_IP[23:16];
					tx_arp_data[38] <=LOCAL_IP[15:8];
					tx_arp_data[39] <=LOCAL_IP[7:0];
					tx_arp_data[40] <=ARP_MAC_src[47:40];	//Target MAC 
					tx_arp_data[41] <=ARP_MAC_src[39:32];
					tx_arp_data[42] <=ARP_MAC_src[31:24];
					tx_arp_data[43] <=ARP_MAC_src[23:16];
					tx_arp_data[44] <=ARP_MAC_src[15:8];
					tx_arp_data[45] <=ARP_MAC_src[7:0];	
					tx_arp_data[46] <=ARP_IP_src[31:24];	//Target IP   
					tx_arp_data[47] <=ARP_IP_src[23:16];
					tx_arp_data[48] <=ARP_IP_src[15:8];
					tx_arp_data[49] <=ARP_IP_src[7:0];
					tx_arp_data[50] <=8'd00; 					//Padding
					tx_arp_data[51] <=8'd00;
					tx_arp_data[52] <=8'd00;
					tx_arp_data[53] <=8'd00;
					tx_arp_data[54] <=8'd00;
					tx_arp_data[55] <=8'd00;
					tx_arp_data[56] <=8'd00;
					tx_arp_data[57] <=8'd00;
					tx_arp_data[58] <=8'd00;
					tx_arp_data[59] <=8'd00;
					tx_arp_data[60] <=8'd00;
					tx_arp_data[61] <=8'd00;
					tx_arp_data[62] <=8'd00;
					tx_arp_data[63] <=8'd00;
					tx_arp_data[64] <=8'd00;
					tx_arp_data[65] <=8'd00;
					tx_arp_data[66] <=8'd00;
					tx_arp_data[67] <=8'd00;
					
					tx_alg_state <= 8'd1;
					mem_i<=0;
					mem_j<=8; //exclude preambule for CRC
				end
				
				8'd1: begin //Generate CRC
					crcen<=1'b1;	//enable crc
					crcrst<=1'b0;
					//*****************************************
					if(mem_j==67) begin
						if(mem_i==0) begin
							tx_data[3:0]<=tx_arp_data[mem_j][3:0];
							mem_i<=mem_i+1;
						end
						else if(mem_i==1) begin
							tx_data[3:0]<=tx_arp_data[mem_j][7:4];
							mem_i<=0;
							mem_j<=0;
							tx_alg_state<=8'd2;
						end
					end
					
					else begin
						if(mem_i==0) begin
							 tx_data[3:0]<=tx_arp_data[mem_j][3:0];
							 mem_i<=mem_i+1;
						end
						else if(mem_i==1) begin
							 tx_data[3:0]<=tx_arp_data[mem_j][7:4];
							 mem_i<=0;
							 mem_j<=mem_j+1;
						end
					end

				end
				
				8'd2: begin
					crcen<=1'b0;
					//CRC
					tx_arp_data[68] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31],  ~crc32[24], ~crc32[25], ~crc32[26], ~crc32[27]};
					tx_arp_data[69] <= {~crc32[20],   ~crc32[21],   ~crc32[22],   ~crc32[23],    ~crc32[16], ~crc32[17], ~crc32[18], ~crc32[19]};
					tx_arp_data[70] <= {~crc32[12],   ~crc32[13],   ~crc32[14],   ~crc32[15],    ~crc32[8],  ~crc32[9],  ~crc32[10], ~crc32[11]};
					tx_arp_data[71] <= {~crc32[4],    ~crc32[5],    ~crc32[6],    ~crc32[7],     ~crc32[0],  ~crc32[1],  ~crc32[2],  ~crc32[3]};
					tx_alg_state<=8'd3;
					mem_i<=0;
					mem_j<=0;
					
					if(!mem_t.lock[0])
						q<=4'd1;
					else if(!mem_t.lock[1])
						q<=4'd2;
					else if(!mem_t.lock[2])
						q<=4'd4;	
					else if(!mem_t.lock[3])
						q<=4'd8;
					else
						q<=4'd0;
					
				end
				
				8'd3: begin //Put packet to MEM

					//Put
					if(q>0)begin
					
						if(mem_j==71) begin
							if(mem_i==0) begin
								mem_t.data[q][mem_j][3:0] <=tx_arp_data[mem_j][3:0];
								mem_i<=mem_i+1;
							end
							else if(mem_i==1) begin
								mem_t.data[q][mem_j][7:4] <=tx_arp_data[mem_j][7:4];
								mem_t.lock[q]<=1'b1;		//set lock
								mem_t.len[q]<=16'd71;	//set len
								mem_i<=0;
								mem_j<=0;
								tx_state<=tx_IDLE;
							end
						end
						
						else begin
							if(mem_i==0) begin
								 mem_t.data[q][mem_j][3:0] <=tx_arp_data[mem_j][3:0];
								 mem_i<=mem_i+1;
							end
							else if(mem_i==1) begin
								 mem_t.data[q][mem_j][7:4] <=tx_arp_data[mem_j][7:4];
								 mem_i<=0;
								 mem_j<=mem_j+1;
							end
						end
						
					end	
				end
				
				default: tx_state<=tx_IDLE;
			endcase
		end
	endcase*/
	
	

	
	//Transmit from prepare memory
   /*case(tx_mem_state)
		4'd0:begin
		
			tx_i <= 16'd0;
			tx_j <= 16'd0;
			tx_en<=0;
		
			if(mem_t.lock[0] && mem_t.len[0] >16'd0)begin
				p<=4'd1;
				tx_mem_state<=4'd1;
			end
			
			else if(mem_t.lock[1] && mem_t.len[1] >16'd0)begin
				p<=4'd2;
				tx_mem_state<=4'd1;
			end
			
			else if(mem_t.lock[2] && mem_t.len[2] >16'd0)begin
				p<=4'd4;
				tx_mem_state<=4'd1;
			end
			
			else if(mem_t.lock[3] && mem_t.len[3] >16'd0)begin
				p<=4'd8;
				tx_mem_state<=4'd1;
			end
			
			else begin
				p<=4'd0;
				tx_mem_state<=4'd0;
			end
		end

		4'd1:begin //SEND DATA FORM MEMORY
			//*************************************************
			//send data
			tx_en<=1'b1;
			//if(tx_j==mem_t.len[p]) begin
			
			if(tx_j==71) begin
				if(i==0) begin
					txd[3:0]<=mem_t.data[p][tx_j][3:0];
					tx_i<=tx_i+1;
				end
				else if(tx_i==1) begin
					txd[3:0]<=mem_t.data[p][tx_j][7:4];
					mem_t.lock[p]<=1'b0;//unset lock
					mem_t.len[p]<=16'd0;//clear len
					tx_i<=0;
					tx_j<=0;
					tx_mem_state<=4'd2;
				end
			end
			
			else begin
				if(tx_i==0) begin
					 txd[3:0]<=mem_t.data[p][tx_j][3:0];
					 tx_i<=tx_i+1;
				end
				else if(tx_i==1) begin
					 txd[3:0]<=mem_t.data[p][tx_j][7:4];
					 tx_i<=0;
					 tx_j<=tx_j+1;
					 tx_byte_cnt<=tx_byte_cnt+1'b1;
				end
			end
		end
		
		4'd2:begin //IGP
			tx_en<=1'b0;
			txd<=4'h00;
			if(igp_cnt<4'd12)
				igp_cnt<=igp_cnt+1'b1;
			else
				tx_mem_state<=4'd0;
		end
		
		default: tx_mem_state<=4'd0;
	endcase*/

//***********************************************************************************************************************
//Prepare packet and put it in memory for transmission
/*
always@(posedge tx_clk) begin
	
	case(tx_state)
		
		tx_IDLE: begin
			tx_alg_state<=8'd0;
			tx_byte_cnt<=16'd0;
			i<=0;j<=0;
			
			if(arp_ok)
				tx_state<=tx_ARP;
		end
		
		tx_ARP: begin
			case(tx_alg_state)	
				8'd0: begin //Prepare ARP
					tx_arp_data[0]  <=8'h55;               //PREAMBULE  
					tx_arp_data[1]  <=8'h55;
					tx_arp_data[2]  <=8'h55;
					tx_arp_data[3]  <=8'h55;
					tx_arp_data[4]  <=8'h55;
					tx_arp_data[5]  <=8'h55;
					tx_arp_data[6]  <=8'h55;
					tx_arp_data[7]	 <=8'hD5; 
					
					tx_arp_data[8]  <=ARP_MAC_src[47:40];	//Dest MAC  
					tx_arp_data[9]  <=ARP_MAC_src[39:32];
					tx_arp_data[10] <=ARP_MAC_src[31:24];
					tx_arp_data[11] <=ARP_MAC_src[23:16];
					tx_arp_data[12] <=ARP_MAC_src[15:8];
					tx_arp_data[13] <=ARP_MAC_src[7:0];	
					
					tx_arp_data[14] <=LOCAL_MAC[47:40];		//Src MAC
					tx_arp_data[15] <=LOCAL_MAC[39:32];
					tx_arp_data[16] <=LOCAL_MAC[31:24];
					tx_arp_data[17] <=LOCAL_MAC[23:16];
					tx_arp_data[18] <=LOCAL_MAC[15:8];
					tx_arp_data[19] <=LOCAL_MAC[7:0];	
					
					tx_arp_data[20] <=ETH_type[15:8]; 		//TYPE 
					tx_arp_data[21] <=ETH_type[7:0];
					tx_arp_data[22] <=8'h00;					//HType
					tx_arp_data[23] <=8'h01;
					tx_arp_data[24] <=8'h08;					//IPv4
					tx_arp_data[25] <=8'h00;
					tx_arp_data[26] <=8'h06;					//Hsz
					tx_arp_data[27] <=8'h04;					//Psz
					tx_arp_data[28] <=8'h00;					//Opcode 0x0002 reply
					tx_arp_data[29] <=8'h02;
					tx_arp_data[30] <=LOCAL_MAC[47:40];		//My MAC  
					tx_arp_data[31] <=LOCAL_MAC[39:32];
					tx_arp_data[32] <=LOCAL_MAC[31:24];
					tx_arp_data[33] <=LOCAL_MAC[23:16];
					tx_arp_data[34] <=LOCAL_MAC[15:8];
					tx_arp_data[35] <=LOCAL_MAC[7:0];
					tx_arp_data[36] <=LOCAL_IP[31:24];		//My IP   
					tx_arp_data[37] <=LOCAL_IP[23:16];
					tx_arp_data[38] <=LOCAL_IP[15:8];
					tx_arp_data[39] <=LOCAL_IP[7:0];
					tx_arp_data[40] <=ARP_MAC_src[47:40];	//Target MAC 
					tx_arp_data[41] <=ARP_MAC_src[39:32];
					tx_arp_data[42] <=ARP_MAC_src[31:24];
					tx_arp_data[43] <=ARP_MAC_src[23:16];
					tx_arp_data[44] <=ARP_MAC_src[15:8];
					tx_arp_data[45] <=ARP_MAC_src[7:0];	
					tx_arp_data[46] <=ARP_IP_src[31:24];	//Target IP   
					tx_arp_data[47] <=ARP_IP_src[23:16];
					tx_arp_data[48] <=ARP_IP_src[15:8];
					tx_arp_data[49] <=ARP_IP_src[7:0];
					tx_arp_data[50] <=8'd00; 					//Padding
					tx_arp_data[51] <=8'd00;
					tx_arp_data[52] <=8'd00;
					tx_arp_data[53] <=8'd00;
					tx_arp_data[54] <=8'd00;
					tx_arp_data[55] <=8'd00;
					tx_arp_data[56] <=8'd00;
					tx_arp_data[57] <=8'd00;
					tx_arp_data[58] <=8'd00;
					tx_arp_data[59] <=8'd00;
					tx_arp_data[60] <=8'd00;
					tx_arp_data[61] <=8'd00;
					tx_arp_data[62] <=8'd00;
					tx_arp_data[63] <=8'd00;
					tx_arp_data[64] <=8'd00;
					tx_arp_data[65] <=8'd00;
					tx_arp_data[66] <=8'd00;
					tx_arp_data[67] <=8'd00;
					
					tx_alg_state <= 8'd1;
					i<=8'd0;
					j<=8'd8; //exclude preambule for CRC
				end
				
				8'd1: begin //Generate CRC
					crcen<=1'b1;	//enable crc
					crcrst<=1'b0;
					//*****************************************
					if(j==67) begin
						if(i==0) begin
							tx_data[3:0]<=tx_arp_data[j][3:0];
							i<=i+1;
						end
						else if(i==1) begin
							tx_data[3:0]<=tx_arp_data[j][7:4];
							i<=0;j<=0;
							tx_alg_state<=8'd2;
						end
					end
					
					else begin
						if(i==0) begin
							 tx_data[3:0]<=tx_arp_data[j][3:0];
							 i<=i+1;
						end
						else if(i==1) begin
							 tx_data[3:0]<=tx_arp_data[j][7:4];
							 i<=0;j<=j+1;
						end
					end

				end
				
				8'd2: begin
					crcen<=1'b0;
					//CRC
					tx_arp_data[68] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31],  ~crc32[24], ~crc32[25], ~crc32[26], ~crc32[27]};
					tx_arp_data[69] <= {~crc32[20],   ~crc32[21],   ~crc32[22],   ~crc32[23],    ~crc32[16], ~crc32[17], ~crc32[18], ~crc32[19]};
					tx_arp_data[70] <= {~crc32[12],   ~crc32[13],   ~crc32[14],   ~crc32[15],    ~crc32[8],  ~crc32[9],  ~crc32[10], ~crc32[11]};
					tx_arp_data[71] <= {~crc32[4],    ~crc32[5],    ~crc32[6],    ~crc32[7],     ~crc32[0],  ~crc32[1],  ~crc32[2],  ~crc32[3]};
					tx_alg_state<=8'd3;
					i<=0;j<=0;
					
					if(!mem_t.lock[0])
						q<=4'd1;
					else if(!mem_t.lock[1])
						q<=4'd2;
					else if(!mem_t.lock[2])
						q<=4'd4;	
					else if(!mem_t.lock[3])
						q<=4'd8;
					else
						q<=4'd0;
					
				end
				
				8'd3: begin //Put packet to MEM

					//Put
					if(q>0)begin
					
						if(j==16'd71) begin
							if(i==0) begin
								mem_t.data[q][j][3:0] <=tx_arp_data[j][3:0];
								i<=i+1;
							end
							else if(i==1) begin
								mem_t.data[q][j][7:4] <=tx_arp_data[j][7:4];
								mem_t.lock[q]<=1'b1;		//set lock
								mem_t.len[q]<=16'd71;	//set len
								i<=0;j<=0;
								tx_state<=tx_IDLE;
							end
						end
						
						else begin
							if(i==0) begin
								 mem_t.data[q][j][3:0] <=tx_arp_data[j][3:0];
								 i<=i+1;
							end
							else if(i==1) begin
								 mem_t.data[q][j][7:4] <=tx_arp_data[j][7:4];
								 i<=0;
								 j<=j+1;
							end
						end
						
					end	
				end
				
				default: tx_state<=tx_IDLE;
			endcase
		end
		
	endcase
end*/





//***********************************************************************************************************************
//SEND PACKET
/*
always@(posedge tx_clk) begin
	case(tx_state)
	
		tx_IDLE: begin
			i<=0;j<=0;
			tx_en<=1'b0;
			crcen<=1'b0;
			crcrst<=1'b1;
			txd<=4'h00;
			tx_alg_state<=8'd0;
			tx_byte_cnt<=16'd0;
			
			if(arp_ok)
				tx_state<=tx_ARP;
			else if(icmp_ok)
				tx_state<=tx_ICMP;
			else if(udp_ok)
				tx_state<=tx_UDP;
			else
				tx_state<=tx_IDLE;
		end
		
		tx_ARP:  begin
			case(tx_alg_state)	
				8'd0: begin //Prepare ARP
					tx_arp_data[0]  <=8'h55;               //PREAMBULE  
					tx_arp_data[1]  <=8'h55;
					tx_arp_data[2]  <=8'h55;
					tx_arp_data[3]  <=8'h55;
					tx_arp_data[4]  <=8'h55;
					tx_arp_data[5]  <=8'h55;
					tx_arp_data[6]  <=8'h55;
					tx_arp_data[7]	 <=8'hD5; 
					
					tx_arp_data[8]  <=ARP_MAC_src[47:40];	//Dest MAC  
					tx_arp_data[9]  <=ARP_MAC_src[39:32];
					tx_arp_data[10] <=ARP_MAC_src[31:24];
					tx_arp_data[11] <=ARP_MAC_src[23:16];
					tx_arp_data[12] <=ARP_MAC_src[15:8];
					tx_arp_data[13] <=ARP_MAC_src[7:0];	
					
					tx_arp_data[14] <=LOCAL_MAC[47:40];		//Src MAC
					tx_arp_data[15] <=LOCAL_MAC[39:32];
					tx_arp_data[16] <=LOCAL_MAC[31:24];
					tx_arp_data[17] <=LOCAL_MAC[23:16];
					tx_arp_data[18] <=LOCAL_MAC[15:8];
					tx_arp_data[19] <=LOCAL_MAC[7:0];	
					
					tx_arp_data[20] <=ETH_type[15:8]; 		//TYPE 
					tx_arp_data[21] <=ETH_type[7:0];
					tx_arp_data[22] <=8'h00;					//HType
					tx_arp_data[23] <=8'h01;
					tx_arp_data[24] <=8'h08;					//IPv4
					tx_arp_data[25] <=8'h00;
					tx_arp_data[26] <=8'h06;					//Hsz
					tx_arp_data[27] <=8'h04;					//Psz
					tx_arp_data[28] <=8'h00;					//Opcode 0x0002 reply
					tx_arp_data[29] <=8'h02;
					tx_arp_data[30] <=LOCAL_MAC[47:40];		//My MAC  
					tx_arp_data[31] <=LOCAL_MAC[39:32];
					tx_arp_data[32] <=LOCAL_MAC[31:24];
					tx_arp_data[33] <=LOCAL_MAC[23:16];
					tx_arp_data[34] <=LOCAL_MAC[15:8];
					tx_arp_data[35] <=LOCAL_MAC[7:0];
					tx_arp_data[36] <=LOCAL_IP[31:24];		//My IP   
					tx_arp_data[37] <=LOCAL_IP[23:16];
					tx_arp_data[38] <=LOCAL_IP[15:8];
					tx_arp_data[39] <=LOCAL_IP[7:0];
					tx_arp_data[40] <=ARP_MAC_src[47:40];	//Target MAC 
					tx_arp_data[41] <=ARP_MAC_src[39:32];
					tx_arp_data[42] <=ARP_MAC_src[31:24];
					tx_arp_data[43] <=ARP_MAC_src[23:16];
					tx_arp_data[44] <=ARP_MAC_src[15:8];
					tx_arp_data[45] <=ARP_MAC_src[7:0];	
					tx_arp_data[46] <=ARP_IP_src[31:24];	//Target IP   
					tx_arp_data[47] <=ARP_IP_src[23:16];
					tx_arp_data[48] <=ARP_IP_src[15:8];
					tx_arp_data[49] <=ARP_IP_src[7:0];
					tx_arp_data[50] <=8'd00; 					//Padding
					tx_arp_data[51] <=8'd00;
					tx_arp_data[52] <=8'd00;
					tx_arp_data[53] <=8'd00;
					tx_arp_data[54] <=8'd00;
					tx_arp_data[55] <=8'd00;
					tx_arp_data[56] <=8'd00;
					tx_arp_data[57] <=8'd00;
					tx_arp_data[58] <=8'd00;
					tx_arp_data[59] <=8'd00;
					tx_arp_data[60] <=8'd00;
					tx_arp_data[61] <=8'd00;
					tx_arp_data[62] <=8'd00;
					tx_arp_data[63] <=8'd00;
					tx_arp_data[64] <=8'd00;
					tx_arp_data[65] <=8'd00;
					tx_arp_data[66] <=8'd00;
					tx_arp_data[67] <=8'd00;
					
					tx_alg_state <= 8'd1;
					i<=8'd0;j<=8'd0;
				end
				
				8'd1: begin //Send ARP
					if(tx_byte_cnt<=16'd7) begin
						tx_en<=1;
						crcen<=1'b0;  	//disable crc
						crcrst<=1'b1; 	//reset crc 
					end
					else begin
						crcen<=1'b1;	//enable crc
						crcrst<=1'b0; 
					end

					//send data
					if(j==67) begin
						if(i==0) begin
							txd[3:0]<=tx_arp_data[j][3:0];
							i<=i+1;
						end
						else if(i==1) begin
							txd[3:0]<=tx_arp_data[j][7:4];
							i<=0;
							j<=0;
							tx_alg_state<=8'd2;
						end
					end
					
					else begin
						if(i==0) begin
							 txd[3:0]<=tx_arp_data[j][3:0];
							 i<=i+1;
						end
						else if(i==1) begin
							 txd[3:0]<=tx_arp_data[j][7:4];
							 i<=0;
							 j<=j+1;
							 tx_byte_cnt<=tx_byte_cnt+1'b1;
						end
					end
				end
				
				8'd2: begin //Send CRC
					crcen<=1'b0;
					if(i==0)  begin
						txd[3:0] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31]};
						i<=i+1;
					end
					else if(i==1) begin
						txd[3:0]<={~crc32[24], ~crc32[25], ~crc32[26], ~crc32[27]};
						i<=i+1;
					end
					else if(i==2) begin
						txd[3:0]<={~crc32[20], ~crc32[21], ~crc32[22], ~crc32[23]};
						i<=i+1;
					end
					else if(i==3) begin
						txd[3:0]<={~crc32[16], ~crc32[17], ~crc32[18], ~crc32[19]};
						i<=i+1;
					end
					else if(i==4) begin
						txd[3:0]<={~crc32[12], ~crc32[13], ~crc32[14], ~crc32[15]};
						i<=i+1;
					end
					else if(i==5) begin
						txd[3:0]<={~crc32[8], ~crc32[9], ~crc32[10], ~crc32[11]};
						i<=i+1;
					end
					else if(i==6) begin
						txd[3:0]<={~crc32[4], ~crc32[5], ~crc32[6], ~crc32[7]};
						i<=i+1;
					end
					else if(i==7) begin
						txd[3:0]<={~crc32[0], ~crc32[1], ~crc32[2], ~crc32[3]};
						i<=i+1;
					end
					else begin
						tx_en<=0;
						txd<=4'h00;
						tx_state<=tx_IDLE;
					end
				end
				
				default: tx_state<=tx_IDLE;
			endcase
		end
		
		tx_ICMP: begin
			case(tx_alg_state)
				8'd0: begin //Prepare ICMP
					icmp_cnt<=icmp_cnt+1'b1;
					//PREAMBULE  
					tx_icmp_data[0]  <=8'h55;					
					tx_icmp_data[1]  <=8'h55;
					tx_icmp_data[2]  <=8'h55;
					tx_icmp_data[3]  <=8'h55;
					tx_icmp_data[4]  <=8'h55;
					tx_icmp_data[5]  <=8'h55;
					tx_icmp_data[6]  <=8'h55;
					tx_icmp_data[7]  <=8'hD5; 
					//ETH
					tx_icmp_data[8]  <=ETH_MAC_src[47:40];	//Dest MAC  
					tx_icmp_data[9]  <=ETH_MAC_src[39:32];
					tx_icmp_data[10] <=ETH_MAC_src[31:24];
					tx_icmp_data[11] <=ETH_MAC_src[23:16];
					tx_icmp_data[12] <=ETH_MAC_src[15:8];
					tx_icmp_data[13] <=ETH_MAC_src[7:0];	
					tx_icmp_data[14] <=LOCAL_MAC[47:40];	//Src MAC
					tx_icmp_data[15] <=LOCAL_MAC[39:32];
					tx_icmp_data[16] <=LOCAL_MAC[31:24];
					tx_icmp_data[17] <=LOCAL_MAC[23:16];
					tx_icmp_data[18] <=LOCAL_MAC[15:8];
					tx_icmp_data[19] <=LOCAL_MAC[7:0];	
					tx_icmp_data[20] <=ETH_type[15:8];		//TYPE 
					tx_icmp_data[21] <=ETH_type[7:0];
					//IPv4
					tx_icmp_data[22] <=8'h45;					//Version number: 4; Header length: 20; 
					tx_icmp_data[23] <=8'h00;
					tx_icmp_data[24] <=IPv4_tlen[15:8];		//IPv4 total length
					tx_icmp_data[25] <=IPv4_tlen[7:0];
					tx_icmp_data[26] <=icmp_cnt[15:8];		//Package serial number
					tx_icmp_data[27] <=icmp_cnt[7:0];
					tx_icmp_data[28] <=8'h00;					//Fragment offset
					tx_icmp_data[29] <=8'h00;
					tx_icmp_data[30] <=IPv4_ttl;				//TTL 64 8'h40;
					tx_icmp_data[31] <=8'h01;					//Protocol: 01(ICMP)
					tx_icmp_data[32] <=8'h00;					//Header checksum
					tx_icmp_data[33] <=8'h00;
					tx_icmp_data[34] <=LOCAL_IP[31:24];		//Source IP   
					tx_icmp_data[35] <=LOCAL_IP[23:16];
					tx_icmp_data[36] <=LOCAL_IP[15:8];
					tx_icmp_data[37] <=LOCAL_IP[7:0];				
					tx_icmp_data[38] <=IPv4_IP_src[31:24];	//Dest IP
					tx_icmp_data[39] <=IPv4_IP_src[23:16];
					tx_icmp_data[40] <=IPv4_IP_src[15:8];
					tx_icmp_data[41] <=IPv4_IP_src[7:0];
					
					case(ICMP_tlen)
						16'd40: begin //ICMP packet for Windows OS
							//tx_state<=tx_IDLE;
							tx_icmp_data[42]  <=8'h00;						//TYPE:0 (Echo (ping) reply)
							tx_icmp_data[43]  <=8'h00;						//CODE:0
							tx_icmp_data[44]  <=8'h00;						//CHECKSUM
							tx_icmp_data[45]  <=8'h00;
							tx_icmp_data[46]  <=ICMP_layer[295:288]; 	//Id BE
							tx_icmp_data[47]  <=ICMP_layer[287:280];	//Id LE
							tx_icmp_data[48]  <=ICMP_layer[279:272];	//SN BE
							tx_icmp_data[49]  <=ICMP_layer[271:264];	//SN LE
							
							//Fill data
							for(n=50, m=0; n<82; n=n+1, m=m+1) begin
								tx_icmp_data[n] <=ICMP_layer[263-(m*8)-:8];
							end
						end
						
						16'd64: begin //ICMP packet for Linux OS
							tx_icmp_data[42]  <=8'h00;						//TYPE:0 (Echo (ping) reply)
							tx_icmp_data[43]  <=8'h00;						//CODE:0
							tx_icmp_data[44]  <=8'h00;						//CHECKSUM
							tx_icmp_data[45]  <=8'h00;
							
							//Fill data
							for(n=46, m=0; n<106; n=n+1, m=m+1) begin
								tx_icmp_data[n] <=ICMP_layer[487-(m*8)-:8];
							end
						end
						
						default: tx_state<=tx_IDLE;
					endcase
					i<=0;j<=0;
					tx_alg_state<=8'd1;
				end
				
				8'd1: begin //Generate CRC for IPv4
					if(i==0) begin											
						check_buffer <={tx_icmp_data[24],tx_icmp_data[25]} + {tx_icmp_data[22],tx_icmp_data[23]} +
											{tx_icmp_data[28],tx_icmp_data[29]} + {tx_icmp_data[26],tx_icmp_data[27]} +
											{tx_icmp_data[32],tx_icmp_data[33]} + {tx_icmp_data[30],tx_icmp_data[31]} +
											{tx_icmp_data[36],tx_icmp_data[37]} + {tx_icmp_data[34],tx_icmp_data[35]} +
											{tx_icmp_data[40],tx_icmp_data[41]} + {tx_icmp_data[38],tx_icmp_data[39]};
						i<=i+1'b1;
					end
					else if(i==1) begin
						check_buffer[15:0]<=check_buffer[31:16]+check_buffer[15:0];
						i<=i+1'b1;
					end
					else begin
						tx_icmp_data[32]<=~check_buffer[15:8];
						tx_icmp_data[33]<=~check_buffer[7:0];
						i<=0;j<=0;
						tx_alg_state<=8'd2;
					end
				end
				
				8'd2: begin //Generate CRC ICMP
					if(i==0) begin
						case(ICMP_tlen)
							16'd40: begin //for Windows OS
								check_buffer <={tx_icmp_data[44],tx_icmp_data[45]}   + {tx_icmp_data[42],tx_icmp_data[43]} +
													{tx_icmp_data[48],tx_icmp_data[49]}   + {tx_icmp_data[46],tx_icmp_data[47]} +
													{tx_icmp_data[52],tx_icmp_data[53]}   + {tx_icmp_data[50],tx_icmp_data[51]} +
													{tx_icmp_data[56],tx_icmp_data[57]}   + {tx_icmp_data[54],tx_icmp_data[55]} +
													{tx_icmp_data[60],tx_icmp_data[61]}   + {tx_icmp_data[58],tx_icmp_data[59]} +	
													{tx_icmp_data[64],tx_icmp_data[65]}   + {tx_icmp_data[62],tx_icmp_data[63]} +
													{tx_icmp_data[68],tx_icmp_data[69]}   + {tx_icmp_data[66],tx_icmp_data[67]} +
													{tx_icmp_data[72],tx_icmp_data[73]}   + {tx_icmp_data[70],tx_icmp_data[71]} +
													{tx_icmp_data[76],tx_icmp_data[77]}   + {tx_icmp_data[74],tx_icmp_data[75]} +
													{tx_icmp_data[80],tx_icmp_data[81]}   + {tx_icmp_data[78],tx_icmp_data[79]} ;
							end
							
							16'd64: begin //for Linux OS
								check_buffer <={tx_icmp_data[44],tx_icmp_data[45]}   + {tx_icmp_data[42],tx_icmp_data[43]} +
													{tx_icmp_data[48],tx_icmp_data[49]}   + {tx_icmp_data[46],tx_icmp_data[47]} +
													{tx_icmp_data[52],tx_icmp_data[53]}   + {tx_icmp_data[50],tx_icmp_data[51]} +
													{tx_icmp_data[56],tx_icmp_data[57]}   + {tx_icmp_data[54],tx_icmp_data[55]} +
													{tx_icmp_data[60],tx_icmp_data[61]}   + {tx_icmp_data[58],tx_icmp_data[59]} +	
													{tx_icmp_data[64],tx_icmp_data[65]}   + {tx_icmp_data[62],tx_icmp_data[63]} +
													{tx_icmp_data[68],tx_icmp_data[69]}   + {tx_icmp_data[66],tx_icmp_data[67]} +
													{tx_icmp_data[72],tx_icmp_data[73]}   + {tx_icmp_data[70],tx_icmp_data[71]} +
													{tx_icmp_data[76],tx_icmp_data[77]}   + {tx_icmp_data[74],tx_icmp_data[75]} +
													{tx_icmp_data[80],tx_icmp_data[81]}   + {tx_icmp_data[78],tx_icmp_data[79]} +
													{tx_icmp_data[84],tx_icmp_data[85]}   + {tx_icmp_data[82],tx_icmp_data[83]} +
													{tx_icmp_data[88],tx_icmp_data[89]}   + {tx_icmp_data[86],tx_icmp_data[87]} +
													{tx_icmp_data[92],tx_icmp_data[93]}   + {tx_icmp_data[90],tx_icmp_data[91]} +
													{tx_icmp_data[96],tx_icmp_data[97]}   + {tx_icmp_data[94],tx_icmp_data[95]} +
													{tx_icmp_data[100],tx_icmp_data[101]} + {tx_icmp_data[98],tx_icmp_data[99]} +
													{tx_icmp_data[104],tx_icmp_data[105]} + {tx_icmp_data[102],tx_icmp_data[103]};	
							end
							
							default: tx_state<=tx_IDLE;
						endcase
					
						i<=i+1'b1;
					end
					else if(i==1) begin
						check_buffer[15:0]<=check_buffer[31:16]+check_buffer[15:0];
						i<=i+1'b1;
					end
					else begin
						tx_icmp_data[44]<=~check_buffer[15:8];
						tx_icmp_data[45]<=~check_buffer[7:0];
						i<=0;j<=0;
						tx_alg_state<=8'd3;
					end
				end
				
				8'd3: begin //Send ICMP
					if(tx_byte_cnt<=16'd7) begin
						tx_en<=1;
						crcen<=1'b0;  	//disable crc
						crcrst<=1'b1; 	//reset crc 
					end
					else begin
						crcen<=1'b1;	//enable crc
						crcrst<=1'b0; 
					end
					
					//send data
					case(ICMP_tlen)
						16'd40: begin //for Windows OS
							if(j==81) begin
								if(i==0) begin
									txd[3:0]<=tx_icmp_data[j][3:0];
									i<=i+1;
								end
								else if(i==1) begin
									txd[3:0]<=tx_icmp_data[j][7:4];
									i<=0;j<=0;
									tx_alg_state<=8'd4;
								end
							end
							else begin
								if(i==0) begin
									 txd[3:0]<=tx_icmp_data[j][3:0];
									 i<=i+1;
								end
								else if(i==1) begin
									 txd[3:0]<=tx_icmp_data[j][7:4];
									 i<=0;
									 j<=j+1;
									 tx_byte_cnt<=tx_byte_cnt+1'b1;
								end
							end
						end
						
						16'd64: begin //for Linux OS
							if(j==105) begin
								if(i==0) begin
									txd[3:0]<=tx_icmp_data[j][3:0];
									i<=i+1;
								end
								else if(i==1) begin
									txd[3:0]<=tx_icmp_data[j][7:4];
									i<=0;j<=0;
									tx_alg_state<=8'd4;
								end
							end
							else begin
								if(i==0) begin
									 txd[3:0]<=tx_icmp_data[j][3:0];
									 i<=i+1;
								end
								else if(i==1) begin
									 txd[3:0]<=tx_icmp_data[j][7:4];
									 i<=0;
									 j<=j+1;
									 tx_byte_cnt<=tx_byte_cnt+1'b1;
								end
							end
						end
					endcase
				end
				
				8'd4: begin //Send CRC
					crcen<=1'b0;
					if(i==0)  begin
						txd[3:0] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31]};
						i<=i+1;
					end
					else if(i==1) begin
						txd[3:0]<={~crc32[24], ~crc32[25], ~crc32[26], ~crc32[27]};
						i<=i+1;
					end
					else if(i==2) begin
						txd[3:0]<={~crc32[20], ~crc32[21], ~crc32[22], ~crc32[23]};
						i<=i+1;
					end
					else if(i==3) begin
						txd[3:0]<={~crc32[16], ~crc32[17], ~crc32[18], ~crc32[19]};
						i<=i+1;
					end
					else if(i==4) begin
						txd[3:0]<={~crc32[12], ~crc32[13], ~crc32[14], ~crc32[15]};
						i<=i+1;
					end
					else if(i==5) begin
						txd[3:0]<={~crc32[8], ~crc32[9], ~crc32[10], ~crc32[11]};
						i<=i+1;
					end
					else if(i==6) begin
						txd[3:0]<={~crc32[4], ~crc32[5], ~crc32[6], ~crc32[7]};
						i<=i+1;
					end
					else if(i==7) begin
						txd[3:0]<={~crc32[0], ~crc32[1], ~crc32[2], ~crc32[3]};
						i<=i+1;
					end
					else  begin
						tx_en<=0;
						txd<=4'h00;
						tx_state<=tx_IDLE;
					end
				end
				
				default: tx_state<=tx_IDLE;
			endcase	
		end
		
		tx_UDP:  begin
			tx_state<=tx_IDLE;
		end

		default: tx_state<=tx_IDLE; 
	endcase		
end*/




				/*4'd1: begin //Prepare FCS
					if(pre_j<=7) begin
						crcen<=0;  	//disable crc
						crcrst<=1; 	//reset crc 
					end
					else begin
						crcen<=1;	//enable crc
						crcrst<=0; 
					end
					//*****************************************
					if(pre_j==49) begin
						tx_data[7:0] <= tx_arp_data[pre_j][7:0];
						pre_state<=4'd2;
					end
					else begin
						tx_data[7:0] <= tx_arp_data[pre_j][7:0];
						pre_j<=pre_j+1'b1;
					end
				end
				
				4'd2: begin //Disable FCS module
					pre_i<=0;
					pre_j<=0;
					crcen<=1'b0;		
					pre_state<=4'd3;
				end
				
				4'd3: begin //Attach Frame check sequence
					tx_arp_data[50] <= {~fcs[24], ~fcs[25], ~fcs[26], ~fcs[27],   ~fcs[28], ~fcs[29], ~fcs[30], ~fcs[31]};
					tx_arp_data[51] <= {~fcs[16], ~fcs[17], ~fcs[18], ~fcs[19],   ~fcs[20], ~fcs[21], ~fcs[22], ~fcs[23]};
					tx_arp_data[52] <= {~fcs[8],  ~fcs[9],  ~fcs[10], ~fcs[11],   ~fcs[12], ~fcs[13], ~fcs[14], ~fcs[15]};
					tx_arp_data[53] <= {~fcs[0],  ~fcs[1],  ~fcs[2],  ~fcs[3],    ~fcs[4],  ~fcs[5],  ~fcs[6],  ~fcs[7] };

					
					
					if(!fifo.lock[0])
						wr<=4'd0;
					else if(!fifo.lock[1]) 
						wr<=4'd1;
					else if(!fifo.lock[2]) 
						wr<=4'd2;
					else if(!fifo.lock[3]) 
						wr<=4'd3;
					else
						prepare_pkt<=PRE_IDLE;
						
					pre_i<=0;
					pre_j<=0;
					pre_state<=4'd4;			
				end*/





		/*rx_55: begin
			if(byte_rxdv && !byte_sig) begin				  
				if(byte_data!=8'h55) begin 
					rx_state<=rx_IDLE;
					byte_counter<=3'd0; 
				end
				else begin 
					if(byte_counter<3'd5)
						byte_counter<=byte_counter+1'b1;
					else begin
						byte_counter<=3'd0;
						rx_state<=rx_D5;
					end
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
		
		rx_D5: begin
			if(byte_rxdv && !byte_sig) begin
				if(datain==8'hd5)
					rx_state<=rx_MAC;
				else
					rx_state<=rx_IDLE;
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end*/
		
				
		/*rx_MAC: begin
			if(byte_rxdv && !byte_sig) begin
				if(state_counter<16'd11) begin
					mymac<={mymac[87:0],datain};
					state_counter<=state_counter+1'b1;
				end
				else begin	
					eth_dest<=mymac[87:40];
					eth_src<={mymac[39:0],datain};
					state_counter<=16'd0;
					rx_state<=rx_ETH_type;	
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
		
		rx_ETH_type: begin
			if(byte_rxdv && !byte_sig) begin
				if(state_counter<5'd1) begin
					ETH_type<={ETH_type[7:0],datain};
					state_counter<=state_counter+1'b1;
				end
				else begin
					ETH_type<={ETH_type[7:0],datain};
					rx_state<=rx_ARP_or_IPv4;
					state_counter<=16'd0;
				end
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end*/
		
				/*rx_ARP_or_IPv4: begin
			if(byte_rxdv && !byte_sig) begin
				case (eth_type)
				
					16'h0800: begin //Receive IPv4
						if(state_counter<16'd19) begin
							IPv4_layer		<={IPv4_layer[151:0],datain[7:0]};
							state_counter	<=state_counter+1'b1;
						end
						else begin
							IPv4_layer	<={IPv4_layer[151:0],datain[7:0]};
							IPv4_tlen	<=IPv4_layer[135:120];
							IPv4_type	<=IPv4_layer[79:72];
							IPv4_IP_src	<=IPv4_layer[55:24];
							IPv4_IP_dst	<={IPv4_layer[23:0],datain[7:0]};

							case (IPv4_layer[79:72])
								8'd01: begin ICMP_tlen	<=IPv4_layer[135:120] - 16'd20;	end	//ICMP
								8'd17: begin UDP_tlen	<=IPv4_layer[135:120] - 16'd20; 	end	//UDP
								default: rx_state<=rx_IDLE;
							endcase
							
							rx_state		<=rx_ICMP_or_UDP;
							state_counter	<=16'd0;
						end
					end
					
					16'h0806: begin //Receive ARP
						if(state_counter<16'd27) begin
							ARP_layer		<={ARP_layer[215:0],datain};
							state_counter	<=state_counter+1'b1;
						end
						else begin
							ARP_MAC_src		<=ARP_layer[151:104];
							ARP_IP_src		<=ARP_layer[103:72];	
							ARP_IP_dst		<={ARP_layer[23:0],datain};	
							state_counter	<=16'd0;
							
							if(LOCAL_IP == {ARP_layer[23:0],datain}) begin
								arp_ok<=1'b1; //Receive ARP Request
							end
							rx_state<=rx_IDLE;
						end
					end
					
					default:  begin 
						rx_state<=rx_IDLE;
					end
					
				endcase	
			end
			else if(!byte_rxdv) rx_state<=rx_IDLE;
		end
					*/
	/*tx_SEND_ARP: begin
			case(tx_alg_state)
				alg_preambule: begin
					tx_en<=1;
					crcen<=1'b0;
					crcrst<=1'b1; //reset crc 
				
					if(j==7) begin
						if(i==0) begin
							txd[3:0]<=preambule[j][3:0];
							i<=i+1;
						end
						else if(i==1) begin
							txd[3:0]<=preambule[j][7:4];
							i<=0;
							j<=0;
							tx_alg_state<=alg_header;
						end
					end
					else begin
						if(i==0) begin
							 txd[3:0]<=preambule[j][3:0];
							 i<=i+1;
						end
						else if(i==1) begin
							 txd[3:0]<=preambule[j][7:4];
							 i<=0;
							 j<=j+1;
						end
					end
				end
				
				alg_header:    begin
					crcen<=1'b1;
					crcrst<=1'b0; 
					
					if(j==59) begin
						if(i==0) begin
							txd[3:0]<=arp_reply[j][3:0];
							i<=i+1;
						end
						else if(i==1) begin
							txd[3:0]<=arp_reply[j][7:4];
							i<=0;
							j<=0;
							tx_alg_state<=alg_crc;
						end
					end
					else begin
						if(i==0) begin
							 txd[3:0]<=arp_reply[j][3:0];
							 i<=i+1;
						end
						else if(i==1) begin
							 txd[3:0]<=arp_reply[j][7:4];
							 i<=0;
							 j<=j+1;
						end
					end
				end
				
				alg_crc:       begin
					crcen<=1'b0;

					if(i==0)  begin
						txd[3:0] <= {~crcnext[28], ~crcnext[29], ~crcnext[30], ~crcnext[31]};
						i<=i+1;
					end
					else if(i==1) begin
						txd[3:0]<={~crc32[24], ~crc32[25], ~crc32[26], ~crc32[27]};
						i<=i+1;
					end
					else if(i==2) begin
						txd[3:0]<={~crc32[20], ~crc32[21], ~crc32[22], ~crc32[23]};
						i<=i+1;
					end
					else if(i==3) begin
						txd[3:0]<={~crc32[16], ~crc32[17], ~crc32[18], ~crc32[19]};
						i<=i+1;
					end
					else if(i==4) begin
						txd[3:0]<={~crc32[12], ~crc32[13], ~crc32[14], ~crc32[15]};
						i<=i+1;
					end
					else if(i==5) begin
						txd[3:0]<={~crc32[8], ~crc32[9], ~crc32[10], ~crc32[11]};
						i<=i+1;
					end
					else if(i==6) begin
						txd[3:0]<={~crc32[4], ~crc32[5], ~crc32[6], ~crc32[7]};
						i<=i+1;
					end
					else if(i==7) begin
						txd[3:0]<={~crc32[0], ~crc32[1], ~crc32[2], ~crc32[3]};
						i<=i+1;
					end
					else  begin
						tx_en<=0;
						tx_state<=tx_IDLE;
					end
				end
			endcase
		end*/				
		

							/*
							tx_icmp_data[46]  <=ICMP_layer[479:472]; 	//Id BE
							tx_icmp_data[47]  <=ICMP_layer[471:464];	//Id LE
							tx_icmp_data[48]  <=ICMP_layer[463:456];	//SN BE
							tx_icmp_data[49]  <=ICMP_layer[455:448];	//SN LE
							tx_icmp_data[50]  <=ICMP_layer[447:440];	//TIMESTAMP:8 bytes
							tx_icmp_data[51]  <=ICMP_layer[439:432];
							tx_icmp_data[52]  <=ICMP_layer[431:424];
							tx_icmp_data[53]  <=ICMP_layer[423:416];
							tx_icmp_data[54]  <=ICMP_layer[415:408];
							tx_icmp_data[55]  <=ICMP_layer[407:400];
							tx_icmp_data[56]  <=ICMP_layer[399:392];
							tx_icmp_data[57]  <=ICMP_layer[391:384];
							tx_icmp_data[58]  <=ICMP_layer[383:376];	//DATA:48 bytes
							tx_icmp_data[59]  <=ICMP_layer[375:368];
							tx_icmp_data[60]  <=ICMP_layer[367:360];
							tx_icmp_data[61]  <=ICMP_layer[359:352];
							tx_icmp_data[62]  <=ICMP_layer[351:344];
							tx_icmp_data[63]  <=ICMP_layer[343:336];
							tx_icmp_data[64]  <=ICMP_layer[335:328];
							tx_icmp_data[65]  <=ICMP_layer[327:320];
							tx_icmp_data[66]  <=ICMP_layer[319:312];
							tx_icmp_data[67]  <=ICMP_layer[311:304];
							tx_icmp_data[68]  <=ICMP_layer[303:296];
							tx_icmp_data[69]  <=ICMP_layer[295:288];
							tx_icmp_data[70]  <=ICMP_layer[287:280];
							tx_icmp_data[71]  <=ICMP_layer[279:272];
							tx_icmp_data[72]  <=ICMP_layer[271:264];
							tx_icmp_data[73]  <=ICMP_layer[263:256];
							tx_icmp_data[74]  <=ICMP_layer[255:248];
							tx_icmp_data[75]  <=ICMP_layer[247:240];
							tx_icmp_data[76]  <=ICMP_layer[239:232];
							tx_icmp_data[77]  <=ICMP_layer[231:224];
							tx_icmp_data[78]  <=ICMP_layer[223:216];
							tx_icmp_data[79]  <=ICMP_layer[215:208];
							tx_icmp_data[80]  <=ICMP_layer[207:200];
							tx_icmp_data[81]  <=ICMP_layer[199:192];
							tx_icmp_data[82]  <=ICMP_layer[191:184];
							tx_icmp_data[83]  <=ICMP_layer[183:176];
							tx_icmp_data[84]  <=ICMP_layer[175:168];
							tx_icmp_data[85]  <=ICMP_layer[167:160];
							tx_icmp_data[86]  <=ICMP_layer[159:152];
							tx_icmp_data[87]  <=ICMP_layer[151:144];
							tx_icmp_data[88]  <=ICMP_layer[143:136];
							tx_icmp_data[89]  <=ICMP_layer[135:128];
							tx_icmp_data[90]  <=ICMP_layer[127:120];
							tx_icmp_data[91]  <=ICMP_layer[119:112];
							tx_icmp_data[92]  <=ICMP_layer[111:104];
							tx_icmp_data[93]  <=ICMP_layer[103:96];
							tx_icmp_data[94]  <=ICMP_layer[95:88];
							tx_icmp_data[95]  <=ICMP_layer[87:80];
							tx_icmp_data[96]  <=ICMP_layer[79:72];
							tx_icmp_data[97]  <=ICMP_layer[71:64];
							tx_icmp_data[98]  <=ICMP_layer[63:56];
							tx_icmp_data[99]  <=ICMP_layer[55:48];
							tx_icmp_data[100] <=ICMP_layer[47:40];
							tx_icmp_data[101] <=ICMP_layer[39:32];
							tx_icmp_data[102] <=ICMP_layer[31:24];
							tx_icmp_data[103] <=ICMP_layer[23:16];
							tx_icmp_data[104] <=ICMP_layer[15:8];
							tx_icmp_data[105] <=ICMP_layer[7:0];*/		