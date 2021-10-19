module double_mac(
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

	//output			arp,
	//output			icmp,
	//output			udp,
	
	output 			eth1_arp_dbg,
	output 			eth1_udp_dbg,
	output 			eth1_icmp_dbg,

	output 			eth1_to_eth2_arp_dbg,
	output 			eth1_to_eth2_icmp_dbg,

	output 			eth2_to_eth1_arp_dbg,
	output 			eth2_to_eth1_icmp_dbg,
	
	output [7:0]	eth1_rx_state_dbg,


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
	output [3:0]	alg_pre_eth1_icmp,
	
	output 			fifo_eth2_arp_empty,
	output 			fifo_eth2_arp_full,
	
	output 			fifo_eth2_icmp_empty,
	output 			fifo_eth2_icmp_full,
	
	output [3:0]	eth1_to_eth2_pre_arp_dbg,
	output [3:0]	eth1_to_eth2_pre_icmp_dbg,
	output [7:0]	eth2_tx_state_dbg,
	
	
	
	//for test
	output [7:0]   eth1_rx_cnt_dbg,
	output [7:0]   eth1_tx_cnt_dbg,
	
	output [7:0]   eth2_rx_cnt_dbg,
	output [7:0]   eth2_tx_cnt_dbg,
	
	output eth2_tx_arp_dbg,
	output eth2_tx_icmp_dbg,
	
	
	
	
	output event_eth1_arp_rd_s,
	output event_eth1_arp_wr_s,
	output event_eth1_arp_ok_s,
	
	output event_eth1_icmp_rd_s,
	output event_eth1_icmp_wr_s,
	output event_eth1_icmp_ok_s,
	
	output event_eth2_arp_rd_s,
	output event_eth2_arp_wr_s,
	output event_eth2_arp_ok_s,
	
	output event_eth2_icmp_rd_s,
	output event_eth2_icmp_wr_s,
	output event_eth2_icmp_ok_s,
	
	output event_eth2_udp_rd_s,
	output event_eth2_udp_wr_s,
	output event_eth2_udp_ok_s,
	
	
	
	
	output [15:0]	fifo_temp_eth2_udp_cnt_s,
	output 			fifo_temp_eth2_udp_wr_s,
	output [7:0] 	fifo_temp_eth2_udp_din_s,
	output 			fifo_temp_eth2_udp_full_s,
	
	output 			fifo_temp_eth2_udp_rd_s,
	output 			fifo_temp_eth2_udp_empty_s,
	output [7:0] 	fifo_temp_eth2_udp_dout_s,
	
	output [15:0] 	fifo_temp_eth2_udp_tail_s,
	output [15:0] 	fifo_temp_eth2_udp_head_s,
	
	output [11:0]  ram_eth2_udp_wr_addr_s,
	output [11:0]  ram_eth2_udp_rd_addr_s,
	
					
	output [47:0]  eth1_ARP_MAC_src_s,
	output [31:0]  eth1_ARP_IP_src_s,
	output [31:0]  eth1_ARP_IP_dst_s
	
);


reg eth1_byte_sig_t, eth1_byte_sig;
reg eth1_byte_rxdv_t, eth1_byte_rxdv;

reg [15:0]  eth1_rx_i, eth1_rx_j;
reg [15:0]  eth1_rx_m, eth1_rx_n;

//reg		  	eth1_arp_ok;
//reg			eth1_udp_ok;
//reg		  	eth1_icmp_ok;

//reg		  	eth1_to_eth2_arp_ok;
//reg		  	eth1_to_eth2_icmp_ok;

reg [15:0]  eth1_icmp_cnt;
reg [31:0]  eth1_chk_buf;

reg [7:0]  	eth1_rx_state;
reg [15:0] 	eth1_rx_byte_cnt;
reg [7:0]  	eth1_rx_alg_state;

reg [7:0]  	eth1_tx_state;  
reg [15:0] 	eth1_tx_byte_cnt;
reg [7:0]  	eth1_tx_alg_state;

reg [7:0]  	eth1_tx_arp_data  [71:0]; 
reg [7:0]  	eth1_tx_icmp_data [109:0]; //8byte Preambule + 14byte Eth + 20byte IPv4 + 64byte ICMP (64 for Linux, 40 for Windows)
reg [7:0]   eth1_tx_adc_data  [55:0];

reg [7:0] 	eth2_to_eth1_arp	[71:0]; 
reg [7:0] 	eth2_to_eth1_icmp [109:0]; 
reg [7:0] 	eth2_to_eth1_udp  [55:0];// 8PRE + 14Eth + 20IP + 8UDP + 1408Data + 4 FCS = 1462 bytes

reg [111:0] eth1_layer;
reg [47:0]  eth1_MAC_dst;
reg [47:0]  eth1_MAC_src;
reg [15:0]  eth1_type;

reg [159:0] eth1_IPv4_layer;
reg [7:0]   eth1_IPv4_vh;
reg [7:0]   eth1_IPv4_dscp;
reg [15:0]  eth1_IPv4_tlen;
reg [15:0]  eth1_IPv4_id;
reg [15:0]  eth1_IPv4_flags;
reg [7:0]   eth1_IPv4_ttl;
reg [7:0]	eth1_IPv4_type;
reg [15:0]  eth1_IPv4_crc;
reg [31:0]	eth1_IPv4_IP_src;
reg [31:0]	eth1_IPv4_IP_dst;						

reg [223:0] eth1_ARP_layer;
reg [47:0]  eth1_ARP_MAC_src;
reg [31:0]  eth1_ARP_IP_src;
reg [31:0]  eth1_ARP_IP_dst;
reg [31:0]  eth1_ARP_FCS;

reg [511:0] eth1_ICMP_layer;
reg [15:0]  eth1_ICMP_tlen;
reg [31:0]  eth1_ICMP_FCS;

reg [63:0]  eth1_UDP_layer;	//8 bytes
reg [79:0]  eth1_UDP_data;		//10 bytes
reg [31:0]  eth1_UDP_fcs;		//4 bytes
reg [15:0]  eth1_UDP_tlen;

reg [15:0]  eth1_UDP_SP;
reg [15:0]	eth1_UDP_DP;
reg [15:0]	eth1_UDP_data_len;

reg [7:0]   eth1_rx_cnt;
reg [7:0]   eth1_tx_cnt;

reg [7:0]   eth2_rx_cnt;
reg [7:0]   eth2_tx_cnt;

assign eth1_ARP_MAC_src_s 	= eth1_ARP_MAC_src;
assign eth1_ARP_IP_src_s 	= eth1_ARP_IP_src;
assign eth1_ARP_IP_dst_s 	= eth1_ARP_IP_dst;



//Events
reg 			event_eth1_arp_rd;
reg 			event_eth1_arp_wr;
wire			event_eth1_arp_ok;

reg 			event_eth1_icmp_rd;
reg 			event_eth1_icmp_wr;
wire			event_eth1_icmp_ok;

reg 			event_eth1_to_eth2_arp_rd;
reg 			event_eth1_to_eth2_arp_wr;
reg 			event_eth1_to_eth2_arp_ok;

reg 			event_eth1_to_eth2_icmp_rd;
reg 			event_eth1_to_eth2_icmp_wr;
reg 			event_eth1_to_eth2_icmp_ok;

reg 			event_eth2_to_eth1_arp_rd;
reg 			event_eth2_to_eth1_arp_wr;
reg 			event_eth2_to_eth1_arp_ok;

reg 			event_eth2_to_eth1_icmp_rd;
reg 			event_eth2_to_eth1_icmp_wr;
reg 			event_eth2_to_eth1_icmp_ok;

reg 			event_eth2_to_eth1_udp_rd;
reg 			event_eth2_to_eth1_udp_wr;
reg 			event_eth2_to_eth1_udp_ok;


parameter	rx_IDLE				=8'd0,
				rx_PREAMBULE		=8'd1,
            rx_ETH_layer		=8'd2,
				rx_IPv4_layer  	=8'd3,
				rx_ARP_layer		=8'd4,
				rx_ARP_TRL			=8'd5,
				rx_ARP_FCS			=8'd6,
				rx_ICMP_layer		=8'd7,
				rx_ICMP_FCS			=8'd8,
				rx_UDP_layer		=8'd9,
				rx_UDP_Data 		=8'd10,
				rx_UDP_FCS			=8'd11,
				rx_DELAY				=8'd12;
				
				//rx_ETH_TRL			=8'd11,
				//rx_ETH_FCS			=8'd12;
				
parameter 	tx_IDLE				=8'd0,
				tx_ADC		 		=8'd1,	//Data from ADC
				tx_ARP		 		=8'd2,	//Answer for request packet from Eth1
				tx_UDP		 		=8'd3,
				tx_ICMP		 		=8'd4,
				tx_FCS	      	=8'd5,
				tx_IGP		   	=8'd6,
				tx_ARP_from_eth2	=8'd7,
				tx_ICMP_from_eth2 =8'd8,
				tx_UDP_from_eth2	=8'd9,
				tx_ADC2				=8'd10;
			
initial begin
	eth1_rx_state<=rx_IDLE;
	eth1_tx_state<=tx_IDLE;
	
	eth1_icmp_cnt<=16'd0;
	
	eth1_rx_byte_cnt<=16'd0;
	eth1_rx_alg_state<=8'd0;
	
	eth1_tx_byte_cnt<=16'd0;
	eth1_tx_alg_state<=8'd0;
	
	//eth1_to_eth2_arp_ok<=1'b0;
	//eth1_to_eth2_icmp_ok<=1'b0;
	
	eth1_rx_cnt<=8'd0;
	eth1_tx_cnt<=8'd0;
	
	eth2_rx_cnt<=8'd0;
	eth2_tx_cnt<=8'd0;
	
	event_eth1_arp_rd		<=1'b0;
	event_eth1_arp_wr		<=1'b0;
	
	event_eth1_icmp_rd	<=1'b0;
	event_eth1_icmp_wr	<=1'b0;
	
	event_eth1_to_eth2_arp_rd	<=1'b0;
	event_eth1_to_eth2_arp_wr	<=1'b0;
	
	event_eth1_to_eth2_icmp_rd	<=1'b0;
	event_eth1_to_eth2_icmp_wr	<=1'b0;
	
	event_eth2_to_eth1_arp_rd	<=1'b0;
	event_eth2_to_eth1_arp_wr	<=1'b0;	
	
	event_eth2_to_eth1_icmp_rd	<=1'b0;
	event_eth2_to_eth1_icmp_wr	<=1'b0;	

	event_eth2_to_eth1_udp_rd	<=1'b0;
	event_eth2_to_eth1_udp_wr	<=1'b0;	
end


assign eth1_rx_state_dbg = eth1_rx_state;
assign eth2_tx_state_dbg = eth2_tx_state;

assign eth1_arp_dbg				= event_eth1_arp_ok;
//assign eth1_udp_dbg				= event_eth1_udp_ok;
assign eth1_icmp_dbg				= event_eth1_icmp_ok;

assign eth1_to_eth2_arp_dbg	= event_eth1_to_eth2_arp_ok;
assign eth1_to_eth2_icmp_dbg 	= event_eth1_to_eth2_icmp_ok;

assign eth2_to_eth1_arp_dbg 	= eth2_arp_ok;
assign eth2_to_eth1_icmp_dbg 	= eth2_icmp_ok;

assign fifo_eth2_arp_empty = fifo_eth1_to_eth2_arp_empty;
assign fifo_eth2_arp_full  = fifo_eth1_to_eth2_arp_full;

assign fifo_eth2_icmp_empty = fifo_eth1_to_eth2_icmp_empty;
assign fifo_eth2_icmp_full  = fifo_eth1_to_eth2_icmp_full;

assign eth1_to_eth2_pre_arp_dbg  = eth1_to_eth2_pre_arp;
assign eth1_to_eth2_pre_icmp_dbg = eth1_to_eth2_pre_icmp;


assign eth1_rx_cnt_dbg = eth1_rx_cnt;
assign eth1_tx_cnt_dbg = eth1_tx_cnt;
	
assign eth2_rx_cnt_dbg = eth2_rx_cnt;
assign eth2_tx_cnt_dbg = eth2_tx_cnt;

assign eth2_tx_arp_dbg  = eth2_tx_arp_s;
assign eth2_tx_icmp_dbg = eth2_tx_icmp_s;

assign event_eth2_udp_rd_s = event_eth2_to_eth1_udp_rd;
assign event_eth2_udp_wr_s = event_eth2_to_eth1_udp_wr;
assign event_eth2_udp_ok_s = event_eth2_to_eth1_udp_ok;





assign event_eth1_arp_rd_s = event_eth1_arp_rd;
assign event_eth1_arp_wr_s = event_eth1_arp_wr;
assign event_eth1_arp_ok_s = event_eth1_arp_ok;

assign event_eth1_icmp_rd_s = event_eth1_icmp_rd;
assign event_eth1_icmp_wr_s = event_eth1_icmp_wr;
assign event_eth1_icmp_ok_s = event_eth1_icmp_ok;

assign event_eth2_arp_rd_s = event_eth1_to_eth2_arp_rd;
assign event_eth2_arp_wr_s = event_eth1_to_eth2_arp_wr;
assign event_eth2_arp_ok_s = event_eth1_to_eth2_arp_ok;

assign event_eth2_icmp_rd_s = event_eth1_to_eth2_icmp_rd;
assign event_eth2_icmp_wr_s = event_eth1_to_eth2_icmp_wr;
assign event_eth2_icmp_ok_s = event_eth1_to_eth2_icmp_ok;



//CRC32 Module
wire	[31:0] eth1_crcnext;
wire	[31:0] eth1_fcs;
reg	eth1_crcrst;
reg	eth1_crcen;

//input 4-bit data
crc eth1_crc32(
	.clk(eth1_tx_clk),
	.reset(eth1_crcrst),
	.enable(eth1_crcen),
	.data(eth1_txd),
	.crc(eth1_fcs),
	.crc_next(eth1_crcnext)
);




//EVENTS
events event_eth1_arp(
	.wr_clk(eth1_rx_clk),
	.rd_clk(eth1_tx_clk),
	.rst(1'b0),
	.rd(event_eth1_arp_rd),
	.wr(event_eth1_arp_wr),
	.out(event_eth1_arp_ok)
);

events event_eth1_icmp(
	.wr_clk(eth1_rx_clk),
	.rd_clk(eth1_tx_clk),
	.rst(1'b0),
	.rd(event_eth1_icmp_rd),
	.wr(event_eth1_icmp_wr),
	.out(event_eth1_icmp_ok)
);

events event_eth1_to_eth2_arp(
	.wr_clk(eth1_rx_clk),
	.rd_clk(eth2_tx_clk),
	.rst(1'b0),
	.rd(event_eth1_to_eth2_arp_rd),
	.wr(event_eth1_to_eth2_arp_wr),
	.out(event_eth1_to_eth2_arp_ok)
);

events event_eth1_to_eth2_icmp(
	.wr_clk(eth1_rx_clk),
	.rd_clk(eth2_tx_clk),
	.rst(1'b0),
	.rd(event_eth1_to_eth2_icmp_rd),
	.wr(event_eth1_to_eth2_icmp_wr),
	.out(event_eth1_to_eth2_icmp_ok)
);

events event_eth2_to_eth1_arp(
	.wr_clk(eth2_rx_clk),
	.rd_clk(eth1_tx_clk),
	.rst(1'b0),
	.rd(event_eth2_to_eth1_arp_rd),
	.wr(event_eth2_to_eth1_arp_wr),
	.out(event_eth2_to_eth1_arp_ok)
);

events event_eth2_to_eth1_udp(
	.wr_clk(eth2_rx_clk),
	.rd_clk(eth1_tx_clk),
	.rst(1'b0),
	.rd(event_eth2_to_eth1_udp_rd),
	.wr(event_eth2_to_eth1_udp_wr),
	.out(event_eth2_to_eth1_udp_ok)
);

events event_eth2_to_eth1_icmp(
	.wr_clk(eth2_rx_clk),
	.rd_clk(eth1_tx_clk),
	.rst(1'b0),
	.rd(event_eth2_to_eth1_icmp_rd),
	.wr(event_eth2_to_eth1_icmp_wr),
	.out(event_eth2_to_eth1_icmp_ok)
);


//4bit->byte
reg [7:0] eth1_mybyte;
reg eth1_sig;
always @(posedge eth1_rx_clk) begin
	if(eth1_rxdv)	begin
		eth1_mybyte	<={eth1_rxd,eth1_mybyte[7:4]};
		eth1_sig		<=~eth1_sig;
	end
	else begin
		eth1_mybyte	<=eth1_mybyte;
		eth1_sig		<=1'b0;
	end
end

//4bit->byte
reg [7:0] eth1_datain;
always @(posedge eth1_rx_clk) begin
	if(eth1_sig&&eth1_rxdv) begin
		eth1_datain	<={eth1_rxd,eth1_mybyte[7:4]};
	end
	else if(!eth1_rxdv)
		eth1_datain	<=8'd0;
end

//generate byte rxdv signal
always @(posedge eth1_rx_clk) begin
	eth1_byte_sig_t	<=eth1_sig;
	eth1_byte_sig		<=eth1_byte_sig_t;
	eth1_byte_rxdv_t	<=eth1_rxdv;
	eth1_byte_rxdv	   <=eth1_byte_rxdv_t;	
end

//***********************************************************************************************************************//
//*ETHERNET1: RECEIVE PACKET'S																														*//
//***********************************************************************************************************************//
always@(posedge eth1_rx_clk) begin
	
	//eth1_rx_cnt<=eth1_rx_cnt+1'b1;
	
	case(eth1_rx_state)
		rx_IDLE:       begin
			
			//eth1_arp_ok		<=1'b0;
			//eth1_udp_ok		<=1'b0;
			//eth1_icmp_ok		<=1'b0;
			
			//eth1_to_eth2_arp_ok	<=1'b0;
			//eth1_to_eth2_icmp_ok	<=1'b0;
			
			event_eth1_arp_wr	<=1'b0;
			event_eth1_icmp_wr<=1'b0;
			
			event_eth1_to_eth2_arp_wr	<=1'b0;
			event_eth1_to_eth2_icmp_wr	<=1'b0;
			
			
			eth1_rx_alg_state	<=8'd0;
			eth1_rx_byte_cnt	<=16'd0;
			
			if(eth1_byte_rxdv && !eth1_byte_sig)  begin
				if(eth1_datain==8'h55)
					eth1_rx_state<=rx_PREAMBULE; 
			end
		end
		
		rx_PREAMBULE:  begin
			if(eth1_byte_rxdv && !eth1_byte_sig) begin				  
				case (eth1_rx_alg_state)
					8'd0: begin
						if(eth1_datain!=8'h55) begin 
							eth1_rx_state<=rx_IDLE;
						end
						else begin 
							if(eth1_rx_byte_cnt<16'd5)
								eth1_rx_byte_cnt<=eth1_rx_byte_cnt+1'b1;
							else begin
								eth1_rx_byte_cnt<=16'd0;
								eth1_rx_alg_state<=8'd1;
							end
						end
					end
					
					8'd1: begin
						if(eth1_datain==8'hd5) begin
							eth1_rx_state<=rx_ETH_layer;
							eth1_rx_alg_state<=8'd0;
						end
						else
							eth1_rx_state<=rx_IDLE;
					end
				endcase
			end
			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;
		end
		
		rx_ETH_layer:  begin
			if(eth1_byte_rxdv && !eth1_byte_sig) begin
				if(eth1_rx_byte_cnt<16'd13) begin
					eth1_layer	<={eth1_layer[103:0],eth1_datain};
					eth1_rx_byte_cnt	<=eth1_rx_byte_cnt+1'b1;
				end
				else begin	
					eth1_layer		<={eth1_layer[103:0],eth1_datain};
					eth1_MAC_dst	<=eth1_layer[103:56];
					eth1_MAC_src	<=eth1_layer[55:8];
					eth1_type		<={eth1_layer[7:0],eth1_datain};
					
					case ({eth1_layer[7:0],eth1_datain}) 						//Ethernet Type Protocol
						16'h0800: begin eth1_rx_state<=rx_IPv4_layer; end 	//Receive IPv4
						16'h0806: begin eth1_rx_state<=rx_ARP_layer;  end	//Receive ARP
						default:  begin eth1_rx_state<=rx_IDLE; 		 end
					endcase
					eth1_rx_byte_cnt <=16'd0;
				end
			end
			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;
		end
		
		rx_IPv4_layer: begin
			if(eth1_byte_rxdv && !eth1_byte_sig) begin
				if(eth1_rx_byte_cnt<16'd19) begin
					eth1_IPv4_layer	<={eth1_IPv4_layer[151:0],eth1_datain[7:0]};
					eth1_rx_byte_cnt 	<=eth1_rx_byte_cnt+1'b1;
				end
				else begin
					eth1_IPv4_layer	<={eth1_IPv4_layer[151:0],eth1_datain[7:0]};
					eth1_IPv4_vh		<=eth1_IPv4_layer[151:144];
					eth1_IPv4_dscp		<=eth1_IPv4_layer[143:136];
					eth1_IPv4_tlen		<=eth1_IPv4_layer[135:120];
					eth1_IPv4_id		<=eth1_IPv4_layer[119:104];
					eth1_IPv4_flags  	<=eth1_IPv4_layer[103:88];
					eth1_IPv4_ttl		<=eth1_IPv4_layer[87:80];
					eth1_IPv4_type		<=eth1_IPv4_layer[79:72];
					eth1_IPv4_crc		<=eth1_IPv4_layer[71:56];
					eth1_IPv4_IP_src	<=eth1_IPv4_layer[55:24];
					eth1_IPv4_IP_dst	<={eth1_IPv4_layer[23:0],eth1_datain[7:0]};
					
					case (eth1_IPv4_layer[79:72])								//IPv4 Type Protocol
						8'd01: begin 												//ICMP
							eth1_ICMP_tlen	<=eth1_IPv4_layer[135:120] - 16'd20; 
							eth1_rx_state  <=rx_ICMP_layer; end	
						8'd17: begin 												//UDP
							eth1_UDP_tlen  <=eth1_IPv4_layer[135:120] - 16'd20; 
							eth1_rx_state  <=rx_UDP_layer;  end
						default: eth1_rx_state<=rx_IDLE;
					endcase
					eth1_rx_byte_cnt <=16'd0;
				end
			end
			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;
		end
		
		rx_ARP_layer:  begin
			if(eth1_byte_rxdv && !eth1_byte_sig) begin
				if(eth1_rx_byte_cnt<16'd27) begin
					eth1_ARP_layer		<={eth1_ARP_layer[215:0],eth1_datain};
					eth1_rx_byte_cnt	<=eth1_rx_byte_cnt+1'b1;
				end
				else begin
					eth1_ARP_layer		<={eth1_ARP_layer[215:0],eth1_datain};//
					
					eth1_ARP_MAC_src	<=eth1_ARP_layer[151:104];
					eth1_ARP_IP_src	<=eth1_ARP_layer[103:72];	
					eth1_ARP_IP_dst	<={eth1_ARP_layer[23:0],eth1_datain};	
					
					
					eth1_rx_byte_cnt	<=16'd0;
					if(eth1_local_ip == {eth1_ARP_layer[23:0],eth1_datain}) begin
						//eth1_arp_ok		<=1'b1; 
						event_eth1_arp_wr	<=1'b1; 
						eth1_rx_state		<=rx_IDLE;
					end
					else begin
						eth1_rx_state	<=rx_ARP_TRL;
					end
				end
			end
			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;
		end
		
		rx_ARP_TRL:    begin //Receive ARP padding 18bytes: 0x00
			if(eth1_byte_rxdv && !eth1_byte_sig) begin
				if(eth1_rx_byte_cnt < 16'd17) begin
					eth1_rx_byte_cnt	<=eth1_rx_byte_cnt+1'b1;
				end
				else begin
					eth1_rx_state		<=rx_ARP_FCS;
					eth1_rx_byte_cnt	<=16'd0;
				end
			end
		end
		
		rx_ARP_FCS:    begin //Receive ethernet frame check sequence for retransmit to Ethernet Port 2
			if(eth1_byte_rxdv && !eth1_byte_sig) begin
				if(eth1_rx_byte_cnt < 16'd3) begin
					eth1_ARP_FCS			<={eth1_ARP_FCS[23:0],eth1_datain};
					eth1_rx_byte_cnt		<=eth1_rx_byte_cnt+1'b1;
				end
				else begin
					eth1_ARP_FCS					<={eth1_ARP_FCS[23:0],eth1_datain};
					//eth1_to_eth2_arp_ok	<=1'b1;
					event_eth1_to_eth2_arp_wr	<=1'b1;
					eth1_rx_byte_cnt				<=16'd0;
					eth1_rx_state					<=rx_IDLE;
				end
			end
			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;
		end
		
		rx_ICMP_layer: begin
			if(eth1_byte_rxdv && !eth1_byte_sig) begin
				if(eth1_rx_byte_cnt<(eth1_ICMP_tlen-1'b1)) begin
					eth1_ICMP_layer	<={eth1_ICMP_layer[503:0],eth1_datain};
					eth1_rx_byte_cnt	<=eth1_rx_byte_cnt+1'b1;
				end
				else begin
					eth1_ICMP_layer	<={eth1_ICMP_layer[503:0],eth1_datain};
					eth1_rx_byte_cnt	<=16'd0;
					if(eth1_local_ip == eth1_IPv4_IP_dst) begin
						//eth1_icmp_ok	<=1'b1; //Receive ICMP
						event_eth1_icmp_wr<=1'b1;
						eth1_rx_state	<=rx_IDLE;
					end
					else begin
						eth1_rx_state	<=rx_ICMP_FCS;
					end
				end
			end
			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;
		end
		
		rx_ICMP_FCS:   begin
			if(eth1_byte_rxdv && !eth1_byte_sig) begin
				if(eth1_rx_byte_cnt < 16'd3) begin
					eth1_ICMP_FCS			<={eth1_ICMP_FCS[23:0],eth1_datain};
					eth1_rx_byte_cnt		<=eth1_rx_byte_cnt+1'b1;
				end
				else begin
					eth1_ICMP_FCS					<={eth1_ICMP_FCS[23:0],eth1_datain};
					//eth1_to_eth2_icmp_ok	<=1'b1;
					event_eth1_to_eth2_icmp_wr	<=1'b1;
					eth1_rx_byte_cnt				<=16'd0;
					eth1_rx_state					<=rx_IDLE;
				end
			end
			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;
		end

		rx_UDP_layer:  begin
			if(eth1_byte_rxdv && !eth1_byte_sig) begin
				if(eth1_rx_byte_cnt < 16'd7) begin 							//8 byte UDP Layer
					eth1_UDP_layer		<={eth1_UDP_layer[55:0],eth1_datain};
					eth1_rx_byte_cnt	<=eth1_rx_byte_cnt+1'b1;
				end
				else begin
					eth1_UDP_layer	 	<={eth1_UDP_layer[55:0],eth1_datain};
					eth1_UDP_SP		 	<=eth1_UDP_layer[55:40];
					eth1_UDP_DP		 	<=eth1_UDP_layer[39:24];
					eth1_UDP_data_len <=(eth1_UDP_layer[23:8]-16'd8);	
					eth1_rx_byte_cnt	<=16'd0;
					if( ({8'd192, 8'd168, 8'd4, 8'd255} == eth1_IPv4_IP_dst) && ((eth1_UDP_layer[23:8]-16'd8) == 16'd10) ) // Broadcast IP and 10 bytes DATA available?
						eth1_rx_state<=rx_UDP_Data;
					else
						eth1_rx_state<=rx_IDLE;
				end
			end
			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;
		end
		
		rx_UDP_Data:   begin
//			if(eth1_byte_rxdv && !eth1_byte_sig) begin
//				if(eth1_rx_byte_cnt < (eth1_UDP_data_len-1'b1)) begin
//					eth1_UDP_data		<={eth1_UDP_data[71:0],eth1_datain};
//					eth1_rx_byte_cnt	<=eth1_rx_byte_cnt+1'b1;
//				end
//				else begin
//					eth1_UDP_data		<={eth1_UDP_data[71:0],eth1_datain};
//					eth1_rx_state		<=rx_IDLE;
//					eth1_rx_byte_cnt	<=16'd0;
//				end	
//			end
//			else if(!eth1_byte_rxdv) eth1_rx_state<=rx_IDLE;

			eth1_rx_state		<=rx_IDLE;
		end
		
		rx_DELAY: 		begin
			if(eth1_rx_byte_cnt < 16'd9) begin
				eth1_rx_byte_cnt	<=eth1_rx_byte_cnt+1'b1;
			end
			else begin
			   eth1_rx_byte_cnt	<=16'd0;
				eth1_rx_state<=rx_IDLE;
			end
		end
		
		default: eth1_rx_state<=rx_IDLE;  
	endcase
end



//***********************************************************************************************************************
reg 	[15:0]eth1_ADC_IPv4_ID;
reg 	[15:0]eth1_tx_i;
reg 	[15:0]eth1_tx_j;
reg 	[15:0]eth1_tx_p;
reg 	[3:0] eth1_tx_igp;
reg  	[7:0] eth1_tx_temp;
reg 	[3:0] eth1_tx_pre;

reg 	[15:0]eth1_pre_arp_bcnt;
reg 	[15:0]eth1_pre_udp_bcnt;
reg 	[15:0]eth1_pre_icmp_bcnt;

reg 	[15:0]eth1_icmp_i;
reg 	[15:0]eth1_icmp_j;

reg 	[15:0]i1;
reg 	[15:0]j1;

reg 	[15:0]i3;
reg 	[15:0]j3;

reg	[3:0] eth1_pre_arp;
reg  	[3:0] eth1_pre_udp;
reg	[3:0] eth1_pre_icmp;

reg	[3:0] eth2_to_eth1_pre_arp;
reg 	[15:0]eth2_to_eth1_pre_arp_bcnt;

reg	[3:0] eth2_to_eth1_pre_icmp;
reg 	[15:0]eth2_to_eth1_pre_icmp_bcnt;

reg	[3:0] eth2_to_eth1_pre_udp;
reg 	[15:0]eth2_to_eth1_pre_udp_bcnt;

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

reg			fifo_eth2_to_eth1_arp_rd;
reg			fifo_eth2_to_eth1_arp_wr;
reg	[7:0]	fifo_eth2_to_eth1_arp_din;
wire	[7:0]	fifo_eth2_to_eth1_arp_dout;
wire			fifo_eth2_to_eth1_arp_empty;
wire			fifo_eth2_to_eth1_arp_full;
wire	[15:0]fifo_eth2_to_eth1_arp_cnt;

reg			fifo_eth2_to_eth1_icmp_rd;
reg			fifo_eth2_to_eth1_icmp_wr;
reg	[7:0]	fifo_eth2_to_eth1_icmp_din;
wire	[7:0]	fifo_eth2_to_eth1_icmp_dout;
wire			fifo_eth2_to_eth1_icmp_empty;
wire			fifo_eth2_to_eth1_icmp_full;
wire	[15:0]fifo_eth2_to_eth1_icmp_cnt;

reg			fifo_eth2_to_eth1_udp_rd;
reg			fifo_eth2_to_eth1_udp_wr;
reg	[7:0]	fifo_eth2_to_eth1_udp_din;
wire	[7:0]	fifo_eth2_to_eth1_udp_dout;
wire			fifo_eth2_to_eth1_udp_empty;
wire			fifo_eth2_to_eth1_udp_full;
wire	[15:0]fifo_eth2_to_eth1_udp_cnt;

reg   [31:0]eth1_adc_crc;
reg	[15:0]eth1_adc_i;
reg   [31:0]pkt_cnt;

initial begin
	fifo_eth1_adc_rd  <=1'b0;
	fifo_eth1_adc_wr  <=1'b0;
	fifo_eth1_arp_wr  <=1'b0;
	fifo_eth1_icmp_wr <=1'b0;
	eth1_ADC_IPv4_ID	<=16'd0;
	eth1_tx_i			<=16'd0;
	eth1_tx_j			<=16'd0;
	eth1_tx_p			<=16'd0;
	eth1_tx_igp			<=4'd0;
	eth1_tx_temp		<=8'd0;
	eth1_pre_arp_bcnt <=16'd0;
	eth1_pre_udp_bcnt <=16'd0;
	eth1_pre_icmp_bcnt<=16'd0;
	eth2_to_eth1_pre_arp_bcnt	<=16'd0;
	eth2_to_eth1_pre_icmp_bcnt <=16'd0;
	eth2_to_eth1_pre_udp_bcnt  <=16'd0;
	eth1_pre_arp<=4'd0;
	eth1_pre_udp<=4'd0;
	eth1_pre_icmp<=4'd0;
	eth2_to_eth1_pre_arp			<=4'd0;
	eth2_to_eth1_pre_icmp		<=4'd0;
	eth2_to_eth1_pre_udp			<=4'd0;
	eth1_tx_pre<=4'd0;
	eth1_adc_i<=16'd0;
	pkt_cnt<=32'd0;
end

//for debug

assign fifo_adc_rd 	 = fifo_eth1_adc_rd; 
assign fifo_adc_wr 	 = adc_ok;
assign fifo_adc_empty = fifo_eth1_adc_empty;
assign fifo_adc_full  = fifo_eth1_adc_full;
assign fifo_adc_cnt   = fifo_eth1_adc_cnt;
assign fifo_adc_dout  = fifo_eth1_adc_dout;
/*
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
assign fifo_icmp_empty = fifo_eth1_icmp_empty;*/

//assign alg_pre_eth1_icmp = pre_eth1_icmp;

/*assign arp			= eth1_arp_ok;
assign udp			= eth1_udp_ok;
assign icmp			= eth1_icmp_ok;*/
//***********************************************************************************************************************

//********************************************************//
//*Put ADC data in FIFO for transmission						*//
//********************************************************//
fifo2 # (8,4096) fifo_eth1_adc
(
	.clk		(eth1_tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth1_adc_rd & !fifo_eth1_adc_empty),
	.wr		(adc_ok      		& !fifo_eth1_adc_full),
	.din		(adc_byte),
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
	.clk		(eth1_tx_clk),
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
	.clk		(eth1_tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth1_icmp_rd & !fifo_eth1_icmp_empty),
	.wr		(fifo_eth1_icmp_wr & !fifo_eth1_icmp_full),
	.din		(fifo_eth1_icmp_din),
	.dout		(fifo_eth1_icmp_dout),
	.empty	(fifo_eth1_icmp_empty),
	.full		(fifo_eth1_icmp_full),
	.cnt		(fifo_eth1_icmp_cnt)
);

//********************************************************//
//*Put ARP Packet in FIFO for transmission						*//
//********************************************************//
fifo2 # (8,512) fifo_eth2_to_eth1_arp
(
	.clk		(eth1_tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth2_to_eth1_arp_rd & !fifo_eth2_to_eth1_arp_empty),
	.wr		(fifo_eth2_to_eth1_arp_wr & !fifo_eth2_to_eth1_arp_full),
	.din		(fifo_eth2_to_eth1_arp_din),
	.dout		(fifo_eth2_to_eth1_arp_dout),
	.empty	(fifo_eth2_to_eth1_arp_empty),
	.full		(fifo_eth2_to_eth1_arp_full),
	.cnt		(fifo_eth2_to_eth1_arp_cnt)
);

//********************************************************//
//*Put ICMP Packet in FIFO for transmission						*//
//********************************************************//
fifo2 # (8,512) fifo_eth2_to_eth1_icmp
(
	.clk		(eth1_tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth2_to_eth1_icmp_rd & !fifo_eth2_to_eth1_icmp_empty),
	.wr		(fifo_eth2_to_eth1_icmp_wr & !fifo_eth2_to_eth1_icmp_full),
	.din		(fifo_eth2_to_eth1_icmp_din),
	.dout		(fifo_eth2_to_eth1_icmp_dout),
	.empty	(fifo_eth2_to_eth1_icmp_empty),
	.full		(fifo_eth2_to_eth1_icmp_full),
	.cnt		(fifo_eth2_to_eth1_icmp_cnt)
);

//********************************************************//
//*Put UDP Packet in FIFO for transmission						*//
//********************************************************//
fifo2 # (8,4096) fifo_eth2_to_eth1_udp
//fifo2 # (8,2048) fifo_eth2_to_eth1_udp
(
	.clk		(eth1_tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth2_to_eth1_udp_rd & !fifo_eth2_to_eth1_udp_empty),
	.wr		(fifo_eth2_to_eth1_udp_wr & !fifo_eth2_to_eth1_udp_full),
	.din		(fifo_eth2_to_eth1_udp_din),
	.dout		(fifo_eth2_to_eth1_udp_dout),
	.empty	(fifo_eth2_to_eth1_udp_empty),
	.full		(fifo_eth2_to_eth1_udp_full),
	.cnt		(fifo_eth2_to_eth1_udp_cnt)
);

//***********************************************************************************************************************//
//*ETHERNET1: TRANSMIT PACKET'S																														*//
//***********************************************************************************************************************//
always@(posedge eth1_tx_clk) begin

	eth1_tx_cnt<=eth1_tx_cnt+1'b1;

	//*******************************************************//
	//Prepare ARP packet from Eth1, and put in FIFO			  *//
	//*******************************************************//
	case(eth1_pre_arp)
		4'd0: begin //Idle
			fifo_eth1_arp_wr<=1'b0;
			eth1_pre_arp_bcnt<=16'd0;
			event_eth1_arp_rd<=1'b0;
			
			if(event_eth1_arp_ok) begin
				eth1_pre_arp<=4'd1;
				event_eth1_arp_rd<=1'b1;
			end
		end
		
		4'd1: begin //Fill
			event_eth1_arp_rd<=1'b0;
			
			eth1_tx_arp_data[0]  <=8'h55;               //PREAMBULE  
			eth1_tx_arp_data[1]  <=8'h55;
			eth1_tx_arp_data[2]  <=8'h55;
			eth1_tx_arp_data[3]  <=8'h55;
			eth1_tx_arp_data[4]  <=8'h55;
			eth1_tx_arp_data[5]  <=8'h55;
			eth1_tx_arp_data[6]  <=8'h55;
			eth1_tx_arp_data[7]	<=8'hD5; 
			
			eth1_tx_arp_data[8]  <=eth1_ARP_MAC_src[47:40];	//Dest MAC  
			eth1_tx_arp_data[9]  <=eth1_ARP_MAC_src[39:32];
			eth1_tx_arp_data[10] <=eth1_ARP_MAC_src[31:24];
			eth1_tx_arp_data[11] <=eth1_ARP_MAC_src[23:16];
			eth1_tx_arp_data[12] <=eth1_ARP_MAC_src[15:8];
			eth1_tx_arp_data[13] <=eth1_ARP_MAC_src[7:0];	
			
			eth1_tx_arp_data[14] <=eth1_local_mac[47:40];	//Src MAC
			eth1_tx_arp_data[15] <=eth1_local_mac[39:32];
			eth1_tx_arp_data[16] <=eth1_local_mac[31:24];
			eth1_tx_arp_data[17] <=eth1_local_mac[23:16];
			eth1_tx_arp_data[18] <=eth1_local_mac[15:8];
			eth1_tx_arp_data[19] <=eth1_local_mac[7:0];	
			
			eth1_tx_arp_data[20] <=eth1_type[15:8]; 			//TYPE 
			eth1_tx_arp_data[21] <=eth1_type[7:0];
			eth1_tx_arp_data[22] <=8'h00;							//HType
			eth1_tx_arp_data[23] <=8'h01;
			eth1_tx_arp_data[24] <=8'h08;							//IPv4
			eth1_tx_arp_data[25] <=8'h00;
			eth1_tx_arp_data[26] <=8'h06;							//Hsz
			eth1_tx_arp_data[27] <=8'h04;							//Psz
			eth1_tx_arp_data[28] <=8'h00;							//Opcode 0x0002 reply
			eth1_tx_arp_data[29] <=8'h02;
			eth1_tx_arp_data[30] <=eth1_local_mac[47:40];	//My MAC  
			eth1_tx_arp_data[31] <=eth1_local_mac[39:32];
			eth1_tx_arp_data[32] <=eth1_local_mac[31:24];
			eth1_tx_arp_data[33] <=eth1_local_mac[23:16];
			eth1_tx_arp_data[34] <=eth1_local_mac[15:8];
			eth1_tx_arp_data[35] <=eth1_local_mac[7:0];
			eth1_tx_arp_data[36] <=eth1_local_ip[31:24];		//My IP   
			eth1_tx_arp_data[37] <=eth1_local_ip[23:16];
			eth1_tx_arp_data[38] <=eth1_local_ip[15:8];
			eth1_tx_arp_data[39] <=eth1_local_ip[7:0];
			eth1_tx_arp_data[40] <=eth1_ARP_MAC_src[47:40];	//Target MAC 
			eth1_tx_arp_data[41] <=eth1_ARP_MAC_src[39:32];
			eth1_tx_arp_data[42] <=eth1_ARP_MAC_src[31:24];
			eth1_tx_arp_data[43] <=eth1_ARP_MAC_src[23:16];
			eth1_tx_arp_data[44] <=eth1_ARP_MAC_src[15:8];
			eth1_tx_arp_data[45] <=eth1_ARP_MAC_src[7:0];	
			eth1_tx_arp_data[46] <=eth1_ARP_IP_src[31:24];	//Target IP   
			eth1_tx_arp_data[47] <=eth1_ARP_IP_src[23:16];
			eth1_tx_arp_data[48] <=eth1_ARP_IP_src[15:8];
			eth1_tx_arp_data[49] <=eth1_ARP_IP_src[7:0];
			
			//Padding 18 bytes
			eth1_tx_arp_data[50] <=8'h00;
			eth1_tx_arp_data[51] <=8'h00;
			eth1_tx_arp_data[52] <=8'h00;
			eth1_tx_arp_data[53] <=8'h00;
			eth1_tx_arp_data[54] <=8'h00;
			eth1_tx_arp_data[55] <=8'h00;
			eth1_tx_arp_data[56] <=8'h00;
			eth1_tx_arp_data[57] <=8'h00;
			eth1_tx_arp_data[58] <=8'h00;
			eth1_tx_arp_data[59] <=8'h00;
			eth1_tx_arp_data[60] <=8'h00;
			eth1_tx_arp_data[61] <=8'h00;
			eth1_tx_arp_data[62] <=8'h00;
			eth1_tx_arp_data[63] <=8'h00;
			eth1_tx_arp_data[64] <=8'h00;
			eth1_tx_arp_data[65] <=8'h00;
			eth1_tx_arp_data[66] <=8'h00;
			eth1_tx_arp_data[67] <=8'h00;
			
			eth1_pre_arp<=4'd2;
		end

		4'd2: begin //Put it FIFO
			//if(eth1_pre_arp_bcnt<16'd49) begin
			if(eth1_pre_arp_bcnt<16'd67) begin
				fifo_eth1_arp_wr	<=1'b1;
				fifo_eth1_arp_din <=eth1_tx_arp_data[eth1_pre_arp_bcnt];
				eth1_pre_arp_bcnt <=eth1_pre_arp_bcnt+1'b1;
			end
			else begin
				fifo_eth1_arp_din <=eth1_tx_arp_data[eth1_pre_arp_bcnt];
				eth1_pre_arp 	<=4'd0;
			end
		end

		default: eth1_pre_arp<=4'd0;
	endcase
	
	//*******************************************************//
	//Prepare ICMP packet from Eth1, and put in FIFO		  *//
	//*******************************************************//
	case(eth1_pre_icmp)
		4'd0: begin //Idle
			eth1_icmp_i<=16'd0;
			eth1_icmp_j<=16'd0;
			fifo_eth1_icmp_wr<=1'b0;
			event_eth1_icmp_rd<=1'b0;

			if(event_eth1_icmp_ok) begin
				eth1_pre_icmp<=4'd1;
				event_eth1_icmp_rd<=1'b1;
			end
		end
		
		4'd1: begin //Fill
			//event_eth1_icmp_rd<=1'b0;
			
			eth1_icmp_cnt<=eth1_icmp_cnt+1'b1;
			//PREAMBULE  
			eth1_tx_icmp_data[0]  <=8'h55;					
			eth1_tx_icmp_data[1]  <=8'h55;
			eth1_tx_icmp_data[2]  <=8'h55;
			eth1_tx_icmp_data[3]  <=8'h55;
			eth1_tx_icmp_data[4]  <=8'h55;
			eth1_tx_icmp_data[5]  <=8'h55;
			eth1_tx_icmp_data[6]  <=8'h55;
			eth1_tx_icmp_data[7]  <=8'hD5; 
			//ETH
			eth1_tx_icmp_data[8]  <=eth1_MAC_src[47:40];		//Dest MAC  
			eth1_tx_icmp_data[9]  <=eth1_MAC_src[39:32];
			eth1_tx_icmp_data[10] <=eth1_MAC_src[31:24];
			eth1_tx_icmp_data[11] <=eth1_MAC_src[23:16];
			eth1_tx_icmp_data[12] <=eth1_MAC_src[15:8];
			eth1_tx_icmp_data[13] <=eth1_MAC_src[7:0];	
			eth1_tx_icmp_data[14] <=eth1_local_mac[47:40];	//Src MAC
			eth1_tx_icmp_data[15] <=eth1_local_mac[39:32];
			eth1_tx_icmp_data[16] <=eth1_local_mac[31:24];
			eth1_tx_icmp_data[17] <=eth1_local_mac[23:16];
			eth1_tx_icmp_data[18] <=eth1_local_mac[15:8];
			eth1_tx_icmp_data[19] <=eth1_local_mac[7:0];	
			eth1_tx_icmp_data[20] <=eth1_type[15:8];			//TYPE 
			eth1_tx_icmp_data[21] <=eth1_type[7:0];
			//IPv4
			eth1_tx_icmp_data[22] <=8'h45;						//Version number: 4; Header length: 20; 
			eth1_tx_icmp_data[23] <=8'h00;
			eth1_tx_icmp_data[24] <=eth1_IPv4_tlen[15:8];	//IPv4 total length
			eth1_tx_icmp_data[25] <=eth1_IPv4_tlen[7:0];
			eth1_tx_icmp_data[26] <=eth1_icmp_cnt[15:8];		//Package serial number
			eth1_tx_icmp_data[27] <=eth1_icmp_cnt[7:0];
			eth1_tx_icmp_data[28] <=8'h00;						//Fragment offset
			eth1_tx_icmp_data[29] <=8'h00;
			eth1_tx_icmp_data[30] <=eth1_IPv4_ttl;				//TTL 64 8'h40;
			eth1_tx_icmp_data[31] <=8'h01;						//Protocol: 01(ICMP)
			eth1_tx_icmp_data[32] <=8'h00;						//Header checksum
			eth1_tx_icmp_data[33] <=8'h00;
			eth1_tx_icmp_data[34] <=eth1_local_ip[31:24];	//Source IP   
			eth1_tx_icmp_data[35] <=eth1_local_ip[23:16];
			eth1_tx_icmp_data[36] <=eth1_local_ip[15:8];
			eth1_tx_icmp_data[37] <=eth1_local_ip[7:0];				
			eth1_tx_icmp_data[38] <=eth1_IPv4_IP_src[31:24];		//Dest IP
			eth1_tx_icmp_data[39] <=eth1_IPv4_IP_src[23:16];
			eth1_tx_icmp_data[40] <=eth1_IPv4_IP_src[15:8];
			eth1_tx_icmp_data[41] <=eth1_IPv4_IP_src[7:0];
			
			case(eth1_ICMP_tlen)
				16'd64: begin //ICMP packet for Linux OS
					eth1_tx_icmp_data[42]  <=8'h00;						//TYPE:0 (Echo (ping) reply)
					eth1_tx_icmp_data[43]  <=8'h00;						//CODE:0
					eth1_tx_icmp_data[44]  <=8'h00;						//CHECKSUM
					eth1_tx_icmp_data[45]  <=8'h00;
					
					//Fill data
					for(int eth1_icmp_i=46, eth1_icmp_j=0; eth1_icmp_i<106; eth1_icmp_i=eth1_icmp_i+1, eth1_icmp_j=eth1_icmp_j+1) begin
						eth1_tx_icmp_data[eth1_icmp_i] <=eth1_ICMP_layer[479-(eth1_icmp_j*8)-:8];//487
					end
				end
				
				default: eth1_pre_icmp<=4'd0;
			endcase
			eth1_pre_icmp<=4'd2;
		end
		
		4'd2: begin //Generate CRC for IPv4
			if(eth1_icmp_i==0) begin											
				eth1_chk_buf <={eth1_tx_icmp_data[24],eth1_tx_icmp_data[25]} + {eth1_tx_icmp_data[22],eth1_tx_icmp_data[23]} +
									{eth1_tx_icmp_data[28],eth1_tx_icmp_data[29]} + {eth1_tx_icmp_data[26],eth1_tx_icmp_data[27]} +
									{eth1_tx_icmp_data[32],eth1_tx_icmp_data[33]} + {eth1_tx_icmp_data[30],eth1_tx_icmp_data[31]} +
									{eth1_tx_icmp_data[36],eth1_tx_icmp_data[37]} + {eth1_tx_icmp_data[34],eth1_tx_icmp_data[35]} +
									{eth1_tx_icmp_data[40],eth1_tx_icmp_data[41]} + {eth1_tx_icmp_data[38],eth1_tx_icmp_data[39]};
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else if(eth1_icmp_i==1) begin
				eth1_chk_buf[15:0]<=eth1_chk_buf[31:16]+eth1_chk_buf[15:0];
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else begin
				eth1_tx_icmp_data[32]<=~eth1_chk_buf[15:8];
				eth1_tx_icmp_data[33]<=~eth1_chk_buf[7:0];
				eth1_icmp_i<=0;
				eth1_icmp_j<=0;
				eth1_pre_icmp<=4'd3;
			end
		end
		
		4'd3: begin //Generate CRC ICMP
			if(eth1_icmp_i==0) begin
				case(eth1_ICMP_tlen)
					16'd64: begin //for Linux OS
						eth1_chk_buf <={eth1_tx_icmp_data[44],eth1_tx_icmp_data[45]}   + {eth1_tx_icmp_data[42],eth1_tx_icmp_data[43]} +
											{eth1_tx_icmp_data[48],eth1_tx_icmp_data[49]}   + {eth1_tx_icmp_data[46],eth1_tx_icmp_data[47]} +
											{eth1_tx_icmp_data[52],eth1_tx_icmp_data[53]}   + {eth1_tx_icmp_data[50],eth1_tx_icmp_data[51]} +
											{eth1_tx_icmp_data[56],eth1_tx_icmp_data[57]}   + {eth1_tx_icmp_data[54],eth1_tx_icmp_data[55]} +
											{eth1_tx_icmp_data[60],eth1_tx_icmp_data[61]}   + {eth1_tx_icmp_data[58],eth1_tx_icmp_data[59]} +	
											{eth1_tx_icmp_data[64],eth1_tx_icmp_data[65]}   + {eth1_tx_icmp_data[62],eth1_tx_icmp_data[63]} +
											{eth1_tx_icmp_data[68],eth1_tx_icmp_data[69]}   + {eth1_tx_icmp_data[66],eth1_tx_icmp_data[67]} +
											{eth1_tx_icmp_data[72],eth1_tx_icmp_data[73]}   + {eth1_tx_icmp_data[70],eth1_tx_icmp_data[71]} +
											{eth1_tx_icmp_data[76],eth1_tx_icmp_data[77]}   + {eth1_tx_icmp_data[74],eth1_tx_icmp_data[75]} +
											{eth1_tx_icmp_data[80],eth1_tx_icmp_data[81]}   + {eth1_tx_icmp_data[78],eth1_tx_icmp_data[79]} +
											{eth1_tx_icmp_data[84],eth1_tx_icmp_data[85]}   + {eth1_tx_icmp_data[82],eth1_tx_icmp_data[83]} +
											{eth1_tx_icmp_data[88],eth1_tx_icmp_data[89]}   + {eth1_tx_icmp_data[86],eth1_tx_icmp_data[87]} +
											{eth1_tx_icmp_data[92],eth1_tx_icmp_data[93]}   + {eth1_tx_icmp_data[90],eth1_tx_icmp_data[91]} +
											{eth1_tx_icmp_data[96],eth1_tx_icmp_data[97]}   + {eth1_tx_icmp_data[94],eth1_tx_icmp_data[95]} +
											{eth1_tx_icmp_data[100],eth1_tx_icmp_data[101]} + {eth1_tx_icmp_data[98],eth1_tx_icmp_data[99]} +
											{eth1_tx_icmp_data[104],eth1_tx_icmp_data[105]} + {eth1_tx_icmp_data[102],eth1_tx_icmp_data[103]};	
					end
					
					default: eth1_pre_icmp<=4'd0;
				endcase
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else if(eth1_icmp_i==1) begin
				eth1_chk_buf[15:0]<=eth1_chk_buf[31:16]+eth1_chk_buf[15:0];
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else begin
				eth1_tx_icmp_data[44]<=~eth1_chk_buf[15:8];
				eth1_tx_icmp_data[45]<=~eth1_chk_buf[7:0];
				eth1_icmp_i<=0;
				eth1_icmp_j<=0;
				case(eth1_ICMP_tlen)
					16'd64:  eth1_pre_icmp<=4'd4;
					default: eth1_pre_icmp<=4'd0;
				endcase
			end
		end
		
		4'd4: begin //Put it FIFO
			//Put in FIFO ICMP data
			if(eth1_icmp_i<16'd105) begin //104
				fifo_eth1_icmp_wr	<=1'b1;
				fifo_eth1_icmp_din<=eth1_tx_icmp_data[eth1_icmp_i];
				eth1_icmp_i<=eth1_icmp_i+1'b1;
			end
			else begin//105
				fifo_eth1_icmp_din<=eth1_tx_icmp_data[eth1_icmp_i];
				eth1_pre_icmp<=4'd0;
			end
			
		end
		
		default: eth1_pre_icmp<=4'd0;
	endcase
	
	//*******************************************************//
	//Prepare ARP packet from Eth2, and put in FIFO		  	  *//
	//*******************************************************//	
	case(eth2_to_eth1_pre_arp)
		4'd0: begin //Idle
			fifo_eth2_to_eth1_arp_wr	<=1'b0;
			eth2_to_eth1_pre_arp_bcnt	<=16'd0;
			event_eth2_to_eth1_arp_rd  <=1'b0;
			
			if(event_eth2_to_eth1_arp_ok) begin
				event_eth2_to_eth1_arp_rd <=1'b1;
				eth2_to_eth1_pre_arp<=4'd1;
			end
		end
		
		4'd1: begin //Fill
			event_eth2_to_eth1_arp_rd <=1'b0;
			
			eth2_to_eth1_arp[0]  <=8'h55;               //PREAMBULE  
			eth2_to_eth1_arp[1]  <=8'h55;
			eth2_to_eth1_arp[2]  <=8'h55;
			eth2_to_eth1_arp[3]  <=8'h55;
			eth2_to_eth1_arp[4]  <=8'h55;
			eth2_to_eth1_arp[5]  <=8'h55;
			eth2_to_eth1_arp[6]  <=8'h55;
			eth2_to_eth1_arp[7]	<=8'hD5; 

			eth2_to_eth1_arp[8]  <=eth2_layer[111:104];//Dest MAC  
			eth2_to_eth1_arp[9]  <=eth2_layer[103:96];
			eth2_to_eth1_arp[10] <=eth2_layer[95:88];
			eth2_to_eth1_arp[11] <=eth2_layer[87:80];
			eth2_to_eth1_arp[12] <=eth2_layer[79:72];
			eth2_to_eth1_arp[13] <=eth2_layer[71:64];
			
			eth2_to_eth1_arp[14] <=eth2_layer[63:56];//Src MAC
			eth2_to_eth1_arp[15] <=eth2_layer[55:48];
			eth2_to_eth1_arp[16] <=eth2_layer[47:40];
			eth2_to_eth1_arp[17] <=eth2_layer[39:32];
			eth2_to_eth1_arp[18] <=eth2_layer[31:24];
			eth2_to_eth1_arp[19] <=eth2_layer[23:16];
			
			eth2_to_eth1_arp[20] <=eth2_layer[15:8];//TYPE 
			eth2_to_eth1_arp[21] <=eth2_layer[7:0];
			
			eth2_to_eth1_arp[22] <=eth2_ARP_layer[223:216];//HType
			eth2_to_eth1_arp[23] <=eth2_ARP_layer[215:208];
			eth2_to_eth1_arp[24] <=eth2_ARP_layer[207:200];//IPv4
			eth2_to_eth1_arp[25] <=eth2_ARP_layer[199:192];
			eth2_to_eth1_arp[26] <=eth2_ARP_layer[191:184];//Hsz
			eth2_to_eth1_arp[27] <=eth2_ARP_layer[183:176];//Psz
			eth2_to_eth1_arp[28] <=eth2_ARP_layer[175:168];//Opcode 0x0002 reply
			eth2_to_eth1_arp[29] <=eth2_ARP_layer[167:160];
			
			eth2_to_eth1_arp[30] <=eth2_ARP_layer[159:152];//My MAC  
			eth2_to_eth1_arp[31] <=eth2_ARP_layer[151:144];
			eth2_to_eth1_arp[32] <=eth2_ARP_layer[143:136];
			eth2_to_eth1_arp[33] <=eth2_ARP_layer[135:128];
			eth2_to_eth1_arp[34] <=eth2_ARP_layer[127:120];
			eth2_to_eth1_arp[35] <=eth2_ARP_layer[119:112];
			
			eth2_to_eth1_arp[36] <=eth2_ARP_layer[111:104];//My IP   
			eth2_to_eth1_arp[37] <=eth2_ARP_layer[103:96];
			eth2_to_eth1_arp[38] <=eth2_ARP_layer[95:88];
			eth2_to_eth1_arp[39] <=eth2_ARP_layer[87:80];
			
			eth2_to_eth1_arp[40] <=eth2_ARP_layer[79:72];//Target MAC 
			eth2_to_eth1_arp[41] <=eth2_ARP_layer[71:64];
			eth2_to_eth1_arp[42] <=eth2_ARP_layer[63:56];
			eth2_to_eth1_arp[43] <=eth2_ARP_layer[55:48];
			eth2_to_eth1_arp[44] <=eth2_ARP_layer[47:40];
			eth2_to_eth1_arp[45] <=eth2_ARP_layer[39:32];
			
			eth2_to_eth1_arp[46] <=eth2_ARP_layer[31:24];//Target IP   
			eth2_to_eth1_arp[47] <=eth2_ARP_layer[23:16];
			eth2_to_eth1_arp[48] <=eth2_ARP_layer[15:8];
			eth2_to_eth1_arp[49] <=eth2_ARP_layer[7:0];
			
			eth2_to_eth1_arp[50] <=8'h00; //Padding
			eth2_to_eth1_arp[51] <=8'h00;
			eth2_to_eth1_arp[52] <=8'h00;
			eth2_to_eth1_arp[53] <=8'h00;
			eth2_to_eth1_arp[54] <=8'h00;
			eth2_to_eth1_arp[55] <=8'h00;
			eth2_to_eth1_arp[56] <=8'h00;
			eth2_to_eth1_arp[57] <=8'h00;
			eth2_to_eth1_arp[58] <=8'h00;
			eth2_to_eth1_arp[59] <=8'h00;
			eth2_to_eth1_arp[60] <=8'h00;
			eth2_to_eth1_arp[61] <=8'h00;
			eth2_to_eth1_arp[62] <=8'h00;
			eth2_to_eth1_arp[63] <=8'h00;
			eth2_to_eth1_arp[64] <=8'h00;
			eth2_to_eth1_arp[65] <=8'h00;
			eth2_to_eth1_arp[66] <=8'h00;
			eth2_to_eth1_arp[67] <=8'h00;
			
			eth2_to_eth1_arp[68] <=eth2_ARP_FCS[31:24];//CRC
			eth2_to_eth1_arp[69] <=eth2_ARP_FCS[23:16];
			eth2_to_eth1_arp[70] <=eth2_ARP_FCS[15:8];
			eth2_to_eth1_arp[71] <=eth2_ARP_FCS[7:0];

			eth2_to_eth1_pre_arp<=4'd2;
		end
		
		4'd2: begin //Put it FIFO
			if(eth2_to_eth1_pre_arp_bcnt<16'd71) begin
				fifo_eth2_to_eth1_arp_wr	<=1'b1;
				fifo_eth2_to_eth1_arp_din 	<=eth2_to_eth1_arp[eth2_to_eth1_pre_arp_bcnt];
				eth2_to_eth1_pre_arp_bcnt  <=eth2_to_eth1_pre_arp_bcnt+1'b1;
			end
			else begin
				fifo_eth2_to_eth1_arp_din 	<=eth2_to_eth1_arp[eth2_to_eth1_pre_arp_bcnt];
				eth2_to_eth1_pre_arp			<=4'd0;
			end
		end
		
		default: eth2_to_eth1_pre_arp<=4'd0;
	endcase
		
	//*******************************************************//
	//Prepare ICMP packet from Eth2, and put in FIFO		  	  *//
	//*******************************************************//	
	case(eth2_to_eth1_pre_icmp)
		4'd0: begin //Idle
			fifo_eth2_to_eth1_icmp_wr	<=1'b0;
			eth2_to_eth1_pre_icmp_bcnt	<=16'd0;
			event_eth2_to_eth1_icmp_rd <=1'b0;
			
			if(event_eth2_to_eth1_icmp_ok) begin
				eth2_to_eth1_pre_icmp		<=4'd1;
				event_eth2_to_eth1_icmp_rd <=1'b1;
			end
		end
		
		4'd1: begin //Fill
			event_eth2_to_eth1_icmp_rd <=1'b0;
			
			eth2_to_eth1_icmp[0]  <=8'h55;               //PREAMBULE  
			eth2_to_eth1_icmp[1]  <=8'h55;
			eth2_to_eth1_icmp[2]  <=8'h55;
			eth2_to_eth1_icmp[3]  <=8'h55;
			eth2_to_eth1_icmp[4]  <=8'h55;
			eth2_to_eth1_icmp[5]  <=8'h55;
			eth2_to_eth1_icmp[6]  <=8'h55;
			eth2_to_eth1_icmp[7]	 <=8'hD5; 

			//Fill ETH Layer
			for(i1=8, j1=0; i1<22; i1=i1+1, j1=j1+1) begin
				eth2_to_eth1_icmp[i1] <=eth2_layer[111-(j1*8)-:8];
			end
			
			//Fill IPv4 Layer
			for(i1=22, j1=0; i1<42; i1=i1+1, j1=j1+1) begin
				eth2_to_eth1_icmp[i1] <=eth2_IPv4_layer[159-(j1*8)-:8];
			end
			
			//Fill ICMP Layer
			for(i1=42, j1=0; i1<106; i1=i1+1, j1=j1+1) begin
				eth2_to_eth1_icmp[i1] <=eth2_ICMP_layer[511-(j1*8)-:8];//487
			end

			eth2_to_eth1_icmp[106] <=eth2_ICMP_FCS[31:24];
			eth2_to_eth1_icmp[107] <=eth2_ICMP_FCS[23:16];
			eth2_to_eth1_icmp[108] <=eth2_ICMP_FCS[15:8];
			eth2_to_eth1_icmp[109] <=eth2_ICMP_FCS[7:0];
			
			eth2_to_eth1_pre_icmp<=4'd2;
		end
		
		4'd2: begin //Put it FIFO
			if(eth2_to_eth1_pre_icmp_bcnt<16'd109) begin
				fifo_eth2_to_eth1_icmp_wr	<=1'b1;
				fifo_eth2_to_eth1_icmp_din	<=eth2_to_eth1_icmp[eth2_to_eth1_pre_icmp_bcnt];
				eth2_to_eth1_pre_icmp_bcnt <=eth2_to_eth1_pre_icmp_bcnt+1'b1;
			end
			else begin
				fifo_eth2_to_eth1_icmp_din	<=eth2_to_eth1_icmp[eth2_to_eth1_pre_icmp_bcnt];
				eth2_to_eth1_pre_icmp		<=4'd0;
			end
		end
		
		
		default: eth2_to_eth1_pre_icmp<=4'd0;
	endcase
	
	//*******************************************************//
	//Prepare UDP packet from Eth2, and put in FIFO		  	  *//
	//*******************************************************//	
	case(eth2_to_eth1_pre_udp)
		4'd0: begin //Idle
			fifo_eth2_to_eth1_udp_wr	<=1'b0;
			eth2_to_eth1_pre_udp_bcnt	<=16'd0;
			event_eth2_to_eth1_udp_rd	<=1'b0;
			fifo_temp_eth2_udp_rd		<=1'b0;
			ram_eth2_udp_rd_addr			<=12'd0;
			
			if(event_eth2_to_eth1_udp_ok)begin //eth2_udp_ok
				eth2_to_eth1_pre_udp			<=4'd1;
				event_eth2_to_eth1_udp_rd	<=1'b1;
				//double_mac:double_mac_inst|event_eth2_to_eth1_udp_wr
			end
		end
	
		4'd1: begin //Fill
			event_eth2_to_eth1_udp_rd	<=1'b0;
		
			eth2_to_eth1_udp[0]  <=8'h55;//PREAMBULE  
			eth2_to_eth1_udp[1]  <=8'h55;
			eth2_to_eth1_udp[2]  <=8'h55;
			eth2_to_eth1_udp[3]  <=8'h55;
			eth2_to_eth1_udp[4]  <=8'h55;
			eth2_to_eth1_udp[5]  <=8'h55;
			eth2_to_eth1_udp[6]  <=8'h55;
			eth2_to_eth1_udp[7]	<=8'hD5; 

			//Fill ETH Layer
			for(i3=8, j3=0; i3<22; i3=i3+1, j3=j3+1) begin
				eth2_to_eth1_udp[i3] <=eth2_layer[111-(j3*8)-:8];
			end
			
			//Fill IPv4 Layer
			for(i3=22, j3=0; i3<42; i3=i3+1, j3=j3+1) begin
				eth2_to_eth1_udp[i3] <=eth2_IPv4_layer[159-(j3*8)-:8];
			end
			
			//Fill UDP Layer
//			for(i3=42, j3=0; i3<1458; i3=i3+1, j3=j3+1) begin
//				eth2_to_eth1_udp[i3] <=eth2_UDP_layer[11327-(j3*8)-:8];
//			end


			eth2_to_eth1_udp[42] <=eth2_UDP_layer[63:56];
			eth2_to_eth1_udp[43] <=eth2_UDP_layer[55:48];
			eth2_to_eth1_udp[44] <=eth2_UDP_layer[47:40];
			eth2_to_eth1_udp[45] <=eth2_UDP_layer[39:32];
			eth2_to_eth1_udp[46] <=eth2_UDP_layer[31:24];
			eth2_to_eth1_udp[47] <=eth2_UDP_layer[23:16];
			eth2_to_eth1_udp[48] <=eth2_UDP_layer[15:8];
			eth2_to_eth1_udp[49] <=eth2_UDP_layer[7:0];


			/*eth2_to_eth1_udp[1458] <=eth2_UDP_FCS[31:24];
			eth2_to_eth1_udp[1459] <=eth2_UDP_FCS[23:16];
			eth2_to_eth1_udp[1460] <=eth2_UDP_FCS[15:8];
			eth2_to_eth1_udp[1461] <=eth2_UDP_FCS[7:0];*/
			
			
			eth2_to_eth1_udp[50] <=eth2_UDP_FCS[31:24];
			eth2_to_eth1_udp[51] <=eth2_UDP_FCS[23:16];
			eth2_to_eth1_udp[52] <=eth2_UDP_FCS[15:8];
			eth2_to_eth1_udp[53] <=eth2_UDP_FCS[7:0];
			
			eth2_to_eth1_pre_udp<=4'd2;
		end
	
		4'd2: begin //Put it FIFO
			if(eth2_to_eth1_pre_udp_bcnt<=16'd49) begin
				fifo_eth2_to_eth1_udp_wr	<=1'b1;
				eth2_to_eth1_pre_udp_bcnt  <=eth2_to_eth1_pre_udp_bcnt+1'b1;
				fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[eth2_to_eth1_pre_udp_bcnt];

				if(eth2_to_eth1_pre_udp_bcnt==16'd49) begin
					fifo_temp_eth2_udp_rd		<=1'b1;	//dummy
					ram_eth2_udp_rd_addr			<=12'd1;
					end
				//else
					//fifo_temp_eth2_udp_rd		<=1'b0;	//dummy
					//ram_eth2_udp_rd_addr			<=12'd1;
			end
			else if( (eth2_to_eth1_pre_udp_bcnt>16'd49) && (eth2_to_eth1_pre_udp_bcnt<1462)) begin //1458
				eth2_to_eth1_pre_udp_bcnt  <=eth2_to_eth1_pre_udp_bcnt+1'b1;
				//fifo_eth2_to_eth1_udp_din	<=eth2_UDP_Data[eth2_to_eth1_pre_udp_bcnt-16'd50];//?????? eth2_UDP_Data -> too much registers!!!!!!!!
				
				//fifo_temp_eth2_udp_rd		<=1'b1;
				fifo_eth2_to_eth1_udp_din	<=fifo_temp_eth2_udp_dout;
				//if(eth2_to_eth1_pre_udp_bcnt>16'd50)
					ram_eth2_udp_rd_addr			<=ram_eth2_udp_rd_addr+1'b1;
				
			end
			else if(eth2_to_eth1_pre_udp_bcnt==16'd1462) begin//1458
				fifo_temp_eth2_udp_rd		<=1'b0;
				ram_eth2_udp_rd_addr			<=12'd0;
				
				//fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[eth2_to_eth1_pre_udp_bcnt];
				fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[50];
				eth2_to_eth1_pre_udp_bcnt  <=eth2_to_eth1_pre_udp_bcnt+1'b1;
			end
			else if(eth2_to_eth1_pre_udp_bcnt==16'd1463) begin //1459
				//fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[eth2_to_eth1_pre_udp_bcnt];
				fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[51];
				eth2_to_eth1_pre_udp_bcnt  <=eth2_to_eth1_pre_udp_bcnt+1'b1;
			end
			else if(eth2_to_eth1_pre_udp_bcnt==16'd1464) begin
				//fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[eth2_to_eth1_pre_udp_bcnt];
				fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[52];
				eth2_to_eth1_pre_udp_bcnt  <=eth2_to_eth1_pre_udp_bcnt+1'b1;
			end
			else begin //if(eth2_to_eth1_pre_udp_bcnt==16'd1461) begin
				//fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[eth2_to_eth1_pre_udp_bcnt];
				fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[53];
				//eth2_to_eth1_pre_udp_bcnt  <=eth2_to_eth1_pre_udp_bcnt+1'b1;
				eth2_to_eth1_pre_udp <=4'd0;
			end
			/*else begin
				//fifo_eth2_to_eth1_udp_din	<=eth2_to_eth1_udp[eth2_to_eth1_pre_udp_bcnt];
				eth2_to_eth1_pre_udp <=4'd0;
			end*/
		end
		
		default: eth2_to_eth1_pre_udp<=4'd0;
	endcase
	
	

	//*******************************************************//
	//*Send packet from FIFO's										  *//
	//*******************************************************//
	case(eth1_tx_state)
		tx_IDLE: begin
			
			//Checking fifo's for transmission
			if(fifo_eth1_adc_cnt>=16'd1408) begin	//1408
				eth1_tx_state<=tx_ADC;
				//eth1_tx_state<=tx_ADC2;
			end
			//else if(fifo_eth1_arp_cnt>=16'd49) begin
			else if(fifo_eth1_arp_cnt>=16'd67) begin
				eth1_tx_state<=tx_ARP;
			end
			else if(fifo_eth1_icmp_cnt>=16'd105) begin
				eth1_tx_state<=tx_ICMP;
			end
			else if(fifo_eth2_to_eth1_arp_cnt>=16'd71) begin
				eth1_tx_state<=tx_ARP_from_eth2;
			end
			else if(fifo_eth2_to_eth1_icmp_cnt>=16'd109) begin
				eth1_tx_state<=tx_ICMP_from_eth2;
			end
			else if(fifo_eth2_to_eth1_udp_cnt>=16'd1465) begin //d1461
				eth1_tx_state<=tx_UDP_from_eth2;
			end
			

			eth1_crcen<=0;
			eth1_crcrst<=1;
			eth1_tx_en<=0;
			
			eth1_tx_i<=0;
			eth1_tx_j<=0;
			eth1_tx_p<=0;

			eth1_tx_igp<=4'd0;
			eth1_tx_pre<=4'd0;
			eth1_tx_temp<=8'd0;
		end
		
		/************************************************************/
		tx_ADC2: begin //test
			if(eth1_tx_j==16'd1408) begin	//561
				fifo_eth1_adc_rd	<=1'b0;
				eth1_tx_state		<=tx_IDLE;
			end
			else begin
				eth1_tx_j			<=eth1_tx_j+1'b1;
				fifo_eth1_adc_rd	<=1'b1;
			end
		end
		
		tx_ADC:  begin
			case(eth1_tx_pre)
				4'd0: begin	//Prepare packet
				
					pkt_cnt					<=pkt_cnt+1'b1;
					eth1_ADC_IPv4_ID		<=eth1_ADC_IPv4_ID+1'b1;	//Increment ID
				
					eth1_tx_adc_data[0]  <=8'h55;	//PREAMBULE  
					eth1_tx_adc_data[1]  <=8'h55;
					eth1_tx_adc_data[2]  <=8'h55;
					eth1_tx_adc_data[3]  <=8'h55;
					eth1_tx_adc_data[4]  <=8'h55;
					eth1_tx_adc_data[5]  <=8'h55;
					eth1_tx_adc_data[6]  <=8'h55;
					eth1_tx_adc_data[7]	<=8'hD5; 
					
					eth1_tx_adc_data[8]  <=8'hFF;	//Dest MAC  
					eth1_tx_adc_data[9]  <=8'hFF;
					eth1_tx_adc_data[10] <=8'hFF;
					eth1_tx_adc_data[11] <=8'hFF;
					eth1_tx_adc_data[12] <=8'hFF;
					eth1_tx_adc_data[13] <=8'hFF;	

					eth1_tx_adc_data[14] <=eth1_local_mac[47:40];		//Src MAC
					eth1_tx_adc_data[15] <=eth1_local_mac[39:32];
					eth1_tx_adc_data[16] <=eth1_local_mac[31:24];
					eth1_tx_adc_data[17] <=eth1_local_mac[23:16];
					eth1_tx_adc_data[18] <=eth1_local_mac[15:8];
					eth1_tx_adc_data[19] <=eth1_local_mac[7:0];	

					eth1_tx_adc_data[20] <=8'h08;				//TYPE 
					eth1_tx_adc_data[21] <=8'h00;
					
					eth1_tx_adc_data[22] <=8'h45;
					eth1_tx_adc_data[23] <=8'h00;
					
					eth1_tx_adc_data[24] <=8'h05;				//Len 20 + 8 + 1412 = 1440 -> 05A0
					eth1_tx_adc_data[25] <=8'hA0;	
					
					eth1_tx_adc_data[26] <=eth1_ADC_IPv4_ID[15:8];
					eth1_tx_adc_data[27] <=eth1_ADC_IPv4_ID[7:0];
					eth1_tx_adc_data[28] <=8'h40;				//Flags
					eth1_tx_adc_data[29] <=8'h00;
					eth1_tx_adc_data[30] <=8'h40;				//TTL
					eth1_tx_adc_data[31] <=8'h11;				//UDP
					
					eth1_tx_adc_data[32] <=8'h00;				//CRC IPv4
					eth1_tx_adc_data[33] <=8'h00;
					
					eth1_tx_adc_data[34] <=eth1_local_ip[31:24];	//My IP   
					eth1_tx_adc_data[35] <=eth1_local_ip[23:16];
					eth1_tx_adc_data[36] <=eth1_local_ip[15:8];
					eth1_tx_adc_data[37] <=eth1_local_ip[7:0];
					
					eth1_tx_adc_data[38] <=8'd192;				//Dst IP
					eth1_tx_adc_data[39] <=8'd168;
					eth1_tx_adc_data[40] <=8'd4;//8'd10;
					eth1_tx_adc_data[41] <=8'd1;
					
//					eth1_tx_adc_data[42] <=8'hBB;	//SP
//					eth1_tx_adc_data[43] <=8'h80;
//					
//					eth1_tx_adc_data[44] <=8'hBB;	//DP
//					eth1_tx_adc_data[45] <=8'h80;
					
					
					eth1_tx_adc_data[42] <=eth1_local_port[15:8];	//SP
					eth1_tx_adc_data[43] <=eth1_local_port[7:0];
					
					eth1_tx_adc_data[44] <=eth1_local_port[15:8];	//DP
					eth1_tx_adc_data[45] <=eth1_local_port[7:0];
					
					
					
					
					
					eth1_tx_adc_data[46] <=8'h05;	//Length = 8 + 4 + 1408
					eth1_tx_adc_data[47] <=8'h8C;
					
					eth1_tx_adc_data[48] <=8'h00;	//CRC
					eth1_tx_adc_data[49] <=8'h00;
					
					eth1_tx_adc_data[50] <=pkt_cnt[7:0];
					eth1_tx_adc_data[51] <=pkt_cnt[15:8];
					eth1_tx_adc_data[52] <=pkt_cnt[23:16];
					eth1_tx_adc_data[53] <=pkt_cnt[31:24];
					
					eth1_tx_pre<=4'd1;
				end
				
				4'd1: begin //Generate CRC for IPv4 
					if(eth1_adc_i==0) begin											
						eth1_adc_crc <={eth1_tx_adc_data[24],eth1_tx_adc_data[25]} + {eth1_tx_adc_data[22],eth1_tx_adc_data[23]} +
											{eth1_tx_adc_data[28],eth1_tx_adc_data[29]} + {eth1_tx_adc_data[26],eth1_tx_adc_data[27]} +
											{eth1_tx_adc_data[32],eth1_tx_adc_data[33]} + {eth1_tx_adc_data[30],eth1_tx_adc_data[31]} +
											{eth1_tx_adc_data[36],eth1_tx_adc_data[37]} + {eth1_tx_adc_data[34],eth1_tx_adc_data[35]} +
											{eth1_tx_adc_data[40],eth1_tx_adc_data[41]} + {eth1_tx_adc_data[38],eth1_tx_adc_data[39]};
						eth1_adc_i<=eth1_adc_i+1'b1;
					end
					else if(eth1_adc_i==1) begin
						eth1_adc_crc[15:0]<=eth1_adc_crc[31:16]+eth1_adc_crc[15:0];
						eth1_adc_i<=eth1_adc_i+1'b1;
					end
					else begin
						eth1_tx_adc_data[32]<=~eth1_adc_crc[15:8];
						eth1_tx_adc_data[33]<=~eth1_adc_crc[7:0];
						eth1_adc_i<=0;
						eth1_tx_pre<=4'd2;
					end
				end
				
				4'd2: begin //Send packet
					
					eth1_tx_en<=1;
					//CRC
					if(eth1_tx_j<=7) begin
						eth1_crcen<=0;  	//Disable module FCS
						eth1_crcrst<=1; 	//Reset value FCS 
					end
					else begin
						eth1_crcen<=1;		//Enable module FCS
						eth1_crcrst<=0; 
					end
					
					//****************************************************************
					//End of Packet
					if(eth1_tx_j==16'd1461) begin	//57
						
						//LSB part
						if(eth1_tx_i==0) begin

							if(eth1_tx_p>49)begin
								eth1_tx_i			<=eth1_tx_i+1'b1;
								//fifo_eth1_adc_rd	<=1'b1;
								eth1_txd[3:0] 		<=fifo_eth1_adc_dout[3:0];
								eth1_tx_temp		<=fifo_eth1_adc_dout;	
							end
							else begin
								eth1_tx_i			<=eth1_tx_i+1'b1;
								eth1_txd[3:0] 		<=eth1_tx_adc_data[eth1_tx_p][3:0];
							end
						end
						
						//MSB part
						else if(eth1_tx_i==1) begin
							if(eth1_tx_p>49)begin
								eth1_tx_i			<=0;
								//fifo_eth1_adc_rd	<=1'b0;
								eth1_txd[3:0] 		<=eth1_tx_temp[7:4];
								eth1_tx_state		<=tx_FCS;
							end
							else begin
								eth1_tx_i			<=0;
								eth1_tx_p			<=eth1_tx_p+1'b1;
								eth1_txd[3:0] 		<=eth1_tx_adc_data[eth1_tx_p][7:4];
							end
						end
					end
					
					
					
					//****************************************************************
					//Start of Packet
					else begin
						//LSB part
						if(eth1_tx_i==0) begin
						
							//Packet counter
//							if(eth1_tx_p==50)begin
//								eth1_tx_i			<=eth1_tx_i+1'b1;
//								eth1_txd[3:0] 		<=pkt_cnt[3:0];
//								eth1_tx_temp		<=pkt_cnt[7:0];
//							end
//							else if(eth1_tx_p==51)begin
//								eth1_tx_i			<=eth1_tx_i+1'b1;
//								eth1_txd[3:0] 		<=pkt_cnt[11:8];
//								eth1_tx_temp		<=pkt_cnt[15:8];
//							end
//							else if(eth1_tx_p==52)begin
//								eth1_tx_i			<=eth1_tx_i+1'b1;
//								eth1_txd[3:0] 		<=pkt_cnt[19:16];
//								eth1_tx_temp		<=pkt_cnt[23:16];
//							end
//							else if(eth1_tx_p==53)begin
//								eth1_tx_i			<=eth1_tx_i+1'b1;
//								eth1_txd[3:0] 		<=pkt_cnt[19:16];
//								eth1_tx_temp		<=pkt_cnt[23:16];
//							end
						
						
							//***********************************************************************//
							//else if(eth1_tx_p>49)begin
							if(eth1_tx_p>53)begin
								eth1_tx_i			<=eth1_tx_i+1'b1;
								fifo_eth1_adc_rd	<=1'b1;
								eth1_txd[3:0] 		<=fifo_eth1_adc_dout[3:0];
								eth1_tx_temp		<=fifo_eth1_adc_dout;
							end
							else begin
								eth1_tx_i			<=eth1_tx_i+1'b1;
								eth1_txd[3:0] 		<=eth1_tx_adc_data[eth1_tx_p][3:0];
								case(eth1_tx_p)
									//16'd49: fifo_eth1_adc_rd	<=1'b1;
									16'd53: fifo_eth1_adc_rd	<=1'b1;
								endcase
							end			
						end
						
						
						//MSB part
						else if(eth1_tx_i==1) begin
							
//							if(eth1_tx_p==50)begin
//								eth1_tx_i			<=0;		
//								eth1_txd[3:0] 		<=eth1_tx_temp[7:4];
//								eth1_tx_j			<=eth1_tx_j+1'b1;
//							end
//							else if(eth1_tx_p==51)begin
//								eth1_tx_i			<=0;		
//								eth1_txd[3:0] 		<=eth1_tx_temp[7:4];
//								eth1_tx_j			<=eth1_tx_j+1'b1;
//							end
//							else if(eth1_tx_p==52)begin
//								eth1_tx_i			<=0;		
//								eth1_txd[3:0] 		<=eth1_tx_temp[7:4];
//								eth1_tx_j			<=eth1_tx_j+1'b1;
//							end
//							else if(eth1_tx_p==53)begin
//								eth1_tx_i			<=0;		
//								eth1_txd[3:0] 		<=eth1_tx_temp[7:4];
//								eth1_tx_j			<=eth1_tx_j+1'b1;
//							end
							
							//***********************************************************************//
							if(eth1_tx_p>53)begin
								eth1_tx_i			<=0;		
								fifo_eth1_adc_rd	<=1'b0;
								eth1_txd[3:0] 		<=eth1_tx_temp[7:4];
								eth1_tx_j			<=eth1_tx_j+1'b1;
							end
							else begin
								eth1_tx_i			<=0;
								eth1_txd[3:0] 		<=eth1_tx_adc_data[eth1_tx_p][7:4];
								eth1_tx_p			<=eth1_tx_p+1'b1;
								eth1_tx_j			<=eth1_tx_j+1'b1;
								case(eth1_tx_p)
									16'd53: fifo_eth1_adc_rd	<=1'b0;
								endcase
							end
						end
					end
					
				end
				
				default: eth1_tx_pre<=4'd0;
			endcase
		end
		
		tx_ARP:  begin
			case(eth1_tx_pre)
				4'd0: begin
					fifo_eth1_arp_rd	<=1'b1;
					eth1_tx_pre<=4'd1;
				end
				
				4'd1: begin
					fifo_eth1_arp_rd	<=1'b0;
					eth1_tx_pre<=4'd2;
				end
				
				4'd2: begin
					eth1_tx_en<=1;
					//CRC
					if(eth1_tx_j<=7) begin
						eth1_crcen<=0;  	//Disable module FCS
						eth1_crcrst<=1; 	//Reset value FCS 
					end
					else begin
						eth1_crcen<=1;	//Enable module FCS
						eth1_crcrst<=0; 
					end
					
					//****************************************************************
					//End of Packet
					//if(eth1_tx_j==16'd49) begin//49
					if(eth1_tx_j==16'd67) begin//67
						if(eth1_tx_i==0) begin
							eth1_tx_i		<=eth1_tx_i+1'b1;
							eth1_txd[3:0] 	<=fifo_eth1_arp_dout[3:0];
							eth1_tx_temp	<=fifo_eth1_arp_dout;	
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i		<=0;
							eth1_txd[3:0] 	<=eth1_tx_temp[7:4];
							eth1_tx_state 	<=tx_FCS;
						end
					end
					
					//Start of Packet
					else begin
						if(eth1_tx_i==0) begin
							eth1_tx_i			<=eth1_tx_i+1'b1;
							fifo_eth1_arp_rd	<=1'b1;
							eth1_txd[3:0] 		<= fifo_eth1_arp_dout[3:0];
							eth1_tx_temp		<= fifo_eth1_arp_dout;	
							 
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i			<=0;
							eth1_tx_j			<=eth1_tx_j+1'b1;
							fifo_eth1_arp_rd	<=1'b0;
							eth1_txd[3:0] 		<= eth1_tx_temp[7:4];
						end
					end
				end
				
				default: eth1_tx_pre<=4'd0;
			endcase
		end
		
		tx_ICMP: begin
			case(eth1_tx_pre)
				//DUMMY
				4'd0: begin
					fifo_eth1_icmp_rd	<=1'b1;
					eth1_tx_pre<=4'd1;
				end
				
				//DUMMY
				4'd1: begin
					fifo_eth1_icmp_rd	<=1'b10;
					eth1_tx_pre<=4'd2;
				end
				
				//Send ICMP
				4'd2: begin
					eth1_tx_en<=1;
					//CRC
					if(eth1_tx_j<=7) begin
						eth1_crcen<=0;  	//Disable module FCS
						eth1_crcrst<=1; 	//Reset value FCS 
					end
					else begin
						eth1_crcen<=1;		//Enable module FCS
						eth1_crcrst<=0; 
					end
					
					//****************************************************************
					//End of Packet
					if(eth1_tx_j==16'd105) begin
						if(eth1_tx_i==0) begin
							eth1_tx_i		<=eth1_tx_i+1'b1;
							eth1_txd[3:0] 	<=fifo_eth1_icmp_dout[3:0];
							eth1_tx_temp	<=fifo_eth1_icmp_dout;	
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i		<=0;
							eth1_tx_j		<=0;
							eth1_txd[3:0] 	<=eth1_tx_temp[7:4];
							eth1_tx_state 	<=tx_FCS;
						end
					end
					
					//Start of Packet
					else begin
						if(eth1_tx_i==0) begin
							eth1_tx_i			<=eth1_tx_i+1'b1;
							fifo_eth1_icmp_rd	<=1'b1;
							eth1_txd[3:0] 		<=fifo_eth1_icmp_dout[3:0];
							eth1_tx_temp		<=fifo_eth1_icmp_dout;	
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i			<=0;
							eth1_tx_j			<=eth1_tx_j+1'b1;
							fifo_eth1_icmp_rd	<=1'b0;
							eth1_txd[3:0] 		<=eth1_tx_temp[7:4];
						end
					end
					
				end
				
				default:	eth1_tx_state<=tx_IDLE;
			endcase	
		end

		tx_ARP_from_eth2:  begin
			case(eth1_tx_pre)
				4'd0: begin	//dummy
					fifo_eth2_to_eth1_arp_rd	<=1'b1;
					eth1_tx_pre<=4'd1;
				end
				
				4'd1: begin	//dummy
					fifo_eth2_to_eth1_arp_rd	<=1'b0;
					eth1_tx_pre<=4'd2;
				end
				
				4'd2: begin //send
					eth1_tx_en<=1;
					
					//****************************************************************
					//End of Packet
					if(eth1_tx_j==16'd71) begin//53
						if(eth1_tx_i==0) begin
							eth1_tx_i		<=eth1_tx_i+1'b1;
							eth1_txd[3:0] 	<=fifo_eth2_to_eth1_arp_dout[3:0];
							eth1_tx_temp	<=fifo_eth2_to_eth1_arp_dout;	
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i		<=0;
							eth1_txd[3:0] 	<=eth1_tx_temp[7:4];
							eth1_tx_state 	<=tx_IGP;
						end
					end
					
					//Start of Packet
					else begin
						if(eth1_tx_i==0) begin
							eth1_tx_i						<=eth1_tx_i+1'b1;
							fifo_eth2_to_eth1_arp_rd	<=1'b1;
							eth1_txd[3:0] 					<=fifo_eth2_to_eth1_arp_dout[3:0];
							eth1_tx_temp					<=fifo_eth2_to_eth1_arp_dout;	 
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i						<=0;
							eth1_tx_j						<=eth1_tx_j+1'b1;
							fifo_eth2_to_eth1_arp_rd	<=1'b0;
							eth1_txd[3:0] 					<=eth1_tx_temp[7:4];
						end
					end
				end
				
				default: eth1_tx_state<=tx_IDLE;
			endcase		
		end
		
		tx_ICMP_from_eth2: begin
			case(eth1_tx_pre)
				4'd0: begin	//dummy
					fifo_eth2_to_eth1_icmp_rd	<=1'b1;
					eth1_tx_pre<=4'd1;
				end
				
				4'd1: begin	//dummy
					fifo_eth2_to_eth1_icmp_rd	<=1'b0;
					eth1_tx_pre<=4'd2;
				end
				
				4'd2: begin //send
					eth1_tx_en<=1;
					
					//****************************************************************
					//End of Packet
					if(eth1_tx_j==16'd109) begin
						if(eth1_tx_i==0) begin
							eth1_tx_i		<=eth1_tx_i+1'b1;
							eth1_txd[3:0] 	<=fifo_eth2_to_eth1_icmp_dout[3:0];
							eth1_tx_temp	<=fifo_eth2_to_eth1_icmp_dout;	
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i		<=0;
							eth1_txd[3:0] 	<=eth1_tx_temp[7:4];
							eth1_tx_state 	<=tx_IGP;
						end
					end
					
					//Start of Packet
					else begin
						if(eth1_tx_i==0) begin
							eth1_tx_i						<=eth1_tx_i+1'b1;
							fifo_eth2_to_eth1_icmp_rd	<=1'b1;
							eth1_txd[3:0] 					<=fifo_eth2_to_eth1_icmp_dout[3:0];
							eth1_tx_temp					<=fifo_eth2_to_eth1_icmp_dout;	 
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i						<=0;
							eth1_tx_j						<=eth1_tx_j+1'b1;
							fifo_eth2_to_eth1_icmp_rd	<=1'b0;
							eth1_txd[3:0] 					<=eth1_tx_temp[7:4];
						end
					end
				end
				
				default: eth1_tx_state<=tx_IDLE;
			endcase	
		end
		
		tx_UDP_from_eth2:  begin
			case(eth1_tx_pre)
				4'd0: begin	//dummy
					fifo_eth2_to_eth1_udp_rd	<=1'b1;
					eth1_tx_pre<=4'd1;
				end
				
				4'd1: begin	//dummy
					fifo_eth2_to_eth1_udp_rd	<=1'b0;
					eth1_tx_pre<=4'd2;
				end
				
				4'd2: begin //send
					eth1_tx_en<=1;
					
					//****************************************************************
					//End of Packet
					if(eth1_tx_j==16'd1465) begin //d1461
						if(eth1_tx_i==0) begin
							eth1_tx_i		<=eth1_tx_i+1'b1;
							eth1_txd[3:0] 	<=fifo_eth2_to_eth1_udp_dout[3:0];
							eth1_tx_temp	<=fifo_eth2_to_eth1_udp_dout;	
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i		<=0;
							eth1_txd[3:0] 	<=eth1_tx_temp[7:4];
							eth1_tx_state 	<=tx_IGP;
						end
					end
					
					//Start of Packet
					else begin
						if(eth1_tx_i==0) begin
							eth1_tx_i						<=eth1_tx_i+1'b1;
							fifo_eth2_to_eth1_udp_rd	<=1'b1;
							eth1_txd[3:0] 					<=fifo_eth2_to_eth1_udp_dout[3:0];
							eth1_tx_temp					<=fifo_eth2_to_eth1_udp_dout;	 
						end
						else if(eth1_tx_i==1) begin
							eth1_tx_i						<=0;
							eth1_tx_j						<=eth1_tx_j+1'b1;
							fifo_eth2_to_eth1_udp_rd	<=1'b0;
							eth1_txd[3:0] 					<=eth1_tx_temp[7:4];
						end
					end
				end
				
				default: eth1_tx_state<=tx_IDLE;
			endcase	
		end
		
		tx_FCS:  begin 
			eth1_crcen<=1'b0;
			if(eth1_tx_i==0)  begin
				eth1_txd[3:0] <= {~eth1_crcnext[28], ~eth1_crcnext[29], ~eth1_crcnext[30], ~eth1_crcnext[31]};
				eth1_tx_i<=eth1_tx_i+1;
			end
			else if(eth1_tx_i==1) begin
				eth1_txd[3:0]<={~eth1_fcs[24], ~eth1_fcs[25], ~eth1_fcs[26], ~eth1_fcs[27]};
				eth1_tx_i<=eth1_tx_i+1;
			end
			else if(eth1_tx_i==2) begin
				eth1_txd[3:0]<={~eth1_fcs[20], ~eth1_fcs[21], ~eth1_fcs[22], ~eth1_fcs[23]};
				eth1_tx_i<=eth1_tx_i+1;
			end
			else if(eth1_tx_i==3) begin
				eth1_txd[3:0]<={~eth1_fcs[16], ~eth1_fcs[17], ~eth1_fcs[18], ~eth1_fcs[19]};
				eth1_tx_i<=eth1_tx_i+1;
			end
			else if(eth1_tx_i==4) begin
				eth1_txd[3:0]<={~eth1_fcs[12], ~eth1_fcs[13], ~eth1_fcs[14], ~eth1_fcs[15]};
				eth1_tx_i<=eth1_tx_i+1;
			end
			else if(eth1_tx_i==5) begin
				eth1_txd[3:0]<={~eth1_fcs[8], ~eth1_fcs[9], ~eth1_fcs[10], ~eth1_fcs[11]};
				eth1_tx_i<=eth1_tx_i+1;
			end
			else if(eth1_tx_i==6) begin
				eth1_txd[3:0]<={~eth1_fcs[4], ~eth1_fcs[5], ~eth1_fcs[6], ~eth1_fcs[7]};
				eth1_tx_i<=eth1_tx_i+1;
			end
			else if(eth1_tx_i==7) begin
				eth1_txd[3:0]<={~eth1_fcs[0], ~eth1_fcs[1], ~eth1_fcs[2], ~eth1_fcs[3]};
				eth1_tx_i<=eth1_tx_i+1;
			end
			else  begin
				eth1_tx_en<=0;
				eth1_txd<=4'h00;
				eth1_tx_i<=0;
				eth1_tx_j<=0;
				eth1_tx_state<=tx_IGP;
			end
		end

		tx_IGP:  begin
			eth1_tx_en<=1'b0;
			eth1_txd<=4'h00;
			if(eth1_tx_igp<4'd12)
				eth1_tx_igp<=eth1_tx_igp+1'b1;
			else
				eth1_tx_state<=tx_IDLE;
		end
		
		default: eth1_tx_state<=tx_IDLE;
	endcase				
end



//***********************************************************************************************************************//
//***********************************************************************************************************************//
//***********************************************************************************************************************//
//*ETHERNET2: RECEIVE PACKET'S																														*//
//***********************************************************************************************************************//
//***********************************************************************************************************************//
//***********************************************************************************************************************//


reg eth2_byte_sig_t;
reg eth2_byte_sig;
reg eth2_byte_rxdv_t;
reg eth2_byte_rxdv;

reg [7:0] eth2_datain;
reg [7:0] eth2_mybyte;
reg eth2_sig;

reg [7:0]  	eth2_rx_state;
reg [15:0] 	eth2_rx_byte_cnt;
reg [7:0]  	eth2_rx_alg_state;

reg [15:0] 	eth2_rx_byte_cnt2;

reg		  	eth2_arp_ok;
reg			eth2_udp_ok;
reg		  	eth2_icmp_ok;

reg [111:0] eth2_layer;
reg [47:0]  eth2_MAC_dst;
reg [47:0]  eth2_MAC_src;
reg [15:0]  eth2_type;

reg [159:0] eth2_IPv4_layer;
reg [7:0]   eth2_IPv4_vh;
reg [7:0]   eth2_IPv4_dscp;
reg [15:0]  eth2_IPv4_tlen;
reg [15:0]  eth2_IPv4_id;
reg [15:0]  eth2_IPv4_flags;
reg [7:0]   eth2_IPv4_ttl;
reg [7:0]	eth2_IPv4_type;
reg [15:0]  eth2_IPv4_crc;
reg [31:0]	eth2_IPv4_IP_src;
reg [31:0]	eth2_IPv4_IP_dst;						

reg [223:0] eth2_ARP_layer;
reg [47:0]  eth2_ARP_MAC_src;
reg [31:0]  eth2_ARP_IP_src;
reg [31:0]  eth2_ARP_IP_dst;
reg [31:0]  eth2_ARP_FCS;

reg [511:0] eth2_ICMP_layer;
reg [15:0]  eth2_ICMP_tlen;
reg [31:0]  eth2_ICMP_FCS;
reg [15:0]  eth2_UDP_tlen;

reg [63:0]  eth2_UDP_layer; //8 bytes
reg [31:0]  eth2_UDP_FCS;

reg			fifo_temp_eth2_udp_rd;
reg			fifo_temp_eth2_udp_wr;
reg	[7:0]	fifo_temp_eth2_udp_din;
wire	[7:0]	fifo_temp_eth2_udp_dout;
wire			fifo_temp_eth2_udp_empty;
wire			fifo_temp_eth2_udp_full;
wire	[15:0]fifo_temp_eth2_udp_cnt;
wire	[15:0]fifo_temp_eth2_udp_tail;	
wire	[15:0]fifo_temp_eth2_udp_head;	

reg			fifo_wr_clk;
reg			fifo_rd_clk;

reg   [11:0]ram_eth2_udp_wr_addr;
reg   [11:0]ram_eth2_udp_rd_addr;

initial begin
	eth2_arp_ok<=1'b0;
	eth2_udp_ok<=1'b0;
	eth2_icmp_ok<=1'b0;
	fifo_temp_eth2_udp_wr<=1'b0;
	fifo_temp_eth2_udp_rd<=1'b0;
	
	fifo_wr_clk<=1'b0;
	fifo_rd_clk<=1'b0;
	
	ram_eth2_udp_wr_addr<=12'd0;
	ram_eth2_udp_rd_addr<=12'd0;
end

//reg [7:0]   eth2_UDP_Data[1407:0];//??????? too much registers 1408*8 = 11264

//********************************************************//
//*Put UDP Packet in FIFO for temp								*//
//********************************************************//
/*
fifo2 # (8,4096) fifo_temp_eth2_udp
(
	.clk		(eth1_tx_clk),
	.rst		(1'b0),
	.rd		(fifo_temp_eth2_udp_rd & !fifo_temp_eth2_udp_empty),
	.wr		(fifo_temp_eth2_udp_wr & !fifo_temp_eth2_udp_full),
	.din		(fifo_temp_eth2_udp_din),
	.dout		(fifo_temp_eth2_udp_dout),
	.empty	(fifo_temp_eth2_udp_empty),
	.full		(fifo_temp_eth2_udp_full),
	.cnt		(fifo_temp_eth2_udp_cnt)
);*/



always @(posedge eth1_tx_clk) begin
	fifo_rd_clk <= ~fifo_rd_clk;
end

always @(posedge eth2_rx_clk) begin
	fifo_wr_clk <= ~fifo_wr_clk;
end

/*
fifo_asy fifo_asy_eth2_udp
(
	.rst		(1'b0),
	.cnt		(fifo_temp_eth2_udp_cnt),
	
	//write
	.wr_clk	(fifo_wr_clk),//eth2_rx_clk
	.wr		(fifo_temp_eth2_udp_wr & !fifo_temp_eth2_udp_full),
	.din		(fifo_temp_eth2_udp_din),
	.full		(fifo_temp_eth2_udp_full),
	
	//read
	.rd_clk	(fifo_rd_clk),//eth1_tx_clk
	.rd		(fifo_temp_eth2_udp_rd & !fifo_temp_eth2_udp_empty),
	.dout		(fifo_temp_eth2_udp_dout),
	.empty	(fifo_temp_eth2_udp_empty)	
);
*/



/*
fifo_asy2 fifo_asy_eth2_udp 
(
	.reset		(1'b0),
	.rd			(fifo_temp_eth2_udp_rd),
	.wr			(fifo_temp_eth2_udp_wr),
	.rd_clk		(eth1_tx_clk), 
	.wr_clk		(fifo_wr_clk), 
	.data_in		(fifo_temp_eth2_udp_din), 
	.data_out	(fifo_temp_eth2_udp_dout), 
	.rd_empty	(fifo_temp_eth2_udp_empty), 
	.wr_full		(fifo_temp_eth2_udp_full), 
	.cnt			(fifo_temp_eth2_udp_cnt)
);*/



/*
fifo3 # (8,4096) fifo_temp_eth2_udp
(
	.rd_clk	(eth1_tx_clk & fifo_temp_eth2_udp_rd),
	.wr_clk	(fifo_wr_clk & fifo_temp_eth2_udp_wr),
	.rst		(1'b0),
	.rd_en	(fifo_temp_eth2_udp_rd & !fifo_temp_eth2_udp_empty),
	.wr_en	(fifo_temp_eth2_udp_wr & !fifo_temp_eth2_udp_full),
	.din		(fifo_temp_eth2_udp_din),
	.dout		(fifo_temp_eth2_udp_dout),
	.empty	(fifo_temp_eth2_udp_empty),
	.full		(fifo_temp_eth2_udp_full),
	.cnt		(fifo_temp_eth2_udp_cnt),
	
	.tail_cnt(fifo_temp_eth2_udp_tail),
	.head_cnt(fifo_temp_eth2_udp_head)
);*/


ram_dual ram_eth2_udp
(
	.wr_clk(fifo_wr_clk), 
	.rd_clk(eth1_tx_clk),
	
	.wr_addr(ram_eth2_udp_wr_addr),
	.rd_addr(ram_eth2_udp_rd_addr), 
	
	.wr(fifo_temp_eth2_udp_wr), 
	.rd(fifo_temp_eth2_udp_rd),
 
	.din(fifo_temp_eth2_udp_din),
	.dout(fifo_temp_eth2_udp_dout)
);





assign fifo_temp_eth2_udp_cnt_s  = fifo_temp_eth2_udp_cnt;
assign fifo_temp_eth2_udp_wr_s   = fifo_temp_eth2_udp_wr;
assign fifo_temp_eth2_udp_din_s  = fifo_temp_eth2_udp_din;
assign fifo_temp_eth2_udp_full_s = fifo_temp_eth2_udp_full;

assign fifo_temp_eth2_udp_rd_s    = fifo_temp_eth2_udp_rd;
assign fifo_temp_eth2_udp_empty_s = fifo_temp_eth2_udp_empty;
assign fifo_temp_eth2_udp_dout_s	 = fifo_temp_eth2_udp_dout;

assign fifo_temp_eth2_udp_tail_s	 = fifo_temp_eth2_udp_tail;
assign fifo_temp_eth2_udp_head_s	 = fifo_temp_eth2_udp_head;

assign ram_eth2_udp_wr_addr_s		 = ram_eth2_udp_wr_addr;
assign ram_eth2_udp_rd_addr_s		 = ram_eth2_udp_rd_addr;


//4bit->byte
always @(posedge eth2_rx_clk) begin
	if(eth2_rxdv)	begin
		eth2_mybyte	<={eth2_rxd,eth2_mybyte[7:4]};
		eth2_sig		<=~eth2_sig;
	end
	else begin
		eth2_mybyte	<=eth2_mybyte;
		eth2_sig		<=1'b0;
	end
end

//4bit->byte
always @(posedge eth2_rx_clk) begin
	if(eth2_sig&&eth2_rxdv) begin
		eth2_datain	<={eth2_rxd,eth2_mybyte[7:4]};
	end
	else if(!eth2_rxdv)
		eth2_datain	<=8'd0;
end

//generate byte rxdv signal
always @(posedge eth2_rx_clk) begin
	eth2_byte_sig_t	<=eth2_sig;
	eth2_byte_sig		<=eth2_byte_sig_t;
	eth2_byte_rxdv_t	<=eth2_rxdv;
	eth2_byte_rxdv	   <=eth2_byte_rxdv_t;	
end

//Eth2 RX
always@(posedge eth2_rx_clk) begin

	eth2_rx_cnt<=eth2_rx_cnt+1'b1;

	case(eth2_rx_state)
		rx_IDLE:       begin
			//eth2_arp_ok			<=1'b0;
			//eth2_udp_ok			<=1'b0;
			//eth2_icmp_ok		<=1'b0;
			
			event_eth2_to_eth1_arp_wr  <=1'b0;
			event_eth2_to_eth1_icmp_wr <=1'b0;
			event_eth2_to_eth1_udp_wr  <=1'b0;
			
			
			fifo_temp_eth2_udp_wr<=1'b0;
			ram_eth2_udp_wr_addr<=12'd0;
			
			eth2_rx_alg_state	<=8'd0;
			eth2_rx_byte_cnt	<=16'd0;
			
			if(eth2_byte_rxdv && !eth2_byte_sig)  begin
				if(eth2_datain==8'h55)
					eth2_rx_state<=rx_PREAMBULE; 
			end
		end
		
		rx_PREAMBULE:  begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin				  
				case (eth2_rx_alg_state)
					8'd0: begin
						if(eth2_datain!=8'h55) begin 
							eth2_rx_state<=rx_IDLE;
						end
						else begin 
							if(eth2_rx_byte_cnt<16'd5)
								eth2_rx_byte_cnt<=eth2_rx_byte_cnt+1'b1;
							else begin
								eth2_rx_byte_cnt<=16'd0;
								eth2_rx_alg_state<=8'd1;
							end
						end
					end
					
					8'd1: begin
						if(eth2_datain==8'hd5) begin
							eth2_rx_state<=rx_ETH_layer;
							eth2_rx_alg_state<=8'd0;
						end
						else
							eth2_rx_state<=rx_IDLE;
					end
				endcase
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		rx_ETH_layer:  begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt<16'd13) begin
					eth2_layer	<={eth2_layer[103:0],eth2_datain};
					eth2_rx_byte_cnt	<=eth2_rx_byte_cnt+1'b1;
				end
				else begin	
					eth2_layer		<={eth2_layer[103:0],eth2_datain};
					eth2_MAC_dst	<=eth2_layer[103:56];
					eth2_MAC_src	<=eth2_layer[55:8];
					eth2_type		<={eth2_layer[7:0],eth2_datain};
					
					case ({eth2_layer[7:0],eth2_datain}) 						//Ethernet Type Protocol
						16'h0800: begin eth2_rx_state<=rx_IPv4_layer; end 	//Receive IPv4
						16'h0806: begin eth2_rx_state<=rx_ARP_layer;  end	//Receive ARP
						default:  begin eth2_rx_state<=rx_IDLE; 		 end
					endcase
					eth2_rx_byte_cnt <=16'd0;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		rx_IPv4_layer: begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt<16'd19) begin
					eth2_IPv4_layer	<={eth2_IPv4_layer[151:0],eth2_datain[7:0]};
					eth2_rx_byte_cnt 	<= eth2_rx_byte_cnt+1'b1;
				end
				else begin
					eth2_IPv4_layer	<={eth2_IPv4_layer[151:0],eth2_datain[7:0]};
					case (eth2_IPv4_layer[79:72])								//IPv4 Type Protocol
						8'd01: begin 												//ICMP
							eth2_ICMP_tlen	<=eth2_IPv4_layer[135:120] - 16'd20; 
							eth2_rx_state  <=rx_ICMP_layer; end	
						8'd17: begin 												//UDP
							eth2_UDP_tlen  <=eth2_IPv4_layer[135:120] - 16'd20; 
							eth2_rx_state  <=rx_UDP_layer;  end
						default: eth2_rx_state<=rx_IDLE;
					endcase
					eth2_rx_byte_cnt <=16'd0;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		rx_ICMP_layer: begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt<(eth2_ICMP_tlen-1'b1)) begin
					eth2_ICMP_layer	<={eth2_ICMP_layer[503:0],eth2_datain};
					eth2_rx_byte_cnt	<=eth2_rx_byte_cnt+1'b1;
				end
				else begin
					eth2_ICMP_layer	<={eth2_ICMP_layer[503:0],eth2_datain};
					eth2_rx_byte_cnt	<=16'd0;
					eth2_rx_state		<=rx_ICMP_FCS;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		rx_ICMP_FCS:   begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt < 16'd3) begin
					eth2_ICMP_FCS		<={eth2_ICMP_FCS[23:0],eth2_datain};
					eth2_rx_byte_cnt	<= eth2_rx_byte_cnt+1'b1;
				end
				else begin
					eth2_ICMP_FCS		<={eth2_ICMP_FCS[23:0],eth2_datain};
					//eth2_icmp_ok		<=1'b1;
					event_eth2_to_eth1_icmp_wr <=1'b1;
					eth2_rx_byte_cnt	<=16'd0;
					//eth2_rx_state		<=rx_DELAY;
					eth2_rx_state	<=rx_IDLE;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end

		rx_ARP_layer:  begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt<16'd27) begin
					eth2_ARP_layer		<={eth2_ARP_layer[215:0],eth2_datain};
					eth2_rx_byte_cnt	<=eth2_rx_byte_cnt+1'b1;
				end
				else begin
					eth2_ARP_layer		<={eth2_ARP_layer[215:0],eth2_datain};//
					eth2_ARP_MAC_src	<=eth2_ARP_layer[151:104];
					eth2_ARP_IP_src	<=eth2_ARP_layer[103:72];	
					eth2_ARP_IP_dst	<={eth2_ARP_layer[23:0],eth2_datain};	
					eth2_rx_byte_cnt	<=16'd0;
					eth2_rx_state		<=rx_ARP_TRL;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		rx_ARP_TRL:    begin //Receive ARP padding 18bytes: 0x00
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt < 16'd17) begin
					eth2_rx_byte_cnt	<=eth2_rx_byte_cnt+1'b1;
				end
				else begin
					eth2_rx_state		<=rx_ARP_FCS;
					eth2_rx_byte_cnt	<=16'd0;
				end
			end
		end
		
		rx_ARP_FCS:    begin //Receive ethernet frame check sequence for retransmit to Ethernet Port 1
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt < 16'd3) begin
					eth2_ARP_FCS			<={eth2_ARP_FCS[23:0],eth2_datain};
					eth2_rx_byte_cnt		<=eth2_rx_byte_cnt+1'b1;
				end
				else begin
					eth2_ARP_FCS			<={eth2_ARP_FCS[23:0],eth2_datain};
					//eth2_arp_ok				<=1'b1;
					event_eth2_to_eth1_arp_wr <=1'b1;
					
					eth2_rx_byte_cnt		<=16'd0;
					//eth2_rx_state			<=rx_DELAY;
					eth2_rx_state			<=rx_IDLE;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		rx_UDP_layer:  begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt<7) begin
					eth2_UDP_layer		<={eth2_UDP_layer[55:0],eth2_datain};
					eth2_rx_byte_cnt	<=eth2_rx_byte_cnt+1'b1;
				end
				else begin
					eth2_UDP_layer		<={eth2_UDP_layer[55:0],eth2_datain};
					eth2_rx_byte_cnt	<=16'd0;
					eth2_rx_state		<=rx_UDP_Data;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		rx_UDP_Data:  begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				if(eth2_rx_byte_cnt  < (eth2_UDP_tlen-9) ) begin
					//eth2_UDP_Data	   <={eth2_UDP_Data[11255:0],eth2_datain};
					//eth2_UDP_Data[eth2_rx_byte_cnt] <= eth2_datain;!!!!!
					
					fifo_temp_eth2_udp_wr	<=1'b1;
					fifo_temp_eth2_udp_din	<=eth2_datain;
					eth2_rx_byte_cnt			<=eth2_rx_byte_cnt+1'b1;
					ram_eth2_udp_wr_addr		<=ram_eth2_udp_wr_addr+1'b1;
				end
				else begin
					//eth2_UDP_Data		<={eth2_UDP_Data[11255:0],eth2_datain};
					//eth2_UDP_Data[eth2_rx_byte_cnt] <= eth2_datain;
					
					fifo_temp_eth2_udp_din	<=eth2_datain;
					eth2_rx_state				<=rx_UDP_FCS;
					eth2_rx_byte_cnt			<=16'd0;
					ram_eth2_udp_wr_addr		<=ram_eth2_udp_wr_addr+1'b1;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		rx_UDP_FCS:   begin
			if(eth2_byte_rxdv && !eth2_byte_sig) begin
				
				fifo_temp_eth2_udp_wr   <=1'b0;
				ram_eth2_udp_wr_addr		<=12'd0;
			
			
				if(eth2_rx_byte_cnt < 16'd3) begin
					eth2_UDP_FCS		<={eth2_UDP_FCS[23:0],eth2_datain};
					eth2_rx_byte_cnt	<=eth2_rx_byte_cnt+1'b1;
				end
				else begin
					eth2_UDP_FCS		<={eth2_UDP_FCS[23:0],eth2_datain};
					//eth2_udp_ok			<=1'b1;
					event_eth2_to_eth1_udp_wr	<=1'b1;
					eth2_rx_byte_cnt	<=16'd0;
					eth2_rx_state		<=rx_IDLE;
				end
			end
			else if(!eth2_byte_rxdv) eth2_rx_state<=rx_IDLE;
		end
		
		
		rx_DELAY: 	  begin
			if(eth2_rx_byte_cnt < 16'd9) begin
				eth2_rx_byte_cnt	<=eth2_rx_byte_cnt+1'b1;
			end
			else begin
			   eth2_rx_byte_cnt	<=16'd0;
				eth2_rx_state<=rx_IDLE;
			end
		end
	
		default: eth2_rx_state<=rx_IDLE;
	endcase
end



	
//***********************************************************************************************************************//
//***********************************************************************************************************************//
//***********************************************************************************************************************//
//*ETHERNET2: PREPARE AND TRANSMIT PACKET'S																										*//
//***********************************************************************************************************************//
//***********************************************************************************************************************//
//***********************************************************************************************************************//
reg			fifo_eth1_to_eth2_arp_rd;
reg			fifo_eth1_to_eth2_arp_wr;
reg	[7:0]	fifo_eth1_to_eth2_arp_din;
wire	[7:0]	fifo_eth1_to_eth2_arp_dout;
wire			fifo_eth1_to_eth2_arp_empty;
wire			fifo_eth1_to_eth2_arp_full;
wire	[15:0]fifo_eth1_to_eth2_arp_cnt;

reg			fifo_eth1_to_eth2_icmp_rd;
reg			fifo_eth1_to_eth2_icmp_wr;
reg	[7:0]	fifo_eth1_to_eth2_icmp_din;
wire	[7:0]	fifo_eth1_to_eth2_icmp_dout;
wire			fifo_eth1_to_eth2_icmp_empty;
wire			fifo_eth1_to_eth2_icmp_full;
wire	[15:0]fifo_eth1_to_eth2_icmp_cnt;

reg 	[7:0] eth2_tx_state;
reg 	[3:0] eth1_to_eth2_pre_arp;
reg 	[3:0] eth1_to_eth2_pre_icmp;

reg 	[15:0]eth2_tx_i;
reg 	[15:0]eth2_tx_j;
reg 	[15:0]eth2_tx_p;

reg 	[15:0]i2;
reg 	[15:0]j2;

reg 	[15:0]eth1_to_eth2_pre_arp_bcnt;
reg 	[15:0]eth1_to_eth2_pre_icmp_bcnt;

reg 	[7:0] eth1_to_eth2_arp[71:0]; 
reg 	[7:0] eth1_to_eth2_icmp[109:0]; 

reg 	[3:0] eth2_tx_igp;
reg 	[3:0] eth2_tx_pre;
reg 	[7:0] eth2_tx_temp;

initial begin
	eth2_tx_state				 	<=tx_IDLE;
	fifo_eth1_to_eth2_arp_wr 	<=1'b0;
	fifo_eth1_to_eth2_icmp_wr 	<=1'b0;
	eth1_to_eth2_pre_arp 	 	<=4'b0;
	eth1_to_eth2_pre_icmp 	 	<=4'b0;
	eth2_tx_i<=16'd0;
	eth2_tx_j<=16'd0;
	eth2_tx_p<=16'd0;
	eth1_to_eth2_pre_arp_bcnt	<=16'd0;
	eth1_to_eth2_pre_icmp_bcnt <=16'd0;
	
	i2<=16'd0;
	j2<=16'd0;
end

//********************************************************//
//*Put ARP Packet in FIFO for transmission						*//
//********************************************************//
fifo2 # (8,512) fifo_eth1_to_eth2_arp
(
	.clk		(eth2_tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth1_to_eth2_arp_rd & !fifo_eth1_to_eth2_arp_empty),
	.wr		(fifo_eth1_to_eth2_arp_wr & !fifo_eth1_to_eth2_arp_full),
	.din		(fifo_eth1_to_eth2_arp_din),
	.dout		(fifo_eth1_to_eth2_arp_dout),
	.empty	(fifo_eth1_to_eth2_arp_empty),
	.full		(fifo_eth1_to_eth2_arp_full),
	.cnt		(fifo_eth1_to_eth2_arp_cnt)
);

//********************************************************//
//*Put ICMP Packet in FIFO for transmission						*//
//********************************************************//
fifo2 # (8,512) fifo_eth1_to_eth2_icmp
(
	.clk		(eth2_tx_clk),
	.rst		(1'b0),
	.rd		(fifo_eth1_to_eth2_icmp_rd & !fifo_eth1_to_eth2_icmp_empty),
	.wr		(fifo_eth1_to_eth2_icmp_wr & !fifo_eth1_to_eth2_icmp_full),
	.din		(fifo_eth1_to_eth2_icmp_din),
	.dout		(fifo_eth1_to_eth2_icmp_dout),
	.empty	(fifo_eth1_to_eth2_icmp_empty),
	.full		(fifo_eth1_to_eth2_icmp_full),
	.cnt		(fifo_eth1_to_eth2_icmp_cnt)
);

//CRC32 Module
wire	[31:0]eth2_crcnext;
wire	[31:0]eth2_fcs;
reg			eth2_crcrst;
reg			eth2_crcen;

//input 4-bit data
crc eth2_crc32(
	.clk(eth2_tx_clk),
	.reset(eth2_crcrst),
	.enable(eth2_crcen),
	.data(eth2_txd),
	.crc(eth2_fcs),
	.crc_next(eth2_crcnext)
);


reg eth2_tx_arp_s;
reg eth2_tx_icmp_s;

//Eth2 TX
always@(posedge eth2_tx_clk) begin

	eth2_tx_cnt		<=eth2_tx_cnt+1'b1;
	
	eth2_tx_arp_s  <= event_eth1_to_eth2_arp_ok;
	eth2_tx_icmp_s <= event_eth1_to_eth2_icmp_ok;
	
	
	//*******************************************************//
	//Prepare ARP packet from Eth1 to Eth2, and put in FIFO *//
	//*******************************************************//
	case(eth1_to_eth2_pre_arp)
		4'd0: begin //Idle
			fifo_eth1_to_eth2_arp_wr	<=1'b0;
			eth1_to_eth2_pre_arp_bcnt	<=16'd0;
			event_eth1_to_eth2_arp_rd	<=1'b0;
			
			if(event_eth1_to_eth2_arp_ok) begin
				eth1_to_eth2_pre_arp<=4'd1;
				event_eth1_to_eth2_arp_rd<=1'b1;
			end
		end
		
		4'd1: begin //Fill
			eth1_to_eth2_arp[0]  <=8'h55;               //PREAMBULE  
			eth1_to_eth2_arp[1]  <=8'h55;
			eth1_to_eth2_arp[2]  <=8'h55;
			eth1_to_eth2_arp[3]  <=8'h55;
			eth1_to_eth2_arp[4]  <=8'h55;
			eth1_to_eth2_arp[5]  <=8'h55;
			eth1_to_eth2_arp[6]  <=8'h55;
			eth1_to_eth2_arp[7]	<=8'hD5; 

			eth1_to_eth2_arp[8]  <=eth1_layer[111:104];//Dest MAC  
			eth1_to_eth2_arp[9]  <=eth1_layer[103:96];
			eth1_to_eth2_arp[10] <=eth1_layer[95:88];
			eth1_to_eth2_arp[11] <=eth1_layer[87:80];
			eth1_to_eth2_arp[12] <=eth1_layer[79:72];
			eth1_to_eth2_arp[13] <=eth1_layer[71:64];
			
			eth1_to_eth2_arp[14] <=eth1_layer[63:56];//Src MAC
			eth1_to_eth2_arp[15] <=eth1_layer[55:48];
			eth1_to_eth2_arp[16] <=eth1_layer[47:40];
			eth1_to_eth2_arp[17] <=eth1_layer[39:32];
			eth1_to_eth2_arp[18] <=eth1_layer[31:24];
			eth1_to_eth2_arp[19] <=eth1_layer[23:16];
			
			eth1_to_eth2_arp[20] <=eth1_layer[15:8];//TYPE 
			eth1_to_eth2_arp[21] <=eth1_layer[7:0];
			
			eth1_to_eth2_arp[22] <=eth1_ARP_layer[223:216];//HType
			eth1_to_eth2_arp[23] <=eth1_ARP_layer[215:208];
			eth1_to_eth2_arp[24] <=eth1_ARP_layer[207:200];//IPv4
			eth1_to_eth2_arp[25] <=eth1_ARP_layer[199:192];
			eth1_to_eth2_arp[26] <=eth1_ARP_layer[191:184];//Hsz
			eth1_to_eth2_arp[27] <=eth1_ARP_layer[183:176];//Psz
			eth1_to_eth2_arp[28] <=eth1_ARP_layer[175:168];//Opcode 0x0002 reply
			eth1_to_eth2_arp[29] <=eth1_ARP_layer[167:160];
			
			eth1_to_eth2_arp[30] <=eth1_ARP_layer[159:152];//My MAC  
			eth1_to_eth2_arp[31] <=eth1_ARP_layer[151:144];
			eth1_to_eth2_arp[32] <=eth1_ARP_layer[143:136];
			eth1_to_eth2_arp[33] <=eth1_ARP_layer[135:128];
			eth1_to_eth2_arp[34] <=eth1_ARP_layer[127:120];
			eth1_to_eth2_arp[35] <=eth1_ARP_layer[119:112];
			
			eth1_to_eth2_arp[36] <=eth1_ARP_layer[111:104];//My IP   
			eth1_to_eth2_arp[37] <=eth1_ARP_layer[103:96];
			eth1_to_eth2_arp[38] <=eth1_ARP_layer[95:88];
			eth1_to_eth2_arp[39] <=eth1_ARP_layer[87:80];
			
			eth1_to_eth2_arp[40] <=eth1_ARP_layer[79:72];//Target MAC 
			eth1_to_eth2_arp[41] <=eth1_ARP_layer[71:64];
			eth1_to_eth2_arp[42] <=eth1_ARP_layer[63:56];
			eth1_to_eth2_arp[43] <=eth1_ARP_layer[55:48];
			eth1_to_eth2_arp[44] <=eth1_ARP_layer[47:40];
			eth1_to_eth2_arp[45] <=eth1_ARP_layer[39:32];
			
			eth1_to_eth2_arp[46] <=eth1_ARP_layer[31:24];//Target IP   
			eth1_to_eth2_arp[47] <=eth1_ARP_layer[23:16];
			eth1_to_eth2_arp[48] <=eth1_ARP_layer[15:8];
			eth1_to_eth2_arp[49] <=eth1_ARP_layer[7:0];
			
			eth1_to_eth2_arp[50] <=8'h00; //Padding
			eth1_to_eth2_arp[51] <=8'h00;
			eth1_to_eth2_arp[52] <=8'h00;
			eth1_to_eth2_arp[53] <=8'h00;
			eth1_to_eth2_arp[54] <=8'h00;
			eth1_to_eth2_arp[55] <=8'h00;
			eth1_to_eth2_arp[56] <=8'h00;
			eth1_to_eth2_arp[57] <=8'h00;
			eth1_to_eth2_arp[58] <=8'h00;
			eth1_to_eth2_arp[59] <=8'h00;
			eth1_to_eth2_arp[60] <=8'h00;
			eth1_to_eth2_arp[61] <=8'h00;
			eth1_to_eth2_arp[62] <=8'h00;
			eth1_to_eth2_arp[63] <=8'h00;
			eth1_to_eth2_arp[64] <=8'h00;
			eth1_to_eth2_arp[65] <=8'h00;
			eth1_to_eth2_arp[66] <=8'h00;
			eth1_to_eth2_arp[67] <=8'h00;
			eth1_to_eth2_arp[68] <=eth1_ARP_FCS[31:24];//CRC
			eth1_to_eth2_arp[69] <=eth1_ARP_FCS[23:16];
			eth1_to_eth2_arp[70] <=eth1_ARP_FCS[15:8];
			eth1_to_eth2_arp[71] <=eth1_ARP_FCS[7:0];

			eth1_to_eth2_pre_arp<=4'd2;
		end
	
		4'd2: begin //Put it FIFO
			if(eth1_to_eth2_pre_arp_bcnt<16'd71) begin
				fifo_eth1_to_eth2_arp_wr	<=1'b1;
				fifo_eth1_to_eth2_arp_din 	<=eth1_to_eth2_arp[eth1_to_eth2_pre_arp_bcnt];
				eth1_to_eth2_pre_arp_bcnt  <=eth1_to_eth2_pre_arp_bcnt+1'b1;
			end
			else begin
				fifo_eth1_to_eth2_arp_din 	<=eth1_to_eth2_arp[eth1_to_eth2_pre_arp_bcnt];
				eth1_to_eth2_pre_arp			<=4'd0;
			end
		end
		
		default: eth1_to_eth2_pre_arp<=4'd0;
	endcase
	
	//*******************************************************//
	//Prepare ICMP packet from Eth1 to Eth2,and put in FIFO *//
	//*******************************************************//	
	case(eth1_to_eth2_pre_icmp)
		4'd0: begin //Idle
			fifo_eth1_to_eth2_icmp_wr	<=1'b0;
			eth1_to_eth2_pre_icmp_bcnt	<=16'd0;
			event_eth1_to_eth2_icmp_rd <=1'b0;
			
			if(event_eth1_to_eth2_icmp_ok) begin
				eth1_to_eth2_pre_icmp<=4'd1;
				event_eth1_to_eth2_icmp_rd<=1'b1;
			end
		end
		
		4'd1: begin //Fill
			event_eth1_to_eth2_icmp_rd<=1'b0;
			
			eth1_to_eth2_icmp[0]  <=8'h55;               //PREAMBULE  
			eth1_to_eth2_icmp[1]  <=8'h55;
			eth1_to_eth2_icmp[2]  <=8'h55;
			eth1_to_eth2_icmp[3]  <=8'h55;
			eth1_to_eth2_icmp[4]  <=8'h55;
			eth1_to_eth2_icmp[5]  <=8'h55;
			eth1_to_eth2_icmp[6]  <=8'h55;
			eth1_to_eth2_icmp[7]	 <=8'hD5; 

			//Fill ETH Layer
			for(i2=8, j2=0; i2<22; i2=i2+1, j2=j2+1) begin
				eth1_to_eth2_icmp[i2] <=eth1_layer[111-(j2*8)-:8];
			end
			
			//Fill IPv4 Layer
			for(i2=22, j2=0; i2<42; i2=i2+1, j2=j2+1) begin
				eth1_to_eth2_icmp[i2] <=eth1_IPv4_layer[159-(j2*8)-:8];
			end
			
			//Fill ICMP Layer
			for(i2=42, j2=0; i2<106; i2=i2+1, j2=j2+1) begin
				eth1_to_eth2_icmp[i2] <=eth1_ICMP_layer[511-(j2*8)-:8];//487
			end

			eth1_to_eth2_icmp[106] <=eth1_ICMP_FCS[31:24];
			eth1_to_eth2_icmp[107] <=eth1_ICMP_FCS[23:16];
			eth1_to_eth2_icmp[108] <=eth1_ICMP_FCS[15:8];
			eth1_to_eth2_icmp[109] <=eth1_ICMP_FCS[7:0];
			
			eth1_to_eth2_pre_icmp<=4'd2;
		end
		
		4'd2: begin //Put it FIFO
			if(eth1_to_eth2_pre_icmp_bcnt<16'd109) begin
				fifo_eth1_to_eth2_icmp_wr	<=1'b1;
				fifo_eth1_to_eth2_icmp_din	<=eth1_to_eth2_icmp[eth1_to_eth2_pre_icmp_bcnt];
				eth1_to_eth2_pre_icmp_bcnt <=eth1_to_eth2_pre_icmp_bcnt+1'b1;
			end
			else begin
				fifo_eth1_to_eth2_icmp_din	<=eth1_to_eth2_icmp[eth1_to_eth2_pre_icmp_bcnt];
				eth1_to_eth2_pre_icmp		<=4'd0;
			end
		end
	
		default: eth1_to_eth2_pre_icmp<=4'd0;
	endcase
	
	
	
	//*******************************************************//
	//*Send packet from FIFO's										  *//
	//*******************************************************//
	case(eth2_tx_state)
		tx_IDLE: begin
		
			eth2_crcen<=0;
			eth2_crcrst<=1;
			eth2_tx_en<=0;
			
			eth2_tx_i<=0;
			eth2_tx_j<=0;
			eth2_tx_p<=0;

			eth2_tx_igp<=4'd0;
			eth2_tx_pre<=4'd0;
			eth2_tx_temp<=8'd0;
			
			if(fifo_eth1_to_eth2_arp_cnt>=16'd71) begin
				eth2_tx_state<=tx_ARP;
			end
			else if(fifo_eth1_to_eth2_icmp_cnt>=16'd109) begin
				eth2_tx_state<=tx_ICMP;
			end
		end
		
		tx_ARP:  begin
			case(eth2_tx_pre)
				4'd0: begin	//dummy
					fifo_eth1_to_eth2_arp_rd	<=1'b1;
					eth2_tx_pre<=4'd1;
				end
				
				4'd1: begin	//dummy
					fifo_eth1_to_eth2_arp_rd	<=1'b0;
					eth2_tx_pre<=4'd2;
				end
				
				4'd2: begin
					eth2_tx_en<=1;
					
					//****************************************************************
					//End of Packet
					if(eth2_tx_j==16'd71) begin//53
						if(eth2_tx_i==0) begin
							eth2_tx_i		<=eth2_tx_i+1'b1;
							eth2_txd[3:0] 	<=fifo_eth1_to_eth2_arp_dout[3:0];
							eth2_tx_temp	<=fifo_eth1_to_eth2_arp_dout;	
						end
						else if(eth2_tx_i==1) begin
							eth2_tx_i		<=0;
							eth2_txd[3:0] 	<=eth2_tx_temp[7:4];
							eth2_tx_state 	<=tx_IGP;
						end
					end
					
					//Start of Packet
					else begin
						if(eth2_tx_i==0) begin
							eth2_tx_i						<=eth2_tx_i+1'b1;
							fifo_eth1_to_eth2_arp_rd	<=1'b1;
							eth2_txd[3:0] 					<=fifo_eth1_to_eth2_arp_dout[3:0];
							eth2_tx_temp					<=fifo_eth1_to_eth2_arp_dout;	
							 
						end
						else if(eth2_tx_i==1) begin
							eth2_tx_i						<=0;
							eth2_tx_j						<=eth2_tx_j+1'b1;
							fifo_eth1_to_eth2_arp_rd	<=1'b0;
							eth2_txd[3:0] 					<=eth2_tx_temp[7:4];
						end
					end
				end
				
				default: eth2_tx_state<=tx_IDLE;
			endcase
		end
		
		tx_ICMP:	begin
			case(eth2_tx_pre)
				4'd0: begin	//dummy
					fifo_eth1_to_eth2_icmp_rd	<=1'b1;
					eth2_tx_pre<=4'd1;
				end
				
				4'd1: begin	//dummy
					fifo_eth1_to_eth2_icmp_rd	<=1'b0;
					eth2_tx_pre<=4'd2;
				end
				
				4'd2: begin //send
					eth2_tx_en<=1;
					
					//****************************************************************
					//End of Packet
					if(eth2_tx_j==16'd109) begin
						if(eth2_tx_i==0) begin
							eth2_tx_i		<=eth2_tx_i+1'b1;
							eth2_txd[3:0] 	<=fifo_eth1_to_eth2_icmp_dout[3:0];
							eth2_tx_temp	<=fifo_eth1_to_eth2_icmp_dout;	
						end
						else if(eth2_tx_i==1) begin
							eth2_tx_i		<=0;
							eth2_txd[3:0] 	<=eth2_tx_temp[7:4];
							eth2_tx_state 	<=tx_IGP;
						end
					end
					
					//Start of Packet
					else begin
						if(eth2_tx_i==0) begin
							eth2_tx_i						<=eth2_tx_i+1'b1;
							fifo_eth1_to_eth2_icmp_rd	<=1'b1;
							eth2_txd[3:0] 					<=fifo_eth1_to_eth2_icmp_dout[3:0];
							eth2_tx_temp					<=fifo_eth1_to_eth2_icmp_dout;	
							 
						end
						else if(eth2_tx_i==1) begin
							eth2_tx_i						<=0;
							eth2_tx_j						<=eth2_tx_j+1'b1;
							fifo_eth1_to_eth2_icmp_rd	<=1'b0;
							eth2_txd[3:0] 					<=eth2_tx_temp[7:4];
						end
					end
				end
				
				default: eth2_tx_state<=tx_IDLE;
			endcase
		end
		
		tx_IGP:  begin
			eth2_tx_en<=1'b0;
			eth2_txd<=4'h00;
			if(eth2_tx_igp<4'd12)
				eth2_tx_igp<=eth2_tx_igp+1'b1;
			else
				eth2_tx_state<=tx_IDLE;
		end
		
		default: eth2_tx_state<=tx_IDLE;
	endcase
	
end

endmodule
	
