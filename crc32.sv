module crc32 (
	input 				clk, 
	input 				reset, 
	input      [7:0] 	data_in, 
	input 				enable, 
	output reg [31:0] crc,
	output     [31:0] crc_next
);

//parameter Tp = 1;
//input clk;
//input reset;
//input [7:0] data_in;
//input enable;
//output [31:0] crc;
//reg  [31:0] crc;
//output [31:0] crc_next;

wire [7:0] Data;
assign Data={data_in[0],data_in[1],data_in[2],data_in[3],data_in[4],data_in[5],data_in[6],data_in[7]};

assign crc_next[0]  = crc[24] ^ crc[30] ^ Data[0] ^ Data[6];
assign crc_next[1]  = crc[24] ^ crc[25] ^ crc[30] ^ crc[31] ^ Data[0] ^ Data[1] ^ Data[6] ^ Data[7];
assign crc_next[2]  = crc[24] ^ crc[25] ^ crc[26] ^ crc[30] ^ crc[31] ^ Data[0] ^ Data[1] ^ Data[2] ^ Data[6] ^ Data[7];
assign crc_next[3]  = crc[25] ^ crc[26] ^ crc[27] ^ crc[31] ^ Data[1] ^ Data[2] ^ Data[3] ^ Data[7];
assign crc_next[4]  = crc[24] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[30] ^ Data[0] ^ Data[2] ^ Data[3] ^ Data[4] ^ Data[6];
assign crc_next[5]  = crc[24] ^ crc[25] ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[30] ^ crc[31] ^ Data[0] ^ Data[1] ^ Data[3] ^ Data[4] ^ Data[5] ^ Data[6] ^ Data[7];
assign crc_next[6]  = crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ crc[30] ^ crc[31] ^ Data[1] ^ Data[2] ^ Data[4] ^ Data[5] ^ Data[6] ^ Data[7];
assign crc_next[7]  = crc[24] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[31] ^ Data[0] ^ Data[2] ^ Data[3] ^ Data[5] ^ Data[7];
assign crc_next[8]  = crc[0]  ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[28] ^ Data[0] ^ Data[1] ^ Data[3] ^ Data[4];
assign crc_next[9]  = crc[1]  ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ Data[1] ^ Data[2] ^ Data[4] ^ Data[5];
assign crc_next[10] = crc[2]  ^ crc[24] ^ crc[26] ^ crc[27] ^ crc[29] ^ Data[0] ^ Data[2] ^ Data[3] ^ Data[5];
assign crc_next[11] = crc[3]  ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[28] ^ Data[0] ^ Data[1] ^ Data[3] ^ Data[4];
assign crc_next[12] = crc[4]  ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ crc[30] ^ Data[0] ^ Data[1] ^ Data[2] ^ Data[4] ^ Data[5] ^ Data[6];
assign crc_next[13] = crc[5]  ^ crc[25] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[30] ^ crc[31] ^ Data[1] ^ Data[2] ^ Data[3] ^ Data[5] ^ Data[6] ^ Data[7];
assign crc_next[14] = crc[6]  ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[30] ^ crc[31] ^ Data[2] ^ Data[3] ^ Data[4] ^ Data[6] ^ Data[7];
assign crc_next[15] = crc[7]  ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[31] ^ Data[3] ^ Data[4] ^ Data[5] ^ Data[7];
assign crc_next[16] = crc[8]  ^ crc[24] ^ crc[28] ^ crc[29] ^ Data[0] ^ Data[4] ^ Data[5];
assign crc_next[17] = crc[9]  ^ crc[25] ^ crc[29] ^ crc[30] ^ Data[1] ^ Data[5] ^ Data[6];
assign crc_next[18] = crc[10] ^ crc[26] ^ crc[30] ^ crc[31] ^ Data[2] ^ Data[6] ^ Data[7];
assign crc_next[19] = crc[11] ^ crc[27] ^ crc[31] ^ Data[3] ^ Data[7];
assign crc_next[20] = crc[12] ^ crc[28] ^ Data[4];
assign crc_next[21] = crc[13] ^ crc[29] ^ Data[5];
assign crc_next[22] = crc[14] ^ crc[24] ^ Data[0];
assign crc_next[23] = crc[15] ^ crc[24] ^ crc[25] ^ crc[30] ^ Data[0] ^ Data[1] ^ Data[6];
assign crc_next[24] = crc[16] ^ crc[25] ^ crc[26] ^ crc[31] ^ Data[1] ^ Data[2] ^ Data[7];
assign crc_next[25] = crc[17] ^ crc[26] ^ crc[27] ^ Data[2] ^ Data[3];
assign crc_next[26] = crc[18] ^ crc[24] ^ crc[27] ^ crc[28] ^ crc[30] ^ Data[0] ^ Data[3] ^ Data[4] ^ Data[6];
assign crc_next[27] = crc[19] ^ crc[25] ^ crc[28] ^ crc[29] ^ crc[31] ^ Data[1] ^ Data[4] ^ Data[5] ^ Data[7];
assign crc_next[28] = crc[20] ^ crc[26] ^ crc[29] ^ crc[30] ^ Data[2] ^ Data[5] ^ Data[6];
assign crc_next[29] = crc[21] ^ crc[27] ^ crc[30] ^ crc[31] ^ Data[3] ^ Data[6] ^ Data[7];
assign crc_next[30] = crc[22] ^ crc[28] ^ crc[31] ^ Data[4] ^ Data[7];
assign crc_next[31] = crc[23] ^ crc[29] ^ Data[5];

always @ (posedge clk, posedge reset) begin
	if (reset) begin
		crc <={32{1'b1}};
	end
	else if (enable)
		crc <=crc_next;
	end
endmodule
