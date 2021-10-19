module crc (
	input 				clk, 
	input 				reset, 
	input      [0:3] 	data, 
	input 				enable, 
	output reg [31:0] crc,
	output     [31:0] crc_next
);

assign crc_next[0]  = enable & (data[0] ^ crc[28]); 
assign crc_next[1]  = enable & (data[1] ^ data[0] ^ crc[28] ^ crc[29]); 
assign crc_next[2]  = enable & (data[2] ^ data[1] ^ data[0] ^ crc[28] ^ crc[29] ^ crc[30]); 
assign crc_next[3]  = enable & (data[3] ^ data[2] ^ data[1] ^ crc[29] ^ crc[30] ^ crc[31]); 
assign crc_next[4]  = (enable & (data[3] ^ data[2] ^ data[0] ^ crc[28] ^ crc[30] ^ crc[31])) ^ crc[0]; 
assign crc_next[5]  = (enable & (data[3] ^ data[1] ^ data[0] ^ crc[28] ^ crc[29] ^ crc[31])) ^ crc[1]; 
assign crc_next[6]  = (enable & (data[2] ^ data[1] ^ crc[29] ^ crc[30])) ^ crc[ 2]; 
assign crc_next[7]  = (enable & (data[3] ^ data[2] ^ data[0] ^ crc[28] ^ crc[30] ^ crc[31])) ^ crc[3]; 
assign crc_next[8]  = (enable & (data[3] ^ data[1] ^ data[0] ^ crc[28] ^ crc[29] ^ crc[31])) ^ crc[4]; 
assign crc_next[9]  = (enable & (data[2] ^ data[1] ^ crc[29] ^ crc[30])) ^ crc[5]; 
assign crc_next[10] = (enable & (data[3] ^ data[2] ^ data[0] ^ crc[28] ^ crc[30] ^ crc[31])) ^ crc[6]; 
assign crc_next[11] = (enable & (data[3] ^ data[1] ^ data[0] ^ crc[28] ^ crc[29] ^ crc[31])) ^ crc[7]; 
assign crc_next[12] = (enable & (data[2] ^ data[1] ^ data[0] ^ crc[28] ^ crc[29] ^ crc[30])) ^ crc[8]; 
assign crc_next[13] = (enable & (data[3] ^ data[2] ^ data[1] ^ crc[29] ^ crc[30] ^ crc[31])) ^ crc[9]; 
assign crc_next[14] = (enable & (data[3] ^ data[2] ^ crc[30] ^ crc[31])) ^ crc[10]; 
assign crc_next[15] = (enable & (data[3] ^ crc[31])) ^ crc[11]; 
assign crc_next[16] = (enable & (data[0] ^ crc[28])) ^ crc[12]; 
assign crc_next[17] = (enable & (data[1] ^ crc[29])) ^ crc[13]; 
assign crc_next[18] = (enable & (data[2] ^ crc[30])) ^ crc[14]; 
assign crc_next[19] = (enable & (data[3] ^ crc[31])) ^ crc[15]; 
assign crc_next[20] = crc[16]; 
assign crc_next[21] = crc[17]; 
assign crc_next[22] = (enable & (data[0] ^ crc[28])) ^ crc[18]; 
assign crc_next[23] = (enable & (data[1] ^ data[0] ^ crc[29] ^ crc[28])) ^ crc[19]; 
assign crc_next[24] = (enable & (data[2] ^ data[1] ^ crc[30] ^ crc[29])) ^ crc[20]; 
assign crc_next[25] = (enable & (data[3] ^ data[2] ^ crc[31] ^ crc[30])) ^ crc[21]; 
assign crc_next[26] = (enable & (data[3] ^ data[0] ^ crc[31] ^ crc[28])) ^ crc[22]; 
assign crc_next[27] = (enable & (data[1] ^ crc[29])) ^ crc[23]; 
assign crc_next[28] = (enable & (data[2] ^ crc[30])) ^ crc[24]; 
assign crc_next[29] = (enable & (data[3] ^ crc[31])) ^ crc[25]; 
assign crc_next[30] = crc[26]; 
assign crc_next[31] = crc[27]; 

always @ (posedge clk or posedge reset)
begin
  if (reset)
    crc <= #1 32'hffffffff;
  else if(enable)
    crc <= crc_next;
end

endmodule
