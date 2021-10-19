 module gray2bin
(
	input  [11:0] G,  //gray code output
   output [11:0] bin //binary input
);

assign bin[11] = G[11];
assign bin[10] = G[11] ^ G[10];
assign bin[9]  = G[11] ^ G[10] ^ G[9];
assign bin[8]  = G[11] ^ G[10] ^ G[9] ^ G[8];
assign bin[7]  = G[11] ^ G[10] ^ G[9] ^ G[8] ^ G[7];
assign bin[6]  = G[11] ^ G[10] ^ G[9] ^ G[8] ^ G[7] ^ G[6];
assign bin[5]  = G[11] ^ G[10] ^ G[9] ^ G[8] ^ G[7] ^ G[6] ^ G[5];
assign bin[4]  = G[11] ^ G[10] ^ G[9] ^ G[8] ^ G[7] ^ G[6] ^ G[5] ^ G[4];
assign bin[3]  = G[11] ^ G[10] ^ G[9] ^ G[8] ^ G[7] ^ G[6] ^ G[5] ^ G[4] ^ G[3];
assign bin[2]  = G[11] ^ G[10] ^ G[9] ^ G[8] ^ G[7] ^ G[6] ^ G[5] ^ G[4] ^ G[3] ^ G[2];
assign bin[1]  = G[11] ^ G[10] ^ G[9] ^ G[8] ^ G[7] ^ G[6] ^ G[5] ^ G[4] ^ G[3] ^ G[2] ^ G[1];
assign bin[0]  = G[11] ^ G[10] ^ G[9] ^ G[8] ^ G[7] ^ G[6] ^ G[5] ^ G[4] ^ G[3] ^ G[2] ^ G[1] ^ G[0];

endmodule
