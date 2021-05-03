module part2
	(
		input [31:0] X,
		input [31:0] Y,
		output inf, nan, zero, overflow, underflow,
		output reg[31:0] result
);

// Design your 32-bit Floating Point unit here. 

wire sign = X[31] ^ Y[31];
wire [9:0] ex_prev = X[30:23] + Y[30:23];
wire [23:0] mantx = {1'b1, X[22:0]};
wire [23:0] manty = {1'b1, Y[22:0]};
wire [47:0] p = mantx * manty;
wire [24:0] truncated_product = p[47:23];

reg sign_out;
reg [22:0] product;

reg [7:0] ex;
reg [9:0] ex_p;
reg [9:0] ex_p2;

reg [24:0] temp;
reg inft, nant, zerot, overflowt, underflowt;

always @(*)begin
	inft = 0;
	nant = 0;
	zerot = 0;
	overflowt = 0;
	underflowt = 0;
	if(truncated_product[24] == 1'b1)begin
		temp = truncated_product >> 1;
		ex_p = ex_prev + 1;
	end
	else begin
		temp = truncated_product;
		ex_p = ex_prev;
	end

	if ((ex_p == 0 && temp == 0) || X == 0 || Y == 0)begin
		sign_out = 1'b0;
		ex = 8'b0;
		product = 23'b0;
		zerot = 1;
	end
	
	else if ((ex_p == 9'd127 && temp !=0) || (X[30:23]==255 && X[22:0]!='b0) || (Y[30:23]==255 && Y[22:0]!='b0))begin
		sign_out = 1'b0;
		ex = 8'd255;
		product = 23'b0;
		nant = 1;
	end
	else if ((ex_p == 9'd127 && temp ==0) || (X[30:23]==255 && X[22:0]=='b0) || (Y[30:23]==255 && Y[22:0]=='b0))begin
		sign_out = 1'b0;
		ex = 8'd255;
		product = 23'b0;
		inft = 1;
	end
	else if(ex_p  > ( 9'd255 + 9'd127))begin
		sign_out = 1'b0;
		ex = 8'd255;
		product = 23'b0;
		overflowt = 1;
	end
	
	else if(ex_p < 9'd127)begin
		sign_out = 1'b0;
		ex = 8'b0;
		product = 23'b0;
		underflowt = 1;
	end
	else begin
		sign_out = sign;
		ex_p2 = ex_p - 8'd127;
		ex = ex_p2[7:0];
		product = temp[22:0];
	end
end

assign result = {sign_out, ex, product};
assign zero = zerot;
assign nan = nant;
assign inf = inft;
assign underflow = underflowt;
assign overflow = overflowt;
endmodule