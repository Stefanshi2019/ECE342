module part1
	(
		input [31:0] X,
		input [31:0] Y,
		output [31:0] result
);

// Design your 32-bit Floating Point unit here. 

wire sign = X[31] ^ Y[31];
wire [7:0] ex_prev = X[30:23] + Y[30:23] - 8'd127;
wire [23:0] mantx = {1'b1, X[22:0]};
wire [23:0] manty = {1'b1, Y[22:0]};
wire [47:0] p = mantx * manty;
wire [24:0] truncated_product = p[47:23];

reg [22:0] product;

reg [7:0] ex;

reg [24:0] temp;

always @(*)begin
	if (truncated_product[24] == 1'b1)begin
		ex = ex_prev + 1'b1;
		temp = truncated_product >> 1;
		product = temp [22:0]; 
	end
	else begin
		ex = ex_prev;
		product = truncated_product[22:0];
	end
end

assign result = {sign, ex, product};

endmodule
