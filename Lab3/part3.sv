// This module uses parameterized instantiation. If values are not set in the testbench the default values specified here are used. 
// The values for EXP and MAN set here are for a IEEE-754 32-bit Floating point representation. 
// TO DO: Edit BITS and BIAS based on the EXP and MAN parameters. 

module part3
	#(parameter EXP = 8,			// Number of bits in the Exponent
	  parameter MAN = 23, 			// Number of bits in the Mantissa
	  parameter BITS = 1 + EXP + MAN,	// Total number of bits in the floating point number
	  parameter BIAS = 2**(EXP-1) - 1		// Value of the bias, based on the exponent. 
	  )
	(
		input [BITS - 1:0] X,
		input [BITS - 1:0] Y,
		output inf, nan, zero, overflow, underflow,
		output reg[BITS - 1:0] result
);

// Design your 32-bit Floating Point unit here. 


wire sign = X[BITS-1] ^ Y[BITS-1];
wire [EXP+2:0] ex_prev = X[BITS-2:MAN] + Y[BITS-2:MAN];
wire [MAN:0] mantx = {1'b1, X[MAN-1:0]};
wire [MAN:0] manty = {1'b1, Y[MAN-1:0]};
wire [MAN*2+1:0] p = mantx * manty;
wire [MAN+1:0] truncated_product = p[MAN*2+1:MAN];

reg sign_out;
reg [MAN-1:0] product;

reg [EXP-1:0] ex;
reg [EXP+2:0] ex_p;
reg [EXP+2:0] ex_p2;

reg [MAN+1:0] temp;
reg inft, nant, zerot, overflowt, underflowt;

always @(*)begin
	inft = 0;
	nant = 0;
	zerot = 0;
	overflowt = 0;
	underflowt = 0;
	if(truncated_product[MAN+1] == 1'b1)begin
		temp = truncated_product >> 1;
		ex_p = ex_prev + 1;
	end
	else begin
		temp = truncated_product;
		ex_p = ex_prev;
	end

	if ((ex_p == 0 && temp == 0) || X == 0 || Y == 0)begin
		sign_out = 1'b0;
		ex = 'b0;
		product = 'b0;
		zerot = 1;
	end
	
	else if ((ex_p == BIAS && temp !=0)||(X[BITS-2:MAN]==2**EXP-1 && X[MAN-1:0]!='b0) || (Y[BITS-2:MAN]==2**EXP-1 && Y[MAN-1:0]!='b0))begin
		sign_out = 1'b0;
		ex = BIAS * 2 + 1;
		product = 'b0;
		nant = 1;
	end
	else if ((ex_p == BIAS && temp ==0) ||(X[BITS-2:MAN]==2**EXP-1 && X[MAN-1:0]=='b0) || (Y[BITS-2:MAN]==2**EXP-1 && Y[MAN-1:0]=='b0))begin
		sign_out = 1'b0;
		ex = BIAS * 2 + 1;
		product = 'b0;
		inft = 1;
	end
	else if(ex_p  > ( BIAS*3 + 1))begin
		sign_out = 1'b0;
		ex = BIAS * 2 + 1;
		product = 'b0;
		overflowt = 1;
	end
	
	else if(ex_p < BIAS)begin
		sign_out = 1'b0;
		ex = 'b0;
		product = 'b0;
		underflowt = 1;
	end
	else begin
		sign_out = sign;
		ex_p2 = ex_p - BIAS;
		ex = ex_p2[EXP-1:0];
		product = temp[MAN-1:0];
	end
end

assign result = {sign_out, ex, product};
assign zero = zerot;
assign nan = nant;
assign inf = inft;
assign underflow = underflowt;
assign overflow = overflowt;

endmodule