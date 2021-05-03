module wallace_mult (
  input [7:0] x,
  input [7:0] y,
  output [15:0] out,
  output [15:0] pp [4]
);

// These signals are created to help you map the wires you need with the diagram provided in the lab document.

wire [15:0] s_lev01; //the first "save" output of level0's CSA array
wire [15:0] c_lev01; //the first "carry" output of level0's CSA array
wire [15:0] s_lev02; //the second "save" output of level0's CSA array
wire [15:0] c_lev02;
wire [15:0] s_lev11;
wire [15:0] c_lev11;
wire [15:0] s_lev12; //the second "save" output of level1's CSA array
wire [15:0] c_lev12;
wire [15:0] s_lev21;
wire [15:0] c_lev21;
wire [15:0] s_lev31;
wire [15:0] c_lev31;

// TODO: complete the hardware design for instantiating the CSA blocks per level.
	wire [15:0] ppi [8];
	//assign product_arr[0] = '0;
	
	genvar i;
	generate
		for(i=0; i<8; i=i+1)begin
			assign ppi[i] = (x * y[i]) << i;
		end
	endgenerate
//	input [width-1:0] op1,
//	input [width-1:0] op2,
//	input [width-1:0] op3,
//	output [width-1:0] S,
//	output [width-1:0] C
//level 0
	csa CSA01(.op1(ppi[0]), .op2(ppi[1]), .op3(ppi[2]), .S(s_lev01), .C(c_lev01));
	csa CSA02(.op1(ppi[3]), .op2(ppi[4]), .op3(ppi[5]), .S(s_lev02), .C(c_lev02));
//level 1
	wire [15:0] c_lev01s = c_lev01 << 1;
	wire [15:0] c_lev02s = c_lev02 << 1;
	csa CSA11(.op1(s_lev01), .op2(s_lev02), .op3(c_lev01s), .S(s_lev11), .C(c_lev11));
	csa CSA12(.op1(c_lev02s), .op2(ppi[6]), .op3(ppi[7]), .S(s_lev12), .C(c_lev12));
//level 2, the save and carry output of level 2 will be pp[2] and pp[3]
	wire [15:0] c_lev11s = c_lev11 << 1;
	wire [15:0] c_lev12s = c_lev12 << 1;
	csa CSA21(.op1(c_lev11s), .op2(s_lev11), .op3(s_lev12), .S(s_lev21), .C(c_lev21));
//level 3, the save and carry output of level 3 will be pp[2] and pp[3]	
  
	wire [15:0] c_lev21s = c_lev21 << 1;
	csa CSA31(.op1(s_lev21), .op2(c_lev21s), .op3(c_lev12s), .S(s_lev31), .C(c_lev31));
	
	wire cout;
	wire [15:0] c_lev31s = c_lev31 << 1;
	rca RCA31(.op1(s_lev31), .op2(c_lev31s), .cin('0), .sum(out), .cout(cout));
	
  assign pp[0] = s_lev21;
  assign pp[1] = c_lev21;


  
  assign pp[2] = s_lev31;
  assign pp[3] = c_lev31;

// Ripple carry adder to calculate the final output.

endmodule





// These modules are provided for you to use in your designs.
// They also serve as examples of parameterized module instantiation.
module rca #(width=16) (
    input  [width-1:0] op1,
    input  [width-1:0] op2,
    input  cin,
    output [width-1:0] sum,
    output cout
);

wire [width:0] temp;
assign temp[0] = cin;
assign cout = temp[width];

genvar i;
for( i=0; i<width; i=i+1) begin
    full_adder u_full_adder(
        .a      (   op1[i]     ),
        .b      (   op2[i]     ),
        .cin    (   temp[i]    ),
        .cout   (   temp[i+1]  ),
        .s      (   sum[i]     )
    );
end

endmodule


module full_adder(
    input a,
    input b,
    input cin,
    output cout,
    output s
);

assign s = a ^ b ^ cin;
assign cout = a & b | (cin & (a ^ b));

endmodule

module csa #(width=16) (
	input [width-1:0] op1,
	input [width-1:0] op2,
	input [width-1:0] op3,
	output [width-1:0] S,
	output [width-1:0] C
);

genvar i;
generate
	for(i=0; i<width; i++) begin
		full_adder u_full_adder(
			.a      (   op1[i]    ),
			.b      (   op2[i]    ),
			.cin    (   op3[i]    ),
			.cout   (   C[i]	  ),
			.s      (   S[i]      )
		);
	end
endgenerate

endmodule

