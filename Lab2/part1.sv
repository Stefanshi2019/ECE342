/*******************************************************/
/********************Multiplier module********************/
/*****************************************************/
// add additional modules as needs, such as full adder, and others

// multiplier module
module mult
(
	input [7:0] x,
	input [7:0] y,
	output [15:0] out,   // Result of the multiplication
	output [15:0] pp [9] // for automarker to check partial products of a multiplication 
);
	// Declare a 9-high, 16-deep array of signals holding sums of the partial products.
	// They represent the _input_ partial sums for that row, coming from above.
	// The input for the "ninth row" is actually the final multiplier output.
	// The first row is tied to 0.
	
	// Make another array to hold the carry signals
	logic [16:0] cin[9];
	assign cin[0] = '0;
	
//	initial begin
//		integer i1;
//		for(i1=0; i1<9; i1=i1+1)begin
//		
//
//			cin[i1] = '0;
//		end
//	end
	
//	genvar i2;
//	generate
//		for(i2 = 0; i2<9; i2 = i2+1)begin
//			assign pp[i2] = '0;
//		end	
//	endgenerate
	assign pp[0] = '0;
	
	wire [15:0] product_arr [9];
	//assign product_arr[0] = '0;
	
	genvar i;
	generate
		for(i=0; i<8; i=i+1)begin
			assign product_arr[i] = (x * y[i]) << i;
		end
	endgenerate
	
	
	genvar j;
	genvar k2;
	generate
		for(j=1; j<=8; ++j)begin
				genvar k;
				// pull down from above
				if(j==1)begin
					for(k=j-1; k<=j+7; ++k)begin
						full_adder FA1(.a(pp[j-1][k]), .b(product_arr[j-1][k]), .cin(cin[j-1][k]), .cout(cin[j][k]), .s(pp[j][k]));
						
					end
					for(k=j+8; k<=15; ++k)begin
						full_adder FA1(.a(pp[j-1][k]), .b(1'b0), .cin(1'b0), .cout(cin[j][k]), .s(pp[j][k]));
						
					end
				end
				
				else begin
					for(k=0; k<j-1; ++k)begin
						full_adder FA1(.a(pp[j-1][k]), .b(1'b0), .cin(1'b0), .cout(cin[j][k]), .s(pp[j][k]));
					end
					// compute 
					for(k=j-1; k<=j+7; ++k)begin
						full_adder FA1(.a(pp[j-1][k]), .b(product_arr[j-1][k]), .cin(cin[j-1][k-1]), .cout(cin[j][k]), .s(pp[j][k]));
						
					end
					
					for(k=j+8; k<=15; ++k)begin
						full_adder FA1(.a(pp[j-1][k]), .b(1'b0), .cin(1'b0), .cout(cin[j][k]), .s(pp[j][k]));
						
					end
				end
		end
		
		wire [15:0] cout;
		wire [15:0] copen;
		// last layer of ripple carry adder, layer = 8

		for(k2=0; k2<=7; ++k2)begin
			full_adder FA1(.a(1'b0), .b(pp[8][k2]), .cin(1'b0), .cout(copen[k2]), .s(cout[k2]));
		end
		
		for(k2=8; k2<=15; ++k2)begin
			full_adder FA3(.a(pp[8][k2]), .b(cin[8][k2-1]), .cin(copen[k2-1]), .cout(copen[k2]), .s(cout[k2]));
		end
		
		//full_adder FA4(.a(product_arr[7][7]), .b(cin[6][13]), .cin(cin[7][13]), .cout(cout[7][15]), .s(pp[7][14]));
	endgenerate
	
	
	
	
	
	
	
	// Cin signals for the final (fast adder) row
	logic [16:8] cin_final;
	assign cin_final[15:8] = cin[7][15:8];
	
	// TODO: complete the following digital logic design of a carry save multiplier (unsigned)
	// Note: generate_hw tutorial can help you describe duplicated modules efficiently
	
	// Note: partial product of each row is the result coming out from a full adder at the end of that row
	
	// Note: a "Fast adder" operates on columns 8 through 15 of final row.
	
	assign out[15:0] = cout[15:0];
		  
endmodule

// The following code is provided for you to use in your design

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