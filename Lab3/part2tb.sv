`timescale 1ns/1ns
module tb();
	logic [31:0] dut_x32, dut_y32;
	logic [31:0] dut_out32;
	wire  inf32, nan32, zero32, overflow32, underflow32;
	
	int input_x32 [];
	int input_y32 [];
  	shortreal x;
	shortreal y;
	shortreal result;
	shortreal out;
	
	real result_abs;
	real out_abs;
	real largest;
	real epsilon = 0.00001;
	
	part2 DUT32(
		.X           (dut_x32),
		.Y           (dut_y32),
		.result      (dut_out32),
		.inf		 (inf32),
		.nan	     (nan32),
		.zero		 (zero32),
		.overflow    (overflow32),
		.underflow 	 (underflow32)
		);
	
	initial begin
		input_x32 = new[6];
		input_y32 = new[6];
		input_x32 = '{32'hba57711a, 32'h34082401, 32'h5c75da81, 32'h07f00000, 32'h00000000, 32'h7f800ff0, 32'hfCFA3E28};
		input_y32 = '{32'hee1818c5, 32'hb328cd45, 32'h2f642a39, 32'h00800000, 32'hb3edcd45, 32'h74749112, 32'hfCFA3E28};
		
		foreach (input_x32[i]) begin
		
			dut_x32 = input_x32[i];
			dut_y32 = input_y32[i];
			x = $bitstoshortreal(dut_x32);
			y = $bitstoshortreal(dut_y32);
			result = x * y;
			#5;
			out = $bitstoshortreal(dut_out32);
			
			result_abs = result > 0 ? result : -result;
			out_abs = out > 0 ? out : -out;
			largest = result_abs > out_abs ? result_abs : out_abs;
			
			if ((((result_abs - out_abs) >  largest * epsilon ) ||inf32 || nan32 || zero32  || overflow32 || underflow32)&& i < 3 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x, y, out,  result);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if (i < 3) begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, result);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
			if ((out != 0 || inf32 || nan32 || zero32  || overflow32 || !underflow32)&& (i == 3 ||i == 7)) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %d", x, y, out, 0);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if ( i == 3 || i == 7)begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, 0);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
			if ((out != 0 || inf32 || nan32 || !zero32  || overflow32 || underflow32) && i == 4 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %d", x, y, out, 0);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if ( i == 4)begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, 0);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
			if ((dut_out32 != 32'h7f800000 || inf32 || !nan32 || zero32  || overflow32 || underflow32) && i == 5) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %d", x, y, out, 32'h7f800000);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if ( i == 5)begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, 32'h7f800000);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
			if ((dut_out32 != 32'h7f800000 || inf32 || nan32 || zero32 || !overflow32 || underflow32) && i == 6) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %d", x, y, out, 32'h7f800000);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end else if ( i == 6)begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, 32'h7f800000);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf32, nan32, zero32,  overflow32, underflow32);
			end
		end
	$stop();
	end
          
endmodule
