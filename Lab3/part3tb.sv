`timescale 1ns/1ns
module tb();
	logic [31:0] dut_x32, dut_y32;
	logic [31:0] dut_out32;
	wire  inf32, nan32, zero32, overflow32, underflow32;
	
	logic [63:0] dut_x64, dut_y64;
	logic [63:0] dut_out64;
	wire  inf64, nan64, zero64, overflow64, underflow64;
	
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
	
	longint input_x64 [];
	longint input_y64 [];
  	real x64;
	real y64;
	real result64;
	real out64;
	
	
	real result_abs64;
	real out_abs64;
	real largest64;
	real epsilon64 = 0.00001;
	
	part3 DUT32(
		.X             (dut_x32),
		.Y             (dut_y32),
      .result      (dut_out32),
		.inf		        (inf32),
		.nan	           (nan32),
		.zero		       (zero32),
		.overflow   (overflow32),
		.underflow (underflow32));
		defparam DUT32.EXP = 8;
		defparam DUT32.MAN = 23;
	
	part3 DUT64(
		.X             (dut_x64),
		.Y             (dut_y64),
      .result      (dut_out64),
		.inf		        (inf64),
		.nan	           (nan64),
		.zero		       (zero64),
		.overflow   (overflow64),
		.underflow (underflow64));
		defparam DUT64.EXP = 11;
		defparam DUT64.MAN = 52;
	
	initial begin
		input_x32 = new[11];
		input_y32 = new[11];
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
		
		
		input_x64 = new[8];
		input_y64 = new[8];
		input_x64 = '{64'hba57711adf44e522, 64'h3478374839238348, 64'hbcdaeffebcda3832, 64'h0011130ebcda3832, 64'h7ff0000000000000};
		input_y64 = '{64'hee1818c5144030e0, 64'h384739e834793238, 64'hf3c2e4d1a8230fe3, 64'h010401d1a8230fe3, 64'h384739e834793238};
		
		foreach (input_x64[i]) begin
			dut_x64 = input_x64[i];
			dut_y64 = input_y64[i];
			x64 = $bitstoreal(dut_x64);
			y64 = $bitstoreal(dut_y64);
			result64 = x64 * y64;
			#5;
			out64 = $bitstoreal(dut_out64);
			
			result_abs64 = result > 0 ? result64 : -result64;
			out_abs64 = out64 > 0 ? out64 : -out64;
			largest64 = result_abs64 > out_abs64 ? result_abs64 : out_abs64;
			
			if ((((result_abs64 - out_abs64) >  largest64 * epsilon64 ) ||inf64 || nan64 || zero64  || overflow64 || underflow64)&& i < 3 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64,  result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end else if (i < 3) begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64, result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end
			
			if ((out64 != 0 ||inf64 || nan64 || zero64  || overflow64 || !underflow64)&& i == 3 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64,  result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end else if (i == 3) begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64, result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end
			if ((dut_out64 != 64'h7ff0000000000000 ||!inf64 || nan64 || zero64  || overflow64 || underflow64)&& i == 4 ) begin
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64,  result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end else if (i == 4) begin
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x64, y64, out64, result64);
				$display("Infinity = %d, NaN = %d, Zero = %d, Overflow = %d, Underflow = %d", inf64, nan64, zero64,  overflow64, underflow64);
			end
		end
	$stop();
	end
          
endmodule
