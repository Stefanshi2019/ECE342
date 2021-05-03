`timescale 1ns/1ns
module tb();
	logic [31:0] dut_x, dut_y;
	logic [31:0] dut_out;
	int input_x [];
	int input_y [];
  	shortreal x;
	shortreal y;
	shortreal result;
	shortreal out;
	
	shortreal result_abs;
	shortreal out_abs;
	shortreal largest;
	shortreal epsilon = 0.00001;
	
	part1 DUT(
		.X(dut_x),
		.Y(dut_y),
      .result(dut_out));
	
	initial begin
		input_x = new[3];
		input_y = new[3];
		input_x = '{32'hba57711a, 32'h34082401, 32'h5c75da81};
		input_y = '{32'hee1818c5, 32'hb328cd45, 32'h2f642a39};
		
		foreach (input_x[i]) begin
		
			dut_x = input_x[i];
			dut_y = input_y[i];
			x = $bitstoshortreal(dut_x);
			y = $bitstoshortreal(dut_y);
			result = x * y;
			#5;
			out = $bitstoshortreal(dut_out);
			
			result_abs = result > 0 ? result : -result;
			out_abs = out > 0 ? out : -out;
			largest = result_abs > out_abs ? result_abs : out_abs;
			
			if ((result_abs - out_abs) >=  largest * epsilon)
				$display("FAIL Functionality Checking: %g * %g = %g, expected %g", x, y, out, result);
			else
				$display("PASS Functionality Checking: %g * %g = %g, expected %g", x, y, out, result);
		end
		
	$stop();
	end
          
endmodule
