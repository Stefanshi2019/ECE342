`timescale 1ns/1ns
module nios_system_tb();
    logic clock;
    logic reset_n;
    logic [7:0] switches;
    logic [7:0] leds;

    nios_system DUT
    (
		.clk_clk         (clock),       //      clk.clk
		.leds_export     (leds),        //     leds.export
		.reset_reset_n   (reset_n),     //    reset.reset_n
		.switches_export (switches)  	// switches.export
    );

    always begin
		#10 clock = ~clock;
    end

    initial begin
		clock = 0;
		reset_n = 0;
		#1000
		reset_n = 1;
		#800000
		switches = 8'b10101111;
		#5000
		if (leds == switches) begin
			$display("LEDs match the switches, PASS.");
		end else begin
			$display("LEDs don't match the switches, FAIL.");
		end
		switches = 8'b10100101;
		#5000
		if (leds == switches) begin
			$display("LEDs match the switches, PASS.");
		end else begin
			$display("LEDs don't match the switches, FAIL.");
		end
    end
endmodule
