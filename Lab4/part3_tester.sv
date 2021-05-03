`timescale 1ns/1ns
module nios_system_tb();
    logic clock;
    logic reset_n;

    integer counter;
    integer start_time;
    integer end_time;
    integer delay;
	integer pass;

    nios_system DUT
    (
        .clk_clk         (clock),        //      clk.clk
        .reset_reset_n   (reset_n)       //    reset.reset_n
    );

    always begin
		#10 clock = ~clock;
    end

    always @(posedge clock) begin
        counter = counter + 1;
    end

    initial begin
		clock = 0;
        counter = 0;
        reset_n = 0;
        start_time = 0;
        end_time = 0;
		pass = 0;
        #1000
        reset_n = 1;
		DUT.onchip_memory2_0.the_altsyncram.m_default.altsyncram_inst.mem_data[3072] = 32'h4059999A;     // 3.4
		DUT.onchip_memory2_0.the_altsyncram.m_default.altsyncram_inst.mem_data[3073] = 32'h4194CCCD;     // 18.6
		wait (DUT.onchip_memory2_0.the_altsyncram.m_default.altsyncram_inst.mem_data[3076] == 1);
		start_time = counter;
		wait (DUT.onchip_memory2_0.the_altsyncram.m_default.altsyncram_inst.mem_data[3074] == 32'h427CF5C3);
        end_time = counter;
        delay = end_time - start_time;
		pass = pass + 1;
		$display("Software multiplication successful, PASS.");
        $display("Software multiplication took %d cycles", delay);
        wait (DUT.avs_fp_mult_0.avs.mul_op_1 == 32'h4059999A);
        wait (DUT.avs_fp_mult_0.avs.mul_op_2 == 32'h4194CCCD);
		wait (DUT.avs_fp_mult_0.avs.mul_start == 1);
        start_time = counter;
        wait (DUT.onchip_memory2_0.the_altsyncram.m_default.altsyncram_inst.mem_data[3075] == 32'h427CF5C3);
		end_time = counter;
		delay = end_time - start_time;
		pass = pass + 1;
		$display("Hardware multiplication successful, PASS.");
        $display("Hardware multiplication took %d cycles", delay);
		$finish;
    end

    initial begin
        #2000000
        if (pass == 0) begin
            $display("Software multiplication did not finish or incorrect, FAIL.");
			$display("Hardware multiplication did not finish or incorrect, FAIL.");
			$stop;
        end else if (pass == 1) begin
			$display("Hardware multiplication did not finish or incorrect, FAIL.");
			$stop;
		end
    end
endmodule
