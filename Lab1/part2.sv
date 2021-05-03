//Top module for game
module top 
(
    // Clock pins
    input                     clk,
    input                     reset,
    input                     enter,
    input               [7:0] guess,
    output              [7:0] actual,
    output                    dp_over,
    output                    dp_under,
	 output logic 					[3:0] dp_tries,
    output                    dp_equal
	
);

	logic over;
	logic under;
	logic equal;

	// Datapath
	logic dp_inc_actual;
	datapath the_datapath
	(
		.clk(clk),
		.reset(reset),
		.i_guess(guess),
		.i_inc_actual(dp_inc_actual),
		.i_tries(dp_tries),
		.o_over(over),
		.o_under(under),
		.o_equal(equal),
      .actual(actual)
	);
	
	// State Machine
	logic ctrl_update_leds;
	control the_control
	(
		.clk(clk),
		.reset(reset),
		.i_enter(enter),
		.o_inc_actual(dp_inc_actual),
		.i_over(over),
		.i_under(under),
		.i_equal(equal),
		.o_update_leds(ctrl_update_leds),
		.o_tries(dp_tries)
	);
	
	// LED controllers
	led_ctrl ledc_under(clk, reset, under, ctrl_update_leds, dp_under);
	led_ctrl ledc_over(clk, reset, over, ctrl_update_leds, dp_over);
	led_ctrl ledc_equal(clk, reset, equal, ctrl_update_leds, dp_equal);
	
endmodule

/*******************************************************/
/********************Control module********************/
/*****************************************************/
module control
(
	input clk,
	input reset,
	
	// Button input
	input i_enter,
	
	// Datapath
	output logic o_inc_actual,
	input i_over,
	input i_under,
	input i_equal,
	
	// LED Control
	output logic o_update_leds,
	output logic  [3:0] o_tries
);

// Declare two objects, 'state' and 'nextstate'
// that are of enum type.
enum int unsigned
{
	S_GEN_RAND,
	S_CHECK,
	S_WAIT_NOENTER,
	S_WAIT_ENTER,
	S_END
} state, nextstate;



integer increment = 1'b0;
reg [3:0] tries = 4'b0111;

// Clocked always block for making state registers
always_ff @ (posedge clk or posedge reset) begin
	o_tries <= tries;
	if (reset) begin
		state <= S_GEN_RAND;
		
	end
	
	else begin
		state <= nextstate;
		
	end
	

end

// always_comb replaces always @* and gives compile-time errors instead of warnings
// if you accidentally infer a latch
always_comb begin
	nextstate = state;
	o_inc_actual = 1'b0;
	o_update_leds = 1'b0;
	
	case (state)
		S_GEN_RAND:begin
			o_inc_actual = 1'b1;	
			tries = 4'b0111;
			
			if (i_enter)begin
				
				nextstate = S_WAIT_ENTER;
				o_inc_actual = 1'b0;
				o_update_leds = 1'b1;
			end
		end
			
		S_CHECK: begin
			tries = tries - 1;
			if(tries == 4'b0000)begin
					nextstate = S_END;
					o_update_leds = 1'b1;
					
			end
				
			else if(i_over || i_under)begin
				nextstate = S_WAIT_NOENTER;
				o_update_leds = 1'b1;
			end
			
			else if(i_equal)begin
				nextstate = S_END;
				o_update_leds = 1'b1;
			end
			
			
		end
		
		S_WAIT_NOENTER:begin
		
			
			
			if(i_enter)begin
				nextstate = S_WAIT_ENTER;
			end
			else
				nextstate = S_WAIT_NOENTER;
		end
				
		S_WAIT_ENTER:
			
			if(!i_enter)begin
				nextstate = S_CHECK;
				
			end
			else
				nextstate = S_WAIT_ENTER;
				
		S_END:begin
			nextstate = S_END;

		end
			
		default:
			nextstate = S_WAIT_NOENTER;
				
		// TODO: complete this
	endcase
end
endmodule

/*******************************************************/
/********************Datapath module*******************/
/*****************************************************/
module datapath
(
	input clk,
	input reset,
	
	// Number entry
	input [7:0] i_guess,
	input [3:0] i_tries,
	// Increment actual
	input i_inc_actual,
	
	// Comparison result
	output o_over,
	output o_under,
	output o_equal,
	output logic [7:0] actual
);

// Update the 'actual' register based on control signals
always_ff @ (posedge clk or posedge reset) begin
	if (reset) actual <= '0;
	else begin
		if (i_inc_actual) actual <= actual + 8'd1;
	end
end

// Generate comparisons
//reg over;
//reg equal;
//reg under;
//always @(*)begin
//	if(i_tries == 3'b111)begin
//		over = 0;
//		equal = 0;
//		under = 0;
//		
//end
assign o_over = ((i_guess > actual) );
assign o_equal = ((i_guess == actual));
assign o_under = ((i_guess < actual) );
endmodule

/*******************************************************/
/********************LED control module****************/
/*****************************************************/
module led_ctrl
(
	input clk,
	input reset,
	
	input i_val,
	input i_enable,
	output logic o_out
);

always_ff @ (posedge clk or posedge reset) begin
	if (reset) o_out <= '0;
	else if (i_enable) o_out <= i_val;
end

endmodule
