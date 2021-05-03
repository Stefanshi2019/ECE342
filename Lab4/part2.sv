module avalon_fp_mult
(
	input clk,
	input reset,
	
	// Avalon Slave
	input [2:0] avs_s1_address,
	input avs_s1_read,
	input avs_s1_write,
	input [31:0] avs_s1_writedata,
	output logic [31:0] avs_s1_readdata,
	output logic avs_s1_waitrequest
);

wire [31:0] op1;
wire [31:0] op2;
wire [31:0] mul_start;
wire [31:0] mul_result;
wire [31:0] mul_status;

assign mul_status[31:4] = 0;

	avalon_slave avs
	(
		.clk(clk),
		.reset(reset),
		.avs_s1_address(avs_s1_address),
		.avs_s1_read(avs_s1_read),
		.avs_s1_write(avs_s1_write),
		.avs_s1_writedata(avs_s1_writedata),
		.avs_s1_readdata(avs_s1_readdata),
		.avs_s1_waitrequest(avs_s1_waitrequest),

		// FP_MULT
		// STUDENTS TO ADD THESE
		.op1(op1),
		.op2(op2),
		.mul_start_out(mul_start),
		.mul_result_in(mul_result),
		.mul_status_in(mul_status)
	);
//	.aclr ( aclr_sig ),
//	.clk_en ( clk_en_sig ),
//	.clock ( clock_sig ),
//	.dataa ( dataa_sig ),
//	.datab ( datab_sig ),
//	.nan ( nan_sig ),
//	.overflow ( overflow_sig ),
//	.result ( result_sig ),
//	.underflow ( underflow_sig ),
//	.zero ( zero_sig )
	fp_mult fpm
	(
		.aclr(reset),
		.clk_en(mul_start[0]),
		.clock(clk),
		.dataa(op1),
		.datab(op2),
		.result(mul_result),
		.overflow(mul_status[3]),
		.underflow(mul_status[2]),
		.zero(mul_status[1]),
		.nan(mul_status[0])
        // STUDENTS TO ADD THESE
	);

endmodule


module avalon_slave
(
	input clk,
	input reset,
	
	// Avalon Slave
	input [2:0] avs_s1_address,
	input avs_s1_read,
	input avs_s1_write,
	input [31:0] avs_s1_writedata,
	output logic [31:0] avs_s1_readdata,
	output logic avs_s1_waitrequest,

	// FP_MULT
    // STUDENTS TO ADD THESE
	 output logic [31:0] op1,
	 output logic [31:0] op2,
	 output logic [31:0] mul_start_out,
	 input [31:0] mul_result_in,
	 input [31:0] mul_status_in
);

// Mem-mapped regs
// Reg0-32:			A
// Reg1-32:			B
// Reg2-04:			Start/Busy
// Reg3-32:			Result
// Reg4-04:			Status (Flags)
reg [31:0] mul_op_1;
reg [31:0] mul_op_2;
reg [31:0] mul_start;
reg [31:0] mul_result;
reg [31:0] mul_status;

assign mul_result = mul_result_in;
assign mul_status = mul_status_in;

// STUDENTS TO ADD THESE

parameter INIT = 3'd0, START = 3'd1, WAIT = 3'd2, END = 3'd3;

reg [3:0] current_state;
reg [3:0] next_state;
reg [3:0] written_signals;

reg [3:0] count;
reg start_count;
always@(*)begin
	if(reset)
		next_state = INIT;
	case(current_state)
		INIT:begin
			next_state = START;
			count = 'd0;
			start_count = 'd0;
			mul_start = 'd0;
			written_signals = 'd0;
		end
		
		START:begin
			start_count = 0;
			//mul_start = 'd0;
			if(avs_s1_write)begin
				if(avs_s1_address == 'd0)begin
					mul_op_1 = avs_s1_writedata;
					written_signals[0] = 'b1;
				end
				else if(avs_s1_address == 'd1)begin
					mul_op_2 = avs_s1_writedata;
					written_signals[1] = 'b1;
				end
				else if(avs_s1_address == 'd2)begin
					written_signals[2] = 'b1;
					//start_count = 'd1;
				end
				
				if(written_signals == 'b111)begin

					next_state = WAIT;
				end
				else begin

					next_state = START;
				end
			end
			else
				next_state = START;
			
		end
		
		WAIT:begin

			if(start_count == 0 && count < 11)begin
				start_count = 1;
				mul_start = 'd1;
				next_state = WAIT;
			end
			else if(count >= 11)begin
				start_count = 0;
				next_state = END;
			end
				
		end
		
		END:begin
			written_signals = 'd0;
			next_state = START;
			mul_start = 'd0;
		end
		
		default:
			next_state = INIT;
	endcase
end

always @(posedge clk)begin

	current_state = next_state;
	if(start_count == 1)
		count = count + 1;
	else
		count = 0;
end

assign op1 = mul_op_1;
assign op2 = mul_op_2;

assign mul_start_out = mul_start;
assign avs_s1_waitrequest = mul_start[0];
assign avs_s1_readdata = (written_signals == 'd7) ? start_count:mul_result_in;
endmodule

