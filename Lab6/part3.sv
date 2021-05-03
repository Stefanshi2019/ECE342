module part3
(
    input                       clk,
    input				        reset
);

	
	parameter IW = 32; // instr width
	parameter REGS = 32; // number of registers
	// cpu signals
	logic [IW-1:0] pc_addr;
	logic pc_rd;
	logic [IW-1:0] pc_rddata;
	logic [3:0] pc_byte_en;
	logic [IW-1:0] ldst_addr;
	logic ldst_rd;
	logic ldst_wr;
	logic [IW-1:0] ldst_rddata;
	logic [IW-1:0] ldst_wrdata;
	logic [3:0] ldst_byte_en;
	logic [IW-1:0] tb_regs [0:REGS-1];
	logic ldst_waitrequest;
	
	logic mem_ldst_wr;
	logic [IW-1:0] i_readdata;		
	logic [7:0] LEDR_reg;
	
	logic fpm_rd, fpm_wr;
	logic [IW-1:0] fpm_readdata;
	
	cpu pweu(clk, reset, pc_addr, pc_rd, pc_rddata, 
				pc_byte_en, ldst_addr, ldst_rd, ldst_wr, 
				i_readdata, ldst_wrdata, ldst_byte_en, ldst_waitrequest, tb_regs);
	
	
	mem mem_inst( .clk(clk), .reset(reset), .p1_addr(ldst_addr[14:2]), .p1_write(mem_ldst_wr), .p1_read(ldst_rd),
					.p1_byteenable(ldst_byte_en), .p1_writedata(ldst_wrdata), .p1_readdata(ldst_rddata),
					.p2_addr(pc_addr[14:2]), .p2_read(pc_rd), .p2_byteenable(pc_byte_en), .p2_readdata(pc_rddata));
					
	avalon_fp_mult fpm( .clk(clk), .reset(reset), .avs_s1_address(ldst_addr[4:2]), .avs_s1_read(fpm_rd),
								.avs_s1_write(fpm_wr), .avs_s1_writedata(ldst_wrdata), .avs_s1_readdata(fpm_readdata),
								.avs_s1_waitrequest(ldst_waitrequest));
								
			
			

	always_comb begin
		fpm_wr = 1'b0;
		fpm_rd = 1'b0;
		mem_ldst_wr = 1'b0;
		if(ldst_rd)begin
			if(ldst_addr == 32'h0000A000 || ldst_addr == 32'h0000A004 || ldst_addr == 32'h0000A008 || ldst_addr == 32'h0000A00C)begin
				fpm_rd = 1'b1;
				i_readdata = fpm_readdata;
			end
			else begin
				i_readdata = ldst_rddata;
			end
		end
		
		if(ldst_wr)begin
			if(ldst_addr == 32'h0000A000 || ldst_addr == 32'h0000A004 || ldst_addr == 32'h0000A008 || ldst_addr == 32'h0000A00C)begin
				fpm_wr = 1'b1;
			end
			else
				mem_ldst_wr = 1'b1;
		end
	end
	assign LEDR = LEDR_reg;
	
	
endmodule




module cpu # (
	parameter IW = 32, // instr width
	parameter REGS = 32 // number of registers
)(
	input clk,
	input reset,
	
	// read only port
	output [IW-1:0] o_pc_addr,				// addr stored in pc
	output o_pc_rd,							// read pc
	input [IW-1:0] i_pc_rddata,			// instruction
	output [3:0] o_pc_byte_en,				// o_pc_byte_en is a byte enable signal for the pc memory port. -
													//	Asserting 4'b1111 will read a whole word, 4'b0011 will read a half word and 4'b0001
													//	will read a byte from address o_pc_addr 
													//(if o_pc_rd is also asserted).
	
	// read/write port
	output [IW-1:0] o_ldst_addr,
	output o_ldst_rd,
	output o_ldst_wr,
	input [IW-1:0] i_ldst_rddata,

	output [IW-1:0] o_ldst_wrdata,
	output [3:0] o_ldst_byte_en,				// 
	input i_ldst_waitrequest,
	output [IW-1:0] o_tb_regs [0:REGS-1] // 32 regs of 32bits
);

	wire [4:0] rs1 = i_pc_rddata[19:15];
	wire [4:0] rs2 = i_pc_rddata[24:20];
	wire [4:0] rd = i_pc_rddata[11:7];
	
	wire [IW-1:0] immediate_data, reg_data1, reg_data2, reg_write_data;
	
	wire pc_inc, pc_branch, pc_jumplink;
	wire [IW-1:0] offset, jumpval, pc;
	RegisterBank reg_bank( .clk(clk), .reset(reset), .write(reg_write), .addr1(rs1),
										.addr2(rs2), .addr3(rd), 
										.input_data(reg_write_data), .output_data1(reg_data1),
										.output_data2(reg_data2), .o_tb_regs(o_tb_regs));
									

	IMMMultiplexer immmux(.instruction(i_pc_rddata), .output_data(immediate_data));
	
	ALU_2 alu2(.clk(clk), .reset(reset), .instruction(i_pc_rddata), .old_pc(pc), .rs1(reg_data1), .rs2(reg_data2),
				.immed(immediate_data), .rd_data(reg_write_data), .PC_inc(pc_inc), .PC_branch(pc_branch), .PC_jumplink(pc_jumplink), .RegWrite(reg_write),
				.o_pc_addr(o_pc_addr), .o_pc_rd(o_pc_rd), .o_pc_offset(offset), .o_pc_jumpval(jumpval),
				.o_ldst_addr(o_ldst_addr), .o_ldst_rd(o_ldst_rd), .o_ldst_wr(o_ldst_wr), .i_ldst_rddata(i_ldst_rddata), .o_ldst_wrdata(o_ldst_wrdata),
				.o_ldst_byte_en(o_ldst_byte_en), .i_ldst_waitrequest(i_ldst_waitrequest));
				// insert here for branch and ldst
				
	PCRegister pc_reg(.clk(clk), .reset(reset), .PC_inc(pc_inc), .PC_branch(pc_branch), .PC_jumplink(pc_jumplink),
							.pc_offset(offset), .pc_newval(jumpval), .PC(pc));
							
	assign o_pc_byte_en = 4'b1111;
//	input clk,
//	input reset,
//	input PC_inc,
//	input PC_branch,
//	input PC_change,
//	input [IW-1:0] pc_offset,
//	input [IW-1:0] pc_newval,
//	output logic [IW-1:0] PC
//	
endmodule

module ALU_2 #(parameter IW = 32)(
	input clk,
	input reset,
	
	input [IW-1:0] instruction,
	input [IW-1:0] old_pc,
	input [IW-1:0] rs1,
	input [IW-1:0] rs2,
	input [IW-1:0] immed,
	
	output logic [IW-1:0] rd_data,
	output logic PC_inc,
	output logic PC_branch,
	output logic PC_jumplink,
	output logic RegWrite,
		
	output logic [IW-1:0] o_pc_addr,
	output logic o_pc_rd,
	output logic [IW-1:0] o_pc_offset,
	output logic [IW-1:0] o_pc_jumpval,

	output logic [IW-1:0] o_ldst_addr,
	output logic o_ldst_rd,
	output logic o_ldst_wr,
	input [IW-1:0] i_ldst_rddata,
	output logic [IW-1:0] o_ldst_wrdata,
	output logic [3:0] o_ldst_byte_en,
	input i_ldst_waitrequest
	);
	
	parameter fet_state = 3'b00, alu_state = 3'b01, wait_state = 3'b10, mem_state = 3'b11;
	reg [2:0] curr_state, next_state;
	
	always @(posedge clk)begin
		if(reset)
			curr_state <= fet_state;
		else
			curr_state <= next_state;
	end
	
	wire [6:0] opcode = instruction[6:0];
	wire [2:0] func3 = instruction[14:12];
	wire [6:0] func7 = instruction[31:25];
	
	reg [IW-1:0] output_data;
	always_comb begin
		case(curr_state)
		fet_state: begin
			next_state = alu_state;
		
			RegWrite = 1'b0;
			PC_inc = 1'b0;
			PC_branch = 1'b0;
			PC_jumplink = 1'b0;
			o_pc_addr = old_pc;
			o_pc_rd = 1'b1;
			o_ldst_rd = 1'b0;
			o_ldst_wr = 1'b0;
		end
		
		alu_state:begin
			if(opcode == 7'b0110011)begin
				case(func3)
				// add or subtract
					3'h0: begin
						if(func7 == 7'h0)
							output_data = rs1 + rs2;
						else if(func7 == 7'h20)
							output_data = rs1 - rs2;
					end
					3'h4: output_data = rs1 ^ rs2;
					3'h6: output_data = rs1 | rs2;
					3'h7: output_data = rs1 & rs2;
					3'h1: output_data = rs1 << rs2[4:0];
					3'h5: begin
						// shift right logical
						if(func7 == 7'h0)
							output_data = rs1 >> rs2[4:0]; 
						// shift right arithmetic
						else if(func7 == 7'h20)
							output_data = rs1 >>> rs2[4:0];
							
					end 	
					
					// set less than signed
					3'h2: output_data = ($signed(rs1) < $signed(rs2)) ? 1:0; 
					
					3'h3: output_data = (rs1 < rs2) ? 1:0;
					default: output_data = 'b0;
				endcase
				
				PC_inc = 1'b1;
				RegWrite = 1'b1;
				next_state = fet_state;
					
			end
			
			else if(opcode == 7'b0010011)begin
				case(func3)
					// add or subtract
					3'h0: output_data = rs1 + immed;
					
					// xor
					3'h4: output_data = rs1 ^ immed;
					
					// or
					3'h6: output_data = rs1 | immed;
					
					// and
					3'h7: output_data = rs1 & immed;
					
					// shift left logical
					3'h1: output_data = rs1 << immed[4:0];
					

					3'h5: begin
						// shift right logical
						if(func7 == 7'h0)
							output_data = rs1 >> immed[4:0]; 
						// shift right arithmetic
						else if(func7 == 7'h20)
							output_data = rs1 >>> immed[4:0];
							
					end 	
					
					// set less than signed
					3'h2: output_data = ($signed(rs1) < $signed(immed)) ? 1:0; 
					
					3'h3: output_data = (rs1 < immed) ? 1:0;
					
					default: output_data = 'b0;
				endcase
				
				PC_inc = 1'b1;
				RegWrite = 1'b1;
				next_state = fet_state;
				
			end
			
			else if(opcode == 7'b0110111) begin //lui
				output_data = immed;
				PC_inc = 1'b1;
				RegWrite = 1'b1;
				next_state = fet_state;
			end
			
			else if(opcode == 7'b0010111) begin //auipc
				output_data = old_pc + (immed);
				PC_inc = 1'b1;
				RegWrite = 1'b1;
				next_state = fet_state;
			end
			
			else if(opcode == 7'b1100011) begin
				case(func3) 
				3'h0: begin //beq
						PC_branch = ($signed(rs1) == $signed(rs2)) ? 1 : 0;
						PC_inc = ~PC_branch;
						o_pc_offset = immed;
				end
				3'h1: begin //bne
					PC_branch = ($signed(rs1) != $signed(rs2)) ? 1 : 0;
					PC_inc = ~PC_branch;
					o_pc_offset = immed;
				end
				3'h4: begin //blt
					PC_branch = ($signed(rs1) < $signed(rs2)) ? 1 : 0;
					PC_inc = ~PC_branch;
					o_pc_offset = immed;
				end
				3'h5:begin //bge
					PC_branch = ($signed(rs1) >= $signed(rs2)) ? 1 : 0;
					PC_inc = ~PC_branch;
					o_pc_offset = immed;
				end
				3'h6: begin //bltu
					PC_branch = (rs1 < rs2) ? 1 : 0;
					PC_inc = ~PC_branch;
					o_pc_offset = {{20{1'b0}}, immed[11:0]};
				end
				3'h7: begin //bgeu
					PC_branch = (rs1 >= rs2) ? 1 : 0;
					PC_inc = ~PC_branch;
					o_pc_offset = {{20{1'b0}}, immed[11:0]};
				end
				endcase
				next_state = fet_state;
			end
			
			else if(opcode == 7'b1101111)begin	//jal
				PC_branch = 1'b1;
				output_data = old_pc + 'd4;
				o_pc_offset = immed;
				RegWrite = 1'b1;
				next_state = fet_state;
			end
			
			else if(opcode == 7'b1100111)begin
				PC_jumplink = 1'b1;
				output_data = old_pc + 'd4;
				o_pc_jumpval = immed + rs1;
				RegWrite = 1'b1;
				next_state = fet_state;
			end
			
			else if(opcode == 7'b0000011)begin	//load
				case (func3)
				3'h0: begin
					o_ldst_addr = rs1 + immed;
					o_ldst_rd = 1'b1;
					o_ldst_byte_en = 4'h1;
				end
				
				3'h1: begin
					o_ldst_addr = rs1 + immed;
					o_ldst_rd = 1'b1;
					o_ldst_byte_en = 4'h3;
				end
				
				3'h2: begin
					o_ldst_addr = rs1 + immed;
					o_ldst_rd = 1'b1;
					o_ldst_byte_en = 4'hf;
				end
				
				
				3'h4: begin
					o_ldst_addr = {{20{1'b0}}, immed[11:0]};
					o_ldst_rd = 1'b1;
					o_ldst_byte_en = 4'h1;
				end
				
				3'h5: begin
					o_ldst_addr = {{20{1'b0}}, immed[11:0]};
					o_ldst_rd = 1'b1;
					o_ldst_byte_en = 4'h3;
				end
				endcase
				
				next_state = mem_state;
			end
			
			else if(opcode == 7'b0100011)begin	//store
				case(func3)
				3'h0: begin //sb
					o_ldst_addr = rs1 + immed;
					o_ldst_wr = 1'b1;
					o_ldst_wrdata = rs2;
					o_ldst_byte_en = 4'h1;
				end
				3'h1: begin //sh
					o_ldst_addr = rs1 + immed;
					o_ldst_wr = 1'b1;
					o_ldst_wrdata = rs2;
					o_ldst_byte_en = 4'h3;
				end
				3'h2: begin //sw
					o_ldst_addr = rs1 + immed;
					o_ldst_wr = 1'b1;
					o_ldst_wrdata = rs2;
					o_ldst_byte_en = 4'hf;
				end
				endcase
				next_state = mem_state;
			end

		end
		wait_state:begin

		end
		mem_state:begin
			if(i_ldst_waitrequest == 1'b1)begin
					next_state = mem_state;
					PC_inc = 1'b0;
			end
			else begin
				if(opcode == 7'b0000011)begin
					output_data = i_ldst_rddata;
					RegWrite = 1'b1;
				end
				PC_inc = 1'b1;
				next_state = fet_state;
			end
			
		end
		
		default:
			next_state = fet_state;
		endcase
	end
	
	assign rd_data = output_data;
	
			
endmodule



module RegisterBank #(parameter IW = 32, parameter REGS = 32)(
	input clk,
	input reset,
	input write,
	input [4:0] addr1,
	input [4:0] addr2,
	input [4:0] addr3,
	input [IW-1:0] input_data,
	output [IW-1:0] output_data1,
	output[IW-1:0] output_data2,
	output [IW-1:0] o_tb_regs [0:REGS-1]
//	output logic done_exe
	);
	reg[IW-1:0] regs [0:REGS-1];
	
	assign output_data1 = regs[addr1];
	assign output_data2 = regs[addr2];
	
	integer i;
	initial begin
		for(i=0; i < REGS; i=i+1)
			regs[i] = 0;
	end
	integer i2;
	always @(posedge clk)begin
		if(reset)begin
		
			for(i=0; i < REGS; i=i+1)
				regs[i] = 0;
		end

		else if(write)begin
			regs[addr3] = input_data;
			//done_exe = 1;
		end
		regs[0] = 'b0;
	end
	assign o_tb_regs = regs;
endmodule


// extracts immediate values according to instruction type
// all immediates are zero-extended
module IMMMultiplexer #(parameter IW = 32) (
	input [IW-1 : 0] instruction,
	output logic [IW-1 : 0] output_data
	);
	wire [6:0] opcode = instruction[6:0];
	reg [IW-1:0] IMMED;
	always@(instruction)begin
		// if type R, no immed, set immed to zero
		if(opcode == 7'b0110011)
			IMMED = 0;
		// if type I, 
		else if(opcode == 7'b0010011 || opcode == 7'b0000011 || opcode == 7'b1100111)
			IMMED = {{20{instruction[31]}}, instruction[31:20]};
		// if type S
		else if(opcode == 7'b0100011)
			IMMED = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
		// if type U
		else if(opcode == 7'b0110111 || opcode == 7'b0010111)
			IMMED = {instruction[31:12], {12{1'b0}}};
		// if type B
		else if(opcode == 7'b1100011)
			IMMED = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
		// if type J
		else if(opcode == 7'b1101111)
			IMMED = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
		
	end
	
	assign output_data = IMMED;
endmodule



module PCRegister #(parameter IW = 32)(
	input clk,
	input reset,
	input PC_inc,
	input PC_branch,
	input PC_jumplink,
	input [IW-1:0] pc_offset,
	input [IW-1:0] pc_newval,
	output logic [IW-1:0] PC
);

	always@ (posedge clk) begin
		if(reset) 
			PC <= 0;
		if(PC_branch) 
			PC <= PC + pc_offset; //not sure if need else if for PC_inc
		if(PC_jumplink) 
			PC <= pc_newval;
		if(PC_inc) 
			PC <= PC + 4;
	end

endmodule




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





// megafunction wizard: %ALTFP_MULT%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: ALTFP_MULT 

// ============================================================
// File Name: fp_mult.v
// Megafunction Name(s):
// 			ALTFP_MULT
//
// Simulation Library Files(s):
// 			lpm
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 18.0.0 Build 614 04/24/2018 SJ Lite Edition
// ************************************************************


//Copyright (C) 2018  Intel Corporation. All rights reserved.
//Your use of Intel Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Intel Program License 
//Subscription Agreement, the Intel Quartus Prime License Agreement,
//the Intel FPGA IP License Agreement, or other applicable license
//agreement, including, without limitation, that your use is for
//the sole purpose of programming logic devices manufactured by
//Intel and sold by Intel or its authorized distributors.  Please
//refer to the applicable agreement for further details.


//altfp_mult CBX_AUTO_BLACKBOX="ALL" DEDICATED_MULTIPLIER_CIRCUITRY="YES" DENORMAL_SUPPORT="NO" DEVICE_FAMILY="Cyclone V" EXCEPTION_HANDLING="NO" PIPELINE=11 REDUCED_FUNCTIONALITY="NO" ROUNDING="TO_NEAREST" WIDTH_EXP=8 WIDTH_MAN=23 aclr clk_en clock dataa datab nan overflow result underflow zero
//VERSION_BEGIN 18.0 cbx_alt_ded_mult_y 2018:04:24:18:04:18:SJ cbx_altbarrel_shift 2018:04:24:18:04:18:SJ cbx_altera_mult_add 2018:04:24:18:04:18:SJ cbx_altera_mult_add_rtl 2018:04:24:18:04:18:SJ cbx_altfp_mult 2018:04:24:18:04:18:SJ cbx_altmult_add 2018:04:24:18:04:18:SJ cbx_cycloneii 2018:04:24:18:04:18:SJ cbx_lpm_add_sub 2018:04:24:18:04:18:SJ cbx_lpm_compare 2018:04:24:18:04:18:SJ cbx_lpm_mult 2018:04:24:18:04:18:SJ cbx_mgl 2018:04:24:18:08:49:SJ cbx_nadder 2018:04:24:18:04:18:SJ cbx_padd 2018:04:24:18:04:18:SJ cbx_parallel_add 2018:04:24:18:04:18:SJ cbx_stratix 2018:04:24:18:04:18:SJ cbx_stratixii 2018:04:24:18:04:18:SJ cbx_util_mgl 2018:04:24:18:04:18:SJ  VERSION_END
// synthesis VERILOG_INPUT_VERSION VERILOG_2001
// altera message_off 10463


//synthesis_resources = lpm_add_sub 4 lpm_mult 1 reg 297 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fp_mult_altfp_mult_her
	( 
	aclr,
	clk_en,
	clock,
	dataa,
	datab,
	nan,
	overflow,
	result,
	underflow,
	zero) ;
	input   aclr;
	input   clk_en;
	input   clock;
	input   [31:0]  dataa;
	input   [31:0]  datab;
	output   nan;
	output   overflow;
	output   [31:0]  result;
	output   underflow;
	output   zero;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
	tri0   aclr;
	tri1   clk_en;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

	reg	dataa_exp_all_one_ff_p1;
	reg	dataa_exp_not_zero_ff_p1;
	reg	dataa_man_not_zero_ff_p1;
	reg	dataa_man_not_zero_ff_p2;
	reg	datab_exp_all_one_ff_p1;
	reg	datab_exp_not_zero_ff_p1;
	reg	datab_man_not_zero_ff_p1;
	reg	datab_man_not_zero_ff_p2;
	reg	[9:0]	delay_exp2_bias;
	reg	[9:0]	delay_exp3_bias;
	reg	[9:0]	delay_exp_bias;
	reg	delay_man_product_msb;
	reg	delay_man_product_msb2;
	reg	delay_man_product_msb_p0;
	reg	delay_man_product_msb_p1;
	reg	[23:0]	delay_round;
	reg	[8:0]	exp_add_p1;
	reg	[9:0]	exp_adj_p1;
	reg	[9:0]	exp_adj_p2;
	reg	[8:0]	exp_bias_p1;
	reg	[8:0]	exp_bias_p2;
	reg	[8:0]	exp_bias_p3;
	reg	[7:0]	exp_result_ff;
	reg	input_is_infinity_dffe_0;
	reg	input_is_infinity_dffe_1;
	reg	input_is_infinity_dffe_2;
	reg	input_is_infinity_dffe_3;
	reg	input_is_infinity_ff1;
	reg	input_is_infinity_ff2;
	reg	input_is_infinity_ff3;
	reg	input_is_infinity_ff4;
	reg	input_is_infinity_ff5;
	reg	input_is_nan_dffe_0;
	reg	input_is_nan_dffe_1;
	reg	input_is_nan_dffe_2;
	reg	input_is_nan_dffe_3;
	reg	input_is_nan_ff1;
	reg	input_is_nan_ff2;
	reg	input_is_nan_ff3;
	reg	input_is_nan_ff4;
	reg	input_is_nan_ff5;
	reg	input_not_zero_dffe_0;
	reg	input_not_zero_dffe_1;
	reg	input_not_zero_dffe_2;
	reg	input_not_zero_dffe_3;
	reg	input_not_zero_ff1;
	reg	input_not_zero_ff2;
	reg	input_not_zero_ff3;
	reg	input_not_zero_ff4;
	reg	input_not_zero_ff5;
	reg	lsb_dffe;
	reg	[22:0]	man_result_ff;
	reg	man_round_carry;
	reg	man_round_carry_p0;
	reg	[23:0]	man_round_p;
	reg	[23:0]	man_round_p0;
	reg	[23:0]	man_round_p1;
	reg	[24:0]	man_round_p2;
	reg	nan_ff;
	reg	overflow_ff;
	reg	round_dffe;
	reg	[0:0]	sign_node_ff0;
	reg	[0:0]	sign_node_ff1;
	reg	[0:0]	sign_node_ff2;
	reg	[0:0]	sign_node_ff3;
	reg	[0:0]	sign_node_ff4;
	reg	[0:0]	sign_node_ff5;
	reg	[0:0]	sign_node_ff6;
	reg	[0:0]	sign_node_ff7;
	reg	[0:0]	sign_node_ff8;
	reg	[0:0]	sign_node_ff9;
	reg	[0:0]	sign_node_ff10;
	reg	sticky_dffe;
	reg	underflow_ff;
	reg	zero_ff;
	wire  [8:0]   wire_exp_add_adder_result;
	wire  [9:0]   wire_exp_adj_adder_result;
	wire  [9:0]   wire_exp_bias_subtr_result;
	wire  [24:0]   wire_man_round_adder_result;
	wire  [47:0]   wire_man_product2_mult_result;
	wire  [9:0]  bias;
	wire  [7:0]  dataa_exp_all_one;
	wire  [7:0]  dataa_exp_not_zero;
	wire  [22:0]  dataa_man_not_zero;
	wire  [7:0]  datab_exp_all_one;
	wire  [7:0]  datab_exp_not_zero;
	wire  [22:0]  datab_man_not_zero;
	wire  exp_is_inf;
	wire  exp_is_zero;
	wire  [9:0]  expmod;
	wire  [7:0]  inf_num;
	wire  lsb_bit;
	wire  [24:0]  man_shift_full;
	wire  [7:0]  result_exp_all_one;
	wire  [8:0]  result_exp_not_zero;
	wire  round_bit;
	wire  round_carry;
	wire  [22:0]  sticky_bit;

	// synopsys translate_off
	initial
		dataa_exp_all_one_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_exp_all_one_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_exp_all_one_ff_p1 <= dataa_exp_all_one[7];
	// synopsys translate_off
	initial
		dataa_exp_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_exp_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_exp_not_zero_ff_p1 <= dataa_exp_not_zero[7];
	// synopsys translate_off
	initial
		dataa_man_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_man_not_zero_ff_p1 <= dataa_man_not_zero[10];
	// synopsys translate_off
	initial
		dataa_man_not_zero_ff_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_not_zero_ff_p2 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_man_not_zero_ff_p2 <= dataa_man_not_zero[22];
	// synopsys translate_off
	initial
		datab_exp_all_one_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_exp_all_one_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_exp_all_one_ff_p1 <= datab_exp_all_one[7];
	// synopsys translate_off
	initial
		datab_exp_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_exp_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_exp_not_zero_ff_p1 <= datab_exp_not_zero[7];
	// synopsys translate_off
	initial
		datab_man_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_man_not_zero_ff_p1 <= datab_man_not_zero[10];
	// synopsys translate_off
	initial
		datab_man_not_zero_ff_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_not_zero_ff_p2 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_man_not_zero_ff_p2 <= datab_man_not_zero[22];
	// synopsys translate_off
	initial
		delay_exp2_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp2_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp2_bias <= delay_exp_bias;
	// synopsys translate_off
	initial
		delay_exp3_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp3_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp3_bias <= delay_exp2_bias;
	// synopsys translate_off
	initial
		delay_exp_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp_bias <= wire_exp_bias_subtr_result;
	// synopsys translate_off
	initial
		delay_man_product_msb = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb <= delay_man_product_msb_p1;
	// synopsys translate_off
	initial
		delay_man_product_msb2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb2 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb2 <= delay_man_product_msb;
	// synopsys translate_off
	initial
		delay_man_product_msb_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb_p0 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb_p0 <= wire_man_product2_mult_result[47];
	// synopsys translate_off
	initial
		delay_man_product_msb_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb_p1 <= delay_man_product_msb_p0;
	// synopsys translate_off
	initial
		delay_round = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_round <= 24'b0;
		else if  (clk_en == 1'b1)   delay_round <= ((man_round_p2[23:0] & {24{(~ man_round_p2[24])}}) | (man_round_p2[24:1] & {24{man_round_p2[24]}}));
	// synopsys translate_off
	initial
		exp_add_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_add_p1 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_add_p1 <= wire_exp_add_adder_result;
	// synopsys translate_off
	initial
		exp_adj_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_p1 <= 10'b0;
		else if  (clk_en == 1'b1)   exp_adj_p1 <= delay_exp3_bias;
	// synopsys translate_off
	initial
		exp_adj_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_p2 <= 10'b0;
		else if  (clk_en == 1'b1)   exp_adj_p2 <= wire_exp_adj_adder_result;
	// synopsys translate_off
	initial
		exp_bias_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p1 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p1 <= exp_add_p1[8:0];
	// synopsys translate_off
	initial
		exp_bias_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p2 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p2 <= exp_bias_p1;
	// synopsys translate_off
	initial
		exp_bias_p3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p3 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p3 <= exp_bias_p2;
	// synopsys translate_off
	initial
		exp_result_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_result_ff <= 8'b0;
		else if  (clk_en == 1'b1)   exp_result_ff <= ((inf_num & {8{((exp_is_inf | input_is_infinity_ff5) | input_is_nan_ff5)}}) | ((exp_adj_p2[7:0] & {8{(~ exp_is_zero)}}) & {8{input_not_zero_ff5}}));
	// synopsys translate_off
	initial
		input_is_infinity_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_0 <= ((dataa_exp_all_one_ff_p1 & (~ (dataa_man_not_zero_ff_p1 | dataa_man_not_zero_ff_p2))) | (datab_exp_all_one_ff_p1 & (~ (datab_man_not_zero_ff_p1 | datab_man_not_zero_ff_p2))));
	// synopsys translate_off
	initial
		input_is_infinity_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_1 <= input_is_infinity_dffe_0;
	// synopsys translate_off
	initial
		input_is_infinity_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_2 <= input_is_infinity_dffe_1;
	// synopsys translate_off
	initial
		input_is_infinity_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_3 <= input_is_infinity_dffe_2;
	// synopsys translate_off
	initial
		input_is_infinity_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff1 <= input_is_infinity_dffe_3;
	// synopsys translate_off
	initial
		input_is_infinity_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff2 <= input_is_infinity_ff1;
	// synopsys translate_off
	initial
		input_is_infinity_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff3 <= input_is_infinity_ff2;
	// synopsys translate_off
	initial
		input_is_infinity_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff4 <= input_is_infinity_ff3;
	// synopsys translate_off
	initial
		input_is_infinity_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff5 <= input_is_infinity_ff4;
	// synopsys translate_off
	initial
		input_is_nan_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_0 <= ((dataa_exp_all_one_ff_p1 & (dataa_man_not_zero_ff_p1 | dataa_man_not_zero_ff_p2)) | (datab_exp_all_one_ff_p1 & (datab_man_not_zero_ff_p1 | datab_man_not_zero_ff_p2)));
	// synopsys translate_off
	initial
		input_is_nan_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_1 <= input_is_nan_dffe_0;
	// synopsys translate_off
	initial
		input_is_nan_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_2 <= input_is_nan_dffe_1;
	// synopsys translate_off
	initial
		input_is_nan_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_3 <= input_is_nan_dffe_2;
	// synopsys translate_off
	initial
		input_is_nan_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff1 <= input_is_nan_dffe_3;
	// synopsys translate_off
	initial
		input_is_nan_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff2 <= input_is_nan_ff1;
	// synopsys translate_off
	initial
		input_is_nan_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff3 <= input_is_nan_ff2;
	// synopsys translate_off
	initial
		input_is_nan_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff4 <= input_is_nan_ff3;
	// synopsys translate_off
	initial
		input_is_nan_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff5 <= input_is_nan_ff4;
	// synopsys translate_off
	initial
		input_not_zero_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_0 <= (dataa_exp_not_zero_ff_p1 & datab_exp_not_zero_ff_p1);
	// synopsys translate_off
	initial
		input_not_zero_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_1 <= input_not_zero_dffe_0;
	// synopsys translate_off
	initial
		input_not_zero_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_2 <= input_not_zero_dffe_1;
	// synopsys translate_off
	initial
		input_not_zero_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_3 <= input_not_zero_dffe_2;
	// synopsys translate_off
	initial
		input_not_zero_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff1 <= input_not_zero_dffe_3;
	// synopsys translate_off
	initial
		input_not_zero_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff2 <= input_not_zero_ff1;
	// synopsys translate_off
	initial
		input_not_zero_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff3 <= input_not_zero_ff2;
	// synopsys translate_off
	initial
		input_not_zero_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff4 <= input_not_zero_ff3;
	// synopsys translate_off
	initial
		input_not_zero_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff5 <= input_not_zero_ff4;
	// synopsys translate_off
	initial
		lsb_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) lsb_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   lsb_dffe <= lsb_bit;
	// synopsys translate_off
	initial
		man_result_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_result_ff <= 23'b0;
		else if  (clk_en == 1'b1)   man_result_ff <= {((((((delay_round[22] & input_not_zero_ff5) & (~ input_is_infinity_ff5)) & (~ exp_is_inf)) & (~ exp_is_zero)) | (input_is_infinity_ff5 & (~ input_not_zero_ff5))) | input_is_nan_ff5), (((((delay_round[21:0] & {22{input_not_zero_ff5}}) & {22{(~ input_is_infinity_ff5)}}) & {22{(~ exp_is_inf)}}) & {22{(~ exp_is_zero)}}) & {22{(~ input_is_nan_ff5)}})};
	// synopsys translate_off
	initial
		man_round_carry = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_carry <= 1'b0;
		else if  (clk_en == 1'b1)   man_round_carry <= man_round_carry_p0;
	// synopsys translate_off
	initial
		man_round_carry_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_carry_p0 <= 1'b0;
		else if  (clk_en == 1'b1)   man_round_carry_p0 <= round_carry;
	// synopsys translate_off
	initial
		man_round_p = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p <= man_shift_full[24:1];
	// synopsys translate_off
	initial
		man_round_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p0 <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p0 <= man_round_p;
	// synopsys translate_off
	initial
		man_round_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p1 <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p1 <= man_round_p0;
	// synopsys translate_off
	initial
		man_round_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p2 <= 25'b0;
		else if  (clk_en == 1'b1)   man_round_p2 <= wire_man_round_adder_result;
	// synopsys translate_off
	initial
		nan_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) nan_ff <= 1'b0;
		else if  (clk_en == 1'b1)   nan_ff <= (input_is_nan_ff5 | (input_is_infinity_ff5 & (~ input_not_zero_ff5)));
	// synopsys translate_off
	initial
		overflow_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) overflow_ff <= 1'b0;
		else if  (clk_en == 1'b1)   overflow_ff <= (((exp_is_inf | input_is_infinity_ff5) & (~ input_is_nan_ff5)) & (~ (input_is_infinity_ff5 & (~ input_not_zero_ff5))));
	// synopsys translate_off
	initial
		round_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) round_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   round_dffe <= round_bit;
	// synopsys translate_off
	initial
		sign_node_ff0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff0 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff0 <= (dataa[31] ^ datab[31]);
	// synopsys translate_off
	initial
		sign_node_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff1 <= sign_node_ff0[0:0];
	// synopsys translate_off
	initial
		sign_node_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff2 <= sign_node_ff1[0:0];
	// synopsys translate_off
	initial
		sign_node_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff3 <= sign_node_ff2[0:0];
	// synopsys translate_off
	initial
		sign_node_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff4 <= sign_node_ff3[0:0];
	// synopsys translate_off
	initial
		sign_node_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff5 <= sign_node_ff4[0:0];
	// synopsys translate_off
	initial
		sign_node_ff6 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff6 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff6 <= sign_node_ff5[0:0];
	// synopsys translate_off
	initial
		sign_node_ff7 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff7 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff7 <= sign_node_ff6[0:0];
	// synopsys translate_off
	initial
		sign_node_ff8 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff8 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff8 <= sign_node_ff7[0:0];
	// synopsys translate_off
	initial
		sign_node_ff9 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff9 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff9 <= sign_node_ff8[0:0];
	// synopsys translate_off
	initial
		sign_node_ff10 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff10 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff10 <= sign_node_ff9[0:0];
	// synopsys translate_off
	initial
		sticky_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_dffe <= sticky_bit[22];
	// synopsys translate_off
	initial
		underflow_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) underflow_ff <= 1'b0;
		else if  (clk_en == 1'b1)   underflow_ff <= (((exp_is_zero & input_not_zero_ff5) & (~ input_is_nan_ff5)) & (~ (input_is_infinity_ff5 & (~ input_not_zero_ff5))));
	// synopsys translate_off
	initial
		zero_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) zero_ff <= 1'b0;
		else if  (clk_en == 1'b1)   zero_ff <= (((exp_is_zero | (~ input_not_zero_ff5)) & (~ input_is_nan_ff5)) & (~ input_is_infinity_ff5));
	lpm_add_sub   exp_add_adder
	( 
	.aclr(aclr),
	.cin(1'b0),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa({1'b0, dataa[30:23]}),
	.datab({1'b0, datab[30:23]}),
	.overflow(),
	.result(wire_exp_add_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.add_sub(1'b1)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_add_adder.lpm_pipeline = 1,
		exp_add_adder.lpm_width = 9,
		exp_add_adder.lpm_type = "lpm_add_sub";
	lpm_add_sub   exp_adj_adder
	( 
	.cin(1'b0),
	.cout(),
	.dataa(exp_adj_p1),
	.datab({expmod[9:0]}),
	.overflow(),
	.result(wire_exp_adj_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_adj_adder.lpm_pipeline = 0,
		exp_adj_adder.lpm_width = 10,
		exp_adj_adder.lpm_type = "lpm_add_sub";
	lpm_add_sub   exp_bias_subtr
	( 
	.cout(),
	.dataa({1'b0, exp_bias_p3}),
	.datab({bias[9:0]}),
	.overflow(),
	.result(wire_exp_bias_subtr_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_bias_subtr.lpm_direction = "SUB",
		exp_bias_subtr.lpm_pipeline = 0,
		exp_bias_subtr.lpm_representation = "UNSIGNED",
		exp_bias_subtr.lpm_width = 10,
		exp_bias_subtr.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_round_adder
	( 
	.cout(),
	.dataa({1'b0, man_round_p1}),
	.datab({{24{1'b0}}, man_round_carry}),
	.overflow(),
	.result(wire_man_round_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_round_adder.lpm_pipeline = 0,
		man_round_adder.lpm_width = 25,
		man_round_adder.lpm_type = "lpm_add_sub";
	lpm_mult   man_product2_mult
	( 
	.aclr(aclr),
	.clken(clk_en),
	.clock(clock),
	.dataa({1'b1, dataa[22:0]}),
	.datab({1'b1, datab[22:0]}),
	.result(wire_man_product2_mult_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.sclr(1'b0),
	.sum({1{1'b0}})
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_product2_mult.lpm_pipeline = 5,
		man_product2_mult.lpm_representation = "UNSIGNED",
		man_product2_mult.lpm_widtha = 24,
		man_product2_mult.lpm_widthb = 24,
		man_product2_mult.lpm_widthp = 48,
		man_product2_mult.lpm_widths = 1,
		man_product2_mult.lpm_type = "lpm_mult",
		man_product2_mult.lpm_hint = "DEDICATED_MULTIPLIER_CIRCUITRY=YES";
	assign
		bias = {{3{1'b0}}, {7{1'b1}}},
		dataa_exp_all_one = {(dataa[30] & dataa_exp_all_one[6]), (dataa[29] & dataa_exp_all_one[5]), (dataa[28] & dataa_exp_all_one[4]), (dataa[27] & dataa_exp_all_one[3]), (dataa[26] & dataa_exp_all_one[2]), (dataa[25] & dataa_exp_all_one[1]), (dataa[24] & dataa_exp_all_one[0]), dataa[23]},
		dataa_exp_not_zero = {(dataa[30] | dataa_exp_not_zero[6]), (dataa[29] | dataa_exp_not_zero[5]), (dataa[28] | dataa_exp_not_zero[4]), (dataa[27] | dataa_exp_not_zero[3]), (dataa[26] | dataa_exp_not_zero[2]), (dataa[25] | dataa_exp_not_zero[1]), (dataa[24] | dataa_exp_not_zero[0]), dataa[23]},
		dataa_man_not_zero = {(dataa[22] | dataa_man_not_zero[21]), (dataa[21] | dataa_man_not_zero[20]), (dataa[20] | dataa_man_not_zero[19]), (dataa[19] | dataa_man_not_zero[18]), (dataa[18] | dataa_man_not_zero[17]), (dataa[17] | dataa_man_not_zero[16]), (dataa[16] | dataa_man_not_zero[15]), (dataa[15] | dataa_man_not_zero[14]), (dataa[14] | dataa_man_not_zero[13]), (dataa[13] | dataa_man_not_zero[12]), (dataa[12] | dataa_man_not_zero[11]), dataa[11], (dataa[10] | dataa_man_not_zero[9]), (dataa[9] | dataa_man_not_zero[8]), (dataa[8] | dataa_man_not_zero[7]), (dataa[7] | dataa_man_not_zero[6]), (dataa[6] | dataa_man_not_zero[5]), (dataa[5] | dataa_man_not_zero[4]), (dataa[4] | dataa_man_not_zero[3]), (dataa[3] | dataa_man_not_zero[2]), (dataa[2] | dataa_man_not_zero[1]), (dataa[1] | dataa_man_not_zero[0]), dataa[0]},
		datab_exp_all_one = {(datab[30] & datab_exp_all_one[6]), (datab[29] & datab_exp_all_one[5]), (datab[28] & datab_exp_all_one[4]), (datab[27] & datab_exp_all_one[3]), (datab[26] & datab_exp_all_one[2]), (datab[25] & datab_exp_all_one[1]), (datab[24] & datab_exp_all_one[0]), datab[23]},
		datab_exp_not_zero = {(datab[30] | datab_exp_not_zero[6]), (datab[29] | datab_exp_not_zero[5]), (datab[28] | datab_exp_not_zero[4]), (datab[27] | datab_exp_not_zero[3]), (datab[26] | datab_exp_not_zero[2]), (datab[25] | datab_exp_not_zero[1]), (datab[24] | datab_exp_not_zero[0]), datab[23]},
		datab_man_not_zero = {(datab[22] | datab_man_not_zero[21]), (datab[21] | datab_man_not_zero[20]), (datab[20] | datab_man_not_zero[19]), (datab[19] | datab_man_not_zero[18]), (datab[18] | datab_man_not_zero[17]), (datab[17] | datab_man_not_zero[16]), (datab[16] | datab_man_not_zero[15]), (datab[15] | datab_man_not_zero[14]), (datab[14] | datab_man_not_zero[13]), (datab[13] | datab_man_not_zero[12]), (datab[12] | datab_man_not_zero[11]), datab[11], (datab[10] | datab_man_not_zero[9]), (datab[9] | datab_man_not_zero[8]), (datab[8] | datab_man_not_zero[7]), (datab[7] | datab_man_not_zero[6]), (datab[6] | datab_man_not_zero[5]), (datab[5] | datab_man_not_zero[4]), (datab[4] | datab_man_not_zero[3]), (datab[3] | datab_man_not_zero[2]), (datab[2] | datab_man_not_zero[1]), (datab[1] | datab_man_not_zero[0]), datab[0]},
		exp_is_inf = (((~ exp_adj_p2[9]) & exp_adj_p2[8]) | ((~ exp_adj_p2[8]) & result_exp_all_one[7])),
		exp_is_zero = (exp_adj_p2[9] | (~ result_exp_not_zero[8])),
		expmod = {{8{1'b0}}, (delay_man_product_msb2 & man_round_p2[24]), (delay_man_product_msb2 ^ man_round_p2[24])},
		inf_num = {8{1'b1}},
		lsb_bit = man_shift_full[1],
		man_shift_full = ((wire_man_product2_mult_result[46:22] & {25{(~ wire_man_product2_mult_result[47])}}) | (wire_man_product2_mult_result[47:23] & {25{wire_man_product2_mult_result[47]}})),
		nan = nan_ff,
		overflow = overflow_ff,
		result = {sign_node_ff10[0:0], exp_result_ff[7:0], man_result_ff[22:0]},
		result_exp_all_one = {(result_exp_all_one[6] & exp_adj_p2[7]), (result_exp_all_one[5] & exp_adj_p2[6]), (result_exp_all_one[4] & exp_adj_p2[5]), (result_exp_all_one[3] & exp_adj_p2[4]), (result_exp_all_one[2] & exp_adj_p2[3]), (result_exp_all_one[1] & exp_adj_p2[2]), (result_exp_all_one[0] & exp_adj_p2[1]), exp_adj_p2[0]},
		result_exp_not_zero = {(result_exp_not_zero[7] | exp_adj_p2[8]), (result_exp_not_zero[6] | exp_adj_p2[7]), (result_exp_not_zero[5] | exp_adj_p2[6]), (result_exp_not_zero[4] | exp_adj_p2[5]), (result_exp_not_zero[3] | exp_adj_p2[4]), (result_exp_not_zero[2] | exp_adj_p2[3]), (result_exp_not_zero[1] | exp_adj_p2[2]), (result_exp_not_zero[0] | exp_adj_p2[1]), exp_adj_p2[0]},
		round_bit = man_shift_full[0],
		round_carry = (round_dffe & (lsb_dffe | sticky_dffe)),
		sticky_bit = {(sticky_bit[21] | (wire_man_product2_mult_result[47] & wire_man_product2_mult_result[22])), (sticky_bit[20] | wire_man_product2_mult_result[21]), (sticky_bit[19] | wire_man_product2_mult_result[20]), (sticky_bit[18] | wire_man_product2_mult_result[19]), (sticky_bit[17] | wire_man_product2_mult_result[18]), (sticky_bit[16] | wire_man_product2_mult_result[17]), (sticky_bit[15] | wire_man_product2_mult_result[16]), (sticky_bit[14] | wire_man_product2_mult_result[15]), (sticky_bit[13] | wire_man_product2_mult_result[14]), (sticky_bit[12] | wire_man_product2_mult_result[13]), (sticky_bit[11] | wire_man_product2_mult_result[12]), (sticky_bit[10] | wire_man_product2_mult_result[11]), (sticky_bit[9] | wire_man_product2_mult_result[10]), (sticky_bit[8] | wire_man_product2_mult_result[9]), (sticky_bit[7] | wire_man_product2_mult_result[8]), (sticky_bit[6] | wire_man_product2_mult_result[7]), (sticky_bit[5] | wire_man_product2_mult_result[6]), (sticky_bit[4] | wire_man_product2_mult_result[5]), (sticky_bit[3] | wire_man_product2_mult_result[4]), (sticky_bit[2] | wire_man_product2_mult_result[3]), (sticky_bit[1] | wire_man_product2_mult_result[2]), (sticky_bit[0] | wire_man_product2_mult_result[1]), wire_man_product2_mult_result[0]},
		underflow = underflow_ff,
		zero = zero_ff;
endmodule //fp_mult_altfp_mult_her
//VALID FILE


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module fp_mult (
	aclr,
	clk_en,
	clock,
	dataa,
	datab,
	nan,
	overflow,
	result,
	underflow,
	zero);

	input	  aclr;
	input	  clk_en;
	input	  clock;
	input	[31:0]  dataa;
	input	[31:0]  datab;
	output	  nan;
	output	  overflow;
	output	[31:0]  result;
	output	  underflow;
	output	  zero;

	wire  sub_wire0;
	wire  sub_wire1;
	wire [31:0] sub_wire2;
	wire  sub_wire3;
	wire  sub_wire4;
	wire  nan = sub_wire0;
	wire  overflow = sub_wire1;
	wire [31:0] result = sub_wire2[31:0];
	wire  underflow = sub_wire3;
	wire  zero = sub_wire4;

	fp_mult_altfp_mult_her	fp_mult_altfp_mult_her_component (
				.aclr (aclr),
				.clk_en (clk_en),
				.clock (clock),
				.dataa (dataa),
				.datab (datab),
				.nan (sub_wire0),
				.overflow (sub_wire1),
				.result (sub_wire2),
				.underflow (sub_wire3),
				.zero (sub_wire4));

endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: PRIVATE: FPM_FORMAT STRING "Single"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone V"
// Retrieval info: CONSTANT: DEDICATED_MULTIPLIER_CIRCUITRY STRING "YES"
// Retrieval info: CONSTANT: DENORMAL_SUPPORT STRING "NO"
// Retrieval info: CONSTANT: EXCEPTION_HANDLING STRING "NO"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "UNUSED"
// Retrieval info: CONSTANT: LPM_HINT STRING "UNUSED"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altfp_mult"
// Retrieval info: CONSTANT: PIPELINE NUMERIC "11"
// Retrieval info: CONSTANT: REDUCED_FUNCTIONALITY STRING "NO"
// Retrieval info: CONSTANT: ROUNDING STRING "TO_NEAREST"
// Retrieval info: CONSTANT: WIDTH_EXP NUMERIC "8"
// Retrieval info: CONSTANT: WIDTH_MAN NUMERIC "23"
// Retrieval info: USED_PORT: aclr 0 0 0 0 INPUT NODEFVAL "aclr"
// Retrieval info: CONNECT: @aclr 0 0 0 0 aclr 0 0 0 0
// Retrieval info: USED_PORT: clk_en 0 0 0 0 INPUT NODEFVAL "clk_en"
// Retrieval info: CONNECT: @clk_en 0 0 0 0 clk_en 0 0 0 0
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT NODEFVAL "clock"
// Retrieval info: CONNECT: @clock 0 0 0 0 clock 0 0 0 0
// Retrieval info: USED_PORT: dataa 0 0 32 0 INPUT NODEFVAL "dataa[31..0]"
// Retrieval info: CONNECT: @dataa 0 0 32 0 dataa 0 0 32 0
// Retrieval info: USED_PORT: datab 0 0 32 0 INPUT NODEFVAL "datab[31..0]"
// Retrieval info: CONNECT: @datab 0 0 32 0 datab 0 0 32 0
// Retrieval info: USED_PORT: nan 0 0 0 0 OUTPUT NODEFVAL "nan"
// Retrieval info: CONNECT: nan 0 0 0 0 @nan 0 0 0 0
// Retrieval info: USED_PORT: overflow 0 0 0 0 OUTPUT NODEFVAL "overflow"
// Retrieval info: CONNECT: overflow 0 0 0 0 @overflow 0 0 0 0
// Retrieval info: USED_PORT: result 0 0 32 0 OUTPUT NODEFVAL "result[31..0]"
// Retrieval info: CONNECT: result 0 0 32 0 @result 0 0 32 0
// Retrieval info: USED_PORT: underflow 0 0 0 0 OUTPUT NODEFVAL "underflow"
// Retrieval info: CONNECT: underflow 0 0 0 0 @underflow 0 0 0 0
// Retrieval info: USED_PORT: zero 0 0 0 0 OUTPUT NODEFVAL "zero"
// Retrieval info: CONNECT: zero 0 0 0 0 @zero 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.v TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.qip TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.bsf TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult_inst.v TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult_bb.v TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.inc TRUE TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL fp_mult.cmp TRUE TRUE
// Retrieval info: LIB_FILE: lpm


