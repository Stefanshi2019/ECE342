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
	
	output [IW-1:0] o_tb_regs [0:REGS-1] // 32 regs of 32bits
);


	
	wire [IW-1:0] immediate_data_D, reg_data1_D, reg_data2_D;
	
	wire pc_inc, pc_branch, pc_jumplink;
	wire [IW-1:0] offset, jumpval;
	
	// variables for pipeline
	wire [IW-1:0] pc_F, pc_D, pc_E, pc_W;
	wire [IW-1:0] instruction_D, instruction_E, instruction_W;
	wire [IW-1:0] reg_data1_E, reg_data2_E, immediate_data_E;
//	wire [IW-1:0] md1, md2;
	wire reg_write_E, reg_write_W;
	wire [IW-1:0] reg_write_data_E, reg_write_data_W;
	assign instruction_D = i_pc_rddata;
	
	
	wire [4:0] rs1 = instruction_D[19:15];
	wire [4:0] rs2 = instruction_D[24:20];
	
	// rd for write back stage
	wire [4:0] rd_E = instruction_E[11:7];
	wire [4:0] rd_W = instruction_W[11:7];
	
	// rs1 and rs2 for forwarding
	logic [IW-1:0] reg_data1_fwd;
	logic [IW-1:0] reg_data2_fwd;
	
	// jal flush signal
	wire brch = pc_branch|| pc_jumplink;
	reg o_pc_rd_reg;
	always@(*)begin
		if(reset)
			o_pc_rd_reg = 1'b0;
		else
			o_pc_rd_reg = 1'b1;
	end
	assign o_pc_rd = o_pc_rd_reg;
	assign o_pc_addr = pc_F;
	//assign reg_write_E[IW-1:1] = {31{1'b0}};
	//assign reg_write_W[IW-1:1] = {31{1'b0}};
	reg [2:0] valid_bits;
	
	always_ff @(posedge clk)begin
		if(reset)
			valid_bits <= 3'b111;
		else if(brch) begin
			valid_bits <= 3'b1;
		end
		else begin
			valid_bits[0] <= 'b1;
			valid_bits[1] <= valid_bits[0];
			valid_bits[2] <= valid_bits[1];
		end
		
	end
	
	reg [2:0] count;
	reg alu_enable;
	always@(posedge clk)begin
		if(reset)
			count = 'd2;
		else if(count == 'd2 && brch)
			count = 'd0;
		else if(count < 'd2)
			count = count + 'd1;
		
		if(count == 'd2)
			alu_enable = 1'b1;
		else
			alu_enable = 1'b0;
	end
	
	wire [IW-1:0] reg_write_data;
	
	RegisterBank reg_bank( .clk(clk), .reset(reset), .write(reg_write_W), .addr1(rs1),
										.addr2(rs2), .addr3(rd_W), 
										.input_data(reg_write_data), .output_data1(reg_data1_D),
										.output_data2(reg_data2_D), .o_tb_regs(o_tb_regs));
									
	
	IMMMultiplexer immmux(.instruction(instruction_D), .output_data(immediate_data_D));
	
	ALU_2 alu2(.clk(clk), .reset(reset), .enable(alu_enable), .instruction(instruction_E), .old_pc(pc_E), .rs1(reg_data1_fwd), .rs2(reg_data2_fwd),
				.immed(immediate_data_E), .output_data(reg_write_data_E), .PC_inc(pc_inc), .PC_branch(pc_branch), .PC_jumplink(pc_jumplink), 
				.RegWrite(reg_write_E),  .o_pc_offset(offset), .o_pc_jumpval(jumpval),
				.o_ldst_addr(o_ldst_addr), .o_ldst_rd(o_ldst_rd), .o_ldst_wr(o_ldst_wr), .o_ldst_wrdata(o_ldst_wrdata),
				.o_ldst_byte_en(o_ldst_byte_en));
				// insert here for branch and ldst
	
//	module PCRegister #(parameter IW = 32)(
//	input clk,
//	input reset,
//	input [IW-1:0] new_PC,
//	output reg [IW-1:0] PC
//);
	wire [IW-1:0] pc_E_out;
	// this is PC_F		
	PCRegister pc_reg(.clk(clk), .reset(reset), .new_PC(pc_E_out), .PC(pc_F));
	// decode state						
	RegPC	PC_D(.clk(clk), .reset(reset), .enable(1'b1), .data_in(pc_F), .data_out(pc_D));
	// Execute state
	Reg	RS1(.clk(clk), .reset(reset), .enable(1'b1), .data_in(reg_data1_D), .data_out(reg_data1_E));
	Reg	RS2(.clk(clk), .reset(reset), .enable(1'b1), .data_in(reg_data2_D), .data_out(reg_data2_E));
	Reg   IMM(.clk(clk), .reset(reset), .enable(1'b1), .data_in(immediate_data_D), .data_out(immediate_data_E));

	RegPC   PC_E(.clk(clk), .reset(reset), .enable(1'b1), .data_in(pc_D), .data_out(pc_E));
	Reg	IR_E(.clk(clk), .reset(reset), .enable(1'b1), .data_in(instruction_D), .data_out(instruction_E));

	// write back state
	RegPC   PC_W(.clk(clk), .reset(reset), .enable(1'b1), .data_in(pc_E), .data_out(pc_W));
	Reg	IR_W(.clk(clk), .reset(reset), .enable(1'b1), .data_in(instruction_E), .data_out(instruction_W));
	Reg	REG_WRITE_DATA(.clk(clk), .reset(reset), .enable(1'b1), .data_in(reg_write_data_E), .data_out(reg_write_data_W));
	Reg	REG_WRITE(.clk(clk), .reset(reset), .enable(1'b1), .data_in(reg_write_E), .data_out(reg_write_W));

	// memory load
	assign reg_write_data = (instruction_W[6:0] == 7'b0000011) ? i_ldst_rddata : reg_write_data_W;
	// memory store
	
	// forwarding logic
	assign reg_data1_fwd = (reg_write_W == 1'b1 && rd_W == instruction_E[19:15] && rd_W != 'h0) ? reg_write_data : reg_data1_E;
	assign reg_data2_fwd = (reg_write_W == 1'b1 && rd_W == instruction_E[24:20] && rd_W != 'h0) ? reg_write_data : reg_data2_E;
	
	PC_adder pc_adder(.clk(clk), .reset(reset), .PC_inc(pc_inc), .PC_branch(pc_branch), .PC_jumplink(pc_jumplink),
							.pc_offset(offset), .pc_newval(jumpval), .PC_E(pc_E), .PC_F(pc_F), .PC(pc_E_out));
//	input clk,
//	input reset,
//	input PC_inc,
//	input PC_branch,
//	input PC_jumplink,
//	input [IW-1:0] pc_offset,
//	input [IW-1:0] pc_newval,
//	input [IW-1:0] PC_E,
//	input [IW-1:0] PC_F,
//	output [IW-1:0] PC
//);

	// 
	
//	wire [IW-1:0] o_ldst_addr_in;
//	wire o_ldst_rd_in;
//	wire o_ldst_wr_in;
//	wire [IW-1:0] o_ldst_wrdata_in;
//	wire [3:0] o_ldst_byte_en_in;
//	
//	wire [IW-1:0] o_ldst_addr_out;
//	wire o_ldst_rd_out;
//	wire o_ldst_wr_out;
//	wire [IW-1:0] o_ldst_wrdata_out;
//	wire [3:0] o_ldst_byte_en_out;
	
//	module MemProc #(parameter IW = 32)(
//	input clk,
//	input reset,
//	input enable,
//	input [IW-1:0] o_ldst_addr_in,
//	input o_ldst_rd_in,
//	input o_ldst_wr_in,
//	input [IW-1:0] o_ldst_wrdata_in,
//	input [3:0] o_ldst_byte_en_in,
//	
//	output logic [IW-1:0] o_ldst_addr_out,
//	output logic o_ldst_rd_out,
//	output logic o_ldst_wr_out,
//	output logic [IW-1:0] o_ldst_wrdata_out,
//	output logic [3:0] o_ldst_byte_en_out

//	output logic [IW-1:0] o_ldst_addr,
//	output logic o_ldst_rd,
//	output logic o_ldst_wr,
//	input [IW-1:0] i_ldst_rddata,
//	output logic [IW-1:0] o_ldst_wrdata,
//	output logic [3:0] o_ldst_byte_en

endmodule


module ALU_2 #(parameter IW = 32)(
	input clk,
	input reset,
	input enable,
	input [IW-1:0] instruction,
	input [IW-1:0] old_pc,
	input [IW-1:0] rs1,
	input [IW-1:0] rs2,
	input [IW-1:0] immed,

	output logic [IW-1:0] output_data,
	//output logic [IW-1:0] rd_data,
	output logic PC_inc,
	output logic PC_branch,
	output logic PC_jumplink,
	output logic RegWrite,
		
	//output logic o_pc_rd,
	output logic [IW-1:0] o_pc_offset,
	output logic [IW-1:0] o_pc_jumpval,

	output logic [IW-1:0] o_ldst_addr,
	output logic o_ldst_rd,
	output logic o_ldst_wr,
	output logic [IW-1:0] o_ldst_wrdata,
	output logic [3:0] o_ldst_byte_en,
	output logic mem_op
	
	);
	
	
	wire [6:0] opcode = instruction[6:0];
	wire [2:0] func3 = instruction[14:12];
	wire [6:0] func7 = instruction[31:25];
	
	

	always @(*) begin	
	
			RegWrite = 1'b0;
			PC_inc = 1'b0;
			PC_branch = 1'b0;
			PC_jumplink = 1'b0;
//			o_pc_addr = old_pc;
//			o_pc_rd = 1'b1;
			o_ldst_rd = 1'b0;
			o_ldst_wr = 1'b0;
			mem_op = 1'b0;
//		end
//		
//		alu_state:begin
		if(enable)begin
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
				//next_state = fet_state;
					
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
				//next_state = fet_state;
				
			end
			
			else if(opcode == 7'b0110111) begin //lui
				output_data = immed;
				PC_inc = 1'b1;
				RegWrite = 1'b1;
				//next_state = fet_state;
			end
			
			else if(opcode == 7'b0010111) begin //auipc
				output_data = old_pc + (immed);
				PC_inc = 1'b1;
				RegWrite = 1'b1;
				//next_state = fet_state;
			end
			
			else if(opcode == 7'b1100011) begin
				case(func3) 
				3'h0: begin //beq
						PC_branch = ($signed(rs1) == $signed(rs2)) ? 1'b1 : 1'b0;
						PC_inc = ~PC_branch;
						o_pc_offset = immed;
				end
				3'h1: begin //bne
					PC_branch = ($signed(rs1) != $signed(rs2)) ? 1'b1 : 1'b0;
					PC_inc = ~PC_branch;
					o_pc_offset = immed;
				end
				3'h4: begin //blt
					PC_branch = ($signed(rs1) < $signed(rs2)) ? 1'b1 : 1'b0;
					PC_inc = ~PC_branch;
					o_pc_offset = immed;
				end
				3'h5:begin //bge
					PC_branch = ($signed(rs1) >= $signed(rs2)) ? 1'b1 : 1'b0;
					PC_inc = ~PC_branch;
					o_pc_offset = immed;
				end
				3'h6: begin //bltu
					PC_branch = (rs1 < rs2) ? 1'b1 : 1'b0;
					PC_inc = ~PC_branch;
					o_pc_offset = {{20{1'b0}}, immed[11:0]};
				end
				3'h7: begin //bgeu
					PC_branch = (rs1 >= rs2) ? 1'b1 : 1'b0;
					PC_inc = ~PC_branch;
					o_pc_offset = {{20{1'b0}}, immed[11:0]};
				end
				default:begin
					PC_branch = 1'b0;
					PC_inc = ~PC_branch;
					o_pc_offset = {{20{1'b0}}, immed[11:0]};
				end
				
				endcase
				//next_state = fet_state;
			end
			
			else if(opcode == 7'b1101111)begin	//jal
				PC_branch = 1'b1;
				output_data = old_pc + 'd4;
				o_pc_offset = immed;
				RegWrite = 1'b1;
				//next_state = fet_state;
			end
			
			else if(opcode == 7'b1100111)begin	// jalr
				PC_jumplink = 1'b1;
				output_data = old_pc + 'd4;
				o_pc_jumpval = immed + rs1;
				RegWrite = 1'b1;
				//next_state = fet_state;
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
				default:begin
					o_ldst_addr = 'b0;
					o_ldst_rd = 1'b0;
					o_ldst_byte_en = 4'h1;
				end
				endcase
				mem_op = 1'b1;
				RegWrite = 1'b1;
				//next_state = mem_state;
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
				default:begin
					o_ldst_addr = 'b0;
					o_ldst_wr = 1'b0;
					o_ldst_wrdata = rs2;
					o_ldst_byte_en = 4'h1;
				end
				endcase
				mem_op = 1'b1;

				//next_state = mem_state;
			end
		end
		//end
		
//		mem_state:begin
//			if(opcode == 7'b0000011)begin
//				output_data = i_ldst_rddata;
//				RegWrite = 1'b1;
//			end
//			PC_inc = 1'b1;
//			next_state = fet_state;
//			
//		end
//		
//		default:
//			next_state = fet_state;
//		endcase
	end
	
	//assign rd_data = output_data;
	
			
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
	always @(*)begin
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
	always@(instruction, opcode)begin
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
		else
			IMMED = 0;
		
	end
	
	assign output_data = IMMED;
endmodule

module PC_adder #(parameter IW = 32)(
	input clk,
	input reset,
	input PC_inc,
	input PC_branch,
	input PC_jumplink,
	input [IW-1:0] pc_offset,
	input [IW-1:0] pc_newval,
	input [IW-1:0] PC_E,
	input [IW-1:0] PC_F,
	output logic [IW-1:0] PC
);
	always@(*)begin
		if(reset) 
			PC = 0;
		else if(PC_branch) 
			PC = PC_E + pc_offset; //not sure if need else if for PC_inc
		else if(PC_jumplink) 
			PC = pc_newval;
//		if(PC_inc) 
		else
			PC = PC_F + 4;
	end

endmodule

module PCRegister #(parameter IW = 32)(
	input clk,
	input reset,
	input [IW-1:0] new_PC,
	output reg [IW-1:0] PC
);

	always@ (posedge clk) begin
		if(reset) 
			PC <= 0;
//		else if(PC_branch) 
//			PC <= PC_E + pc_offset; //not sure if need else if for PC_inc
//		else if(PC_jumplink) 
//			PC <= pc_newval;
//		if(PC_inc) 
		else
			PC <= new_PC;
	end

endmodule

module Reg #(parameter IW = 32)(
	input clk,
	input reset,
	input enable,
	input [IW-1:0] data_in,
	output reg [IW-1:0] data_out
);

	always_ff @(posedge clk)begin
		if(reset)
			data_out <= 0;
			//valid_out = 0;
		
		else if(enable)
			data_out <= data_in;
			//valid_out <= valid_in;
		else
			data_out <= data_out;
	end


endmodule

module RegPC #(parameter IW = 32)(
	input clk,
	input reset,
	input enable,
	input [IW-1:0] data_in,
	output reg [IW-1:0] data_out
);

	always_ff @(posedge clk)begin
		if(reset)
			data_out <= {32{1'b1}};
			//valid_out = 0;
		
		else if(enable)
			data_out <= data_in;
			//valid_out <= valid_in;
		else
			data_out <= data_out;
	end


endmodule

module MemProc #(parameter IW = 32)(
	input clk,
	input reset,
	input enable,
	input [IW-1:0] o_ldst_addr_in,
	input o_ldst_rd_in,
	input o_ldst_wr_in,
	input [IW-1:0] o_ldst_wrdata_in,
	input [3:0] o_ldst_byte_en_in,
	
	output logic [IW-1:0] o_ldst_addr_out,
	output logic o_ldst_rd_out,
	output logic o_ldst_wr_out,
	output logic [IW-1:0] o_ldst_wrdata_out,
	output logic [3:0] o_ldst_byte_en_out,
	output reg enable_sig
);
	always_ff @(posedge clk)begin
		enable_sig <= enable;
		if(enable)begin
			o_ldst_addr_out <= o_ldst_addr_in;
			o_ldst_rd_out <= o_ldst_rd_in;
			o_ldst_wr_out <= o_ldst_wr_in;
			o_ldst_wrdata_out <= o_ldst_wrdata_in;
			o_ldst_byte_en_out <= o_ldst_byte_en_in;
		end
		else begin
			o_ldst_addr_out <= 'b0;
			o_ldst_rd_out <= 'b0;
			o_ldst_wr_out <= 'b0;
			o_ldst_wrdata_out <= 'b0;
			o_ldst_byte_en_out <= 'b1;
		end
	end
	
endmodule
	
//	wire [IW-1:0] o_ldst_addr_out;
//	wire o_ldst_rd_out;
//	wire o_ldst_wr_out;
//	wire [IW-1:0] o_ldst_wrdata_out;
//	wire [3:0] o_ldst_byte_en_out;
//module RegPC_out #(parameter IW = 32)(
//	input clk,
//	input reset,
//	input enable,
//	input [IW-1:0] data_in,
//	output reg [IW-1:0] data_out
//);
//
//	always_ff @(posedge clk)begin
//		if(reset)
//			data_out <= {32{1'b1}};
//			//valid_out = 0;
//		
//		else if(enable)
//			data_out <= data_in;
//			//valid_out <= valid_in;
//		else
//			data_out <= data_out;
//	end
//
//
//endmodule