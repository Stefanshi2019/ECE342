module part3
(
    input                       clk,
    input				        reset
);

	

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