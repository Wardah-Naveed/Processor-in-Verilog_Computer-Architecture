`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/24/2023 03:37:38 AM
// Design Name: 
// Module Name: pipelined_register
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module MIPS_Processor(
  input wire clk, input wire reset
);
  // Submodule instances and wires
  wire [31:0] pc_count;
  wire [31:0] instruction;
  wire [31:0] if_id_instruction;
  wire [31:0] id_ex_instruction;
  wire [5:0] alu_control;
  wire reg_write;
  wire mem_read;
  wire mem_write;
  wire reg_dst;
  wire branch;
  wire jump;
  wire alu_src;
  wire [1:0] jump_sel;
  wire [31:0] sign_extended;
  wire [31:0] reg_out_a;
  wire [31:0] reg_out_b;
  wire [31:0] alu_src_mux_out;
  wire [31:0] alu_result;
  wire alu_zero;
  wire [31:0] ex_mem_result;
  wire [31:0] mem_data;
  wire [31:0] mem_wb_mux_out;
  wire mem_to_reg;
  wire [31:0] mem_wb_data;

  // Submodule instances
  ProgramCounter pc(.clk(clk), .count(pc_count));
  InstructionMemory im(.address(pc_count[1:0]), .instruction(instruction));
  IF_ID_Register if_id_reg(.clk(clk), .instruction_in(instruction), .instruction_out(if_id_instruction));
  ControlUnit control_unit(.opcode(if_id_instruction[31:26]), .func(if_id_instruction[5:0]), .alu_control(alu_control),
                           .reg_write(reg_write), .mem_read(mem_read), .mem_write(mem_write),
                           .reg_dst(reg_dst), .branch(branch), .jump(jump), .alu_src(alu_src), .jump_sel(jump_sel));
  SignExtender_16to32 sign_extender(.input_16(if_id_instruction[15:0]), .output_32(sign_extended));
  registerfile regfile(.DA(if_id_instruction[15:11]), .BA(if_id_instruction[20:16]), .AA(if_id_instruction[25:21]),
                       .data(alu_result), .OUTA(reg_out_a), .OUTB(reg_out_b), .write(reg_write));
  ID_EX_Register id_ex_reg(.clk(clk), .instruction_in(if_id_instruction), .instruction_out(id_ex_instruction));
  multiplexer32bit mux_alu_src(.a(id_ex_instruction[15:0]), .b(sign_extended), .out(alu_src_mux_out), .sel(alu_src));
  ALU_32bit alu(.operand1(reg_out_a), .operand2(alu_src_mux_out), .alu_control(alu_control),
                .result(alu_result), .zero(alu_zero));
  EX_MEM_Register ex_mem_reg(.clk(clk), .result_in(alu_result), .result_out(ex_mem_result));
  DataMemory data_mem(.address(ex_mem_result), .write_data(reg_out_b), .write_enable(mem_write),
                      .read_enable(mem_read), .read_data(mem_data));
  multiplexer32bit mux_mem_wb(.a(alu_result), .b(mem_data), .out(mem_wb_mux_out), .sel(mem_to_reg));
  MEM_WB_Register mem_wb_reg(.clk(clk), .data_in(mem_wb_mux_out), .data_out(mem_wb_data));

  // Output wire
  wire [31:0] result;

  // Assign result value
  assign result = mem_to_reg ? mem_wb_data : ex_mem_result;

endmodule

module MIPSStimulus();
reg clk; //input clock
wire [31:0]PcOut,Instruction,RF_D1,RF_D2,ALUOut; //local
MIPS_Processor  myMIPS(clk);
initial clk = 0;
always #10 clk = ~clk;
assign PcOut= myMIPS.pc.count;
    assign Instruction=myMIPS.im.instruction;
    assign RF_D1=myMIPS.regfile.OUTA;
    assign RF_D2=myMIPS.regfile.OUTB;
    assign ALUOut=myMIPS.alu.result;
endmodule

