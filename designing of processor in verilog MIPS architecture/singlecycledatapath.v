`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/23/2023 11:57:39 PM
// Design Name: 
// Module Name: singlecycledatapath
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

module testbench_top_module(); //test bench
    reg clk; //input clock
    wire [31:0]PcOut,Instruction,RF_D1,RF_D2,ALUOut; //local variables to check output
    mainModule main(clk); //top module is called
    assign PcOut= main.data.pc_unit.count;
    assign Instruction=main.data.instr_mem.instruction;
    assign RF_D1=main.data.reg_file.OUTA;
    assign RF_D2=main.data.reg_file.OUTB;
    assign ALUOut=main.data.alu.result;
    initial begin
        clk=0; //initialize clock
    end
    always #50 clk=~clk;
endmodule

module mainModule(clk);
   input clk;
  wire [3:0] alu_control;
  wire reg_write, mem_read, mem_write,reg_dst, branch,jump,alu_src;
  wire [1:0] jump_sel;
  wire [5:0] opcode;
  wire [5:0] func ;   
  Datapath data(alu_control,reg_write, mem_read,mem_write, reg_dst,branch, jump, alu_src, jump_sel, clk, opcode, func);
  ControlUnit cnt( opcode,func,alu_control, reg_write,  mem_read,  mem_write, reg_dst, branch, jump, alu_src, jump_sel);
endmodule

module Datapath (
  input [3:0] alu_control,
  input reg_write,
  input mem_read,
  input mem_write,
  input reg_dst,
  input branch,
  input jump,
  input alu_src,
  input [1:0] jump_sel,
  input clk,
  output [5:0] opcode,
  output [5:0] func
);
  // Declare signals
  wire [31:0] pc;
  wire [31:0] instr;
  wire [31:0] imm;
  wire [31:0] alu_result;
  wire zero;
  wire [31:0] mem_data;
  wire [31:0] reg_a, reg_b;
  wire [4:0] reg_da, reg_ba, reg_aa;
  wire [1:0]addr;
   assign opcode = instr[31:26];
   assign  func = instr[5:0];
  // Instantiate modules
  ProgramCounter pc_unit (.clk(clk), .count(pc));
  assign addr= pc[1:0];
  InstructionMemory instr_mem (.address(addr), .instruction(instr));
  SignExtender_16to32 extender (.input_16(instr[15:0]), .output_32(imm));
  registerfile reg_file (.DA(reg_da), .BA(reg_ba), .AA(reg_aa), .data(mem_data), .OUTA(reg_a), .OUTB(reg_b),  .write(reg_write), .reset(1'b1));
  ALU_32bit alu (.operand1(reg_a), .operand2(imm), .alu_control(alu_control), .result(alu_result), .zero(zero));
  DataMemory data_mem (.address(alu_result), .write_data(reg_b), .write_enable(mem_write), .read_enable(mem_read), .read_data(mem_data));
  

  // Mux for selecting source for reg_b
  multiplexer5bit mux_b (
    .a(reg_aa),
    .b(instr[25:21]),
    .out(reg_b[4:0]),
    .sel(alu_src)
  );

  // Mux for selecting destination register
  multiplexer5bit mux_dst (
    .a(instr[20:16]),
    .b(instr[15:11]),
    .out(reg_da),
    .sel(reg_dst)
  );

  // Mux for selecting branch condition
  multiplexer32bit mux_branch (
    .a(pc + 32'h4),
    .b(pc + (imm << 2)),
    .out(pc),
    .sel(branch)
  );

  // Mux for selecting jump address
  multiplexer32bit mux_jump (
    .a(pc + 32'h4),
    .b({instr[31:28],instr[25:0], 2'b00}),
    .out(pc),
    .sel(jump)
  );

  // Mux for selecting jump address source
  multiplexer5bit mux_jump_sel (
    .a(instr[20:16]),
    .b(instr[15:11]),
    .out(reg_ba),
    .sel(jump)
  );
  // Assign values to opcode and func based on instr
endmodule






