`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/23/2023 09:46:58 PM
// Design Name: 
// Module Name: elemets
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
///////////PROGRAM COUNTER
module ProgramCounter(
  input wire clk,
  output reg [31:0] count
);
  reg reset=1;
  always @(posedge clk) begin
    if (reset)begin
      count <= 32'h0;
      reset =0;
      end
    else begin
      count <= count + 32'h00000001;
  end 
  end 
endmodule

////////////////32 bit adder
module BitAdder32(input [31:0] a, input [31:0] b,output reg [31:0] sum);
  always @(*) begin
    sum = a + b;
  end
endmodule

module registerfile(DA,BA, AA, data, OUTA, OUTB, write,reset);
    input [4:0] DA,BA,AA; // addresses 
    input [31:0] data; // input
    output reg [31:0] OUTA, OUTB; // output
    input write, reset; // control
    reg [31:0] MEM [3:0]; // registerfile
    integer i;
    initial begin 
     // initializing the register file
    MEM [0] = 32'h02404000;
    MEM [1] = 32'h00300501;
    MEM [2] = 32'h00000100;
    MEM [3] = 32'h00150000;
    end  
    always @(*)
    if (reset ==0) // if reset==0 then all data is cleared 
     begin
        OUTA = 32'h00000000;
        OUTB = 32'h00000000;
         for (i=0; i<4;i=i+1)begin
           MEM[i] =32'h0;
         end 
     end 
     else 
     begin
        if (write ==1) // if write is off so all the data stored will be displayed as output
        begin
              if (DA == AA) // DA==AA
              begin
                 case (AA)
                 5'b00000 : MEM[0]=OUTA;
                 5'b00001 : MEM[1]=OUTA;
                 5'b00010 : MEM[2]=OUTA;
                 5'b00011 : MEM[3]=OUTA;
                 default : MEM[AA]= 32'h12473292;
                 endcase
              end   
              if (DA == BA)//id BA==AA
              begin
                 case (BA)
                 5'b00000 : MEM[0]=OUTB;
                 5'b00001 : MEM[1]=OUTB;
                 5'b00010 : MEM[2]=OUTB;
                 5'b00011 : MEM[3]=OUTB;
                 default : MEM [BA]= 32'h15782248;
                 endcase
              end   
        end    
        else 
        begin 
            // if writw is on 
             case (DA) // declaring the data equal to inpurt 
             DA:  MEM[DA] =data;
             endcase
              if (DA == AA) // DA==AA
              begin
                 case (AA)
                 5'b00000 :OUTA = MEM[0];
                 5'b00001 :OUTA = MEM[1];
                 5'b00010 :OUTA = MEM[2];
                 5'b00011 :OUTA = MEM[3];
                 default : OUTA= 32'h00472F35;
                 endcase
              end  
              else 
                 OUTA= 32'h012345100; 
              if (DA == BA)//id BA==AA
              begin
                 case (BA)
                 5'b00000 :OUTA = MEM[0];
                 5'b00001 :OUTA = MEM[1];
                 5'b00010 :OUTA = MEM[2];
                 5'b00011 :OUTA = MEM[3];
                 default : OUTB= 32'h1243BC00;
                 endcase
              end   
              else 
                 OUTA= 32'h012005100;
        end     
  end
 endmodule
///////instruction memory
module InstructionMemory(input [1:0] address,output reg [31:0] instruction);
integer i=0;
  reg [31:0] MEM [1:0];
  // Initialize memory with random instructions
  initial begin
    // Load random instructions into memory
    MEM [0] = 32'h00000100;
    MEM [1] = 32'h00150000;
    MEM [2] = 32'h00000100;
    MEM [3] = 32'h00150000;
    end  
  always @(*) begin
    case (address)
    2'b00: instruction = MEM[0];
    2'b01: instruction = MEM[1];
    default: instruction=32'h00000100; 
    endcase
  end  
endmodule

//////////////data memory
module DataMemory(
  input [31:0] address,              // Input: Address for read/write operations
  input [31:0] write_data,           // Input: Data to be written
  input write_enable, read_enable,   // Input: Write and read enable signals
  output reg [31:0] read_data        // Output: Data read from the memory
);
  integer i=0;
  reg [31:0] memory [31:0];          // Internal memory array
  // Write operation
    initial begin
    // Load random instructions into memory
    for (i = 0; i <32; i = i + 1) begin
      memory[i] = $random;
    end
  end
  always @(address) begin
    if (write_enable) begin
      memory[address] <= write_data; // Write the data to the specified address
    end
  end
  // Read operation
  always @(address) begin
    if (read_enable) begin
      read_data <= memory[address];  // Read the data from the specified address
    end
  end
endmodule

//////////SIGN EXTENDER:
module SignExtender_16to32(input [15:0] input_16, output [31:0] output_32);
   assign output_32 = { {16{input_16[15]}}, input_16 };
endmodule

///////////ALU 
module ALU_32bit(
  input [31:0] operand1,
  input [31:0] operand2,
  input [3:0] alu_control,
  output reg [31:0] result,
  output reg zero // register flag
);
  always @(*) begin
    case (alu_control)
      4'b0000: result = operand1 + operand2;   // Add
      4'b0001: result = operand1 - operand2;   // Subtract
      4'b0010: result = operand1 & operand2;   // Bitwise AND
      4'b0011: result = operand1 | operand2;   // Bitwise OR
      4'b0100: result = operand1 ^ operand2;   // Bitwise XOR
      4'b0101: result = operand1 << operand2;  // Shift left
      4'b0110: result = operand1 >> operand2;  // Shift right (arithmetic)
      default: result = 32'b0;                 // Default: Output 0
    endcase
    zero = (result == 0); // Set zero flag if result is zero
  end
endmodule


////////////////MULTIPLEXERS
////5 bit
module multiplexer5bit(a,b,out,sel);
    input [4:0] a,b; // inputs
    output reg [4:0] out; // outputs 
    input sel; // select 
    always @ (*)
    begin
    case (sel) // mux impementation
      1'b0: out =a;
      1'b1: out =b;
    endcase
    end
endmodule

/// 32 bit:
module multiplexer32bit(a,b,out,sel);
    input [31:0] a,b; // inputs 
    output reg [31:0] out; 
    input sel;
    always @ (*)
    begin
    case (sel) // mux implementation
      1'b0: out =a;
      1'b1: out =b;
    endcase
    end
endmodule


////////IF/ID
module IF_ID_Register(
  input wire clk,
  input wire reset,
  input wire [31:0] instruction_in,
  output reg [31:0] instruction_out
);
  always @(posedge clk) begin
    if (reset) begin
      instruction_out <= 32'h0;
    end else begin
      instruction_out <= instruction_in;
    end
  end
endmodule

///////ID/EX
module ID_EX_Register(
  input wire clk,
  input wire reset,
  input wire [31:0] instruction_in,
  output reg [31:0] instruction_out
  // Add other input/output ports as needed for other control signals and data
);
  always @(posedge clk) begin
    if (reset) begin
      instruction_out <= 32'h0;
    end else begin
      instruction_out <= instruction_in;
    end
  end
endmodule

////////EX/MEM
module EX_MEM_Register(
  input wire clk,
  input wire reset,
  input wire [31:0] result_in,
  output reg [31:0] result_out
  // Add other input/output ports as needed for other control signals and data
);
  always @(posedge clk) begin
    if (reset) begin
      result_out <= 32'h0;
    end else begin
      result_out <= result_in;
    end
  end
endmodule

////////MEM/WB
module MEM_WB_Register(
  input wire clk,
  input wire reset,
  input wire [31:0] data_in,
  output reg [31:0] data_out
  // Add other input/output ports as needed for other control signals and data
);
  always @(posedge clk) begin
    if (reset) begin
      data_out <= 32'h0;
    end else begin
      data_out <= data_in;
    end
  end
endmodule


/////////////////////////CONTROL UNIT
module ControlUnit (
  input [5:0] opcode,
  input [5:0] func,
  output reg [3:0] alu_control,
  output reg reg_write,
  output reg mem_read,
  output reg mem_write,
  output reg reg_dst,
  output reg branch,
  output reg jump,
  output reg alu_src,
  output reg [1:0] jump_sel
);
  // Control signals for each instruction type
  localparam R_TYPE = 6'b000000;
  localparam LW = 6'b100011;
  localparam SW = 6'b101011;
  localparam BEQ = 6'b000100;
  localparam J = 6'b000010;

  // ALU Control signals based on the function field
  reg [3:0] alu_op;
  always @(func) begin
    case (func)
      6'b100000, 6'b100010: alu_op = 4'b0010; // ADD, SUB
      6'b100100: alu_op = 4'b0000; // AND
      6'b100101: alu_op = 4'b0001; // OR
      6'b101010: alu_op = 4'b0101; // SLT
      default: alu_op = 4'b0000; // Default: 0
    endcase
  end
  // Control signals assignment based on opcode
  always @(opcode, func) begin
    case (opcode)
      R_TYPE: begin
        alu_control = alu_op;
        reg_write = 1;
        mem_read = 0;
        mem_write = 0;
        reg_dst = 1;
        branch = 0;
        jump = 0;
        alu_src = 0;
        jump_sel = 2'b00;
      end
      LW: begin
        alu_control = 4'b0010; // ALU control for addition
        reg_write = 1;
        mem_read = 1;
        mem_write = 0;
        reg_dst = 0;
        branch = 0;
        jump = 0;
        alu_src = 1;
        jump_sel = 2'b00;
      end
      SW: begin
        alu_control = 4'b0010; // ALU control for addition
        reg_write = 0;
        mem_read = 0;
        mem_write = 1;
        reg_dst = 0;
        branch = 0;
        jump = 0;
        alu_src = 1;
        jump_sel = 2'b00;
      end
      BEQ: begin
        alu_control = 4'b0110; // ALU control for subtraction
        reg_write = 0;
        mem_read = 0;
        mem_write = 0;
        reg_dst = 0;
        branch = 1;
        jump = 0;
        alu_src = 0;
        jump_sel = 2'b00;
      end
      J: begin
        alu_control = 4'b0000; // ALU control for addition
        reg_write = 0;
        mem_read = 0;
        mem_write = 0;
        reg_dst = 0;
        branch = 0;
        jump = 1;
        alu_src = 0;
        jump_sel = 2'b01; // jump_sel specifies jump instruction type
      end
      default: begin
        alu_control = 4'b0000;
        reg_write = 0;
        mem_read = 0;
        mem_write = 0;
        reg_dst = 0;
        branch = 0;
        jump = 0;
        alu_src = 0;
        jump_sel = 2'b00;
      end
    endcase
  end
endmodule


//////////////// control delay slot  
module ControlDelaySlot(
  input wire clk,
  input wire reset,
  input wire branch_taken,
  input wire [7:0] instruction,
  output reg [7:0] output_instruction
);
  reg [7:0] delayed_instruction;
  
  always @(posedge clk) begin
    if (reset) begin
      output_instruction <= 8'h00;     // Reset the output instruction to a NOP instruction
      delayed_instruction <= 8'h00;    // Reset the delayed instruction to a NOP instruction
    end else begin
      if (branch_taken) begin
        output_instruction <= delayed_instruction;   // Execute the delayed instruction
        delayed_instruction <= instruction;          // Fetch the new instruction for the delay slot
      end else begin
        output_instruction <= instruction;            // Execute the current instruction
        delayed_instruction <= 8'h00;                  // Clear the delayed instruction for non-branch instructions
      end
    end
  end
endmodule

////////////////HAZARD DETECTION UNIT 
module HazardDetectionUnit(
  input wire [31:0] if_id_instruction,
  input wire [31:0] id_ex_instruction,
  input wire branch,
  input wire [5:0] alu_control,
  input wire [31:0] ex_mem_result,
  output reg hazard_detected
);
  always @(*) begin
    if (
      (if_id_instruction[20:16] != 0 && if_id_instruction[20:16] == id_ex_instruction[15:11]) || // Read after write hazard for rs
      (if_id_instruction[25:21] != 0 && if_id_instruction[25:21] == id_ex_instruction[15:11]) || // Read after write hazard for rt
      (branch && (alu_control != 4'b0000) && (if_id_instruction[20:16] == id_ex_instruction[20:16] || if_id_instruction[20:16] == id_ex_instruction[15:11])) || // Data hazard on rs or rt due to branch instruction
      (ex_mem_result[20:16] != 0 && ex_mem_result[20:16] == id_ex_instruction[15:11]) // Data hazard on rt due to load instruction
    ) begin
      hazard_detected = 1'b1;
    end else begin
      hazard_detected = 1'b0;
    end
  end
endmodule


///////////////FORWARDING UNIT
module ForwardingUnit(
  input wire [4:0] rs, // RS (source register)
  input wire [4:0] rt, // RT (target register)
  input wire [4:0] ex_mem_rd, // EX/MEM RD (destination register)
  input wire [4:0] mem_wb_rd, // MEM/WB RD (destination register)
  input wire ex_mem_mem_to_reg, // EX/MEM MEM_TO_REG signal
  input wire mem_wb_mem_to_reg, // MEM/WB MEM_TO_REG signal
  output reg [1:0] forward_A, // Forwarding signal for RS
  output reg [1:0] forward_B // Forwarding signal for RT
);

  always @(*) begin
    forward_A = 2'b00; // No forwarding
    forward_B = 2'b00; // No forwarding

    // Forward from EX/MEM stage
    if (ex_mem_rd != 5'b00000 && ex_mem_rd == rs)
      forward_A = 2'b10; // Forward from EX/MEM register
    if (ex_mem_rd != 5'b00000 && ex_mem_rd == rt)
      forward_B = 2'b10; // Forward from EX/MEM register

    // Forward from MEM/WB stage
    if (ex_mem_mem_to_reg == 1'b0 && mem_wb_rd != 5'b00000 && mem_wb_rd == rs)
      forward_A = 2'b01; // Forward from MEM/WB register
    if (ex_mem_mem_to_reg == 1'b0 && mem_wb_rd != 5'b00000 && mem_wb_rd == rt)
      forward_B = 2'b01; // Forward from MEM/WB register
  end

endmodule


//////////////CONTROL-Hazard removal unit
module HazardRemovalUnit(
  input wire ID_EX_RegWrite,
  input wire EX_MEM_RegRead,
  input wire EX_MEM_MemRead,
  input wire [4:0] EX_MEM_Rd,
  input wire [4:0] IF_ID_Rs,
  input wire [4:0] IF_ID_Rt,
  output reg stall,
  output reg flush
);
  always @(*) begin
    stall = 0;
    flush = 0;
    if ((ID_EX_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == IF_ID_Rs)) ||
        (ID_EX_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == IF_ID_Rt) && (EX_MEM_MemRead == 1))) begin
      stall = 1;
    end else if ((ID_EX_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == IF_ID_Rt)) ||
               (ID_EX_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == IF_ID_Rs) && (EX_MEM_MemRead == 1))) begin
      flush = 1;
    end
  end
endmodule


//////////////STRUCTURAL HAZRAD REMOVAL
module StructuralHazardRemovalUnit(
  input wire clk,
  input wire reset,
  input wire stall,
  output reg enable
);
  reg previous_stall;
  
  always @(posedge clk) begin
    if (reset) begin
      enable <= 1'b0;
      previous_stall <= 1'b0;
    end else begin
      if (stall == 1'b1 && previous_stall == 1'b0) begin
        enable <= 1'b0;
      end else begin
        enable <= 1'b1;
      end
      previous_stall <= stall;
    end
  end
endmodule


///////////// REMOVAL OF LOAD HAZARD
module LoadHazardRemovalUnit(
  input wire clk,          // Clock signal
  input wire reset,        // Reset signal
  input wire [4:0] rs1,    // Source register 1
  input wire [4:0] rs2,    // Source register 2
  input wire [4:0] rd,     // Destination register
  input wire [4:0] ld_rd,  // Register being loaded
  output reg enable        // Hazard enable signal
);
  reg previous_ld_rd;      // Register to store the previous loaded register value
  
  always @(posedge clk) begin
    if (reset) begin
      enable <= 1'b0;                // Reset the enable signal to 0
      previous_ld_rd <= 0;           // Reset the previous loaded register value to 0
    end else begin
      if (ld_rd == rs1 || ld_rd == rs2) begin
        enable <= 1'b0;              // Hazard detected: stall the pipeline stage
      end else if (ld_rd == rd && previous_ld_rd != ld_rd) begin
        enable <= 1'b0;              // Load-use dependency hazard detected: stall the pipeline stage
      end else begin
        enable <= 1'b1;              // No hazard detected: allow the pipeline stage to proceed
      end
      previous_ld_rd <= ld_rd;       // Store the current loaded register value for the next cycle
    end
  end
endmodule

