//module ProgramCounter_TB;
//  // Inputs
//  reg clk;
//  // Outputs
//  wire [31:0] count;
//   // Instantiate the DUT
//  ProgramCounter dut (.clk(clk),.count(count));
//   // Testbench stimulus
//  initial begin
//    // Initialize inputs
//    clk = 0;
//    #10;
//    $display("count = %d", count);  
//    #10;  // Two clock cycles to reach positive edge
//    $display("count = %d", count);
//    #10; 
//    $display("count = %d", count);
//    #5;
//    $display("count = %d", count);
//    #10;  
//    $display("count = %d", count);
//    // End simulation
//    $finish;
//  end
//  // Clock generation
//  always begin
//    #5;
//    clk = ~clk;
//  end
//endmodule

//module BitAdder32_TB;
//  // Inputs
//  reg [31:0] a;
//  reg [31:0] b;
//  // Outputs
//  wire [31:0] sum;
//  // Instantiate the DUT
//  BitAdder32 dut (.a(a),.b(b),.sum(sum));
//  // Testbench stimulus
//  initial begin
//    // Initialize inputs
//    a = 10;
//    b = 20;
//    // Apply inputs and observe outputs
//    #10;
//    $display("a = %h, b = %h, sum = %h", a, b, sum);
//    // Change inputs and observe outputs
//    a = 50;
//    b = 70;
//    #10;
//    $display("a = %d, b = %d, sum = %d", a, b, sum);
//    // Add more test cases as needed
//    // End simulation
//    $finish;
//  end
  
//endmodule

// module testbench_regfile;
//    reg [1:0] DA,BA,AA;
//    reg [31:0] data;
//    wire [31:0] OUTA, OUTB;
//    reg clk, write, reset;
//    registerfile object (DA,BA, AA, data, OUTA, OUTB, clk, write,reset);
//    initial begin
//    // test values 
//    clk =0;
//    data =32'h00001010; write=0; reset=1; DA = 2'b10; AA = 2'b00; BA= 2'b01; #20
//    data =32'h01010100; write=0; reset=1; DA = 2'b10; AA = 2'b10; BA= 2'b00; #20
//    data =32'h01000100; write=1; reset=1; DA = 2'b11; AA = 2'b11; BA= 2'b11; #20
//    data =32'h01011100; write=1; reset=1; DA = 2'b10; AA = 2'b11; BA= 2'b01; #20
//    data =32'h00001111; write=1; reset=0; DA = 2'b11; AA = 2'b01; BA= 2'b11; #20
//    data =32'h11110000; write=0; reset=1; DA = 2'b00; AA = 2'b10; BA= 2'b00; #20
//    data =32'h00010111; write=1; reset=0; DA = 2'b00; AA = 2'b00; BA= 2'b10;
//    end
//endmodule



//module DataMemory_TB;
//  // Inputs
//  reg [31:0] address;
//  reg [31:0] write_data;
//  reg write_enable, read_enable;
//  reg clk;
//  // Outputs
//  wire [31:0] read_data;
//  // Instantiate the DUT
//  DataMemory dut (
//    .address(address),
//    .write_data(write_data),
//    .write_enable(write_enable),
//    .read_enable(read_enable),
//    .read_data(read_data)
//  );
//  // Clock generation
//  always #5 clk = ~clk;
//  // Testbench stimulus
//  initial begin
//    // Initialize inputs
//    address = 0;
//    write_data = 32'h01234567;
//    write_enable = 1;
//    read_enable = 1;
//    clk = 0;
//    // Wait for memory initialization
//    #10;
//    // Read initial data from memory
//    $display("Initial Read: address = %d, read_data = %h", address, read_data);
//    // Write data to memory
//    #10;
//    address = 10;
//    write_data = 32'h89ABCDEF;
//    write_enable = 1;
//    read_enable = 0;
//    $display("Write: address = %d, write_data = %h", address, write_data);
//    // Read data from memory
//    #10;
//    address = 10;
//    write_enable = 0;
//    read_enable = 1;
//    $display("Read: address = %d, read_data = %h", address, read_data);
//    // Add more test cases as needed
//    // End simulation
//    $finish;
//  end
//endmodule

//module SignExtender_16to32_TB;  // Inputs
//  reg [15:0] input_16;
//  // Outputs
//  wire [31:0] output_32;
//  // Instantiate the DUT
//  SignExtender_16to32 dut (.input_16(input_16), .output_32(output_32) );
//  // Testbench stimulus
//  initial begin
//    // Test case 1: Positive value
//    input_16 = 16'h1234;
//    #10;
//    $display("Input (16-bit): %h", input_16);
//    $display("Output (32-bit): %h", output_32);
//    // Test case 2: Negative value
//    input_16 = 16'hFFFF;
//    #10;
//    $display("Input (16-bit): %h", input_16);
//    $display("Output (32-bit): %h", output_32);
//    // Add more test cases as needed
//    // End simulation
//    $finish;
//  end
//endmodule




//module ALU_32bit_TB;

//  // Inputs
//  reg [31:0] operand1;
//  reg [31:0] operand2;
//  reg [3:0] alu_control;

//  // Outputs
//  wire [31:0] result;
//  wire zero;

//  // Instantiate the DUT
//  ALU_32bit dut (
//    .operand1(operand1),
//    .operand2(operand2),
//    .alu_control(alu_control),
//    .result(result),
//    .zero(zero)
//  );

//  // Testbench stimulus
//  initial begin
//    // Test case 1: Addition
//    operand1 = 32'h12345678;
//    operand2 = 32'h87654321;
//    alu_control = 4'b0010;  // Add
//    #10;
//    $display("%h , %h = %h, Zero flag: %d", operand1, operand2, result, zero);
//    $finish;
//  end

//endmodule

//module ControlUnit_TB;
//  reg [5:0] opcode;
//  reg [5:0] func;
//  wire [3:0] alu_control;
//  wire reg_write, mem_read, mem_write, reg_dst, branch, jump, alu_src;
//  wire [1:0] jump_sel;

//  // Instantiate the Control Unit module
//  ControlUnit control_unit (
//    .opcode(opcode),
//    .func(func),
//    .alu_control(alu_control),
//    .reg_write(reg_write),
//    .mem_read(mem_read),
//    .mem_write(mem_write),
//    .reg_dst(reg_dst),
//    .branch(branch),
//    .jump(jump),
//    .alu_src(alu_src),
//    .jump_sel(jump_sel)
//  );

//  // Initialize inputs
//  initial begin
//    opcode = 6'b000000; // R-Type
//    func = 6'b100000;   // ADD
//    #10;
    
//    opcode = 6'b100011; // LW
//    func = 6'b000000;   // N/A (ignored for LW)
//    #10;
    
//    opcode = 6'b101011; // SW
//    func = 6'b000000;   // N/A (ignored for SW)
//    #10;
    
//    opcode = 6'b000100; // BEQ
//    func = 6'b000000;   // N/A (ignored for BEQ)
//    #10;
    
//    opcode = 6'b000010; // J
//    func = 6'b000000;   // N/A (ignored for J)
//    #10;
    
//    // Add more test cases here if needed
    
//    $finish;
//  end

//  // Display outputs
//  always @(alu_control, reg_write, mem_read, mem_write, reg_dst, branch, jump, alu_src, jump_sel) begin
//    $display("ALU Control: %b", alu_control);
//    $display("Reg Write: %b", reg_write);
//    $display("Mem Read: %b", mem_read);
//    $display("Mem Write: %b", mem_write);
//    $display("Reg Dst: %b", reg_dst);
//    $display("Branch: %b", branch);
//    $display("Jump: %b", jump);
//    $display("ALU Src: %b", alu_src);
//    $display("Jump Sel: %b", jump_sel);
//    $display("-------------------------------");
//  end

//endmodule