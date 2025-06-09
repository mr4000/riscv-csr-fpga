`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.02.2025 11:32:24
// Design Name: 
// Module Name: risc_csr_tb
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


`timescale 1ns / 1ps

module riscv_csr_tb;

    reg clk;
    reg rst;
    reg intr;
    reg opcode_valid;
    reg [57:0] opcode_instr;
    reg [31:0] opcode_opcode;
    reg [31:0] opcode_pc;
    reg [4:0] opcode_rd_idx;
    reg [4:0] opcode_ra_idx;
    reg [4:0] opcode_rb_idx;
    reg [31:0] opcode_ra_operand;
    reg [31:0] opcode_rb_operand;
    reg branch_exec_request;
    reg [31:0] branch_exec_pc;
    reg [31:0] cpu_id;
    reg [31:0] reset_vector;
    reg fault_store;
    reg fault_load;
    reg fault_misaligned_store;
    reg fault_misaligned_load;
    reg fault_page_store;
    reg fault_page_load;
    reg [31:0] fault_addr;
    
    wire [4:0] writeback_idx;
    wire writeback_squash;
    wire [31:0] writeback_value;
    wire stall;
    wire branch_csr_request;
    wire [31:0] branch_csr_pc;
    
    riscv_csr dut (
        .clk_i(clk),
        .rst_i(rst),
        .intr_i(intr),
        .opcode_valid_i(opcode_valid),
        .opcode_instr_i(opcode_instr),
        .opcode_opcode_i(opcode_opcode),
        .opcode_pc_i(opcode_pc),
        .opcode_rd_idx_i(opcode_rd_idx),
        .opcode_ra_idx_i(opcode_ra_idx),
        .opcode_rb_idx_i(opcode_rb_idx),
        .opcode_ra_operand_i(opcode_ra_operand),
        .opcode_rb_operand_i(opcode_rb_operand),
        .branch_exec_request_i(branch_exec_request),
        .branch_exec_pc_i(branch_exec_pc),
        .cpu_id_i(cpu_id),
        .reset_vector_i(reset_vector),
        .fault_store_i(fault_store),
        .fault_load_i(fault_load),
        .fault_misaligned_store_i(fault_misaligned_store),
        .fault_misaligned_load_i(fault_misaligned_load),
        .fault_page_store_i(fault_page_store),
        .fault_page_load_i(fault_page_load),
        .fault_addr_i(fault_addr),
        .writeback_idx_o(writeback_idx),
        .writeback_squash_o(writeback_squash),
        .writeback_value_o(writeback_value),
        .stall_o(stall),
        .branch_csr_request_o(branch_csr_request),
        .branch_csr_pc_o(branch_csr_pc)
    );
    
    initial begin
        clk = 0;
        rst = 1;
        intr = 0;
        opcode_valid = 0;
        opcode_instr = 0;
        opcode_opcode = 1;
        opcode_pc = 0;
        opcode_rd_idx = 0;
        opcode_ra_idx = 0;
        opcode_rb_idx = 0;
        opcode_ra_operand = 0;
        opcode_rb_operand = 0;
        branch_exec_request = 0;
        branch_exec_pc = 0;
        cpu_id = 0;
        reset_vector = 32'h00000000;
        fault_store = 0;
        fault_load = 0;
        fault_misaligned_store = 0;
        fault_misaligned_load = 0;
        fault_page_store = 0;
        fault_page_load = 0;
        fault_addr = 0;
        
        #10 rst = 0;
        
        #100
        // Ecall
        opcode_valid  = 1;
        opcode_opcode = 32'h00000073;  // ECALL opcode
        opcode_pc    = 32'h1004;      // Example PC that will be captured in mepc
        
        #10;
         opcode_valid  = 0;
         
         #20  // Reading Mcause for Ecall through CSRRW Instruction
         opcode_valid = 1;
         opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00010, 7'b1110011};  // CSRRW instruction
         opcode_ra_operand = 32'h11111100;
         
          #10;
         opcode_valid  = 0;
         
         #20
         
    
    
        // Allow time for exception processing (e.g., updating mepc, mcause, etc.)    
        // Return from Exception via ERET ---

         opcode_valid  = 1;
         opcode_opcode = 32'h10200073;  // ERET/MRET opcode
                                        // For ERET, opcode_pc_i is typically not used; the return target is taken from mepc.
         #10;
         opcode_valid  = 0;
         
         #20  //EBREAK
         
         opcode_valid  = 1;
         opcode_opcode = 32'h00100073;
         
         #10
         opcode_valid = 0;
         
        #20 //Reading Mcause of MRET/ERET
          opcode_valid = 1;
          opcode_opcode = {12'h342, 5'b00101, 3'b111, 5'b00111, 7'b1110011};   
         
         
        #20
         opcode_valid = 0;
             
        #20    // CSRRS Instruction Reading Mstatus
          opcode_valid = 1;
          opcode_opcode = {12'h300, 5'b00011, 3'b010, 5'b00010, 7'b1110011};  // CSRRS x3, mstatus, x1
          opcode_ra_operand = 32'h0000000F;  // rs1 = 0x0000000F //set the bits in those  specified by this Reg.(last 4 digit in this case)
          
        #10
         opcode_valid = 0;  
         
        #20 // CSRRC Instruction
          opcode_valid = 1;
          opcode_opcode = {12'h300, 5'b00010, 3'b011, 5'b00101, 7'b1110011};  // CSRRC x3, mstatus, x1 //Reading CSRRS
          //opcode_ra_operand = 32'h000000F0;  // rs1 = 0x0000000F
          opcode_ra_operand = 32'h0000000F;//clear the bits in those  specified by this Reg.(last 4 digit in this case)
          
        #10
         opcode_valid = 0;
          
        #20 //CSRRWI
          opcode_valid = 1;
          opcode_opcode = {12'h300, 5'b00101, 3'b101, 5'b00110, 7'b1110011};  // CSRRWI x3, mstatus, x1  //Reading CSRRC
          
        #10
         opcode_valid = 0;
        
        #20 //CSRRSI
          opcode_valid = 1;
          opcode_opcode = {12'h300, 5'b00010, 3'b110, 5'b00111, 7'b1110011};  // CSRRSI x3, mstatus, x1 //Reading CSRRWI
          
        #10
         opcode_valid = 0;
         
        #20 //CSRRCI
          opcode_valid = 1;
          opcode_opcode = {12'h300, 5'b00101, 3'b111, 5'b00111, 7'b1110011};  // CSRRC1 x3, mstatus, x1 
          
        #10
         opcode_valid = 0;
         
        #20 
          opcode_valid = 1;
          opcode_opcode = {12'h300, 5'b00101, 3'b111, 5'b00111, 7'b1110011};  // //CSRRCI Read 
          
        #10
         opcode_valid = 0;
         
         // Step 2: Trigger an interrupt.
        // When intr is asserted, the module will set temp_mip[11] = 1.
        // Since mie now has bit 11 enabled, the condition (mie & mip != 0)
        // will be true, and the module should set mcause to MCAUSE_INTERRUPT.
        // It should then request a branch to mtvec + (MCAUSE_INTERRUPT << 2).
        #20
        intr = 1;
        #10;
        intr = 0;
        #10
        
          opcode_valid = 1;
          opcode_opcode = {12'h342, 5'b00011, 3'b001, 5'b00010, 7'b1110011};  // CSRRW x3, mstatus, x1
          opcode_ra_operand = 32'h0000000F;  // rs1 = 0x0000000F 
      
      #10
      opcode_valid = 0;
      
      #20
         fault_misaligned_load = 1;  //4
         #10
        fault_misaligned_load = 0;
        #10
        opcode_valid = 1;
      opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00000, 7'b1110011};  // CSRRW instruction
      opcode_ra_operand = 32'h00000000;
      #10
      opcode_valid = 0;
       
       
       #20
        fault_load = 1;    //5
        #10
        fault_load = 0;
        #10
        opcode_valid = 1;
       opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00000, 7'b1110011};  // CSRRW instruction
       opcode_ra_operand = 32'h00000001;
       #10
      opcode_valid = 0;
       
        
       #20
        fault_misaligned_store = 1; //6
        #10
        fault_misaligned_store = 0;
        #10
        opcode_valid = 1;
      opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00000, 7'b1110011};  // CSRRW instruction
      opcode_ra_operand = 32'h00000002;
       #10
      opcode_valid = 0;
       
       
        
       #20
        fault_store = 1;   //7
        #10
        fault_store = 0;
        #10
        opcode_valid = 1;
      opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00000, 7'b1110011};  // CSRRW instruction
      opcode_ra_operand = 32'h00000003;
       #10
      opcode_valid = 0;
        
        
       #20
        fault_page_load = 1;   //13
        #10
        fault_page_load = 0; 
        #10
        opcode_valid = 1;
      opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00000, 7'b1110011};  // CSRRW instruction
      opcode_ra_operand = 32'h00000004;
       #10
      opcode_valid = 0;
       
        
       #20
        fault_page_store = 1;   //15 
         #10
        fault_page_store = 0;
        #10
        opcode_valid = 1;
      opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00000, 7'b1110011};  // CSRRW instruction
      opcode_ra_operand = 32'h00000005;
       #10
      opcode_valid = 0;
      
      
      #20
        fault_page_store = 1;   //15 
        #10
        intr = 1;
        #10;
        intr = 0;
        #10
        fault_page_store = 0;
        #10
        opcode_valid = 1;
      opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00000, 7'b1110011};  // CSRRW instruction
      opcode_ra_operand = 32'h00000005;
       
       #10
      opcode_valid = 0;
      
      //Misaligned Fetch,Invalid Instruction
        
/////////////////////////////////////////////////////////////////////////////////////////
      
      #20//illegal instruction
     opcode_valid = 1;
     opcode_opcode = {12'h300, 5'b00001, 3'b001, 5'b00010, 7'b1110011};  // CSRRW instruction
     opcode_ra_operand = 32'h00000000;
        
     #10;
     opcode_valid  = 0;
     #20
      opcode_valid  = 1;
         opcode_opcode = 32'h10200073;  // ERET/MRET opcode
    
     #10;
     opcode_valid  = 0;
     
     #20
     opcode_valid = 1;
     opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00010, 7'b1110011};  // CSRRW instruction
     opcode_ra_operand = 32'h00000000;
     #10;
     opcode_valid  = 0;
      #20;
      
      #20
     opcode_valid = 1;
     opcode_opcode = {12'h300, 5'b00001, 3'b001, 5'b00010, 7'b1110011};  // CSRRW instruction
     opcode_ra_operand = 32'hFFFFFFFF;
        
     #10;
     opcode_valid  = 0;
     #20
      opcode_valid  = 1;
         opcode_opcode = 32'h10200073;  // ERET/MRET opcode
         
     #10;
     opcode_valid  = 0;
     #20
     opcode_valid = 1;
     opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00010, 7'b1110011};  // CSRRW instruction
     opcode_ra_operand = 32'h00000000;
     #10;
     opcode_valid  = 0;
     
     
    //////////////////////////////////////////////////////////////////////////////////////////
    //Misaligned Fetch
    #20
    opcode_pc = 32'h1;
    #10
    opcode_pc = 32'h0;
    #20
     opcode_valid = 1;
     opcode_opcode = {12'h342, 5'b00001, 3'b001, 5'b00010, 7'b1110011};  // CSRRW instruction
     opcode_ra_operand = 32'h00000000;
    #10;
     opcode_valid  = 0;
      
      

      

        #20 $finish;

    end
    
    always #5 clk = ~clk;
    
endmodule