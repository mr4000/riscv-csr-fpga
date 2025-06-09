`include "def.v"

module riscv_csr (
    input  wire         clk_i,
    input  wire         rst_i,
    input  wire         intr_i,       //External interrupt input from Exception and Interrupt
    input  wire         opcode_valid_i,  //Indicates if the current opcode is valid from Decode
    input  wire [57:0]  opcode_instr_i,  //instruction input from Decode
    input  wire [31:0]  opcode_opcode_i,  //opcode input from Decode
    input  wire [31:0]  opcode_pc_i,    //Program counter for the current instruction from Decode
    input  wire [4:0]   opcode_rd_idx_i,  // Destination register index from Decode
    input  wire [4:0]   opcode_ra_idx_i,  // First source register index from Decode
    input  wire [4:0]   opcode_rb_idx_i,  //Second source register index from Decode
    input  wire [31:0]  opcode_ra_operand_i,  //Value of the first source register from Decode
    input  wire [31:0]  opcode_rb_operand_i,   //Value of the second source register from Decode
    input  wire         branch_exec_request_i,   // Indicates a branch execution request from Exec
    input  wire [31:0]  branch_exec_pc_i,       // Target PC for the branch execution from Exec
    input  wire [31:0]  cpu_id_i,               //CPU identification number from Exception and Interrupt
    input  wire [31:0]  reset_vector_i,         //Reset vector address from Exception and Interrupt
    input  wire         fault_store_i,          //Indicates a store fault from LSU
    input  wire         fault_load_i,           // Indicates a load fault from LSU
    input  wire         fault_misaligned_store_i, //Indicates a misaligned store fault from LSU
    input  wire         fault_misaligned_load_i,  //Indicates a misaligned load fault from LSU
    input  wire         fault_page_store_i,       //Indicates a page fault during store from LSU
    input  wire         fault_page_load_i,        //Indicates a page fault during load from LSU
    input  wire [31:0]  fault_addr_i,              // Address where a fault occurred from LSU
    
    output reg  [4:0]   writeback_idx_o,            //Index of the register to be written back to Decode
    output reg          writeback_squash_o,         //Indicates if the writeback should be squashed to Decode
    output reg  [31:0]  writeback_value_o,          // Value to be written back to Decode
    output reg          stall_o,                    //Indicates if the pipeline should stall to Decode
    output reg          branch_csr_request_o,       //Indicates a branch request due to CSR operation to Decode
    output reg  [31:0]  branch_csr_pc_o             // Target PC for the CSR-induced branch to Decode
);

    //--------------------------------------------------------------------------
    // Internal registers for the key CSRs and machine privilege
    //--------------------------------------------------------------------------    
    reg [1:0]   mpriv;
    reg [31:0]  mstatus;
    reg [31:0]  mtvec;
    reg [31:0]  mepc;
    reg [31:0]  mcause;
    reg [31:0]  mip;
    reg [31:0]  mie;
    reg [31:0]  mscratch;
    reg [31:0]  mcycle;

    //--------------------------------------------------------------------------
    // Next-state (temporary) signals computed combinationally.
    //--------------------------------------------------------------------------    
    reg [1:0]   next_mpriv;
    reg [31:0]  next_mstatus;
    reg [31:0]  next_mtvec;
    reg [31:0]  next_mepc;
    reg [31:0]  next_mcause;
    reg [31:0]  next_mip;
    reg [31:0]  next_mie;
    reg [31:0]  next_mscratch;
    reg [31:0]  next_mcycle;

    reg         next_branch_csr_request;
    reg [31:0]  next_branch_csr_pc;
    reg         next_stall;
    reg [4:0]   next_writeback_idx;
    reg [31:0]  next_writeback_value;
    reg         next_writeback_squash;

    //--------------------------------------------------------------------------
    // Instruction decode signals (based on opcode masks/fields from def.v)
    //--------------------------------------------------------------------------    
    wire ecall_w   = opcode_valid_i && ( ((opcode_opcode_i & `INST_ECALL_MASK)  == `INST_ECALL)  ||
                                         (opcode_instr_i[`ENUM_INST_ECALL]  == 1'b1) );
    wire ebreak_w  = opcode_valid_i && ( ((opcode_opcode_i & `INST_EBREAK_MASK) == `INST_EBREAK) ||
                                         (opcode_instr_i[`ENUM_INST_EBREAK] == 1'b1) );
    wire eret_w    = opcode_valid_i && ( ((opcode_opcode_i & `INST_MRET_MASK)   == `INST_MRET)   ||
                                         (opcode_instr_i[`ENUM_INST_ERET]   == 1'b1) );
    wire csrrw_w   = opcode_valid_i && ( ((opcode_opcode_i & `INST_CSRRW_MASK)  == `INST_CSRRW)  ||
                                         (opcode_instr_i[`ENUM_INST_CSRRW]  == 1'b1) );
    wire csrrs_w   = opcode_valid_i && ( ((opcode_opcode_i & `INST_CSRRS_MASK)  == `INST_CSRRS)  ||
                                         (opcode_instr_i[`ENUM_INST_CSRRS]  == 1'b1) );
    wire csrrc_w   = opcode_valid_i && ( ((opcode_opcode_i & `INST_CSRRC_MASK)  == `INST_CSRRC)  ||
                                         (opcode_instr_i[`ENUM_INST_CSRRC]  == 1'b1) );
    wire csrrwi_w  = opcode_valid_i && ( ((opcode_opcode_i & `INST_CSRRWI_MASK) == `INST_CSRRWI) ||
                                         (opcode_instr_i[`ENUM_INST_CSRRWI] == 1'b1) );
    wire csrrsi_w  = opcode_valid_i && ( ((opcode_opcode_i & `INST_CSRRSI_MASK) == `INST_CSRRSI) ||
                                         (opcode_instr_i[`ENUM_INST_CSRRSI] == 1'b1) );
    wire csrrci_w  = opcode_valid_i && ( ((opcode_opcode_i & `INST_CSRRCI_MASK) == `INST_CSRRCI) ||
                                         (opcode_instr_i[`ENUM_INST_CSRRCI] == 1'b1) );

    //--------------------------------------------------------------------------
    // Determine the mask for a given CSR address (from opcode bits 31:20)
    // This mask controls which bits are writable.
    //--------------------------------------------------------------------------    
    reg [31:0] csr_mask;
    always @(*) begin
        case (opcode_opcode_i[31:20])
            12'h300: csr_mask = `CSR_MSTATUS_MASK;
            12'h305: csr_mask = `CSR_MTVEC_MASK;
            12'h341: csr_mask = `CSR_MEPC_MASK;
            12'h342: csr_mask = `CSR_MCAUSE_MASK;
            12'h344: csr_mask = `CSR_MIP_MASK;
            12'h304: csr_mask = `CSR_MIE_MASK;
            12'hC00: csr_mask = `CSR_MCYCLE_MASK;
            12'h340: csr_mask = `CSR_MSCRATCH_MASK;
            default: csr_mask = 32'h0;
        endcase
    end

    //--------------------------------------------------------------------------
    // Sequential block: update state on clock edge or asynchronous reset.
    //--------------------------------------------------------------------------    
    always @(posedge clk_i) begin
        if (rst_i) begin
            mpriv           <= `PRIV_MACHINE;
            mstatus         <= 32'h00000000;
            mtvec           <= reset_vector_i;
            mepc            <= 32'b0;
            mcause          <= 32'b0;
            mip             <= 32'b0;
            mie             <= 32'b0;
            mscratch        <= 32'b0;
            mcycle          <= 32'b0;
            
            branch_csr_request_o <= 1'b0;
            branch_csr_pc_o      <= 32'b0;
            stall_o              <= 1'b0;
            writeback_idx_o      <= 5'b0;
            writeback_value_o    <= 32'b0;
            writeback_squash_o   <= 1'b0;
        end else begin
            // Increment cycle counter
            mcycle <= mcycle + 1;
            
            // Update all CSRs and outputs from the next-state signals
            mpriv    <= next_mpriv;
            mstatus  <= next_mstatus;
            mtvec    <= next_mtvec;
            mepc     <= next_mepc;
            mcause   <= next_mcause;
            mip      <= next_mip;
            mie      <= next_mie;
            mscratch <= next_mscratch;
            // mcycle is updated above
            
            branch_csr_request_o <= next_branch_csr_request;
            branch_csr_pc_o      <= next_branch_csr_pc;
            stall_o              <= next_stall;
            writeback_idx_o      <= next_writeback_idx;
            writeback_value_o    <= next_writeback_value;
            writeback_squash_o   <= next_writeback_squash;
        end
    end

    //--------------------------------------------------------------------------
    // Combinational block: compute next state and outputs.
    // Note: all outputs and next_* signals are given default assignments to
    //       avoid inferred latches.
    //--------------------------------------------------------------------------    
    always @(*) begin
        // Default: next state equals current state
        next_mpriv    = mpriv;
        next_mstatus  = mstatus;
        next_mtvec    = mtvec;
        next_mepc     = mepc;
        next_mcause   = mcause;
        next_mip      = mip;
        next_mie      = mie;
        next_mscratch = mscratch;
        next_mcycle   = mcycle;

        // Default output signals
        next_branch_csr_request = 1'b0;
        next_branch_csr_pc      = 32'b0;
        next_stall              = 1'b0;
        next_writeback_idx      = 5'b0;
        next_writeback_value    = 32'b0;
        next_writeback_squash   = 1'b0;

        // Update the interrupt pending bit (assume bit 11 is used)
        next_mip[11] = intr_i;

        //--------------------------------------------------------------------------
        // Exception and Trap Handling
        //--------------------------------------------------------------------------
        if (ebreak_w || ecall_w || eret_w ||
            fault_store_i || fault_load_i || fault_misaligned_store_i || (opcode_pc_i[1:0] != 2'b00)  ||
            fault_misaligned_load_i || fault_page_store_i || fault_page_load_i || intr_i) begin

            next_stall = 1'b0;
            next_writeback_idx = 5'b0;
            next_writeback_value = 32'b0;
            next_writeback_squash = 1'b0;

            if (ebreak_w) begin
                next_mcause = `MCAUSE_BREAKPOINT;
                next_mstatus[12:11] = mpriv;  // Save current privilege in MPP
                next_mstatus[7]     = mstatus[3];  // Save interrupt enable bits (example mapping)
                next_mstatus[5]     = mstatus[1];
                next_mstatus[4]     = mstatus[0];
                next_mstatus[3]     = 1'b0;
                next_mstatus[1]     = 1'b0;
                next_mstatus[0]     = 1'b0;
                next_mpriv = `PRIV_MACHINE;
                next_mepc = opcode_pc_i;
                next_branch_csr_pc = mtvec + (next_mcause << 2);
                next_branch_csr_request = 1'b1;
            end else if (ecall_w) begin
                case (mpriv)
                    `PRIV_USER:    next_mcause = `MCAUSE_ECALL_U;
                    `PRIV_SUPER:   next_mcause = `MCAUSE_ECALL_S;
                    `PRIV_MACHINE: next_mcause = `MCAUSE_ECALL_M;
                    default:       next_mcause = `MCAUSE_ECALL_M;
                endcase
                next_mstatus[12:11] = mpriv;
                next_mstatus[7]     = mstatus[3];
                next_mstatus[5]     = mstatus[1];
                next_mstatus[4]     = mstatus[0];
                next_mstatus[3]     = 1'b0;
                next_mstatus[1]     = 1'b0;
                next_mstatus[0]     = 1'b0;
                next_mpriv = `PRIV_MACHINE;
                next_mepc = opcode_pc_i;
                next_branch_csr_pc = mtvec + (next_mcause << 2);
                next_branch_csr_request = 1'b1;
            end else if (eret_w) begin
                if (mpriv != `PRIV_MACHINE) begin
                    next_mcause = `MCAUSE_ILLEGAL_INSTRUCTION;
                    next_mstatus[12:11] = mpriv;
                    next_mstatus[7]     = mstatus[3];
                    next_mstatus[5]     = mstatus[1];
                    next_mstatus[4]     = mstatus[0];
                    next_mstatus[3]     = 1'b0;
                    next_mstatus[1]     = 1'b0;
                    next_mstatus[0]     = 1'b0;
                    next_mpriv = `PRIV_MACHINE;
                    next_mepc   = opcode_pc_i;
                    next_branch_csr_pc = mtvec + (next_mcause << 2);
                    next_branch_csr_request = 1'b1;
                end else begin
                    // Restore privilege and interrupt enables
                    next_mpriv = mstatus[12:11];
                    next_mstatus[3] = mstatus[7];
                    next_mstatus[1] = mstatus[5];
                    next_mstatus[0] = mstatus[4];
                    next_mepc = mepc;
                    next_branch_csr_pc = mepc;
                    next_branch_csr_request = 1'b1;
                end
            end else if (opcode_pc_i[1:0] != 2'b00) begin
                // Misaligned fetch
                next_mcause = `MCAUSE_MISALIGNED_FETCH;
                next_mstatus[12:11] = mpriv;
                next_mstatus[7]     = mstatus[3];
                next_mstatus[5]     = mstatus[1];
                next_mstatus[4]     = mstatus[0];
                next_mstatus[3]     = 1'b0;
                next_mstatus[1]     = 1'b0;
                next_mstatus[0]     = 1'b0;
                next_mpriv = `PRIV_MACHINE;
                next_mepc  = opcode_pc_i;
                next_branch_csr_pc = mtvec + (next_mcause << 2);
                next_branch_csr_request = 1'b1;
            end else if (fault_store_i || fault_load_i ||
                         fault_misaligned_store_i || fault_misaligned_load_i ||
                         fault_page_store_i || fault_page_load_i || intr_i) begin
                if (fault_misaligned_load_i)
                    next_mcause = `MCAUSE_MISALIGNED_LOAD;
                else if (fault_load_i)
                    next_mcause = `MCAUSE_FAULT_LOAD;
                else if (fault_misaligned_store_i)
                    next_mcause = `MCAUSE_MISALIGNED_STORE;
                else if (fault_store_i)
                    next_mcause = `MCAUSE_FAULT_STORE;
                else if (fault_page_load_i)
                    next_mcause = `MCAUSE_PAGE_FAULT_LOAD;
                else if (fault_page_store_i)
                    next_mcause = `MCAUSE_PAGE_FAULT_STORE;
                else if (intr_i || (mie & mip) != 32'b0)
                    next_mcause = `MCAUSE_INTERRUPT;
                    
                next_mstatus[12:11] = mpriv;
                next_mstatus[7]     = mstatus[3];
                next_mstatus[5]     = mstatus[1];
                next_mstatus[4]     = mstatus[0];
                next_mstatus[3]     = 1'b0;
                next_mstatus[1]     = 1'b0;
                next_mstatus[0]     = 1'b0;
                next_mpriv = `PRIV_MACHINE;
                next_mepc = opcode_pc_i;
                next_branch_csr_pc = mtvec + (next_mcause << 2);
                next_branch_csr_request = 1'b1;
            end

        //--------------------------------------------------------------------------
        // CSR Access Instructions (reads/writes)
        //--------------------------------------------------------------------------
        end else if (csrrw_w || csrrs_w || csrrc_w ||
                     csrrwi_w || csrrsi_w || csrrci_w) begin

            // Check for illegal access (e.g. not in machine mode)
            if ((((opcode_opcode_i & `INST_CSRRW_MASK)  != `INST_CSRRW) &&
                 ((opcode_opcode_i & `INST_CSRRS_MASK)  != `INST_CSRRS) &&
                 ((opcode_opcode_i & `INST_CSRRC_MASK)  != `INST_CSRRC) &&
                 ((opcode_opcode_i & `INST_CSRRWI_MASK) != `INST_CSRRWI) &&
                 ((opcode_opcode_i & `INST_CSRRSI_MASK) != `INST_CSRRSI) &&
                 ((opcode_opcode_i & `INST_CSRRCI_MASK) != `INST_CSRRCI)) ||
                (mpriv != `PRIV_MACHINE)) begin

                next_stall = 1'b0;
                next_mcause = `MCAUSE_ILLEGAL_INSTRUCTION;
                next_mstatus[12:11] = mpriv;
                next_mstatus[7]     = mstatus[3];
                next_mstatus[5]     = mstatus[1];
                next_mstatus[4]     = mstatus[0];
                next_mstatus[3]     = 1'b0;
                next_mstatus[1]     = 1'b0;
                next_mstatus[0]     = 1'b0;
                next_mpriv = `PRIV_MACHINE;
                next_mepc  = opcode_pc_i;
                next_branch_csr_pc = mtvec + (next_mcause << 2);
                next_branch_csr_request = 1'b1;
                next_writeback_value = 32'b0;
                next_writeback_idx   = 5'b0;
                next_writeback_squash= 1'b0;
            end else begin
                next_stall = 1'b1;
                next_branch_csr_request = 1'b0;
                next_branch_csr_pc = 32'b0;
                next_writeback_squash = (opcode_opcode_i[11:7] == 5'b0);

                // Use a case statement to update one of our key CSRs.
                // (You can expand this case if you support additional CSRs.)
                case (opcode_opcode_i[31:20])
                    12'h300: begin // mstatus
                        if (csrrw_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mstatus;
                            next_mstatus = opcode_ra_operand_i;
                        end else if (csrrs_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mstatus;
                            next_mstatus = mstatus | (opcode_ra_operand_i & csr_mask);
                        end else if (csrrc_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mstatus;
                            next_mstatus = mstatus & (~opcode_ra_operand_i | ~csr_mask);
                        end else if (csrrwi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mstatus;
                            next_mstatus = {27'b0, opcode_opcode_i[19:15]};
                        end else if (csrrsi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mstatus;
                            next_mstatus = mstatus | ({27'b0, opcode_opcode_i[19:15]} & csr_mask);
                        end else if (csrrci_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mstatus;
                            next_mstatus = mstatus & ((~{27'b0, opcode_opcode_i[19:15]}) | ~csr_mask);
                        end
                    end
                    12'h305: begin // mtvec
                        if (csrrw_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mtvec;
                            next_mtvec = opcode_ra_operand_i;
                        end else if (csrrs_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mtvec;
                            next_mtvec = mtvec | (opcode_ra_operand_i & csr_mask);
                        end else if (csrrc_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mtvec;
                            next_mtvec = mtvec & (~opcode_ra_operand_i | ~csr_mask);
                        end else if (csrrwi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mtvec;
                            next_mtvec = {27'b0, opcode_opcode_i[19:15]};
                        end else if (csrrsi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mtvec;
                            next_mtvec = mtvec | ({27'b0, opcode_opcode_i[19:15]} & csr_mask);
                        end else if (csrrci_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mtvec;
                            next_mtvec = mtvec & ((~{27'b0, opcode_opcode_i[19:15]}) | ~csr_mask);
                        end
                    end
                    12'h341: begin // mepc
                        if (csrrw_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mepc;
                            next_mepc = opcode_ra_operand_i;
                        end else if (csrrs_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mepc;
                            next_mepc = mepc | (opcode_ra_operand_i & csr_mask);
                        end else if (csrrc_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mepc;
                            next_mepc = mepc & (~opcode_ra_operand_i | ~csr_mask);
                        end else if (csrrwi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mepc;
                            next_mepc = {27'b0, opcode_opcode_i[19:15]};
                        end else if (csrrsi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mepc;
                            next_mepc = mepc | ({27'b0, opcode_opcode_i[19:15]} & csr_mask);
                        end else if (csrrci_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mepc;
                            next_mepc = mepc & ((~{27'b0, opcode_opcode_i[19:15]}) | ~csr_mask);
                        end
                    end
                    12'h342: begin // mcause
                        if (csrrw_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mcause;
                            next_mcause = opcode_ra_operand_i;
                        end else if (csrrs_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mcause;
                            next_mcause = mcause | (opcode_ra_operand_i & csr_mask);
                        end else if (csrrc_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mcause;
                            next_mcause = mcause & (~opcode_ra_operand_i | ~csr_mask);
                        end else if (csrrwi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mcause;
                            next_mcause = {27'b0, opcode_opcode_i[19:15]};
                        end else if (csrrsi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mcause;
                            next_mcause = mcause | ({27'b0, opcode_opcode_i[19:15]} & csr_mask);
                        end else if (csrrci_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mcause;
                            next_mcause = mcause & ((~{27'b0, opcode_opcode_i[19:15]}) | ~csr_mask);
                        end
                    end
                    12'h344: begin // mip
                        if (csrrw_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mip;
                            next_mip = opcode_ra_operand_i;
                        end else if (csrrs_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mip;
                            next_mip = mip | (opcode_ra_operand_i & csr_mask);
                        end else if (csrrc_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mip;
                            next_mip = mip & (~opcode_ra_operand_i | ~csr_mask);
                        end else if (csrrwi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mip;
                            next_mip = {27'b0, opcode_opcode_i[19:15]};
                        end else if (csrrsi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mip;
                            next_mip = mip | ({27'b0, opcode_opcode_i[19:15]} & csr_mask);
                        end else if (csrrci_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mip;
                            next_mip = mip & ((~{27'b0, opcode_opcode_i[19:15]}) | ~csr_mask);
                        end
                    end
                    12'h304: begin // mie
                        if (csrrw_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mie;
                            next_mie = opcode_ra_operand_i;
                        end else if (csrrs_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mie;
                            next_mie = mie | (opcode_ra_operand_i & csr_mask);
                        end else if (csrrc_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mie;
                            next_mie = mie & (~opcode_ra_operand_i | ~csr_mask);
                        end else if (csrrwi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mie;
                            next_mie = {27'b0, opcode_opcode_i[19:15]};
                        end else if (csrrsi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mie;
                            next_mie = mie | ({27'b0, opcode_opcode_i[19:15]} & csr_mask);
                        end else if (csrrci_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mie;
                            next_mie = mie & ((~{27'b0, opcode_opcode_i[19:15]}) | ~csr_mask);
                        end
                    end
                    12'h340: begin // mscratch
                        if (csrrw_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mscratch;
                            next_mscratch = opcode_ra_operand_i;
                        end else if (csrrs_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mscratch;
                            next_mscratch = mscratch | (opcode_ra_operand_i & csr_mask);
                        end else if (csrrc_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mscratch;
                            next_mscratch = mscratch & (~opcode_ra_operand_i | ~csr_mask);
                        end else if (csrrwi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mscratch;
                            next_mscratch = {27'b0, opcode_opcode_i[19:15]};
                        end else if (csrrsi_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mscratch;
                            next_mscratch = mscratch | ({27'b0, opcode_opcode_i[19:15]} & csr_mask);
                        end else if (csrrci_w) begin
                            next_writeback_idx = opcode_opcode_i[11:7];
                            next_writeback_value = mscratch;
                            next_mscratch = mscratch & ((~{27'b0, opcode_opcode_i[19:15]}) | ~csr_mask);
                        end
                    end
                    default: begin
                        next_writeback_idx = opcode_opcode_i[11:7];
                        next_writeback_value = 32'b0;
                    end
                endcase
            end

        //--------------------------------------------------------------------------
        // Branch requests from the execute stage
        //--------------------------------------------------------------------------
        end else if (branch_exec_request_i) begin
            next_writeback_squash = 1'b0;
            next_stall = 1'b0;
            next_branch_csr_request = 1'b1;
            next_branch_csr_pc = branch_exec_pc_i;
            next_writeback_value = 32'b0;
            next_writeback_idx = 5'b0;
        end else begin
            next_branch_csr_request = 1'b0;
            next_stall = 1'b0;
            next_writeback_idx = 5'b0;
            next_writeback_value = 32'b0;
            next_branch_csr_pc = 32'b0;
            next_writeback_squash = 1'b0;
        end
    end

endmodule