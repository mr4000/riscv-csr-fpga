# RISC-V CSR Module (EE705 VLSI Design Lab 2025)

This project implements a Control and Status Register (CSR) module in Verilog for a RISC-V processor (RV32I), developed as part of the EE705 VLSI Design Lab course.

## 🔧 Features

* Implements key CSR registers: `mepc`, `mcause`, `mstatus`, `mtvec`, `mip`, `mie`, `mpriv`, `mcycle`, `mscratch`
* Supports CSR instructions: `CSRRW`, `CSRRS`, `CSRRC`, `CSRRWI`, `CSRRSI`, `CSRRCI`
* Handles external interrupts and exceptions (e.g., illegal instruction, breakpoint, ecall)
* Supports MRET for returning from interrupts and exceptions
* CSR access control based on privilege level
* Integrated writeback and branch interface
* Fully compliant with the RISC-V RV32I and privileged ISA specifications

## 🗂️ Folder Structure

```
CSR_Design_Dynamos/
├── Verilog_code_and_Testbench/
│   ├── riscv_csr.v              # Main CSR module
│   ├── risc_csr_tb.v            # Testbench for CSR module
│   ├── def.v                    # CSR register definitions and macros
├── project_report.pdf       # Final lab report
 
  



## ▶️ How to Run

1. Open Vivado or ModelSim
2. Load files from `Verilog_code_and_Testbench/`
3. Run testbench `risc_csr_tb.v` and observe outputs
4. Use ILA/VIO setups in `fpga/` folder for FPGA testing

## 🛠 Tools Used

* Vivado (Xilinx Design Suite)
* Git (Version Control)
* Verilog HDL
* Xilinx FPGA Board (e.g., PYNQ-Z2)

## 👨‍🎓 Author

**Manish Ranjan**
M.Tech VLSI Design, IIT Bombay
EE705 VLSI Design Lab Project (2025)

For any questions or suggestions, feel free to open an issue or fork the repo!
