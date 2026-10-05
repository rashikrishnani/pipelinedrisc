# Pipelined RISC Processor

A 5-stage pipelined RISC-style processor written in Verilog, with full data-hazard
forwarding, load-use stalling, and branch/jump handling.

## Pipeline stages

| Stage | Register | Purpose |
|---|---|---|
| IF (Fetch) | `IR_fetch` | Fetch instruction from `instructionmemory` at the current `pc` |
| ID (Decode) | `IR_decode` | Decode opcode/registers, read `registers` file, detect hazards |
| EX (Execute) | `IR_execute` | Run the ALU (with forwarded operands) |
| MEM (Memory) | `IR_mem` | Access `datamemory` for LOAD/STORE |
| WB (Write-back) | `IR_write` | Write ALU/LOAD result back into `registers` |

Each stage's instruction word is carried forward one clock at a time through the
`IR_fetch -> IR_decode -> IR_execute -> IR_mem -> IR_write` chain in
`pipelinedrisc`'s main `always @(posedge clk)` block.

## Instruction format (16 bits)

```
 15        13 12      10 9        7 6        4 3        0
+------------+----------+----------+----------+----------+
|   opcode   |    rd    |    rs    |    rt    |  funct   |
+------------+----------+----------+----------+----------+
```

- `opcode` (`IR[15:13]`) selects the instruction class.
- `rd` (`IR[12:10]`) is the destination register for ALU/LOAD instructions.
- `rs` (`IR[9:7]`) is the first source register.
- `rt` (`IR[6:4]`) is the second source register, *or* for ADDI/SUBI the low 3
  bits of `rt` (`IR[6:4]`) are reused as an immediate value (`imm`, zero-extended
  to 8 bits).
- `funct` (`IR[3:0]`) selects the ALU operation when `opcode == ALU`.

### Opcodes

| Opcode | Value | Meaning |
|---|---|---|
| `LOAD`  | `3'b000` | `rd <= mem[address]` |
| `STORE` | `3'b001` | `mem[address] <= rs` |
| `ALU`   | `3'b010` | `rd <= f(rs, rt_or_imm)` per `funct` |
| `MOV`   | `3'b100` | `rd <= rs` |
| `BEQ`   | `3'b110` | branch to `IR[6:0]` if `rs == rd` (register compare, see Branching below) |
| `JMP`   | `3'b111` | unconditional jump to `IR[6:0]` |

### ALU `funct` codes (when `opcode == ALU`)

| funct | Operation | funct | Operation |
|---|---|---|---|
| `0000` | `rs + rt` | `1000` | `rs == rt` |
| `0001` | `rs - rt` | `1001` | `rs != rt` |
| `0010` | `rs * rt` | `1010` | `rs << rt` |
| `0011` | `rs & rt` | `1011` | `rs >> rt` |
| `0100` | `rs \| rt` | `1100` | `rs <<< rt` (arithmetic) |
| `0101` | `rs ^ rt` | `1101` | `rs >>> rt` (arithmetic) |
| `0110` | `rs < rt`  | `1110` | `rs + imm` (ADDI) |
| `0111` | `rs > rt`  | `1111` | `rs - imm` (SUBI) |

ADDI/SUBI (`funct == 1110/1111`) substitute the zero-extended immediate
(`IR_execute[6:4]`) for `rt` at the ALU input mux (`aluB`).

## Hazard handling

The design stalls the pipeline (freezes IF/ID, bubbles EX) whenever a hazard
can't be resolved by forwarding alone, and forwards ALU/LOAD results from
later pipeline stages back into EX whenever it can:

- **`load_hazard` / `hazard_mem` / `hazard_write`** -- a LOAD currently in
  EX, MEM, or about to write back in WB targets a register the instruction in
  ID is about to read. Forwarding can't help here because the loaded value
  isn't available yet, so the pipeline stalls one cycle.
- **`store_hazard`** -- a STORE in ID needs the value just computed by an
  ALU/LOAD instruction still in EX or MEM; stalls until that value is ready.
- **`branch_hazard`** -- a BEQ in ID needs operands still being computed by
  an ALU/LOAD instruction in EX or MEM; stalls until the comparison operands
  are valid.
- **Forwarding paths** (`fwdA_mem`, `fwdA_write`, `fwdB_mem`, `fwdB_write`,
  `store_fwd_mem`, `store_fwd_write`) bypass EX/MEM and MEM/WB results
  directly into the ALU's `aluA`/`aluB` inputs, or into the value a STORE
  writes to memory, without needing a stall.
- **`stall`** is the OR of every hazard above, and gates the PC (`!stall`
  feeds `pc_write`) and inserts a bubble (`IR_execute <= 16'b0`) into EX.

## Branching

`branch_fire` resolves in the ID stage: a BEQ fires if the two decoded
operands (`rsdata`, `rd_decodedata`) are equal, and JMP always fires. On a
fire, `IR_fetch` and `IR_decode` are squashed (set to 0) and `branch_pc` /
`branch_en` redirect the PC to `IR_decode[6:0]` on the next cycle.

## Modules

| Module | Role |
|---|---|
| `pipelinedrisc` | Top-level pipeline: stage registers, hazard detection, forwarding, stage interconnect |
| `pc` | Program counter: increments each cycle, loads `branch_pc` when `load_en` is asserted, holds when `pc_write` is low (stall) |
| `instructionmemory` | 128 x 16-bit instruction ROM, combinationally read by `pc` |
| `datamemory` | 1024 x 8-bit data RAM, synchronous write / combinational read |
| `registers` | 8 x 8-bit register file with write-first bypass (same-cycle write-then-read returns the new value) |
| `alu` | Combinational ALU; operation selected by `opcode`/`funct` as above |

## Top-level ports

```verilog
module pipelinedrisc(
    input  clk, rst,
    output [7:0] rsdata, rtdata, rd
);
```

- `clk`, `rst` -- clock and synchronous reset (active high).
- `rsdata`, `rtdata` -- live register-file read ports for the instruction
  currently in ID (useful for observing decode-stage operands in simulation).
- `rd` -- combinational ALU output for the instruction currently in EX.

## Simulating

This is a synchronous design with no top-level stimulus generator built in.
To simulate:

1. Preload `instructionmemory`'s `mem` array (e.g. via `$readmemh`/`$readmemb`
   in a testbench, or by initializing it directly) with your program.
2. Instantiate `pipelinedrisc`, drive `clk` and pulse `rst`.
3. Observe `rsdata`/`rtdata`/`rd`, or add testbench probes into `registers.regs`
   and `datamemory.mem` to check final architectural state.

No XDC/timing files are included here since this processor has an internal
clock port (`clk`) already, so a standard `create_clock -period <T> [get_ports clk]`
constraint is enough for timing closure -- no virtual clock or wrapper needed,
unlike the combinational multiplier designs earlier in this conversation.
