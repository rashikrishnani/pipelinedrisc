module pipelinedrisc(input clk,rst,output [7:0] rsdata,rtdata,rd);
reg [15:0] IR_fetch,IR_decode,IR_execute,IR_mem,IR_write;
reg [7:0] A_store, A_mem;
reg [7:0] alu_ex_out;     // result of instruction currently in EX (captured every cycle)
reg [7:0] alu_result_mem; // alu_ex_out shifted one cycle (= result of inst now in MEM)
wire [6:0] pc;
wire [15:0] out;
reg [7:0] in, in2;
wire [7:0] data;
wire [7:0] rddata;
wire [2:0] rsreg, rtreg;
reg [7:0] A_execute, B_execute;
wire [2:0] rd_decode;
reg [2:0] rd_decodereg;
reg [9:0] addr_execute, addr_mem;
wire [7:0] rd_decodedata;
wire stall;
reg write, readdata, writeregister;

// Explicit WB-stage control pipeline
reg [2:0] wb_rd;
reg       wb_is_load;
reg       wb_is_alu;
reg [7:0] wb_alu_val;

// Branch control
reg [6:0] branch_pc;
reg       branch_en;

assign rsreg     = IR_decode[9:7];
assign rtreg     = IR_decode[6:4];
assign rd_decode = IR_decode[12:10];

wire [2:0] rd_execute = IR_execute[12:10];
wire [2:0] rd_mem     = IR_mem[12:10];
wire [2:0] rd_write   = wb_rd;

parameter LOAD=3'b000, STORE=3'b001,ALU=3'b010;
parameter  BEQ=3'b110, JMP=3'b111;
wire [7:0] imm = {5'b00000,IR_execute[6:4]};

parameter ADDI = 4'b1110;
parameter SUBI = 4'b1111;
wire isImm =
        (IR_execute[3:0]==ADDI) ||
        (IR_execute[3:0]==SUBI);
wire isImm_execute = (IR_execute[3:0]==ADDI)||(IR_execute[3:0]==SUBI);
wire isImm_decode  = (IR_decode[3:0]==ADDI)||(IR_decode[3:0]==SUBI);

wire isALU_execute = (IR_execute[15:13]==ALU);
wire isALU_mem      = (IR_mem[15:13]==ALU);
wire isALU_decode   = (IR_decode[15:13]==ALU);

wire isALU_write   = wb_is_alu;
wire isLOAD_write  = wb_is_load;

wire load_hazard = isALU_decode && (IR_execute!=0) && (IR_execute[15:13]==LOAD) &&
                   ((rd_execute==rsreg)||(!isImm_decode && rd_execute==rtreg));
wire hazard_mem  = isALU_decode && (IR_mem!=0) && (IR_mem[15:13]==LOAD) &&
                   ((rd_mem==rsreg)||(!isImm_decode && rd_mem==rtreg));
wire hazard_write= isALU_decode && (rd_write!=0) && isLOAD_write &&
                   ((rd_write==rsreg)||(!isImm_decode && rd_write==rtreg));


wire store_hazard=
    (IR_decode[15:13]==STORE) && (
        (isALU_execute && (rd_execute==rd_decode) && (rd_execute!=0)) ||
        (isALU_mem     && (rd_mem==rd_decode)     && (rd_mem!=0))     ||
        ((IR_execute[15:13]==LOAD) && (rd_execute==rd_decode) && (rd_execute!=0)) ||
        ((IR_mem[15:13]==LOAD)     && (rd_mem==rd_decode)     && (rd_mem!=0))
    );
wire branch_hazard =
    (IR_decode[15:13]==BEQ) && (
        (isALU_execute && ((rd_execute==IR_decode[12:10])||(rd_execute==IR_decode[9:7])) && (rd_execute!=3'b000)) ||
        (isALU_mem     && ((rd_mem==IR_decode[12:10])    ||(rd_mem==IR_decode[9:7]))     && (rd_mem!=3'b000))     ||
        ((IR_execute[15:13]==LOAD) && ((rd_execute==IR_decode[12:10])||(rd_execute==IR_decode[9:7])) && (rd_execute!=3'b000)) ||
        ((IR_mem[15:13]==LOAD)     && ((rd_mem==IR_decode[12:10])    ||(rd_mem==IR_decode[9:7]))     && (rd_mem!=3'b000))
    );

wire branch_fire = (IR_decode[15:13]==BEQ && rd_decodedata==rsdata) ||
                   (IR_decode[15:13]==JMP);

assign stall = load_hazard|hazard_mem|hazard_write|store_hazard|branch_hazard;

wire [7:0] aluA, aluB;
wire [7:0] wb_fwd = wb_alu_val;

wire fwdA_mem   = isALU_execute && isALU_mem   && (rd_mem!=0) && (rd_mem==IR_execute[9:7]);
wire fwdA_write = isALU_execute && (rd_write!=0) && (isALU_write||isLOAD_write) && (rd_write==IR_execute[9:7]);
wire fwdB_mem =
    !isImm_execute &&
    isALU_execute &&
    isALU_mem &&
    (rd_mem!=0) &&
    (rd_mem==IR_execute[6:4]);

wire fwdB_write =
    !isImm_execute &&
    isALU_execute &&
    (IR_write!=0) &&
    (isALU_write||IR_write[15:13]==LOAD) &&
    (rd_write!=0) &&
    (rd_write==IR_execute[6:4]);

assign aluA = fwdA_mem ? alu_ex_out : fwdA_write ? wb_fwd : A_execute;
assign aluB =  isImm       ? imm :fwdB_mem ? alu_ex_out : fwdB_write ? wb_fwd : B_execute;

wire store_fwd_mem   = (IR_decode[15:13]==STORE) && isALU_mem   && (rd_mem!=0) && (rd_mem==rd_decode);
wire store_fwd_write = (IR_decode[15:13]==STORE) && (rd_write!=0) && (isALU_write||isLOAD_write) && (rd_write==rd_decode);

always @(posedge clk or posedge rst) begin
    if(rst) begin
        write<=0; readdata<=0; writeregister<=0;
        IR_fetch<=0; IR_decode<=0; IR_execute<=0; IR_mem<=0; IR_write<=0;
        addr_execute<=0; addr_mem<=0;
        A_execute<=0; B_execute<=0; A_mem<=0;
        A_store<=0; rd_decodereg<=0;
        in<=0; in2<=0; alu_ex_out<=0; alu_result_mem<=0;
        branch_pc<=0; branch_en<=0;
        wb_rd<=0; wb_is_load<=0; wb_is_alu<=0; wb_alu_val<=0;
    end
    else begin
        write<=0; readdata<=0; writeregister<=0;
        branch_en <= 0;

        if(stall) begin
            IR_fetch     <= IR_fetch;
            IR_decode    <= IR_decode;
            rd_decodereg <= rd_decodereg;
            IR_execute   <= 16'b0;
            if((isALU_write||isLOAD_write) &&
               (rd_write!=0) && (rd_write==rd_decodereg))
                A_store <= wb_fwd;
        end
        else begin
            IR_fetch     <= out;
            IR_execute   <= IR_decode;
            IR_decode    <= IR_fetch;
            A_execute    <= rsdata;
            B_execute    <= rtdata;
            rd_decodereg <= rd_decode;

            // addr_execute captures THIS cycle's IR_decode address directly
            if(IR_decode[15:13]==LOAD || IR_decode[15:13]==STORE)
                addr_execute <= IR_decode[9:0];

            if(IR_decode[15:13]==STORE) begin
                if(store_fwd_mem)        A_store <= alu_ex_out;
                else if(store_fwd_write) A_store <= wb_fwd;
                else                     A_store <= rd_decodedata;
            end

            if(branch_fire) begin
                IR_fetch  <= 0;
                IR_decode <= 0;
                branch_pc <= IR_decode[6:0];
                branch_en <= 1;
            end
        end

        // alu_ex_out: capture EX-stage ALU result EVERY cycle unconditionally
        // (even during a stall caused by the instruction currently in EX,
        // e.g. store_hazard) since that result still needs to flow to MEM/WB.
        alu_ex_out <= rd;

        // alu_result_mem: shifts alu_ex_out one cycle behind (always)
        alu_result_mem <= alu_ex_out;

        // EX -> MEM
        IR_mem   <= IR_execute;
        addr_mem <= addr_execute;
        A_mem    <= A_store;

        // STORE write: gate on IR_execute (matches addr_execute's timing)
        // instead of IR_mem, so write/address land together correctly.
        if(IR_execute[15:13]==STORE && IR_execute!=0) write <= 1;
        in <= A_store;

        // MEM -> WB
        IR_write <= IR_mem;

        // ---- WB ----
        wb_rd      <= rd_mem;
        wb_is_load <= (IR_mem[15:13]==LOAD) && (IR_mem!=0);
        wb_is_alu  <= isALU_mem;
        if(IR_mem[15:13]==LOAD && IR_mem!=0) begin
            wb_alu_val    <= data;
            writeregister <= 1;
            in2           <= data;
        end
        else if(isALU_mem) begin
            wb_alu_val    <= alu_ex_out;
            writeregister <= 1;
            in2           <= alu_ex_out;
        end
    end
end

pc p1(clk, rst, !stall, branch_pc, branch_en, pc);
instructionmemory m1(pc, out);
datamemory    m2(in, addr_mem, write, readdata, clk, data);
registers     r1(in2, wb_rd, rsreg, rtreg, rd_decode, writeregister,
                 clk, rsdata, rtdata, rddata, rd_decodedata);
alu           a1(aluA, aluB, IR_execute[15:13],IR_execute[3:0], rd);
endmodule

module pc(input clk, rst, pc_write, input [6:0] load_pc, input load_en,
          output reg [6:0] pc);
always @(posedge clk or posedge rst) begin
    if(rst) pc <= 7'd0;
    else if(pc_write) begin
        if(load_en) pc <= load_pc;
        else        pc <= pc + 1;
    end
end
endmodule

module datamemory(input [7:0] d,input [9:0]address,input write,readdata,clk,output [7:0] out);
reg [7:0] mem [0:1023];
assign out = mem[address];
always@(posedge clk)begin
    if(write)begin
    mem[address]<=d;
    end
end
endmodule

module instructionmemory(input [6:0]pc,output [15:0] out);
reg [15:0] mem[0:127];
assign out=mem[pc];
endmodule

module registers(input [7:0]d,input [2:0]rd,rs,rt,rd_decode,input writeregister,clk,output [7:0] rsdata,rtdata,rddata,rd_decodedata);
reg [7:0] regs [0:7];
initial begin
    for(integer i=0;i<8;i=i+1)
        regs[i] = 0;
end
// Write-first bypass: avoids same-cycle write-then-read hazard
assign rsdata = (writeregister && rd!=0 && rd==rs) ? d : regs[rs];
assign rtdata = (writeregister && rd!=0 && rd==rt) ? d : regs[rt];
assign rddata = regs[rd];
assign rd_decodedata = (writeregister && rd!=0 && rd==rd_decode) ? d : regs[rd_decode];
always @(posedge clk)begin
if(writeregister)begin
    regs[rd]<=d;
end
end
endmodule

module alu(input [7:0] rs,rt,input [2:0] opcode,input [3:0] funct,output reg[7:0] rd);

always @(*) begin
    rd   = 8'b0;
    case(opcode)

        3'b010: begin // ALU
         case(funct)
            4'b0000:
           rd=rs+rt;
           4'b0001:
           rd=rs-rt;
           4'b0010:
           rd=rs*rt;
           4'b0011:
           rd=rs&rt;
           4'b0100:
           rd=rs|rt;
           4'b0101:
           rd=rs^rt;
           4'b0110:
           rd=rs<rt;
            4'b0111:
           rd=rs>rt;
            4'b1000:
           rd=rs==rt;
            4'b1001:
           rd=rs!=rt;
            4'b1010:
           rd=rs<<rt;
            4'b1011:
           rd=rs>>rt;
            4'b1100:
           rd=$signed(rs) <<< rt;
           4'b1101:
           rd= $signed(rs) >>> rt;
           4'b1110:
           rd=rs+rt;
           4'b1111:
           rd=rs-rt;
         endcase
        end
        3'b100:begin //mov
           rd=rs;
        end
        default: begin// for jump
         rd=8'b0;
        end 
    endcase

end


endmodule
