module Multi_Cycle_Computer #(parameter WIDTH = 32)(
    input clk, reset,
    input [3:0] debug_reg_select, fsm_state,
    output [WIDTH-1:0] debug_reg_out, fetchPC
    );
    
wire PCWrite, IRWrite, MemWrite, AdrSrc, ALUSrcA, RegWrite, shifter_rot_control, ShifterRes;
wire [1:0] ImmSrc, ALUSrcB, ResultSrc;
wire [2:0] RegSrc;
wire [3:0] ALUControl, ALUFlags;
wire [WIDTH-1:0] INSTRUCTION;

my_datapath my_datapath (
    .clk(clk),
    .reset(reset),
    .PCWrite(PCWrite),
    .IRWrite(IRWrite),
    .MemWrite(MemWrite), 
    .AdrSrc(AdrSrc),
    .ALUSrcA(ALUSrcA),
    .RegWrite(RegWrite),
    .shifter_rot_control(shifter_rot_control),
    .ShifterRes(ShifterRes),
    .ImmSrc(ImmSrc),
    .ALUSrcB(ALUSrcB),
    .ResultSrc(ResultSrc),
    .RegSrc(RegSrc),
    .Debug_Source_select(debug_reg_select),
    .ALUControl(ALUControl),
    .ALUFlags(ALUFlags),
    .INSTRUCTION(INSTRUCTION),
    .Debug_out(debug_reg_out),
    .PC(fetchPC)
);

my_controller my_controller ( 
    .clk(clk),
    .reset(reset),
    .PCWrite(PCWrite),
    .IRWrite(IRWrite),
    .MemWrite(MemWrite), 
    .AdrSrc(AdrSrc),
    .ALUSrcA(ALUSrcA),
    .RegWrite(RegWrite),
    .shifter_rot_control(shifter_rot_control),
    .ShifterRes(ShifterRes),
    .ImmSrc(ImmSrc),
    .ALUSrcB(ALUSrcB),
    .ResultSrc(ResultSrc),
    .RegSrc(RegSrc),
    .ALUControl(ALUControl),
    .ALUFlags(ALUFlags),
    .INSTRUCTION(INSTRUCTION),
    .fsm_state(fsm_state)
);
    
endmodule
