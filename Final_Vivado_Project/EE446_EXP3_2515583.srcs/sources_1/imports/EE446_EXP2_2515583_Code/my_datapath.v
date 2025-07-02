module my_datapath #(parameter WIDTH=32)(
    input clk, reset,
    input PCWrite, IRWrite, MemWrite, AdrSrc, ALUSrcA, RegWrite, shifter_rot_control, ShifterRes,
    input [1:0] ImmSrc, ALUSrcB, ResultSrc,
    input [2:0] RegSrc,
    input [3:0] Debug_Source_select, ALUControl,
    output [WIDTH-1:0] INSTRUCTION, Debug_out, PC,
    output [3:0] ALUFlags
);

wire [WIDTH-1:0] PCNot, Adr, WriteData, ReadData, Data, Result, RD1, RD2, ExtImm;
wire [1:0] shifter_control_value;
wire [4:0] shifter_shamt_value;
wire [3:0] RA1, RA2, Rd_value;
wire [WIDTH-1:0] shifter_result, shifter_input, A_wire, SrcA, SrcB, ALUResult, ALUOut;


Register_rsten #(.WIDTH(WIDTH)) reg_PC (
    .clk(clk),
    .reset(reset),
    .we(PCWrite),
    .DATA(Result),
    .OUT(PC)
);

Mux_2to1 #(.WIDTH(WIDTH)) mux_Adr (
    .select(AdrSrc),
    .input_0(PC),
    .input_1(Result),
    .output_value(Adr)
);

ID_memory #(.ADDR_WIDTH(WIDTH), .BYTE_SIZE(WIDTH)) IDMemory (
    .clk(clk),
    .WE(MemWrite),
    .WD(WriteData),
    .RD(ReadData),
    .ADDR(Adr)
);

Register_en #(.WIDTH(WIDTH)) reg_INST (
    .clk(clk),
    .en(IRWrite),
    .DATA(ReadData),
    .OUT(INSTRUCTION)
);

Register_simple #(.WIDTH(WIDTH)) reg_DATA (
    .clk(clk),
    .DATA(ReadData),
    .OUT(Data)
);

Mux_2to1 #(.WIDTH(4)) mux_RA1 (
	.select(RegSrc[0]),
	.input_0(INSTRUCTION[19:16]),
	.input_1(4'b1111),
	.output_value(RA1)
);
		
Mux_2to1 #(.WIDTH(4)) mux_RA2 (
    .select(RegSrc[1]),
    .input_0(INSTRUCTION[3:0]),
    .input_1(INSTRUCTION[15:12]),
    .output_value(RA2)
);

Mux_2to1 #(.WIDTH(4)) mux_Rd (
   .select(RegSrc[2]),
   .input_0(INSTRUCTION[15:12]),
   .input_1(4'b1110),
   .output_value(Rd_value)
);

wire [WIDTH-1:0] regWriteData;


Mux_2to1 #(.WIDTH(WIDTH)) mux_WriteData (
   .select(RegSrc[2]),
   .input_0(Result),
   .input_1(PC),
   .output_value(regWriteData)
);

Register_file #(.WIDTH(WIDTH)) register_file (
		.clk(clk),
		.write_enable(RegWrite),
		.reset(reset),
		.Source_select_0(RA1),
		.Source_select_1(RA2),
		.Debug_Source_select(Debug_Source_select),
		.Destination_select(Rd_value),
		.DATA(regWriteData),
		.Reg_15(Result),
		.out_0(RD1),
		.out_1(RD2),
		.Debug_out(Debug_out)	
	);

Register_simple #(.WIDTH(WIDTH)) reg_A (
    .clk(clk),
    .DATA(RD1),
    .OUT(A_wire)
);

Register_simple #(.WIDTH(WIDTH)) reg_WD (
    .clk(clk),
    .DATA(RD2),
    .OUT(WriteData)
);

Extender extender (
    .Extended_data(ExtImm),
    .DATA(INSTRUCTION[23:0]),
    .select(ImmSrc)
);

Mux_4to1 #(.WIDTH(WIDTH)) mux_shifter_input (
    .select(ALUSrcB),
    .input_0(WriteData),
    .input_1(ExtImm),
    .input_2(32'h4),
    .input_3(32'h0),
    .output_value(shifter_input)
);

Mux_2to1 #(.WIDTH(5)) mux_shifter_shamt (
    .select(shifter_rot_control),
    .input_0(INSTRUCTION[11:7]),
    .input_1({INSTRUCTION[11:8], 1'b0}),
    .output_value(shifter_shamt_value)
    );
    
Mux_2to1 #(.WIDTH(2)) mux_shifter_control (
    .select(shifter_rot_control),
    .input_0(INSTRUCTION[6:5]),
    .input_1(2'b11),
    .output_value(shifter_control_value)
    );

shifter #(.WIDTH(WIDTH)) shifter0 (
    .control(shifter_control_value),
    .shamt(shifter_shamt_value),
    .DATA(shifter_input),
    .OUT(shifter_result)
);

Mux_2to1 #(.WIDTH(WIDTH)) mux_SrcB (
    .select(ShifterRes),
    .input_0(shifter_input),
    .input_1(shifter_result),
    .output_value(SrcB)
);

Mux_2to1 #(.WIDTH(WIDTH)) mux_SrcA (
    .select(ALUSrcA),
    .input_0(A_wire),
    .input_1(PC),
    .output_value(SrcA)
);

ALU #(.WIDTH(WIDTH)) alu0  (
    .control(ALUControl),
    .DATA_A(SrcA),
    .DATA_B(SrcB),
    .OUT(ALUResult),
    .CI(1'b0),
    .CO(ALUFlags[1]),
    .OVF(ALUFlags[0]),
    .N(ALUFlags[3]),
    .Z(ALUFlags[2])
);

Register_simple #(.WIDTH(WIDTH)) reg_ALURes (
    .clk(clk),
    .DATA(ALUResult),
    .OUT(ALUOut)
);

Mux_4to1 #(.WIDTH(WIDTH)) mux_res (
    .select(ResultSrc),
    .input_0(ALUOut),
    .input_1(Data),
    .input_2(ALUResult),
    .input_3(32'h0),
    .output_value(Result)
);

endmodule