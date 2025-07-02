module my_controller #(parameter WIDTH = 32) (
    input clk, reset,
    input [WIDTH-1:0] INSTRUCTION,
    input [3:0] ALUFlags,
    output reg [3:0] ALUControl,
    output PCWrite, IRWrite, MemWrite, RegWrite, PCS, AdrSrc,
    output reg  ALUSrcA, shifter_rot_control, ShifterRes,
    output reg [1:0] ALUSrcB, ResultSrc, ImmSrc,
    output [2:0] RegSrc,
    output [3:0] fsm_state
);
// FSM variables
reg [3:0] state, nextstate;
localparam S0  = 4'h0;
localparam S1  = 4'h1;
localparam S2  = 4'h2;
localparam S3  = 4'h3;
localparam S4  = 4'h4;
localparam S5  = 4'h5;
localparam S6  = 4'h6;
localparam S7  = 4'h7;
localparam S8  = 4'h8;
localparam S9  = 4'h9;
localparam S10 = 4'hA;
localparam S11 = 4'hB;
localparam S12 = 4'hC;
localparam S13 = 4'hD;
localparam S14 = 4'hE;
localparam S15 = 4'hF;
/////////

wire DP, MEM, BRANCH, BX;
wire [3:0] cmd;
wire DPReg, DPImm, I, S;

// Decoding
wire [1:0] Op;
wire [3:0] cond, Rd;
wire [5:0] funct;

assign cond = INSTRUCTION[31:28];
assign Op = INSTRUCTION[27:26];
assign funct = INSTRUCTION[25:20];
assign Rd = INSTRUCTION[15:12];
assign cmd = funct[4:1];
assign I = funct[5];
assign S = funct[0];


assign DP = ~Op[1] && ~Op[0];
assign MEM = ~Op[1] && Op[0];
assign BRANCH = Op[1] && ~Op[0];
assign BX = (INSTRUCTION[27:4] == 24'h12FFF1);

assign DPReg = ~I && DP;
assign DPImm = I && DP;
//////

assign fsm_state = state;

// Finite State Machine
always @(posedge clk) begin
    if (reset) begin
        state <= S0;
    end else begin
        state <= nextstate;
    end
end

always @(*) begin
    case (state)
        S0: nextstate <= S1;
        S1: begin 
                if (BX && DP) begin
                    nextstate <= S10;
                end else if (MEM) begin 
                    nextstate <= S2; 
                end else if (DPReg) begin
                    nextstate <= S6;
                end else if (DPImm) begin
                    nextstate <= S7;
                end else if (BRANCH) begin
                    nextstate <= S9;
                end
            end
        S2: begin
                if (S) begin
                    nextstate <= S3;
                end else begin
                    nextstate <= S5;
                end
            end
        S3: nextstate <= S4;
        S4: nextstate <= S0;
        S5: nextstate <= S0;
        S6: nextstate <= S8; 
        S7: nextstate <= S8;
        S8: nextstate <= S0;
        S9: nextstate <= S0;
        S10:nextstate <= S0;
        default: nextstate <= S0;
    endcase
end
////////////////


wire [1:0] FlagW, FlagWrite;
reg CondEx, CondExDelay;
//reg FlagZ, FlagC, FlagN, FlagV;
wire Z, N,C,V;

// Conditional Enable
assign FlagW[1] = (DP && S) && ((state == S6) || (state == S7));
assign FlagW[0] = (DP && S) && ((cmd == 4'b0100) || (cmd == 4'b0010) || (cmd == 4'b1010)) && ((state == S6) || (state == S7));
assign FlagWrite[0] = FlagW[0] & CondEx;
assign FlagWrite[1] = FlagW[1] & CondEx;
// Registers with enable for flag setting


//always @(posedge clk) begin
//    if (reset) begin
//        FlagZ <= 1'b0;
//    end else begin
//        if (FlagWrite[1]) begin
//            FlagZ <= ALUFlags[2];
//            FlagN <= ALUFlags[3];
//        end else begin
//            FlagZ <= FlagZ;
//            FlagN <= FlagN;
//        end
        
//        if (FlagWrite[0]) begin
//            FlagV <= ALUFlags[0];
//            FlagC <= ALUFlags[1];
//        end else begin
//            FlagV <= FlagV;
//            FlagC <= FlagC;
//        end
//    end

//end


Register_rsten #(.WIDTH(1)) N_reg (
	.clk(clk),
	.reset(reset),
	.we(FlagWrite[1]),
	.DATA(ALUFlags[3]),
	.OUT(N)
);

Register_rsten #(.WIDTH(1)) Z_reg (
	.clk(clk),
	.reset(reset),
	.we(FlagWrite[1]),
	.DATA(ALUFlags[2]),
	.OUT(Z)
);

Register_rsten #(.WIDTH(1)) C_reg (
	.clk(clk),
	.reset(reset),
	.we(FlagWrite[0]),
	.DATA(ALUFlags[1]),
	.OUT(C)
);

Register_rsten #(.WIDTH(1)) V_reg (
	.clk(clk),
	.reset(reset),
	.we(FlagWrite[0]),
	.DATA(ALUFlags[0]),
	.OUT(V)
);
	
// Conditional Check

//assign CondEx = (cond == 4'b1110) || ((cond == 4'b0000) && FlagZ) || ((cond == 4'b0001) && ~FlagZ);

always @(*) begin
	case (cond)
		4'h0: CondEx = Z;
		4'h1: CondEx = ~Z;
		4'h2: CondEx = C;
		4'h3: CondEx = ~C;
		4'h4: CondEx = N;
		4'h5: CondEx = ~N;
		4'h6: CondEx = V;
		4'h7: CondEx = ~V;
		4'h8: CondEx = ~Z && C;
		4'h9: CondEx = Z || ~C;
		4'ha: CondEx = ~(N ^ V);
		4'hb: CondEx = N ^ V;
		4'hc: CondEx = ~Z && ~(N ^ V);
		4'hd: CondEx = Z || (N ^ V);
		4'he: CondEx = 1'b1;
		default: CondEx = 1'b0;
	endcase
end

always @(posedge clk) begin
	if (reset) begin 
	   CondExDelay <= 1'b0; 
	end else begin 
	   CondExDelay <= CondEx; 
	end
end
///////

wire BranchW, RegW, MemW, NextPC;

localparam AND=4'b0000,
		  EXOR=4'b0001,
		  SUBAB=4'b0010,
		  SUBBA=4'b0011,
		  ADD=4'b0100,
		  ADDC=4'b0101,
		  SUBCAB=4'b0110,
		  SUBCBA=4'b0111,
		  ORR=4'b1100,
		  MOVE=4'b1101,
		  BITCLEAR=4'b1110,
		  MOVEN=4'b1111;


// Logics
assign PCS = BranchW || ((Rd == 4'b1111) && RegW);
assign PCWrite = NextPC || (PCS && CondExDelay);
assign RegWrite = RegW && CondExDelay;
assign MemWrite = MemW && CondExDelay;

assign IRWrite = (state == S0);
assign NextPC = (state == S0);
assign AdrSrc = (state == S3) || (state == S5);
assign RegW = (state == S4) || ((state == S8) && (cmd != 4'b1010)) || ((state == S9) && INSTRUCTION[24]);
assign BranchW = (state == S9) || (state == S10);
assign MemW = (state == S5);

// RegSrc Logic
assign RegSrc[0] = BRANCH;
assign RegSrc[1] = MEM;
assign RegSrc[2] = BRANCH && INSTRUCTION[24];

// Extender Logic
always @(*) begin
	if (DPImm) begin
		ImmSrc = 2'b00;
	end else if (MEM) begin
		ImmSrc = 2'b01;
	end else if (BRANCH) begin
		ImmSrc = 2'b10;
	end else begin
		ImmSrc = 2'b11;
	end
end

always @(*) begin
    case (state) 
        S0: begin
            ALUSrcA = 1'b1;       // A, PC
            ALUSrcB = 2'b10;       // WriteData, ExtImm, 4
            ResultSrc = 2'b10;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = ADD; 
            end
            
        S1: begin
            ALUSrcA = 1'b1;       // A, PC
            ALUSrcB = 2'b10;       // WriteData, ExtImm, 4
            ResultSrc = 2'b10;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = ADD; 
            end
            
        S2: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b01;       // WriteData, ExtImm, 4
            ResultSrc = 2'b00;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = ADD; 
            end
            
        S3: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b00;       // WriteData, ExtImm, 4
            ResultSrc = 2'b00;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = ADD; 
            end
            
        S4: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b00;       // WriteData, ExtImm, 4
            ResultSrc = 2'b01;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = ADD; 
            end
            
        S5: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b00;       // WriteData, ExtImm, 4
            ResultSrc = 2'b00;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = ADD; 
            end
            
        S6: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b00;       // WriteData, ExtImm, 4
            ResultSrc = 2'b00;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = ~(cmd == 4'b1010);            // Unshifted, Shifted
            
            if (cmd == 4'b1010) begin
                ALUControl = 4'b0010; // SUB for CMP
            end else if (cmd == 4'b1001) begin
                ALUControl = 4'b1101; // To Move the Rm to SrcB
            end else begin
                ALUControl = cmd;
            end
            end
            
        S7: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b01;       // WriteData, ExtImm, 4
            ResultSrc = 2'b01;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b1;   // RR
            ShifterRes = ~(cmd == 4'b1010);            // Unshifted, Shifted
            
            if (cmd == 4'b1010) begin
                ALUControl = 4'b0010; // SUB for CMP
            end else if (cmd == 4'b1001) begin
                ALUControl = 4'b1101; // To Move the Rm to SrcB
            end else begin
                ALUControl = cmd;
            end 
            end
            
        S8: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b01;       // WriteData, ExtImm, 4
            ResultSrc = 2'b00;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b1;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = MOVE; 
            end
            
        S9: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b01;       // WriteData, ExtImm, 4
            ResultSrc = 2'b10;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = ADD; 
            end
            
        S10: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b00;       // WriteData, ExtImm, 4
            ResultSrc = 2'b10;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = MOVE; 
            end
       default: begin
            ALUSrcA = 1'b0;       // A, PC
            ALUSrcB = 2'b00;       // WriteData, ExtImm, 4
            ResultSrc = 2'b00;    // ALUOut, Data, ALUResult
            
            shifter_rot_control = 1'b0;   // RR
            ShifterRes = 1'b0;            // Unshifted, Shifted
            
            ALUControl = ADD; 
            end
    endcase
end

endmodule
