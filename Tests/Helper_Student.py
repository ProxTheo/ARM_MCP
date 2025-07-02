class Timing:
    # Define how many cycles each operation takes
    Data_Processing = 4
    Memory_Load = 5
    Memory_Store = 4
    Branch = 3

def ToHex(value):
    try:
        ret = hex(value.integer)
    except: #If there are 'x's in the value
        ret = "0b" + str(value)
    return ret

#Populate the below functions as in the example lines of code to print your values for debugging
def Log_Datapath(dut,logger):
    #Log whatever signal you want from the datapath, called before positive clock edge
    logger.debug("************ DUT DATAPATH Signals ***************")
    #dut._log.info("reset:%s", ToHex(dut.my_datapath.reset.value))
    #dut._log.info("ALUSrcA:%s", ToHex(dut.my_datapath.ALUSrcA.value))
    #dut._log.info("ALUSrcB:%s", ToHex(dut.my_datapath.ALUSrcB.value))
    #dut._log.info("SrcA:%s", ToHex(dut.my_datapath.SrcA.value))
    #dut._log.info("SrcB:%s", ToHex(dut.my_datapath.SrcB.value))
    #dut._log.info("ALUResult:%s", ToHex(dut.my_datapath.ALUResult.value))
    #dut._log.info("MemWrite:%s", ToHex(dut.my_datapath.MemWrite.value))
    #dut._log.info("RegWrite:%s", ToHex(dut.my_datapath.RegWrite.value))
    #dut._log.info("PCSrc:%s", ToHex(dut.my_datapath.PCSrc.value))
    #dut._log.info("MemtoReg:%s", ToHex(dut.my_datapath.MemtoReg.value))
    #dut._log.info("RegSrc:%s", ToHex(dut.my_datapath.RegSrc.value))
    #dut._log.info("ImmSrc:%s", ToHex(dut.my_datapath.ImmSrc.value))
    #dut._log.info("ALUControl:%s", ToHex(dut.my_datapath.ALUControl.value))
    #dut._log.info("CO:%s", ToHex(dut.my_datapath.CO.value))
    #dut._log.info("OVF:%s", ToHex(dut.my_datapath.OVF.value))
    #dut._log.info("N:%s", ToHex(dut.my_datapath.N.value))
    #dut._log.info("Flags:%s", ToHex(dut.my_datapath.ALUFlags.value))
    #dut._log.info("CarryIN:%s", ToHex(dut.my_datapath.CarryIN.value))
    #dut._log.info("ShiftControl:%s", ToHex(dut.my_datapath.shifter_rot_control.value))
    #dut._log.info("ExtImm:%s", ToHex(dut.my_datapath.ExtImm.value))
    #dut._log.info("shamt:%s", ToHex(dut.my_datapath.shamt.value))
    #dut._log.info("PC:%s", ToHex(dut.my_datapath.PC.value))
    #dut._log.info("Instruction:%s", ToHex(dut.my_datapath.INSTRUCTION.value))
    #dut._log.info("regWriteData:%s", ToHex(dut.my_datapath.regWriteData.value))
    #dut._log.info("Result:%s", ToHex(dut.my_datapath.Result.value))

def Log_Controller(dut,logger):
    #Log whatever signal you want from the controller, called before positive clock edge
    logger.debug("************ DUT Controller Signals ***************")
    #dut._log.info("Op:%s", ToHex(dut.my_controller.Op.value))
    #dut._log.info("Funct:%s", ToHex(dut.my_controller.Funct.value))
    #dut._log.info("Rd:%s", ToHex(dut.my_controller.Rd.value))
    #dut._log.info("Src2:%s", ToHex(dut.my_controller.Src2.value))
    #dut._log.info("PCSrc:%s", ToHex(dut.my_controller.PCSrc.value))
    #dut._log.info("RegWrite:%s", ToHex(dut.my_controller.RegWrite.value))
    #dut._log.info("MemWrite:%s", ToHex(dut.my_controller.MemWrite.value))
    #dut._log.info("ALUSrc:%s", ToHex(dut.my_controller.ALUSrc.value))
    #dut._log.info("MemtoReg:%s", ToHex(dut.my_controller.MemtoReg.value))
    #dut._log.info("ALUControl:%s", ToHex(dut.my_controller.ALUControl.value))
    #dut._log.info("FlagWrite:%s", ToHex(dut.my_controller.FlagW.value))
    #dut._log.info("ImmSrc:%s", ToHex(dut.my_controller.ImmSrc.value))
    #dut._log.info("RegSrc:%s", ToHex(dut.my_controller.RegSrc.value))
    #dut._log.info("ShiftControl:%s", ToHex(dut.my_controller.ShiftControl.value))
    #dut._log.info("shamt:%s", ToHex(dut.my_controller.shamt.value))
    #dut._log.info("CondEx:%s", ToHex(dut.my_controller.CondEx.value))
    #dut._log.info("CondExDelay:%s", ToHex(dut.my_controller.CondExDelay.value))
    #dut._log.info("state:%s", ToHex(dut.my_controller.state.value))
    #dut._log.info("Z:%s", ToHex(dut.my_controller.Z.value))
    