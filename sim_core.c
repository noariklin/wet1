/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
SIM_coreState core_State;

typedef struct ControlSignals_ {
	int RegWrite;
	int ALUControl; //0 for +, 1 for -
	int Branch;
	int BrControl; //0 for BREQ, 1 for BRNEQ
	int MemRead;
	int MemWrite;
	int MemToReg;
}ControlSignals;

ControlSignals PipelineSignals[SIM_PIPELINE_DEPTH];

struct FetchToDecode_ {
	SIM_cmd cmd;
	int32_t PCP4;
}FetchToDecode;

struct DecodeToExe_ {
	int32_t val1;
	int32_t val2;
	int32_t pcp4;
	int dstIdx;
}DecodeToExe;

struct ExeToMem_ {
	int32_t ALUOut;
	bool Zero;
	int dstIdx;
}ExeToMem;

struct MemToWB_ {
	int32_t ALUOut;
	int32_t MEMOut;
	int dstIdx;
}MemToWB;

struct WBResult_ {
	int32_t value;
	int dstIdx;
}WBResult;

int PCSrc = 0; 
int32_t PCTarget; //PC+4+dst
int MemResult;
int Stall = 0;

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void) {
	core_State.pc = 0x0;
	for (int i = 0; i <= 31; i++) {
		core_State.regFile[i] = 0x0;
	}
	// ������ NOP ��� �����
	SIM_MemInstRead(0x0, &core_State.pipeStageState[0].cmd);
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
	WBState();
	MemResult = MemoryState();
	if (MemResult == -1) {
		core_State.regFile[WBResult.dstIdx] = WBResult.value;
		if (core_State.pipeStageState[1].cmd.src1 == WBResult.dstIdx) {
			core_State.pipeStageState[1].src1Val = WBResult.value;
		}
		else if(!core_State.pipeStageState[1].cmd.isSrc2Imm && (core_State.pipeStageState[1].cmd.src2 == WBResult.dstIdx)){
			core_State.pipeStageState[1].src2Val = WBResult.value;
		}
	}
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
}

void FetchStage() {
	FetchToDecode.cmd = core_State.pipeStageState[0].cmd;
	FetchToDecode.PCP4 = core_State.pc + 0x4;
	if (PCSrc == 0) {
		core_State.pc += 0x4;
	}
	else {
		core_State.pc = PCTarget;
	}
	SIM_MemInstRead(core_State.pc, &core_State.pipeStageState[0].cmd);
}

void DecodeSignalsState(SIM_cmd_opcode opc) {
	switch (opc)
	{
	case CMD_ADD:
	case CMD_ADDI:
		PipelineSignals[1].ALUControl = 0;
		PipelineSignals[1].RegWrite = 1;
		PipelineSignals[1].MemWrite = 0;
		PipelineSignals[1].MemRead = 0;
		PipelineSignals[1].MemToReg = 0;
		PipelineSignals[1].Branch = 0;
		PipelineSignals[1].BrControl = 0;
		break;
	case CMD_SUB:
	case CMD_SUBI:
		PipelineSignals[1].ALUControl = 1;
		PipelineSignals[1].RegWrite = 1;
		PipelineSignals[1].MemWrite = 0;
		PipelineSignals[1].MemRead = 0;
		PipelineSignals[1].MemToReg = 0;
		PipelineSignals[1].Branch = 0;
		PipelineSignals[1].BrControl = 0;
		break;
	case CMD_LOAD:
		PipelineSignals[1].ALUControl = 0;
		PipelineSignals[1].RegWrite = 1;
		PipelineSignals[1].MemWrite = 0;
		PipelineSignals[1].MemRead = 1;
		PipelineSignals[1].MemToReg = 1;
		PipelineSignals[1].Branch = 0;
		PipelineSignals[1].BrControl = 0;
		break;
	case CMD_STORE:
		PipelineSignals[1].ALUControl = 0;
		PipelineSignals[1].RegWrite = 0;
		PipelineSignals[1].MemWrite = 1;
		PipelineSignals[1].MemRead = 0;
		PipelineSignals[1].MemToReg = 0;
		PipelineSignals[1].Branch = 0;
		PipelineSignals[1].BrControl = 0;
		break;
	case CMD_BR:
		PipelineSignals[1].ALUControl = 0;
		PipelineSignals[1].RegWrite = 0;
		PipelineSignals[1].MemWrite = 0;
		PipelineSignals[1].MemRead = 0;
		PipelineSignals[1].MemToReg = 0;
		PipelineSignals[1].Branch = 1;
		PipelineSignals[1].BrControl = 0;
		break;
	case CMD_BREQ:
		PipelineSignals[1].ALUControl = 1;
		PipelineSignals[1].RegWrite = 0;
		PipelineSignals[1].MemWrite = 0;
		PipelineSignals[1].MemRead = 0;
		PipelineSignals[1].MemToReg = 0;
		PipelineSignals[1].Branch = 1;
		PipelineSignals[1].BrControl = 0;
		break;
	case CMD_BRNEQ:
		PipelineSignals[1].ALUControl = 1;
		PipelineSignals[1].RegWrite = 0;
		PipelineSignals[1].MemWrite = 0;
		PipelineSignals[1].MemRead = 0;
		PipelineSignals[1].MemToReg = 0;
		PipelineSignals[1].Branch = 1;
		PipelineSignals[1].BrControl = 1;
		break;
	default:
		break;
	}
}

void DecodeState() {
	SIM_cmd_opcode opc = FetchToDecode.cmd.opcode;
	DecodeSignalsState(opc);
	core_State.pipeStageState[1].cmd = FetchToDecode.cmd;
	switch (opc)
	{
	case CMD_ADD:
	case CMD_SUB:
	case CMD_ADDI:
	case CMD_SUBI:
	case CMD_LOAD:
	case CMD_BREQ:
	case CMD_BRNEQ:
	case CMD_STORE:
		core_State.pipeStageState[1].src1Val = core_State.regFile[core_State.pipeStageState[1].cmd.src1];
		if (core_State.pipeStageState[1].cmd.isSrc2Imm) {
			core_State.pipeStageState[1].src2Val = core_State.pipeStageState[1].cmd.src2;
		}
		else {
			core_State.pipeStageState[1].src2Val = core_State.regFile[core_State.pipeStageState[1].cmd.src2];
		}
		break;
	case CMD_BR:
	case CMD_HALT:
	case CMD_NOP:
		core_State.pipeStageState[1].src1Val = 0x0;
		core_State.pipeStageState[1].src2Val = 0x0;
		break;
	default:
		break;
	}
	Stall = HDU();
	if (Stall == 0) {
		switch (opc) //update the buffer
		{
		case CMD_ADD:
		case CMD_SUB:
		case CMD_ADDI:
		case CMD_SUBI:
		case CMD_LOAD:
		case CMD_BREQ:
		case CMD_BRNEQ:
			DecodeToExe.val1 = core_State.regFile[FetchToDecode.cmd.src1];
			if (FetchToDecode.cmd.isSrc2Imm) {
				DecodeToExe.val2 = FetchToDecode.cmd.src2;
			}
			else {
				DecodeToExe.val2 = core_State.regFile[FetchToDecode.cmd.src2];
			}
			DecodeToExe.dstIdx = FetchToDecode.cmd.dst;
			break;
		case CMD_STORE:
			DecodeToExe.val1 = core_State.regFile[FetchToDecode.cmd.dst];
			if (FetchToDecode.cmd.isSrc2Imm) {
				DecodeToExe.val2 = FetchToDecode.cmd.src2;
			}
			else {
				DecodeToExe.val2 = core_State.regFile[FetchToDecode.cmd.src2];
			}
			DecodeToExe.dstIdx = FetchToDecode.cmd.src1;
			break;
		case CMD_BR:
			DecodeToExe.val1 = 0x0;
			DecodeToExe.val2 = 0x0;
			DecodeToExe.dstIdx = FetchToDecode.cmd.dst;
			break;
		case CMD_HALT:
		case CMD_NOP:
			DecodeToExe.val1 = 0x0;
			DecodeToExe.val2 = 0x0;
			DecodeToExe.dstIdx = 0;
			break;
		default:
			break;
		} 
		DecodeToExe.pcp4 = FetchToDecode.PCP4;
	}
	else {
		DecodeToExe.val1 = 0;
		DecodeToExe.val2 = 0;
		DecodeToExe.pcp4 = 0;
		DecodeToExe.dstIdx = 0;
	}
}

void ExecuteState() {
	ExeToMem.dstIdx = DecodeToExe.dstIdx;
	int32_t dstVal = core_State.regFile[DecodeToExe.dstIdx];
	int32_t val1 = DecodeToExe.val1;
	int32_t val2 = DecodeToExe.val2;
	dstVal = 0x4 * dstVal;
	PCTarget = DecodeToExe.pcp4 + dstVal;
	if (PipelineSignals[2].ALUControl == 0) {
		ExeToMem.ALUOut = val1 + val2;
	}
	else {
		ExeToMem.ALUOut = (val1 - val2);
		if (ExeToMem.ALUOut == 0) ExeToMem.Zero = true;
		else ExeToMem.Zero = false;
	}
}

int MemoryState() {
	if (PipelineSignals[3].Branch == 1) {
		if ((PipelineSignals[3].BrControl == 0 && ExeToMem.Zero) || (PipelineSignals[3].BrControl == 1 && !ExeToMem.Zero)) {
			PCSrc = 1;
			return 1;
		}
		else PCSrc = 0;
		return 0;
	}
	int loadResult;
	int32_t addr = ExeToMem.ALUOut * 0x4;
	MemToWB.dstIdx = ExeToMem.dstIdx;
	MemToWB.ALUOut = ExeToMem.ALUOut;
	if (PipelineSignals[3].MemRead == 1) { //LOAD
		loadResult = SIM_MemDataRead(addr, MemToWB.MEMOut);
		return loadResult;
	}
	else if (PipelineSignals[3].MemWrite == 1) { //STORE
		int32_t data = core_State.regFile[ExeToMem.dstIdx];
		SIM_MemDataWrite(addr, data);
		return 0;
	}
	return 0;
}

void WBState() {
	if (PipelineSignals[4].RegWrite == 1) {
		WBResult.dstIdx = MemToWB.dstIdx;
		if (PipelineSignals[4].MemToReg == 0) {
			WBResult.value = MemToWB.ALUOut;
		}
		else {
			WBResult.value = MemToWB.MEMOut;
		}
	}
	else {
		WBResult.dstIdx = 0;
		WBResult.value = 0x0;
	}
}

int HDU() {
	int idx1 = -1;
	int idx2 = -1;
	int idx3 = -1;
	switch (core_State.pipeStageState[1].cmd.opcode)
	{
	case CMD_ADD:
	case CMD_SUB:
		idx1 = core_State.pipeStageState[1].cmd.src1;
		idx2 = core_State.pipeStageState[1].cmd.src2;
		break;
	case CMD_ADDI:
	case CMD_SUBI:
		idx1 = core_State.pipeStageState[1].cmd.src1;
		break;
	case CMD_LOAD:
		idx1 = core_State.pipeStageState[1].cmd.src1;
		if (!core_State.pipeStageState[1].cmd.isSrc2Imm) {
			idx2 = core_State.pipeStageState[1].cmd.src2;
		}
		break;
	case CMD_STORE:
		idx3 = core_State.pipeStageState[1].cmd.dst;
		if (!core_State.pipeStageState[1].cmd.isSrc2Imm) {
			idx2 = core_State.pipeStageState[1].cmd.src2;
		}
		break;
	case CMD_BR:
		idx3 = core_State.pipeStageState[1].cmd.dst;
		break;
	case CMD_BREQ:
	case CMD_BRNEQ:
		idx1 = core_State.pipeStageState[1].cmd.src1;
		if (!core_State.pipeStageState[1].cmd.isSrc2Imm) {
			idx2 = core_State.pipeStageState[1].cmd.src2;
		}
		idx3 = core_State.pipeStageState[1].cmd.dst;
		break;
	case CMD_HALT:
	case CMD_NOP:
		return 0;
	default:
		break;
	}
	if (PipelineSignals[2].RegWrite == 1) {
		int idx = core_State.pipeStageState[2].cmd.dst;
		if ((idx == idx1) || (idx == idx2) || (idx == idx3)) return 3;
	}
	if (PipelineSignals[3].RegWrite == 1) {
		int idx = core_State.pipeStageState[3].cmd.dst;
		if ((idx == idx1) || (idx == idx2) || (idx == idx3)) return 2;
	}
	if (PipelineSignals[4].RegWrite == 1) {
		int idx = core_State.pipeStageState[4].cmd.dst;
		if ((idx == idx1) || (idx == idx2) || (idx == idx3)) return 1;
	}
	if ((core_State.pipeStageState[1].cmd.opcode == CMD_STORE) || (PipelineSignals[2].RegWrite == 1)) {
		if (core_State.pipeStageState[1].cmd.src1 == core_State.pipeStageState[2].cmd.dst) return 1;
	}
	return 0;
}