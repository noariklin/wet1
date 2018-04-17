/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#include <stdbool.h>
#include <assert.h>
#include <string.h>


SIM_coreState core_State;
void WBState();
int HDU();
int MemoryState();
void ControlStats(int);
void ExecuteState();
void DecodeState();
void FetchStage();
void UpdateVal(pipeStage);
void doNop(pipeStage);

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
int MemResult=0;
int PMemResult=0;
int nextMemResult = 0;
int CStall = 0;
int32_t PCnextTarget;
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
	doNop(FETCH);
	doNop(DECODE);
	doNop(EXECUTE);
	doNop(MEMORY);
	doNop(WRITEBACK);
	SIM_MemInstRead(0x0, &core_State.pipeStageState[FETCH].cmd);
	return 0;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
	ControlStats(MemResult);
	WBState();
	if (split_regfile) {
		core_State.regFile[WBResult.dstIdx] = WBResult.value;
	}
	PMemResult = MemResult;
    MemResult = MemoryState();
	if (MemResult == -1) { //lode

		UpdateVal(DECODE);

		FetchStage();
		core_State.pc -= 0x4;
		return;
	}
	else if (MemResult == 0) {
		ExecuteState();
		if (CStall == 0) {
			DecodeState();
			//FetchStage();
		}
		if (!split_regfile) {
			core_State.regFile[WBResult.dstIdx] = WBResult.value;
		}

	else if (MemResult == 1) {
		FetchStage();
            //core_State.pc += 0x4;
	}
	//MemResult = nextMemResult;

    }
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
	curState->pc = core_State.pc;
	for (int i = 0; i < SIM_REGFILE_SIZE; ++i) {
		curState->regFile[i] = core_State.regFile[i];
	}
	curState->pipeStageState[FETCH] = core_State.pipeStageState[FETCH];
	curState->pipeStageState[DECODE] = core_State.pipeStageState[DECODE];
	curState->pipeStageState[EXECUTE] = core_State.pipeStageState[EXECUTE];
	curState->pipeStageState[MEMORY] = core_State.pipeStageState[MEMORY];
	curState->pipeStageState[WRITEBACK] = core_State.pipeStageState[WRITEBACK];
}

void FetchStage() {
	if (PCSrc == 0) {
		core_State.pc += 0x4;
	}
	else {
	    if (PCSrc == 1) {
            core_State.pc = PCTarget;
        } else{
	       //PCnextTarget = PCTarget;
            if (PCSrc == 2) {
                core_State.pc += 0x4;
                --PCSrc;
            }
	    }
	}
	SIM_MemInstRead((uint32_t)core_State.pc, &core_State.pipeStageState[0].cmd);
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

void UpdateVal(pipeStage stage) {
	switch (core_State.pipeStageState[stage].cmd.opcode)
	{
	case CMD_ADD:
	case CMD_SUB:
	case CMD_ADDI:
	case CMD_SUBI:
	case CMD_LOAD:
	case CMD_BREQ:
	case CMD_BRNEQ:
	case CMD_STORE:
		core_State.pipeStageState[stage].src1Val = core_State.regFile[core_State.pipeStageState[stage].cmd.src1];
		if (core_State.pipeStageState[stage].cmd.isSrc2Imm) {
			core_State.pipeStageState[stage].src2Val = core_State.pipeStageState[stage].cmd.src2;
		}
		else {
			core_State.pipeStageState[stage].src2Val = core_State.regFile[core_State.pipeStageState[stage].cmd.src2];
		}
		break;
	case CMD_BR:
	case CMD_HALT:
	case CMD_NOP:
		core_State.pipeStageState[stage].src1Val = 0x0;
		core_State.pipeStageState[stage].src2Val = 0x0;
		break;
	default:
		break;
	}
}

void DecodeState() {
	SIM_cmd_opcode opc = core_State.pipeStageState[DECODE].cmd.opcode;
	DecodeSignalsState(opc);
	UpdateVal(DECODE);
		switch (opc) //update the buffer
		{
			case CMD_ADD:
			case CMD_SUB:
			case CMD_ADDI:
			case CMD_SUBI:
			case CMD_LOAD:
			case CMD_BREQ:
			case CMD_BRNEQ:
				DecodeToExe.val1 = core_State.regFile[core_State.pipeStageState[DECODE].cmd.src1];
				if (core_State.pipeStageState[DECODE].cmd.isSrc2Imm) {
					DecodeToExe.val2 = core_State.pipeStageState[DECODE].cmd.src2;
				}
				else {
					DecodeToExe.val2 = core_State.regFile[core_State.pipeStageState[DECODE].cmd.src2];
				}
				DecodeToExe.dstIdx = core_State.pipeStageState[DECODE].cmd.dst;
				break;
			case CMD_STORE:
				DecodeToExe.val1 = core_State.regFile[core_State.pipeStageState[DECODE].cmd.dst];
				if (core_State.pipeStageState[DECODE].cmd.isSrc2Imm) {
					DecodeToExe.val2 = core_State.pipeStageState[DECODE].cmd.src2;
				}
				else {
					DecodeToExe.val2 = core_State.regFile[core_State.pipeStageState[DECODE].cmd.src2];
				}
				DecodeToExe.dstIdx = core_State.pipeStageState[DECODE].cmd.src1;
				break;
			case CMD_BR:
				DecodeToExe.val1 = 0x0;
				DecodeToExe.val2 = 0x0;
				DecodeToExe.dstIdx = core_State.pipeStageState[DECODE].cmd.dst;
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
		DecodeToExe.pcp4 = core_State.pc + 0x4;
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
	if (PipelineSignals[MEMORY].MemRead == 1) { //LOAD

		loadResult = SIM_MemDataRead((uint32_t)addr, &(MemToWB.MEMOut));
		return loadResult;
	}
	else if (PipelineSignals[MEMORY].MemWrite == 1) { //STORE
		int32_t data = core_State.regFile[ExeToMem.dstIdx];
		SIM_MemDataWrite((uint32_t)addr, data);
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
	if (PipelineSignals[EXECUTE].RegWrite == 1) {
		int idx = core_State.pipeStageState[EXECUTE].cmd.dst;
		if ((idx == idx1) || (idx == idx2) || (idx == idx3)) return 1;
	}
	if (PipelineSignals[MEMORY].RegWrite == 1) {
		int idx = core_State.pipeStageState[MEMORY].cmd.dst;
		if ((idx == idx1) || (idx == idx2) || (idx == idx3)) return 1;
	}
	if ((PipelineSignals[WRITEBACK].RegWrite == 1) && (!split_regfile)) {
		int idx = core_State.pipeStageState[WRITEBACK].cmd.dst;
		if ((idx == idx1) || (idx == idx2) || (idx == idx3)) return 1;
	}
	if ((core_State.pipeStageState[DECODE].cmd.opcode == CMD_STORE) && (PipelineSignals[EXECUTE].RegWrite == 1) && 
																									(!split_regfile)) {
		if (core_State.pipeStageState[1].cmd.src1 == core_State.pipeStageState[2].cmd.dst) return 1;
	}
	return 0;
}


void doNop (pipeStage stage_to_nop){

	PipelineSignals[stage_to_nop].ALUControl = 0;
	PipelineSignals[stage_to_nop].Branch = 0;
	PipelineSignals[stage_to_nop].BrControl = 0;
	PipelineSignals[stage_to_nop].MemRead = 0;
	PipelineSignals[stage_to_nop].MemToReg = 0;
	PipelineSignals[stage_to_nop].MemWrite = 0;
	PipelineSignals[stage_to_nop].RegWrite = 0;

	core_State.pipeStageState[stage_to_nop].cmd.opcode = CMD_NOP;
	core_State.pipeStageState[stage_to_nop].cmd.dst = 0;
	core_State.pipeStageState[stage_to_nop].cmd.isSrc2Imm = false;
	core_State.pipeStageState[stage_to_nop].cmd.src1 = 0;
	core_State.pipeStageState[stage_to_nop].cmd.src2 = 0;
	core_State.pipeStageState[stage_to_nop].src1Val = 0x0;
	core_State.pipeStageState[stage_to_nop].src2Val = 0x0;

}



void ControlStats (int isMemRead) {

	switch (isMemRead) {
		case 0:
			CStall = HDU();
			PipelineSignals[WRITEBACK] = PipelineSignals[MEMORY];
			core_State.pipeStageState[WRITEBACK] = core_State.pipeStageState[MEMORY];
			PipelineSignals[MEMORY] = PipelineSignals[EXECUTE];
			core_State.pipeStageState[MEMORY] = core_State.pipeStageState[EXECUTE];
			if (CStall != 0) {
				doNop(EXECUTE);
			} else {
				PipelineSignals[EXECUTE] = PipelineSignals[DECODE];
				core_State.pipeStageState[EXECUTE] = core_State.pipeStageState[DECODE];
				core_State.pipeStageState[DECODE] = core_State.pipeStageState[FETCH];

			}

			break;
		case -1:
			doNop(WRITEBACK);
			break;
		case 1:
			PipelineSignals[WRITEBACK] = PipelineSignals[MEMORY];
			core_State.pipeStageState[WRITEBACK] = core_State.pipeStageState[MEMORY];
			doNop(MEMORY);
			doNop(EXECUTE);
			doNop(DECODE);
			break;
		default:
			break;
	}

}
