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
void nextStage();
int Forwarding_HDU();

pipeStage stageFW;

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

struct FetchToDecode_{
    SIM_cmd cmd;
    int32_t pc4p;
}FetchToDecode;


struct DecodeToExe_ {
	int32_t val1;
	int32_t val2;
	int32_t val3;
	int32_t pcp4;
	int dstIdx;
}DecodeToExe, nextDecodeToExe;

struct ExeToMem_ {
	int32_t ALUOut;
	int32_t val3;
	bool Zero;
	int dstIdx;
}ExeToMem, nextExeToMem;

struct MemToWB_ {
	int32_t ALUOut;
	int32_t MEMOut;
	int dstIdx;
}MemToWB, nextMemToWB;

struct WBResult_ {
	int32_t value;
	int dstIdx;
}WBResult;

int PCSrc = 0;
int nextPCSrc = 0;
int32_t PCTarget , nextPCTarget; //PC+4+dst
int MemResult=0;
int nextMemResult = 0;
int CStall = 0;
int FStall =0;
int TMP = -1;

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
    if (!split_regfile) {
        core_State.regFile[WBResult.dstIdx] = WBResult.value;
    }
	ControlStats(MemResult);
	if(MemResult == 0){
	    nextStage();
        FetchStage();
	}
	if(MemResult == 1){
	    PCSrc = 1;

	    FetchStage();
	}
    WBState();
    if (split_regfile ) {
        core_State.regFile[WBResult.dstIdx] = WBResult.value;
    }
    nextMemResult = MemoryState();
    ExecuteState();
    DecodeState();
    if (FStall == 1)doNop(EXECUTE);
	MemResult = nextMemResult;

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
	else if(PCSrc == 1){
            core_State.pc = PCTarget;
	}
	else{
	    core_State.pc = core_State.pc;
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
	SIM_cmd_opcode opc = FetchToDecode.cmd.opcode;
	DecodeSignalsState(opc);
	UpdateVal(DECODE);
	int32_t val1 = 0;
	int32_t val2 = 0;
	int32_t val3 = 0;
	int dstIdx = 0;
		switch (opc) //update the buffer
		{
			case CMD_ADD:
			case CMD_SUB:
			case CMD_ADDI:
			case CMD_SUBI:
			case CMD_LOAD:
			case CMD_BREQ:
			case CMD_BRNEQ:
				if ( (forwarding && core_State.pipeStageState[MEMORY].cmd.dst && PipelineSignals[MEMORY].MemToReg == 1) &&
							(core_State.pipeStageState[MEMORY].cmd.dst == core_State.pipeStageState[EXECUTE].cmd.src1 ||
							 core_State.pipeStageState[MEMORY].cmd.dst == core_State.pipeStageState[EXECUTE].cmd.src2) && nextMemResult != -1){
						FStall = 1;
						//doNop(EXECUTE);
					break;
					//}
				} else {
					val1 = core_State.regFile[FetchToDecode.cmd.src1];
					if (FetchToDecode.cmd.isSrc2Imm) {
						val2 = FetchToDecode.cmd.src2;
					} else {
						val2 = core_State.regFile[FetchToDecode.cmd.src2];
					}
					dstIdx = FetchToDecode.cmd.dst;
					val3 = core_State.regFile[FetchToDecode.cmd.dst];
				}
				break;
            case CMD_STORE:
                val1 = core_State.regFile[FetchToDecode.cmd.dst];
                if (FetchToDecode.cmd.isSrc2Imm) {
                    val2 = FetchToDecode.cmd.src2;
                } else {
                    val2 = core_State.regFile[FetchToDecode.cmd.src2];
                }
                dstIdx = FetchToDecode.cmd.src1;
                val3 = core_State.regFile[dstIdx];
                break;
			case CMD_BR:
				val1 = 0x0;
				val2 = 0x0;
				dstIdx = FetchToDecode.cmd.dst;
				val3 = core_State.regFile[dstIdx];
				break;
			case CMD_HALT:
			case CMD_NOP:
				val1 = 0x0;
				val2 = 0x0;
				val3 = 0x0;
				dstIdx = 0;
				break;
			default:
				break;
		}

		nextDecodeToExe.val1 = val1;
		nextDecodeToExe.val2 = val2;
		nextDecodeToExe.val3 = val3;
		nextDecodeToExe.dstIdx = dstIdx;
		nextDecodeToExe.pcp4 = FetchToDecode.pc4p;

}

void ExecuteState() {
			int32_t val1 =0x0;
			int32_t val2 = 0x0;
			int32_t ALUOut;
	bool Zero = true;
	int FwResult = Forwarding_HDU();
	if (FwResult && forwarding){
		switch (FwResult) {
			case 1: val1 = core_State.pipeStageState[stageFW].src1Val;
				break;
			case 2: val2 = core_State.pipeStageState[stageFW].src2Val;
				break;
			case 3: val2 = core_State.pipeStageState[stageFW].src1Val;
				val1 = core_State.pipeStageState[stageFW].src2Val;
				break;

			case 4:val2 = core_State.pipeStageState[stageFW].src2Val;
				val1 = core_State.pipeStageState[stageFW].src1Val;
				break;
				default: break;
		}
	}
	else {
		 val1 = DecodeToExe.val1;
		 val2 = DecodeToExe.val2;
	}
	if (PipelineSignals[2].ALUControl == 0) {
		ALUOut = val1 + val2;
	} else {
		ALUOut = (val1 - val2);
		if (ALUOut == 0) Zero = true;
		else Zero = false;
	}
		int32_t dstVal = DecodeToExe.val3;
		nextPCTarget = DecodeToExe.pcp4 + dstVal;
		nextExeToMem.dstIdx = DecodeToExe.dstIdx;
		nextExeToMem.ALUOut = ALUOut;
		nextExeToMem.Zero = Zero;
		nextExeToMem.val3 = DecodeToExe.val3;
}

int MemoryState() { // !!!!!!!!!!!!!!!!!!!!!!! ---!!!!!!!!!!!!!!!!!!!!!!! ---!!!!!!!!!!!!!!!!!!!!!!! ---!!!!!!!!!!!!!!!!!!!!!!! ---!!!!!!!!!!!!!!!!!!!!!!! ---!!!!!!!!!!!!!!!!!!!!!!! ---
	nextPCSrc = 0;
	++TMP;

	if (PipelineSignals[3].Branch == 1) {
		if (((PipelineSignals[3].BrControl == 0) && ExeToMem.Zero) || ((PipelineSignals[3].BrControl == 1) && !ExeToMem.Zero)) {
			nextPCSrc = 1;
			return 1;
		}
		else return 0;
	}
	int loadResult;
	int32_t addr = ExeToMem.ALUOut;
	nextMemToWB.dstIdx = ExeToMem.dstIdx;
	nextMemToWB.ALUOut = ExeToMem.ALUOut;
	if (PipelineSignals[MEMORY].MemRead == 1) { //LOAD
		loadResult = SIM_MemDataRead((uint32_t)addr, &(nextMemToWB.MEMOut));
		if(loadResult == -1) {
			nextPCSrc = -1;
		}else if((PipelineSignals[DECODE].MemToReg == 1) && core_State.pipeStageState[DECODE].cmd.opcode == CMD_LOAD && forwarding && core_State.pipeStageState[DECODE].cmd.dst && PipelineSignals[DECODE].RegWrite == 1) {
			if (core_State.pipeStageState[DECODE].cmd.dst == core_State.pipeStageState[FETCH].cmd.src2 ||
				core_State.pipeStageState[DECODE].cmd.dst == core_State.pipeStageState[FETCH].cmd.src1) {
				FStall = 1;
				nextPCSrc = -1;
				//doNop(EXECUTE);
				return 7;
			}

		}
		return loadResult;
	}
	else if (PipelineSignals[MEMORY].MemWrite == 1) { //STORE
		int32_t data = ExeToMem.val3;
		SIM_MemDataWrite((uint32_t)addr, data);
		return 0;
	}
	if ((PipelineSignals[DECODE].MemToReg == 1) && core_State.pipeStageState[DECODE].cmd.opcode == CMD_LOAD && forwarding && core_State.pipeStageState[DECODE].cmd.dst && PipelineSignals[DECODE].RegWrite == 1){
		if (core_State.pipeStageState[DECODE].cmd.dst == core_State.pipeStageState[FETCH].cmd.src2 || core_State.pipeStageState[DECODE].cmd.dst == core_State.pipeStageState[FETCH].cmd.src1){
			FStall = 1;
			//nextPCSrc = -1;
			//doNop(EXECUTE);
			return 7;
		}
	}
	return 0;
}

void WBState() {
	if (PipelineSignals[WRITEBACK].RegWrite == 1) {
		WBResult.dstIdx = MemToWB.dstIdx;
		if (PipelineSignals[WRITEBACK].MemToReg == 0) {
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
	int opcID = core_State.pipeStageState[1].cmd.opcode;

	switch (opcID)
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
            idx1 = core_State.pipeStageState[1].cmd.src1;
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
	if (forwarding ){
	/*	if (PipelineSignals[WRITEBACK].RegWrite == 1) {
			int idx = core_State.pipeStageState[WRITEBACK].cmd.dst;
			if ((idx == idx1) || (idx == idx2) || (idx == idx3)) return 1;
		} else {*/
			return 0;

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

	return 0;
}

int Forwarding_HDU() {
	int idx1 = -1;
	int idx2 = -1;
	int idx3 = -1;
	switch (core_State.pipeStageState[1].cmd.opcode) {
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
			idx1 = core_State.pipeStageState[1].cmd.src1;
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
	if ((PipelineSignals[WRITEBACK].RegWrite == 1) && (PipelineSignals[MEMORY].RegWrite == 1) && (forwarding) ) {
		int idxW = core_State.pipeStageState[WRITEBACK].cmd.dst;
		int idxM = core_State.pipeStageState[MEMORY].cmd.dst;
		stageFW = MEMORY;
		if ((idxW == idx1) && (idxM == idx1)) return 1;
		if ((idxW == idx2) && (idxM == idx2)) return 2;
		if ((idxW == idx2) && (idxM == idx1)) return 3;
		if ((idxW == idx1) && (idxM == idx2)) return 4;

	}else if ((PipelineSignals[WRITEBACK].RegWrite == 1) && (forwarding)) {
		int idx = core_State.pipeStageState[WRITEBACK].cmd.dst;
		stageFW = WRITEBACK;
		if (idx == idx1) return 1;
		if (idx == idx2) return 2;
		if (idx == idx3) return 3;
//		if ((idx == idx1) || (idx == idx2) || (idx == idx3)) return 1;
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

void nextStage(){
    if(CStall == 0){
        FetchToDecode.pc4p = core_State.pc + 0x4;
        FetchToDecode.cmd = core_State.pipeStageState[FETCH].cmd;
        DecodeToExe.val1 = nextDecodeToExe.val1;
        DecodeToExe.val2 = nextDecodeToExe.val2;
        DecodeToExe.val3 = nextDecodeToExe.val3;
        DecodeToExe.dstIdx = nextDecodeToExe.dstIdx;
        DecodeToExe.pcp4 = nextDecodeToExe.pcp4;
        PCSrc = nextPCSrc;
    }
    else PCSrc = -1;

    PCTarget = nextPCTarget;

    ExeToMem.dstIdx = nextExeToMem.dstIdx;
    ExeToMem.val3 = nextExeToMem.val3;
    ExeToMem.Zero = nextExeToMem.Zero;
    ExeToMem.ALUOut = nextExeToMem.ALUOut;

    MemToWB.ALUOut = nextMemToWB.ALUOut;
    MemToWB.dstIdx = nextMemToWB.dstIdx;
    MemToWB.MEMOut = nextMemToWB.MEMOut;
}

void ControlStats (int isMemRead) {
	//if (forwarding) return;
	switch (isMemRead) {
		case 0:
			CStall = HDU();
		//	FStall = Forwarding_HDU();
			PipelineSignals[WRITEBACK] = PipelineSignals[MEMORY];
			core_State.pipeStageState[WRITEBACK] = core_State.pipeStageState[MEMORY];
			PipelineSignals[MEMORY] = PipelineSignals[EXECUTE];
			core_State.pipeStageState[MEMORY] = core_State.pipeStageState[EXECUTE];
			if (CStall != 0 && forwarding == 0) { //} || FStall == 1) {
			//	FStall=0;
				doNop(EXECUTE);
			} else {
				if(FStall) {
					FStall = 0;
					break;
				}
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
