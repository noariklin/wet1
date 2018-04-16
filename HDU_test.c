#include "sim_api.h"
#include "test_utilities.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


bool split_regfile = false;

void DumpCoreState(SIM_coreState *state) {
    int i;
    SIM_cmd *curCmd;
    char const *curCmdStr;

    PipeStageState pipeStageState[SIM_PIPELINE_DEPTH];

    printf("PC = 0x%X\n", state->pc);
    printf("Register file:\n");
    for (i = 0; i < SIM_REGFILE_SIZE; ++i)
        printf("\tR%d = 0x%X", i, state->regFile[i]);
    printf("\nCommand at each pipe stage:\n");
    for (i = 0; i < SIM_PIPELINE_DEPTH; ++i)
    {
        curCmd = &state->pipeStageState[i].cmd;
        if (curCmd->opcode > CMD_MAX)
            curCmdStr = "<Invalid Cmd.>";
        else
            curCmdStr = cmdStr[curCmd->opcode];
        printf("\t%s : %s $%d , $%d(=0x%X) , %s%d(=0x%X)\n", pipeStageStr[i],
               curCmdStr, curCmd->dst, curCmd->src1,
               state->pipeStageState[i].src1Val,
               (curCmd->isSrc2Imm ? "" : "$"), curCmd->src2,
               state->pipeStageState[i].src2Val);
    }
}

static bool testClkTick() {
	char *h_result;


    SIM_CoreClkTick();

//    SIM_CoreClkTick("es1@gmail.com",6,"cm@gmail.com",6,3,5,100,4,(TechnionFaculty)6,&o_result);
//	ASSERT_TEST(h_result == ORDER_SUCCESS);
//	Order order2 = orderCreate(NULL,1,NULL,4,5,3,200,5,(TechnionFaculty)5,&o_result);
//	ASSERT_TEST(h_result == ORDER_NULL_ARGUMENT);


	return true;
}

int main(int argc, char const *argv[]) {

        int i, simDuration;
  //      forwarding = false // split_regfile = false;

    char const *memF = "example2.img" ;
        char const *memName = memF;
        int numS = 7;
//example1
        SIM_coreState curState;
        printf("%s \n" , memF);
        /* Initialized simulation modules */
        if (SIM_MemReset(memName) != 0) {
            fprintf(stderr, "Failed initializing memory simulator!\n");
            exit(2);
        }

        printf("Resetting core...\n");
        if (SIM_CoreReset() != 0) {
            fprintf(stderr, "Failed resetting core!\n");
            exit(3);
        }

        /* Running simulation */
        //simDuration = atoi(simDurationStr);
       /* if (simDuration <= 0) {
            fprintf(stderr, "Invalid simulation duration argument: %s\n",
                    simDurationStr);
            exit(4);
        }*/

        printf("Running simulation for %d cycles\n", numS);
        printf("Simulation on cycle %d. The state is:\n", 0);
        SIM_CoreGetState(&curState);
        DumpCoreState(&curState);
        for (i = 0; i < numS; i++) {
            SIM_CoreClkTick();
            SIM_MemClkTick();
            printf("\n\nSimulation on cycle %d. The state is:\n", i+1);
            SIM_CoreGetState(&curState);
            DumpCoreState(&curState);
        }


        return 0;
    }
