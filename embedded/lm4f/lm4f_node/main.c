#include <stdlib.h>                     // atoi
#include <inc/hw_types.h>		        // tBoolean
#include <RASLib/inc/init.h>            // InitializeMCU
#include <RASLib/inc/uart.h>            // InitializeUART, Printf
#include <RASLib/inc/rasstring.h>       // SPrintf
#include <RASLib/inc/encoder.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/json_protocol.h>
#include "handlers.h"

/**
*	Publisher array for IGVC 2014 LM4F uController.
*		Used in Main for initializations.
*
*	subArray:	SPLM - Set Power Left Motor=>int POWER
*				SPRM - Set Power Right Motor=>int POWER
*				SVLX - Set velocity of x axis(2x desired encoder ticks per message=>int VELOCITY
*		  		SVAZ - Set velocity of about Z (2x desired encoder ticks per message)=>int ANGVELOCITY
*			 	RSTE - Reset Encoder Values=>NONE
*
*	pubArray:	Publishes state information of uController to debug topic.
*/
tSub SPLM,SPRM,SVLX,SVAX,RSTE;
tPub SPLMdebug, SPRMdebug, SVLXdebug, SVAXdebug;
tSub* subArray[NUMSUB] = {&SPLM, &SPRM, &SVLX, &SVAX, &RSTE};
tPub* pubArray[NUMPUB] = {&SPLMdebug, &SPRMdebug, &SVLXdebug, &SVAXdebug};

/**
*	Array of jsonkey strings for initializations
*/
char* subKey[NUMSUB] = {"SPLM", "SPRM", "SVLX", "SVAX", "RSTE"};
char* pubKey[NUMPUB] = {"SPLMdebug", "SPRMdebug", "SVLXdebug", "SVAXdebug"};

int main(void) {
    int i;
    InitializeMCU();
    InitializeUART();

	// Initialize subscribers
	// subHandlers Array located in handlers.c
	for(i=0;i<NUMSUB;i++) {
	    InitializeSubscriber(subArray[i], subKey[i], 0, subHandlers[i]);
	}

	// Initialize publishers
	for(i=0;i<NUMPUB;i++) {
    	InitializePublisher(pubArray[i], pubKey[i], 0, pubHandlers[i]);
    }

    BeginPublishing(.1);
    BeginSubscribing(.1);
    
    while(1);
}
