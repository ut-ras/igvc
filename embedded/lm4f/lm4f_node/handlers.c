// Handlers for pub/sub
// Kevin Gilbert
// November 30 2013
#include <RASLib/inc/json_protocol.h>
#include <RASLib/inc/rasstring.h>       // SPrintf
#include "handlers.h"

/**
*	Message Buffers for subscribers/publishers
*/
char msgBuffSPLM[BUFFERSIZE];
char msgBuffSPRM[BUFFERSIZE];
char msgBuffSVLX[BUFFERSIZE];
char msgBuffSVAX[BUFFERSIZE];
char msgBuffRSTE[BUFFERSIZE];
char* msgBuffer[NUMSUB+NUMPUB] = {msgBuffSPLM, msgBuffSPRM, msgBuffSVLX, msgBuffSVAX, msgBuffRSTE};

/**
*	Subscriber Handlers
*   
*/								   
void SPLM_handler(void* data, char *jsonvalue) {
    SPrintf(msgBuffSPLM, "%s", jsonvalue);
}

void SPRM_handler(void* data, char *jsonvalue) {
    SPrintf(msgBuffSPRM, "%s", jsonvalue);
}

void SVLX_handler(void* data, char *jsonvalue) {
    SPrintf(msgBuffSVLX, "%s", jsonvalue);
}

void SVAX_handler(void* data, char *jsonvalue) {
    SPrintf(msgBuffSVAX, "%s", jsonvalue);
}

void RSTE_handler(void* data, char *jsonvalue) {
    SPrintf(msgBuffRSTE, "%s", jsonvalue);
}

void (*subHandlers[NUMSUB])(void*,char*) = {SPLM_handler, SPRM_handler,
									           SVLX_handler, SVAX_handler, 
									           RSTE_handler};

/**
* 	Publisher Handlers
*/

char* SPLMdebug_handler(void* data) {
    return msgBuffSPLM;
}

char* SPRMdebug_handler(void* data) {
    return msgBuffSPRM;
}

char* SVLXdebug_handler(void* data) {
    return msgBuffSVLX;
}

char* SVAXdebug_handler(void* data) {
    return msgBuffSVAX;
}

char* (*pubHandlers[NUMPUB])(void*) = {SPLMdebug_handler, SPRMdebug_handler,
									           SVLXdebug_handler, SVAXdebug_handler};
									           
