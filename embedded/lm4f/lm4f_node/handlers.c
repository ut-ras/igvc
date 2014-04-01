// Handlers for pub/sub
// Kevin Gilbert
// November 30 2013
#include <RASLib/inc/json_protocol.h>
#include <RASLib/inc/rasstring.h>       // SPrintf
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/servo.h>
#include <RASLib/inc/motor.h>
#include <StellarisWare/inc/hw_types.h>
#include "handlers.h"

/**
*	Message Buffers for subscribers/publishers
*/
char msgBuffSPLM[BUFFERSIZE] = {'-'};
char msgBuffSPRM[BUFFERSIZE] = {'-'};
char msgBuffSVLX[BUFFERSIZE] = {'-'};
char msgBuffSVAX[BUFFERSIZE] = {'-'};
char msgBuffRSTE[BUFFERSIZE] = {'-'};
char* msgBuffer[NUMSUB+NUMPUB] = {msgBuffSPLM, msgBuffSPRM, msgBuffSVLX, msgBuffSVAX, msgBuffRSTE};

extern tBoolean kill, newCmd;
extern tMotor *left, *right;

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
    int val;
    float speed;
    SPrintf(msgBuffSVLX, "%s", jsonvalue);
    newCmd=true;
    val = (int) (jsonvalue[2]-0x30)*10;
    val += (int) (jsonvalue[3]-0x30);
    SetPin(PIN_F1,((val%2)==0));
    
    //SetServo(left, speed);
    //SetServo(right, speed);
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
									           
