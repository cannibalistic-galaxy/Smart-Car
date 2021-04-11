#ifndef _KEYSCAN_H
#define _KEYSCAN_H

#include "headfile.h"

#define sw1 A0
#define sw2 C7
#define sw3 C6
#define sw4 I3

extern uint8 sw1_status;
extern uint8 sw2_status;
extern uint8 sw3_status;
extern uint8 sw4_status;

extern int8 key_number;
extern uint8 key_value;
extern uint8 bomamode;


extern void boma_init(void);
extern void boma_mode(void);


#endif