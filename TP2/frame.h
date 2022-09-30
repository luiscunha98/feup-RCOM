//FRAME MACROS IN ORDER TO AVOID TYPING IT MULTIPLE TIMES
#ifndef FRAME_H
#define FRAME_H

//Once frames are composed by various 1 byte "fields" we will use stdint library
#include <stdint.h>

//Global Frame info
#define FRAME_SIZE 5
#define FRAME_FLAG 0x7E
#define FRAME_A 0x03

//SET Frame info
#define SETFRAME_C 0x03
#define SETFRAME_BBC 0x00

//UA Frame info
#define UAFRAME_C 0x07
#define UAFRAME_BBC 0x04

// -- STATE MACROS JUST TO MAKE IT MORE READABLE -- //
#define STATE0 0
#define STATE1 1
#define STATE2 2
#define STATE3 3
#define STATE4 4

#endif