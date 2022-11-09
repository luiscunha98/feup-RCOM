//FRAME MACROS IN ORDER TO AVOID TYPING IT MULTIPLE TIMES
#ifndef FRAME_H
#define FRAME_H

//Global Frame info
#define FRAME_FLAG 0x7E
#define FRAME_A 0x03
#define llClose_FRAME_A 0x01

//SET Frame info
#define SETFRAME_C 0x03
#define SETFRAME_BCC 0x00

//UA Frame info
#define UAFRAME_C 0x07
#define UAFRAME_BCC 0x04

//Control Frame Info
#define CTRL_START 0x02
#define CTRL_END 0x03

//Byte Stuffing 
#define ESC 0X7D
#define ESC_ESC 0x5D
#define FLAG_ESC 0X5E

//Information Frame
#define IFRAME0 0x00
#define IFRAME1 0x40

//Receiver Ready(RR) Frame
#define C_RR0 0x05
#define C_RR1 0x85

//Reject(REJ) Frame
#define C_REJ0 0x01
#define C_REJ1 0x81

//DISC Frame info
#define DISCFRAME_C 0x0B
#define DISCFRAME_BCC 0x0A

//UA llclose Frame info
#define UAcloseFRAME_C 0x07
#define UAcloseFRAME_BBC 0x06

// -- STATE MACROS JUST TO MAKE IT MORE READABLE -- //
#define STATE0 0
#define STATE1 1
#define STATE2 2
#define STATE3 3
#define STATE4 4

#endif