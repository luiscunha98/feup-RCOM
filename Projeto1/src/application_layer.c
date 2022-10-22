// Application layer protocol implementation

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//Custom header files
#include "application_layer.h"
#include "link_layer.h"
#include "frame.h"

typedef enum{
    SET, 
} frameType; 

void assembleFrame(char* frame, frameType frametype){
    switch (frametype)
    {
    case SET:
        frame[0] = FRAME_FLAG; 
        frame[1] = FRAME_A; 
        frame[2] = SETFRAME_C; 
        frame[3] = SETFRAME_BCC; 
        frame[4] = FRAME_FLAG;
        break;
    default:
        perror("Invalid frame type"); 
        break;
    }
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{

    if(strlen(serialPort) > 20){
        perror("Invalid serial Port"); 
        return; 
    }
    
    FILE* file; 
    file = fopen(filename, "r"); 

    int fd = open(serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPort);
        exit(-1);
    }

    LinkLayerRole llrole; 
    
    if(strcmp(role, "tx") == 0) //Transmiter
        llrole = LlTx; 
    else if (strcmp(role, "rx") == 0) //Receiver
        llrole = LlRx; 
    else{
        perror(role); 
        return; 
    }

    appLayer applicationl;
    applicationl.fileDescriptor = fd; 
    applicationl.status = llrole; 

    linkLayer llayer;
    if((sizeof(llayer.port)/sizeof(char)) >= (sizeof(serialPort)/sizeof(char)))
        strcpy(llayer.port, serialPort); 
    else 
        perror("Incorrect serialPort size"); 
    llayer.baudRate = baudRate;
    llayer.sequenceNumber = 0; 
    llayer.timeout = timeout; 
    llayer.numTransmissions = nTries; 
    llayer.role = llrole; 
    llayer.fd = applicationl.fileDescriptor;

    frameType frametype; 

    if(llrole == LlTx){ 
        frametype = SET; 
        assembleFrame(llayer.frame, frametype); 
        if(llopen(llayer) == 1) perror("LLOPEN: Something Went Wrong while trying to establish the connection"); 
    } 
    else if(llrole == LlRx){ 
        if(llopen(llayer) == 1) perror("LLOPEN: Something Went Wrong while trying to establish the connection");  
    }

}
