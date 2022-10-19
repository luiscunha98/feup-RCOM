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

    int flag = 0; 
    
    if(strstr(role, "tx") == 0) //Transmiter
        flag = 0; 
    else if (strstr(role, "rx") == 0) //Receiver
        flag = 1; 
    else{
        perror(role); 
        return; 
    }

    appLayer applicationl;
    applicationl.fileDescriptor = fd; 
    applicationl.status = flag; 

    linkLayer llayer;
    if((sizeof(llayer.port)/sizeof(char)) >= (sizeof(serialPort)/sizeof(char)))
        strcpy(llayer.port, serialPort); 
    else 
        perror("Incorrect serialPort size"); 
    llayer.baudRate = baudRate;
    llayer.sequenceNumber = 0; 
    llayer.timeout = timeout; 
    llayer.numTransmissions = nTries; 

    llopen(llayer); 

}
