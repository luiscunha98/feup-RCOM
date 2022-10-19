// Link layer protocol implementation

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>


#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(linkLayer connectionParameters)
{
    // TODO
    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    //UA Frame
    char uaFrame[FRAME_SIZE] = {0}; 
    uaFrame[0] = FRAME_FLAG; 
    uaFrame[1] = FRAME_A;
    uaFrame[2] = UAFRAME_C; 
    uaFrame[3] = UAFRAME_BBC; 
    uaFrame[4] = FRAME_FLAG;  

    //SET Frame
    char setFrame[FRAME_SIZE] = {0};
    setFrame[0] = FRAME_FLAG;
    setFrame[4] = FRAME_FLAG; 

    //Auxiliary buffer to help process the byte read
    char aux_buf[1] = {0}; 

    //pos = 1, because pos = 0 and pos = 4 are always equal to the frame flag so we won't fill them in our state machine
    int pos = 1;  

    //State Machine Variable -> Helps track the current state
    int currState = 0; 

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
