// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>


//-- Signal Library -> Responsible for the alarm part --//
#include <signal.h>

//-- Header file with useful frame macros --// 
#include "frame.h"


// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

volatile int STOP = FALSE;

// LAB2 MACROS
#define MAXATTEMPTS 3

//Alarm global variables
int alarmEnabled = FALSE; 
int alarmCount = 0; 

//Time out flag
int timeout = TRUE; 


// --- FUNCTIONS --- //


//Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    timeout = TRUE; 
}


int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

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

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    //UA Frame
    uint8_t uaFrame[FRAME_SIZE]; 
    uaFrame[0] = FRAME_FLAG; 
    uaFrame[4] = FRAME_FLAG; 

    //pos = 1, because pos = 0 and pos = 4 are always equal to the frame flag so we won't fill them in our state machine
    int pos = 1;  

    //Create SET Frame
    uint8_t setFrame[FRAME_SIZE]; 
    setFrame[0] = FRAME_FLAG; 
    setFrame[1] = FRAME_A; 
    setFrame[2] = SETFRAME_C; 
    setFrame[3] = SETFRAME_BBC; 
    setFrame[4] = FRAME_FLAG; 

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    //VERIFIES IF IT RECEIVES A RESPONSE OR A VALID FRAME BETWEEN A CERTAIN TIME 
    //OTHERWISE RESEND SET FRAME AFTER A CERTAIN TIMEOUT AND ATTEMPS MAX 3 TIMES
    
    //NEEDS ALARM FIX (FOR SOME REASON ALARM IS NOT FULLY WORKING
    //IT JUST SEEMS TO BE STUCK ON AN INFINITE LOOP)

    while(alarmCount < MAXATTEMPTS){

        if(alarmEnabled == FALSE && timeout == TRUE){
            alarm(3); // Set alarm to be triggered in 3s (time-out time)
            alarmEnabled = TRUE;  //Enables Alarm flag
            timeout = FALSE;  //Disables Timeout flag
        }

        //Sends SET Frame
        int bytes = write(fd, setFrame, FRAME_SIZE); 
        //printf("%d bytes written\n", bytes); 
        // Wait until all bytes have been written to the serial port
        sleep(1);

        /// --- STATE MACHINE PART --- ///

        //Auxiliary buffer to help process the byte read
        uint8_t aux_buf[1] = {0}; 

        //State Machine Variable -> Helps track the current state
        int currState = 0;   

        //Read a frame from the serial port  
        while (STOP == FALSE){    
            //IMPORTANT NOTE: The char must always be read inside the state, otherwise if it gets read outside the switch ("directly in the loop") the state machine will "break"    
                
            //STATE MACHINE IN ORDER TO PROCESS THE RECEIVER READING
            switch (currState){
                case STATE0: //read 1 char
                    read(fd, aux_buf, 1);
                    //Updates the current state 
                    if(*aux_buf == FRAME_FLAG){
                        currState = 1; 
                    }
                    //Continues on the same state
                    else{
                        currState = 0; 
                    }
                    break;
                case STATE1: //read 1 char
                    read(fd, aux_buf, 1);
                    //Updates the current state 
                    if(*aux_buf != FRAME_FLAG){
                        currState = 2; 
                        //Saves char
                        uaFrame[pos] = (*aux_buf); 
                        pos++; 
                        //
                    }
                    //Continues on the same state
                    else{
                        currState = 1; 
                    }
                    break;
                case STATE2: //read 1 char & save char
                    read(fd, aux_buf, 1);
                    if(*aux_buf != FRAME_FLAG && !timeout){
                        currState = 2; 
                        //Saves char
                        uaFrame[pos] = (*aux_buf); 
                        pos++; 
                    }
                    else{
                        currState = 3;
                    }
                    break;
                case STATE3: //read 1 char & save char
                    if(uaFrame[3] == (uaFrame[1]^uaFrame[2])){
                        //It's a valid frame !!!
                        currState = 4; 
                    }
                    else{
                        //It's not a valid frame !!!
                        currState = 0; 
                    }
                    break; 
                case STATE4: //evaluate C
                    if(uaFrame[2] == UAFRAME_C){ //It is a UA Frame!!! 
                        STOP = TRUE; 
                        //Deactivate alarm
                        alarm(0); 
                    }
                    else{
                        //It is not a SET Frame so we will reprocess everything again 
                        currState = 0; 
                    }
                    break;
                default:
                    printf("Something went wrong while processing SET Frame\n"); 
                    break;
            }
        }
    }

    if(STOP == FALSE){
        printf("Something went wrong while receiving a response from the receiver\n"); 
    }

    //Prints in the stdout(fd = 1) the frame read from the serial port
    if(STOP == TRUE){
        printf("UA FRAME CONTENT: "); 
        for(int i=0; i<pos+1; i++){
            printf("%x ", uaFrame[i]);
        }
        printf("\n"); 
        printf("Frame Communication Completed Succesfully \n"); 
    }
    
    sleep(1); 
 
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
