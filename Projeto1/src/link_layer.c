// Link layer protocol implementation

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>


#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

/////////////// LLOPEN STUFF ///////////////

#include "frame.h"

//STOP FLAG FOR DETERMINING WHETHER THE FRAME WAS CORRECTLY READ/RECEIVED
volatile int STOP = FALSE;

//Alarm global variables
int alarmEnabled = FALSE; 
int alarmCount = 0;

//Global variable defining alarm max attempts
int maxattempts = 0; 

/* Global variables needed for llopen() */

struct termios oldtio;
int fd = 0; 
char* llopenFrame; 

/// LLOPEN EXTRA FUNCTIONS /// 
sendLlopenFrame(){
    write(fd, llopenFrame, MAX_PAYLOAD_SIZE); 
    sleep(1); // Wait until all bytes have been written to the serial port
}

// -- MAYBE PUT THE ALARM FUNCTIONS ON A SEPARATE FILE LATER ON --- //
// --- ALARM FUNCTIONS (They will be common for each transmitter function) --- //

int checkForAlarmCount(){
    /* We close straigh up the connection, once it makes no sense to call llclose(), once the receiver might not be "on". 
    For that reason, in order to waste double the time to close the connection, we will keep the things simple and close the connection 
    right here (in this case only)*/
    if(alarmCount == maxattempts){
        printf("Something went wrong while receiving a response from the receiver. Ending the execution due to an error\n"); 
        sleep(1); 
        // Restore the old port settings
        if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
        {
            perror("tcsetattr");
            exit(-1);
        }
        close(fd);
        exit(-1); 
    }
}

//Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE; //Disables Alarm Flag (Alarm not active at the momment)
    alarm(3); // Set alarm to be triggered in 3s (time-out time)
    alarmCount++; //Increases the number of attempts (number of times alarm was triggered)
    alarmEnabled = TRUE; //Enables Alarm Flag (Alarm currently active)
    sendLlopenFrame(); //Resends setFrame (void params, once the fd and setFrame are global variables to make the job easier)
    checkForAlarmCount(); //This function is necessary to "bypass" the buggy go to behaviour of the alarmHandler function
}

// --- STATE MACHINE FUNCTION --- //
void stateMachine(char* newFrame, int fd, char frame_C, int isReceiver){

    /*State Machine Aux Variables*/
    int currState = 0; //Integer value of the Start State 
    int pos = 1; //pos = 1, because pos = 0 and pos = 4 are always equal to the frame flag so we won't fill them in our state machine
    char aux_buf[1] = {0}; //Auxiliary buffer to help the processing of the byte read

    //Read a frame from the serial port -> Cycle Stops if a valid frame has been read (STOP = TRUE) or when the max number of attemps have been reached
    while (STOP == FALSE){    
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
                    newFrame[pos] = (*aux_buf); 
                    pos++; 
                }
                //Continues on the same state
                else{
                    currState = 1; 
                }
                break;
            case STATE2: //read 1 char & save char
                read(fd, aux_buf, 1);
                if(*aux_buf != FRAME_FLAG){
                    currState = 2; 
                    //Saves char
                    newFrame[pos] = (*aux_buf); 
                    pos++; 
                }
                else{
                    currState = 3;
                }
                break;
            case STATE3: //read 1 char & save char
                if(newFrame[3] == (newFrame[1]^newFrame[2])){
                    //It's a valid frame !!!
                    currState = 4; 
                }
                else{
                    //It's not a valid frame !!!
                    currState = 0; 
                }
                break; 
            case STATE4: //evaluate C
                if(newFrame[2] == frame_C){ //It is a (UA/SET) Frame!!! 
                    if(isReceiver){ 
                        char uaFrame[MAX_PAYLOAD_SIZE]; 
                        uaFrame[0] = FRAME_FLAG; 
                        uaFrame[1] = FRAME_A; 
                        uaFrame[2] = UAFRAME_C; 
                        uaFrame[3] = UAFRAME_BCC;  
                        uaFrame[4] = FRAME_FLAG;
                        write(fd, uaFrame, MAX_PAYLOAD_SIZE); 
                        sleep(1); // Wait until all bytes have been written to the serial port
                        STOP = TRUE; 
                    }
                    else{
                        STOP = TRUE;
                        alarm(0); //Deactivate alarm
                    }
                }
                else{
                    //It is not a (UA/SET) Frame so we will reprocess everything again 
                    currState = 0; 
                }
                break;
            default:
                printf("Something went wrong while processing the received Frame\n"); 
                break;
        }
    }
} 

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(linkLayer connectionParameters)
{   
    fd = connectionParameters.fd;

    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
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

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    //Creates a New Frame (The one that will be received)
    char newFrame[MAX_PAYLOAD_SIZE]; 
    newFrame[0] = FRAME_FLAG; 
    newFrame[4] = FRAME_FLAG; 
    

    char frame_C; //Variables to store C byte will differ on State Machine depending if it is Transmitter or Receiver

    if(connectionParameters.role == LlTx){
        frame_C = UAFRAME_C; 

        maxattempts = connectionParameters.numTransmissions; //Sets the alarm Max Attempts

        //Sets the global variable that will be responsible for frame retransmission(TRANSMITTER) or Response(RECEIVER)
        llopenFrame = connectionParameters.frame; 

        sendLlopenFrame(); //Sends the SET Frame First time

        //Activates Alarm for the first time
        alarm(3); 
        alarmEnabled = TRUE; 

        //-- Proccesses via State Machine if it recieves the correct UA Frame --//
        stateMachine(newFrame, fd, frame_C, 0); 

        printf("Frame Communication Completed Succesfully \n"); 
        printf("LLOPEN: CONNECTION ESTABLISHED SUCCESSFULLY \n");
    }
    else if(connectionParameters.role == LlRx){
        frame_C = SETFRAME_C; 

        stateMachine(newFrame, fd, frame_C, 1);
    }
    else{
        perror("Invalid role assigned \n"); 
        return 1; 
    }

    return 0; 
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
