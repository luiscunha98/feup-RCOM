// Link layer protocol implementation

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include "frame.h"
#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

//LinkLayer Global Variable
linkLayer conParameters; 
char* txFrame = NULL; 
int txFrameSize = 0; 

/* Global variables needed for llopen() */
struct termios oldtio;

//STOP FLAG FOR DETERMINING WHETHER THE FRAME WAS CORRECTLY READ/RECEIVED
volatile int STOP = FALSE;

//Alarm global variables
int alarmEnabled = FALSE; 
int alarmCount = 0;

/////////////// UTILS FUNCTIONS ///////////////
sendTxFrame(){
    write(conParameters.fd, txFrame, txFrameSize); 
    sleep(1); // Wait until all bytes have been written to the serial port
}

/////////////// LLWRITE FUNCTIONS ///////////////

char* stuffing(char* unstuffedBuffer, int* size){
    int j=0; 
    int length = *size; 
    char* aux = (char*)malloc(*size); 
    for(int i=0; i<*size; i++, j++){
        if(j >= length){
            length = length+(length/2);
			aux = (unsigned char*) realloc(aux, length);

        }
        if(unstuffedBuffer[i] == FRAME_FLAG){
            aux[j] = ESC; 
            j++; 
            aux[j] = 0X5E; 
        }
        if(unstuffedBuffer[i] == ESC){
            aux[j] = ESC; 
            j++; 
            aux[j] = 0X5D; 
        }
        else
            aux[j] = unstuffedBuffer[i]; 
    }

    *size =  j;
    return aux; 
}

char* add_frame_header(unsigned char* stuffed_frame, int *size){
    unsigned char* ret = (char*)malloc(*size+5);
	ret[0] = FRAME_FLAG;
	ret[1] = FRAME_A;
	if(conParameters.sequenceNumber == 0)
        ret[2] = IFRAME0; 
    elseif(conParameters.sequenceNumber == 1)
        ret[2] = IFRAME1; 
    else{
        perror("Invalid Sequence Number"); 
        exit(-1);         
    }
	ret[3] = ret[1]^ret[2];
	for(int i=0; i<*size; i++){
		ret[i+4] = stuffed_frame[i];
	}
	ret[*size+4] = FRAME_FLAG;
	*size += 5; 

	return ret;
}


/////////////// LLREAD STUFF ///////////////
//byteDestuffing

/////////////// LLOPEN STUFF ///////////////

// -- MAYBE PUT THE ALARM FUNCTIONS ON A SEPARATE FILE LATER ON --- //
// --- ALARM FUNCTIONS (They will be common for each transmitter function) --- //

int checkForAlarmCount(){
    /* We close straigh up the connection, once it makes no sense to call llclose(), once the receiver might not be "on". 
    For that reason, in order to waste double the time to close the connection, we will keep the things simple and close the connection 
    right here (in this case only)*/
    if(alarmCount == conParameters.numTransmissions){
        printf("Something went wrong while receiving a response from the receiver. Ending the execution due to an error\n"); 
        sleep(1); 
        // Restore the old port settings
        if (tcsetattr(conParameters.fd, TCSANOW, &oldtio) == -1)
        {
            perror("tcsetattr");
            exit(-1);
        }
        close(conParameters.fd);
        exit(-1); 
    }
}

//Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE; //Disables Alarm Flag (Alarm not active at the momment)
    alarm(conParameters.timeout); // Set alarm to be triggered in time-out time
    alarmCount++; //Increases the number of attempts (number of times alarm was triggered)
    alarmEnabled = TRUE; //Enables Alarm Flag (Alarm currently active)
    sendTxFrame(); //Resends setFrame (void params, once the fd and setFrame are global variables to make the job easier)
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
    conParameters = connectionParameters; 

    struct termios newtio;

    // Save current port settings
    if (tcgetattr(conParameters.fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = conParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    tcflush(conParameters.fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(conParameters.fd, TCSANOW, &newtio) == -1)
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

    if(conParameters.role == LlTx){
        frame_C = UAFRAME_C; 

        //Sets the global variable that will be responsible for frame retransmission(TRANSMITTER) or Response(RECEIVER)
        txFrame = conParameters.frame;
        txFrameSize = MAX_PAYLOAD_SIZE; //Depois ver melhor (Assim funciona, mas penso que basta apenas ser = 5)

        sendTxFrame(); //Sends the SET Frame First time

        //Activates Alarm for the first time
        alarm(conParameters.timeout); 
        alarmEnabled = TRUE; 

        //-- Proccesses via State Machine if it recieves the correct UA Frame --//
        stateMachine(newFrame, conParameters.fd, frame_C, 0); 

        printf("Frame Communication Completed Succesfully \n"); 
        printf("LLOPEN: CONNECTION ESTABLISHED SUCCESSFULLY \n");
    }
    else if(conParameters.role == LlRx){
        frame_C = SETFRAME_C; 
        stateMachine(newFrame, conParameters.fd, frame_C, 1);
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
    if(bufSize < 0) return -1; 

    //Firstly Assembles Information Data and get the BCC2 without Stuffing
    char dataFrame[bufSize+1];
	char BCC2 = 0x00;
	for(int i=0; i<bufSize; i++){
		dataFrame[i] = buf[i];
		BCC2 ^=buf[i];
	}
	dataFrame[bufSize] = BCC2;
    bufSize++; 

    //Does Byte Stuffing on the information Frame Array 
    char* stuffedBuffer = stuffing(dataFrame, &bufSize); 

    //Assembles Information Frame (Add Headers to the data frame)
	char* infoFrame = add_frame_header(stuffedBuffer, &bufSize);

    txFrame = infoFrame; 
    txFrameSize = bufSize; 

    sendTxFrame(); //Sends Information Frame

    //Activates Alarm for the first time
    alarm(connectionParameters.timeout); 
    alarmEnabled = TRUE;

    /* STAND BY PQ PRIMEIRO TEM DE SE FAZER O READ*/

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    //Verifies Frame Header
    if(packet[1]^packet[2] == packet[3]) //Verify if BCC1 is valid

    //Does the DeStuffing

    //Verifies BCC2

    //Verifies if frame is not duplicat

    //If it is valid, then writes it to the file, by sending the data to the application layer

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int fd)
{
    // TODO

    //Received Frame
    char newFrame[MAX_PAYLOAD_SIZE]; 
    newFrame[0] = FRAME_FLAG; 
    newFrame[4] = FRAME_FLAG; 

    //DUVIDAS NESTA PARTE AINDA 
    //VER ISTO DEPOIS 
    //DE QUALQUER MANEIRA O CODIGO JA ESTA TODO FEITO (Depois é fazer a tal adaptação)

    return 1;
}
