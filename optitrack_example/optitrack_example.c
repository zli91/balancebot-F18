/*******************************************************************************
* optitrack_example.c
*
* Use this template to save ground truth data from optitrack, or as an example
* for how to implement reading optitrack data over XBee
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <inttypes.h>
#include <sys/ioctl.h>

#include "../lcmtypes/balancebot_msg_t.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/balancebot_gate_t.h"

#include "../optitrack/common/serial.h"

FILE* f1;
FILE* ptr_file;
const int baudRate = 57600;
const char port[] = "/dev/ttyO5";
const char headByte = 0x1B;
const char tailByte = 0xFF;
int num_gates = 4;
int fd;
uint64_t last_utime = 0;
int bytes_avail = 0;
int err_counter = 0;

void getData(balancebot_msg_t *BBmsg);
void printData(balancebot_msg_t BBmsg);

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
    
    //open serial port non-blocking
    fd = serial_open(port,baudRate,0);

    if(fd == -1){
        printf("Failed to open Serial Port: %s", port);
        return -1;
    }

    //construct message for storage
    balancebot_msg_t BBmsg;
    pose_xyt_t BBpose;
    balancebot_gate_t BBgates[num_gates];
    BBmsg.pose = BBpose;
    BBmsg.num_gates = num_gates;
    BBmsg.gates = BBgates;
    int packetLength = balancebot_msg_t_encoded_size(&BBmsg)+2;
    printf("packetLength: %d\n", packetLength);
    
    // Print Header
    printf("\n");
    printf("   Time   |");
    printf("Buf|");
    printf("Err|");
    printf("  Rate  |");
    printf("    X    |");
    printf("    Y    |");
    printf("    θ    |");
    printf("#GATES|");
    printf("   G1X   |");
    printf("   G1Y   |");
    printf("\n");

    while(1)
    {
        //check bytes in serial buffer
        ioctl(fd, FIONREAD, &bytes_avail);
        //printf("bytes: %d\n",bytes_avail);
        if(bytes_avail >= packetLength){
            getData(&BBmsg);
            printData(BBmsg);
        }
        usleep(100);
    }
    serial_close(fd);
	return 0;
}

void getData(balancebot_msg_t* BBmsg){
    char *ptr;
    int packetLength = balancebot_msg_t_encoded_size(BBmsg)+2;
    char *dataPacket = (char*) malloc (packetLength);
    ptr = dataPacket;
    while(read(fd, ptr, 1) > 0){
        // if the first Byte is wrong keep looking
        if((ptr == dataPacket)&&(*ptr != headByte)){
            continue;
        }
        ptr++;
        // Once we have all of the Bytes check to make sure first and last are good
        if((ptr-dataPacket) == packetLength){
            if((dataPacket[0] != headByte) || (dataPacket[packetLength-1] != tailByte)){
                err_counter += 1;
            } 
            else{
                //packet is good, decode it into BBmsg
                int status = balancebot_msg_t_decode(dataPacket, 1, packetLength-2, BBmsg);
                if (status < 0) {
                    fprintf (stderr, "error %d decoding balancebot_msg_t!!!\n", status);;
                }
                // if we have less than a full message in the serial buffer
                // we are done, we'll get the next one next time
                ioctl(fd, FIONREAD, &bytes_avail);
                if(bytes_avail < packetLength){
                    break;
                }
            }
            //keep reading until buffer is almost empty
            ptr = dataPacket;
        }
    }
}

void printData(balancebot_msg_t BBmsg){
    if      (BBmsg.utime < 1000000)   printf("    %"PRId64"|",BBmsg.utime);
    else if (BBmsg.utime < 10000000)  printf("   %"PRId64"|",BBmsg.utime);
    else if (BBmsg.utime < 100000000) printf("  %"PRId64"|",BBmsg.utime);
    else if (BBmsg.utime < 1000000000)printf(" %"PRId64"|",BBmsg.utime);
    else                              printf("%"PRId64"|",BBmsg.utime);   

    if (bytes_avail < 10)       printf("  %d|",bytes_avail);
    else if (bytes_avail < 100) printf(" %d|",bytes_avail);
    else                        printf("%d|",bytes_avail);

    if (err_counter < 10)       printf("  %d|",err_counter);
    else if (err_counter < 100) printf(" %d|",err_counter);
    else                        printf("%d|",err_counter);

    printf("  %3.2f |", 1.0E6/(BBmsg.utime - last_utime));
    printf("%+7.6f|"  ,BBmsg.pose.x);
    printf("%+7.6f|"  ,BBmsg.pose.y);
    printf("%+7.6f|"  ,BBmsg.pose.theta);
    printf("  %d   |"    ,BBmsg.num_gates);
    if(BBmsg.num_gates > 0){
        printf("%+7.6f|"  ,(BBmsg.gates[0].left_post[0]+BBmsg.gates[0].right_post[0])/2.0);
        printf("%+7.6f|"  ,(BBmsg.gates[0].left_post[1]+BBmsg.gates[0].right_post[1])/2.0);
    }
    if(BBmsg.num_gates == 4){
        ptr_file = fopen("gates.txt","w");
        if(!ptr_file){

        }else{
            int i = 0;
            for(i=0;i<4;i++){
                fprintf(ptr_file,"%f\t%f\t%f\t%f\n",BBmsg.gates[i].left_post[0], BBmsg.gates[i].left_post[1], BBmsg.gates[i].right_post[0], BBmsg.gates[i].right_post[1]); 
            }
            fclose(ptr_file);
        }
    }
    printf("         \r");
    fflush(stdout);

    last_utime = BBmsg.utime;
}