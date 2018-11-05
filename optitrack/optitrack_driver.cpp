#include <optitrack/optitrack.hpp>
#include <optitrack/common/getopt.h>
#include <optitrack/common/timestamp.h>
#include <optitrack/common/serial.h>

#include <lcmtypes/balancebot_msg_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/balancebot_gate_t.hpp>

#include <errno.h>  //Errors for read/write
#include <unistd.h> // read / write / sleep
#include <stdio.h>

#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>
#include <iostream>
#include <cmath>
#include <sys/time.h>
#include <sys/select.h>
#include <termios.h>

// Below for PRId64
#include <cinttypes>
#include <inttypes.h>

int main(int argc, char** argv)
{
    const char* kInterfaceArg = "interface";
    const char* kSerialBaudArg = "baudrate";
    const char* kXbeeAddrArg = "xbeeAddr";
    const char* kSerialPort = "serialPort";
    const char* kBBRigidBodyArg = "balancebot";
    const char* kNumGatesArg = "number of gates";
    const char* kTestingFakeData  = "fakeData";
    
    getopt_t* gopt = getopt_create();
    getopt_add_bool(gopt,   'h', "help", 0, "Display this help message.\n");
    getopt_add_string(gopt, 'i', kInterfaceArg, "127.0.0.1", "Local network interface used when connecting to the Optitrack network.");
    getopt_add_int(gopt,    'b', kSerialBaudArg, "57600", "Serial baudrate for communication via XBee");
    getopt_add_string(gopt, 'd', kSerialPort, "/dev/ttyUSB1", "Serial port used to send the XBee packets out"); 
    getopt_add_int(gopt,    'x', kXbeeAddrArg, "1", "Address of target XBee");
    getopt_add_int(gopt,    'r', kBBRigidBodyArg, "5", "Id of Balancebot rigid body to publish pose for.");
    getopt_add_int(gopt,    'n', kNumGatesArg, "4", "Number of Gates (IDs 1-N)");
    getopt_add_int(gopt,   'T', kTestingFakeData, "0", "Send fake, hardcoded data instead of optitrack for testing");  
    
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    std::string interface = getopt_get_string(gopt, kInterfaceArg);
    int baudRate = getopt_get_int(gopt, kSerialBaudArg);
    std::string serialPort = getopt_get_string(gopt, kSerialPort);
    int xbeeAddr = getopt_get_int(gopt, kXbeeAddrArg);
    int BBrigidBodyId = getopt_get_int(gopt, kBBRigidBodyArg);
    int NumGates = getopt_get_int(gopt, kNumGatesArg);
    int testingFakeData = getopt_get_int(gopt, kTestingFakeData);

    //open serial port
    int fd = serial_open(serialPort.c_str(),baudRate,1);    // blocking while sending
    if(fd == -1)
    {
        printf("Failed to open Serial Port");
        return 1;
    }

    char config_mode[] = "+++";
    char dest_addr_read[] = "ATDL\r\n";
    char dest_addr_cmd[80];
    sprintf(dest_addr_cmd, "ATDL %d\r\n", xbeeAddr);
    char cmd_null[] = "ATCN\r\n";
    char xbee_resp[] = "OK";
    char resp[10];

    //Program XBee Desitnation Address
    printf("Programming XBee...\n");

    write(fd,config_mode,sizeof(config_mode)-1);
    usleep(1E5);
    read(fd,resp,sizeof(resp));
    if(!strcmp(resp,xbee_resp)){
        printf("Recieved Incorrect Respone\n");
    }
    
    write(fd,dest_addr_cmd,sizeof(dest_addr_cmd)-1);
    usleep(1E5);
    read(fd,resp,sizeof(resp));
    if(!strcmp(resp,xbee_resp)){
        printf("Recieved Incorrect Respone\n");
    }

    write(fd,dest_addr_read,sizeof(dest_addr_read)-1);
    usleep(1E5);
    read(fd,resp,sizeof(resp));
    char dest_addr[10];
    sprintf(dest_addr,"%d",xbeeAddr);
    if(!strcmp(resp,dest_addr)){
        printf("Recieved Incorrect Respone\n");
    }
    
    write(fd,cmd_null,sizeof(cmd_null)-1);
    usleep(1E5);
    read(fd,resp,sizeof(resp));
    if(!strcmp(resp,xbee_resp)){
        printf("Recieved Incorrect Respone\n");
    } 
    printf("RESP:%s\n",resp);



    // Get Initial Time
    int64_t init_time64_u = utime_now();
    int64_t time64_u = utime_now();
    int64_t time_u = time64_u - init_time64_u;

    balancebot_msg_t BBmsg;
    BBmsg.utime = time_u;
    BBmsg.num_gates = NumGates;
    

    if (testingFakeData) {
        unsigned int hz = testingFakeData;

        balancebot_gate_t gate;
        gate.left_post[0] = 0.0;
        gate.left_post[1] = 1.0;
        gate.right_post[0] = 0.0;
        gate.right_post[1] = 2.0;
        BBmsg.gates.push_back(gate);
        gate.left_post[0] = 1.0;
        gate.left_post[1] = 0.0;
        gate.right_post[0] = 2.0;
        gate.right_post[1] = 0.0;
        BBmsg.gates.push_back(gate);
        gate.left_post[0] = 0.0;
        gate.left_post[1] = -1.0;
        gate.right_post[0] = 0.0;
        gate.right_post[1] = -2.0;
        BBmsg.gates.push_back(gate);
        gate.left_post[0] = -1.0;
        gate.left_post[1] = 0.0;
        gate.right_post[0] = -2.0;
        gate.right_post[1] = 0.0;
        BBmsg.gates.push_back(gate);
        
        pose_xyt_t Pose;
        Pose.utime = time_u;
        Pose.x = 0.5;
        Pose.y = -0.5;
        Pose.theta = 0.25;
        BBmsg.pose = Pose;

        printf("Sending Fake Data...\n");


        int packetLength = BBmsg.getEncodedSize()+2;
        char *dataPacket = (char*) malloc (packetLength);
        printf("Packet Length: %d\n", packetLength);
        
        while (1) {
            usleep(1E6/hz);  
            time64_u = utime_now();
            time_u = time64_u - init_time64_u;
            BBmsg.utime = time_u;
            // Construct Serial Message

            BBmsg.encode(dataPacket,1,packetLength);
            dataPacket[0] = 0x1B;
            dataPacket[packetLength-1] = 0xFF;
            // send serial message  (returns # of bytes sent on success)
            if(write(fd,dataPacket,packetLength) > 0) {
                // "flush" the data 
                fsync(fd);
            } 
            else {
                printf("Error: %d \n",errno);
            }    
        }
    }

    // If there's no interface specified, then we'll need to guess
    if (interface.length() == 0)
    {
        interface = guess_optitrack_network_interface();
    }
    // If there's still no interface, we have a problem
    if (interface.length() == 0)
    {
        printf("[optitrack_driver] error could not determine network interface for receiving multicast packets.\n");
        return -1;
    }
    
    SOCKET dataSocket = create_optitrack_data_socket(interface, PORT_DATA);
    
    if (dataSocket == -1) {
        printf("[optitrack_driver] error failed to create socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
        return -1;
    } else {
        printf("[optitrack_driver] successfully created socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
    }
    
    // Code from DataListenThread function in PacketClient.cpp
    char packet[20000];
    socklen_t addrLen = sizeof(sockaddr);
    sockaddr_in incomingAddress;
    std::vector<optitrack_message_t> incomingMessages;

    printf("\nOptitrack Driver Running...\n");
    while (1) {
        balancebot_msg_t BBmsg;
        BBmsg.num_gates = NumGates;
        // Block until we receive a datagram from the network
        recvfrom(dataSocket, packet, sizeof(packet), 0, (sockaddr*)&incomingAddress, &addrLen);
        incomingMessages = parse_optitrack_packet_into_messages(packet, sizeof(packet));
        time64_u = utime_now();
        time_u = time64_u - init_time64_u; 
        for(auto& msg : incomingMessages) {
            if (msg.id <= NumGates)
            {
                int pair = 0;
                float m[3][2];
                balancebot_gate_t gate;
                int i;
                for(i=0;i<3;i++){
                    m[i][0] = msg.markers[i].x;
                    m[i][1] = msg.markers[i].z;
                    //printf("Marker %d: (%f, %f)\n",i,m[i][0],m[i][1]);
                }
                for(i=0; i<2;i++){
                    float xd = m[i][0]-m[i+1][0];
                    float yd = m[i][1]-m[i+1][1];
                    float d = sqrt(xd*xd+yd*yd);
                    //printf("Dist (%d,%d) = %f\n",i,i+1,d);
                    if( d < 0.2){
                        pair = i+1;
                        break;
                    }
                }

                if(pair == 1){ // left = (m0, m1)
                    gate.left_post[0] = (m[0][0]+m[1][0])/2;
                    gate.left_post[1] = (m[0][1]+m[1][1])/2;
                    gate.right_post[0] = m[2][0];
                    gate.right_post[1] = m[2][1];
                }
                else if(pair == 2){ // left = (m1, m2)
                    gate.left_post[0] = (m[1][0]+m[2][0])/2;
                    gate.left_post[1] = (m[1][1]+m[2][1])/2;
                    gate.right_post[0] = m[0][0];
                    gate.right_post[1] = m[0][1];
                }
                
                else{ // left = (m0, m2)
                    gate.left_post[0] = (m[0][0]+m[2][0])/2;
                    gate.left_post[1] = (m[0][1]+m[2][1])/2;
                    gate.right_post[0] = m[1][0];
                    gate.right_post[1] = m[1][1];
                }
                //printf("ID:%d, left=%d\n",msg.id,pair);
                BBmsg.gates.push_back(gate); 
            }

            else if(msg.id == BBrigidBodyId) {
                pose_xyt_t Pose;
                Pose.utime = time_u;
                Pose.x = msg.x;
                Pose.y = -msg.z; //optitrack z is world -y
                double roll;
                double pitch;
                double yaw;
                toEulerAngle(msg, roll, pitch, yaw);
                Pose.theta = yaw;
                BBmsg.pose = Pose;
            }            
        }
        BBmsg.utime = time_u;

        int packetLength = BBmsg.getEncodedSize()+2;
        char *dataPacket = (char*) malloc (packetLength);
        BBmsg.encode(dataPacket,1,packetLength);
        dataPacket[0] = 0x1b;
        dataPacket[packetLength-1] = 0xFF;
        // send serial message  (returns # of bytes sent on success)
        if(write(fd,dataPacket,packetLength) > 0) {
            // "flush" the data 
            fsync(fd);
        } 
        else {
            printf("Error: %d \n",errno);
        }  

    }
    // Cleanup options now that we've parsed everything we need
    serial_close(fd);
    getopt_destroy(gopt);

    return 0;
}
