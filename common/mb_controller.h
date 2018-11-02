#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH "pid.cfg"
#define PI 3.1415926

typedef struct controller_pid controller_pid_t;
struct controller_pid{
    float Kp; 
    float Ki; 
    float Kd; 
    float Tf;
    float Ts;  
    float e_0;   
    float e_1; 
    float e_2; 
    float u_0;   
    float u_1; 
    float u_2; 
};
int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry, rob_data_t* rob_data);
int mb_controller_cleanup();

#endif

