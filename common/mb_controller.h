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

typedef struct mb_pid mb_pid_t;
struct mb_pid{
    float kp; 
    float ki; 
    float kd; 
    float Tf;  // 1/cutoff_frequency [s]
    float d1;  // Ts/2+Tf
    float d2;  // Ts/2-Tf
    float e;   // e[k]
    float e_1; // e[k-1]
    float e_2; // e[k-2]
    float u;   // u[k]
    float u_1; // u[k-1]
    float u_2; // u[k-2]
};
mb_pid_t inner;
int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, double Kp1, double Ki1, double Kd1, double Kp2, double Ki2, double Kd2);
int mb_controller_cleanup();

#endif

