#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"
#include <rc/math/filter.h>

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

controller_pid_t inner_loop, outer_loop, turning_loop;


void controller_pid_init(controller_pid_t* controller){
    controller->e_0 = 0;
    controller->e_1 = 0;
    controller->e_2 = 0;
    controller->u_0 = 0;
    controller->u_1 = 0;
    controller->u_2 = 0;
}

void controller_set_pid(controller_pid_t* controller, float Kp, float Ki, float Kd, float Tf, float Ts){
    controller->Kp = Kp;
    controller->Ki = Ki;
    controller->Kd = Kd;
    controller->Tf = Tf;
    controller->Ts = Ts;
}

float controller_march(controller_pid_t* controller, float input, float min, float max){
    controller->e_2 = controller->e_1;
    controller->e_1 = controller->e_0;
    controller->e_0 = input;
    controller->u_2 = controller->u_1;
    controller->u_1 = controller->u_0;
    
    float Kp = controller->Kp;
    float Ki = controller->Ki;
    float Kd = controller->Kd;
    float Tf = controller->Tf;
    float Ts = controller->Ts;
    float e_0 = controller->e_0;
    float e_1 = controller->e_1;
    float e_2 = controller->e_2;
    float u_1 = controller->u_1;
    float u_2 = controller->u_2;
    float a = 1 + 0.5*Ts/Tf;
    float b = 1 - 0.5*Ts/Tf;
    
    float U = 2*u_1 - b*u_2;
    float KP = Kp * (a*e_0 - 2*e_1 + b*e_2);
    float KI = Ki * (0.5*Ts*a*e_0 + 0.5*Ts*Ts/Tf*e_1 - 0.5*Ts*b*e_2);
    float KD = Kd * (1/Tf*e_0 - 2/Tf*e_1 + 1/Tf*e_2);
    
    float u_0 = (U + KP + KD + KI)/a;
    if(u_0 < min) u_0 = min;
    if(u_0 > max) u_0 = max;
    controller->u_0 = u_0;
    return u_0;
}

int mb_controller_init(mb_state_t* mb_state){
    //mb_controller_load_config();
    /* TODO initialize your controllers here*/
    controller_pid_init(&inner_loop);
    controller_pid_init(&outer_loop);
    controller_pid_init(&turning_loop);
    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    /* TODO parse your config file here*/
    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry, rob_data_t* rob_data){
    /*TODO: Write your controller here*/

    // Outer Loop
    controller_set_pid(&outer_loop, rob_data->kp2, rob_data->ki2, rob_data->kd2, 0.074, DT);
    float theta_r = controller_march(&outer_loop, mb_state->phi_r - mb_state->phi , -0.5, 0.5);

    // Inner Loop
    controller_set_pid(&inner_loop, rob_data->kp1, rob_data->ki1, rob_data->kd1, 0.0002222, DT);
    float duty = controller_march(&inner_loop, theta_r- mb_state->theta , -1.0, 1.0);

    // Turning Loop
    controller_set_pid(&turning_loop, rob_data->kp3, rob_data->ki3, rob_data->kd3, 0.01, DT);
    float diff_duty = controller_march(&turning_loop, mb_state->psi_r - mb_odometry->psi , -0.8, 0.8);

    if(duty + diff_duty < -1.0) mb_state->left_cmd = -1.0;
    else if(duty + diff_duty > 1.0) mb_state->left_cmd = 1.0;
    else mb_state->left_cmd = duty + diff_duty;

    if(duty - diff_duty > 1.0) mb_state->right_cmd = 1.0;
    else if(duty - diff_duty < -1.0) mb_state->right_cmd = -1.0;
    else mb_state->right_cmd = duty - diff_duty;
    
    //printf("theta_r= %f\tduty= %f\tdiff_duty=%f\tpsi_r=%f\n",theta_r, duty, diff_duty, mb_state->psi_r);

    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){

    return 0;
}
