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

static rc_filter_t D1 = RC_FILTER_INITIALIZER;
static rc_filter_t D2 = RC_FILTER_INITIALIZER;
mb_pid_t inner, outer;
controller_pid_t inner_loop, outer_loop;

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


void pid_cont(mb_pid_t* mb_pid, float u_min, float u_max){
    
    float kp = mb_pid->kp;
    float ki = mb_pid->ki;
    float kd = mb_pid->kd;
    float d1 = mb_pid->d1;
    float d2 = mb_pid->d2;
    float e = mb_pid->e;
    float e_1 = mb_pid->e_1;
    float e_2 = mb_pid->e_2;
    float u_1 = mb_pid->u_1;
    float u_2 = mb_pid->u_2;

    float temp_u = -(d2-d1)/d1*u_1 + d2/d1*u_2;
    float temp_P = kp*(e+(d2-d1)/d1*e_1-d2/d1*e_2);
    float temp_I = ki*DT*0.5*(e+(d1+d2)/d1*e_1+d2/d1*e_2);
    float temp_D = kd*(e-2.0*e_1+e_2)/d1;

    float sum = temp_u + temp_P + temp_I + temp_D;
    if(sum>u_max) sum = u_max;
    if(sum<u_min) sum = u_min;
    mb_pid->u = sum;
}


void mb_pid_init(mb_pid_t* pid){
    pid->d1 = DT/2.0 + pid->Tf;
    pid->d2 = DT/2.0 - pid->Tf;
    pid->e = 0;
    pid->e_1 = 0;
    pid->e_2 = 0;
    pid->u = 0;
    pid->u_1 = 0;
    pid->u_2 = 0;
}

int mb_controller_init(mb_state_t* mb_state){
    mb_controller_load_config();
    inner.Tf = 0.0002222;
    mb_pid_init(&inner);
    outer.Tf = 0.07414;
    mb_pid_init(&outer);
    /* CK */
    controller_pid_init(&inner_loop);
    /* TODO initialize your controllers here*/
    // *mb_state = (mb_state_t){.e_phi=0.0, .ei1=0.0, .ei2=0.0, .eo1=0.0, .eo2=0.0, .e_theta=0.0, .ui1=0.0, .ui2=0.0, .uo1=0.0, .uo2=0.0};

    // double num[3] = {0.1152, -0.2284, 0.1132};
    // double den[3] = {0.1152, -0.2284, 0.1132};
    rc_filter_enable_saturation(&D1,-1.0,1.0);
    //rc_filter_enable_soft_start(&D1,0.7);
    return 0;
    //return rc_filter_pid(&D1, -5.474, -15, -0.00, 0.04, 0.01)==0 && rc_filter_pid(&D2, 0.02214, 0.009945, 0.04473, 0.04, 0.01)==0;
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

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, double Kp1, double Ki1, double Kd1, double Kp2, double Ki2, double Kd2){
    /*TODO: Write your controller here*/
    /* CK */
    //controller_set_pid(&inner_loop, Kp1, Ki1, Kd1, 0.0002222, 0.01);
    //float duty = controller_march(&inner_loop, 0 - mb_state->theta , -1.0, 1.0);
    
    outer.kp = Kp2;
    outer.ki = Ki2;
    outer.kd = Kd2;
    inner.kp = Kp1;
    inner.ki = Ki1;
    inner.kd = Kd1;

    outer.e_2 = outer.e_1;
    outer.e_1 = outer.e;

    outer.e = 0 - mb_state->phi;
    outer.u_2 = outer.u_1;
    outer.u_1 = outer.u;
    pid_cont(&outer, -0.35, 0.35);

    // inner loop
    inner.e_2 = inner.e_1;
    inner.e_1 = inner.e;
    inner.e = /*outer.u*/ - mb_state->theta;
    inner.u_2 = inner.u_1;
    inner.u_1 = inner.u;
    pid_cont(&inner, -1.0, 1.0);

    float duty = inner.u;
    //if(duty>0) duty+=0.1;
    //else duty -=0.1; 
    mb_state->left_cmd = duty;
    printf("duty = %f\n", duty);
    /*
    // Outer loop
    rc_filter_pid(&D2, Kp2, Ki2, Kd2, 0.01, 0.01);
    float phi_r = mb_setpoints->phi_r;
    float e_phi = phi_r - mb_state->phi;
    float theta_r = rc_filter_march(&D2, e_phi);

    // ck's
    // float e_phi = vel*DT/(0.5*WHEEL_DIAMETER) - mb_state->phi;
    printf("phi_r=%0.4f\te_phi=%0.4f\tmb_state->phi=%0.4f\ttheta_r = %0.4f\tmb_theta = %0.4f\n", phi_r, e_phi, mb_state->phi, theta_r,mb_state->theta);
    

    // Inner loop

    rc_filter_pid(&D1, Kp1, Ki1, Kd1, 0.04, 0.01);
    rc_filter_enable_saturation(&D1,-1.0,1.0);
    //float theta_r = 0.0;
    float e_theta = (theta_r - mb_state->theta);
    double duty = rc_filter_march(&D1, e_theta);
    printf("duty = %f\n", duty);
    if (duty < 0.2 && duty > 0) duty = 0.2;
    else if(duty > -0.2 && duty <= 0) duty =-0.2;    
    if (duty < -1.0) duty = -1.0;
    else if(duty > 1.0) duty = 1.0;
    mb_state->left_cmd = duty;
    mb_state->right_cmd = duty;

    */
    //if(e_theta<0.025) mb_controller_cleanup();

    /* old method */
    // // float phi_r = mb_state->setpoint->fwd_velocity * 0;  // need to convert m/s into rad

    // /* Outer loop control */
    // // float ao1 = 0.1152, ao2 = -0.2284, ao3 = 0.1132;
    // // float bo1 = 1, bo2 = -1.95, bo3 = 0.9499;
    // // float eo1 = mb_state->eo1, eo2 = mb_state->eo2;
    // // float uo1 = mb_state->uo1, uo2 = mb_state->uo2;

    // // float theta_r;                              // output of outer loop controller which is the reference for inner loop
    // // mb_state->e_phi = phi_r - mb_state->phi;
    // // theta_r = 1/bo1 * (ao1*mb_state->e_phi + ao2*eo1 + ao3*eo2 - bo2*uo1 - bo3*uo2);

    // float theta_r = 0;  // test inner loop only
    // /* Inner loop control */
    // float ai1 = -26.29, ai2 = 43.14, ai3 = -17.7;
    // float bi1 = 1.0, bi2 = -0.08385, bi3 = -0.9161;
    // float ei1 = mb_state->ei1, ei2 = mb_state->ei2;
    // float ui1 = mb_state->ui1, ui2 = mb_state->ui2;

    // mb_state->e_theta = theta_r - mb_state->theta;
    // printf("etheta = %f\n", mb_state->e_theta);
    // float duty = 1/bi1 * (ai1*mb_state->e_theta + ai2*ei1 + ai3*ei2 - bi2*ui1 - bi3*ui2);
    
    // // update the current state 
    // // theta is updated by the IMU
    // mb_state->phi = mb_state->left_encoder * 2*PI /979.2;
    // mb_state->left_cmd = duty;
    // mb_state->right_cmd = duty;
    // printf("duty = %f\n", mb_state->left_cmd);
    // // update the error
    // mb_state->eo2 = mb_state->eo1;
    // mb_state->eo1 = mb_state->e_phi;

    // mb_state->ei2 = mb_state->ei1;
    // mb_state->ei1 = mb_state->e_theta;

    // // update input inner loop
    // mb_state->ui2 = mb_state->ui1;
    // mb_state->ui1 = duty;

    // // update input outer loop TODO
    
    
    // // print 
    // printf("%f\t%f\n",mb_state->theta, mb_state->e_theta);

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
    rc_filter_free(&D1);
    rc_filter_free(&D2);
    return 0;
}
