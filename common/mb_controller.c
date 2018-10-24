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

// static rc_filter_t D1 = RC_FILTER_INITIALIZER;

int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    // double num[3] = {0.1152, -0.2284, 0.1132};
    // double den[3] = {0.1152, -0.2284, 0.1132};
    
    // double DT = 0.01;
    // return rc_filter_alloc_from_arrays(&D1, DT, num, 3, den, 3);
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

int mb_controller_update(mb_state_t* mb_state){
    /*TODO: Write your controller here*/
    // float theta_r = 0.0;
    // float e_theta = theta_r - mb_state->theta;
    // double duty = rc_filter_march(&D1, e_theta);

    // mb_state.left_cmd = duty;
    // mb_state.right_cmd = duty;



    // mb_state_t state = *mb_state;
    // mb_setpoints_t* setpoint = mb_state->setpoint;
    float phi_r = mb_state->setpoint->fwd_velocity * 0;  // need to convert m/s into rad

    /* Outer loop control */
    float ao1 = 0.1152, ao2 = -0.2284, ao3 = 0.1132;
    float bo1 = 1, bo2 = -1.95, bo3 = 0.9499;

    float theta_r;                              // output of outer loop controller which is the reference for inner loop
    mb_state.e_phi = phi_r - mb_state.phi;
    theta_r = 1/bo1 * (ao1*mb_state.e_phi + ao2*eo1 + ao3*eo2 - bo2*uo1 - bo3*uo2);


    /* Inner loop control */
    float ai1 = 0.1152, ai2 = -0.2284, ai3 = 0.1132;
    float bi1 = 1, bi2 = -1.95, bi3 = 0.9499;

    mb_state.e_theta = theta_r - mb_state.theta;
    float duty = 1/bi1 * (ai1*mb_state.e_phi + ai2*ei1 + ai3*ei2 - bi2*ui1 - bi3*ui2);

    // update the current state 
    // theta is updated by the IMU
    mb_state.phi = mb_state.left_encoder * 2*PI /979.2;
    mb_state.left_cmd = duty;
    mb_state.right_cmd = duty;

    // update the error
    mb_state.eo2 = mb_state.eo1;
    mb_state.eo1 = mb_state.e_phi;

    mb_state.ei2 = mb_state.ei1;
    mb_state.ei1 = mb_state.e_theta;
    

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

    return 0;//rc_filter_free(D1);
}