/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
	/* TODO */
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->psi = theta;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
	/* TODO */
	float left = (mb_state->left_encoder - mb_state->prev_left_encoder)/(GEAR_RATIO*ENCODER_RES)*(2*PI)*WHEEL_DIAMETER/2;
	float right = (mb_state->right_encoder - mb_state->prev_right_encoder)/(GEAR_RATIO*ENCODER_RES)*(2*PI)*WHEEL_DIAMETER/2;
	float x = mb_odometry->x;
	float y = mb_odometry->y;
	float psi = mb_odometry->psi;
	float dtheta = -(right - left) / WHEEL_BASE;
	float d = (right + left) / 2;

	x = x + d * cos(psi + dtheta/2);
	y = y + d * sin(psi + dtheta/2);
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->psi = psi + dtheta;
	//printf("x = %f\t y = %f\t psi=%f\t", x,y,psi + dtheta);
}


float mb_clamp_radians(float angle){


    return 0;
}