/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

const float dtheta_thresh = 0.05;

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
	/* TODO */
	mb_odometry->x   = x;
	mb_odometry->y   = y;
	mb_odometry->psi = theta;
}

void mb_odometry_copy(mb_odometry_t* mb_odometry_out, mb_odometry_t* mb_odometry_in){
	mb_odometry_out->x   = mb_odometry_in->x;
	mb_odometry_out->y   = mb_odometry_in->y;
	mb_odometry_out->psi = mb_odometry_in->psi;
}

float mb_odometry_distance(mb_odometry_t* mb_odometry1, mb_odometry_t* mb_odometry2){
	return sqrt( pow(mb_odometry1->x-mb_odometry2->x,2) + pow(mb_odometry1.y-mb_odometry2.y,2));
}

float mb_odometry_angles(mb_odometry_t* mb_odometry_final, mb_odometry_t* mb_odometry_initial){
	return mb_odometry_final->psi - mb_odometry_initial->psi;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
	/* TODO */
	float left  = (mb_state->left_encoder - mb_state->prev_left_encoder)/(GEAR_RATIO*ENCODER_RES)*(2*PI)*WHEEL_DIAMETER/2;
	float right = (mb_state->right_encoder - mb_state->prev_right_encoder)/(GEAR_RATIO*ENCODER_RES)*(2*PI)*WHEEL_DIAMETER/2;
	float x = mb_odometry->x;
	float y = mb_odometry->y;
	float psi = mb_odometry->psi;
	float dtheta = -(right - left) / WHEEL_BASE;
	float d = (right + left) / 2;

	/* Gyrodometry */
	float dyaw = mb_state->yaw - mb_state->prev_yaw;
	if(dyaw > PI/2) dyaw = PI - dyaw;
	else if(dyaw < -PI/2) dyaw = -PI - dyaw;
	
	// need testing
	//float dtheta_diff = abs(dyaw - dtheta);
	//if(dtheta_diff > dtheta_thresh) dtheta = dyaw;


	x = x + d * cos(psi + dtheta/2);
	y = y + d * sin(psi + dtheta/2);
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->psi = psi + dtheta;
}


float mb_clamp_radians(float angle){


    return 0;
}