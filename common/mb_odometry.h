/*******************************************************************************
* mb_odometry.h
*
* 
*******************************************************************************/
#ifndef MB_ODOMETRY_H
#define MB_ODOMETRY_H

#include "mb_defs.h"
#include "mb_structs.h"

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta);
void mb_odometry_copy(mb_odometry_t* mb_odometry_out, mb_odometry_t* mb_odometry_in);
float mb_odometry_distance(mb_odometry_t* mb_odometry1, mb_odometry_t* mb_odometry2);
float mb_odometry_angles(mb_odometry_t* mb_odometry_final, mb_odometry_t* mb_odometry_initial);
void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state);
float mb_clamp_radians(float angle);
int mb_in_range(float num, float min, float max);

#endif