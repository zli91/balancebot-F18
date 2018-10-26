#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH "pid.cfg"
#define PI 3.1415926

int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state, double Kp1, double Ki1, double Kd1, double Kp2, double Ki2, double Kd2);
int mb_controller_cleanup();

#endif

