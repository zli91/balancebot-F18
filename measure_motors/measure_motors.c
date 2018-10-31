/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>
#include "../common/mb_motor.h"

FILE* f1;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();


    rc_set_state(RUNNING);
    while(rc_get_state()!=EXITING){
        mb_motor_init();
        int count = 0;
        double current = 0;
        /* test K */
        mb_motor_set(1, -1.0);
        while(1){
            count = rc_encoder_eqep_read(1);
            current = mb_motor_read_current(1);
            printf("%d\t%f\n",count,current);
            rc_nanosleep(5E7);
            if(count > 5000 || count < -5000){
                mb_motor_set(1, 0.0);
                break;
            }
        }
        int i = 0;
        for(i = 0;i<80;i++){
            count = rc_encoder_eqep_read(1);
            current = mb_motor_read_current(1);
            printf("%d\t%f\n",count,current);
            rc_nanosleep(5E7);
        }
        /* test c (static friction)
        int i = 0, count2 = 0;
        for(i = 0;i<100;i++){
            mb_motor_set(1, i*0.001);
            current = mb_motor_read_current(1);
            count = -rc_encoder_eqep_read(1);
            rc_nanosleep(1E7);
            count2 = -rc_encoder_eqep_read(1);
            rc_nanosleep(1E7);
            printf("%f\n",current);
            if(count2-count>=5){
                mb_motor_brake(1);
                break;
            }
        }*/
        
        /* test I_M
        mb_motor_set(1, -1.0);
        while(1){
            count = rc_encoder_eqep_read(1);
            current = mb_motor_read_current(1);
            printf("%d\t%f\n",count,current);
            rc_nanosleep(1E7);
            if(count > 1000 || count < -1000){
                mb_motor_brake(1);
                break;
            }
        }
        */
        
        rc_set_state(EXITING);
    }
	// exit cleanly
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}