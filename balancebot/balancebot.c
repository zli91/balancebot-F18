/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>

#include "balancebot.h"

/*******************************************************************************
* int main() 
*
*******************************************************************************/
rob_data_t rob_data;
int dsm_ch5, dsm_ch7;
const float straight_distance = 11.0; //meters 
const float left_turn_distance = 1.0; // meters
const float fwd_vel_max = 0.6;
const float turn_vel_max = 2;
mb_odometry_t tmp_odometry;

void robot_init(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, rob_data_t* rob_data){
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	rob_data-> kp1 = -5.15; //-6.086;
    rob_data-> ki1 = -10.423; //-18.798;
    rob_data-> kd1 = -0.209; //-0.198;

    rob_data-> kp2 = -0.0204;//-0.0163;
    rob_data-> ki2 = -0.0208;//-0.0021;
    rob_data-> kd2 = -0.0106;//-0.0133;
    
    rob_data-> kp3 = 1.2;
    rob_data-> ki3 = 0.4;
    rob_data-> kd3 = 0.04;

    rob_data-> body_angle = 0.023;

    mb_state-> phi_old = 0;
	mb_state-> psi_old = 0;
	mb_state-> yaw = 0;
	mb_setpoints-> fwd_velocity = 0;
	mb_setpoints-> turn_velocity = 0;

}

void position_init(){
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);
	mb_state.phi_old = 0;
	mb_state.psi_old = 0;
	mb_state.phi_r = 0;
	mb_state.psi_r = 0;
	mb_setpoints.fwd_velocity = 0;
	mb_setpoints.turn_velocity = 0;
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);
}

float velocity_mapping(float initial, float current, float target, float max_speed){
	if (current-initial<=0) return 0.5 * max_speed;
	else if(current-target<=0) return 0;
	else{
		float alpha = target - initial;
		float beta = 0.5 * max_speed;
		float x = current - initial;
		return -4*beta/pow(x/alpha,2) + 4*beta/alpha*x + (0.3 * max_speed);
	}
}

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

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;


	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init(&mb_state);

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);
	mb_odometry_init(&tmp_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	//Initialize
	robot_init(&mb_state, &mb_setpoints, &rob_data);

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X] - rob_data.body_angle;
	mb_state.prev_yaw = mb_state.yaw;
	mb_state.yaw = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	// Read encoders
	mb_state.prev_left_encoder = mb_state.left_encoder;
	mb_state.prev_right_encoder = mb_state.right_encoder;
	mb_state.left_encoder = -rc_encoder_eqep_read(2);
	mb_state.right_encoder = rc_encoder_eqep_read(1);

	// Average wheel angle
	mb_state.phi = 0.5*(mb_state.left_encoder + mb_state.right_encoder)/(GEAR_RATIO*ENCODER_RES)*(2*3.14159);

	// Update command velocities
    float vel = mb_setpoints.fwd_velocity;
    float avel = mb_setpoints.turn_velocity;

    mb_state.phi_r = mb_state.phi_old + vel*DT/(0.5*WHEEL_DIAMETER);
    mb_state.phi_old = mb_state.phi_r;

    mb_state.psi_r = mb_state.psi_old + avel*DT;
    mb_state.psi_old = mb_state.psi_r;

	
    // Update odometry 
 	mb_odometry_update(&mb_odometry, &mb_state);

    // Calculate controller outputs    
	if(mb_setpoints.manual_ctl == 0){
    	//send motor commands
    	mb_controller_update(&mb_state, &mb_setpoints, &mb_odometry, &rob_data);
		mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
		mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
    	}else if(mb_setpoints.manual_ctl == 1){
    	//send motor commands
    	mb_controller_update(&mb_state, &mb_setpoints, &mb_odometry, &rob_data);
		mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
		mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
    	}

   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){
	int init_switch = 0;
	float distance = 0;
	float angles = 0;
	int left_turn_task = ROB_FORWARD;
	int left_turn_count = 0;
	while(1){
		if(rc_dsm_is_new_data()){
			if(rc_dsm_ch_normalized(7)>=0.9) dsm_ch7 = 1;
			else if(rc_dsm_ch_normalized(7)<=-0.9) dsm_ch7 = -1;
			else dsm_ch7 = 0;
			if(rc_dsm_ch_normalized(5)>=0.9) dsm_ch5 = 1;
			else if(rc_dsm_ch_normalized(5)<=-0.9) dsm_ch5 = -1;
			else dsm_ch5 = 0;

			if(dsm_ch7 == 0 && dsm_ch5 ==0){
				/* Auto mode - Default */
				mb_setpoints.manual_ctl = 0;
				mb_setpoints.fwd_velocity = 0;
				mb_setpoints.turn_velocity = 0;
			}else if(dsm_ch7 == 0 && dsm_ch5 == -1){
				/* Auto mode - initialize */
				position_init();
			}else if(dsm_ch7 == 0 && dsm_ch5 == 1){
				/* Auto mode - Empty channel */
				mb_setpoints.manual_ctl = 0;
			}else if(dsm_ch7 == 1 && dsm_ch5 == 0){
				/* Manual mode - Default */
				mb_setpoints.manual_ctl = 1;
				mb_setpoints.fwd_velocity = fwd_vel_max * rc_dsm_ch_normalized(3);
				mb_setpoints.turn_velocity = turn_vel_max * rc_dsm_ch_normalized(4);
			}else if(dsm_ch7 == 1 && dsm_ch5 == -1){
				/* Manual mode - Tune Outer Loop */
				mb_setpoints.manual_ctl = 1;
				//rob_data.kp2 += + 0.001 * rc_dsm_ch_normalized(2);
				//rob_data.ki2 += + 0.0001 * rc_dsm_ch_normalized(4);
				//rob_data.kd2 += + 0.0001* rc_dsm_ch_normalized(3);
				rob_data.kp3 += + 0.001 * rc_dsm_ch_normalized(2);
				rob_data.ki3 += + 0.001 * rc_dsm_ch_normalized(4);
				rob_data.kd3 += + 0.001* rc_dsm_ch_normalized(3);
			}else if(dsm_ch7 == 1 && dsm_ch5 == 1){
				/* Manual mode - Tune Inner Loop */
				mb_setpoints.manual_ctl = 1;
				rob_data.kp1 += 0.1 * rc_dsm_ch_normalized(2);
				rob_data.ki1 += + 0.01 * rc_dsm_ch_normalized(4);
				rob_data.kd1 += + 0.001* rc_dsm_ch_normalized(3);
			}else if(dsm_ch7 == -1 && dsm_ch5 == 0){
				/* Task mode - Default */
				init_switch = 1;
				left_turn_task = ROB_FORWARD;
				left_turn_count = 0;
			}else if(dsm_ch7 == -1 && dsm_ch5 == -1){
				/* Task mode - Straight Line Drag Racing */
				mb_setpoints.manual_ctl = 0;
				if(init_switch) mb_odometry_copy(&tmp_odometry, &mb_odometry);
				init_switch = 0;
				distance = mb_odometry_distance(&mb_odometry, &tmp_odometry);
				//mb_setpoints.fwd_velocity = velocity_mapping(0.0, distance, straight_distance, fwd_vel_max);
				if(distance > straight_distance) mb_setpoints.fwd_velocity = 0;
				else if(distance > 0.90*straight_distance) mb_setpoints.fwd_velocity = fwd_vel_max * 0.5;
				else if(distance > 0.70*straight_distance) mb_setpoints.fwd_velocity = fwd_vel_max * 0.7;
				else if(distance > 0.30*straight_distance) mb_setpoints.fwd_velocity = fwd_vel_max * 0.8;
				else if(distance > 0.10*straight_distance) mb_setpoints.fwd_velocity = fwd_vel_max * 0.7;
				else mb_setpoints.fwd_velocity = fwd_vel_max * 0.5;
			}else if(dsm_ch7 == -1 && dsm_ch5 == 1){
				/* Task mode - 4 Left Turns */
				if(init_switch){
					mb_odometry_copy(&tmp_odometry, &mb_odometry);	
				} 
				init_switch = 0;
				mb_state.psi_r = tmp_odometry.psi + PI/2;
				mb_state.psi_old = tmp_odometry.psi + PI/2;
				mb_setpoints.turn_velocity = 0;
				/*
				switch (left_turn_task)
				{
					case ROB_FORWARD:
						distance = mb_odometry_distance(&mb_odometry, &tmp_odometry);
						if(distance >= left_turn_distance){
							mb_setpoints.fwd_velocity = 0;
							left_turn_count ++;
							if(left_turn_count >= 4) left_turn_task = ROB_STOP;
							else left_turn_task = ROB_TURN;
						}else mb_setpoints.fwd_velocity = fwd_vel_max * 0.5;
						break;
					case ROB_TURN:
						angles = mb_odometry_angles(&mb_odometry, &tmp_odometry);
						if(angles >= PI/2-0.17){
							mb_setpoints.turn_velocity = 0;
							left_turn_task = ROB_FORWARD;
							init_switch = 1;
						}else mb_setpoints.turn_velocity = turn_vel_max;
						break;
					case ROB_STOP:
						mb_setpoints.fwd_velocity = 0;
						mb_setpoints.turn_velocity = 0;
						break;
				}
				*/

			}else printf("ERROR: No DSM channel\n");
		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}


/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |");
			printf("           ODOMETRY          |");
			printf("                            PID                            |");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf("   KP1   |");
			printf("   KI1   |");
			printf("   KD1   |");
			printf("   KP2   |");
			printf("   KI2   |");
			printf("   KD2   |");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%7.3f  |", mb_odometry.psi);
			printf("%7.3f  |", rob_data.kp1);
			printf("%7.3f  |", rob_data.ki1);
			printf("%7.3f  |", rob_data.kd1);
			printf("%7.4f  |", rob_data.kp3);
			printf("%7.4f  |", rob_data.ki3);
			printf("%7.4f  |", rob_data.kd3);
			printf("%7.3f  |", mb_setpoints.fwd_velocity);
			printf("%7.4f  |", mb_state.yaw);
			printf("%7.4f  |", mb_state.psi_r);
			printf("%7.4f  |", (mb_state.left_cmd-mb_state.right_cmd)/2);
			//printf("%6.4f|", mpu_data.dmp_TaitBryan[TB_YAW_Z]);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 
