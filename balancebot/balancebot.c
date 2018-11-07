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

#include <sys/types.h>
#include <inttypes.h>
#include <sys/ioctl.h>

#include "../lcmtypes/balancebot_msg_t.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/balancebot_gate_t.h"

#include "../optitrack/common/serial.h"


#include "balancebot.h"

rob_data_t rob_data;
mb_odometry_t tmp_odometry;
mb_odometry_t array_odometry[3];
const int num_odometry = 3;
int cnt_num_odom = 0;
int optitrak_task = RTR_R1;
int copy_switch = 0;
float init_phi = 0;
float init_psi = 0;
float target_psi = 0;
float target_distance = 0;

int dsm_ch5 = 0, dsm_ch7 = 0;
const float STRAIGHT_DISTANCE = 11.0; //meters 
float left_turn_distance = 1.0; // meters
const float FWD_VEL_MAX = 0.8;
const float TURN_VEL_MAX = 4;
int init_switch = 0;
float distance = 0;
int left_turn_task = ROB_FORWARD;
int left_turn_count = 0;

/* Motion Capture Parameters */
FILE* f1;
const int baudRate = 57600;
const char port[] = "/dev/ttyO5";
const char headByte = 0x1B;
const char tailByte = 0xFF;
int num_gates = 4;
int fd;
uint64_t last_utime = 0;
int bytes_avail = 0;
int err_counter = 0;
balancebot_msg_t BBmsg;
pose_xyt_t BBpose;
balancebot_gate_t BBgates[num_gates];
/*****************************/

void getData(balancebot_msg_t* BBmsg){
    char *ptr;
    int packetLength = balancebot_msg_t_encoded_size(BBmsg)+2;
    char *dataPacket = (char*) malloc (packetLength);
    ptr = dataPacket;
    while(read(fd, ptr, 1) > 0){
        // if the first Byte is wrong keep looking
        if((ptr == dataPacket)&&(*ptr != headByte)){
            continue;
        }
        ptr++;
        // Once we have all of the Bytes check to make sure first and last are good
        if((ptr-dataPacket) == packetLength){
            if((dataPacket[0] != headByte) || (dataPacket[packetLength-1] != tailByte)){
                err_counter += 1;
            } 
            else{
                //packet is good, decode it into BBmsg
                int status = balancebot_msg_t_decode(dataPacket, 1, packetLength-2, BBmsg);
                if (status < 0) {
                    fprintf (stderr, "error %d decoding balancebot_msg_t!!!\n", status);;
                }
                // if we have less than a full message in the serial buffer
                // we are done, we'll get the next one next time
                ioctl(fd, FIONREAD, &bytes_avail);
                if(bytes_avail < packetLength){
                    break;
                }
            }
            //keep reading until buffer is almost empty
            ptr = dataPacket;
        }
    }
}

void robot_init(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, rob_data_t* rob_data){
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	rob_data-> kp1 = -5.15; //-6.086;
    rob_data-> ki1 = -10.423; //-18.798;
    rob_data-> kd1 = -0.209; //-0.198;

    rob_data-> kp2 = -0.0204;//-0.0163;
    rob_data-> ki2 = -0.0208;//-0.0021;
    rob_data-> kd2 = -0.0106;//-0.0133;
    
    rob_data-> kp3 = -2.0;
    rob_data-> ki3 = -0.4;
    rob_data-> kd3 = -0.04;

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

float turn_velocity_control(mb_state_t* mb_state, float initial_psi, float final_psi){
	float dpsi = final_psi - initial_psi;
	float psi_error = final_psi - mb_state->psi_old;
	float turn_velocity = TURN_VEL_MAX * 1.5 * psi_error;
	if( (dpsi >= 0 && psi_error <= 0.018) || (dpsi < 0 && psi_error >= -0.018)){
		mb_state->psi_r = final_psi;
		return 0;
	}else if(turn_velocity >= 0.8 * TURN_VEL_MAX) return 0.8 * TURN_VEL_MAX;
	else return turn_velocity;
}

void forward_velocity_control(mb_state_t* mb_state, float initial_phi, float distance){
	float rad = distance/(0.5*WHEEL_DIAMETER);
	float phi_target = initial_phi + rad; 
	float phi_error = phi_target - mb_state->phi_old;
	float forward_velocity = FORWARD_VEL_MAX * 0.125 * phi_error;
	if( rad >= 0 && phi_error <= 0.18) || (rad < 0 && phi_error >= -0.18)){
		mb_state->phi_r = phi_target;
		return 0;
	}else if(turn_velocity >= 0.6 * TURN_VEL_MAX) return 0.6 * TURN_VEL_MAX;
	else return turn_velocity;
}

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
	printf("starting motion capture thread... \n");
	pthread_t  motion_capture_thread;
	rc_pthread_create(&motion_capture_thread, motion_capture_loop, (void*) NULL, SCHED_FIFO, 50);

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

	rc_nanosleep(5E9); // wait for imu to stabilize

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
	mb_state.phi = 0.5*(mb_state.left_encoder + mb_state.right_encoder)/(GEAR_RATIO*ENCODER_RES)*(2*PI);
	
    // Update odometry 
 	mb_odometry_update(&mb_odometry, &mb_state);

	// Task mode Operation
	if(dsm_ch7 == -1 && dsm_ch5 == 0){
		/* Task mode - Default */
		mb_setpoints.manual_ctl = 0;
		init_switch = 1;
		left_turn_task = ROB_FORWARD;
		left_turn_count = 0;
	}else if(dsm_ch7 == -1 && dsm_ch5 == -1){
		/* Task mode - Straight Line Drag Racing */
		mb_setpoints.manual_ctl = 0;
		if(init_switch) mb_odometry_copy(&tmp_odometry, &mb_odometry);
		init_switch = 0;
		distance = mb_odometry_distance(&mb_odometry, &tmp_odometry);
		if(distance > STRAIGHT_DISTANCE) mb_setpoints.fwd_velocity = 0;
		else if(distance > 0.90*STRAIGHT_DISTANCE) mb_setpoints.fwd_velocity = FWD_VEL_MAX * 0.5;
		else if(distance > 0.70*STRAIGHT_DISTANCE) mb_setpoints.fwd_velocity = FWD_VEL_MAX * 0.7;
		else if(distance > 0.30*STRAIGHT_DISTANCE) mb_setpoints.fwd_velocity = FWD_VEL_MAX * 0.8;
		else if(distance > 0.10*STRAIGHT_DISTANCE) mb_setpoints.fwd_velocity = FWD_VEL_MAX * 0.7;
		else mb_setpoints.fwd_velocity = FWD_VEL_MAX * 0.5;
	}else if(dsm_ch7 == -1 && dsm_ch5 == 1){
		/* Task mode - 4 Left Turns */
		mb_setpoints.manual_ctl = 0;
		if(init_switch)	mb_odometry_copy(&tmp_odometry, &mb_odometry);
		init_switch = 0;
		switch (left_turn_task)
		{
			case ROB_FORWARD:
				distance = mb_odometry_distance(&mb_odometry, &tmp_odometry);
				if(left_turn_count == 0) left_turn_distance = 0.85;
				else if (left_turn_count == 3) left_turn_distance = 1.0;
				else left_turn_distance = 0.80;
				if(distance >= left_turn_distance){
					mb_setpoints.fwd_velocity = FWD_VEL_MAX * 0.3;
					left_turn_count ++;
					if(left_turn_count >= 4){
						left_turn_task = ROB_STOP;
						left_turn_count = 0;
					} 
					else left_turn_task = ROB_TURN;
				}else mb_setpoints.fwd_velocity = FWD_VEL_MAX * 0.5;
				break;
			case ROB_TURN:
				if(mb_state.psi_r - tmp_odometry.psi >= PI/2){
					mb_state.psi_r = tmp_odometry.psi + PI/2;
					mb_setpoints.turn_velocity = 0;
					init_switch = 1;
					left_turn_task = ROB_FORWARD;
				}else mb_setpoints.turn_velocity = TURN_VEL_MAX * 0.70;
				break;
			case ROB_STOP:
				mb_setpoints.fwd_velocity = 0;
				mb_setpoints.turn_velocity = 0;
				break;
		}
	}else if(dsm_ch7 == 0 && dsm_ch5 == 1){
		/* Task mode - Optitrak */
		mb_setpoints.manual_ctl = 0;
		if(init_switch){
			/* initialize odometry to world coordinates 
			   and generate all the waypoints */
			mb_odometry_init(&mb_odometry, 0.0, 0.0, 0.0); //Initialize to Optitrak coordinate
			mb_state.psi_old = 0;
			copy_switch = 1;
			cnt_num_odom = 0;
			mb_odometry_init(&array_odometry[0], 0.5, 0.0, 0.0);
			mb_odometry_init(&array_odometry[1], 0.5, 0.5, 0.0);
			mb_odometry_init(&array_odometry[2], 0.0, 0.5, 0.0);
		}
		if(copy_switch){
			init_phi = mb_state.phi;
			init_psi = mb_odometry.psi;
			target_psi = atan2(array_odometry[cnt_num_odom].y-mb_odometry.y, array_odometry[cnt_num_odom].x-mb_odometry.x);
			target_distance = mb_odometry_distance(&array_odometry[cnt_num_odom], &mb_odometry);
		} 
		init_switch = 0;
		copy_switch = 0;
		switch (optitrak_task)
		{
			case RTR_R1:
				mb_setpoints.turn_velocity = turn_velocity_control(&mb_state, init_psi, target_psi);
				if(mb_setpoints.turn_velocity == 0){
					optitrak_task = RTR_T;
					copy_switch = 1;
				}
				break;
			case RTR_T:
				mb_state.forward_velocity = forward_velocity_control(&mb_state, init_phi, target_distance);
				if(mb_state.forward_velocity == 0){
					optitrak_task = RTR_R2;
					copy_switch = 1;
				}
				break;
			case RTR_R2:
				mb_setpoints.turn_velocity = turn_velocity_control(&mb_state, init_psi, array_odometry[cnt_num_odom].psi);
				if(mb_setpoints.turn_velocity == 0){
					optitrak_task = RTR_R1;
					cnt_num_odom ++;
					copy_switch = 1;
				}
				if(cnt_num_odom > num_odometry-1) optitrak_task = RTR_STOP; 
				break;
			case RTR_STOP:
				break;
		}


	}
	// Update command velocities
    float vel = mb_setpoints.fwd_velocity; // (rad/sec)
    float avel = mb_setpoints.turn_velocity; // (m/sec)
	if(mb_setpoints.manual_ctl == 0 || mb_setpoints.manual_ctl == 1){
		mb_state.phi_r = mb_state.phi_old + vel*DT/(0.5*WHEEL_DIAMETER);
		mb_state.phi_old = mb_state.phi_r;

		mb_state.psi_r = mb_state.psi_old + avel*DT;
		mb_state.psi_old = mb_state.psi_r;
	}
       
	if(mb_setpoints.manual_ctl == 0 || mb_setpoints.manual_ctl == 1){
		// calculate controller outputs 
    	mb_controller_update(&mb_state, &mb_setpoints, &mb_odometry, &rob_data);
		// send motor commands
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
				init_switch = 1;
				mb_setpoints.fwd_velocity = 0;
				mb_setpoints.turn_velocity = 0;
			}else if(dsm_ch7 == 0 && dsm_ch5 == -1){
				/* Auto mode - initialize */
				position_init();
			}else if(dsm_ch7 == 1 && dsm_ch5 == 0){
				/* Manual mode - Default */
				mb_setpoints.manual_ctl = 1;
				mb_setpoints.fwd_velocity = FWD_VEL_MAX * rc_dsm_ch_normalized(3);
				mb_setpoints.turn_velocity = TURN_VEL_MAX * rc_dsm_ch_normalized(4);
				rob_data.body_angle = 0.1 * rc_dsm_ch_normalized(1);
				// mb_setpoints.psi_r = PI/2 * rc_dsm_ch_normalized(4);
			}else if(dsm_ch7 == 1 && dsm_ch5 == -1){
				/* Manual mode - Tune Outer Loop */
				mb_setpoints.manual_ctl = 1;
				rob_data.kp2 += 0.001 * rc_dsm_ch_normalized(2);
				rob_data.ki2 += 0.0001 * rc_dsm_ch_normalized(4);
				rob_data.kd2 += 0.0001* rc_dsm_ch_normalized(3);
				//rob_data.kp3 += 0.01 * rc_dsm_ch_normalized(2);
				//rob_data.ki3 += 0.001 * rc_dsm_ch_normalized(4);
				//rob_data.kd3 += 0.001* rc_dsm_ch_normalized(3);
			}else if(dsm_ch7 == 1 && dsm_ch5 == 1){
				/* Manual mode - Tune Inner Loop */
				mb_setpoints.manual_ctl = 1;
				rob_data.kp1 += 0.1 * rc_dsm_ch_normalized(2);
				rob_data.ki1 += 0.01 * rc_dsm_ch_normalized(4);
				rob_data.kd1 += 0.001* rc_dsm_ch_normalized(3);
			}
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
			printf("%7.4f  |", rob_data.kp2);
			printf("%7.4f  |", rob_data.ki2);
			printf("%7.4f  |", rob_data.kd2);
			printf("%7.3f  |", mb_setpoints.fwd_velocity);
			printf("%7.4f  |", mb_state.yaw);
			printf("%7.4f  |", mb_state.psi_r);
			printf("%7.4f  |", (mb_state.left_cmd-mb_state.right_cmd)/2);
			printf("%7.4f  |", rob_data.body_angle);

			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 

void* motion_capture_loop(void* ptr){
	//open serial port non-blocking
    fd = serial_open(port,baudRate,0);

    if(fd == -1){
        printf("Failed to open Serial Port: %s", port);
        return -1;
    }

    //construct message for storage
    BBmsg.pose = BBpose;
    BBmsg.num_gates = num_gates;
    BBmsg.gates = BBgates;
    int packetLength = balancebot_msg_t_encoded_size(&BBmsg)+2;
    while(1)
    {
        //check bytes in serial buffer
        ioctl(fd, FIONREAD, &bytes_avail);
        //printf("bytes: %d\n",bytes_avail);
		
		pthread_mutex_lock(&state_mutex);
        if(bytes_avail >= packetLength) getData(&BBmsg);
		pthread_mutex_unlock(&state_mutex);
		
		rc_nanosleep(1E9/MOTION_CAP_HZ);
    }
    serial_close(fd);
	return NULL;
}