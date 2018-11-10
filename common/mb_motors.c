/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/
#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init(){
    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz){
    // motor GPIO init
    init_flag = 1;
    rc_gpio_init(MDIR1_CHIP, MDIR1_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(MDIR2_CHIP, MDIR2_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(MOT_BRAKE_EN, GPIOHANDLE_REQUEST_OUTPUT);
    // ADC init
    rc_adc_init();
    // PWM init
    rc_pwm_init(1,pwm_freq_hz);
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }
    rc_pwm_cleanup(1);
    rc_gpio_cleanup(MDIR1_CHIP, MDIR1_PIN);
    rc_gpio_cleanup(MDIR2_CHIP, MDIR2_PIN);
    rc_adc_cleanup();
    return 0;
}

/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }
    rc_gpio_set_value(MOT_BRAKE_EN,brake_en);
    if(brake_en == 1){
        rc_pwm_set_duty(1,'A',0.0);
        rc_pwm_set_duty(1,'B',0.0);
    }
    return 0; 
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signalsmotor
* returns 0 on success
*******************************************************************************/
int mb_motor_disable(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }
    rc_pwm_set_duty(1,'A',0.0);
    rc_pwm_set_duty(1,'B',0.0);
    return 0;
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    int dir = 1;
    if(duty < 0){
        duty = duty * -1.0;
        dir = 0;
    }
    switch(motor){
        case RIGHT_MOTOR:
            rc_pwm_set_duty(1,'A',duty);
            rc_gpio_set_value(MDIR1_CHIP, MDIR1_PIN, dir);
            break;
        case LEFT_MOTOR:
            if(dir == 1) dir = 0;
            else dir = 1;
            rc_pwm_set_duty(1,'B',duty);
            rc_gpio_set_value(MDIR2_CHIP, MDIR2_PIN, dir);
            break;
    }
    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty){

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    mb_motor_set(RIGHT_MOTOR, duty);
    mb_motor_set(LEFT_MOTOR, duty);
    return 0;
}


/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor){
    //DRV8801 driver board CS pin puts out 500mV/A
    switch(motor){
        case RIGHT_MOTOR:
            return rc_adc_read_volt(MOT_1_CS)/0.5;
        case LEFT_MOTOR:
            return rc_adc_read_volt(MOT_2_CS)/0.5;
    }
    return -1;
}

