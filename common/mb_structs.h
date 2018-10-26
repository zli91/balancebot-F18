#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
    float phi_old;
    float phi_r;
};

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   phi;               // average wheel angle (rad)
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed
    /* outer loop */
    // error from previous states
    float e_phi;
    float eo1;
    float eo2;

    // input from previous states
    float uo1;
    float uo2;

    /* inner loop */
    // error from previous states
    float e_theta;
    float ei1;
    float ei2;

    // input from previous states
    float ui1;
    float ui2;

    // include setpoint here at state
    mb_setpoints_t* setpoint;
};


typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
};

#endif
