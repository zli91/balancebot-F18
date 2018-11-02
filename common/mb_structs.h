#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;

};

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float theta;             // body angle (rad)
    float phi;               // average wheel angle (rad)
    int left_encoder;      // left encoder counts since last reading
    int right_encoder;     // right encoder counts since last reading

    //outputs
    float left_cmd;  //left wheel command [-1..1]
    float right_cmd; //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed
    float phi_old;
    float phi_r;

    float psi_old;
    float psi_r;

    int prev_left_encoder;      
    int prev_right_encoder;     
};


typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{
    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
};

typedef struct rob_data rob_data_t;
struct rob_data{
    double kp1;
    double ki1;
    double kd1;

    double kp2;
    double ki2;
    double kd2;
    
    double kp3;
    double ki3;
    double kd3;

    double body_angle;
};



#endif
