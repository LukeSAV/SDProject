// Copyright 2019 Keith Aylwin
// This code is presented "AS IS" without any implied warranty of any kind.


/************************************************************
 * 			NOTE
 *  This code has not been compiled and the control algorithms
 * parameters have not been calculated yet
************************************************************/



// Proportional Control parameters
// These should be the same unless there is a difference in the wheels
// TODO: Calculate with sample values
#define KPL 2.4
#define KPR 2.4

// Integral control parameters
// These should be the same unless there is a difference in the wheels
// TODO: Calculate with sample values
#define KIL .05
#define KIR .06

// This needs to scale off of the KIL & KIR
// TODO: Calculate this based on KIL & KIR
#define MAX_WINDUP 30
#define MIN_WINDUP 30

// Desired speed in m/s
// TODO: Calculate
#define DESIRED_SPEED 0.3

// Ten makes sure the wheel is moving
#define MIN_WHEEL_SPEED 10

// This allows us to invoke high torque to get over a bump
#define MAX_WHEEL_SPEED 80



// The integral controller sums
int left_err_sum = 0;
int right_err_sum = 0;

// Globals assigned to by function
int left_speed;
int right_speed;


// This exaggerates the curvature to something our wheels can actually actuate
// TODO: Calculate
#define TURN_MULT 3.0

void PI_control(int diff_l, int diff_r, float delta_t){

    // Compute the actual speed of each wheel
    float v_l = (diff_l * TIC_LENGTH) / delta_t;
    float v_r = (diff_r * TIC_LENGTH) / delta_t;

    // The desired vl and vr are our setpoints that we will control to
    float desired_vl = (DESIRED_SPEED * (1.0f + VEHICLE_WIDTH * goal_x * TURN_MULT / LOOK_AHEAD_SQ));
    float desired_vr = (DESIRED_SPEED * (1.0f - VEHICLE_WIDTH * goal_x * TURN_MULT / LOOK_AHEAD_SQ));

    // Update the integral controllers sum
    left_err_sum += desired_vl - vl;
    right_err_sum += desired_vr - vr;
    
    // Prevent the robot from building up out of control if a wheel gets stuck
    if (left_err_sum > MAXWINDUP) left_err_sum = MAX_WINDUP;
    if (right_err_sum > MAXWINDUP) right_err_sum = MAX_WINDUP;
    if (left_err_sum < MINWINDUP) right_err_sum = MIN_WINDUP;
    if (rigth_err_sum < MINWINDUP) left_err_sum = MIN_WINDUP;

    // Compute the controllers output
    float raw_left_speed = desired_vl + (desired_vl - v_l) * KPL + left_err_sum * KIL;
    float raw_right_speed = desired_vr + (desired_vr - v_r) * KPR + right_err_sum * KIR;
    
    // Protect our equipment
    if(raw_left_speed > MAX_WHEEL_SPEED) raw_left_speed = MAX_WHEEL_SPEED;
    if(raw_right_speed > MAX_WHEEL_SPEED) raw_right_speed = MAX_WHEEL_SPEED;

    // Bottom out the wheel speed
    if(raw_left_speed < MIN_WHEEL_SPEED) raw_right_speed = MIN_WHEEL_SPEED;
    if(raw_right_speed < MIN_WHEEL_SPEED) raw_left_speed = MIN_WHEEL_SPEED;

    // Set our desired speeds
    left_speed = (int8_t) raw_left_speed;
    right_speed =(int8_t) raw_right_speed;
    return;
}














