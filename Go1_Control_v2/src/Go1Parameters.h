//
// Created by Amir 02/23
// Ref. ShuoYang Github
//

#ifndef GO1_PARAMETERS_H
#define GO1_PARAMETERS_H

// control time related
//#define CTRL_FREQUENCY 2.5  // ms
#define GRF_UPDATE_FREQUENCY 2.0 // 2.5 // ms
#define MAIN_UPDATE_FREQUENCY 2.0 //2.5 // ms
#define HARDWARE_FEEDBACK_FREQUENCY 2.0  // ms

// constant define
// joy stick command interprate
#define JOY_CMD_BODY_HEIGHT_MAX 0.32     // m
#define JOY_CMD_BODY_HEIGHT_MIN 0.1     // m
#define JOY_CMD_BODY_HEIGHT_VEL 0.04    // m/s
#define JOY_CMD_VELX_MAX 0.6         // m/s
#define JOY_CMD_VELY_MAX 0.3            // m/s
#define JOY_CMD_YAW_MAX 0.8             // rad 
#define JOY_CMD_YAW_RATE_MAX 0.2			//rad/s
#define JOY_CMD_PITCH_MAX 0.4           // rad //rad/s
#define JOY_CMD_ROLL_MAX 0.4            // rad //

// 
//Constant defined for keyboard control
#define STEP_VELX 0.01 //m/s
#define STEP_VELY 0.005 //m/s
#define STEP_VEL_BODY_HEIGHT 0.005 //m/s
#define STEP_YAW_RATE 0.01 //rad/S
#define STEP_BODY_HEIGHT 0.01 //m
#define STEP_COUNTER_PER_SWING 5 //m/s
#define COUNTER_PER_SWING_MIN 75 // 0.15ms
#define COUNTER_PER_SWING_MAX 200 // 0.4ms


/////////////////////////

//
#define GAIN_SIZE 2 //HLIP_Planner GainSize
// mpc
#define PLAN_HORIZON 10
#define MPC_STATE_DIM 13
#define MPC_CONSTRAINT_DIM 20

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

#define LOWER_LEG_LENGTH 0.21

#define FOOT_FORCE_LOW 1.0
#define FOOT_FORCE_HIGH 180.0

// RockyDRS
// #define FOOT_SWING_CLEARANCE1 0.15f
// #define FOOT_SWING_CLEARANCE2 0.22f

// Used for Case 1 to Case4
#define FOOT_SWING_CLEARANCE1 0.02f
#define FOOT_SWING_CLEARANCE2 0.15f


#define FOOT_DELTA_X_LIMIT 0.20
#define FOOT_DELTA_Y_LIMIT 0.10

#define ERROR_CURVE_ALREADY_SET 184
#define ERROR_CURVE_NOT_SET 185

#endif //GO1_PARAMETERS_H
