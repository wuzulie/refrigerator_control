#ifndef __PID_H_
#define __PID_H_

#include "stdbool.h"
#define DEFAULT_PID_INTEGRATION_LIMIT  1000.0f

typedef struct
{
	float setpoint;      
	float feedback;
	float error_now;       
	float error_pre;    //error[-1]
	float error_last;   //error[-2]   
	float error_int;       
	float kp;           
	float ki;           
	float kd;           
	float output;
	float int_limit;     
	float dt;     
    float last_derivative;	
	float d_lpf_alpha;
} PID_Controller;

void PID_Init(PID_Controller *PID,float P,float I,float D);
float PID_Calc_Inc(PID_Controller *PID,float sp,float feedback,float dt);
float PID_Calc_Loc(PID_Controller *PID,float sp,float feedback,float dt);
float PID_Calc_Loc_Error(PID_Controller *PID,float error,float dt);
float get_i( PID_Controller* pid ,float error, float dt);
float get_d( PID_Controller* pid ,float error, float dt);
float rate_to_motor_roll(PID_Controller *PID, float rate_target, float feedback,float dt,bool limit, bool thr_low);
float rate_to_motor_pitch(PID_Controller *PID, float rate_target, float feedback,float dt,bool limit, bool thr_low);
float rate_to_motor_yaw(PID_Controller *PID, float rate_target, float feedback,float dt,bool limit, bool thr_low);
float acc_to_motor_thr(PID_Controller *PID, float accel_error, float dt, bool limit_up, bool limit_down);
void PID_SetIntegLimit(PID_Controller* PID, const float limit);
void PID_SetParam_P(PID_Controller* PID, float p);
void PID_SetParam_I(PID_Controller* PID, float i);
void PID_SetParam_D(PID_Controller* PID, float d);
void PID_Reset(PID_Controller *PID);
#endif
