#include "pid.h"
#include "hg_math.hpp"
#include "hg_defines.h"
#include <math.h>

void PID_Init(PID_Controller *PID,float P,float I,float D)
{
    PID->dt = 0.0f;
    PID->error_int = 0.0f;
    PID->error_now = 0.0f;
    PID->error_pre = 0.0f;
    PID->error_last = 0.0f;
    PID->feedback = 0.0f;
    PID->int_limit = 1000.0f;
    PID->kp = P;
    PID->ki = I;
    PID->kd = D;
    PID->output = 0;
    PID->setpoint = 0;
    PID->last_derivative = NAN;
}

float PID_Calc_Inc(PID_Controller *PID,float sp,float feedback,float dt)
{
    register float Incpid;

    PID->feedback = feedback;
    PID->setpoint = sp;
    PID->error_now = PID->setpoint - feedback;

    PID->dt = dt;
    if(PID->dt == 0.0f)
    {
        PID->error_last = PID->error_pre;
        PID->error_pre = PID->error_now;
    }
    else
    {
        Incpid = PID->kp * (PID->error_now - PID->error_pre);
        Incpid = Incpid + PID->ki * PID->error_now * PID->dt;
        Incpid = Incpid + PID->kd * (PID->error_now - 2 * PID->error_pre + PID->error_last) / PID->dt;

        PID->output += Incpid;

        PID->error_last = PID->error_pre;
        PID->error_pre = PID->error_now;
    }
    return PID->output;
}

float PID_Calc_Loc(PID_Controller *PID,float sp,float feedback,float dt)
{
    PID->setpoint = sp;

    PID->error_now = PID->setpoint - feedback;
    PID->dt = dt;
    PID->error_int += PID->error_now * PID->dt;
    if (PID->error_int > PID->int_limit)
    {
        PID->error_int = PID->int_limit;
    }
    else if (PID->error_int < -(PID->int_limit))
    {
        PID->error_int = -(PID->int_limit);
    }
    if(PID->dt == 0.0f)
    {
        PID->error_pre = PID->error_now;
        PID->error_last = PID->error_pre;
        PID->output = 0;

    }
    else
    {
        PID->output = PID->kp * PID->error_now + PID->ki * PID->error_int + PID->kd * ((PID->error_now - PID->error_pre) / PID->dt);

        PID->error_pre = PID->error_now;
        PID->error_last = PID->error_pre;
    }
    return PID->output;
}

float PID_Calc_Loc_Error(PID_Controller *PID,float error,float dt)
{
    PID->error_now = error;
    PID->dt = dt;
    PID->error_int += PID->error_now * PID->dt;
    if (PID->error_int > PID->int_limit)
    {
        PID->error_int = PID->int_limit;
    }
    else if (PID->error_int < -(PID->int_limit))
    {
        PID->error_int = -(PID->int_limit);
    }
    if(PID->dt == 0.0f)
    {
        PID->error_pre = PID->error_now;
        PID->error_last = PID->error_pre;
        PID->output = 0;

    }
    else
    {
        PID->output = PID->kp * PID->error_now + PID->ki * PID->error_int + PID->kd * ((PID->error_now - PID->error_pre) / PID->dt);

        PID->error_pre = PID->error_now;
        PID->error_last = PID->error_pre;
    }
    return PID->output;
}


float get_i( PID_Controller* pid ,float error, float dt)
{
    if((pid->ki != 0) && (dt != 0))
    {
        if(isfinite(error))
        {
            pid->error_int += ((float)error * pid->ki) * dt;
        }

        if (pid->error_int < -pid->int_limit)
        {
            pid->error_int = -pid->int_limit;
        }
        else if (pid->error_int > pid->int_limit)
        {
            pid->error_int = pid->int_limit;
        }

        if(!isfinite(pid->error_int))
        {
            pid->error_int = 0;
        }
        return pid->error_int;
    }
    return 0;
}


float get_d( PID_Controller* pid , float error, float dt)
{
    float derivative;
    if ((pid->kd != 0) && (dt != 0))
    {

        if (isfinite(pid->last_derivative))
        {
            // calculate instantaneous derivative
            derivative = (error - pid->error_last) / dt;
        }
        else
        {
            // we've just done a reset, suppress the first derivative
            // term as we don't want a sudden change in input to cause
            // a large D output change
            derivative = 0;
            pid->last_derivative = 0;
        }

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        derivative = pid->last_derivative + pid->d_lpf_alpha * (derivative - pid->last_derivative);

        // update state
        pid->error_last             = error;
        pid->last_derivative    = derivative;

        // add in derivative component
        return pid->kd * derivative;
    }
    return 0;
}


float rate_to_motor_roll(PID_Controller *PID, float rate_target, float feedback, float dt, bool limit, bool thr_low)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    current_rate = feedback ;

    // calculate error and call pid controller
    rate_error = rate_target - current_rate;
    p = rate_error * PID->kp;

    // get i term
    i = PID->error_int;

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if ((!limit && !thr_low) || ((i>0&&rate_error<0)||(i<0&&rate_error>0)))
    {
        i = get_i(PID ,rate_error, dt);
    }

    // get d term
    d = get_d(PID,rate_error, dt);

    return constrain_float((p+i+d), -1.0f, 1.0f);

}

float rate_to_motor_pitch(PID_Controller *PID, float rate_target, float feedback, float dt, bool limit, bool thr_low)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    current_rate = feedback ;

    // calculate error and call pid controller
    rate_error = rate_target - current_rate;
    p = rate_error * PID->kp;

    // get i term
    i = PID->error_int;

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if ((!limit && !thr_low) || ((i>0&&rate_error<0)||(i<0&&rate_error>0)))
    {
        i = get_i(PID ,rate_error, dt);
    }
    // get d term
    d = get_d(PID,rate_error, dt);

    return constrain_float((p+i+d), -1.0f, 1.0f);

}

float rate_to_motor_yaw(PID_Controller *PID, float rate_target, float feedback, float dt, bool limit, bool thr_low)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    current_rate = feedback ;

    // calculate error and call pid controller
    rate_error = rate_target - current_rate;
    p = rate_error * PID->kp;

    // get i term
    i = PID->error_int;

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if ((!limit && !thr_low) || ((i>0&&rate_error<0)||(i<0&&rate_error>0)))
    {
        i = get_i(PID ,rate_error, dt);
    }

    // get d term
    d = get_d(PID,rate_error, dt);

    return constrain_float((p+i+d), -1.0f, 1.0f);

}

float acc_to_motor_thr(PID_Controller *PID, float accel_error, float dt, bool limit_up, bool limit_down)
{
    float p,i,d;            // used to capture pid values for logging

    // separately calculate p, i, d values for logging
    p = accel_error * PID->kp;

    // get i term
    i = PID->error_int;

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    // To-Do: should this be replaced with limits check from attitude_controller?
    if ((!limit_down && !limit_up) || (i>0&&accel_error<0) || (i<0&&accel_error>0))
    {
        i = get_i(PID ,accel_error, dt);
    }

    // get d term
    d = get_d(PID,accel_error, dt);

    PID->output = p + i + d;
    return PID->output;
}

void PID_SetIntegLimit(PID_Controller* PID, const float limit)
{
    PID->int_limit = limit;
}
void PID_SetParam_P(PID_Controller* PID, float p)
{
    PID->kp = p;
}
void PID_SetParam_I(PID_Controller* PID, float i)
{
    PID->ki = i;
}
void PID_SetParam_D(PID_Controller* PID, float d)
{
    PID->kd = d;
}
void PID_Reset(PID_Controller *PID)
{
    PID->dt = 0.0f;
    PID->error_int = 0.0f;
    PID->error_now = 0.0f;
    PID->error_pre = 0.0f;
    PID->error_last = 0.0f;
    PID->last_derivative = NAN;
    PID->feedback = 0.0f;
    PID->output = 0;
    PID->setpoint = 0;
}
