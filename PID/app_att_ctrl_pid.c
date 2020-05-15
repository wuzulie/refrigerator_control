#include "app_att_ctrl_pid.h"
#include "uorb_object.h"
#include "drivers.h"
#include "finsh.h"
#include "param.h"
#include "pid.h"
#include "hrt.h"
#include "hg_defines.h"
#include "hg_math.hpp"
#include <math.h>
#include "app_navigator.h"
#include "flight_check.h"
#include "param_default.h"

bool run_att_ctrl_pid = true;
bool att_ctrl_pid_running = false;

static att_ctrl_params_s att_ctrl_params;

ORB_DEFINE(actuator_control, actuator_control_t);
ORB_DEFINE(vehicle_att_modifier, vehicle_att_modifier_t);

static void matrix3f_mul_matrix3f(float matrix3f_1[3][3], float matrix3f_2[3][3], float matrix3f_out[3][3]);
static void vector_cross_product(float vector_1[3], float vector_2[3], float vector_out[3]);
static float vector3f_dot_product(float vector_1[3], float vector_2[3]);
static void quaternion_to_dcm(float q_f[4], float matrix3f_out[3][3]);
static void quaternion_from_dcm(float dcm[3][3], float q_out[4]);

void att_ctrl_pid(char* cmd)
{
    rt_thread_t tid = RT_NULL;
    if (strcmp(cmd, "start") == 0)
    {

        if (att_ctrl_pid_running == true)
        {
            rt_kprintf("att contorller is already started\r\n");
        }
        else
        {
            run_att_ctrl_pid = true;
            tid = rt_thread_create("att ctrl",
                                   app_att_ctrl_pid_main, RT_NULL,
                                   4096, 8, 10);
            if (tid != RT_NULL)
                rt_thread_startup(tid);
        }
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        if (att_ctrl_pid_running == false)
        {
            rt_kprintf("att contorller is already stopped\r\n");
        }
        else
        {
            run_att_ctrl_pid = false;
        }
    }
    else
    {
        rt_kprintf("%s command not found\r\n", cmd);
        rt_kprintf("usage:connector {start|stop}\r\n");
    }
}

static void att_ctrl_parameters_update()
{
    param_get("MC_ROLL_P", &att_ctrl_params.att_p[0]);
    param_get("MC_ROLL_I", &att_ctrl_params.att_i[0]);
    param_get("MC_ROLL_D", &att_ctrl_params.att_d[0]);
    param_get("MC_PITCH_P", &att_ctrl_params.att_p[1]);
    param_get("MC_PITCH_I", &att_ctrl_params.att_i[1]);
    param_get("MC_PITCH_D", &att_ctrl_params.att_d[1]);
    param_get("MC_YAW_P", &att_ctrl_params.att_p[2]);
    param_get("MC_YAW_I", &att_ctrl_params.att_i[2]);
    param_get("MC_YAW_D", &att_ctrl_params.att_d[2]);
    param_get("MC_ROLLRATE_P", &att_ctrl_params.att_rate_p[0]);
    param_get("MC_ROLLRATE_I", &att_ctrl_params.att_rate_i[0]);
    param_get("MC_ROLLRATE_D", &att_ctrl_params.att_rate_d[0]);
    param_get("MC_PITCHRATE_P", &att_ctrl_params.att_rate_p[1]);
    param_get("MC_PITCHRATE_I", &att_ctrl_params.att_rate_i[1]);
    param_get("MC_PITCHRATE_D", &att_ctrl_params.att_rate_d[1]);
    param_get("MC_YAWRATE_P", &att_ctrl_params.att_rate_p[2]);
    param_get("MC_YAWRATE_I", &att_ctrl_params.att_rate_i[2]);
    param_get("MC_YAWRATE_D", &att_ctrl_params.att_rate_d[2]);  
    
    param_get("MPC_PM_RP_RATE", &att_ctrl_params.pmode_rate_max[0]);
    param_get("MPC_PM_RP_RATE", &att_ctrl_params.pmode_rate_max[1]);
    param_get("MPC_AM_RP_RATE", &att_ctrl_params.amode_rate_max[0]);
    param_get("MPC_AM_RP_RATE", &att_ctrl_params.amode_rate_max[1]);
    param_get("MPC_SM_RP_RATE", &att_ctrl_params.smode_rate_max[0]);
    param_get("MPC_SM_RP_RATE", &att_ctrl_params.smode_rate_max[1]);
    
    att_ctrl_params.yaw_ff = 0.5f;

    att_ctrl_params.pmode_rate_max[0] = radians(att_ctrl_params.pmode_rate_max[0]);
    att_ctrl_params.pmode_rate_max[1] = radians(att_ctrl_params.pmode_rate_max[1]);
    att_ctrl_params.pmode_rate_max[2] = radians(90.0f);    
    att_ctrl_params.amode_rate_max[0] = radians(att_ctrl_params.amode_rate_max[0]);
    att_ctrl_params.amode_rate_max[1] = radians(att_ctrl_params.amode_rate_max[1]);
    att_ctrl_params.amode_rate_max[2] = radians(120.0f);    
    att_ctrl_params.smode_rate_max[0] = radians(att_ctrl_params.smode_rate_max[0]);
    att_ctrl_params.smode_rate_max[1] = radians(att_ctrl_params.smode_rate_max[1]);
    att_ctrl_params.smode_rate_max[2] = radians(120.0f);
    
    att_ctrl_params.auto_rate_max[0] = radians(120.0f);
    att_ctrl_params.auto_rate_max[1] = radians(120.0f);
    att_ctrl_params.auto_rate_max[2] = radians(120.0f);
}

void ImuTempControl(PID_Controller *PID, float imutemp)
{
	static hrt_abstime last_us = 0;   
	int32_t error = 0; 
    static float PIDTerm = 0;
    float dt = (hrt_absolute_time() - last_us) * 1e-6;
	last_us = hrt_absolute_time();
	static uint8_t heat_status = 0;
    float p,i,d;
	static bool _init = false;
	static int imu_temp_set = DEFAULT_IMU_TEMP;
	
	if (!_init)
	{
		_init = true;
		
		if( param_get( "IMU_TEMP", &imu_temp_set) != RT_EOK )
		{
			imu_temp_set = DEFAULT_IMU_TEMP;
		}
		
		mavlink_sensor_present_set(MAV_SYS_STATUS_IMU_HEATING);
		mavlink_sensor_enabled_set(MAV_SYS_STATUS_IMU_HEATING);
	}

	error = imu_temp_set * 100 - imutemp * 100;	  
	
//	if (imutemp > imu_temp_set -2)
//	{
//		mavlink_sensor_present_set(MAV_SYS_STATUS_IMU_HEATING);
//		mavlink_sensor_health_set(MAV_SYS_STATUS_IMU_HEATING);
//	}
//	else
//    {
////        mavlink_sensor_present_set(MAV_SYS_STATUS_IMU_HEATING);
////		mavlink_sensor_health_set(MAV_SYS_STATUS_IMU_HEATING);
//		mavlink_sensor_health_reset(MAV_SYS_STATUS_IMU_HEATING);
//    }
    mavlink_sensor_present_set(MAV_SYS_STATUS_IMU_HEATING);
	mavlink_sensor_health_set(MAV_SYS_STATUS_IMU_HEATING);
	
	if(imutemp < imu_temp_set && !heat_status)
	{
		temppwmset(1000);
		return;
	}
	else
	{
		heat_status = 1;
	}
	
	if(imutemp > imu_temp_set + 1)
	{
		temppwmset(0);
		PID->error_int = 0;
	}
		
	// get p term
    p = error * PID->kp;

    // get i term
    i = get_i(PID, error, dt);

    // get d term
    d = get_d(PID, error, dt);

	PIDTerm = constrain_float((p+i+d), 0, 1000.0f);
	temppwmset((int16_t)PIDTerm);
}

extern Navigator_t navigator;
void app_att_ctrl_pid_main(void* parameter)
{
    uint32_t    ctrl_state_sub = 0;
    uint32_t    att_sp_sub = 0;    
    uint32_t    control_mode_sub = 0; 
    uint32_t    status_sub = 0;    
    uint32_t    motor_sub = 0;    
    uint32_t    param_sub = 0;    
	uint32_t    imu_temperature_sub = 0;  
    
    control_state_t                 ctrl_state = {0};   
    vehicle_attitude_setpoint_t     att_sp = {0};    
    vehicle_control_mode_t          control_mode = {0};    
    vehicle_status_t                status = {0};
    vehicle_motor_pwm_t             motor = {0};
        
    vehicle_att_modifier_t          att_mod = {0};
    actuator_control_t              output = {0};

    float rates_sp[3] = {0};
    float mc_rate_max[3] = {0};
    int16_t thrust_sp = 0;
        
    hrt_abstime last_time = 0;
    hrt_abstime ground_touched_prev_time = 0;
    bool        ground_touched_prev = false;
    float dt = 0;

	imutemperature_raw_t imu_tempe;
	
    PID_Controller roll_rate_ctrler;
    PID_Controller pitch_rate_ctrler;
    PID_Controller yaw_rate_ctrler;
	PID_Controller temp_rate_ctrler;
    
    if (orb_subscribe_auto(ORB_ID(control_state), &ctrl_state_sub, 10) != RT_EOK)
    {
        rt_kprintf("[ATT]can't subscribe ORB_ID(control_state).\n");
    }
    
    if (orb_subscribe_auto(ORB_ID(vehicle_attitude_setpoint), &att_sp_sub, 10) != RT_EOK)
    {
        rt_kprintf("[ATT]can't subscribe ORB_ID(vehicle_attitude_setpoint).\n");
    }

    if (orb_subscribe_auto(ORB_ID(vehicle_control_mode), &control_mode_sub, 10) != RT_EOK)
    {
        rt_kprintf("[ATT]can't subscribe ORB_ID(vehicle_control_mode).\n");
    }

    if (orb_subscribe_auto(ORB_ID(vehicle_status), &status_sub, 10) != RT_EOK)
    {
        rt_kprintf("[ATT]can not subscribe ORB_ID(vehicle_status)\n");
    }
    
    if (orb_subscribe_auto(ORB_ID(vehicle_motor_pwm), &motor_sub, 10) != RT_EOK)
    {
        rt_kprintf("[ATT]can not subscribe ORB_ID(vehicle_motor_pwm)\n");
    }

    if (orb_subscribe_auto(ORB_ID(parameter_update), &param_sub, 10) != RT_EOK)
    {
        rt_kprintf("[ATT]can not subscribe ORB_ID(parameter_update)\n");
    }
	
	if (orb_subscribe_auto(ORB_ID(imutemperature_raw), &imu_temperature_sub, 10) != RT_EOK)
    {
        rt_kprintf("[ATT]can not subscribe ORB_ID(imutemperature_raw)\n");
    }
    
    att_ctrl_parameters_update();

    PID_Init(&roll_rate_ctrler, att_ctrl_params.att_rate_p[0] * 0.1f, att_ctrl_params.att_rate_i[0] * 0.1f, att_ctrl_params.att_rate_d[0] * 0.1f);
    PID_Init(&pitch_rate_ctrler, att_ctrl_params.att_rate_p[1] * 0.1f, att_ctrl_params.att_rate_i[1] * 0.1f, att_ctrl_params.att_rate_d[1] * 0.1f);
    PID_Init(&yaw_rate_ctrler, att_ctrl_params.att_rate_p[2] * 0.1f, att_ctrl_params.att_rate_i[2] * 0.1f, att_ctrl_params.att_rate_d[2] * 0.1f);
	PID_Init(&temp_rate_ctrler, 15.0f, 2.0f, 10.0f);

    PID_SetIntegLimit(&roll_rate_ctrler, 0.3f);
    PID_SetIntegLimit(&pitch_rate_ctrler, 0.3f);
    PID_SetIntegLimit(&yaw_rate_ctrler, 0.4f);
	PID_SetIntegLimit(&temp_rate_ctrler, 500.0f);

    roll_rate_ctrler.d_lpf_alpha = 0.002f / (0.002f + (1 / (2 * M_PI_F * 30.0f)));
    pitch_rate_ctrler.d_lpf_alpha = 0.002f / (0.002f + (1 / (2 * M_PI_F * 30.0f)));
    yaw_rate_ctrler.d_lpf_alpha = 0.002f / (0.002f + (1 / (2 * M_PI_F * 20.0f)));

    
    att_ctrl_pid_running = true;

   
    while (run_att_ctrl_pid)
    {
        if (orb_check(&ctrl_state_sub, RT_WAITING_FOREVER) == RT_EOK)
        {
            orb_copy(ORB_ID(control_state), &ctrl_state);
        }
        
        if (orb_check(&control_mode_sub, 0) == RT_EOK)
        {
            orb_copy(ORB_ID(vehicle_control_mode), &control_mode);
        }  
                
        if (orb_check(&status_sub, 0) == RT_EOK)
        {
            orb_copy(ORB_ID(vehicle_status), &status);
        }  
        
        if (orb_check(&motor_sub, 0) == RT_EOK)
        {
            orb_copy(ORB_ID(vehicle_motor_pwm), &motor);
        }   
        
        if (orb_check(&param_sub, 0) == RT_EOK)
        {
            att_ctrl_parameters_update();
            PID_Init(&roll_rate_ctrler, att_ctrl_params.att_rate_p[0] * 0.1f, att_ctrl_params.att_rate_i[0] * 0.1f, att_ctrl_params.att_rate_d[0] * 0.1f);
            PID_Init(&pitch_rate_ctrler, att_ctrl_params.att_rate_p[1] * 0.1f, att_ctrl_params.att_rate_i[1] * 0.1f, att_ctrl_params.att_rate_d[1] * 0.1f);
            PID_Init(&yaw_rate_ctrler, att_ctrl_params.att_rate_p[2] * 0.1f, att_ctrl_params.att_rate_i[2] * 0.1f, att_ctrl_params.att_rate_d[2] * 0.1f);

            PID_SetIntegLimit(&roll_rate_ctrler, 0.3f);
            PID_SetIntegLimit(&pitch_rate_ctrler, 0.3f);
            PID_SetIntegLimit(&yaw_rate_ctrler, 0.3f);         
        }
		
		if (orb_check(&imu_temperature_sub, 0) == RT_EOK)
        {
            orb_copy(ORB_ID(imutemperature_raw), &imu_tempe);
			ImuTempControl(&temp_rate_ctrler, imu_tempe.mpu_tempe);
        }
		else
		{
			// When not check IMU, then cancle the imu controler.
			if (!sensor_check_healthy(MAV_SYS_STATUS_SENSOR_3D_GYRO))
			{
				temppwmset(0);
			}
		}

        hrt_abstime now = hrt_absolute_time();
        dt = last_time > 0 ? (now - last_time) * 0.000001f : 0.0f;
        last_time = now;
        dt = 0.002f;
        
        /* the max rotation rate for different mode */
        if(status.flight_mode == FLIGHT_MODE_POSHOLD)
        {
            mc_rate_max[0] = att_ctrl_params.pmode_rate_max[0];
            mc_rate_max[1] = att_ctrl_params.pmode_rate_max[1];
            mc_rate_max[2] = att_ctrl_params.pmode_rate_max[2];
        }
        else if (status.flight_mode == FLIGHT_MODE_ALTHOLD)
        {
            mc_rate_max[0] = att_ctrl_params.amode_rate_max[0];
            mc_rate_max[1] = att_ctrl_params.amode_rate_max[1];
            mc_rate_max[2] = att_ctrl_params.amode_rate_max[2];
        }
        else if (status.flight_mode == FLIGHT_MODE_SPORT)
        {
            mc_rate_max[0] = att_ctrl_params.smode_rate_max[0];
            mc_rate_max[1] = att_ctrl_params.smode_rate_max[1];
            mc_rate_max[2] = att_ctrl_params.smode_rate_max[2];
        }
        else
        {
            if(fabs(mc_rate_max[0]) < FLT_EPSILON)
                mc_rate_max[0] = att_ctrl_params.amode_rate_max[0];
            if(fabs(mc_rate_max[1]) < FLT_EPSILON)
                mc_rate_max[1] = att_ctrl_params.amode_rate_max[1];
            if(fabs(mc_rate_max[2]) < FLT_EPSILON)
                mc_rate_max[2] = att_ctrl_params.amode_rate_max[2];
        }
        
        if (control_mode.flag_control_attitude_enabled)
        {
            if (orb_check(&att_sp_sub, 0) == RT_EOK)
            {
                orb_copy(ORB_ID(vehicle_attitude_setpoint), &att_sp);
            }
        
            thrust_sp = att_sp.thrust * 1000;

            /* construct attitude setpoint rotation matrix */
            float R_sp[3][3];
            rt_memcpy(R_sp, att_sp.R_body, sizeof(R_sp));
            
            /* get current rotation matrix from control state quaternions */
            float q_att[4] = {ctrl_state.q[0], ctrl_state.q[1], ctrl_state.q[2], ctrl_state.q[3]};
            float R[3][3];
            quaternion_to_dcm(q_att, R);
            
            /* all input data is ready, run controller itself */

            /* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
            float R_z[3] = {R[0][2], R[1][2], R[2][2]};
            float R_sp_z[3] = {R_sp[0][2], R_sp[1][2], R_sp[2][2]};

            /* axis and sin(angle) of desired rotation */
            float R_z_cross_R_sp_z[3];
            vector_cross_product(R_z, R_sp_z, R_z_cross_R_sp_z);
            
            float e_R[3];
            for (int i = 0; i < 3; i++)
            {
                float c = 0.0f;
                for (int j = 0; j < 3; j++)
                {
                  c += R[j][i] * R_z_cross_R_sp_z[j];
                }
                if (isfinite(c))
                {
                    e_R[i] = c;
                }                
            }
        
            /* calculate angle error */
            float e_R_z_sin = sqrtf(e_R[0]*e_R[0] + e_R[1]*e_R[1] + e_R[2]*e_R[2]);
            float e_R_z_cos = vector3f_dot_product(R_z, R_sp_z);

            /* calculate weight for yaw control */
            float yaw_w = R_sp[2][2] * R_sp[2][2];

            /* calculate rotation matrix after roll/pitch only rotation */
            float R_rp[3][3];

            if (e_R_z_sin > 0.0f) 
            {
                /* get axis-angle representation */
                float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
                float e_R_z_axis[3] = {e_R[0]/e_R_z_sin, e_R[1]/e_R_z_sin, e_R[2]/e_R_z_sin};

                e_R[0] = e_R_z_axis[0] * e_R_z_angle;
                e_R[1] = e_R_z_axis[1] * e_R_z_angle;
                e_R[2] = e_R_z_axis[2] * e_R_z_angle;

                /* cross product matrix for e_R_axis */
                float e_R_cp[3][3] = {0};
                e_R_cp[0][1] = -e_R_z_axis[2];
                e_R_cp[0][2] = e_R_z_axis[1];
                e_R_cp[1][0] = e_R_z_axis[2];
                e_R_cp[1][2] = -e_R_z_axis[0];
                e_R_cp[2][0] = -e_R_z_axis[1];
                e_R_cp[2][1] = e_R_z_axis[0];

                /* rotation matrix for roll/pitch only rotation */
                float _I[3][3] = {1,0,0, 0,1,0, 0,0,1};
                float rotation_matrix[3][3], temp_matrix[3][3];
                
                matrix3f_mul_matrix3f(e_R_cp, e_R_cp, temp_matrix);
                rotation_matrix[0][0] = _I[0][0] + e_R_cp[0][0] * e_R_z_sin + temp_matrix[0][0] * (1.0f - e_R_z_cos);
                rotation_matrix[0][1] = _I[0][1] + e_R_cp[0][1] * e_R_z_sin + temp_matrix[0][1] * (1.0f - e_R_z_cos);
                rotation_matrix[0][2] = _I[0][2] + e_R_cp[0][2] * e_R_z_sin + temp_matrix[0][2] * (1.0f - e_R_z_cos);
                
                rotation_matrix[1][0] = _I[1][0] + e_R_cp[1][0] * e_R_z_sin + temp_matrix[1][0] * (1.0f - e_R_z_cos);
                rotation_matrix[1][1] = _I[1][1] + e_R_cp[1][1] * e_R_z_sin + temp_matrix[1][1] * (1.0f - e_R_z_cos);
                rotation_matrix[1][2] = _I[1][2] + e_R_cp[1][2] * e_R_z_sin + temp_matrix[1][2] * (1.0f - e_R_z_cos);
                
                rotation_matrix[2][0] = _I[2][0] + e_R_cp[2][0] * e_R_z_sin + temp_matrix[2][0] * (1.0f - e_R_z_cos);
                rotation_matrix[2][1] = _I[2][1] + e_R_cp[2][1] * e_R_z_sin + temp_matrix[2][1] * (1.0f - e_R_z_cos);
                rotation_matrix[2][2] = _I[2][2] + e_R_cp[2][2] * e_R_z_sin + temp_matrix[2][2] * (1.0f - e_R_z_cos);
                
                matrix3f_mul_matrix3f(R, rotation_matrix, R_rp);
            } 
            else 
            {
                /* zero roll/pitch rotation */
                rt_memcpy(R_rp, R, sizeof(R_rp));
            }

            /* R_rp and R_sp has the same Z axis, calculate yaw error */
            float R_sp_x[3] = {R_sp[0][0], R_sp[1][0], R_sp[2][0]};
            float R_rp_x[3] = {R_rp[0][0], R_rp[1][0], R_rp[2][0]};
            float R_rp_x_cross_R_sp_x[3];
    
            vector_cross_product(R_rp_x, R_sp_x, R_rp_x_cross_R_sp_x);
            e_R[2] = atan2f(vector3f_dot_product(R_rp_x_cross_R_sp_x, R_sp_z), vector3f_dot_product(R_rp_x, R_sp_x)) * yaw_w;

            if (e_R_z_cos < 0.0f) 
            {
                /* for large thrust vector rotations use another rotation method:
                * calculate angle and axis for R -> R_sp rotation directly */
                float q_error[4];
                float R_transposed[3][3] = {R[0][0], R[1][0], R[2][0],
                                            R[0][1], R[1][1], R[2][1],
                                            R[0][2], R[1][2], R[2][2]};
                float R_trans_mul_R_sp[3][3];
                                    
                matrix3f_mul_matrix3f(R_transposed, R_sp,R_trans_mul_R_sp );
                quaternion_from_dcm(R_trans_mul_R_sp, q_error);
             
                float e_R_d[3];
                if(q_error[0] >= 0.0f)
                {
                    e_R_d[0] = q_error[1]*2.0f;
                    e_R_d[1] = q_error[2]*2.0f;
                    e_R_d[2] = q_error[3]*2.0f;
                }
                else
                {
                    e_R_d[0] = -q_error[1]*2.0f;
                    e_R_d[1] = -q_error[2]*2.0f;
                    e_R_d[2] = -q_error[3]*2.0f;        
                }

                /* use fusion of Z axis based rotation and direct rotation */
                float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
                e_R[0] = e_R[0] * (1.0f - direct_w) + e_R_d[0] * direct_w;
                e_R[1] = e_R[1] * (1.0f - direct_w) + e_R_d[1] * direct_w;
                e_R[2] = e_R[2] * (1.0f - direct_w) + e_R_d[2] * direct_w;
            }

            /* calculate angular rates setpoint */
			float accel_rp_max = radians(800.0f);
			rates_sp[0] = sqrt_controller(e_R[0], att_ctrl_params.att_p[0], accel_rp_max);
			rates_sp[1] = sqrt_controller(e_R[1], att_ctrl_params.att_p[1], accel_rp_max);
			rates_sp[2] = sqrt_controller(e_R[2], att_ctrl_params.att_p[2], accel_rp_max);
			
			rates_sp[0] += e_R[1] * ctrl_state.yaw_rate;
			rates_sp[1] += -e_R[0] * ctrl_state.yaw_rate;

			// convert earth frame rates to body frame rates
			float rate_bf_desired[3];
			rate_bf_desired[0] = att_sp.roll_sp_move_rate - ctrl_state.sin_pitch * att_sp.yaw_sp_move_rate * yaw_w;
			rate_bf_desired[1] = ctrl_state.cos_roll  * att_sp.pitch_sp_move_rate + ctrl_state.sin_roll * ctrl_state.cos_pitch * att_sp.yaw_sp_move_rate * yaw_w;
			rate_bf_desired[2] = -ctrl_state.sin_roll * att_sp.pitch_sp_move_rate + ctrl_state.cos_pitch * ctrl_state.cos_roll * att_sp.yaw_sp_move_rate * yaw_w;
			
			rates_sp[0] += rate_bf_desired[0];
			rates_sp[1] += rate_bf_desired[1];
            rates_sp[2] += rate_bf_desired[2];
			
			float rate_rp_total;			
			rate_rp_total = sqrtf(rates_sp[0]*rates_sp[0] + rates_sp[1]*rates_sp[1]);
			
            /* limit rates */
            if (control_mode.flag_control_auto_enabled) 
            {
				if((att_ctrl_params.auto_rate_max[0] > 0) && (rate_rp_total > att_ctrl_params.auto_rate_max[0]))
				{
					rates_sp[0] = att_ctrl_params.auto_rate_max[0] * rates_sp[0]/rate_rp_total;
					rates_sp[1] = att_ctrl_params.auto_rate_max[1] * rates_sp[1]/rate_rp_total;
				}
                rates_sp[2] = constrain_float(rates_sp[2], -att_ctrl_params.auto_rate_max[2], att_ctrl_params.auto_rate_max[2]);
            } 
            else 
            {
				if((mc_rate_max[0] > 0) && (rate_rp_total > mc_rate_max[0]))
				{
					rates_sp[0] = mc_rate_max[0] * rates_sp[0]/rate_rp_total;
					rates_sp[1] = mc_rate_max[1] * rates_sp[1]/rate_rp_total;
				}
                rates_sp[2] = constrain_float(rates_sp[2], -mc_rate_max[2], mc_rate_max[2]);
            }
        }
        
        if (control_mode.flag_control_rates_enabled)
        {
            /* reset integral if disarmed */
            if (!status.armd || thrust_sp == 0)
            {                        
                att_mod.roll  = 0.0f;
                att_mod.pitch = 0.0f;
                att_mod.yaw   = 0.0f;
                output.control[0] = 0;
                output.control[1] = 0;
                output.control[2] = 0;
                output.control[3] = 0;
                PID_Reset(&roll_rate_ctrler);
                PID_Reset(&pitch_rate_ctrler);
                PID_Reset(&yaw_rate_ctrler);
                output.armd = status.armd;
                output.timestamp = hrt_absolute_time();
                orb_publish(ORB_ID(actuator_control), &output);
                orb_publish(ORB_ID(vehicle_att_modifier), &att_mod);  
            }
            else
            {
                /* current body angular rates */
                float rates[3];
                rates[0] = ctrl_state.roll_rate;
                rates[1] = ctrl_state.pitch_rate;
                rates[2] = ctrl_state.yaw_rate;

                bool thr_low = (thrust_sp < 250);
                att_mod.roll  = rate_to_motor_roll(&roll_rate_ctrler, rates_sp[0], rates[0] , dt, motor.limit_roll_pitch, thr_low);
                att_mod.pitch = rate_to_motor_pitch(&pitch_rate_ctrler, rates_sp[1], rates[1] , dt , motor.limit_roll_pitch, thr_low);

                if(!ground_touched_prev && (navigator._ground_touched_maybe || navigator._ground_touched))
                {
                    ground_touched_prev_time = now;
                    rt_kprintf("[ATTCTRL]landing,ready to loss yaw control!\n");
                }
                ground_touched_prev = (navigator._ground_touched||navigator._ground_touched_maybe);
                if((navigator._ground_touched_maybe || navigator._ground_touched))
                {
                    if(now - ground_touched_prev_time < 3000000)
                    {
                        att_mod.yaw   = 0.0f;
                    }
                    else
                    {
                        navigator._ground_touched_maybe = false;
                        att_mod.yaw   = rate_to_motor_yaw(&yaw_rate_ctrler, rates_sp[2], rates[2] , dt , motor.limit_yaw, thr_low);
                    }
                }
                else
                {
                    att_mod.yaw   = rate_to_motor_yaw(&yaw_rate_ctrler, rates_sp[2], rates[2] , dt , motor.limit_yaw, thr_low);
                }
                
                /* publish actuator controls */
                output.control[0] = (isfinite(att_mod.roll)) ? att_mod.roll * 500 : 0;
                output.control[1] = (isfinite(att_mod.pitch)) ? att_mod.pitch * 500 : 0;
                output.control[2] = (isfinite(att_mod.yaw)) ? att_mod.yaw * 500 : 0;
                output.control[3] = (isfinite(thrust_sp)) ? thrust_sp : 0;
                
                output.armd = status.armd;
                output.timestamp = hrt_absolute_time();     
                orb_publish(ORB_ID(actuator_control), &output);
                orb_publish(ORB_ID(vehicle_att_modifier), &att_mod);              
            }         
        }
    }

    att_ctrl_pid_running = false;
    rt_kprintf("att ctrl is stopped\r\n");
}

static void matrix3f_mul_matrix3f(float matrix3f_1[3][3], float matrix3f_2[3][3], float matrix3f_out[3][3])
{
    matrix3f_out[0][0] = matrix3f_1[0][0] * matrix3f_2[0][0] + matrix3f_1[0][1] * matrix3f_2[1][0] + matrix3f_1[0][2] * matrix3f_2[2][0];
    matrix3f_out[0][1] = matrix3f_1[0][0] * matrix3f_2[0][1] + matrix3f_1[0][1] * matrix3f_2[1][1] + matrix3f_1[0][2] * matrix3f_2[2][1];
    matrix3f_out[0][2] = matrix3f_1[0][0] * matrix3f_2[0][2] + matrix3f_1[0][1] * matrix3f_2[1][2] + matrix3f_1[0][2] * matrix3f_2[2][2];
    
    matrix3f_out[1][0] = matrix3f_1[1][0] * matrix3f_2[0][0] + matrix3f_1[1][1] * matrix3f_2[1][0] + matrix3f_1[1][2] * matrix3f_2[2][0];
    matrix3f_out[1][1] = matrix3f_1[1][0] * matrix3f_2[0][1] + matrix3f_1[1][1] * matrix3f_2[1][1] + matrix3f_1[1][2] * matrix3f_2[2][1];
    matrix3f_out[1][2] = matrix3f_1[1][0] * matrix3f_2[0][2] + matrix3f_1[1][1] * matrix3f_2[1][2] + matrix3f_1[1][2] * matrix3f_2[2][2];
    
    matrix3f_out[2][0] = matrix3f_1[2][0] * matrix3f_2[0][0] + matrix3f_1[2][1] * matrix3f_2[1][0] + matrix3f_1[2][2] * matrix3f_2[2][0];
    matrix3f_out[2][1] = matrix3f_1[2][0] * matrix3f_2[0][1] + matrix3f_1[2][1] * matrix3f_2[1][1] + matrix3f_1[2][2] * matrix3f_2[2][1];
    matrix3f_out[2][2] = matrix3f_1[2][0] * matrix3f_2[0][2] + matrix3f_1[2][1] * matrix3f_2[1][2] + matrix3f_1[2][2] * matrix3f_2[2][2];    
}

static void vector_cross_product(float vector_1[3], float vector_2[3], float vector_out[3])
{
	vector_out[0] = vector_1[1]*vector_2[2] - vector_1[2]*vector_2[1];
	vector_out[1] = vector_1[2]*vector_2[0] - vector_1[0]*vector_2[2];
	vector_out[2] = vector_1[0]*vector_2[1] - vector_1[1]*vector_2[0];
}

static float vector3f_dot_product(float vector_1[3], float vector_2[3])
{
    return vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1] + vector_1[2] * vector_2[2];
}

static void quaternion_to_dcm(float q_f[4], float matrix3f_out[3][3])
{
    float aSq = q_f[0] * q_f[0];
    float bSq = q_f[1] * q_f[1];
    float cSq = q_f[2] * q_f[2];
    float dSq = q_f[3] * q_f[3]; 
    
    matrix3f_out[0][0] = aSq + bSq - cSq - dSq;
    matrix3f_out[0][1] = 2.0f * (q_f[1] * q_f[2] - q_f[0] * q_f[3]);
    matrix3f_out[0][2] = 2.0f * (q_f[0] * q_f[2] + q_f[1] * q_f[3]);
    matrix3f_out[1][0] = 2.0f * (q_f[1] * q_f[2] + q_f[0] * q_f[3]);
    matrix3f_out[1][1] = aSq - bSq + cSq - dSq;
    matrix3f_out[1][2] = 2.0f * (q_f[2] * q_f[3] - q_f[0] * q_f[1]);
    matrix3f_out[2][0] = 2.0f * (q_f[1] * q_f[3] - q_f[0] * q_f[2]);
    matrix3f_out[2][1] = 2.0f * (q_f[0] * q_f[1] + q_f[2] * q_f[3]);
    matrix3f_out[2][2] = aSq - bSq - cSq + dSq;
}

static void quaternion_from_dcm(float dcm[3][3], float q_out[4])
{
    float tr = dcm[0][0] + dcm[1][1] + dcm[2][2];
    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f);
        q_out[0] = s * 0.5f;
        s = 0.5f / s;
        q_out[1] = (dcm[2][1] - dcm[1][2]) * s;
        q_out[2] = (dcm[0][2] - dcm[2][0]) * s;
        q_out[3] = (dcm[1][0] - dcm[0][1]) * s;
    } else {
        /* Find maximum diagonal element in dcm
        * store index in dcm_i */
        int dcm_i = 0;
        for (int i = 1; i < 3; i++) {
            if (dcm[i][i] > dcm[dcm_i][dcm_i]) {
                dcm_i = i;
            }
        }
        int dcm_j = (dcm_i + 1) % 3;
        int dcm_k = (dcm_i + 2) % 3;
        float s = sqrtf((dcm[dcm_i][dcm_i] - dcm[dcm_j][dcm_j] -
        dcm[dcm_k][dcm_k]) + 1.0f);
        q_out[dcm_i + 1] = s * 0.5f;
        s = 0.5f / s;
        q_out[dcm_j + 1] = (dcm[dcm_i][dcm_j] + dcm[dcm_j][dcm_i]) * s;
        q_out[dcm_k + 1] = (dcm[dcm_k][dcm_i] + dcm[dcm_i][dcm_k]) * s;
        q_out[0] = (dcm[dcm_k][dcm_j] - dcm[dcm_j][dcm_k]) * s;
    }    
}
