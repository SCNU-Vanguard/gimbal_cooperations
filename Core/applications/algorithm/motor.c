#include "motor.h"
#include "pid.h"
Motor_send motor_ready[MOTOR_NUM];
Motor_list motor_data[MOTOR_NUM];

/*
 * @brief  	设置电机目标值
 * @param	电机结构体指针
 * @param	目标值
 * @param   ABS->absolute target;
 *          INCR->add from perious target
 *          FEED->change feedforward
 * @retval 	无
 */
void MotorSetTar(Motor_send *motor,float val, ValSet_Type_e type)
{
    if ( type == ABS )
		motor -> target = val;
	else if ( type == INCR )
		motor -> target += val;
//         else
// 		sErrorHandel (OUT_OF_ENUM);
}
//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi)
double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
	// X/8192 *2*PI
	//           0        pi     -pi        8192   0       -PI
	return (x-in_min)*(out_max-out_min)/(in_max-in_min);
}

void Motor_Calc(gimbal_control_t *feedback_update)
{
	static float tar=0,real=0;

	//yaw轴计算
	tar=msp(motor_ready[0].target,0,8191,-pi,pi);
	real=msp(feedback_update->gimbal_yaw_motor.absolute_angle,0,8191,-pi,pi);
    //过零点处理
	// while(tar-real > PI)
	//  	real += 2 * PI ;
	// while(tar-real < -PI)
	//  	real -= 2 * PI ;
	motor_ready[MOTOR_YAW].output_Position=pid_calc_raw(&gimbal_yaw_angle_pid,tar,real);
    motor_ready[MOTOR_YAW].output=pid_calc_speed(&gimbal_yaw_speed_pid,motor_ready[MOTOR_YAW].output_Position,motor_data[MOTOR_YAW].speed);

	//pitch轴计算
	tar=msp(motor_ready[1].target,0,360,-pi,pi);
	real=msp(feedback_update->gimbal_pitch_motor.absolute_angle,0,360,-pi,pi);
    //过零点处理
	// while(tar-real > 4096)
	//  	real += 8191 ;
	// while(tar-real < -4096)
	//  	real -= 8191 ;
	motor_ready[MOTOR_PITCH].output_Position=pid_calc_raw(&gimbal_yaw_angle_pid,tar,real);
    motor_ready[MOTOR_PITCH].output=pid_calc_speed(&gimbal_yaw_speed_pid,motor_ready[MOTOR_PITCH].output_Position,motor_data[MOTOR_PITCH].speed);

}

void Motor_return(gimbal_control_t *feedback_update){
	if (feedback_update == NULL ) {
		return;
	}
	static float tar=0,real=0;
	//pitch轴计算
	tar=msp(motor_ready[1].target,0,8191,-pi,pi);
	real=msp(feedback_update->gimbal_pitch_motor.motor_gyro,0,8191,-pi,pi);
	motor_ready[MOTOR_PITCH].output_Position=pid_calc_raw_return(&gimbal_yaw_angle_pid_return,tar,real);
    motor_ready[MOTOR_PITCH].output=pid_calc_speed(&gimbal_yaw_speed_pid_return,motor_ready[MOTOR_PITCH].output_Position,motor_data[MOTOR_PITCH].speed);
}
