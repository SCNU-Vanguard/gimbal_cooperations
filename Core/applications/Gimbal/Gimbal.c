#include "Gimbal.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include <math.h>
#include "CAN_receive.h"
#include "QuaternionEKF.h"
#include "VPC.h"

uint8_t   GIMBAL_OFFSET_FLAG=1; //云台标志位
gimbal_control_t  gimbal_control;


float add_yaw;
float add_pitch;

SemaphoreHandle_t g_xSemTicks;


typedef struct{
    float diff_yaw;
    float diff_pitch;
}tempdata_t;

tempdata_t temp_data;


void Gimbal_task(void){
	g_xSemTicks=xSemaphoreCreateBinary( );
    //等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    while (1)
    {

      gimbal_control.Ctl_mode=1;  //云台远程操控模式   0 为视觉自动模式  1为遥控器模式


      if (gimbal_control.Ctl_mode==1)//远程操控模式
		{
            gimbal_detact_calibration(&gimbal_control);
            gimbal_feedback_update(&gimbal_control,&add_yaw,&add_pitch,gimbal_control.Ctl_mode);
            gimbal_angle_limit(&gimbal_control,&add_yaw,&add_pitch);
            //以absolute_angle_set为目标值，absolute_angle为当前值，进行pid串级环的运算，并将值存到motor_ready[]结构体中
            Motor_Calc(&gimbal_control);
        }

       else if (gimbal_control.Ctl_mode==0)//视觉自动模式
       {
           temp_data.diff_yaw=msp(aim_packet_from_nuc.yaw_diff,-pi,pi,0,8191);
           temp_data.diff_pitch=msp(aim_packet_from_nuc.pitch_diff,-pi,pi,0,8191);

          gimbal_detact_calibration(&gimbal_control);
          gimbal_feedback_update(&gimbal_control,&temp_data.diff_yaw,&temp_data.diff_pitch,gimbal_control.Ctl_mode);
          gimbal_angle_limit(&gimbal_control,&temp_data.diff_yaw,&temp_data.diff_pitch);
          Motor_Calc(&gimbal_control);

       }
       
    }
}
/**
  * @brief          计算ecd与offset_ecd(中值)之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd = ECD_RANGE-relative_ecd;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    //return relative_ecd * MOTOR_ECD_TO_RAD;
    return relative_ecd;
}

//云台更新数据
static void gimbal_feedback_update(gimbal_control_t *feedback_update,float *add_yaw,float *add_pitch,uint8_t Crtl_mode){


    //更新电机实时角度
    feedback_update->gimbal_pitch_motor.motor_gyro=motor_data[1].angle;
    feedback_update->gimbal_yaw_motor.motor_gyro=motor_data[0].angle;

    //更新姿态角实时角度
    //feedback_update->gimbal_pitch_motor.absolute_angle=imu_Angle.Pitch;
    //feedback_update->gimbal_yaw_motor.absolute_angle=imu_Angle.Yaw;
		//feedback_update->gimbal_pitch_motor.absolute_angle=QEKF_INS.Roll;
    //feedback_update->gimbal_yaw_motor.absolute_angle=QEKF_INS.Yaw;
	feedback_update->gimbal_pitch_motor.absolute_angle=INS.Roll;
    feedback_update->gimbal_yaw_motor.absolute_angle=INS.Yaw;

    if(Crtl_mode==1){
    //更新遥控器实时角度
    feedback_update->gimbal_rc_ctrl=get_remote_control_point();
    *add_yaw=msp(feedback_update->gimbal_rc_ctrl->rc.ch[2],-660,660,-180,180);
    *add_pitch=msp(feedback_update->gimbal_rc_ctrl->rc.ch[3],-660,660,-90,90);
    }

    else if(Crtl_mode==0){
    //更新视觉控制实时角度
    {
        *add_yaw = msp(aim_packet_from_nuc.yaw_diff,0,8191,-180,180);
        *add_pitch = msp(aim_packet_from_nuc.pitch_diff,0,8191,-90,90);
        aim_packet_from_nuc.yaw+=*add_yaw;
        aim_packet_from_nuc.pitch+=*add_pitch;
    }


    feedback_update->gimbal_pitch_motor.absolute_angle_set=*add_pitch;
    feedback_update->gimbal_yaw_motor.absolute_angle_set=*add_yaw;
    //更新电机目标机械角度
    feedback_update->gimbal_pitch_motor.motor_gyro_set=feedback_update->gimbal_pitch_motor.motor_gyro+*add_pitch;
    feedback_update->gimbal_yaw_motor.motor_gyro_set=feedback_update->gimbal_yaw_motor.motor_gyro+*add_yaw;

    //计算设置角度后的目标角度与中值的相对角度，并以此为标准，因为最大和最小限幅值也是根据相对角度来定的，这样避免目标角度出现负值
    feedback_update->gimbal_pitch_motor.relative_angle=motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.motor_gyro,OFFSET_ECD);
    feedback_update->gimbal_yaw_motor.relative_angle_set=motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.motor_gyro_set,0);
    }
    
    xSemaphoreGive(g_xSemVPC);

    //计算云台相对于最大限幅值的相对角度，同时判断此时电机处于左值还是右值
    // if(feedback_update->gimbal_pitch_motor.relative_angle_set-PITCH_Limit_Hight>0)
    // feedback_update->gimbal_pitch_motor.relative_angle=fabs(feedback_update->gimbal_pitch_motor.relative_angle_set-PITCH_Limit_Hight);
    // else if(feedback_update->gimbal_pitch_motor.relative_angle_set-PITCH_Limit_Hight<0)
    // feedback_update->gimbal_pitch_motor.relative_angle=fabs(feedback_update->gimbal_pitch_motor.relative_angle_set-PITCH_Limit_Low);

    // if(feedback_update->gimbal_yaw_motor.relative_angle_set-YAW_Limit_Hight>0)
    // feedback_update->gimbal_yaw_motor.relative_angle=fabs(feedback_update->gimbal_yaw_motor.relative_angle_set-YAW_Limit_Hight);
    // else if(feedback_update->gimbal_yaw_motor.relative_angle_set-YAW_Limit_Hight<0)
    // feedback_update->gimbal_yaw_motor.relative_angle=fabs(feedback_update->gimbal_yaw_motor.relative_angle_set-YAW_Limit_Low);


}


//云台校准中值并执行归中
void gimbal_detact_calibration(gimbal_control_t *gimbal_motort){
   
    //添加标志位判断有无执行过归中，如果有，则不再归中
    while(GIMBAL_GET_FLAG(GIMBAL_OFFSET_FLAG)){
        
        static uint16_t int_time=0;
        static uint16_t int_stop_time=0;
        gimbal_motort->gimbal_pitch_motor.motor_gyro=motor_data[1].angle;
				gimbal_motort->gimbal_yaw_motor.motor_gyro=motor_data[0].angle;
        MotorSetTar(&motor_ready[0], 1941, ABS);  
        MotorSetTar(&motor_ready[1], OFFSET_ECD, ABS);
				Motor_return(gimbal_motort);
		
        int_time++;
        if(fabs(gimbal_motort->gimbal_yaw_motor.motor_gyro-1941)<GIMBAL_INIT_ANGLE_ERROR){
            //if(int_stop_time<GIMBAL_INIT_STOP_TIME){
            int_stop_time++;
            //}
           }
        //此处为源码的逻辑，但觉得int_time无需重复累加两次，故删去
        //    else{
        //     if(int_time<GIMBAL_INIT_TIME){
        //         int_time++;
        //     }
        //    }

           //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
        //    if(int_time < GIMBAL_INIT_TIME && int_stop_time < GIMBAL_INIT_STOP_TIME){
        //     return;
        //    }else{
        if(int_time > GIMBAL_INIT_TIME && int_stop_time > GIMBAL_INIT_STOP_TIME){
            //信号量释放
            xSemaphoreGive(g_xSemTicks);
            int_stop_time = 0;
            int_time = 0;
            GIMBAL_FLAG_RESET(GIMBAL_OFFSET_FLAG);
           }
       
    }


}


//云台限幅
void gimbal_angle_limit(gimbal_control_t *gimbal_motort,float *add_yaw,float *add_pitch){
	 if (gimbal_motort == NULL || add_pitch == NULL||add_yaw==NULL) {
        return;
    }
	 //pitch

    //根据当前输入值的正负来判断处于左值还是右值，从而决定目标值,并且加上一个死区，防止频繁切换
    if(*add_pitch<0){
        if(gimbal_motort->gimbal_pitch_motor.absolute_angle<=-40){
            MotorSetTar(&motor_ready[1],PITCH_Limit_Hight,ABS);

    }else{
        MotorSetTar(&motor_ready[1],gimbal_motort->gimbal_pitch_motor.absolute_angle_set,ABS);
    }
    }else if(*add_pitch>0){
        if(gimbal_motort->gimbal_pitch_motor.absolute_angle>=35){
                MotorSetTar(&motor_ready[1],PITCH_Limit_Low,ABS);
    }else{
				MotorSetTar(&motor_ready[1],gimbal_motort->gimbal_pitch_motor.absolute_angle_set,ABS);
    }
}   else{
				MotorSetTar(&motor_ready[1],gimbal_motort->gimbal_pitch_motor.absolute_angle_set,ABS);
}


    //yaw
    if(*add_yaw<0){
        if(*add_yaw<=-50){
        if(gimbal_motort->gimbal_yaw_motor.absolute_angle<=-50){
            MotorSetTar(&motor_ready[0],PITCH_Limit_Hight,ABS);
        }
    }else{
        MotorSetTar(&motor_ready[0],gimbal_motort->gimbal_yaw_motor.absolute_angle_set,ABS);
    }
    }else if(*add_yaw>0){
        if(*add_yaw>=50){
        if(gimbal_motort->gimbal_yaw_motor.absolute_angle>=50){
                MotorSetTar(&motor_ready[0],PITCH_Limit_Low,ABS);
        }
    }else{
				MotorSetTar(&motor_ready[0],gimbal_motort->gimbal_yaw_motor.absolute_angle_set,ABS);
    }
}else{
				MotorSetTar(&motor_ready[0],gimbal_motort->gimbal_yaw_motor.absolute_angle_set,ABS);
}


}
