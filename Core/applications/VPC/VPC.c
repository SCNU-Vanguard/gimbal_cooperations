#include "VPC.h"



 void VPC_Receive(void)
 {
   /* Wait for notification from Gimbal that new aim data is ready. */
   xSemaphoreTake(g_xSemVPC, portMAX_DELAY);

   /* After being notified, unpack and process the latest buffer. */
   UnPack_Data_ROS2(buf_receive_from_nuc, &aim_packet_from_nuc, (sizeof(aim_packet_from_nuc)+1));

   /* Process data (e.g. send/visualize) */
   vofa_demo2(aim_packet_from_nuc.yaw_diff, aim_packet_from_nuc.pitch_diff, &huart6);

 }
 

void VPC_Init(void)
{
  aim_packet_to_nuc.detect_color=0;//1-blue 0-red
  Send_Packet_Init(&aim_packet_to_nuc);

}

 void VPC_Task(void *argument)
 {
    VPC_Init();
    TickType_t lastWakeTime = xTaskGetTickCount();

    for(;;)
     {
      VPC_Receive();
         Pack_And_Send_Data_ROS2(&aim_packet_to_nuc);     //我们要发什么呢？🤔
         vTaskDelayUntil(&lastWakeTime, VPC_TASK_PERIOD);
     }

 }


