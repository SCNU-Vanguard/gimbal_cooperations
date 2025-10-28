#include "VPC.h"



 void VPC_Receive(void)
 {
   xSemaphoreTake(g_xSemVPC, portMAX_DELAY);
   UnPack_Data_ROS2(buf_receive_from_nuc, &aim_packet_from_nuc, (sizeof(aim_packet_from_nuc)+1));
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
         Pack_And_Send_Data_ROS2(&aim_packet_to_nuc); 
         vTaskDelayUntil(&lastWakeTime, VPC_TASK_PERIOD);
     }

 }


