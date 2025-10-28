#include "VPC.h"



 void VPC_Receive(void)
 {
  /* Wait until Serial notifies that a validated packet is ready. */
  if (g_xSemVPC == NULL) {
    /* semaphore not ready yet; avoid calling null handle */
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }
  xSemaphoreTake(g_xSemVPC, pdMS_TO_TICKS(10));
  /* Serial already copied validated frame into aim_packet_from_nuc in UnPack_Data_ROS2 */
 }
 

void VPC_Init(void)
{
  aim_packet_to_nuc.detect_color=0;//1-blue 0-red
  Send_Packet_Init(&aim_packet_to_nuc);

}

 void VPC_Task(void *argument)
 {
    VPC_Init();
    /* one-shot debug notification to host to confirm task start */
    {
      const char msg[] = "VPC task started\r\n";
      CDC_SendFeed((uint8_t*)msg, sizeof(msg)-1);
    }
    TickType_t lastWakeTime = xTaskGetTickCount();

    for(;;)
     {
      VPC_Receive();
         Pack_And_Send_Data_ROS2(&aim_packet_to_nuc); 
         vTaskDelayUntil(&lastWakeTime, VPC_TASK_PERIOD);
     }

 }


