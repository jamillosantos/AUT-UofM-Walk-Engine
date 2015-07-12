//define instraction for communicate with PC...
#define Ins_Set_Walk                 201  
#define Ins_Set_Head_Joints          202  
#define Ins_Get_Euler_States         203  

#define Ins_Set_Parameter            204  
#define Ins_Get_Parameter            205  

#define No_Request                   0
#define Request_Euler_State          1
#define Request_Parameter            2

void vSerial_USB_Tx_Task(void *pvParameters ){
  
  vTaskSuspendAll();
  Serial2.println("AUT_UofM:> USB Tx Task Start Sucssecfully!");
  xTaskResumeAll(); 
  
  portTickType xLastWakeTime;
  const portTickType xFrequency = 100;  //10ms for each loop run time means 100Hz of task frequency
  xLastWakeTime = xTaskGetTickCount ();
  
  //main task loop
  for( ;; ){   
    SUB_Loop_Cnt++;
    Send_Euler_State();
    
    //if(Debug_Mode){
    //  RTOS_Error_Log("SUB Task:",SUB_Loop_Cnt);
    //}
    
    digitalWrite(GREEN_LED_485EXP, 0);   
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); 
    digitalWrite(GREEN_LED_485EXP, 1);    
  }
}

/* 
* send Euler state to the USB serial Port
*/
void Send_Euler_State(){
  
   byte PData[10];
   
   PData[0]=(byte)254;
   PData[1]=(byte)254;
   PData[2]=(byte)100;
   
   unsigned long T =(unsigned long)((MPU_X+Pi) * 1000);         
   PData[3]= _LOBYTE(T);
   PData[4]= _HIBYTE(T);
      
   T =(unsigned long)((MPU_Y+Pi)*1000); 
   PData[5]= _LOBYTE(T);
   PData[6]= _HIBYTE(T);
   
   T =(unsigned long)((MPU_Z+Pi)*1000); 
   PData[7]= _LOBYTE(T);
   PData[8]= _HIBYTE(T);
    
   for(byte i=0;i<=8;i++){
      SerialUSB.print((char)PData[i]);
   } 
}
