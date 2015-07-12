//define instraction for communicate with PC...
#define Ins_Set_Walk                 201  
#define Ins_Set_Head_Joints          202  
#define Ins_Get_Euler_States         203  

#define Ins_Set_Parameter            204  
#define Ins_Get_Parameter            205  

#define No_Request                   0
#define Request_Euler_State          1
#define Request_Parameter            2

//Robot State Main Task
void vRobot_State_Task( void *pvParameters ){
  
  vTaskSuspendAll();
  Serial2.println("AUT_UofM:> Robot State Task Start Sucssecfully!");
  xTaskResumeAll();
  
  portTickType xLastWakeTime;
  const portTickType xFrequency = 100;  //10ms for each loop run time means 100Hz of task frequency
  xLastWakeTime = xTaskGetTickCount ();
  vTaskDelay(500);
  
  double Acc=0;
  double Last_Acc=0;
  double Last_Vx=0;
  
  //Queue <double> DQueue;
  
  for(;;){
    
    RSL_Loop_Cnt++;
    
    //if(Debug_Mode){
    //  RTOS_Error_Log("RST Task:",RSL_Loop_Cnt);
    //}
    
    vTaskSuspendAll();
    Send_Euler_State();
    xTaskResumeAll();
    
    //run motion if key pressed
    if(digitalRead(BUTTON1_485EXP) == 1){
      Motion_Ins=Motion_1;   
      //Serial2.println("AUT_UofM:> Key Presed!");
    }
    
    //if(digitalRead(BUTTON2_485EXP) == 1){
    //  RTOS_Error_Log("Key 2 Pressed!");
    //}
    
    
    //
    if(digitalRead(11) == 1){
      Internal_Motion_Request=Stop_Motion;
    }
    else{
      Internal_Motion_Request=No_Motion;  //release the robot stop
    }
    
    /*
    if((System_Voltage/10)>20.0){
      Serial2.println("\t No Voltage!");
    }
    else{
      Serial2.print("\tV=");Serial2.print(System_Voltage/10);Serial2.println("v");
    }
    */ 
    
    //check for voltage and error if..
    if(System_Voltage<=(int)(WEP[P_Min_Voltage_Limit]+2)){
      togglePin(Buzzer_Pin);
      //Serial2.println("AUT_UofM:> LOW Woltage! ERROR... v="); Serial2.println(System_Voltage);
    }
    else{
      digitalWrite(Buzzer_Pin, LOW);
    }
    
    //get robot state for fall
    Robot_State();
    
    //check for robot fall state
    if(Check_Robot_Fall==1){
      if ((Internal_Motion_Request!=No_Motion ) && (Internal_Motion_Request!=Stop_Motion)) {
        Actuators_Update=0;
        switch (Internal_Motion_Request){
          case Stand_Up_Front:
            Set_Head(0,-0.6,1023,1023);
            break;
          case Stand_Up_Back:
            Set_Head(0,0.5,1023,1023);
            break;
        }
      }
      else{
        Actuators_Update=1;
      }
    }
    
    
    //push recovery
    Last_Acc=Acc;
    Acc=(0.9 * Acc)+(0.1 * MPU.get_Az());
      
    if (abs(Last_Acc-Acc)>0.15){
      Last_Vx=Vx;
      Vx=((Last_Acc-Acc)*1);
      vTaskDelay(500);
      Vx=Last_Vx;
    }
    
    //Serial2.print(" A= ***** > ");Serial2.print((Last_Acc-Acc));Serial2.println(" ");
      
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); 
  }
}
