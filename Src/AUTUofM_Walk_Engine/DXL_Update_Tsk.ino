//Dynamixel joints Id  (id must be diffrent and betwin 1~20)
#define Id_Right_Arm_Pitch           1-1
#define Id_Right_Arm_Roll            3-1
#define Id_Right_Arm_Elbow           5-1
#define Id_Right_Hip_Yaw             7-1
#define Id_Right_Hip_Roll            9-1
#define Id_Right_Hip_Pitch           11-1
#define Id_Right_Knee                13-1
#define Id_Right_Foot_Pitch          15-1
#define Id_Right_Foot_Roll           17-1
#define Id_Left_Arm_Pitch            2-1
#define Id_Left_Arm_Roll             4-1
#define Id_Left_Arm_Elbow            6-1
#define Id_Left_Hip_Yaw              8-1
#define Id_Left_Hip_Roll             10-1
#define Id_Left_Hip_Pitch            12-1
#define Id_Left_Knee                 14-1
#define Id_Left_Foot_Pitch           16-1
#define Id_Left_Foot_Roll            18-1

#define Id_Head_Pan                  30
#define Id_Head_Tilt                 31

//this is fix for humanoid with 20 DOF
#define NUM_OF_DXL            (18)  

//global array for set joint angle
double Angle[NUM_OF_DXL];
double Speed[NUM_OF_DXL];
byte   D_TORQUE_ENABLE[NUM_OF_DXL];
byte   D_STATUS_LED[NUM_OF_DXL];

//byte   D_KD_GAIN[NUM_OF_DXL];   
//byte   D_KI_GAIN[NUM_OF_DXL];
//byte   D_KP_GAIN[NUM_OF_DXL];

byte   id[NUM_OF_DXL];              //this is not global 
int    D_GOAL_POSITION[NUM_OF_DXL]; //this is not global 
int    D_MOVING_SPEED[NUM_OF_DXL];  //this is also!

//address registers for dynamixel
#define P_TORQUE_ENABLE     24
#define P_GOAL_POSITION_L   30
#define P_PRESENT_VOLTAGE   42
#define P_P_Gain            28
#define P_I_Gain            27
#define P_D_Gain            26
#define P_Baud_Rate         4
#define P_Return_Delay_Time 5

//joints direction for robot joints
int Joints_Direction[NUM_OF_DXL];

//main task for dynamixel update
void vDynamixel_Update_Task( void *pvParameters ){ 
  
  vTaskSuspendAll();
  Serial2.println("AUT_UofM:> Dynamixel Update Task Start Sucssecfully!");
  xTaskResumeAll();
  
  vTaskSuspendAll();
  Dxl.writeByte(BROADCAST_ID,P_Return_Delay_Time,0);
  xTaskResumeAll();
  
  portTickType xLastWakeTime;
  const portTickType xFrequency = 25;  //10ms for each loop run time means 100Hz of task frequency
  xLastWakeTime = xTaskGetTickCount ();
  
  vTaskSuspendAll();
  Serial2.print("AUT_UofM:> Configure Robot Walk Engine...");
  DXL_Check(); //check for all dynamixel exist
  Init_Dxls_First_Time();
  Serial2.print("AUT_UofM:> OK!");
  xTaskResumeAll();
  
  vTaskDelay(2500);
  
  
  //main task loop
  for( ;; ){
    DXL_Loop_Cnt++;
    
    if(DXL_Loop_Cnt==1){
      vTaskSuspendAll();
      System_Voltage=(0.5*System_Voltage)+(0.5*((int)Dxl.readByte(Id_Head_Pan, P_PRESENT_VOLTAGE)));
      xTaskResumeAll();
    }
    
    if(DXL_Loop_Cnt==1){
      vTaskSuspendAll();
      Dxl.writeByte(BROADCAST_ID,P_P_Gain,32);
      Dxl.writeByte(BROADCAST_ID,P_I_Gain,1);
      Dxl.writeByte(BROADCAST_ID,P_D_Gain,0);
      
      Dxl.writeByte(Id_Head_Pan ,P_P_Gain,20);
      Dxl.writeByte(Id_Head_Tilt,P_P_Gain,15);
      xTaskResumeAll();
    }
    
    //update actuators position and speed
    if(Actuators_Update==0){
      Dxl.writeByte(BROADCAST_ID,P_TORQUE_ENABLE,0);
    }
    else{
      DXL_Write_PS();
    }
     
    DXL_Write_Head();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void Set_Head(double Pan, double Tilt, double Pan_Speed, double Tilt_Speed){
  Head_Pan_Angle=Pan;
  Head_Tilt_Angle=Tilt;
  Head_Pan_Speed=Pan_Speed;
  Head_Tilt_Speed=Tilt_Speed;
}

void DXL_Write_Head(){
  vTaskSuspendAll();
  Dxl.setPosition(Id_Head_Pan,2048+(((int)((Head_Pan_Angle*RAD2DEG)*DEG2DXL))*1),(int)Head_Pan_Speed);
  Dxl.setPosition(Id_Head_Tilt,2048+(((int)((Head_Tilt_Angle*RAD2DEG)*DEG2DXL))*1),(int)Head_Tilt_Speed);  
  xTaskResumeAll();
}

void DXL_Write_PS(){      
      //fill the send pocket for sync write of: //goal position //speed
      Dxl.setTxPacketId(BROADCAST_ID);
      Dxl.setTxPacketInstruction(INST_SYNC_WRITE);
      Dxl.setTxPacketParameter(0, P_GOAL_POSITION_L);  //start register to write
      Dxl.setTxPacketParameter(1, 4);                  //count of write per motor
      for(byte i=0; i<=NUM_OF_DXL-1; i++ ){
        if(D_TORQUE_ENABLE[i]==1) {
          Dxl.setTxPacketParameter((2+5*i), id[i]);
        }    
        else {
          Dxl.setTxPacketParameter((2+5*i), 250);
        }
        
        D_GOAL_POSITION[i]=2048+(((int)((Angle[i]*RAD2DEG)*DEG2DXL))*(int)Joints_Direction[i]);  //init goal position for first time     
        Dxl.setTxPacketParameter((2+5*i)+1, _LOBYTE(D_GOAL_POSITION[i]));  //goal position low
        Dxl.setTxPacketParameter((2+5*i)+2, _HIBYTE(D_GOAL_POSITION[i])); //goal position high
        
        D_MOVING_SPEED[i] = (unsigned long)(Speed[i]*1023);
        Dxl.setTxPacketParameter((2+5*i)+3, _LOBYTE(D_MOVING_SPEED[i]));  //speed low
        Dxl.setTxPacketParameter((2+5*i)+4, _HIBYTE(D_MOVING_SPEED[i])); //speed high
      }
      Dxl.setTxPacketLength((4+1)*((byte)(NUM_OF_DXL))+4); //??
      vTaskSuspendAll();
      Dxl.txrxPacket();    //Send and resive packet
      xTaskResumeAll();
}


void Init_Dxls_First_Time(){
  byte i=0;
  
  //init dxl angle and speed for first time 
  for(i=0; i<=NUM_OF_DXL-1; i++ ){
    Angle[i]=0.0;
    Speed[i]=0.01;
  }
  
  //initialize head joint 
  Set_Head(0,0.5,100,100); 
  
  Joints_Direction[Id_Right_Arm_Pitch] =  1;
  Joints_Direction[Id_Right_Arm_Roll]  =  1;
  Joints_Direction[Id_Right_Arm_Elbow] =  1;
  Joints_Direction[Id_Right_Hip_Yaw]   =  1;
  Joints_Direction[Id_Right_Hip_Roll]  =  1;
  Joints_Direction[Id_Right_Hip_Pitch] =  1;
  Joints_Direction[Id_Right_Knee]      =  1;
  Joints_Direction[Id_Right_Foot_Pitch]= -1;
  Joints_Direction[Id_Right_Foot_Roll] = -1;

  Joints_Direction[Id_Left_Arm_Pitch]  = -1;
  Joints_Direction[Id_Left_Arm_Roll]   = -1;
  Joints_Direction[Id_Left_Arm_Elbow]  = -1;
  Joints_Direction[Id_Left_Hip_Yaw]    = -1;
  Joints_Direction[Id_Left_Hip_Roll]   = -1;
  Joints_Direction[Id_Left_Hip_Pitch]  = -1;
  Joints_Direction[Id_Left_Knee]       = -1;
  Joints_Direction[Id_Left_Foot_Pitch] =  1;
  Joints_Direction[Id_Left_Foot_Roll]  =  1;
    
  //update the defult arrays for send for first time
  for(i=0; i<=NUM_OF_DXL-1; i++ ){
      id[i] = i+1;
      
      if ((Speed[i]>=0.0)&&(Speed[i]<=1.0)){ 
          //D_MOVING_SPEED[i] = (int)(Speed[i]*1023);   //initialize speed for first time 
          D_MOVING_SPEED[i]=1;
      } 
      else{  
          D_MOVING_SPEED[i]=100;
      }
            
      if ((Angle[i]>=-3.141)&&(Angle[i]<=3.141)){ 
          D_GOAL_POSITION[i]=2048+((int)((Angle[i]*RAD2DEG)*DEG2DXL));  //init goal position for first time
      }
      else{ 
          D_GOAL_POSITION[i]=2048;
      }
      
      //defult torque enable is true
      D_TORQUE_ENABLE[i]=1;
      D_STATUS_LED[i]=1;
      
      //set PID gains to defult
      //D_KD_GAIN[i]=0;
      //D_KI_GAIN[i]=0;
      //D_KP_GAIN[i]=32;
  }
}

void DXL_Check(){
  int model;
  Serial2.println(" ");
  Serial2.println("AUT_UofM:> Check DXL Exist...");
  
  for (int i=1; i<32; i++){
    Serial2.print("AUT_UofM:> ID=");
    Serial2.print(i);
    
    model = Dxl.readWord(i, 0);

    if(model == 12)
      Serial2.println(": AX-12A");

    else if(model == 300)
      Serial2.println(": AX-12W");

    else if(model == 18)
      Serial2.println(": AX-18A");

    else if(model == 29)
      Serial2.println(": MX-28");     

    else if(model == 54)
      Serial2.println(": MX-64");

    else if(model == 64)
      Serial2.println(": MX-106");

    else if(model == 350)
      Serial2.println(": XL-320");   

    else{
      if(model == 65535) model = 0;
      Serial2.print(": Unknown : "); 
      Serial2.println(model);
    }
  }
  Serial2.println("AUT_UofM:> Check DXL DONE!");
}
