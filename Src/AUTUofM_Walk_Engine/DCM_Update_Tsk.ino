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
#define NUM_OF_DXL                   18  

#define MPU_GY80_VCC_Pin             8
#define MPU_GY80_GND_Pin             7
#define MPU_GY80_SCL_Pin             6
#define MPU_GY80_SDA_Pin             5

//global array for set joint angle
double Angle[NUM_OF_DXL];
double Speed[NUM_OF_DXL];
byte   D_TORQUE_ENABLE[NUM_OF_DXL];
byte   D_STATUS_LED[NUM_OF_DXL];

byte   id[NUM_OF_DXL];              //this is not global 
int    D_GOAL_POSITION[NUM_OF_DXL]; //this is not global 
int    D_MOVING_SPEED[NUM_OF_DXL];  //this is also!

unsigned long int DCM_Loop_Cnt=0;

//address registers for dynamixel
#define P_TORQUE_ENABLE       24
#define P_GOAL_POSITION_L     30
#define P_PRESENT_VOLTAGE     42
#define P_P_Gain              28
#define P_I_Gain              27
#define P_D_Gain              26
#define P_Baud_Rate           4
#define P_Return_Delay_Time   5
#define P_Status_LED          25

//joints direction for robot joints
int Joints_Direction[NUM_OF_DXL];

//main task for dynamixel update
void vDCM_Update_Task( void *pvParameters ){ 
  
  //Initialize walk engine parameters in first time
  Set_Walk_Engine_Parameters((byte)Teen_Size_Robot_Num);
  
  portTickType xLastWakeTime;
  const portTickType xFrequency = 10;  //10ms for each loop run time means 100Hz of task frequency
  xLastWakeTime = xTaskGetTickCount ();
  
  vTaskSuspendAll();
  Init_Dxls_First_Time();
  
  //initialize GY80 power pins
  pinMode(MPU_GY80_VCC_Pin, OUTPUT); digitalWrite(MPU_GY80_VCC_Pin, HIGH);  //gnd of mpu-GY80
  pinMode(MPU_GY80_GND_Pin, OUTPUT); digitalWrite(MPU_GY80_GND_Pin, LOW);   //vcc of mpu-GY80 
  vTaskDelay(10);
  
  //Initialize i2c comunication port (sda and scl)
  Wire.begin(MPU_GY80_SDA_Pin,MPU_GY80_SCL_Pin); 
  vTaskDelay(10);
  
  //Configuration MPU-9150...
  MPU.init(); //initialize MPU
  MPU.initDrift(20); //calculate drrift
  
  kalmanX.setRmeasure(WEP[P_Kalman_Roll_RM_Rate]);
  kalmanY.setRmeasure(WEP[P_Kalman_Pitch_RM_Rate]);
  
  xTaskResumeAll();
  
  vTaskDelay(10);
  DXL_Write_PS();
  vTaskDelay(10);
  
  MPU_X=0.0;
  MPU_Y=0.0;
  MPU_Z=0.0;
  
  //Kalman Filter Initialize...
  kalmanX.setAngle(0.0); // Set starting angle
  kalmanY.setAngle(0.0);
  
  /*
  for(int i=0;i<=10;i++){ 
      vTaskSuspendAll();
      Dxl.writeByte(BROADCAST_ID,P_P_Gain,25);
      Dxl.writeByte(BROADCAST_ID,P_I_Gain,1);
      Dxl.writeByte(BROADCAST_ID,P_D_Gain,0);
      
      Dxl.writeByte(Id_Head_Pan ,P_P_Gain,20);
      Dxl.writeByte(Id_Head_Tilt,P_P_Gain,15);
      
      Dxl.writeByte(Id_Right_Arm_Pitch,P_P_Gain,5);
      Dxl.writeByte(Id_Right_Arm_Roll,P_P_Gain,5);
      Dxl.writeByte(Id_Left_Arm_Pitch,P_P_Gain,5);
      Dxl.writeByte(Id_Left_Arm_Roll,P_P_Gain,5);
      
      xTaskResumeAll();
      vTaskDelay(5);
  }
  */
  
  for(unsigned int i=0;i<=1000;i++){
    togglePin(BLUE_LED_485EXP);
    Calculate_Euler_Angles();
    vTaskDelay(10);
  }
    
  Check_Robot_Fall=1;
  
  byte LED_Mod=1;
  
  //main task loop
  for( ;; ){
    DCM_Loop_Cnt++;
    if(DCM_Loop_Cnt>=100) DCM_Loop_Cnt=0;  //50
    
    //check for data recive from USB serial
    if(SerialUSB.available()==7){
      byte buffer[7];
      SerialUSB.read(buffer,7);
      vTaskSuspendAll();
      digitalWrite(BOARD_LED_PIN,LOW);
      if(buffer[0]==254){
                //walk data update
                Vx = (double) ((((byte)buffer[1])-100) / 100.0);
                Vy = (double) ((((byte)buffer[2])-100) / 100.0);
                Vt = (double) ((((byte)buffer[3])-100) / 100.0);
                
                Motion_Ins = (Internal_Motion_Request==No_Motion ) ? buffer[4] : No_Motion;
                                
                if (Internal_Motion_Request==No_Motion ){
                  Set_Head((double)((((byte)buffer[5]-100)*Pi)/100.0),(double)((((byte)buffer[6]-100)*Pi)/100.0),WEP[P_Head_Pan_Speed],WEP[P_Head_Tilt_Speed]);
                }
      }
      digitalWrite(BOARD_LED_PIN,HIGH);
      xTaskResumeAll();
    }
    
    //calculate euler angles
    Calculate_Euler_Angles();
    
    //update voltage and PID
    if(DCM_Loop_Cnt==1){
      vTaskSuspendAll();
      System_Voltage=(0.5*System_Voltage)+(0.5*((int)Dxl.readByte(Id_Head_Pan, P_PRESENT_VOLTAGE)));
      
      if((Motion_Ins==No_Motion) && (Internal_Motion_Request==No_Motion )){
        Dxl.writeByte(BROADCAST_ID,P_P_Gain ,25);
        Dxl.writeByte(BROADCAST_ID,P_I_Gain ,1);
        Dxl.writeByte(BROADCAST_ID,P_D_Gain ,0);
      
        Dxl.writeByte(Id_Head_Pan ,P_P_Gain ,20);
        Dxl.writeByte(Id_Head_Tilt,P_P_Gain ,15);
      
        Dxl.writeByte(Id_Right_Arm_Pitch,P_P_Gain ,5);
        Dxl.writeByte(Id_Right_Arm_Roll,P_P_Gain  ,5);
        Dxl.writeByte(Id_Right_Arm_Elbow,P_P_Gain ,5);
        
        Dxl.writeByte(Id_Left_Arm_Pitch,P_P_Gain  ,5);
        Dxl.writeByte(Id_Left_Arm_Roll,P_P_Gain   ,5);
        Dxl.writeByte(Id_Left_Arm_Elbow,P_P_Gain  ,5);
      }   
      xTaskResumeAll();
    }
    
    //send data to the usb serial port
    if((DCM_Loop_Cnt%10)==0){ 
      digitalWrite(GREEN_LED_485EXP, LOW);
      Send_Euler_State();
      digitalWrite(GREEN_LED_485EXP, HIGH);
    }
    
    //motors led
    if ((DCM_Loop_Cnt==1) || (DCM_Loop_Cnt==5)){
      vTaskSuspendAll();
      Dxl.writeByte(BROADCAST_ID,P_Status_LED  ,LED_Mod);
      xTaskResumeAll();
      LED_Mod= (LED_Mod==1) ? 0 : 1;
    }
    
    
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
        //Stand_Init(0.05);
        Actuators_Update=1;
      }
    }
    
    /*
    double Acc=0;
    double Last_Acc=0;
    double Last_Vx=0;
  
    //push recovery
    Last_Acc=Acc;
    Acc=(0.9 * Acc)+(0.1 * MPU.get_Az());
      
    if (abs(Last_Acc-Acc)>0.15){
      Last_Vx=Vx;
      Vx=((Last_Acc-Acc)*1);
      vTaskDelay(500);
      Vx=Last_Vx;
    }
    */
    
    //chek for pickup
    if(digitalRead(11) == 1){
      Internal_Motion_Request=Stop_Motion;
    }
    else{
      Internal_Motion_Request=No_Motion;  //release the robot stop
    }
    
      
    //run motion if key pressed
    if(digitalRead(BUTTON1_485EXP) == 1){
      Motion_Ins=Motion_1;   
    }
    
    //if(digitalRead(BUTTON2_485EXP) == 1){
    //  
    //}
     
    //update actuators position and speed
    if(Actuators_Update==0){
      vTaskSuspendAll();
      Dxl.writeByte(BROADCAST_ID,P_TORQUE_ENABLE,0);
      xTaskResumeAll();
    }
    else{
      DXL_Write_PS();
    }
    
    //update head update 
    DXL_Write_Head();
    
    //wait for RTOS task
    digitalWrite(BLUE_LED_485EXP, LOW); 
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    digitalWrite(BLUE_LED_485EXP, HIGH);
  }
}

void Set_Head(double Pan, double Tilt, double Pan_Speed, double Tilt_Speed){
  Head_Pan_Angle=Pan;
  Head_Tilt_Angle=Tilt;
  
  //check for head limitation
  if(Head_Pan_Angle >  (Pi/2.0)) Head_Pan_Angle =  (Pi/2.0);
  if(Head_Pan_Angle < -(Pi/2.0)) Head_Pan_Angle = -(Pi/2.0);
  
  //check for head tilit limitation
  if(Head_Tilt_Angle >  (Pi/2.0)) Head_Tilt_Angle =  (Pi/2.0);
  if(Head_Tilt_Angle < -(Pi/3.0)) Head_Tilt_Angle = -(Pi/3.0);
  
  Head_Pan_Speed=Pan_Speed;
  Head_Tilt_Speed=Tilt_Speed;
}

void DXL_Write_Head(){
  vTaskSuspendAll();
  Dxl.setPosition(Id_Head_Pan,2048+(((int)((Head_Pan_Angle*RAD2DEG)*DEG2DXL))*1),(int)Head_Pan_Speed);
  Dxl.setPosition(Id_Head_Tilt,2048+(((int)(((Head_Tilt_Angle-MPU_X)*RAD2DEG)*DEG2DXL))*1),(int)Head_Tilt_Speed);  
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
  /*
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
  */
}

double dt=0 , PrevTime=0;
double MPU_Angle_X=0, MPU_Angle_Y=0, MPU_Angle_Z=0;
//MQueue <double> XQueue; //make queue with 10 element
//MQueue <double> YQueue; //make queue with 10 element
//double Ti[10]={1, 2, 3 ,4 ,5 ,6 ,7 ,8 ,9 ,10};
//int POLY_DEGREE = 4;
//double coefficients[4]; //These will be the calculated coefficients
//double dataArray[10]; //Real points we wanna fit into
//double MPU_X_P=0;
//double MPU_Y_P=0;

void Calculate_Euler_Angles(){
    //calculate deltaT for kalman filter calculation
    dt = (double)((micros() - PrevTime) / 1000000.0); // Calculate delta time
    PrevTime = micros(); 
    //if(dt>100.0)dt=0.0001;    
    
    vTaskSuspendAll();
    noInterrupts();  
    MPU.init_Mag(); 
    MPU.ReadXYZ(); //read all data from MPU and normalize them  
    interrupts();
    xTaskResumeAll();
    
    //calculate kalman filter of x,y,z
    MPU_Angle_X = (kalmanX.getAngle((atan(MPU.get_Az() / sqrt((MPU.get_Ax() * MPU.get_Ax()) + (MPU.get_Ay() * MPU.get_Ay()))) * RAD_TO_DEG), -MPU.get_Gx(), dt))*DEG2RAD; // Calculate the angle using a Kalman filter    
    MPU_X = MPU_Angle_X + WEP[P_IMU_X_Angle_Offset];
    MPU_Angle_Y = (kalmanY.getAngle((atan2(MPU.get_Ax(), MPU.get_Ay()) * RAD_TO_DEG),  MPU.get_Gz(), dt))*DEG2RAD;  
    MPU_Y = MPU_Angle_Y + WEP[P_IMU_Y_Angle_Offset];
    MPU_Angle_Z = (0.5 * MPU_Angle_Z) + (0.5 * (atan2 ( MPU._getRaw_CX() , MPU._getRaw_CY() ) * RAD_TO_DEG));
    MPU_Z = MPU_Angle_Z * DEG2RAD; 
    
    //XQueue.forcePush(MPU_X);
    //YQueue.forcePush(MPU_Y);
    
    /*
    for(byte i=0; i<XQueue.size(); i++){
      dataArray[i]=XQueue[i];
    }
    pplsq(Ti, dataArray, XQueue.size(), coefficients, POLY_DEGREE); //fit and return squared residual
    MPU_X_P = Poly_Calculate(XQueue.size()+1, coefficients, POLY_DEGREE);
    */
    
    /*
    for(byte i=0; i<YQueue.size(); i++){
      dataArray[i]=YQueue[i];
    }
    pplsq(Ti, dataArray, YQueue.size(), coefficients, POLY_DEGREE); //fit and return squared residual
    MPU_Y_P = Poly_Calculate(YQueue.size()+1, coefficients, POLY_DEGREE);
    */
    
    //gyro lowpass
    Gyro_X= (WEP[P_Gyro_X_LowPass_Gain]*Gyro_X) + ((1-WEP[P_Gyro_X_LowPass_Gain])* -MPU.get_Gx());
    Gyro_Y= (WEP[P_Gyro_Y_LowPass_Gain]*Gyro_Y) + ((1-WEP[P_Gyro_Y_LowPass_Gain])*  MPU.get_Gz());
}

