//Walk_Engine Main Task
void vWalk_Engine_Task( void *pvParameters ){
    
  vTaskSuspendAll();
  //Configure Robot Walk Engine... 
  Init_Robot_First();
  Vx=0.0001;        //velocity of X (forward) direction from PC (min -1 to max 1)
  Vy=0.0;        //velocity of Y (sideward) direction
  Vt=0.0;        //velocity of T (rotate) speed (per radian)  
  Buzzer(200);
  xTaskResumeAll();
  
  Stand_Init_T(0.05,1);
  vTaskDelay(1000);
  //stand for first time
  Stand_Init_T(0.05,300);
  
  //main task loop
  for( ;; ){ 
    togglePin(RED_LED_485EXP);
    
    if ((((Vx!=0) ||(Vy!=0)||(Vt!=0)) && (Motion_Ins==No_Motion) && (Internal_Motion_Request==No_Motion )) && (System_Voltage>=(int)WEP[P_Min_Voltage_Limit])) {
       //start gait
       WEP[P_Left_Leg_Y_Offset]=-50;
       WEP[P_Right_Leg_Y_Offset]=50;
       Stand_Init_T(1.0,3);  // last one was 2
       Set_Walk_Engine_Parameters((byte)Teen_Size_Robot_Num);
       Omni_Gait(WEP[Vx_Offset],WEP[Vy_Offset],WEP[Vt_Offset]); //execute omni-directional start gait
       
       //main gait
       while((((Vx!=0) ||(Vy!=0)||(Vt!=0)) && (Motion_Ins==No_Motion) && (Internal_Motion_Request==No_Motion )) && (System_Voltage>=(int)WEP[P_Min_Voltage_Limit])){
         Omni_Gait(Vx+WEP[Vx_Offset],Vy+WEP[Vy_Offset],Vt+WEP[Vt_Offset]); //execute omni-directional gait 
         togglePin(RED_LED_485EXP);
       }
       
       //finish gate
       Omni_Gait(WEP[Vx_Offset],WEP[Vy_Offset],WEP[Vt_Offset]); //execute omni-directional end gait
    }
    else{
    
      //firts run gait generation with vx=vy=vt=0 for semi gait generation to stand position
      if (Internal_Motion_Request==No_Motion ){
        Stand_Init(0.5);
      }
      
      if (Internal_Motion_Request==Stop_Motion ){
        Stand_Init(0.05);
      }
      
      vTaskDelay(20);
      
      //run standup motions
      switch(Internal_Motion_Request) { //check for instrcation
            case (byte)Stand_Up_Front: 
                 //run stand up
                 Stand_Init_T(1.0, 50);
                 Check_Robot_Fall=0;
                 Actuators_Update=1;
                 Internal_Motion_Request=No_Motion;
                 Motion_Stand_Up_Front((byte)Teen_Size_Robot_Num);
                 Check_Robot_Fall=1;
                 Stand_Init_T(1.0, 50);
                 break;
            case (byte)Stand_Up_Back:
                 //run stand up 
                 Stand_Init_T(1.0, 50);
                 Check_Robot_Fall=0;
                 Actuators_Update=1;
                 Internal_Motion_Request=No_Motion;
                 Motion_Stand_Up_Back((byte)Teen_Size_Robot_Num);
                 Check_Robot_Fall=1;
                 Stand_Init_T(1.0, 50);              
                 break;
      }
      
      //run the motion
      switch(Motion_Ins){  //check for instrcation
            case (byte)Motion_1:{
                 //run motion 1
                 Stand_Init_T(1.0, 20);
                 Run_R_Kik_Motion((byte)Teen_Size_Robot_Num);
                 Stand_Init_T(1.0, 20);
                 Motion_Ins=No_Motion;
                 }
                 break;
            case (byte)Motion_2:
                 //run motion 2
                 Stand_Init_T(1.0, 20);
                 Run_L_Kik_Motion((byte)Teen_Size_Robot_Num);
                 Stand_Init_T(1.0, 20);
                 Motion_Ins=No_Motion;
                 break;
            case (byte)Motion_3:
                 Stand_Init_T(1.0, 10);
                 
                 Motion_Ins=No_Motion; 
                 break;
            case (byte)Motion_4:
                 Stand_Init_T(1.0, 10);
                 
                 Motion_Ins=No_Motion;
                 break;
            case (byte)Motion_5:
                 
                 Motion_Ins=No_Motion;
                 break;
            case (byte)Motion_6:
                 
                 Motion_Ins=No_Motion;
                 break;
            case (byte)Motion_7:
                 
                 Motion_Ins=No_Motion;
                 break;
            case (byte)Motion_8:
                 
                 Motion_Ins=No_Motion;
                 break;
            case (byte)Motion_9:
                 
                 Motion_Ins=No_Motion;
                 break;
            case (byte)Motion_10:
            
                 Motion_Ins=No_Motion;
                 break;
        }//external motion request switch           
    }//main if/else
  }
}

//omni directional gaite generation
void Omni_Gait(double vx, double vy, double vt){
  double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
  double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
  double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
  double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve
  
  double Joint_Speed = 0.3;
  
  //gait generate with for loop form  0~3.14
  for(double t=0; t<=TwoPi ;t+=WEP[P_Motion_Resolution]){ 
   
    if (Vx >  0.5)  Vx=  0.5;
    if (Vx < -0.5)  Vx= -0.5;
    
    if (Vy >  0.5)  Vy=  0.5;
    if (Vy < -0.5)  Vy= -0.5;
    
    if (Vt >  0.1)  Vt=  0.1;
    if (Vt < -0.1)  Vt= -0.1;
     
    //-----------------------
    if ( ((t>=Pi-WEP[P_Motion_Resolution])&&(t<=Pi+WEP[P_Motion_Resolution])) || ((t>=TwoPi-WEP[P_Motion_Resolution])&&(t<=TwoPi+WEP[P_Motion_Resolution])) ){
        vTaskDelay(WEP[P_Double_Support_Sleep]);    
    }
        
    if ( ((t>=(Pi/2.0)-WEP[P_Motion_Resolution])&&(t<=(Pi/2.0)+WEP[P_Motion_Resolution])) || ((t>=((3.0*Pi)/2.0)-WEP[P_Motion_Resolution])&&(t<=((3.0*Pi)/2.0)+WEP[P_Motion_Resolution])) ){ 
        vTaskDelay(WEP[P_Single_Support_Sleep]);
    }
    
    //right leg initialize
    //if t<pi the right leg in fly state and other wise in support state
    R_Leg_Ik[I_X]     = (-(cos(t)*(vx*100.0)))+ (vx * WEP[P_Body_X_Swing_Gain] * 100.0); 
    R_Leg_Ik[I_Y]     = (t<=Pi) ? (sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0)) + (sin(t)*(WEP[P_Fly_Y_Swing_Gain]*100.0)) + (cos(t-Pi)*(vy*50.0)) 
                        : (sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0))+(cos(t-Pi)*(vy*50.0))+(sin(t)*WEP[P_Support_Y_Swing_Gain]);
    R_Leg_Ik[I_Z]     = (t<=Pi) ? (sin(t)*(WEP[P_Fly_Z_Swing_Gain]*100.0)) : (sin(t)*(WEP[P_Support_Z_Swing_Gain]*100.0));
    R_Leg_Ik[I_Roll]  = (t<=Pi) ? (-sin(t)*(WEP[P_Fly_Roll_Gain])) : (-sin(t)*(WEP[P_Support_Roll_Gain])); 
    R_Leg_Ik[I_Pitch] = (t<=Pi) ? 0.0 : (sin(t)*WEP[P_Support_Pitch_Gain]);
    R_Leg_Ik[I_Yaw]   = (cos(t-Pi)*(vt)); //(cos(t-Pi)*(vt));
  
    //left leh initialize
    L_Leg_Ik[I_X]     = (-(cos(t-Pi)*(vx*100.0)))+ (vx * WEP[P_Body_X_Swing_Gain] * 100.0);
    L_Leg_Ik[I_Y]     = (t>=Pi) ? (-sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0)) + (-sin(t)*(WEP[P_Fly_Y_Swing_Gain]*100.0)) +(cos(t-Pi)*(vy*50.0)) 
                        : (-sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0))+(cos(t-Pi)*(vy*50.0))+(sin(t)*WEP[P_Support_Y_Swing_Gain]);
    L_Leg_Ik[I_Z]     = (t>=Pi) ? ((sin(t-Pi))*(WEP[P_Fly_Z_Swing_Gain]*100.0)) : ((sin(t-Pi))*(WEP[P_Support_Z_Swing_Gain]*100.0));
    L_Leg_Ik[I_Roll]  = (t>=Pi) ? (sin(t)*(WEP[P_Fly_Roll_Gain])) : (sin(t)*(WEP[P_Support_Roll_Gain])); 
    L_Leg_Ik[I_Pitch] = (t>=Pi) ? 0.0 : (sin(t-Pi)*WEP[P_Support_Pitch_Gain]);
    L_Leg_Ik[I_Yaw]   = (cos(t-Pi)*(vt)); //(cos(t-Pi)*(vt));
    
    //right arm initialize
    R_Arm[I_A_Pitch]   = ((cos(t)) * vx * 1.3); // + Arm_Hopping_Val; // + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
    R_Arm[I_A_Roll]    = 0.0;
    R_Arm[I_A_Elbow]   = 0.0;
    R_Arm[I_A_Vp]      = 0.2;
    R_Arm[I_A_Vr]      = 0.05;
    R_Arm[I_A_Ve]      = 0.05;
  
    //left arm initialize
    L_Arm[I_A_Pitch]   = ((cos(t-Pi)) * vx * 1.3); // + Arm_Hopping_Val;// + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2); 
    L_Arm[I_A_Roll]    = 0.0;
    L_Arm[I_A_Elbow]   = 0.0;
    L_Arm[I_A_Vp]      = 0.2;
    L_Arm[I_A_Vr]      = 0.05;
    L_Arm[I_A_Ve]      = 0.05;
  
    //update robotis joints
    Update_Ik(Joint_Speed, Joint_Speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm); 
    vTaskDelay(WEP[P_Gait_Frequency]*100);
  }//main gait timi for ins
}


//get robot state for fall detection
byte Get_Robot_State(double Roll, double Pitch){
  if(Roll>=WEP[P_Fall_Roll_Thershold]){
    return Fallen_Left;
  }
  
  if(Roll<=(-WEP[P_Fall_Roll_Thershold])){
    return Fallen_Right;
  }
    
  if(Pitch>=WEP[P_Fall_Pitch_Thershold]){
    return Fallen_Front;
  }
      
  if(Pitch<=(-WEP[P_Fall_Pitch_Thershold])){
    return Fallen_Back;
  }
    
  return Normal_Stand;  
}

//check robot state and run the stand function
void Robot_State(){
    switch (Get_Robot_State(MPU_Y, MPU_X)){
      case (byte)Fallen_Front:{
           //run stand motion from front
           Internal_Motion_Request=Stand_Up_Front;
           }
           break;
      case (byte)Fallen_Back:{
           //run stand motion from front
           Internal_Motion_Request=Stand_Up_Back;
           }
           break;
      case (byte)Fallen_Right:{
           //run stand motion from front
           Internal_Motion_Request=Stand_Up_Front;  //?????????
           }
           break;  
      case (byte)Fallen_Left:{
           //run stand motion from front
           Internal_Motion_Request=Stand_Up_Back;  //??????
           }
           break;  
      case (byte)Normal_Stand:
           //Internal_Motion_Request 
           break; 
    }//switch case
}

void Init_Robot_First(){
    
  Stand_Init(0.1);  //ititialie robot to stand position 
  
  Vx=0.0;        //velocity of X (forward) direction from PC (min -1 to max 1)
  Vy=0.0;        //velocity of Y (sideward) direction
  Vt=0.0;        //velocity of T (rotate) speed (per radian)
  
  Motion_Ins = No_Motion;
  Internal_Motion_Request = No_Motion;
}

//initialize robot to stand
void Stand_Init(double _Speed){
  double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
  double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
  double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
  double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve

  //right leg initialize
  R_Leg_Ik[I_X]     =0.0; 
  R_Leg_Ik[I_Y]     =0.0;
  R_Leg_Ik[I_Z]     =0.0;
  R_Leg_Ik[I_Roll]  =0.0;
  R_Leg_Ik[I_Pitch] =0.0;
  R_Leg_Ik[I_Yaw]   =0.0;
  
  //left leh initialize
  L_Leg_Ik[I_X]     =0.0; 
  L_Leg_Ik[I_Y]     =0.0;
  L_Leg_Ik[I_Z]     =0.0;
  L_Leg_Ik[I_Roll]  =0.0;
  L_Leg_Ik[I_Pitch] =0.0;
  L_Leg_Ik[I_Yaw]   =0.0;
  
  //right arm initialize
  R_Arm[I_A_Pitch]   =0.0;
  R_Arm[I_A_Roll]    =0.0;
  R_Arm[I_A_Elbow]   =0.0;
  R_Arm[I_A_Vp]      =_Speed;
  R_Arm[I_A_Vr]      =_Speed;
  R_Arm[I_A_Ve]      =_Speed;
  
  //left arm initialize
  L_Arm[I_A_Pitch]   =0.0;
  L_Arm[I_A_Roll]    =0.0;
  L_Arm[I_A_Elbow]   =0.0;
  L_Arm[I_A_Vp]      =_Speed;
  L_Arm[I_A_Vr]      =_Speed;
  L_Arm[I_A_Ve]      =_Speed;
  
  //update robotis joints
  Update_Ik(_Speed, _Speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
}

//initialize robot to stand
void Stand_Init_T(double _Speed, unsigned int T){
  double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
  double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
  double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
  double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve

  //update robotis joints
  for(unsigned int Cnt=0;Cnt<=T;Cnt++){
    //right leg initialize
    R_Leg_Ik[I_X]     =0.0; 
    R_Leg_Ik[I_Y]     =0.0;
    R_Leg_Ik[I_Z]     =0.0;
    R_Leg_Ik[I_Roll]  =0.0;
    R_Leg_Ik[I_Pitch] =0.0;
    R_Leg_Ik[I_Yaw]   =0.0;
  
    //left leh initialize
    L_Leg_Ik[I_X]     =0.0; 
    L_Leg_Ik[I_Y]     =0.0;
    L_Leg_Ik[I_Z]     =0.0;
    L_Leg_Ik[I_Roll]  =0.0;
    L_Leg_Ik[I_Pitch] =0.0;
    L_Leg_Ik[I_Yaw]   =0.0;
    
    //right arm initialize
    R_Arm[I_A_Pitch]   =0.0;
    R_Arm[I_A_Roll]    =0.0;
    R_Arm[I_A_Elbow]   =0.0;
    R_Arm[I_A_Vp]      =_Speed;
    R_Arm[I_A_Vr]      =_Speed;
    R_Arm[I_A_Ve]      =_Speed;
    
    //left arm initialize
    L_Arm[I_A_Pitch]   =0.0;
    L_Arm[I_A_Roll]    =0.0;
    L_Arm[I_A_Elbow]   =0.0;
    L_Arm[I_A_Vp]      =_Speed;
    L_Arm[I_A_Vr]      =_Speed;
    L_Arm[I_A_Ve]      =_Speed;
  
    Update_Ik(_Speed, _Speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
    vTaskDelay(20);
  }
}
