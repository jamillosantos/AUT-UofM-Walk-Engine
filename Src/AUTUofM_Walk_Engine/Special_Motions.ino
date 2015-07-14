  
  void Motion_Stand_Up_Front(byte Robot_Num){
    // Ten Size Akbar=0  Asghar=1
    if (Robot_Num==0){
      Motion_Stand_Up_Front_1();
    }
    else if(Robot_Num==1){
      Motion_Stand_Up_Front_2();
    }
  }
  
  
  void Motion_Stand_Up_Back(byte Robot_Num){
    // Ten Size Akbar=0  Asghar=1
    if (Robot_Num==0){
      Motion_Stand_Up_Back_1();
    }
    else if(Robot_Num==1){
      Motion_Stand_Up_Back_2();
    }
  }
  
  void Run_R_Kik_Motion(byte Robot_Num){
    double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
    double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
    double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
    double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve
  
    //akbar
    if (Robot_Num==0){
      for(int i=0;i<=50;i++){
     delay(10);
    //right leg initialize
    L_Leg_Ik[I_X]     =0; 
    L_Leg_Ik[I_Y]     =-50;
    L_Leg_Ik[I_Z]     =0.0;
    L_Leg_Ik[I_Roll]  =0;
    L_Leg_Ik[I_Pitch] =0.00;
    L_Leg_Ik[I_Yaw]   =0.0;
    //left leh initialize
    R_Leg_Ik[I_X]     =0; 
    R_Leg_Ik[I_Y]     =60;
    R_Leg_Ik[I_Z]     =0;
    R_Leg_Ik[I_Roll]  =0;
    R_Leg_Ik[I_Pitch] =0.00;
    R_Leg_Ik[I_Yaw]   =0.0; 
    //right arm initialize
    L_Arm[I_A_Pitch]   =0.0;
    L_Arm[I_A_Roll]    =0.3;
    L_Arm[I_A_Elbow]   =0.0;
    L_Arm[I_A_Vp]      =0.1;
    L_Arm[I_A_Vr]      =0.1;
    L_Arm[I_A_Ve]      =0.1;
    //left arm initialize
    R_Arm[I_A_Pitch]   =0.0;
    R_Arm[I_A_Roll]    =0.0;
    R_Arm[I_A_Elbow]   =0.0;
    R_Arm[I_A_Vp]      =0.1;
    R_Arm[I_A_Vr]      =0.1;
    R_Arm[I_A_Ve]      =0.1; 
    //update robotis joints
    Update_Ik(0.02, 0.02, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
    }
    vTaskDelay(400);
    if (Robot_Num==0){
      for(double t=0; t<=Pi ;t+=0.032){       
        //right leg initialize
        R_Leg_Ik[I_X]     =(sin(2.3 * (t-1.5)) *(1.2 * 100.0));
        if ((t>=1) && (t<=2.8)) R_Leg_Ik[I_X]=220; 
        R_Leg_Ik[I_Y]     =(sin(t)*(0.7 * 100.0));
        R_Leg_Ik[I_Z]     =(t>=0.3) ? (sin((t*0.8)-0.01)  *(1 * 100.0)) : -15 ;
        R_Leg_Ik[I_Roll]  =(sin(t)*(-0.1));
        R_Leg_Ik[I_Pitch] =0.0;
        R_Leg_Ik[I_Yaw]   =0.0;
    
        //left leh initialize
        L_Leg_Ik[I_X]     =0.0; 
        L_Leg_Ik[I_Y]     =(sin(0.8*t)*(-0.95 * 100.0));
        L_Leg_Ik[I_Z]     =0.0;
        L_Leg_Ik[I_Roll]  =(sin(0.8*t)*(0.4));
        L_Leg_Ik[I_Pitch] =(sin(0.8*t)*(-0.05));
        L_Leg_Ik[I_Yaw]   =0.0;
    
        //right arm initialize
        R_Arm[I_A_Pitch]   = 0.0; //(cos(t)); // + Arm_Hopping_Val; // + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
        R_Arm[I_A_Roll]    = 0.0;
        R_Arm[I_A_Elbow]   = 0.0;
        R_Arm[I_A_Vp]      = 0.2;
        R_Arm[I_A_Vr]      = 0.05;
        R_Arm[I_A_Ve]      = 0.05;
    
        //left arm initialize
        L_Arm[I_A_Pitch]   = 0.0; // ((cos(t-Pi)) * vx * 1.3); // + Arm_Hopping_Val;// + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2); 
        L_Arm[I_A_Roll]    = 0.0;
        L_Arm[I_A_Elbow]   = 0.0;
        L_Arm[I_A_Vp]      = 0.2;
        L_Arm[I_A_Vr]      = 0.05;
        L_Arm[I_A_Ve]      = 0.05;
    
        //update robotis joints
        Update_Ik(1.0, 1.0, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm); 
        vTaskDelay(7);
      }//main gait timi for ins 
      Stand_Init_T(1.0, 10);
    }
    }
 
    //asghar
    else if(Robot_Num==1){
  
      for(double t=0; t<=Pi ;t+=0.027){       
        //right leg initialize
        R_Leg_Ik[I_X]     =(sin(2.3 * (t-1.5)) *(1.8 * 100.0));
        if ((t>=1.2) && (t<=1.7)) R_Leg_Ik[I_X]=180; 
        R_Leg_Ik[I_Y]     =(sin(t)*(0.8 * 110.0));
        R_Leg_Ik[I_Z]     =(t>=0.5) ? (sin((t*0.8)-0.01)  *(1.2 * 100.0)) : -15 ;
        R_Leg_Ik[I_Roll]  =(sin(t)*(-0.1));
        R_Leg_Ik[I_Pitch] =0.0;
        R_Leg_Ik[I_Yaw]   =0.0;
    
        //left leh initialize
        L_Leg_Ik[I_X]     =0.0; 
        L_Leg_Ik[I_Y]     =(sin(0.8*t)*(-0.95 * 100.0));
        L_Leg_Ik[I_Z]     =0.0;
        L_Leg_Ik[I_Roll]  =(sin(0.8*t)*(0.15));
        L_Leg_Ik[I_Pitch] =(sin(0.8*t)*(-0.05));
        L_Leg_Ik[I_Yaw]   =0.0;
    
        //right arm initialize
        R_Arm[I_A_Pitch]   = 0.0; //(cos(t)); // + Arm_Hopping_Val; // + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
        R_Arm[I_A_Roll]    = 0.0;
        R_Arm[I_A_Elbow]   = 0.0;
        R_Arm[I_A_Vp]      = 0.3;
        R_Arm[I_A_Vr]      = 0.05;
        R_Arm[I_A_Ve]      = 0.05;
    
        //left arm initialize
        L_Arm[I_A_Pitch]   = 0.0; // ((cos(t-Pi)) * vx * 1.3); // + Arm_Hopping_Val;// + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2); 
        L_Arm[I_A_Roll]    = 0.0;
        L_Arm[I_A_Elbow]   = 0.0;
        L_Arm[I_A_Vp]      = 0.2;
        L_Arm[I_A_Vr]      = 0.05;
        L_Arm[I_A_Ve]      = 0.05;
    
        //update robotis joints
        Update_Ik(1.0, 1.0, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm); 
        vTaskDelay(7);
      }//main gait timi for ins 
    }

  }
  
  
  
  
  
  void Run_L_Kik_Motion(byte Robot_Num){
    double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
    double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
    double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
    double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve
  
    // Ten Size Akbar=0  Asghar=1
    if (Robot_Num==0){
      for(int i=0;i<=50;i++){
     delay(10);
    //right leg initialize
    R_Leg_Ik[I_X]     =0; 
    R_Leg_Ik[I_Y]     =-90;
    R_Leg_Ik[I_Z]     =0.0;
    R_Leg_Ik[I_Roll]  =0;
    R_Leg_Ik[I_Pitch] =0.00;
    R_Leg_Ik[I_Yaw]   =0.0;
    //left leh initialize
    L_Leg_Ik[I_X]     =0; 
    L_Leg_Ik[I_Y]     =90;
    L_Leg_Ik[I_Z]     =0;
    L_Leg_Ik[I_Roll]  =0;
    L_Leg_Ik[I_Pitch] =0.00;
    L_Leg_Ik[I_Yaw]   =0.0; 
    //right arm initialize
    R_Arm[I_A_Pitch]   =0.0;
    R_Arm[I_A_Roll]    =0.3;
    R_Arm[I_A_Elbow]   =0.0;
    R_Arm[I_A_Vp]      =0.1;
    R_Arm[I_A_Vr]      =0.1;
    R_Arm[I_A_Ve]      =0.1;
    //left arm initialize
    L_Arm[I_A_Pitch]   =0.0;
    L_Arm[I_A_Roll]    =0.0;
    L_Arm[I_A_Elbow]   =0.0;
    L_Arm[I_A_Vp]      =0.1;
    L_Arm[I_A_Vr]      =0.1;
    L_Arm[I_A_Ve]      =0.1; 
    //update robotis joints
    Update_Ik(0.02, 0.02, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
    }
    vTaskDelay(400);
    
      for(double t=0; t<=Pi ;t+=0.030){       
        //right leg initialize
        L_Leg_Ik[I_X]     =(sin(2.3 * (t-1.5)) *(1 * 100.0));
        if ((t>=1) && (t<=2.8)) L_Leg_Ik[I_X]=200; 
        L_Leg_Ik[I_Y]     =(sin(t)*(1 * 100.0));
        L_Leg_Ik[I_Z]     =(t>=0.5) ? (sin((t*0.8)-0.01)  *(1.2 * 100.0)) : -10 ;
        L_Leg_Ik[I_Roll]  =(sin(t)*(-0.13));
        L_Leg_Ik[I_Pitch] =0.0;
        L_Leg_Ik[I_Yaw]   =0.0;
    
        //left leh initialize
        R_Leg_Ik[I_X]     =0.0; 
        R_Leg_Ik[I_Y]     =(sin(0.8*t)*(-1 * 100.0));
        R_Leg_Ik[I_Z]     =0.0;
        R_Leg_Ik[I_Roll]  =(sin(0.8*t)*(0.8));
        R_Leg_Ik[I_Pitch] =(sin(0.8*t)*(-0.07));
        R_Leg_Ik[I_Yaw]   =0.0;
    
        //right arm initialize
        L_Arm[I_A_Pitch]   = 0.0; //(cos(t)); // + Arm_Hopping_Val; // + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
        L_Arm[I_A_Roll]    = 0.0;
        L_Arm[I_A_Elbow]   = 0.0;
        L_Arm[I_A_Vp]      = 0.2;
        L_Arm[I_A_Vr]      = 0.05;
        L_Arm[I_A_Ve]      = 0.05;
    
        //left arm initialize
        R_Arm[I_A_Pitch]   = 0.0; // ((cos(t-Pi)) * vx * 1.3); // + Arm_Hopping_Val;// + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2); 
        R_Arm[I_A_Roll]    = 0.0;
        R_Arm[I_A_Elbow]   = 0.0;
        R_Arm[I_A_Vp]      = 0.2;
        R_Arm[I_A_Vr]      = 0.05;
        R_Arm[I_A_Ve]      = 0.05;
    
        //update robotis joints
        Update_Ik(1.0, 1.0, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm); 
        vTaskDelay(7);
      }//main gait timi for ins 
      Stand_Init_T(1.0, 10);
    }
 
    //asghar
    else if(Robot_Num==1){
  
      for(double t=0; t<=Pi ;t+=0.027){       
        //right leg initialize
        L_Leg_Ik[I_X]     =(sin(2.3 * (t-1.5)) *(1.8 * 100.0));
        if ((t>=1.2) && (t<=1.7)) L_Leg_Ik[I_X]=180; 
        L_Leg_Ik[I_Y]     =(sin(t)*(0.8 * 110.0));
        L_Leg_Ik[I_Z]     =(t>=0.5) ? (sin((t*0.8)-0.01)  *(1.2 * 100.0)) : -15 ;
        L_Leg_Ik[I_Roll]  =(sin(t)*(-0.1));
        L_Leg_Ik[I_Pitch] =0.0;
        L_Leg_Ik[I_Yaw]   =0.0;
    
        //left leh initialize
        R_Leg_Ik[I_X]     =0.0; 
        R_Leg_Ik[I_Y]     =(sin(0.8*t)*(-0.95 * 100.0));
        R_Leg_Ik[I_Z]     =0.0;
        R_Leg_Ik[I_Roll]  =(sin(0.8*t)*(0.15));
        R_Leg_Ik[I_Pitch] =(sin(0.8*t)*(-0.05));
        R_Leg_Ik[I_Yaw]   =0.0;
    
        //right arm initialize
        L_Arm[I_A_Pitch]   = 0.0; //(cos(t)); // + Arm_Hopping_Val; // + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
        L_Arm[I_A_Roll]    = 0.0;
        L_Arm[I_A_Elbow]   = 0.0;
        L_Arm[I_A_Vp]      = 0.3;
        L_Arm[I_A_Vr]      = 0.05;
        L_Arm[I_A_Ve]      = 0.05;
    
        //left arm initialize
        R_Arm[I_A_Pitch]   = 0.0; // ((cos(t-Pi)) * vx * 1.3); // + Arm_Hopping_Val;// + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2); 
        R_Arm[I_A_Roll]    = 0.0;
        R_Arm[I_A_Elbow]   = 0.0;
        R_Arm[I_A_Vp]      = 0.2;
        R_Arm[I_A_Vr]      = 0.05;
        R_Arm[I_A_Ve]      = 0.05;
    
        //update robotis joints
        Update_Ik(1.0, 1.0, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm); 
        vTaskDelay(7);
      }//main gait timi for ins 
    }
  }
  
  
  
  
  void Motion_Stand_Up_Front_1(){
    byte delay_x=60;
    double _Speed=0.3;
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    //22222***********************************************************************************************************
    for(int i=0;i<=15;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -35*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -35*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =0*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =0*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =0*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =0*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //33333***********************************************************************************************************
    for(int i=0;i<=15;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 75*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 110*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 75*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 110*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =80*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =80*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //444444**********************************************************************************************************
    for(int i=0;i<=15;i++){
      vTaskDelay(delay_x);
     //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 75*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 75*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =90*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =90*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-145*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-145*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];    
   
    
    }
    //555555***********************************************************************************************************
    _Speed=0.1;
    for(int i=0;i<=20;i++){
      vTaskDelay(delay_x);
      //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= 0.2;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -15*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= 0.2;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -15*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =45*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =45*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-145*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-145*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
  //***********************************************************************************************************
    for(int i=0;i<=50;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -5*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -5*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = 0.3;
    Speed[Id_Right_Knee]       = 0.01;;
    Speed[Id_Right_Foot_Pitch] = 0.01;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = 0.3;
    Speed[Id_Left_Knee]       = 0.01;;
    Speed[Id_Left_Foot_Pitch] = 0.01;
    Speed[Id_Left_Foot_Roll]  = _Speed;
  
    Angle[Id_Right_Hip_Pitch]  =50*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =50*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-85*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-85*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =50*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =50*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];   
   
    
    }
  //***********************************************************************************************************
  for(int cnt=0;cnt<300;cnt++)
    {
      Stand_Init(0.1);
      vTaskDelay(delay_x);
    } 
  } 
  
  
  
  
  
  
  
  
  void Motion_Stand_Up_Front_2(){
    byte delay_x=60;
    double _Speed=0.1;
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
      //111111111***********************************************************************************************************
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 0*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = 0*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 0*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = 0*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =0*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =0*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset]; 
    
    Angle[Id_Right_Knee]       =-0*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-0*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //22222***********************************************************************************************************
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -35*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = 0*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -35*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = 0*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =0*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =0*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =0*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =0*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //33333***********************************************************************************************************
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 75*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 110*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 75*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 110*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =80*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =80*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //444444**********************************************************************************************************
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
     //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 75*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 75*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =90*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =90*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-145*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-145*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];    
   
    
    }
    //555555***********************************************************************************************************
    
    for(int i=0;i<=60;i++){
      vTaskDelay(delay_x);
      //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= 0.2;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -10*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= 0.2;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -10*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =45*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =45*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-145*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-145*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
  //66666***********************************************************************************************************
    _Speed=0.01;
    for(int i=0;i<=20;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -20*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -20*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = 0.03;
    Speed[Id_Right_Knee]       = 0.01;
    Speed[Id_Right_Foot_Pitch] = 0.01;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = 0.30;
    Speed[Id_Left_Knee]       = 0.01;
    Speed[Id_Left_Foot_Pitch] = 0.01;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =40*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =40*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =75*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =75*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
      //777777***********************************************************************************************************
    
    for(int i=0;i<=20;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 5*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 5*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = 0.02;
    Speed[Id_Right_Knee]       = 0.01;;
    Speed[Id_Right_Foot_Pitch] = 0.01;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = 0.02;
    Speed[Id_Left_Knee]       = 0.01;;
    Speed[Id_Left_Foot_Pitch] = 0.01;
    Speed[Id_Left_Foot_Roll]  = _Speed;
  
    Angle[Id_Right_Hip_Pitch]  =50*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =50*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-85*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-85*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =50*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =50*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];   
   
    
    }
    //888888***********************************************************************************************************
  for(int cnt=0;cnt<100;cnt++)
    {
      Stand_Init(0.01);
      vTaskDelay(delay_x);
    } 
    for(int cnt=0;cnt<100;cnt++)
    {
      Stand_Init(0.1);
      vTaskDelay(delay_x);
    } 
  } 
   
  
  //____________________________________________________________________________________________________________
  //____________________________________________________________________________________________________________
  
  
  
  
  
  void  Motion_Stand_Up_Back_1(){
      byte delay_x=30;
    double _Speed=0.09;
    for(int i=0;i<=20;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 0*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = 0*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 0*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = 0*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =45*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =45*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =-45*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =-45*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //***********************************************************************************************************
    for(int i=0;i<=20;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = 0*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = 0*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =90*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =90*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =-45*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =-45*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //***********************************************************************************************************
      for(int i=0;i<=20;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =90*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =90*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =-45*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =-45*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //***********************************************************************************************************
    for(int i=0;i<=20;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =90*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =90*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =-45*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =-45*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //***********************************************************************************************************
  
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =35*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =35*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =30*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =30*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset]; 
   
    
    }
      //***********************************************************************************************************
  
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =-25*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =-25*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset]; 
   
    
    }
    
    //***********************************************************************************************************
   _Speed=0.01;
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -30*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -30*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =15*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =15*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];    
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =80*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =80*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset]; 
   
  
    
    }
      //888888***********************************************************************************************************
    
    _Speed=0.01;
    for(int i=0;i<=20;i++){
      vTaskDelay(delay_x*2);
      //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= 0.2;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -15*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= 0.2;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -15*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = 0.03;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = 0.03;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =15*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =15*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =80*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =80*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
  //***********************************************************************************************************
  //***********************************************************************************************************
    for(int i=0;i<=50;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -5*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -5*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = 0.03;
    Speed[Id_Right_Knee]       = 0.01;;
    Speed[Id_Right_Foot_Pitch] = 0.01;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = 0.03;
    Speed[Id_Left_Knee]       = 0.01;;
    Speed[Id_Left_Foot_Pitch] = 0.01;
    Speed[Id_Left_Foot_Roll]  = _Speed;
  
    Angle[Id_Right_Hip_Pitch]  =35*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =35*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-85*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-85*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =50*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =50*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];   
   
    
    }
  //***********************************************************************************************************
    //***********************************************************************************************************
    for(int i=0;i<=50;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 20*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 45*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 20*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 45*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = 0.03;
    Speed[Id_Right_Knee]       = 0.01;;
    Speed[Id_Right_Foot_Pitch] = 0.01;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = 0.03;
    Speed[Id_Left_Knee]       = 0.01;;
    Speed[Id_Left_Foot_Pitch] = 0.01;
    Speed[Id_Left_Foot_Roll]  = _Speed;
  
    Angle[Id_Right_Hip_Pitch]  =35*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =35*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-75*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-75*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =35*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =35*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];   
   
    
    }
  //***********************************************************************************************************
  for(int cnt=0;cnt<300;cnt++)
    {
      Stand_Init(0.02);
      vTaskDelay(delay_x);
    }
  
  }
  
  
  
  
  
  
  
  
  
  
  
  //____________________________________________________________________________________________________________
  //____________________________________________________________________________________________________________
  
  
  
  
  
  
  
  
  
  
  void  Motion_Stand_Up_Back_2(){
      byte delay_x=30;
    double _Speed=0.09;
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 0*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = 0*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 0*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = 0*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =45*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =45*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =-45*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =-45*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //***********************************************************************************************************
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = 0*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = 0*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =90*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =90*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =-45*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =-45*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //***********************************************************************************************************
      for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =90*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =90*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =-45*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =-45*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //***********************************************************************************************************
    for(int i=0;i<=30;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 90*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 90*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =90*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =90*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-90*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-90*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =-45*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =-45*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
    //***********************************************************************************************************
  
    for(int i=0;i<=50;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =35*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =35*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =30*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =30*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset]; 
   
    
    }
      //***********************************************************************************************************
  
    for(int i=0;i<=50;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -90*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =-25*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =-25*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset]; 
   
    
    }
    
    //***********************************************************************************************************
   
    for(int i=0;i<=50;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 0*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -90*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 0*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -90*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = _Speed;
    Speed[Id_Right_Knee]       = _Speed;
    Speed[Id_Right_Foot_Pitch] = _Speed;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = _Speed;
    Speed[Id_Left_Knee]       = _Speed;
    Speed[Id_Left_Foot_Pitch] = _Speed;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =35*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =35*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];    
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =85*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =85*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset]; 
   
  
    
    }
      //888888***********************************************************************************************************
    _Speed=0.01;
    for(int i=0;i<=50;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= -20*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= -20*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = 0.03;
    Speed[Id_Right_Knee]       = 0.01;
    Speed[Id_Right_Foot_Pitch] = 0.01;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = 0.30;
    Speed[Id_Left_Knee]       = 0.01;
    Speed[Id_Left_Foot_Pitch] = 0.01;
    Speed[Id_Left_Foot_Roll]  = _Speed;
    
    Angle[Id_Right_Hip_Pitch]  =40*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =40*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-125*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-125*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =75*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =75*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];  
   
    
    }
      //888888***********************************************************************************************************
    _Speed=0.01;  
    for(int i=0;i<=50;i++){
      vTaskDelay(delay_x);
    //all speed are betwin 0~1
    //al joint are betwin -3.14 ~ 3.14
    //right arm joints update
    Speed[Id_Right_Arm_Pitch]= _Speed;  
    Speed[Id_Right_Arm_Roll] = _Speed;
    Speed[Id_Right_Arm_Elbow]= _Speed;
    
    Angle[Id_Right_Arm_Pitch]= 5*DEG2RAD;
    Angle[Id_Right_Arm_Roll] = -85*DEG2RAD ;
    Angle[Id_Right_Arm_Elbow]= 0*DEG2RAD;
  
    //left arm joints update  
    Speed[Id_Left_Arm_Pitch]= _Speed;
    Speed[Id_Left_Arm_Roll] = _Speed;
    Speed[Id_Left_Arm_Elbow]= _Speed;
    
    Angle[Id_Left_Arm_Pitch]= 5*DEG2RAD;
    Angle[Id_Left_Arm_Roll] = -85*DEG2RAD;
    Angle[Id_Left_Arm_Elbow]= 0*DEG2RAD;
    
   
    //set right leg speeds
    Speed[Id_Right_Hip_Yaw]    = _Speed;
    Speed[Id_Right_Hip_Roll]   = _Speed;
    Speed[Id_Right_Hip_Pitch]  = 0.02;
    Speed[Id_Right_Knee]       = 0.01;;
    Speed[Id_Right_Foot_Pitch] = 0.01;
    Speed[Id_Right_Foot_Roll]  = _Speed;
  
    //set left leg speeds
    Speed[Id_Left_Hip_Yaw]    = _Speed;
    Speed[Id_Left_Hip_Roll]   = _Speed;
    Speed[Id_Left_Hip_Pitch]  = 0.02;
    Speed[Id_Left_Knee]       = 0.01;;
    Speed[Id_Left_Foot_Pitch] = 0.01;
    Speed[Id_Left_Foot_Roll]  = _Speed;
  
    Angle[Id_Right_Hip_Pitch]  =50*DEG2RAD + WEP[P_Right_Leg_Hip_Pitch_Offset];
    Angle[Id_Left_Hip_Pitch]   =50*DEG2RAD + WEP[P_Left_Leg_Hip_Pitch_Offset];  
    
    Angle[Id_Right_Hip_Yaw]   = 0 + WEP[P_Right_Leg_Hip_Yaw_Offset];  
    Angle[Id_Left_Hip_Yaw]    = 0 + WEP[P_Left_Leg_Hip_Yaw_Offset];    
    
    Angle[Id_Right_Hip_Roll]  = 0 + WEP[P_Right_Leg_Hip_Roll_Offset];
    Angle[Id_Left_Hip_Roll]   = 0 + WEP[P_Left_Leg_Hip_Roll_Offset];  
      
    Angle[Id_Right_Foot_Roll] = 0 + WEP[P_Right_Leg_Foot_Roll_Offset]; 
    Angle[Id_Left_Foot_Roll]  = 0 + WEP[P_Left_Leg_Foot_Roll_Offset];  
    
    Angle[Id_Right_Knee]       =-85*DEG2RAD + WEP[P_Right_Leg_Knee_Offset]; 
    Angle[Id_Left_Knee]        =-85*DEG2RAD + WEP[P_Left_Leg_Knee_Offset]; 
    
    Angle[Id_Right_Foot_Pitch] =50*DEG2RAD + WEP[P_Right_Leg_Foot_Pitch_Offset];
    Angle[Id_Left_Foot_Pitch]  =50*DEG2RAD + WEP[P_Left_Leg_Foot_Pitch_Offset];   
   
    
    }
    //888888***********************************************************************************************************
  for(int cnt=0;cnt<300;cnt++)
    {
      Stand_Init(0.01);
      vTaskDelay(delay_x);
    } 
  
  }
  

