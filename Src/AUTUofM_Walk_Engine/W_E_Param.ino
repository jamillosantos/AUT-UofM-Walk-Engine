void Set_Walk_Engine_Parameters(byte Robot_Num){
  
  //defult value for head joint speed
  WEP[P_Head_Pan_Speed] =1000;
  WEP[P_Head_Tilt_Speed]=1000;
  
  WEP[P_Min_Voltage_Limit]=130; //128 voltage minimum for loop and buzzer error
  
  //fall thershold
  WEP[P_Fall_Roll_Thershold] =0.5;              
  WEP[P_Fall_Pitch_Thershold]=0.5; 
    
  //akbar
  if (Robot_Num==0){
    WEP[P_Leg_Length]=471.0;
    
    WEP[Vx_Offset]=-0.05;
    WEP[Vy_Offset]=0;
    WEP[Vt_Offset]=-0.009;
    
    //Walk Engine Parameters
    WEP[P_Motion_Resolution]=0.25; //0.15
    WEP[P_Gait_Frequency]=0.25;  //0.15                    
    WEP[P_Double_Support_Sleep]=40;               
    WEP[P_Single_Support_Sleep]=0;                 
    WEP[P_Fly_Roll_Gain]=0.005; //-0.01                       
    WEP[P_Fly_Pitch_Gain]=0.8;                     
    WEP[P_Fly_Yaw_Gain]=0;                     
    WEP[P_Fly_X_Swing_Gain]=0;                  
    WEP[P_Fly_Y_Swing_Gain]=0.05;                  
    WEP[P_Fly_Z_Swing_Gain]=0.75;                   
    WEP[P_Support_Roll_Gain]=0.15;               
    WEP[P_Support_Pitch_Gain]=0;                
    WEP[P_Support_Yaw_Gain]=0;               
    WEP[P_Support_X_Swing_Gain]=0;           
    WEP[P_Support_Y_Swing_Gain]=0;            
    WEP[P_Support_Z_Swing_Gain]=0.0;              
    WEP[P_Body_X_Swing_Gain]=0;                   
    WEP[P_Body_Y_Swing_Gain]=0.15; //0.04                 
    WEP[P_Body_Z_Swing_Gain]=0;                   

    //stablization parameters
    WEP[P_Stablizer_Arm_Pitch_Gain]=-2.2;   //-2.5       
    WEP[P_Stablizer_Arm_Roll_Gain]=1.2;      //2       
    WEP[P_Stablizer_Arm_Elbow_Gain]=0;           
    WEP[P_Stablizer_Hip_Roll_Gain]=0;   //-1.5        
    WEP[P_Stablizer_Hip_Pitch_Gain]=-0.2; //-1           
    WEP[P_Stablizer_Knee_Gain]=0; //-1               
    WEP[P_Stablizer_Foot_Pitch_Gain]=-0.3;  //-1          
    WEP[P_Stablizer_Foot_Roll_Gain]=0; //-1           
    WEP[P_Stablizer_COM_X_Shift_Gain]=10; //200         
    WEP[P_Stablizer_COM_Y_Shift_Gain]=30;//200        

    WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]=-0.000001;  //0.001      
    WEP[P_Gyro_Stablizer_Arm_Roll_Gain]=0;       
    WEP[P_Gyro_Stablizer_Arm_Elbow_Gain]=0;      
    WEP[P_Gyro_Stablizer_Hip_Roll_Gain]=0;      
    WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]=0;     
    WEP[P_Gyro_Stablizer_Knee_Gain]=0;           
    WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]=-0.00001; //-0.001     
    WEP[P_Gyro_Stablizer_Foot_Roll_Gain]=0;     
    WEP[P_Gyro_Stablizer_COM_X_Shift_Gain]=0;//0.5     
    WEP[P_Gyro_Stablizer_COM_Y_Shift_Gain]=0;   

    //hopping gait gain
    WEP[P_Stablizer_Hopping_Gait_X_Gain]=0;     
    WEP[P_Stablizer_Hopping_Gait_Y_Gain]=0;     

    //both leg offset in inverse kinematic (body COM)
    WEP[P_COM_X_offset]=0;                       
    WEP[P_COM_Y_offset]=25;                       
    WEP[P_COM_Z_offset]=80;                       
    WEP[P_COM_Roll_offset]=0;                   
    WEP[P_COM_Pitch_offset]=0;                   
    WEP[P_COM_Yaw_offset]=0;                   
 
    //legs joints offset 
    WEP[P_Left_Leg_Hip_Yaw_Offset]=0;           
    WEP[P_Left_Leg_Hip_Roll_Offset]=0;          
    WEP[P_Left_Leg_Hip_Pitch_Offset]=0.1;          
    WEP[P_Left_Leg_Knee_Offset]=0;              
    WEP[P_Left_Leg_Foot_Pitch_Offset]=0;         
    WEP[P_Left_Leg_Foot_Roll_Offset]=0;           

    WEP[P_Right_Leg_Hip_Yaw_Offset]=0;          
    WEP[P_Right_Leg_Hip_Roll_Offset]=0;          
    WEP[P_Right_Leg_Hip_Pitch_Offset]=0.1;         
    WEP[P_Right_Leg_Knee_Offset]=0;             
    WEP[P_Right_Leg_Foot_Pitch_Offset]=0;        
    WEP[P_Right_Leg_Foot_Roll_Offset]=0;         

    //Left leg inverse kinematic offset 
    WEP[P_Left_Leg_X_Offset]=0;                
    WEP[P_Left_Leg_Y_Offset]=10;                
    WEP[P_Left_Leg_Z_Offset]=-3;               
    WEP[P_Left_Leg_Roll_Offset]=0;              
    WEP[P_Left_Leg_Pitch_Offset]=0;            
    WEP[P_Left_Leg_Yaw_Offset]=0;             

    WEP[P_Right_Leg_X_Offset]=0;              
    WEP[P_Right_Leg_Y_Offset]=10;             
    WEP[P_Right_Leg_Z_Offset]=0;             
    WEP[P_Right_Leg_Roll_Offset]=0;             
    WEP[P_Right_Leg_Pitch_Offset]=0;            
    WEP[P_Right_Leg_Yaw_Offset]=0;             
    
    WEP[P_R_Arm_Pitch_offset]=-0.2;               
    WEP[P_R_Arm_Roll_offset]=-1.45;               
    WEP[P_R_Arm_Elbow_offset]=1.1;              
    
    WEP[P_L_Arm_Pitch_offset]=-0.2;                
    WEP[P_L_Arm_Roll_offset]=-1.45;               
    WEP[P_L_Arm_Elbow_offset]=1.1;               
                
    //imu offset
    WEP[P_IMU_X_Angle_Offset]= 0.04;              
    WEP[P_IMU_Y_Angle_Offset]=-0.03;               
    
    //MPU filtering parametrs 
    WEP[P_Gyro_X_LowPass_Gain]=0.85;              
    WEP[P_Gyro_Y_LowPass_Gain]=0.7;             
    
    //kalman filter r mesurement value
    WEP[P_Kalman_Roll_RM_Rate]=500;               
    WEP[P_Kalman_Pitch_RM_Rate]=200;             
    WEP[P_Kalman_Yaw_RM_Rate]=50;                
    
    //smoothing ratio
    WEP[P_Vx_Smoothing_Ratio]=0;               
    WEP[P_Vy_Smoothing_Ratio]=0;               
    WEP[P_Vt_Smoothing_Ratio]=0;
  }
  //--------------------------------------------------------
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  //asghar
  //********************************************************
  else if (Robot_Num==1){
    WEP[P_Leg_Length]=365.0;
    
    WEP[Vx_Offset]=-0.1;
    WEP[Vy_Offset]=0;
    WEP[Vt_Offset]=0;
    
    //Walk Engine Parameters
    WEP[P_Motion_Resolution]=0.25; //0.15
    WEP[P_Gait_Frequency]=0.25;  //0.15                    
    WEP[P_Double_Support_Sleep]=40;               
    WEP[P_Single_Support_Sleep]=0;                 
    WEP[P_Fly_Roll_Gain]=0.005; //-0.01                       
    WEP[P_Fly_Pitch_Gain]=0.8;                     
    WEP[P_Fly_Yaw_Gain]=0;                     
    WEP[P_Fly_X_Swing_Gain]=0;                  
    WEP[P_Fly_Y_Swing_Gain]=0.05;                  
    WEP[P_Fly_Z_Swing_Gain]=0.7;                   
    WEP[P_Support_Roll_Gain]=0.1;               
    WEP[P_Support_Pitch_Gain]=0;                
    WEP[P_Support_Yaw_Gain]=0;               
    WEP[P_Support_X_Swing_Gain]=0;           
    WEP[P_Support_Y_Swing_Gain]=0;            
    WEP[P_Support_Z_Swing_Gain]=0.0;              
    WEP[P_Body_X_Swing_Gain]=0;                   
    WEP[P_Body_Y_Swing_Gain]=0.2; //0.15                
    WEP[P_Body_Z_Swing_Gain]=0;                  

    //stablization parameters
    WEP[P_Stablizer_Arm_Pitch_Gain]=-3.0;   //-2.5       
    WEP[P_Stablizer_Arm_Roll_Gain]=1.2;      //2       
    WEP[P_Stablizer_Arm_Elbow_Gain]=0;           
    WEP[P_Stablizer_Hip_Roll_Gain]=0;   //-1.5        
    WEP[P_Stablizer_Hip_Pitch_Gain]=-0.2; //-1           
    WEP[P_Stablizer_Knee_Gain]=0; //-1               
    WEP[P_Stablizer_Foot_Pitch_Gain]=-0.4;  //-1          
    WEP[P_Stablizer_Foot_Roll_Gain]=0; //-1           
    WEP[P_Stablizer_COM_X_Shift_Gain]=10; //200         
    WEP[P_Stablizer_COM_Y_Shift_Gain]=30;//200        

    WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]=-0.00001;  //0.001      
    WEP[P_Gyro_Stablizer_Arm_Roll_Gain]=0;       
    WEP[P_Gyro_Stablizer_Arm_Elbow_Gain]=0;      
    WEP[P_Gyro_Stablizer_Hip_Roll_Gain]=0;      
    WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]=0;     
    WEP[P_Gyro_Stablizer_Knee_Gain]=0;           
    WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]=-0.000002; //-0.001     
    WEP[P_Gyro_Stablizer_Foot_Roll_Gain]=0;     
    WEP[P_Gyro_Stablizer_COM_X_Shift_Gain]=0;//0.5     
    WEP[P_Gyro_Stablizer_COM_Y_Shift_Gain]=0;   

    //hopping gait gain
    WEP[P_Stablizer_Hopping_Gait_X_Gain]=0;     
    WEP[P_Stablizer_Hopping_Gait_Y_Gain]=0;     

    //both leg offset in inverse kinematic (body COM)
    WEP[P_COM_X_offset]=0;                       
    WEP[P_COM_Y_offset]=25;                       
    WEP[P_COM_Z_offset]=60;                       
    WEP[P_COM_Roll_offset]=0;                   
    WEP[P_COM_Pitch_offset]=0.0;                   
    WEP[P_COM_Yaw_offset]=0.0;                   
 
    //legs joints offset 
    WEP[P_Left_Leg_Hip_Yaw_Offset]=0;           
    WEP[P_Left_Leg_Hip_Roll_Offset]=0;          
    WEP[P_Left_Leg_Hip_Pitch_Offset]=0.1;          
    WEP[P_Left_Leg_Knee_Offset]=0;              
    WEP[P_Left_Leg_Foot_Pitch_Offset]=0.02;         
    WEP[P_Left_Leg_Foot_Roll_Offset]=0;           

    WEP[P_Right_Leg_Hip_Yaw_Offset]=0;          
    WEP[P_Right_Leg_Hip_Roll_Offset]=0;          
    WEP[P_Right_Leg_Hip_Pitch_Offset]=0.1;         
    WEP[P_Right_Leg_Knee_Offset]=0.0;             
    WEP[P_Right_Leg_Foot_Pitch_Offset]=0.21;        
    WEP[P_Right_Leg_Foot_Roll_Offset]=0;         

    //Left leg inverse kinematic offset 
    WEP[P_Left_Leg_X_Offset]=0;                
    WEP[P_Left_Leg_Y_Offset]=0;                
    WEP[P_Left_Leg_Z_Offset]=0;               
    WEP[P_Left_Leg_Roll_Offset]=0;              
    WEP[P_Left_Leg_Pitch_Offset]=0;            
    WEP[P_Left_Leg_Yaw_Offset]=0;             

    WEP[P_Right_Leg_X_Offset]=0;              
    WEP[P_Right_Leg_Y_Offset]=0;             
    WEP[P_Right_Leg_Z_Offset]=0;             
    WEP[P_Right_Leg_Roll_Offset]=0;             
    WEP[P_Right_Leg_Pitch_Offset]=0;            
    WEP[P_Right_Leg_Yaw_Offset]=0;             
    
    WEP[P_R_Arm_Pitch_offset]=-0.3;               
    WEP[P_R_Arm_Roll_offset]=-1.45;               
    WEP[P_R_Arm_Elbow_offset]=1.3;              
    
    WEP[P_L_Arm_Pitch_offset]=-0.3;                
    WEP[P_L_Arm_Roll_offset]=-1.45;               
    WEP[P_L_Arm_Elbow_offset]=1.3;                     
                   
    //imu offset
    WEP[P_IMU_X_Angle_Offset]= 0.0;              
    WEP[P_IMU_Y_Angle_Offset]= -0.05;
    
    //MPU filtering parametrs 
    WEP[P_Gyro_X_LowPass_Gain]=0.9;              
    WEP[P_Gyro_Y_LowPass_Gain]=0.7;             
    
    //kalman filter r mesurement value
    WEP[P_Kalman_Roll_RM_Rate]=800;               
    WEP[P_Kalman_Pitch_RM_Rate]=200;             
    WEP[P_Kalman_Yaw_RM_Rate]=50;                
    
    //smoothing ratio
    WEP[P_Vx_Smoothing_Ratio]=0;               
    WEP[P_Vy_Smoothing_Ratio]=0;               
    WEP[P_Vt_Smoothing_Ratio]=0;                    
  }
}
