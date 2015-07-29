//parameters address for setting in walk engine 

//Walk Engine Parameters
#define P_Motion_Resolution                   1  //motion resoulotion (min=0.001  max=0.1)  
#define P_Gait_Frequency                      2  //gait frequency (min=0.001  max=1.0) 
#define P_Double_Support_Sleep                3  //double support sleep (min=0.001  max=1.0)  
#define P_Single_Support_Sleep                4  //single support sleep (min=0.001  max=1.0)  
#define P_Fly_Roll_Gain                       5  //fly leg roll gain  
#define P_Fly_Pitch_Gain                      6  //fly leg pitch gain 
#define P_Fly_Yaw_Gain                        7  //fly leg yaw gain
#define P_Fly_X_Swing_Gain                    8
#define P_Fly_Y_Swing_Gain                    9
#define P_Fly_Z_Swing_Gain                    10  //fly leg step height gain (min=0.001  max=1.0)  
#define P_Support_Roll_Gain                   11  //support leg roll gain  
#define P_Support_Pitch_Gain                  12  //support leg pitch gain
#define P_Support_Yaw_Gain                    13  //support leg yaw gain  
#define P_Support_X_Swing_Gain                14  
#define P_Support_Y_Swing_Gain                15  //support leg Y gain (sideward) 
#define P_Support_Z_Swing_Gain                16  //support leg Z gain (topdown) this parameter also can use for push  
#define P_Body_X_Swing_Gain                   17
#define P_Body_Y_Swing_Gain                   18  //body sideward swing gain (for swing both of legs in y axis during walk)  
#define P_Body_Z_Swing_Gain                   19  //body topdown swing gain (for swing both of legs in Z axis during walk)

//stablization parameters
#define P_Stablizer_Arm_Pitch_Gain            20 //add
#define P_Stablizer_Arm_Roll_Gain             21 //add
#define P_Stablizer_Arm_Elbow_Gain            22
#define P_Stablizer_Hip_Roll_Gain             23 //add
#define P_Stablizer_Hip_Pitch_Gain            24 //add
#define P_Stablizer_Knee_Gain                 25 //add
#define P_Stablizer_Foot_Pitch_Gain           26 //add
#define P_Stablizer_Foot_Roll_Gain            27 //add
#define P_Stablizer_COM_X_Shift_Gain          28 //add
#define P_Stablizer_COM_Y_Shift_Gain          29 //add

#define P_Gyro_Stablizer_Arm_Pitch_Gain       30 //add
#define P_Gyro_Stablizer_Arm_Roll_Gain        31
#define P_Gyro_Stablizer_Arm_Elbow_Gain       32
#define P_Gyro_Stablizer_Hip_Roll_Gain        33 //add
#define P_Gyro_Stablizer_Hip_Pitch_Gain       34 //add
#define P_Gyro_Stablizer_Knee_Gain            35 //add
#define P_Gyro_Stablizer_Foot_Pitch_Gain      36 //add
#define P_Gyro_Stablizer_Foot_Roll_Gain       37 //add
#define P_Gyro_Stablizer_COM_X_Shift_Gain     38 //add
#define P_Gyro_Stablizer_COM_Y_Shift_Gain     39 //add

//hopping gait gain
#define P_Stablizer_Hopping_Gait_X_Gain       40
#define P_Stablizer_Hopping_Gait_Y_Gain       41

//both leg offset in inverse kinematic (body COM)
#define P_COM_X_offset                        42 //add
#define P_COM_Y_offset                        43 //add
#define P_COM_Z_offset                        44 //add
#define P_COM_Roll_offset                     45 //add
#define P_COM_Pitch_offset                    46 //add
#define P_COM_Yaw_offset                      47 //add
 
//legs joints offset 
#define P_Left_Leg_Hip_Yaw_Offset             48 //add
#define P_Left_Leg_Hip_Roll_Offset            49 //add
#define P_Left_Leg_Hip_Pitch_Offset           50 //add
#define P_Left_Leg_Knee_Offset                51 //add
#define P_Left_Leg_Foot_Pitch_Offset          52 //add
#define P_Left_Leg_Foot_Roll_Offset           53 //add

#define P_Right_Leg_Hip_Yaw_Offset            54 //add
#define P_Right_Leg_Hip_Roll_Offset           55 //add
#define P_Right_Leg_Hip_Pitch_Offset          56 //add
#define P_Right_Leg_Knee_Offset               57 //add
#define P_Right_Leg_Foot_Pitch_Offset         58 //add
#define P_Right_Leg_Foot_Roll_Offset          59 //add

//Left leg inverse kinematic offset 
#define P_Left_Leg_X_Offset                   60 //add
#define P_Left_Leg_Y_Offset                   61 //add
#define P_Left_Leg_Z_Offset                   62 //add
#define P_Left_Leg_Roll_Offset                63 //add
#define P_Left_Leg_Pitch_Offset               64 //add
#define P_Left_Leg_Yaw_Offset                 65 //add

#define P_Right_Leg_X_Offset                  66 //add
#define P_Right_Leg_Y_Offset                  67 //add
#define P_Right_Leg_Z_Offset                  68 //add
#define P_Right_Leg_Roll_Offset               69 //add
#define P_Right_Leg_Pitch_Offset              70 //add
#define P_Right_Leg_Yaw_Offset                71 //add
    
#define P_R_Arm_Pitch_offset                  72 //add
#define P_R_Arm_Roll_offset                   73 //add
#define P_R_Arm_Elbow_offset                  74 //add

#define P_L_Arm_Pitch_offset                  75 //add
#define P_L_Arm_Roll_offset                   76 //add
#define P_L_Arm_Elbow_offset                  77 //add

//fall thershold
#define P_Fall_Roll_Thershold                 78
#define P_Fall_Pitch_Thershold                79

//imu offset
#define P_IMU_X_Angle_Offset                  80 //add
#define P_IMU_Y_Angle_Offset                  81 //add

//MPU filtering parametrs 
#define P_Gyro_X_LowPass_Gain                 82 //add
#define P_Gyro_Y_LowPass_Gain                 83 //add

//kalman filter r mesurement value
#define P_Kalman_Roll_RM_Rate                 84
#define P_Kalman_Pitch_RM_Rate                85
#define P_Kalman_Yaw_RM_Rate                  86

//smoothing ratio
#define P_Vx_Smoothing_Ratio                  87 
#define P_Vy_Smoothing_Ratio                  88 
#define P_Vt_Smoothing_Ratio                  89 

#define P_Leg_Length                          90 //add

#define P_Head_Pan_Speed                      91
#define P_Head_Tilt_Speed                     92

#define P_Min_Voltage_Limit                   93

#define Vx_Offset                             94
#define Vy_Offset                             95
#define Vt_Offset                             96

