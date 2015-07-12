#define MPU_GY80_VCC_Pin      8
#define MPU_GY80_GND_Pin      7
#define MPU_GY80_SCL_Pin      6
#define MPU_GY80_SDA_Pin      5

//MPU task
void vMPU_Kalman_Task( void *pvParameters ){
  
  vTaskSuspendAll();
  Serial2.println("AUT_UofM:> MPU Kalman Task Start Sucssecfully!");
  xTaskResumeAll();
  
  vTaskSuspendAll();
  Serial2.print("AUT_UofM:> Initialize I2C MPU...");
  //initialize GY80 power pins
  pinMode(MPU_GY80_VCC_Pin, OUTPUT); digitalWrite(MPU_GY80_VCC_Pin, HIGH);  //gnd of mpu-GY80
  pinMode(MPU_GY80_GND_Pin, OUTPUT); digitalWrite(MPU_GY80_GND_Pin, LOW);   //vcc of mpu-GY80
  
  vTaskDelay(10);
  //Initialize i2c comunication port (sda and scl)
  Wire.begin(MPU_GY80_SDA_Pin,MPU_GY80_SCL_Pin); 
  vTaskDelay(10);
  Serial2.println("OK!");
  
  Serial2.print("AUT_UofM:> Configuration MPU-9150...");
  MPU.init(); //initialize MPU
  MPU.initDrift(20); //calculate drrift
  Serial2.println("OK!");
  xTaskResumeAll();
  
  vTaskSuspendAll();
  Serial2.print("AUT_UofM:> Kalman Filter Initialize...");
  kalmanX.setAngle(0.0); // Set starting angle
  kalmanY.setAngle(0.0);
  kalmanZ.setAngle(0.0);
  kalmanX.setRmeasure(WEP[P_Kalman_Roll_RM_Rate]);
  kalmanY.setRmeasure(WEP[P_Kalman_Pitch_RM_Rate]);
  kalmanZ.setRmeasure(WEP[P_Kalman_Yaw_RM_Rate]);
  Serial2.println("OK!");
  xTaskResumeAll();
  
  portTickType xLastWakeTime;
  const portTickType xFrequency = 25;  //10ms for each loop run time means 100Hz of task frequency
  xLastWakeTime = xTaskGetTickCount ();
    
  //main task loop
  for( ;; ){
    MPU_Loop_Cnt++;    
    Calculate_Euler_Angles();
    
    //if(Debug_Mode){
    //  RTOS_Error_Log("MPU Task:",MPU_Loop_Cnt);
    //}
    
    digitalWrite(BLUE_LED_485EXP, HIGH); 
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    digitalWrite(BLUE_LED_485EXP, LOW);
  }
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
    if(dt>100.0)dt=0.0001;    
    
    vTaskSuspendAll();  
    //MPU.init(); //initialize MPU 
    MPU.init_Mag(); 
    MPU.ReadXYZ(); //read all data from MPU and normalize them  
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
    
    //MPU_Z= (kalmanZ.getAngle(MPU_Angle_Z,  MPU.get_Gy(), dt))*DEG2RAD;
}

