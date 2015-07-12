void UsbInterrupt(byte* buffer, byte nCount){
  //vTaskSuspendAll();
  noInterrupts();
  vTaskSuspendAll();
  if(nCount==13){
    digitalWrite(BOARD_LED_PIN,LOW);
    //if(Debug_Mode){
    //  RTOS_Error_Log("USB Int:",nCount);
    //}
    if((buffer[0]==254) && (buffer[1]==254)){
                //walk data update
                unsigned long T=0;
                T =((unsigned long) ((unsigned long)(buffer[3]<<8) + ((byte)buffer[2])));
                double tVx=0.0;
                tVx =(T<=32767) ?  (double)(T/1000.0) : (double)(-((T-32767) / 1000.0));
                if((tVx>-1.0) && (tVx<1.0)){
                  Vx=tVx; 
                }
                
                T =((unsigned long) ((unsigned long)(buffer[5]<<8) + ((byte)buffer[4])));
                double tVy=0.0;
                tVy = (T<=32767) ?  (double)(T/1000.0) : (double)(-((T-32767) / 1000.0));
                if((tVy>-1.0) && (tVy<1.0)){
                  Vy=tVy;
                }
                      
                T =((unsigned long) ((unsigned long)(buffer[7]<<8) + ((byte)buffer[6])));
                double tVt=0.0;
                tVt = (T<=32767) ?  (double)(T/1000.0) : (double)(-((T-32767) / 1000.0));
                if((tVt>-1.0) && (tVt<1.0)){
                  Vt=tVt;
                }
                
                if (Internal_Motion_Request==No_Motion ){
                  Motion_Ins=buffer[8];
                }
                    
                //head data update
                T =((unsigned long) ((unsigned long)(buffer[10] << 8) + ((byte)buffer[9])));
                double Pan_Angle=0.0;
                Pan_Angle = (T<=32767) ?  (double)(T/1000.0) : (double)(-((T-32767) / 1000.0));       
                
                T =((unsigned long) ((unsigned long)(buffer[12] << 8) + ((byte)buffer[11])));
                double Tilt_Angle=0.0;
                Tilt_Angle = (T<=32767) ?  (double)(T/1000.0) : (double)(-((T-32767) / 1000.0));
                
                if (Internal_Motion_Request==No_Motion ){
                  //Serial2.print("AUT_UofM:> Head Pan:"); Serial2.print(Pan_Angle); Serial2.print(" \t Tilt:"); Serial2.println(Tilt_Angle);
                  Set_Head(Pan_Angle,Tilt_Angle,WEP[P_Head_Pan_Speed],WEP[P_Head_Tilt_Speed]);
                }
    }
    //xTaskResumeAll();
    digitalWrite(BOARD_LED_PIN,HIGH);
  }
  xTaskResumeAll();
  interrupts();  
  //xTaskResumeAll();
}

/*
* real time clock initialize
*/
void RTC_Setup_Timer(long Period){
  Timer.pause();                                  // Pause the timer while we're configuring it
  Timer.setPeriod(Period);                // Set up period in microseconds
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE); // Set up an interrupt on channel 1
  Timer.setCompare(TIMER_CH1, 1);                 // Interrupt 1 count after each update
  Timer.attachInterrupt(TIMER_CH1, RTC_INT);      // atach interrupt function to timer
  Timer.refresh();                                // Refresh the timer's count, prescale, and overflow
  Timer.resume(); 
}

/*
* Hardware interrupt for real time clock
* this interrupt will heald in every 1 sec
*/
void RTC_INT(void) { 
  noInterrupts();
  DCM_Loop_Hz=DCM_Loop_Cnt; DCM_Loop_Cnt=0;
  WEL_Loop_Hz=WEL_Loop_Cnt; WEL_Loop_Cnt=0;  
  RSL_Loop_Hz=RSL_Loop_Cnt; RSL_Loop_Cnt=0;
  interrupts(); 
  //if(Debug_Mode){
  //   RTOS_Error_Log("RTC Clock:",millis());
  //}
  
  //Serial2.print("AUT_UofM:>");
  //Serial2.print("\t DCM=");Serial2.print(DCM_Loop_Hz); Serial2.print("Hz");
  //Serial2.print("\t WEL=");Serial2.print(WEL_Loop_Hz); Serial2.print("Hz");
  //Serial2.print("\t RSL=");Serial2.print(RSL_Loop_Hz); Serial2.println("Hz");
}

/*
 initialize expantion board pins
*/
void Initialize_Expander_Pins(){
  pinMode(RED_LED_485EXP  , OUTPUT); 
  pinMode(GREEN_LED_485EXP, OUTPUT); 
  pinMode(BLUE_LED_485EXP , OUTPUT); 
  
  pinMode(BUTTON1_485EXP, INPUT); 
  pinMode(BUTTON2_485EXP, INPUT);
  
  digitalWrite(RED_LED_485EXP  , HIGH);
  digitalWrite(GREEN_LED_485EXP, HIGH);
  digitalWrite(BLUE_LED_485EXP , HIGH);
}

/*
 initialize onboard pins
*/
void initialize_Onboard_Pins(){
  pinMode(BOARD_LED_PIN     , OUTPUT);   //config onboard led
  pinMode(BOARD_BUTTON_PIN  , INPUT_PULLDOWN);  //configh onboard micro switch
  pinMode(Buzzer_Pin        , OUTPUT);
  digitalWrite(Buzzer_Pin   , LOW);
  
  pinMode(10, OUTPUT);
  pinMode(11, INPUT_PULLDOWN);
  digitalWrite(10, HIGH);
}

//set the buzzer for a time
void Buzzer(unsigned int Time){
  digitalWrite(Buzzer_Pin, HIGH);
  delay(Time);
  digitalWrite(Buzzer_Pin, LOW);
}

void RTOS_Error_Log(char * P){
  vTaskSuspendAll();
  Serial2.print("AUT_UofM:> ");Serial2.println(P);
  xTaskResumeAll();
}

void RTOS_Error_Log(char * P, int i){
  vTaskSuspendAll();
  Serial2.print("AUT_UofM:> ");Serial2.print(P);Serial2.println(i);
  xTaskResumeAll();
}

//calculate the polynomilan function
double Poly_Calculate(double x, double* coefficients, int degree)
{
    double pows[degree];
    pows[0] = 1;
    double y = coefficients[0];
    for(int i = 1 ; i < degree ; i++)
    {
        pows[i] = x * pows[i-1];
        y += coefficients[i] * pows[i];
    }
    return y;
}



/* 
* send Euler state to the USB serial Port
*/
void Send_Euler_State(){
  
   byte PData[10];
   
   PData[0]=(byte)254;
   PData[1]=(byte)254;
   PData[2]=(byte)100;
   
   unsigned long T =0;
   T = (unsigned long)((MPU_X+Pi) * 1000);         
   PData[3]= _LOBYTE(T);
   PData[4]= _HIBYTE(T);
      
   T =(unsigned long)((MPU_Y+Pi)*1000); 
   PData[5]= _LOBYTE(T);
   PData[6]= _HIBYTE(T);
   
   T =(unsigned long)((MPU_Z+Pi)*1000); 
   PData[7]= _LOBYTE(T);
   PData[8]= _HIBYTE(T);
   
   noInterrupts(); 
   for(byte i=0;i<=8;i++){  
      SerialUSB.print((char)PData[i]);  
   } 
   interrupts();
}
