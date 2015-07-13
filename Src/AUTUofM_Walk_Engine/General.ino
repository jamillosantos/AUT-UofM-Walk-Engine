void UsbInterrupt(byte* buffer, byte nCount){
  noInterrupts();
  vTaskSuspendAll();
  if(nCount==7){
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
  }
  xTaskResumeAll();
  interrupts();  
}


/*
* real time clock initialize
*/
/*
void RTC_Setup_Timer(long Period){
  Timer.pause();                                  // Pause the timer while we're configuring it
  Timer.setPeriod(Period);                // Set up period in microseconds
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE); // Set up an interrupt on channel 1
  Timer.setCompare(TIMER_CH1, 1);                 // Interrupt 1 count after each update
  Timer.attachInterrupt(TIMER_CH1, RTC_INT);      // atach interrupt function to timer
  Timer.refresh();                                // Refresh the timer's count, prescale, and overflow
  Timer.resume(); 
}
*/

/*
* Hardware interrupt for real time clock
* this interrupt will heald in every 1 sec
*/
//void RTC_INT(void) { 
  //noInterrupts();
  //DCM_Loop_Hz=DCM_Loop_Cnt; DCM_Loop_Cnt=0;
  //WEL_Loop_Hz=WEL_Loop_Cnt; WEL_Loop_Cnt=0;  
  //RSL_Loop_Hz=RSL_Loop_Cnt; RSL_Loop_Cnt=0;
  //interrupts(); 
  //if(Debug_Mode){
  //   RTOS_Error_Log("RTC Clock:",millis());
  //}
  
  //Serial2.print("AUT_UofM:>");
  //Serial2.print("\t DCM=");Serial2.print(DCM_Loop_Hz); Serial2.print("Hz");
  //Serial2.print("\t WEL=");Serial2.print(WEL_Loop_Hz); Serial2.print("Hz");
  //Serial2.print("\t RSL=");Serial2.print(RSL_Loop_Hz); Serial2.println("Hz");
//}

/*
 initialize expantion board pins
*/
void Initialize_Expander_Pins(){
  pinMode(RED_LED_485EXP  , OUTPUT); 
  pinMode(GREEN_LED_485EXP, OUTPUT); 
  pinMode(BLUE_LED_485EXP , OUTPUT); 
  
  pinMode(BUTTON1_485EXP  , INPUT); 
  pinMode(BUTTON2_485EXP  , INPUT);
  
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

/*
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
*/

/*
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
*/


/* 
* send Euler state to the USB serial Port
*/
void Send_Euler_State(){
  
   byte PData[10];
   
   PData[0]=(byte)254;
   PData[1]=(byte)254;
   
   unsigned long T =0;
   T = (unsigned long)((MPU_X+Pi) * 1000);         
   PData[2]= _LOBYTE(T);
   PData[3]= _HIBYTE(T);
      
   T =(unsigned long)((MPU_Y+Pi)*1000); 
   PData[4]= _LOBYTE(T);
   PData[5]= _HIBYTE(T);
   
   T =(unsigned long)((MPU_Z+Pi)*1000); 
   PData[6]= _LOBYTE(T);
   PData[7]= _HIBYTE(T);
   
   SerialUSB.write(PData,8);
   
   //noInterrupts(); 
   //for(byte i=0;i<=7;i++){  
   //   SerialUSB.print((char)PData[i]);  
   //} 
   //interrupts();
   
}
