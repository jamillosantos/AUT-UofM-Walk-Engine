//#include "MQueue.h"
#include "Wire.h"
#include "MPUGY80.h"
#include "KalmanFilter2D.h"
#include "MapleFreeRTOS.h"   // RTOS Lib
//#include "EEPROM.h"
//#include "orpol.h"
//#include "pplsq.h"

// Ten Size Akbar=0  Asghar=1
#define Teen_Size_Robot_Num   (0)  //AK
//#define Teen_Size_Robot_Num   (1) //AS

//motion numbers table
#define Motion_1                     0x01
#define Motion_2                     0x02
#define Motion_3                     0x03
#define Motion_4                     0x04
#define Motion_5                     0x05
#define Motion_6                     0x06
#define Motion_7                     0x07
#define Motion_8                     0x08
#define Motion_9                     0x09
#define Motion_10                    0x0A
#define No_Motion                    0x64
#define Stop_Motion                  0x70

//internal motion number value
#define Stand_Up_Front               1
#define Stand_Up_Back                2

//fall state values
#define Fallen_Front                 1
#define Fallen_Back                  2
#define Fallen_Right                 3
#define Fallen_Left                  4
#define Normal_Stand                 5

HardwareTimer Timer(1);      // Hardware timer for RTC
MPUGY80 MPU;
Dynamixel Dxl(DXL_BUS_SERIAL3);            // Dynamixel on Serial1(USART1)
//EEPROM CM_EEPROM;
//HardwareSPI spi(1);

//creat kalman objects
KalmanFilter2D kalmanX; // Create the Kalman instances
KalmanFilter2D kalmanY;
KalmanFilter2D kalmanZ;

unsigned long int MPU_Loop_Hz=0 , MPU_Loop_Cnt=0;
unsigned long int DXL_Loop_Hz=0 , DXL_Loop_Cnt=0;
unsigned long int WEL_Loop_Hz=0 , WEL_Loop_Cnt=0;
unsigned long int RSL_Loop_Hz=0 , RSL_Loop_Cnt=0;

double MPU_X=0, MPU_Y=0, MPU_Z=0;
double Gyro_X=0, Gyro_Y=0;
double WEP[100];
double System_Voltage=160;
byte   Actuators_Update=1;

//walk engine arrays for update robot inverse kinematic
double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve

//walk engine parameters (these are sent from PC)
//main walk parameters
double Vx=0.0;        //velocity of X (forward) direction from PC
double Vy=0.0;        //velocity of Y (sideward) direction
double Vt=0.0;        //velocity of T (rotate) speed
byte   Motion_Ins=0;    //the motion request (from pc and must be run in real-time)
byte   Internal_Motion_Request=0;
byte   Check_Robot_Fall=1;
double Head_Pan_Angle=0;
double Head_Tilt_Angle=0;
double Head_Pan_Speed=10;
double Head_Tilt_Speed=10;

void setup() {
  //initialize usb as a serial port
  //SerialUSB.begin();                // Config serialUSB port
  SerialUSB.attachInterrupt(UsbInterrupt);
  
    //Serial2 Serial initialize
  Serial2.begin(115200); 
  
  //initialize Dynamixel defult bus (1000000bps) 5=3000000
  Dxl.begin(Boudrate_1000000bps);
  
  //initialize onboard pins
  initialize_Onboard_Pins();
  
  //initialize expantion board pins
  Initialize_Expander_Pins();
  
  //CM_EEPROM.begin();
  //for(int j=0;j < 100;j++){
  //  CM_EEPROM.write(j,j+2);// write i*2 to virtual address 0~9
  //}
  
    //initialize internal timer
  RTC_Setup_Timer(1000000);         //initialize RTC for 1 mili secound
  
  xTaskCreate( vRobot_State_Task,      ( signed char * ) "Robot_State"            , 128, NULL, 1, NULL );
  xTaskCreate( vMPU_Kalman_Task,       ( signed char * ) "MPU_Kalman"             , 512, NULL, 1, NULL );
  xTaskCreate( vDynamixel_Update_Task, ( signed char * ) "Dynamixel_Update_Task"  , 256, NULL, 1, NULL );
  xTaskCreate( vWalk_Engine_Task,      ( signed char * ) "Walk_Engine_Task"       , 512, NULL, 5, NULL );
  xTaskCreate( vSerial_USB_Tx_Task,    ( signed char * ) "Serial_Tx"              , 128, NULL, 1, NULL );

  vTaskStartScheduler();
}

void loop() {
  
  // put your main code here, to run repeatedly: 

}

