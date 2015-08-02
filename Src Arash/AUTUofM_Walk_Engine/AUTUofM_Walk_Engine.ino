#include "Wire.h"
#include "MPUGY80.h"
#include "KalmanFilter2D.h"
#include "MapleFreeRTOS.h"   // RTOS Lib

// Ten Size Akbar=0  Asghar=1
#define Teen_Size_Robot_Num   (0)  //AUT Akbar
//#define Teen_Size_Robot_Num     (1) //AS

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

MPUGY80 MPU;
Dynamixel Dxl(DXL_BUS_SERIAL3);            // Dynamixel on Serial1(USART1)

//creat kalman objects
KalmanFilter2D kalmanX; // Create the Kalman instances
KalmanFilter2D kalmanY;

double MPU_X=0, MPU_Y=0, MPU_Z=0;
double Gyro_X=0, Gyro_Y=0;
double System_Voltage=160; //defult voltage
byte   Actuators_Update=1;
byte   Actuators_Update_PID=1;
double WEP[110];

//walk engine parameters (these are sent from PC)
//main walk parameters
double Vx=0.0;          //velocity of X (forward) direction from PC
double Vy=0.0;          //velocity of Y (sideward) direction
double Vt=0.0;          //velocity of T (rotate) speed
byte   Motion_Ins=0;    //the motion request (from pc and must be run in real-time)
byte   Internal_Motion_Request=0;
byte   Check_Robot_Fall=1;
double Head_Pan_Angle=0;
double Head_Tilt_Angle=0;
double Head_Pan_Speed=10;
double Head_Tilt_Speed=10;

void setup() {
  //initialize usb as a serial port
  SerialUSB.begin();                // Config serialUSB port
  //SerialUSB.attachInterrupt(UsbInterrupt);
  
  //initialize Dynamixel defult bus (1000000bps) 5=3000000
  Dxl.begin(Boudrate_1000000bps);
  
  //initialize onboard pins
  initialize_Onboard_Pins();
  
  //initialize expantion board pins
  Initialize_Expander_Pins();

  xTaskCreate( vWalk_Engine_Task,      ( signed char * ) "Walk_Engine_Task"       , 512, NULL, 10, NULL );
  xTaskCreate( vDCM_Update_Task,       ( signed char * ) "DCM_Update_Task"        , 512., NULL, 1, NULL );
  vTaskStartScheduler();
}

void loop() {
  
  // put your main code here, to run repeatedly: 

}

