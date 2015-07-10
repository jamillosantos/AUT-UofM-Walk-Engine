/*
 * MPUGY80.h
 *
 *  Created on: 2015. 9. june.
 *      Author: Mojtaba Karimi
 *      Email:  mojtaba_k@live.com
 *      Web:    mojtabakarimi.ml
 */
#ifndef MPUGY80_h
#define MPUGY80_h

#include "wirish.h"
#include <Wire.h>
#include <math.h>
////////// define RC-100 button key value ////////////////
//#define RC100_BTN_U		(1)


//accelorometer address
#define ADXL345_Address         (0x53)     // ADXL345 device address
#define ADXL345_POWER_CTL       (0x2d)
#define ADXL345_DATA_FORMAT     (0x31)
#define ADXL345_DATAX0          (0x32)
#define ADXL345_DATAX1          (0x33)
#define ADXL345_DATAY0          (0x34)
#define ADXL345_DATAY1          (0x35)
#define ADXL345_DATAZ0          (0x36)
#define ADXL345_DATAZ1          (0x37)

//compass address
#define HMC5883L_Address        (0x1E)     // Address of i2c Device Magnometer HMC5883
#define HMC5883L_Config         (0x02)
#define HMC5883L_DATAX0         (0x03)
#define HMC5883L_DATAX1         (0x04)
#define HMC5883L_DATAY0         (0x05)
#define HMC5883L_DATAY1         (0x06)
#define HMC5883L_DATAZ0         (0x07)
#define HMC5883L_DATAZ1         (0x08)

//Internal register of Gyro sensor L3G4200D
#define L3G4200D_Address        (105)      // I2C address of the L3G4200D
#define CTRL_REG1               (0x20)
#define CTRL_REG2               (0x21)
#define CTRL_REG3               (0x22)
#define CTRL_REG4               (0x23)
#define CTRL_REG5               (0x24)

#define Gyro_Scale              (70.0) //define the gyro scale that represent it in deg/s from gyro data


class MPUGY80 
{
  public:
    MPUGY80() 
    {
      AX=0;AY=0;AZ=0;
      GX=0;GY=0;GZ=0;
      CX=0;CY=0;CZ=0;
      
      Raw_Ax=0;Raw_Ay=0;Raw_Az=0;
      Raw_Gx=0;Raw_Gy=0;Raw_Gz=0;
      Raw_Cx=0;Raw_Cy=0;Raw_Cz=0;
      
      Gyr_x_Drift=0.0;Gyr_y_Drift=0.0;Gyr_z_Drift=0.0;
      norm=0.0;
    }
    
    void init()
    {
      //accel init  
      ADXL345_setRangeSetting(16);   //set range to +- 16g for gravity
      i2c_writeRegister(ADXL345_Address,ADXL345_POWER_CTL, 0);      
      i2c_writeRegister(ADXL345_Address,ADXL345_POWER_CTL, 16);
      i2c_writeRegister(ADXL345_Address,ADXL345_POWER_CTL, 8);
          
      //compass init
      i2c_writeRegister(HMC5883L_Address, HMC5883L_Config, 0x00);
          
      // Enable x, y, z and turn off power down:
      i2c_writeRegister(L3G4200D_Address, CTRL_REG1, 0b11111111); //for 800 Hz and 110 cut off

      // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
      i2c_writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

      // Configure CTRL_REG3 to generate data ready interrupt on INT2
      // No interrupts used on INT1, if you'd like to configure INT1
      // or INT2 otherwise, consult the datasheet:
      i2c_writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

      // CTRL_REG4 controls the full-scale range, among other things:
      //250 dps
      //i2c_writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
      
      //500 dps
      i2c_writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
      
      //2000 dps
      //i2c_writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
      
      // CTRL_REG5 controls high-pass filtering of outputs, use it
      i2c_writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
      
      //initDrift(1);
    }
    
    void init_Mag()
    {
          
      //compass init
      i2c_writeRegister(HMC5883L_Address, HMC5883L_Config, 0x00);
    }
    
    /*
    * Sets the range setting, possible values are: 2, 4, 8, 16
    */
    void ADXL345_setRangeSetting(int val) 
    {
	byte _s;
	byte _b;	
	switch (val) 
        {
		case 2:  
			_s = B00000000; 
			break;
		case 4:  
			_s = B00000001; 
			break;
		case 8:  
			_s = B00000010; 
			break;
		case 16: 
			_s = B00000011; 
			break;
		default: 
			_s = B00000000;
	}
	_b= (byte) i2c_readRegister(ADXL345_Address,ADXL345_DATA_FORMAT);
	_s |= (_b & B11101100);
	i2c_writeRegister(ADXL345_Address,ADXL345_DATA_FORMAT, _s);
    }
    
    /*
    * calculate gyro drifft
    */
    void initDrift(unsigned long int Filter_Sampling)
    {
      for(unsigned long int i=0;i<=(Filter_Sampling);i++)
      {  
         Gyr_x_Drift += (double) ((1.0/Filter_Sampling)*_getRaw_GX()); 
         Gyr_y_Drift += (double) ((1.0/Filter_Sampling)*_getRaw_GY());
         Gyr_z_Drift += (double) ((1.0/Filter_Sampling)*_getRaw_GZ());
      }
    }
    
    //get temperature
    double Get_Temp()
    { 
      //return (double) ((((signed short)(((byte) i2c_readRegister(MPU9150_I2C_ADDRESS, MPU9150_TEMP_OUT_H)) << 8) | ((byte) i2c_readRegister(MPU9150_I2C_ADDRESS, MPU9150_TEMP_OUT_L)))+12412.0)/340.0);
      return 0;  
    }
    
    //get x raw data from accel
    double _getRaw_AX() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(ADXL345_Address, ADXL345_DATAX1)) << 8) | ((byte) i2c_readRegister(ADXL345_Address, ADXL345_DATAX0)));
    }
    //get y raw data from accel
    double _getRaw_AY() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(ADXL345_Address, ADXL345_DATAY1)) << 8) | ((byte) i2c_readRegister(ADXL345_Address, ADXL345_DATAY0)));
    }
    //get z raw data from accel
    double _getRaw_AZ() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(ADXL345_Address, ADXL345_DATAZ1)) << 8) | ((byte) i2c_readRegister(ADXL345_Address, ADXL345_DATAZ0)));
    }
     
    //get x raw data from gyro
    double _getRaw_GX() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(L3G4200D_Address, 0x29)) << 8) | ((byte) i2c_readRegister(L3G4200D_Address, 0x28)));     
    }
    //get x raw data from gyro
    double _getRaw_GY() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(L3G4200D_Address, 0x2B)) << 8) | ((byte) i2c_readRegister(L3G4200D_Address, 0x2A)));    
    }
    //get x raw data from gyro
    double _getRaw_GZ() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(L3G4200D_Address, 0x2D)) << 8) | ((byte) i2c_readRegister(L3G4200D_Address, 0x2C)));
    }
    
    //get x raw data from compass
    double _getRaw_CX() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(HMC5883L_Address, HMC5883L_DATAX0)) << 8) | ((byte) i2c_readRegister(HMC5883L_Address, HMC5883L_DATAX1)));
    }
    //get x raw data from compass
    double _getRaw_CY() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(HMC5883L_Address, HMC5883L_DATAY0)) << 8) | ((byte) i2c_readRegister(HMC5883L_Address, HMC5883L_DATAY1)));
    }
    //get x raw data from compass
    double _getRaw_CZ() 
    {
          return (double)((signed short)(((byte) i2c_readRegister(HMC5883L_Address, HMC5883L_DATAZ0)) << 8) | ((byte) i2c_readRegister(HMC5883L_Address, HMC5883L_DATAZ1)));
    }
    
    //read accel, gyro and compass
    void ReadXYZ()
    { 
       Raw_Ax=_getRaw_AX();
       Raw_Ay=_getRaw_AY();
       Raw_Az=_getRaw_AZ();
          
       norm = (double) sqrt((Raw_Ax*Raw_Ax) + (Raw_Ay*Raw_Ay) + (Raw_Az*Raw_Az));       
       AX =(double)(Raw_Ax / norm);
       AY =(double)(Raw_Ay / norm);
       AZ =(double)(Raw_Az / norm); 
       //-------------------------
       Raw_Gx=_getRaw_GX();
       Raw_Gy=_getRaw_GY();
       Raw_Gz=_getRaw_GZ();
       
       GX =(double)(((Raw_Gx-Gyr_x_Drift)*Gyro_Scale) / 10000.0);
       GY =(double)(((Raw_Gy-Gyr_y_Drift)*Gyro_Scale) / 10000.0);
       GZ =(double)(((Raw_Gz-Gyr_z_Drift)*Gyro_Scale) / 10000.0);    
       //------------------------
       Raw_Cx=_getRaw_CX();
       Raw_Cy=_getRaw_CY();
       Raw_Cz=_getRaw_CZ();
          
       norm = (double) sqrt((Raw_Cx*Raw_Cx) + (Raw_Cy*Raw_Cy) + (Raw_Cz*Raw_Cz));       
       CX =(double)(Raw_Cx / norm);
       CY =(double)(Raw_Cy / norm);
       CZ =(double)(Raw_Cz / norm);
    }
    
    double get_Ax(){
      return (double)AX;
    }
    double get_Ay(){
      return (double)AY;
    }
    double get_Az(){
      return (double)AZ;
    }
    
    
    double get_Gx(){
      return (double)GX;
    }
    double get_Gy(){
      return (double)GY;
    }
    double get_Gz(){
      return (double)GZ;
    }
    

    double get_Cx(){
      return (double)CX;
    }
    double get_Cy(){
      return (double)CY;
    }
    double get_Cz(){
      return (double)CZ;
    }


    //
    /*
	* Write register to i2c device with specific i2c
	*/
	void i2c_writeRegister(byte deviceAddress, byte address, byte val) 
	{
    	Wire.beginTransmission(deviceAddress); // start transmission to device 
    	Wire.write(address);       // send register address
    	Wire.write(val);           // send value to write
    	Wire.endTransmission();    // end transmission
	}

	/*
	* Read a byte from i2c device with specific address
	*/
	byte i2c_readRegister(byte deviceAddress, byte address)
	{
    	unsigned int Dt=millis();
    	Wire.beginTransmission(deviceAddress);
    	Wire.write(address);                 // register to read
    	Wire.endTransmission();
    	Wire.requestFrom(deviceAddress, 2);  // read a byte
    	while((!Wire.available())&&(millis()<(Dt+100))) { } //wait until ready or time out!
    	return (byte)Wire.read();
	}

  //private varibles
  private:
    double AX,AY,AZ;
    double GX,GY,GZ;
    double CX,CY,CZ;
    
    double Raw_Ax,Raw_Ay,Raw_Az;
    double Raw_Gx,Raw_Gy,Raw_Gz;
    double Raw_Cx,Raw_Cy,Raw_Cz;
    
    double Gyr_x_Drift;
    double Gyr_y_Drift;
    double Gyr_z_Drift;
    
    double norm;
};

#endif
