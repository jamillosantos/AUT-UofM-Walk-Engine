/*
 * MPUGY80.h
 *
 *  Created on: 2015. 9. june.
 *      Author: Mojtaba Karimi
 *      Email:  mojtaba_k@live.com
 *      Web:    mojtabakarimi.ml
 */
#ifndef KalmanFilter2D_h
#define KalmanFilter2D_h

#include <math.h>
////////// define RC-100 button key value ////////////////
//#define RC100_BTN_U		(1)

class KalmanFilter2D 
{
  public:
    KalmanFilter2D() 
    {
        /* We will set the variables like so, these can also be tuned by the user */
        Q_angle   = 0.1; // Process noise variance for the accelerometer
        Q_bias    = 0.01;  // Process noise variance for the gyro bias
        R_measure = 140.0; // Measurement noise variance - this is actually the variance of the measurement noise

        angle = 0.0; // Reset the angle
        bias  = 0.0; // Reset bias

        P[0][0] = 0.0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 0.0;
    };
    
    /*
    * The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    */
    double getAngle(double newAngle, double newRate, double dt) 
    {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        rate = newRate - bias;
        angle += dt * rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        S = P[0][0] + R_measure;
        /* Step 5 */
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        y = newAngle - angle;
        /* Step 6 */
        angle += K[0] * y;
        bias += K[1] * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];

        return (double)angle;
    };
    
    /*
    * set start angle
    * Used to set angle, this should be set as the starting angle
    */
    void setAngle(double newAngle)
    { 
      angle = newAngle; 
    }; 
    
    /*
    * Return the unbiased rate
    */
    double getRate() 
    { 
      return rate; 
    }; 

    /* 
    * These are used to tune the Kalman filter 
    */
    void setQangle(double newQ_angle) 
    {
      Q_angle = newQ_angle; 
    };
    
    /* 
    * set q bias to kalman filter 
    */
    void setQbias(double newQ_bias) 
    { 
      Q_bias = newQ_bias; 
    };
    
    /* 
    * set r mesure to kalman filter
    */
    void setRmeasure(double newR_measure) 
    { 
      R_measure = newR_measure; 
    };
    
    /* 
    * get q angle
    */
    double getQangle() 
    { 
      return Q_angle; 
    };
    
    /* 
    * get q bias from kalman filter internal 
    */
    double getQbias() 
    { 
      return Q_bias; 
    };
    
    /* 
    * get r mesure from kalman filter 
    */
    double getRmeasure() 
    { 
      return R_measure; 
    };

  /*
  * private internal value for kalman filter
  */
  private:
    /* Kalman filter variables */
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias;  // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error
};

#endif
