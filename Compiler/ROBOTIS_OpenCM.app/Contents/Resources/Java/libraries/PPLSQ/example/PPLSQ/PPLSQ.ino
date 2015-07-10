#include <orpol.h>
#include <pplsq.h>

#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

int POLY_DEGREE = 4;
int TEST_SIZE = 100;
  
double dataArray[100]; //Real points we wanna fit into
double time[100];
double fittedPoints[100]; //Fitted points, just for graphing
double coefficients[4]; //These will be the calculated coefficients
  
void setup() 
{
  // put your setup code here, to run once:
  SerialUSB.begin();
  
}

void loop() 
{  
  for(int i=0 ; i < TEST_SIZE ; i++) //Some samples... 
    {
        dataArray[i] = cosd(i) + 0.3*( ((rand() % 10) / 100.0) -0.1);
        //dataArray[i] = cosd(i);
        time[i] = i;
    }
  //int t=micros();
  double sq = pplsq(time, dataArray, TEST_SIZE, coefficients, POLY_DEGREE); //fit and return squared residual
  //t=micros()-t;
  for(int i=0 ; i < TEST_SIZE ; i++)
    {
        fittedPoints[i] = f(i, coefficients, POLY_DEGREE);
    }
    
  for(int i=0 ; i < TEST_SIZE ; i++)
    {
       SerialUSB.print(i);SerialUSB.print("\t");
       SerialUSB.print(dataArray[i]);SerialUSB.print("\t");
       SerialUSB.println(fittedPoints[i]);
    }  
    
  SerialUSB.println("-----------");
  delay(5000);
}

double f(double x, double* coefficients, int degree)
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
