#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

#define CTE_THRESHOLD 4

PID::PID() 
{
  
  p_error = 0;
  i_error = 0;
  d_error = 0;

  /*
  * Coefficients
  */ 
  Kp = 0.1;
  Ki = 0.0;
  Kd = 2.5;
  
  use_twiddle = false;
  request_reset = false;
  
  i_error_min = -10;
  i_error_max = 10;
  
  best_err = 1000; 
  err_tolerance = 0.001;
  
  dp_error = 0.05;
  di_error = 0.0001;
  dd_error = 0.1;
  
  iter = 1;
  check = 1;
  index = 0;
  
  total_error = 0;
  
  count = 0;
  
  max_cte = 0;
  
}

PID::~PID() 
{
}

void PID::Init(double Kp_, double Ki_, double Kd_, double i_error_min_ , double i_error_max_, bool use_twiddle_, double err_tolerance_) 
{

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  i_error_min = i_error_min_;
  i_error_max = i_error_max_;
  
  use_twiddle = use_twiddle_;
  err_tolerance = err_tolerance_;
  
  request_reset = false;
  
  max_cte = 0;
}

void PID::ReInit()
{
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  iter = 1;
  
  total_error = 0; 
  request_reset = false;   
}

double PID::UpdateError(double cte) 
{
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  
  // Set the upper and lower bounds for i_error
  if(i_error > i_error_max)
    i_error = i_error_max;
  if(i_error < i_error_min)
    i_error = i_error_min;
    
  if((use_twiddle == true))
  {
    Twiddle(cte);
  }
    
  return double((-(Kp*p_error + Ki*i_error + Kd*d_error)));
}

double PID::TotalError(double cte) 
{
  if(total_error == 0)
  {
    total_error += fabs(cte);
  }
  else
  {
    total_error += fabs(cte);
  }
  
  if(max_cte < fabs(cte))
    max_cte = fabs(cte);
    
  return total_error;
}

double PID::AverageError(double total_error, int iter) 
{
  if(iter <= MIN_FRAMES)
  {
    return total_error;
  }
  else
  {
    return total_error/(iter - MIN_FRAMES);
  }
}


void PID::Twiddle(double cte) 
{

  double p[3] = {Kp, Ki, Kd};
  double dp[3] =  {dp_error, di_error, dd_error};

  if(( (fabs(dp_error) + fabs(di_error) + fabs(dd_error)) > err_tolerance) && (check < 2))
  {
    

    if((iter < MIN_FRAMES))
    {
      
    }
    else
    {
      //Calculate Total Error
      TotalError(cte);
      
      
      if(((iter % MAX_FRAMES) == 0) /*|| (fabs(cte) > 3)*/)
      {
        if((AverageError(total_error,iter) < best_err) && (check == 1))
        { 
          // case for positive
          
          // if((fabs(cte) < CTE_THRESHOLD))
          {  
            best_err = AverageError(total_error,iter);
            
          }
            
          dp[index] *= 1.3;
          
          if(count >= COUNT_THRESHOLD)
          {
            index = (index + 1)%3;
            count = 0;
          }
          else
          {
            count++;  
          }
          request_reset = true;
          p[index] += dp[index];
        }
        else
        {
          if(check == 1)
          {
            check = 0;
            p[index] -= 2 * dp[index];
            request_reset = true;
          }
          else if(check == 0)
          {

            
            if((AverageError(total_error,iter) < best_err) && (check == 0))
            { 
              //if((fabs(cte) < CTE_THRESHOLD))
              {
                // case for negative
                best_err = AverageError(total_error,iter);
              }
              dp[index] *= 1.3;
            }
            else
            {
              p[index] += dp[index];
              dp[index] /= 1.3;
            }            
            
            check = 1;
            if(count >= COUNT_THRESHOLD)
            {
              index = (index + 1)%3;
              count = 0;
            }
            else
            {
              count++;  
            }

            request_reset = true;
            p[index] += dp[index];
          }
        
        }
       
      }
    }

    //p[index] += dp[index];
    std::cout << "Twiddle Iteration:" << iter++;
    std::cout << "\tCTE: " << cte; 
    std::cout << "\tTE: " << total_error;
    std::cout << "\tBE: " << best_err;    
   
  }
  else
  {
    if(check < 2)
    {
      check = 2; // Stop twiddling once the values reached
      std::cout << "\nTwiddle Final Values" << std::endl; 
      std::cout << "\tCTE: " << cte;
      std::cout << "\tTE: " << total_error;
      std::cout << "\tBE: " << best_err;
      iter = MAX_FRAMES;
    }
  
  }
  
  
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
  
  dp_error = dp[0];
  di_error = dp[1];
  dd_error = dp[2];
  
  std::cout << "\tMax CTE:" << max_cte;
  std::cout << "\tKp: " << p[0];    
  std::cout << "\tKi: " << p[1];    
  std::cout << "\tKd: " << p[2]; 
  std::cout << "\tdp: " << dp[0];    
  std::cout << "\tdi: " << dp[1];    
  std::cout << "\tdd: " << dp[2] << std::endl;  
}

