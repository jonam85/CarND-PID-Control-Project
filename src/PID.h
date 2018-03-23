#ifndef PID_H
#define PID_H

#define MIN_FRAMES 50 // Wait for min frames till speed picks up
#define MAX_FRAMES 1000
#define COUNT_THRESHOLD 10

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  double i_error_min;
  double i_error_max;
  
  bool use_twiddle;
  bool request_reset;
  
  double best_err;
  double err_tolerance;
  
  double dp_error;
  double di_error;
  double dd_error;
  
  int iter;
  int check;
  int index;
  
  int count;
  
  double total_error;
  double max_cte;
  
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_, double i_error_min_ , double i_error_max_, bool use_twiddle_, double err_tolerance_);
  
  void ReInit();
  /*
  * Update the PID error variables given cross track error.
  */
  double UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double cte);
  
  double AverageError(double total_error, int iter);
  
  void Twiddle(double cte);
};

#endif /* PID_H */
