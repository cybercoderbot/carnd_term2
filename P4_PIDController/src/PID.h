#ifndef PID_H
#define PID_H
#include <vector>

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
  
  
  /*
  * Twiddle variables
  */
  std::vector<double> dp;
  
//   double best_error = 999.9;
  double total_error;
  double best_error = std::numeric_limits<double>::max();
  int param_index = 0;
  int current_state = 0; // possible values 0, 1, 2, 3

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Convenience function for adding amount (dp) to a PID controller parameter based on index
  */
  void AddToParameterAtIndex(int index, double amount);
};

#endif /* PID_H */
