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
  int step, param_index;
  // number of steps to allow changes to settle, then to evaluate error
  int n_settle_steps, n_eval_steps;
  bool tried_adding, tried_subtracting, twiddle;
  
//   double best_error = 999.9;
  double total_error;
  double best_error = std::numeric_limits<double>::max();
  int last_optimize_index = 0;
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
