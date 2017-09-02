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
  std::vector<double> p;
  double prev_cte;
  double total_cte;
  double error;
  bool found_best_error;
  std::vector<double> dp;
  std::vector<bool> twiddle_mark;
  std::vector<bool> twiddle_up;
  std::vector<bool> twiddle_down;
  bool found_error;
  int t;
  double tolerance;
  double best_error;
  bool initialized;

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
  void Init(double Kp, double Ki, double Kd, double tolerance=0.2);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double cte);

  double Steer(double cte);
};

#endif /* PID_H */
