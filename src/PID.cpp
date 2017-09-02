#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  p_error = 0;
  i_error = 0;
  d_error = 0;
  p = {0, 0, 0};
  prev_cte = 0;
  total_cte = 0;
  error = 0;
  found_best_error = false;
  dp = {1, 1, 1};
  twiddle_mark = {false, false, false};
  twiddle_up = {false, false, false};
  twiddle_down = {false, false, false};
  found_error = false;
  t = 0;
  tolerance = 0.2;
  best_error = 1000;
}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd, double tol) {
  p = {kp, ki, kd};
  t = 0;
  error = 0;
  prev_cte = 0;
  total_cte = 0;
  found_error = false;
  tolerance = tol;
}

void PID::UpdateError(double current_cte) {
  prev_cte = current_cte;
}

double PID::TotalError(double current_cte) {
  total_cte = total_cte + current_cte;
  return total_cte;
}

double PID::Steer(double current_cte) {
  // calculate steering
  p_error = current_cte;
  d_error = current_cte - prev_cte;
  i_error = TotalError(current_cte);
  double steer = - p_error * p[0] - d_error * p[1] - i_error * p[2];
  UpdateError(current_cte);
  return steer;
}

