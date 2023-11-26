#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:

  double Kp_;
  double Ki_;
  double Kd_;

  std::vector<double> error_vector_;
  PID();
  virtual ~PID();

  void Init(double Kp, double Ki, double Kd);

  std::vector<double> UpdateError(double cte);

  double TotalError();
};

#endif
