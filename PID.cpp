#include "PID.h"
using namespace std;

PID::PID() {}
PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	error_vector_.resize(3);
}

vector<double> PID::UpdateError(double cte) {
	error_vector_[2] = cte - error_vector_[0];
	error_vector_[0] = cte;
	error_vector_[1] += cte;
	return error_vector_;
}

double PID::TotalError() {
	return -(Kp_*error_vector_[0] + Ki_*error_vector_[1] + Kd_*error_vector_[2]);
}
