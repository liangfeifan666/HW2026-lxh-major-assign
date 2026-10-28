/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 带积分限幅与条件积分抗饱和的位置式 PID
 *******************************************************************************
 */
#ifndef PID_HPP_
#define PID_HPP_

namespace hello_world {
namespace pid {

class Pid {
 public:
  Pid(float kp, float ki, float kd, float integral_limit,
      float output_limit, float sample_time = 0.001f);

  float calculate(float target, float feedback);
  void reset();

  float integral() const { return integral_; }

 private:
  float kp_;
  float ki_;
  float kd_;
  float integral_limit_;
  float output_limit_;
  float sample_time_;
  float integral_ = 0.0f;
  float previous_error_ = 0.0f;
};

}  // namespace pid
}  // namespace hello_world

#endif /* PID_HPP_ */
