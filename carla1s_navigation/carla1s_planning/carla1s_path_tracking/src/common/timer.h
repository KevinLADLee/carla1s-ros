
#ifndef CARLA1S_ROS_TIMER_H
#define CARLA1S_ROS_TIMER_H

#include <chrono>

namespace carla1s {
class Timer{

 public:
  Timer(unsigned int frequency) {
    timer_frequency_ = frequency;
    double need_time = 1.0 / static_cast<double>(timer_frequency_);
    elapsed_time_ = std::chrono::duration<double>(need_time);
  };

  inline void TimerStart(){
    begin_time_ = std::chrono::steady_clock::now();
  }

  inline auto TimerEnd(){
    auto now = std::chrono::steady_clock::now();
    auto sleep_time = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time_ - (now - begin_time_));
    return sleep_time;
  }

 private:
  decltype(std::chrono::steady_clock::now()) begin_time_;
  unsigned int timer_frequency_;
  std::chrono::duration<double, std::milli> elapsed_time_;
};
}
#endif //CARLA1S_ROS_TIMER_H
