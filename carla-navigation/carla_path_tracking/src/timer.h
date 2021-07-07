
#ifndef CARLA1S_ROS_TIMER_H
#define CARLA1S_ROS_TIMER_H

#include <chrono>

namespace carla1s {
class Timer{
 public:
  Timer(unsigned int frequency) {
    timer_frequency_ = frequency;
  };
  void TimerStart(){
    begin_time_ = std::chrono::steady_clock::now();
  }

  auto TimerEnd(){
    auto sleep_time = std::chrono::milliseconds(0);
    if(timer_frequency_==0){
      return sleep_time;
    } else{
      auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time_);
      auto need_time = 1000 / timer_frequency_;
      sleep_time = std::chrono::milliseconds(need_time) - cost_time;
      if (sleep_time <= std::chrono::milliseconds(0)) {
        sleep_time = std::chrono::milliseconds(0);
      }
      return sleep_time;
    }
  }

 private:
  decltype(std::chrono::steady_clock::now()) begin_time_;
  unsigned int timer_frequency_;
};
}
#endif //CARLA1S_ROS_TIMER_H
