#ifndef _loopTime_h_
#define _loopTime_h_

#include <iostream>
#include <string.h>
#include <math.h>

class LoopTime
{
public:
  LoopTime(double_t s);
  ~LoopTime();
  void setTime(double_t s);
  void sleep();
private:
  struct timespec next_time;
  double_t loopTime;
};

class TimeoutCheck
{
public:
  TimeoutCheck(double_t s, double_t errRange = 0.05);
  ~TimeoutCheck();
  bool check(const char *log);
private:
  struct timespec last_time;
  struct timespec real_time;
  double_t loopTime;
  double_t _errRange;
};

#endif
