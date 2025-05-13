#include "loopTime.h"

LoopTime::LoopTime(double_t s)
{
  setTime(s);
}

LoopTime::~LoopTime()
{
}

void LoopTime::setTime(double_t s)
{
  loopTime = s;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
}

void LoopTime::sleep()
{
  next_time.tv_sec += (next_time.tv_nsec + loopTime * 1e9) / 1e9;
  next_time.tv_nsec = (int)(next_time.tv_nsec + loopTime * 1e9) % (int)1e9;
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
}

TimeoutCheck::TimeoutCheck(double_t s, double_t errRange)
{
  loopTime = s;
  _errRange = errRange;
  clock_gettime(CLOCK_MONOTONIC, &last_time);
}

TimeoutCheck::~TimeoutCheck()
{
}

bool TimeoutCheck::check(const char *log)
{
  clock_gettime(CLOCK_MONOTONIC, &real_time);
  double_t time = (real_time.tv_sec + real_time.tv_nsec * 1e-9) - (last_time.tv_sec + last_time.tv_nsec * 1e-9);
  last_time = real_time;
  if (time > loopTime * (1 + _errRange))
  {
    if (log != NULL)
    {
      std::cout << log << " timeout(s), goal: " << loopTime << ", real: " << (double_t)time << std::endl;
    }
    return true;
  }
  return false;
}