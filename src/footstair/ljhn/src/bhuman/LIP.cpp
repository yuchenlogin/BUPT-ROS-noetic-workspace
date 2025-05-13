#include "LIP.h"
#include "BHAssert.h"
#include <iostream>


void LIP::setLIPHeight(float LIPHeight)
{
  ASSERT(LIPHeight != 0.f);
  height = LIPHeight;
  k = std::sqrt(Constants::g / height);
}

LIP& LIP::update(float timePassed, float zmp)
{
  const float sinhkt = std::sinh(k * timePassed);
  const float coshkt = std::cosh(k * timePassed);

  const float newPosition = (position - zmp) * coshkt + (velocity / k) * sinhkt + zmp;
  velocity = (position - zmp) * k * sinhkt + velocity * coshkt;
  position = newPosition;
  return *this;
}

float LIP::energy(float zmp) const
{
  return 0.5f * (velocity*velocity - k * k * (position - zmp)*(position - zmp));   //？？？？？？？？？？？
}

float LIP::energy(float pos, float vel, float comHeight)
{
  return 0.5f * ((vel*vel) - (Constants::g / comHeight) * (pos*pos));
}

float LIP::requiredPositionForPosition(float pos, float time, float zmp) const
{
  const float sinhkt = std::sinh(k * time);
  const float coshkt = std::cosh(k * time);

  return zmp + (pos - zmp - velocity * sinhkt / k) / coshkt;
}

float LIP::requiredVelocityForPosition(float pos, float time, float zmp) const
{
  ASSERT(time != 0.f);
  const float sinhkt = std::sinh(k * time);
  const float coshkt = std::cosh(k * time);

  return k * ((pos - zmp) - (position - zmp) * coshkt) / sinhkt;
}

float LIP::requiredVelocityForVelocity(float vel, float time, float zmp) const
{
  const float sinhkt = std::sinh(k * time);
  const float coshkt = std::cosh(k * time);

  return (vel - (position - zmp) * k * sinhkt) / coshkt;
}

float LIP::requiredZMPForPosition(float pos, float time) const
{
  const float kT = k * time;
  const float a = std::exp(kT);
  const float b = std::exp(-kT);

  // This formula produces bullshit at the end of a step, although it is just a conversion from the formula in exponentil form...
  //return (position * std::cosh(kT) + (velocity / k) * std::cosh(kT) - pos) / (std::cosh(kT) - 1.f);

  return (position * (a + b) + (velocity / k) * (a - b) - 2.f * pos) / (a + b - 2.f);
}

float LIP::requiredZMPForVelocity(float vel, float time) const
{
  ASSERT(time != 0.f);
  const float kT = k * time;
  const float a = std::exp(kT);
  const float b = std::exp(-kT);

  return (velocity * (a + b) * 0.5f - vel) / (k * (a - b) * 0.5f) + position;
}



double fxPos(double t, double w, double x0, double v0){
  double fx = x0*cosh(w*t)+v0/w*sinh(w*t);
  return fx;
}
double dfxPos(double t, double w, double x0, double v0){
  double dfx = x0*w*sinh(w*t)+v0*cosh(w*t);
  return dfx;
}

float LIP::timeToPosition(float pos, float zmp) const
{

  double t1=0.1, t2=0;
  while(true) {
      t2 = t1 - (fxPos(t1,k,position-zmp,velocity) - pos) / dfxPos(t1,k,position-zmp,velocity);
      if (fabs((t1 - t2))<0.000001) break;
      t1 = t2;
  }
  return t2;

}

double fxVel(double t, double w, double x0, double v0){
  double fx = x0*w*sinh(w*t)+v0*cosh(w*t);
  return fx;
}
double dfxVel(double t, double w, double x0, double v0){
  double dfx = x0*w*w*cosh(w*t)+v0*w*sinh(w*t);
  return dfx;
}

bool LIP::timeToVelocity(double *t, float vel, float zmp) const
{
  int count = 0;
  double t1=0.1, t2=0;
  while(true) {
      t2 = t1 - (fxVel(t1,k,position-zmp,velocity) - vel) / dfxVel(t1,k,position-zmp,velocity);
      if (fabs((t1 - t2))<0.000001) 
        break;
      t1 = t2;

      count++;
      if(count>10){
        *t = 0;
        return false;
      }
  }
  *t = t2;
  return true;
}



// float LIP::timeToPosition(float pos, float zmp) const
// {
//   const float a = position - zmp + velocity / k;
//   const float b = position - zmp - velocity / k;
//   const float p_z = pos - zmp;

//   const float c = std::sqrt(p_z*p_z - a * b);

//   const float res1 = std::log((p_z + c) / a) / k;
//   const float res2 = std::log((p_z - c) / a) / k;

//   if(!std::isfinite(res1))
//     return res2;
//   else if(!std::isfinite(res2))
//     return res1;
//   else
//     return res1 > res2 ? res1 : res2;
// }


// float LIP::timeToVelocity(float vel, float zmp) const
// {
//   const float a = (position - zmp)*k + velocity;
//   const float b = (position - zmp)*k - velocity;

//   const float c = std::sqrt((vel*vel) / (k*k) + a * b);

//   const float res1 = std::log((vel / k + c) / a) / k;
//   const float res2 = std::log((vel / k - c) / a) / k;

//   if(!std::isfinite(res1))
//     return res2;
//   else if(!std::isfinite(res2))
//     return res1;
//   else
//     return res1 > res2 ? res1 : res2;
// }
