#include "walkDataStruct.h"

using namespace GaitManager;

Posture::Posture()
{
  x = 0.0;
  y = 0.0;
  z = 0.0;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
}

Posture::~Posture()
{
}

void Posture::zero()
{
  x = 0.0;
  y = 0.0;
  z = 0.0;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
}
