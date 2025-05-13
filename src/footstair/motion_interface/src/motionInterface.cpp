#include "motionInterface.h"
#include "vrep/vrepBridge.h"
#include "roban/dxlOperate.h"

namespace Motion
{
  bool motionInterfaceSetup(std::string platform, MotionInterface *&interfacePtr)
  {
    if (strcmp(platform.c_str(), "sim") == 0)
    {
      interfacePtr = new vrepBridge::VrepMotion;
    }
    else if (strcmp(platform.c_str(), "roban") == 0)
    {
      interfacePtr = new Roban::DxlOperate;
    }
    return true;
  }
} // namespace Motion
