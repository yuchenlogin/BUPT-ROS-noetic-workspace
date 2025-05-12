#ifndef _walkDataStruct_h_
#define _walkDataStruct_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

class Posture
{
public:
    double_t x;
    double_t y;
    double_t z;
    double_t roll;
    double_t pitch;
    double_t yaw;

public:
    Posture();
    ~Posture();
    void zero();
};

#endif
