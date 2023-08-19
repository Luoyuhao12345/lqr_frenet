#include "pid.h"

PID::PID(float a, float b, float c)
{
    p = a;
    i = b;
    d = c;
    total_e = 0;
    cur_e = 0;
}

float PID::pid(float steer, float e)
{
    float pi = 3.1415;
    float k = 2;
    e = -e;
    float cmp = p*e;
    if(cmp > k*pi/180) cmp = k*pi/180;
    if(cmp < -k*pi/180) cmp = -k*pi/180;
    float c = steer+cmp;
    return c;
}