#pragma once

#include <math.h>

inline double mod_angle(double a){
	a = fmod(a,360);
	if (a > 180)
		a -= 360;
	else if (a < -180)
		a += 360;
	return a;
}