#pragma once
#include "types/structs.h"

// Read only
extern position_t global_pos;
extern position_t global_pos_std_dev;
extern position_t global_vel;
extern position_t global_acc;
extern position_t global_target;

void updatePositionData();

void setPosition(position_t incommingPos);
void setPosition(double x, double y, double a);
void setTarget(position_t incommingPos);
void setTarget(double x, double y, double a);