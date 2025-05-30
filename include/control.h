#pragma once

#include "Wheel.h"
#include "types/structs.h"

// Updates the speeds of three wheels based on the current and target positions.
// The controller computes the distance and orientation error and then determines
// commanded speeds using proportional gains.
void updateWheels();