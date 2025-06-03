#pragma once

#include <stdint.h>

#include "interface/drive_interface.h"

extern drive_interface* robotI2cInterface;

void I2CDataSwitch(uint8_t* data, int size);