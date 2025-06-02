#pragma once

#include "stdint.h"

#define PI 3.14159265359
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI

typedef struct{
	double x = 0;
    double y = 0;
	double a = 0;
}position_t;

typedef struct{
	double x = 0;
    double y = 0;
	double a = 0;
	uint64_t time = 0;
}position_time_t;

typedef enum {
    /// @brief Success
    ret_OK = 0,

    /// @brief Fail
    ret_FAIL = -1,
} return_t;