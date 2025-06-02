#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define STRUCT_PACK __attribute__((__packed__))

typedef struct {
    bool is_error1:1;
    bool is_error2:1;
} STRUCT_PACK status_t;

typedef struct {
    double x = 0.0; // in mm
    double y = 0.0; // in mm
    double a = 0.0; // in degrees
} STRUCT_PACK packed_vector_t;

typedef struct {
    packed_vector_t pos;
    packed_vector_t vel;
    packed_vector_t acc;
} STRUCT_PACK packed_motion_t;

typedef struct {
    double A;
    double B;
    double C;
} STRUCT_PACK packed_motor_t;

// Increment this version number when the I2C protocol changes.
#define I2C_VERSION 0x01

#define CMD_GET_VERSION 0x01
#define CMD_SET_GREEN_LED 0x11
#define CMD_SET_RED_LED 0x12
#define CMD_GET_MOTION 0x21
#define CMD_SET_COORDINATES 0x22
#define CMD_GET_TARGET 0x23
#define CMD_SET_TARGET 0x24
#define CMD_DISABLE 0x31
#define CMD_ENABLE 0x32
#define CMD_GET_CURRENT 0x41
#define CMD_GET_SPEED 0x42
#define CMD_SET_BRAKE_STATE 0x51
#define CMD_SET_MAX_TORQUE 0x52
#define CMD_GET_STATUS 0x61

class drive_interface
{
public:
    drive_interface(){};
    ~drive_interface(){};

    uint8_t get_version();

    void set_green_led(bool status);
    void set_red_led(bool status);

    packed_motion_t get_motion();
    void set_coordinates(packed_vector_t pos);

    packed_vector_t get_target();
    void set_target(packed_vector_t pos);

    void disable();
    void enable();

    packed_motor_t get_current(); // in Amps
    packed_motor_t get_speed(); // in rps

    void set_brake_state(bool enable);

    void set_max_torque(double current); // in Amps
    
    status_t get_status();
};

// Generic pack: struct -> byte buffer
inline void pack(uint8_t* buffer, const void* src_struct, size_t size) {
    memcpy(buffer, src_struct, size);
}

// Generic unpack: byte buffer -> struct
inline void unpack(const uint8_t* buffer, void* dest_struct, size_t size) {
    memcpy(dest_struct, buffer, size);
}

/*

    packed_motion_t motion = {
        .pos = {1.0, 2.0, 3.0},
        .vel = {4.0, 5.0, 6.0},
        .acc = {7.0, 8.0, 9.0}
    };

    uint8_t buffer[sizeof(packed_motion_t)];

    // Pack the struct into the buffer
    pack(buffer, &motion, sizeof(motion));

    // Unpack into a new struct
    packed_motion_t received;
    unpack(buffer, &received, sizeof(received));

    std::cout << "Received acc.a = " << received.acc.a << std::endl;
    
*/