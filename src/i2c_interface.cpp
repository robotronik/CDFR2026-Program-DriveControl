#include "i2c_interface.h"
#include "I2C.h"
#include "interface/drive_interface.h"

uint8_t dataRet[64]; // Response buffer
int dataRetSize = 0;

drive_interface* robotI2cInterface;

void I2CDataSwitch(uint8_t* data, int size)
{
    uint8_t* dataPtr = data + 1;
    switch (data[0]){
        case CMD_GET_VERSION:
            dataRet[0] = I2C_VERSION;
            dataRetSize = sizeof(uint8_t);
        break;
        case CMD_SET_GREEN_LED:
            robotI2cInterface->set_green_led(data[1]);
            break;
        case CMD_SET_RED_LED:
            robotI2cInterface->set_red_led(data[1]);
            break;
        case CMD_GET_MOTION:
            {
                packed_motion_t motion = robotI2cInterface->get_motion();
                pack(dataRet, &motion, sizeof(packed_motion_t));
                dataRetSize = sizeof(packed_motion_t);
            }
            break;
        case CMD_SET_COORDINATES:
            {
                packed_position_t pos;
                unpack(dataPtr, &pos, sizeof(packed_position_t));
                robotI2cInterface->set_coordinates(pos);
            }
            break;
        case CMD_GET_TARGET:
            {
                packed_position_t target = robotI2cInterface->get_target();
                pack(dataRet, &target, sizeof(packed_position_t));
                dataRetSize = sizeof(packed_position_t);
            }
            break;
        case CMD_SET_TARGET:
            {
                packed_position_t pos;
                unpack(dataPtr, &pos, sizeof(packed_position_t));
                robotI2cInterface->set_target(pos);
            }
            break;
        case CMD_DISABLE:
            robotI2cInterface->disable();
            break;
        case CMD_ENABLE:
            robotI2cInterface->enable();
            break;
        case CMD_GET_CURRENT:
            {
                packed_motor_t current = robotI2cInterface->get_current();
                pack(dataRet, &current, sizeof(packed_motor_t));
                dataRetSize = sizeof(packed_motor_t);
            }
            break;
        case CMD_GET_SPEED:
            {
                packed_motor_t speed = robotI2cInterface->get_speed();
                pack(dataRet, &speed, sizeof(packed_motor_t));
                dataRetSize = sizeof(packed_motor_t);
            }
            break;
        case CMD_SET_BRAKE_STATE:
            robotI2cInterface->set_brake_state(data[1]);
            break;
        case CMD_SET_MAX_TORQUE:
            {
                double current;
                unpack(dataPtr, &current, sizeof(double));
                robotI2cInterface->set_max_torque(current);
            }
            break;
        case CMD_GET_STATUS:
            {
                status_t status = robotI2cInterface->get_status();
                pack(dataRet, &status, sizeof(status_t));
                dataRetSize = sizeof(status_t);
            }
            break;
        default:
            // Handle unknown command
            dataRet[0] = 0xFF; // Error code for unknown command
            dataRetSize = 1;
            break;
    }
    //if (dataRetSize > 0)
        //I2CDataSend(dataRet, dataRetSize);
}