#include "config.h"
#include "Motor.h"
#include "Wheel.h"
#include "uart.h"
#include "clock.h"
#include "I2C.h"
#include "led.h"
#include "types/sequence.h"
#include "position.h"
#include "control.h"
#include "interface/drive_interface.h"
#include "i2c_interface.h"
#include "button.h"
#include "odometry/I2Cdevice.h"
#include "odometry/OTOS.h"
#include "odometry/odometry_fusion.h"


void I2CRecieveData(uint8_t* data, int size) {
    I2CDataSwitch(data, size);
}

void testMotors();
void testloop(sequence* seq);

int main(void)
{
	//SETUP
	clock_setup();
	ledSetup();
	RedLED_Set();
    buttonSetup();
	usartSetup();
	DriveSetup();
	setupDeviceI2C();
	i2c_setup();
	setCallbackReceive(I2CRecieveData);

	usartprintf("Starting...\n");

	delay_ms(200);

	odometry_config_t odo_cfg;
	odo_cfg.sensor_spacing_mm = OTOS_SENSOR_SPACING_MM;
	odo_cfg.lowpass_alpha = 0.75f;
	odo_cfg.address_left = OTOS_LEFT_ADDRESS;
	odo_cfg.address_right = OTOS_RIGHT_ADDRESS;
	odometrySetup(odo_cfg);

	wheelA =  new Wheel(DISTANCE_WHEEL,180, DIAMETER_WHEEL, motorA);
	wheelB =  new Wheel(DISTANCE_WHEEL,-60, DIAMETER_WHEEL, motorB);
	wheelC =  new Wheel(DISTANCE_WHEEL, 60, DIAMETER_WHEEL, motorC);

	setPosition(0, 0, 0);
	setTarget(0, 0, 0);
	
	RedLED_Clear();
//
//	Main Loop of the robot
//
	sequence ledToggleSeq;
    sequence mySeq;
    sequence dbg;
    bool isDebug = false;

	while (1) {
		uint32_t start_time = micros();

		updatePositionData();
		calculateSlip();
		updateWheels();

        if (isDebug) {
            testloop(&mySeq);
        }
        else if (readTestButton()) {
            isDebug = true;
            mySeq.reset();
        }

		if (readPushButton())
			testMotors();

		// Write the position to debug console
		dbg.interval([](){
			odometryDebugPrint();
		},30);

		//BLINK LED
		ledToggleSeq.interval([](){
			if (!is_odometry_ok){
				GreenLED_Clear();
				RedLED_Toggle();
			}
			else {
				RedLED_Clear();
				GreenLED_Toggle();
			}
		},250);

		while (micros() - start_time < 4000); // 250Hz loop
	}

	return 0;
}

//
// Test motor
// Accelerate Forward -> Decelerate Forward -> Accelerate backward -> Decelerate backward
//
void testMotors(){
	setPosition(0, 0, 0);
	return;
    DriveEnable();
	ResetDrive();
	double s = 30.0;

    for (int i = 0; i < 100; i++) {
        usartprintf("%g\n",i);
		motorA->SetSpeedSigned(s);
		motorB->SetSpeedSigned(s);
		motorC->SetSpeedSigned(s);
		motorA->PrintValues();
		motorB->PrintValues();
		motorC->PrintValues();
        delay_ms(3);
    }
    for (int i = 0; i < 100; i++) {
        usartprintf("%g\n",i);
		motorA->SetSpeedSigned(-s);
		motorB->SetSpeedSigned(-s);
		motorC->SetSpeedSigned(-s);
		motorA->PrintValues();
		motorB->PrintValues();
		motorC->PrintValues();
        delay_ms(3);
    }

	DriveDisable();
}

void testloop(sequence* seq) {
	seq->start();

	seq->delay([](){
        robotI2cInterface->set_coordinates({0,0,0});
        robotI2cInterface->enable();
		robotI2cInterface->set_target({0,0,90});
	},0);

	seq->delay([](){
		robotI2cInterface->set_target({0,0,0});
	},2000);
}