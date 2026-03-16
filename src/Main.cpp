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

void I2CRecieveData(uint8_t* data, int size) {
    I2CDataSwitch(data, size);
}

void testMotors();
void testloop(sequence* seq);

int main(void)
{
	//SETUP
	clock_setup();
	setupDeviceI2C();
	ledSetup();
    buttonSetup();
	usartSetup();
	DriveSetup();
	i2c_setup();
	setCallbackReceive(I2CRecieveData);

	usartprintf("Start\n");
	delay_ms(200);


	i2cDevice = new I2CDevice(kDefaultAddress);
	otos = new OTOS(i2cDevice);

	wheelA =  new Wheel(DISTANCE_WHEEL,180, DIAMETER_WHEEL, motorA);
	wheelB =  new Wheel(DISTANCE_WHEEL,-60, DIAMETER_WHEEL, motorB);
	wheelC =  new Wheel(DISTANCE_WHEEL, 60, DIAMETER_WHEEL, motorC);

	usartprintf("Looking for OTOS...\n");
	RedLED_Toggle();

	// Check the connection with the OTOS
	while (otos->begin() != ret_OK) {
		usartprintf("OTOS not connected\n");
		RedLED_Toggle();
		delay_ms(200); 
	}
	RedLED_Clear();
	usartprintf("OTOS connected\n");
	
	delay_ms(100);

	otos->setLinearScalar(1.04f); // This should be 1.0f, but we found that 1.04f gives better position tracking, likely to compensate for some scaling issue with the sensor measurements
	otos->setAngularScalar(1.0f);

	// Reset the position of the robot
	if (otos->calibrateImu() == ret_OK)
		usartprintf("OTOS IMU calibrated\n");
	else
		usartprintf("OTOS IMU calibration failed\n");

	otos->resetTracking();
	setPosition(0, 0, 0);
	setTarget(0, 0, 0);
	
//
//	Main Loop of the robot
//
	sequence ledToggleSeq;
    sequence mySeq;
    sequence dbg;
    bool isDebug = true;
	mySeq.reset();

	while (1) {
		uint32_t start_time = micros();

		updatePositionData();
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
			//usartprintf(">x:%.1lf+/-%.1lf mm, y:%.1lf+/-%.1lf mm, a:%.1lf+/-%.1lf deg\r\n", global_pos.x, global_pos_std_dev.x, global_pos.y, global_pos_std_dev.y, global_pos.a, global_pos_std_dev.a);
			usartprintf(">tx:%.1lf,\t ty:%.1lf,\t ta:%.1lf\r\n", global_target.x, global_target.y, global_target.a);
			usartprintf("> x:%.1lf,\t  y:%.1lf,\t  a:%.1lf\r\n", global_pos.x, global_pos.y, global_pos.a);
			//otos_status_t status;
			//otos->getStatus(status);
			//usartprintf("Status Tilt:%d, Optic:%d, PAA Err:%d, LSM Err:%d\r\n", status.warnTiltAngle, status.warnOpticalTracking, status.errorPaa, status.errorLsm);

		},50);

		//BLINK LED
		ledToggleSeq.interval([](){
			GreenLED_Toggle();
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