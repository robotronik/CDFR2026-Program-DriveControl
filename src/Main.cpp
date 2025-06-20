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

// If debug is define, the code will be compiled with debug information
// #define MUTE_UART

#ifdef SIMULATION
	#include "hardware_interface.h"
	#define main stm_main
#endif

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
    buttonSetup();
	usartSetup();
	DriveSetup();
	i2c_setup();
	setCallbackReceive(I2CRecieveData);
	setupDeviceI2C();

	usartprintf("Start\n");
	delay_ms(100);


	i2cDevice = new I2CDevice(kDefaultAddress);
	otos = new OTOS(i2cDevice);

	wheelA =  new Wheel(DISTANCE_WHEEL,180, DIAMETER_WHEEL, motorA);
	wheelB =  new Wheel(DISTANCE_WHEEL,-60, DIAMETER_WHEEL, motorB);
	wheelC =  new Wheel(DISTANCE_WHEEL, 60, DIAMETER_WHEEL, motorC);
	
	usartprintf("Looking for OTOS...\n");

	// Check the connection with the OTOS
	while (otos->begin() != ret_OK) {
		usartprintf("OTOS not connected\n");
		RedLED_Toggle();
		delay_ms(200);
	}
	usartprintf("OTOS connected\n");
	RedLED_Clear();

	// Reset the position of the robot
	otos->calibrateImu();

	setPosition(0, 0, 0);
	setTarget(0, 0, 0);
	
	
    DriveEnable();


//
//	Main Loop of the robot
//
	sequence ledToggleSeq;
    sequence mySeq;
    sequence dbg;
    bool isDebug = false;

	while (1) {
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
			usartprintf(">x:%.1lf,y:%.1lf,a:%.1lf\r\n", global_pos.x, global_pos.y, global_pos.a);;
		},100);

		//BLINK LED
		ledToggleSeq.interval([](){
			GreenLED_Toggle();
		},750);
	}

	return 0;
}

//
// Test motor
// Accelerate Forward -> Decelerate Forward -> Accelerate backward -> Decelerate backward
//
void testMotors(){
    DriveEnable();
    for (double i = 0.0; i <= 100.0; i+=5) {
        usartprintf("%g\n",i);
		motorA->SetSpeedSigned(i);
		motorB->SetSpeedSigned(i);
		motorC->SetSpeedSigned(i);
		motorA->PrintValues();
		motorB->PrintValues();
		motorC->PrintValues();
        delay_ms(100);
    }
    for (double i = 100.0; i > -100.0; i-=5) {
        usartprintf("%g\n",i);
		motorA->SetSpeedSigned(i);
		motorB->SetSpeedSigned(i);
		motorC->SetSpeedSigned(i);
		motorA->PrintValues();
		motorB->PrintValues();
		motorC->PrintValues();
        delay_ms(100);
    }
    for (double i = -100.0; i <= 0.0; i+=5) {
        usartprintf("%g\n",i);
		motorA->SetSpeedSigned(i);
		motorB->SetSpeedSigned(i);
		motorC->SetSpeedSigned(i);
		motorA->PrintValues();
		motorB->PrintValues();
		motorC->PrintValues();
        delay_ms(100);
    }

	
	motorB->SetSpeedSigned(100);
	delay_ms(1000);
	motorB->SetSpeedSigned(0);
	delay_ms(500);
	motorB->SetSpeedSigned(-100);
	delay_ms(1000);
	motorB->SetSpeedSigned(0);

	//DriveDisable();
}

void testloop(sequence* seq) {
	seq->start();

	seq->delay([](){
        robotI2cInterface->set_coordinates({0,0,0});
        robotI2cInterface->enable();
		robotI2cInterface->set_target({200,0,0});
	},0);

	seq->delay([](){
		robotI2cInterface->set_target({0,0,0});
	},2000);
}