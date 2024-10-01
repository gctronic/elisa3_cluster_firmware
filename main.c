
#include <avr\io.h>
#include <avr\interrupt.h>

#include <stdlib.h>
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"
#include "sensors.h"
#include "irCommunication.h"
#include "motors.h"

#define DEFAULT_SPEED 15

uint8_t inClusterState = 0;

int main(void) {

	unsigned long int demoStartTime = 0;
	unsigned int currRand = 0;
	uint8_t demoState = 0;
	int16_t desiredSpeed = 0;
	uint8_t oaEnabled = 0;

	initPeripherals();

	initBehaviors();

	speedStepCounter = getTime100MicroSec();
	
	// I noticed that I have to wait a little before calibrating in order to have the sensors to be 
	// well calibrated (sensors noise eliminated). Don't sure why, maybe due to the sensitivity of the 
	// sensor that stabilizes...
	demoStartTime = getTime100MicroSec();
	while((getTime100MicroSec() - demoStartTime) < PAUSE_300_MSEC);
	calibrateSensors();

	demoStartTime = getTime100MicroSec();

	srand(TCNT3+proximityValue[1]+proximityValue[9]);
	currRand = rand() % 256;	// 0 to 255

	while(1) {
		
		handleRFCommands();

		switch(demoState) {
			case 0:
				irCommInit();
				oaEnabled = 1;
				desiredSpeed = DEFAULT_SPEED;
				demoState = 1;
				break;

			case 1:
				irCommTasks();
				if(irCommDataSent()==1) {
					irCommSendData(currRand);
				}
				if(irCommDataAvailable()==1) {
					demoStartTime = getTime100MicroSec();
					irCommLastData = irCommReadData();
					if(irCommLastData != currRand) {  // Stop moving when receiving something from another robot to form a cluster. Avoid to stop when receiving bouncing transmitted data.
						inClusterState = 1; // See mirf.c, line 522						
						//updateGreenLed(0);						
						oaEnabled = 0;
						desiredSpeed = 0;
					}
				}
				if((getTime100MicroSec()-demoStartTime) >= (PAUSE_5_SEC)) {
					inClusterState = 0; // See mirf.c, line 522					
					//updateGreenLed(255);					
					oaEnabled = 1;
					desiredSpeed = DEFAULT_SPEED;
				}
				break;
		}
		if(oaEnabled) {
			enableObstacleAvoidance();
		} else {
			disableObstacleAvoidance();
		}
		setLeftSpeed(desiredSpeed);
		setRightSpeed(desiredSpeed);
		handleMotorsWithSpeedController();  

	} // while(1)

}
