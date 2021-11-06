#include "hFramework.h"
//#include "hCloudClient.h"
#include <stddef.h>
#include <stdio.h>
#include <Lego_Touch.h>
#include <DistanceSensor.h>


using namespace hModules;
using namespace hSensors;
using namespace hFramework;

void hMain()
{
	sys.setLogDev(&Serial);
	DistanceSensor sens(hSens1);
	hLegoSensor_simple ls(hSens5);
	Lego_Touch touchSensor(ls);
	
	const int demandedDist = 10;
	
	const double pConst = 35;
	const int minMotorPower = 300;
	bool touchBtnState = false;
	bool lastTouchPressed = false;

	for (;;)
	{

		bool touchPressed = touchSensor.isPressed();
		if (!lastTouchPressed && touchPressed){
			touchBtnState = !touchBtnState;
		}
		lastTouchPressed = touchPressed;

		int currDist = sens.getDistance();
		int diff = currDist - demandedDist;

		if (touchBtnState && abs(diff) > 2 && diff < 100){
			int motorPower = max((int) abs(diff * pConst), minMotorPower);
			motorPower = motorPower * ((diff > 0) - (diff < 0));
			hMot1.setPower(motorPower);
		}
		else {
			hMot1.setPower(0);
		}
		sys.delay(20);
		Serial.printf("distance from sensor %d \r\n", currDist);
		Serial.printf("touchBtnState %d \r\n\n\n", touchBtnState);	
	}
		
}
