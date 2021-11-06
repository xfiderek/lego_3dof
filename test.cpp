
#include "hFramework.h"
//#include "hCloudClient.h"
#include <stddef.h>
#include <stdio.h>
#include <Lego_Touch.h>
#include <DistanceSensor.h>


using namespace hModules;
using namespace hSensors;
using namespace hFramework;

// void hMain()
// {
// 	sys.setLogDev(&Serial);
// 	DistanceSensor sens(hSens1);
// 	hLegoSensor_simple ls(hSens5);
// 	Lego_Touch touchSensor(ls);
	
// 	const int demandedDist = 10;
	
// 	const double pConst = 35;
// 	const int minMotorPower = 300;
// 	bool touchBtnState = false;
// 	bool lastTouchPressed = false;

// 	for (;;)
// 	{

// 		bool touchPressed = touchSensor.isPressed();
// 		if (!lastTouchPressed && touchPressed){
// 			touchBtnState = !touchBtnState;
// 		}
// 		lastTouchPressed = touchPressed;

// 		int currDist = sens.getDistance();
// 		int diff = currDist - demandedDist;

// 		if (touchBtnState && abs(diff) > 2 && diff < 100){
// 			int motorPower = max((int) abs(diff * pConst), minMotorPower);
// 			motorPower = motorPower * ((diff > 0) - (diff < 0));
// 			hMot1.setPower(motorPower);
// 		}
// 		else {
// 			hMot1.setPower(0);
// 		}
// 		sys.delay(20);
// 		Serial.printf("distance from sensor %d \r\n", currDist);
// 		Serial.printf("touchBtnState %d \r\n\n\n", touchBtnState);	
// 	}
		
// }



void hMain()
{   
    sys.setLogDev(&Serial);
    int encod_val;
    int desired_angle = 0;
    int step_value = 50;
    char c;
    hMot1.setEncoderPolarity(Polarity::Reversed);
    while (true)
    {
        sys.delay(100);
        encod_val = (int) hMot1.getEncoderCnt();
        Serial.printf("next iter");
        c = Serial.getch();
        if (c == 'a')
            {
                desired_angle = step_value;
            }
        else if (c == 'b')
            {
                desired_angle = -step_value;
            }
        else
            {
                desired_angle = 0;
            }
        hMot1.rotRel(desired_angle, 200, true, INFINITE); //relative rotate 500 encoder ticks left with 20% of power with blocking task
        Serial.printf("Encoder value: %d \r\n", encod_val);
    }

}
