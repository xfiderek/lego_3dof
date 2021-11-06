
#include "hFramework.h"
//#include "hCloudClient.h"
#include <stddef.h>
#include <stdio.h>
#include <Lego_Touch.h>
#include <DistanceSensor.h>
#include <cmath>
#include <stdlib.h>

using namespace hModules;
using namespace hSensors;
using namespace hFramework;

const double ticksPerRot = 720.0;
const double h1Transmission = 54.0 / 24.0;
const double h2Transmission = -1.0;

// z = 16, x = 14, y = -1

// ikstart
const float l1 = 11.2;
const float l2 = 9.72;
const float l3 = 12.4;

const float xOffset = -7.2;
const float yOffset = 0;
const float zOffset = 5.2;

struct ArmState {
    double q1, q2, q3;
};

struct IkSolutions {
    double q1, q2a, q2b, q3a, q3b;
};

inline double rad2deg(double rad){
    return rad / 3.14 * 180; 
}

bool almostEqual(double x, double y, double epsilon = 0.01f){
   return fabs(x - y) < epsilon;
}

double eeHeadingWhenQ2IsZero(double q3){
    return atan2(l2 * sin(q3), l1 + l2 * cos(q3));
}

IkSolutions solveIk(float x, float y, float z) {
    IkSolutions solutions;
    solutions.q1 = atan2(y, x);
    
    // x w ukladzie przegubu 2
    double x2 = sqrt(x * x + y * y) - 0.8; 
    double z2 = z - l1;

    // promien koncowki w ukladzie pzregubu 2
    double d2 = sqrt(x2 * x2 + z2 * z2);
    
    // kat pod jakim jest koncowka w ukladzie przegubu 2   
    double theta = atan2(z2, x2);

    // rownanie nr 7  - https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html
    double c3 = (d2 * d2 - l2 * l2 - l3 * l3) / (2 * l2 * l3);
    if (almostEqual(c3, 1)){
        c3 = 1;
    }
    else if (almostEqual(c3, -1)){
        c3 = -1;
    }

    if (c3 > 1 || c3 < -1) {
        throw std::range_error("cosine should be between -1 and 1");
    }
    solutions.q3a = acos(c3);
    solutions.q3b = -solutions.q3a;

    solutions.q2a = theta - eeHeadingWhenQ2IsZero(solutions.q3a);
    solutions.q2b = theta - eeHeadingWhenQ2IsZero(solutions.q3b);

    Serial.printf("ik solution found maaaaan \n");
    // Serial.printf("q1: " << rad2deg(solutions.q1); 
    // Serial.printf("q2: " << rad2deg(solutions.q2a) << " " << rad2deg(solutions.q2b) << std::endl;
    // Serial.printf("q3: " << rad2deg(solutions.q3a) << " " << rad2deg(solutions.q3b) << std::endl;
    // Serial.printf("-------------------------" << std::endl;


    return solutions;
}

ArmState pickSolution(IkSolutions& solutions) {
    ArmState targetState;
    if (solutions.q2a > solutions.q2b){
        targetState.q1 = solutions.q1;
        targetState.q2 = solutions.q2a;
        targetState.q3 = solutions.q3a;
    }
    else {
        targetState.q1 = solutions.q1;
        targetState.q2 = solutions.q2b;
        targetState.q3 = solutions.q3b;
    }
    return targetState;
}

// ikend


double toAngle(int ticks){
    double angle = (double) ticks / ticksPerRot * 360.0;
    return ((angle > 180) || (angle < - 180)) ? std::fmod(angle, 180) : angle; // range -180, 180
}

int toTicks(double relAngle){
    return relAngle / 360 * ticksPerRot;
}

int calibrate(hMotor& mot){
    bool done = false;
    int step_value = 3;
    int desired_ticks;
    while (!done) {
        char c = Serial.getch();
        switch (c) {
        case 'z': {
            desired_ticks = -step_value;
            break;
        }
        case 'x': {
            desired_ticks = step_value;
            break;
        }
        case 'q': {
            done = true;
        }
        default: {
            desired_ticks = 0;
        }
        }
        mot.rotRel(desired_ticks, 500, false, INFINITE); //relative rotate 500 encoder ticks left with 20% of power with blocking task
        sys.delay(100);
        Serial.printf("\n encoder ctn: %d", mot.getEncoderCnt());
        Serial.printf("\n encoder ctn float: %f", (double) mot.getEncoderCnt());
    }
    return (double) mot.getEncoderCnt() ;
    
}

double pickSmallerDiff(double currAngle, double targetAngle){
    double diff = targetAngle - currAngle;
    if (abs(360 - abs(diff)) < abs(diff)) {
        int sgn = (diff > 0) - (diff < 0) ? 1 : -1;
        return - sgn * (360 - abs(diff)); 
    } 
    else {
        return diff;
    }
    
}

class MotorState {
private:
    hMotor* mot1;
    hMotor* mot2;
    hMotor* mot3;
    int mot1Offset, mot2Offset, mot3Offset;
    static double getMotor(hMotor* mot, int motOffset, double trans=1.0){
        Serial.printf("\n encoder ctn: %f", (double)mot->getEncoderCnt());

        Serial.printf("\n motor offset: %d",motOffset);

        Serial.printf("\n absolute angle %f", toAngle((double) (mot->getEncoderCnt()) - motOffset));
        return toAngle(((double) (mot->getEncoderCnt()) - motOffset) / trans);
    }

    static void setMotor(hMotor* mot, int motOffset, double angle, double trans=1.0){
        double currAngle = getMotor(mot, motOffset, trans);
        Serial.printf("\n target motor angle: %f \r\n", angle);
        
        Serial.printf("Current motor angle: %f \r\n", currAngle);

        double relAngle = pickSmallerDiff(currAngle, angle);
        Serial.printf("relative motor travel angle: %f \r\n", relAngle);
        mot->rotRel( (int) ((double)toTicks(relAngle) * trans), 500, false);
    }

public:
    MotorState(hMotor* mott1, hMotor* mott2, hMotor* mott3, int mot1Ticks, int mot2Ticks, int mot3Ticks){
        this->mot1 = mott1;
        this->mot2 = mott2;
        this->mot3 = mott3;
        this->mot1Offset = mot1Ticks;
        this->mot2Offset = mot2Ticks;
        this->mot3Offset = mot3Ticks;
    }

    double getMot1(){
        return getMotor(mot1, mot1Offset, h1Transmission);
    }

    double getMot2(){
        return getMotor(mot2, mot2Offset, h2Transmission);
    }
    double getMot3(){
        return getMotor(mot3, mot3Offset);
    }

    void setMot1(double angle){
        Serial.printf("target motor angle in setmot1: %f \r\n", angle);
        setMotor(mot1, mot1Offset, angle, h1Transmission);
    }

    void setMot2(double angle){
        setMotor(mot2, mot2Offset, angle, h2Transmission);
    }

    void setMot3(double angle){
        setMotor(mot3, mot3Offset, angle);
    }

};


void moveCartesian(MotorState& motorState, double x, double y, double z){
    IkSolutions solutions;
    ArmState armState;
    x = x - xOffset;
    y = y - yOffset;
    z = z - zOffset;
    Serial.printf("liczymy IK");
    try {
        solutions = solveIk(x, y, z);
        armState = pickSolution(solutions);
        Serial.printf("obliczone q1: %f \n", rad2deg(armState.q1));
        Serial.printf("obliczone q2: %f \n", rad2deg(armState.q2));
        Serial.printf("obliczone q3: %f \n", rad2deg(armState.q3));
        motorState.setMot1(rad2deg(armState.q1));
        sys.delay(1000);
        motorState.setMot2(rad2deg(armState.q2));
        sys.delay(1000);
        motorState.setMot3(rad2deg(armState.q3));
    }
    catch (std::range_error) {
        Serial.printf("\n poza zasiegiem");
        Serial.printf("\n poza zasiegiem");
        Serial.printf("\n poza zasiegiem");
        Serial.printf("\n poza zasiegiem");
        Serial.printf("\n poza zasiegiem");
    
    }
}

void hMain()
{   

    // calibrate();
    // hMotor* mot;
    // mot = &hMot1;
    sys.setLogDev(&Serial);
    // int encod_val;
    // int desired_ticks = 0;
    // int step_value = 10;
    // char c;
    hMot1.setEncoderPolarity(Polarity::Reversed);
    hMot2.setEncoderPolarity(Polarity::Reversed);
    hMot3.setEncoderPolarity(Polarity::Reversed);

    hMot1.rotRel(0, 200, false);
    hMot2.rotRel(0, 200, false);
    hMot3.rotRel(0, 200, false);

    int hMot1Offset = calibrate(hMot1);
    int hMot2Offset = calibrate(hMot2);
    int hMot3Offset = calibrate(hMot3);

    Serial.printf("hMot1Offset value: %d \r\n", hMot1Offset);
    Serial.printf("hMot2Offset value: %d \r\n", hMot2Offset);
    Serial.printf("hMot3Offset value: %d \r\n", hMot3Offset);

    MotorState motorState(&hMot1, &hMot2, &hMot3, hMot1Offset, hMot2Offset, hMot3Offset);
    
    Serial.printf("Motor value: %f \r\n", motorState.getMot1());

    // motorState.setMot1(180.0);
    // Serial.printf("\n\n 180 raeched. Motor value: %f \r\n", motorState.getMot1());
    // sys.delay(5000);
    // motorState.setMot1(-180.0);
    while (true) {
        Serial.printf("\nWpisz wartosci\n");
        int x = Serial.getch();
        x = x - 48;
        int y = Serial.getch();
        
        y = y - 48;
        int z = Serial.getch();

        z = z - 48;

        // motorState.setMot1(180.0);

        // Serial.getch();
        // motorState.setMot1(0.0);
        moveCartesian(motorState, x, y, z);
        
        Serial.printf("\n Wpisane x: %d \n", x);
        Serial.printf("\n Wpisane y: %d \n", y);
        Serial.printf("\n Wpisane z: %d \n", z);
        

        Serial.printf("\n potwierdz ze dojechales \n");
        Serial.getch();
        sys.delay(1000);
    }
    


   


    // calibrate(hMot1);
    // IServo &servo = hMot1.useAsServo(); // enable usage of hMot1 as servo
    // servo.calibrate(-90, 500, 90, 1500);
    
    
    // calibrate 
   
    
    // while (true)
    // {
    //     Serial.printf("next iter");
    //  // servo.rotAbs(desired_ticks); // rotate motor absolutely to set value

    // hMot1.resetEncoderCnt();
    
    // }

}
