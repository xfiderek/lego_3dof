
#include "hFramework.h"
//#include "hCloudClient.h"
#include <stddef.h>
#include <stdio.h>
#include <Lego_Touch.h>
#include <DistanceSensor.h>
#include <cmath>
#include <stdlib.h>
#include <trajectory_factories.h>
#include <vector>
#include <vector3.h>
#include <helpers.h>
#include <Lego_Touch.h>

#define PI 3.14159265

using namespace hModules;
using namespace hSensors;
using namespace hFramework;

const double ticksPerRot = 720.0;
// const double h1Transmission = 54.0 / 24.0;
const double h1Transmission = - 54.0 / 8.0;

const double h2Transmission = - 40.0 / 24.0;

const double h3Transmission = 1.0;

// z = 16, x = 14, y = -1

// ikstart
const double l1 = 14.2;
const double l2 = 9.633;
// const float l3 = 14.7;
const double l3 = 14.7;

// const float xOffset = -7.2;
const double xOffset = 0.0;
const double wyosiowanie12 = 2.4;

const double yOffset = 0;
const double zOffset = 5.3;

struct ArmState {
    double q1, q2, q3;
};

struct IkSolutions {
    double q1, q2a, q2b, q3a, q3b;
};

bool KILLED = false;

void handleLiveFkMode();

double toAngle(int ticks){
    double angle = (double) ticks / ticksPerRot * 360.0;
    angle = atan2(sin(angle / 180.0 * PI), cos(angle / 180.0 * PI));
    angle = angle / PI * 180;
    return angle;
}


double eeHeadingWhenQ2IsZero(double q3){
    return atan2(l3 * sin(q3), l2 + l3 * cos(q3));
}

IkSolutions solveIk(float x, float y, float z) {
    IkSolutions solutions;
    solutions.q1 = atan2(y, x);
    
    // x w ukladzie przegubu 2
    double x2 = sqrt(x * x + y * y) - wyosiowanie12; 
    double z2 = z - l1;

    // Serial.printf("x2 : %f \n", x2);
    // Serial.printf("z2 : %f \n", z2);


    // promien koncowki w ukladzie pzregubu 2
    double d2 = sqrt(x2 * x2 + z2 * z2);

    // Serial.printf("d2 : %f \n", d2);

    // kat pod jakim jest koncowka w ukladzie przegubu 2   
    double theta = atan2(z2, x2);

    // Serial.printf("theta: %f \n", theta);

    // rownanie nr 7  - https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html
    double c3 = (d2 * d2 - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);

    // Serial.printf("c3 : %f \n", c3);
    
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

    // Serial.printf("q2a : %f \n", solutions.q2a);

    // Serial.printf("q2b : %f \n", solutions.q2b);
    
    // Serial.printf("q3a : %f \n", solutions.q3a);

    // Serial.printf("q3b : %f \n", solutions.q3b);


    Serial.printf("ik solution found \n");


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

int toTicks(double relAngle){
    return relAngle / 360 * ticksPerRot;
}



class MotorState {
private:
    hMotor* mot1;
    hMotor* mot2;
    hMotor* mot3;
    int mot1Offset, mot2Offset, mot3Offset;
    static double getMotor(hMotor* mot, int motOffset, double trans=1.0){
        // Serial.printf("\n encoder ctn: %f", (double)mot->getEncoderCnt());

        // Serial.printf("\n motor offset: %d",motOffset);

        // Serial.printf("\n absolute angle %f", toAngle((double) (mot->getEncoderCnt()) - motOffset));
        return toAngle(((double) (mot->getEncoderCnt()) - motOffset) / trans);
    }

    static void setMotor(hMotor* mot, int motOffset, double angle, double trans=1.0){
        double currAngle = getMotor(mot, motOffset, trans);
        // Serial.printf("\n target motor angle: %f \r\n", angle);
        
        // Serial.printf("Current motor angle: %f \r\n", currAngle);

        double relAngle = pickSmallerDiff(currAngle, angle);
        // Serial.printf("relative motor travel angle: %f \r\n", relAngle);
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
        setMotor(mot1, mot1Offset, angle, h1Transmission);
    }

    void setMot2(double angle){
        setMotor(mot2, mot2Offset, angle, h2Transmission);
    }

    void setMot3(double angle){
        setMotor(mot3, mot3Offset, angle, h3Transmission);
    }

    void moveForward(double q1Deg, double q2Deg, double q3Deg, double delayBetweenAxesMs=1000){
        this->setMot3(q3Deg);
        sys.delay(delayBetweenAxesMs);
        this->setMot2(q2Deg);
        sys.delay(delayBetweenAxesMs);
        this->setMot1(q1Deg);
    }

    void moveCartesian(double x, double y, double z, double delayBetweenAxesMs=1000){
        IkSolutions solutions;
        ArmState armState;
        x = x - xOffset;
        y = y - yOffset;
        z = z - zOffset;
        Serial.printf("liczymy IK \n");
        try {
            solutions = solveIk(x, y, z);
            armState = pickSolution(solutions);

            double q1 = rad2deg(armState.q1);
            double q2 = rad2deg(armState.q2);
            double q3 = rad2deg(armState.q3);

            Serial.printf("obliczone q1: %f \n", q1);
            Serial.printf("obliczone q2: %f \n", q2);
            Serial.printf("obliczone q3: %f \n", q3);
            
            this->moveForward(q1, q2, q3, delayBetweenAxesMs);
        }

        catch (std::range_error) {
            Serial.printf("\n poza zasiegiem");
            Serial.printf("\n poza zasiegiem");
            Serial.printf("\n poza zasiegiem");
            Serial.printf("\n poza zasiegiem");
            Serial.printf("\n poza zasiegiem");
        }
    }


    Vector3 solveForward(){
        double q1 = deg2rad(this->getMot1());
        double q2 = deg2rad(this->getMot2());
        double q3 = deg2rad(this->getMot3());
        
        Vector3 point;
        // x, y, z

        // x, y - , q2 & q3 set radius, q1 sets angle
        double xy_radius = l2 * cos(q2) + l3 * cos(q3+q2) + wyosiowanie12;
        
        point.x = xy_radius * cos(q1) + xOffset;
        point.y = xy_radius * sin(q1) + yOffset;
        point.z = l1 + l2* sin(q2) + l3 * sin(q2+q3) + zOffset;
        
        return point;
}
};



void handleSimpleCartesianMode(MotorState& motorState){
    Serial.printf("\nWpisz wartosci\n");
    double x = readSerialDouble();
    double y = readSerialDouble();
    double z = readSerialDouble();

    
    Serial.printf("\n Wpisane x: %f \n", x);
    Serial.printf("\n Wpisane y: %f \n", y);
    Serial.printf("\n Wpisane z: %f \n", z);
    sys.delay(1000);
    motorState.moveCartesian(x, y, z);
    
    sys.delay(3000);

    Serial.printf("\n obecne q1 %f \n", motorState.getMot1());
    Serial.printf("\n obecne q2 %f \n", motorState.getMot2());
    Serial.printf("\n obecne q3 %f \n", motorState.getMot3());
    
    Vector3 position = motorState.solveForward();

    Serial.printf("\n Obecne x: %f \n", position.x);
    Serial.printf("\n Obecne y: %f \n", position.y);
    Serial.printf("\n Obecne z: %f \n", position.z);
    

    Serial.printf("\n potwierdz ze dojechales \n");
    Serial.getch();
}


void handleSimpleFkMode(MotorState& motorState){
    Serial.printf("\nWpisz wartosci\n");
    double q1 = readSerialDouble();
    double q2 = readSerialDouble();
    double q3 = readSerialDouble();

    // motorState.setMot1(180.0);

    // Serial.getch();
    // motorState.setMot1(0.0);
    
    Serial.printf("\n Wpisane q1: %f \n", q1);
    Serial.printf("\n Wpisane q2: %f \n", q2);
    Serial.printf("\n Wpisane q3: %f \n", q3);
    
    sys.delay(1500);

    motorState.moveForward(q1, q2, q3);

    sys.delay(3000);



    Serial.printf("\n obecne q1 %f \n", motorState.getMot1());
    Serial.printf("\n obecne q2 %f \n", motorState.getMot2());
    Serial.printf("\n obecne q3 %f \n", motorState.getMot3());
    


    Vector3 position = motorState.solveForward();
    Serial.printf("\n Obecne x: %f \n", position.x);
    Serial.printf("\n Obecne y: %f \n", position.y);
    Serial.printf("\n Obecne z: %f \n", position.z);
    

    Serial.printf("\n potwierdz ze dojechales \n");
    Serial.getch();
}

void handleLiveCartesianMode(MotorState& motorState){
    double incr = 0.5;

    Serial.printf("\n\n Sterowanie\n\n");
    Serial.printf("w -> os x kierunek dodatni\n");
    Serial.printf("s -> os x kierunek ujemny\n");
    Serial.printf("a -> os y kierunek dodatni\n");
    Serial.printf("d -> os y kierunek ujemny\n");
    Serial.printf("i -> os z kierunek dodatni\n");
    Serial.printf("k -> os z kierunek ujemny\n");
    Serial.printf("q -> wyjscie\n\n");

    char c = Serial.getch();
    
    while (c != 'q'){
        Vector3 eePose = motorState.solveForward();
        switch (c) {
        case 'w':
            {
                eePose.x = eePose.x + incr;
                break;    
            }
            
        case 's':
            {
                eePose.x = eePose.x - incr;
                break;    
            }

        case 'a':
            {
                eePose.y = eePose.y + incr;
                break;    
            }
        
        case 'd':
            {
                eePose.y = eePose.y - incr;
                break;    
            }

        case 'i':
            {
                eePose.z = eePose.z + incr;
                break;    
            }

        case 'k':
            {
                eePose.z = eePose.z - incr;
                break;    
            }

        }
            

        motorState.moveCartesian(eePose.x, eePose.y, eePose.z, 10);
        c = Serial.getch();
        }
    } 

void handleLiveFkMode(){
     int incr1 = 10;
     int incr2 = 5;
     int incr3 = 2;

    Serial.printf("\n\n Sterowanie\n\n");
    Serial.printf("w -> czlon 2 kierunek dodatni\n");
    Serial.printf("s -> czlon 2 kierunek ujemny\n");
    Serial.printf("a -> czlon 1 kierunek dodatni\n");
    Serial.printf("d -> czlon 1 kierunek ujemny\n");
    Serial.printf("i -> czlon 3 kierunek dodatni\n");
    Serial.printf("k -> czlon 3 kierunek ujemny\n");
    Serial.printf("q -> wyjscie\n\n");

    char c = Serial.getch();
    
    while (c != 'q'){
        switch (c) {
        case 'w':
            {
                hMot2.rotRel(-incr2, 400, false);
                break;    
            }
            
        case 's':
            {
                hMot2.rotRel(incr2, 400, false);
                break;    
            }

        case 'a':
            {
                hMot1.rotRel(-incr1, 400, false);
                break;    
            }
        
        case 'd':
            {
                hMot1.rotRel(incr1, 400, false);
                break;    
            }

        case 'i':
            {
                hMot3.rotRel(incr3, 400, false);
                break;    
            }

        case 'k':
            {
                hMot3.rotRel(-incr3, 400, false);
                break;    
            }

        }
        c = Serial.getch();
    }
}

void handleCircleTrajectory(MotorState& motorState){
    Serial.printf("\n\n Wpisz x: \n");
    double x = readSerialDouble();

    Serial.printf("\n Wpisz y: \n");
    double y = readSerialDouble();


    Serial.printf("\n Wpisz z: \n");
    double z = readSerialDouble();

    Serial.printf("\n Wpisz promien: \n");
    double radius = readSerialDouble();

    
    Serial.printf("\n Wpisane x: %f \n", x);
    Serial.printf("\n Wpisane y: %f \n", y);
    Serial.printf("\n Wpisane z: %f \n", z);
    Serial.printf("\n Wpisany promien: %f \n\n", radius);

    std::vector<Vector3> points;
    // circleFactory(x, y, z, radius, points);

    // for (auto point: points){
    //     motorState.moveCartesian(point.x, point.y, point.z, 100);
    // }
}

void printMenu(){
    Serial.printf("\n 1 - simple ik \n 2 - simple fk \n");
    Serial.printf("3 - live ik \n");
    Serial.printf("4 - live fk \n");
    Serial.printf("5 - narysuj kolko \n");
    Serial.printf("\n");

}

void killSwitchLoop(){
    hLegoSensor_simple ls(hSens5);
    Lego_Touch sensor(ls);
    bool prevIterPressed = false;

    while (true){
        bool pressed = sensor.isPressed();
        int stateDelta = pressed - prevIterPressed;
        
        if (stateDelta > 0){
            KILLED = !KILLED;
            Serial.printf("\n\n killed switch state changed. Current kill switch state: %d \n\n", KILLED);
        }
        
        if (KILLED){

            hMot1.rotAbs(hMot1.getEncoderCnt(), 200, false);
            hMot2.rotAbs(hMot2.getEncoderCnt(), 200, false);
            hMot3.rotAbs(hMot3.getEncoderCnt(), 200, false);
            // hMot1.setPowerLimit(0);
            // hMot2.setPowerLimit(0);
            // hMot3.setPowerLimit(0);
        }


        prevIterPressed = pressed;
        sys.delay(10);        
    }
}

void hMain()
{   
    sys.taskCreate(killSwitchLoop); // this creates a task that will execute `encoder` concurrently


    sys.setLogDev(&Serial);

    hMot1.setEncoderPolarity(Polarity::Reversed);
    hMot2.setEncoderPolarity(Polarity::Reversed);
    hMot3.setEncoderPolarity(Polarity::Reversed);

    hMot1.rotRel(0, 200, false);
    hMot2.rotRel(0, 200, false);
    hMot3.rotRel(0, 200, false);
    Serial.printf("\n\n");
    Serial.printf("\n\nCalibrating motors");
    
    handleLiveFkMode();
    int hMot1Offset = hMot1.getEncoderCnt();
    int hMot2Offset = hMot2.getEncoderCnt();
    int hMot3Offset = hMot3.getEncoderCnt();

    Serial.printf("\n\n ---All motors calibrated-- \n\n");

    MotorState motorState(&hMot1, &hMot2, &hMot3, hMot1Offset, hMot2Offset, hMot3Offset);
    

    while (true){
        printMenu();
        char c = Serial.getch();
        switch (c) {
        case '1':
            {
                handleSimpleCartesianMode(motorState);
                break;
            }
        
        case '2': 
            {
                handleSimpleFkMode(motorState);
                break;
            }

        case '3':
            {
                handleLiveCartesianMode(motorState);
                break;
            }
        case '4':
            {
                handleLiveFkMode();
                break;
            }
        case '5':
            {
                handleCircleTrajectory(motorState);
                break;
            }




        }
    }
    
}
