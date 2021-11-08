
#ifndef HELPERS_DEFINITION
#define HELPERS_DEFINITION

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

double readSerialDouble();

double pickSmallerDiff(double currAngle, double targetAngle);

double rad2deg(double rad);

double deg2rad(double deg);

bool almostEqual(double x, double y, double epsilon = 0.01f);

#endif
