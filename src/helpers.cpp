#include <helpers.h>



double readSerialDouble()
{
    char buffer[20];
    int cnt = 0;
    char c;
    do {
        c = Serial.getch();
        Serial.printf("%c", c);
        buffer[cnt] = c;
        cnt++;
    } while (c!= '\n');

    Serial.printf("\n\r");
    return atof(buffer);
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


double rad2deg(double rad){
    return rad / 3.14 * 180; 
}

double deg2rad(double deg){
    return deg * 3.14 / 180;
}

bool almostEqual(double x, double y, double epsilon){
   return fabs(x - y) < epsilon;
}
