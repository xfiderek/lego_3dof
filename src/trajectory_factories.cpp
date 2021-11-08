#include <trajectory_factories.h>

void circleFactory(double xCenter, double yCenter, double zCenter, double radius, std::vector<Vector3>& points, int samples){
    for (int i = 0; i < samples ; i++){
        double angle = (1.0*i) / samples;
        double x = xCenter + cos(angle) * radius;
        double y = yCenter + sin(angle) * radius;
        
        Vector3 point;
        point.x = x;
        point.y = y;
        point.z = zCenter;
        points.push_back(point);
    }

}