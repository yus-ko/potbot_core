#include <potbot_lib/diff_drive_agent.h>

namespace potbot_lib
{

    void DiffDriveAgent::update()
    {
        yaw                             += omega*deltatime;
        x                               += v*deltatime*cos(yaw);
        y                               += v*deltatime*sin(yaw);
    }

    double DiffDriveAgent::getDistance(const Point& p)
    {
        return hypot(x-p.x,y-p.y);
    }

    double DiffDriveAgent::getDistance(const Pose& p)
    {
        return getDistance(p.position);
    }

    double DiffDriveAgent::getAngle(const Point& p)
    {
        return atan2(p.y - y, p.x - x);
    }

    double DiffDriveAgent::getAngle(const Pose& p)
    {
        return getAngle(p.position);
    }
}