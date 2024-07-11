#ifndef H_DIFFDRIVEAGENT_
#define H_DIFFDRIVEAGENT_

#include <potbot_lib/utility.h>

namespace potbot_lib
{

    class DiffDriveAgent
    {
        public:
            double x                            = 0.0;  //x軸位置 [m]
            double y                            = 0.0;  //y軸位置 [m]
            double yaw                          = 0.0;  //z軸回転 [rad]
            double v                            = 0.0;  //並進速度 [m/s]
            double omega                        = 0.0;  //回転角速度 [rad/s]

            double deltatime                    = 0.02; //単位時間 [s]

            DiffDriveAgent( const double x              = 0.0,
                            const double y              = 0.0,
                            const double yaw            = 0.0,
                            const double v              = 0.0,
                            const double omega          = 0.0,
                            const double deltatime      = 0.02):
                            x(x),
                            y(y),
                            yaw(yaw),
                            v(v),
                            omega(omega),
                            deltatime(deltatime){};

            void update();

            double getDistance(const Point& p);
            double getDistance(const Pose& p);
            double getAngle(const Point& p);
            double getAngle(const Pose& p);
    };
}

#endif	// H_DIFFDRIVECONTROLLER_