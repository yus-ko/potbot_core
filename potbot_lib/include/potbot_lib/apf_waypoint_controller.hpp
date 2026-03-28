#pragma once
#include <vector>
#include <cmath>
#include "potbot_lib/diff_drive_agent.hpp"
#include "potbot_lib/utility.hpp"
#include "potbot_lib/artificial_potential_field.hpp"

namespace potbot_lib {
namespace controller {

class ApfWaypointController : public DiffDriveAgent {
public:
    ApfWaypointController();
    ~ApfWaypointController() = default;

    void setGlobalPath(const std::vector<Pose>& path);
    void setObstacles(const std::vector<Point>& obstacles);
    void setParams(double k_att, double k_rep, double d_th,
                   double k_v, double k_omega,
                   double v_max, double omega_max,
                   double waypoint_tolerance, double goal_tolerance);
    void computeCommand();
    bool reachedGoal() const;
    bool isPathEmpty() const;
    size_t getCurrentWaypointIndex() const { return waypoint_index_; }
    Point getCurrentWaypoint() const;

private:
    void computeForce(double wx, double wy, double& fx, double& fy);
    void updateWaypoint();
    double normalizeAngle(double angle) const;

    double k_att_              = 1.0;
    double k_rep_              = 1.0;
    double d_th_               = 0.5;
    double k_v_                = 0.5;
    double k_omega_            = 2.0;
    double v_max_              = 0.3;
    double omega_max_          = 1.5;
    double waypoint_tolerance_ = 0.2;
    double goal_tolerance_     = 0.05;

    std::vector<Pose>  global_path_;
    size_t             waypoint_index_ = 0;
    std::vector<Point> obstacles_;
    ArtificialPotentialField apf_;
};

} // namespace controller
} // namespace potbot_lib
