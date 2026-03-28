#include "potbot_lib/apf_waypoint_controller.hpp"

#include <cmath>
#include <algorithm>

namespace potbot_lib {
namespace controller {

ApfWaypointController::ApfWaypointController()
{
    apf_.setParams(k_att_, k_rep_, d_th_);
}

void ApfWaypointController::setGlobalPath(const std::vector<Pose>& path)
{
    global_path_   = path;
    waypoint_index_ = 0;
}

void ApfWaypointController::setObstacles(const std::vector<Point>& obstacles)
{
    obstacles_ = obstacles;
}

void ApfWaypointController::setParams(double k_att, double k_rep, double d_th,
                                       double k_v, double k_omega,
                                       double v_max, double omega_max,
                                       double waypoint_tolerance, double goal_tolerance,
                                       double lookahead_distance)
{
    k_att_              = k_att;
    k_rep_              = k_rep;
    d_th_               = d_th;
    k_v_                = k_v;
    k_omega_            = k_omega;
    v_max_              = v_max;
    omega_max_          = omega_max;
    waypoint_tolerance_ = waypoint_tolerance;
    goal_tolerance_     = goal_tolerance;
    lookahead_distance_ = lookahead_distance;

    apf_.setParams(k_att_, k_rep_, d_th_);
}

bool ApfWaypointController::isPathEmpty() const
{
    return global_path_.empty();
}

bool ApfWaypointController::reachedGoal() const
{
    if (isPathEmpty()) {
        return false;
    }
    const Pose& goal = global_path_.back();
    double dx = goal.position.x - x;
    double dy = goal.position.y - y;
    double dist = std::sqrt(dx * dx + dy * dy);
    return dist < goal_tolerance_;
}

Point ApfWaypointController::getCurrentWaypoint() const
{
    if (isPathEmpty()) {
        return Point{x, y, 0.0};
    }
    const Pose& wp = global_path_[waypoint_index_];
    return Point{wp.position.x, wp.position.y, 0.0};
}

void ApfWaypointController::updateWaypoint()
{
    if (isPathEmpty()) {
        return;
    }

    // 近傍のwaypointをまとめてスキップ（1ステップずつだと引力が弱まり停止する）
    while (waypoint_index_ < global_path_.size() - 1) {
        const Pose& wp = global_path_[waypoint_index_];
        double dx   = wp.position.x - x;
        double dy   = wp.position.y - y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < waypoint_tolerance_) {
            waypoint_index_++;
        } else {
            break;
        }
    }
}

void ApfWaypointController::computeForce(double wx, double wy, double& fx, double& fy)
{
    apf_.clearObstacles();
    for (const auto& obs : obstacles_) {
        apf_.setObstacle(obs.x, obs.y);
    }
    apf_.getForce(x, y, wx, wy, fx, fy);
}

double ApfWaypointController::normalizeAngle(double angle) const
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void ApfWaypointController::computeCommand()
{
    if (isPathEmpty() || reachedGoal()) {
        v     = 0.0;
        omega = 0.0;
        return;
    }

    updateWaypoint();

    // ルックアヘッド: 現在waypointが近すぎる場合は引力が弱くなるため、
    // ルックアヘッド距離以上離れた前方のwaypointを引力ターゲットにする
    size_t target_idx = waypoint_index_;
    for (size_t i = waypoint_index_; i < global_path_.size(); i++) {
        double dx = global_path_[i].position.x - x;
        double dy = global_path_[i].position.y - y;
        if (std::sqrt(dx * dx + dy * dy) >= lookahead_distance_) {
            target_idx = i;
            break;
        }
        target_idx = i;  // 全て近い場合は最遠のwaypointを使う
    }

    const Pose& wp = global_path_[target_idx];
    double fx = 0.0, fy = 0.0;
    computeForce(wp.position.x, wp.position.y, fx, fy);

    double desired_heading = std::atan2(fy, fx);
    double heading_error   = normalizeAngle(desired_heading - yaw);
    double force_magnitude = std::sqrt(fx * fx + fy * fy);

    double cmd_v = std::min(k_v_ * force_magnitude, v_max_);

    double cmd_omega = k_omega_ * heading_error;
    cmd_omega = std::max(-omega_max_, std::min(omega_max_, cmd_omega));

    v     = cmd_v;
    omega = cmd_omega;
}

} // namespace controller
} // namespace potbot_lib
