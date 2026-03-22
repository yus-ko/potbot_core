#include <potbot_ros/scan_clustering.hpp>

#include <cmath>
#include <limits>

namespace potbot_lib
{

    ScanClustering::ScanClustering()
    {
    }

    void ScanClustering::setClusters(const sensor_msgs::msg::LaserScan & scan)
    {
        clusters_.clear();

        std::vector<ScanPoint> valid_points;

        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            double r = scan.ranges[i];
            if (r < min_scan_range_ || r > max_scan_range_)
            {
                continue;
            }

            double theta = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            ScanPoint sp;
            sp.index = static_cast<int>(i);
            sp.r = r;
            sp.theta = theta;
            sp.x = r * std::cos(theta);
            sp.y = r * std::sin(theta);

            valid_points.push_back(sp);
        }

        euclideanClustering(valid_points);
        segmentation();
    }

    void ScanClustering::getClusters(std::vector<Segment> & clusters_arg)
    {
        clusters_arg = clusters_;
    }

    void ScanClustering::euclideanClustering(const std::vector<ScanPoint> & points)
    {
        clusters_.clear();

        if (points.empty())
        {
            return;
        }

        Segment current_segment;
        current_segment.points.push_back(points[0]);

        for (size_t i = 1; i < points.size(); ++i)
        {
            const ScanPoint & prev = points[i - 1];
            const ScanPoint & curr = points[i];

            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist <= cluster_threshold_)
            {
                current_segment.points.push_back(curr);
            }
            else
            {
                clusters_.push_back(current_segment);
                current_segment = Segment();
                current_segment.points.push_back(curr);
            }
        }
        clusters_.push_back(current_segment);

        for (size_t i = 0; i < clusters_.size(); ++i)
        {
            clusters_[i].id = static_cast<int>(i);
        }
    }

    void ScanClustering::euclideanClustering()
    {
        std::vector<ScanPoint> all_points;
        for (const auto & seg : clusters_)
        {
            for (const auto & p : seg.points)
            {
                all_points.push_back(p);
            }
        }
        euclideanClustering(all_points);
    }

    void ScanClustering::segmentation()
    {
        for (auto & seg : clusters_)
        {
            if (seg.points.empty())
            {
                continue;
            }

            double sum_x = 0.0;
            double sum_y = 0.0;
            for (const auto & p : seg.points)
            {
                sum_x += p.x;
                sum_y += p.y;
            }
            seg.x = sum_x / static_cast<double>(seg.points.size());
            seg.y = sum_y / static_cast<double>(seg.points.size());

            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();

            for (const auto & p : seg.points)
            {
                if (p.x < min_x) min_x = p.x;
                if (p.x > max_x) max_x = p.x;
                if (p.y < min_y) min_y = p.y;
                if (p.y > max_y) max_y = p.y;
            }

            seg.width = max_x - min_x;
            seg.height = max_y - min_y;

            double max_dist_sq = 0.0;
            for (size_t i = 0; i < seg.points.size(); ++i)
            {
                for (size_t j = i + 1; j < seg.points.size(); ++j)
                {
                    double dx = seg.points[j].x - seg.points[i].x;
                    double dy = seg.points[j].y - seg.points[i].y;
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq > max_dist_sq)
                    {
                        max_dist_sq = dist_sq;
                    }
                }
            }
            seg.radius = std::sqrt(max_dist_sq) / 2.0;
        }
    }

    void ScanClustering::toMarkerarray(visualization_msgs::msg::MarkerArray & ma)
    {
        ma.markers.clear();

        for (const auto & seg : clusters_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser";
            marker.id = seg.id;
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            for (const auto & p : seg.points)
            {
                geometry_msgs::msg::Point gp;
                gp.x = p.x;
                gp.y = p.y;
                gp.z = 0.0;
                marker.points.push_back(gp);
            }

            ma.markers.push_back(marker);
        }
    }

} // namespace potbot_lib
