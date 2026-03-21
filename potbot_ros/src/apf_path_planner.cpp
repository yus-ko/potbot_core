#include <potbot_ros/apf_path_planner.hpp>

namespace potbot_lib{

    namespace path_planner{

        void getPathMsgFromCsv(nav_msgs::msg::Path& path_msg,const std::string& csv_fullpath)
        {
            nav_msgs::msg::Path init;
            path_msg = init;

            std::string str_buf;
            std::string str_conma_buf;
            std::ifstream ifs_csv_file(csv_fullpath);

            double line_buf[3];
            while (getline(ifs_csv_file, str_buf)) 
            {    
                
                std::istringstream i_stream(str_buf);// 「,」区切りごとにデータを読み込むためにistringstream型にする
                
                int i = 0;
                while (getline(i_stream, str_conma_buf, ',')) // 「,」区切りごとにデータを読み込む
                {
                    line_buf[i++] = std::stod(str_conma_buf);
                }
                geometry_msgs::msg::PoseStamped pose;
                pose.pose = potbot_lib::utility::get_pose(line_buf[0], line_buf[1], 0, 0, 0, line_buf[2]);
                path_msg.poses.push_back(pose);
            }
            
        }

        // void get_search_index_APF(Field& field, std::vector<size_t>, )
        // APFPathPlannerROS::APFPathPlannerROS(ArtificialPotentialField *apf) : 
        // APFPathPlanner::APFPathPlanner(apf){}

        APFPathPlannerROS::APFPathPlannerROS(std::shared_ptr<ArtificialPotentialFieldROS> apf_ros) : APFPathPlanner::APFPathPlanner(apf_ros->getApf())
        {
            node_ = apf_ros->getNode();
            frame_id_global_ = apf_ros->getFrameIdGlobal();
            // private_nh.getParam("frame_id_global",           frame_id_global_);
            // pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("path", 1);
            // pub_raw_path_ = node_->create_publisher<nav_msgs::msg::Path>("debug/raw_path", 1);

        }

        // void APFPathPlannerROS::reconfigureCB(const potbot_lib::APFPathPlannerConfig& param, uint32_t level)
        // {
        //     max_path_length_ = param.max_path_length;
        //     path_search_range_ = param.path_search_range;
        //     path_weight_potential_ = param.weight_potential_field;
        //     path_weight_pose_ = param.weight_angle;
        // }

        void APFPathPlannerROS::getLoopEdges(visualization_msgs::msg::MarkerArray& msg)
        {
            msg.markers.clear();
            int id = 0;
            for (const auto& loops:loop_edges_)
            {
                visualization_msgs::msg::Marker marker;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.id = id++;
                marker.color = potbot_lib::color::get_msg(marker.id);
                marker.scale.x = 0.05;
                marker.lifetime.sec = 0;
                marker.lifetime.nanosec = 0.5*1e-9;
                marker.pose = utility::get_pose();
                marker.ns = std::to_string(marker.id);
                for (const auto& point:loops)
                {
                    geometry_msgs::msg::Point point_msg = utility::get_point(point.x, point.y);
                    marker.points.push_back(point_msg);
                }
                msg.markers.push_back(marker);
            }
        }

        void APFPathPlannerROS::getPath(std::vector<geometry_msgs::msg::PoseStamped> &msg)
        {
            msg.clear();
            for (const auto& point : path_)
            {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.pose = potbot_lib::utility::get_pose(point);
                msg.push_back(pose_msg);
            }
        }

        void APFPathPlannerROS::getPath(nav_msgs::msg::Path &msg)
        {
            getPath(msg.poses);
        }

        void APFPathPlannerROS::publishPath()
        {
            nav_msgs::msg::Path path_msg;
            getPath(path_msg);
            path_msg.header.frame_id = frame_id_global_;
            path_msg.header.stamp = node_->get_clock()->now();
            pub_path_->publish(path_msg);
        }

        void APFPathPlannerROS::publishRawPath()
        {
            nav_msgs::msg::Path path_msg;
            getPath(path_msg);
            path_msg.header.frame_id = frame_id_global_;
            path_msg.header.stamp = node_->get_clock()->now();
            pub_raw_path_->publish(path_msg);
        }
    }
}