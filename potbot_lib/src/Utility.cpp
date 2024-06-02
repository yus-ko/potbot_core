#include <potbot_lib/Utility.h>

namespace potbot_lib{
    namespace color{
        std_msgs::ColorRGBA get_msg(const int color_id)
        {
            std_msgs::ColorRGBA color;
            if(color_id < 0)
            {
                static std::random_device rd;
                static std::mt19937 gen(rd());
                static std::uniform_real_distribution<> dis(0.0, 1.0);
                color.r = dis(gen);
                color.g = dis(gen);
                color.b = dis(gen);
                color.a = 1;
            }
            else
            {
                int num = color_id%8;
                if(num == potbot_lib::color::RED)
                {
                    color.r = 1; color.g = 0; color.b = 0; color.a = 1;
                }
                else if(num == potbot_lib::color::GREEN)
                {
                    color.r = 0; color.g = 1; color.b = 0; color.a = 1;
                }else if(num == potbot_lib::color::BLUE)
                {
                    color.r = 0; color.g = 0; color.b = 1; color.a = 1;
                }else if(num == potbot_lib::color::YELLOW)
                {
                    color.r = 1; color.g = 1; color.b = 0; color.a = 1;
                }else if(num == potbot_lib::color::LIGHT_BLUE)
                {
                    color.r = 0; color.g = 1; color.b = 1; color.a = 1;
                }else if(num == potbot_lib::color::PURPLE)
                {
                    color.r = 1; color.g = 0; color.b = 1; color.a = 1;
                }else if(num == potbot_lib::color::BLACK)
                {
                    color.r = 0; color.g = 0; color.b = 0; color.a = 1;
                }else if(num == potbot_lib::color::WHITE)
                {
                    color.r = 1; color.g = 1; color.b = 1; color.a = 1;
                }
            }
            
            return color;
        }

        std_msgs::ColorRGBA get_msg(const std::string color_name)
        {
            if (color_name == "r" || color_name == "red")
            {
                return get_msg(potbot_lib::color::RED);
            }
            else if(color_name == "g" || color_name == "green")
            {
                return get_msg(potbot_lib::color::GREEN);
            }
            else if(color_name == "b" || color_name == "blue")
            {
                return get_msg(potbot_lib::color::BLUE);
            }
            else if(color_name == "y" || color_name == "yellow")
            {
                return get_msg(potbot_lib::color::YELLOW);
            }
            else if(color_name == "lb" || color_name == "light_blue")
            {
                return get_msg(potbot_lib::color::LIGHT_BLUE);
            }
            else if(color_name == "p" || color_name == "purple")
            {
                return get_msg(potbot_lib::color::PURPLE);
            }
            else if(color_name == "k" || color_name == "black")
            {
                return get_msg(potbot_lib::color::BLACK);
            }
            else if(color_name == "w" || color_name == "white")
            {
                return get_msg(potbot_lib::color::WHITE);
            }
            else if(color_name == "random")
            {
                return get_msg(-1);
            }
            else
            {
                return get_msg(potbot_lib::color::RED);
            }
        }
    }

    namespace utility{

        void get_RPY(const geometry_msgs::Quaternion& orientation, double &roll, double &pitch, double &yaw)
        {
            tf2::Quaternion quat;
            tf2::convert(orientation, quat);
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        }

        geometry_msgs::Quaternion get_Quat(const double roll, const double pitch, const double yaw)
        {
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            geometry_msgs::Quaternion orientation;
            tf2::convert(quat, orientation);
            return orientation;
        }

        geometry_msgs::Point get_Point(const double x, const double y, const double z)
        {
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            return point;
        }

        geometry_msgs::Pose get_Pose(const double x, const double y, const double z, const double roll, const double pitch, const double yaw)
        {
            geometry_msgs::Pose pose;
            pose.position = get_Point(x,y,z);
            pose.orientation = get_Quat(roll,pitch,yaw);
            return pose;
        }

        geometry_msgs::Pose get_Pose(const geometry_msgs::Point& p, const double roll, const double pitch, const double yaw)
        {
            return get_Pose(p.x, p.y, p.z, roll,pitch,yaw);
        }

        double get_Yaw(const geometry_msgs::Quaternion& orientation)
        {
            double roll, pitch, yaw;
            get_RPY(orientation, roll, pitch, yaw);
            return yaw;
        }

        double get_Distance(const geometry_msgs::Point& position1, const geometry_msgs::Point& position2)
        {
            return sqrt(pow(position2.x - position1.x,2) + pow(position2.y - position1.y,2) + pow(position2.z - position1.z,2));
        }

        double get_Distance(const geometry_msgs::Pose& position1, const geometry_msgs::Pose& position2)
        {
            return get_Distance(position1.position, position2.position);
        }

        double get_Distance(const geometry_msgs::PoseStamped& position1, const geometry_msgs::PoseStamped& position2)
        {
            return get_Distance(position1.pose.position, position2.pose.position);
        }

        double get_Distance(const nav_msgs::Odometry& position1, const nav_msgs::Odometry& position2)
        {
            return get_Distance(position1.pose.pose.position, position2.pose.pose.position);
        }

        void print_Pose(const geometry_msgs::Pose& pose)
        {
            double r,p,y;
            get_RPY(pose.orientation,r,p,y);
            ROS_INFO("(x,y,z) = (%f, %f, %f) (r,p,y) = (%f, %f, %f)", 
                        pose.position.x, pose.position.y, pose.position.z,
                        r/M_PI*180, p/M_PI*180, y/M_PI*180);
        }

        void print_Pose(const geometry_msgs::PoseStamped& pose)
        {
            print_Pose(pose.pose);
        }

        void print_Pose(const nav_msgs::Odometry& pose)
        {
            print_Pose(pose.pose.pose);
        }

        geometry_msgs::PoseStamped get_tf(const tf2_ros::Buffer &buffer, const geometry_msgs::PoseStamped& pose_in, const std::string target_frame_id)
        {
            geometry_msgs::PoseStamped pose_out;
            geometry_msgs::TransformStamped transformStamped;
            
            try
            {
                transformStamped = buffer.lookupTransform(target_frame_id, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(1.0));
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN_STREAM("get_tf TF2 exception: " << ex.what());
            }
            
            tf2::doTransform(pose_in, pose_out, transformStamped);
            return pose_out;
        }

        geometry_msgs::PointStamped get_tf(const tf2_ros::Buffer &buffer, const geometry_msgs::PointStamped& point_in, const std::string target_frame_id)
        {
            geometry_msgs::PoseStamped pose_stamped_in;
            pose_stamped_in.header = point_in.header;
            pose_stamped_in.pose = get_Pose(point_in.point,0,0,0);
            geometry_msgs::PoseStamped pose_out = get_tf(buffer, pose_stamped_in, target_frame_id);
            geometry_msgs::PointStamped point_out;
            point_out.header = pose_out.header;
            point_out.point = pose_out.pose.position;

            return point_out;
        }

        geometry_msgs::PoseStamped get_tf(const tf2_ros::Buffer &buffer, const nav_msgs::Odometry& pose_in, const std::string target_frame_id)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = pose_in.header;
            ps.pose = pose_in.pose.pose;
            return get_tf(buffer, ps, target_frame_id);
        }

        void get_tf(const tf2_ros::Buffer &buffer,const potbot_msgs::Obstacle& obstacle_in,const std::string target_frame_id, potbot_msgs::Obstacle& obstacle_out)
        {
            obstacle_out = obstacle_in;

            geometry_msgs::PoseStamped pose_in;
            pose_in.header = obstacle_in.header;
            pose_in.pose = obstacle_in.pose;

            geometry_msgs::PoseStamped pose_out = get_tf(buffer, pose_in, target_frame_id);
            obstacle_out.header = pose_out.header;
            obstacle_out.pose = pose_out.pose;

            for(size_t i = 0; i < obstacle_in.points.size(); i++)
            {
                geometry_msgs::PointStamped point_in;
                point_in.header = obstacle_in.header;
                point_in.point = obstacle_in.points[i];
                geometry_msgs::PointStamped point_out = get_tf(buffer, point_in, target_frame_id);
                obstacle_out.points[i] = point_out.point;
            }
        }

        void get_tf(const tf2_ros::Buffer &buffer, const potbot_msgs::ObstacleArray& obscales_in,const std::string target_frame_id, potbot_msgs::ObstacleArray& obscales_out)
        {
            obscales_out = obscales_in;
            obscales_out.header.frame_id = target_frame_id;
            for (size_t i = 0; i < obscales_in.data.size(); i++)
            {
                get_tf(buffer, obscales_in.data[i], target_frame_id, obscales_out.data[i]);
            }
        }

        void get_tf(const tf2_ros::Buffer &buffer, const nav_msgs::Path& path_in, const std::string target_frame_id, nav_msgs::Path& path_out)
        {
            path_out.header = path_in.header;
            path_out.header.frame_id = target_frame_id;
            path_out.poses.clear();

            for (const auto p_raw: path_in.poses)
            {
                geometry_msgs::PoseStamped pose_in;
                pose_in.header = path_in.header;
                pose_in.pose = p_raw.pose;

                geometry_msgs::PoseStamped pose_out = get_tf(buffer, pose_in, target_frame_id);
                path_out.poses.push_back(pose_out);
            }
            
        }

        geometry_msgs::PoseStamped get_Pose_from_tf(const tf2_ros::Buffer &buffer, const std::string source_frame_id, const std::string target_frame_id)
        {
            geometry_msgs::TransformStamped transformStamped;
            try {
                transformStamped = buffer.lookupTransform(source_frame_id, target_frame_id, ros::Time(0));
            } catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
            }
            geometry_msgs::PoseStamped pose;
            pose.header = transformStamped.header;
            pose.pose.position.x = transformStamped.transform.translation.x;
            pose.pose.position.y = transformStamped.transform.translation.y;
            pose.pose.position.z = transformStamped.transform.translation.z;
            pose.pose.orientation.x = transformStamped.transform.rotation.x;
            pose.pose.orientation.y = transformStamped.transform.rotation.y;
            pose.pose.orientation.z = transformStamped.transform.rotation.z;
            pose.pose.orientation.w = transformStamped.transform.rotation.w;
            return pose;
        }

        int get_WorldCoordinate(std::string target_frame, ros::Time time, geometry_msgs::PoseStamped &Wcood, tf2_ros::Buffer &buffer)
        {
            tf2_ros::TransformListener tf_listener(buffer);

            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = buffer.lookupTransform("map", target_frame, time);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN_STREAM("get_WorldCoordinate TF2 exception: " << ex.what());
                return FAIL;
            }
            // ROS_INFO("x: %f, y: %f, z: %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            Wcood.header = transformStamped.header;
            Wcood.pose.position.x = transformStamped.transform.translation.x;
            Wcood.pose.position.y = transformStamped.transform.translation.y;
            Wcood.pose.position.z = transformStamped.transform.translation.z;
            Wcood.pose.orientation.x = transformStamped.transform.rotation.x;
            Wcood.pose.orientation.y = transformStamped.transform.rotation.y;
            Wcood.pose.orientation.z = transformStamped.transform.rotation.z;
            Wcood.pose.orientation.w = transformStamped.transform.rotation.w;
            return SUCCESS;
        }

        geometry_msgs::Point get_MapCoordinate(int index, nav_msgs::MapMetaData info)
        {
            geometry_msgs::Point p;
            p.x = (index % info.width) * info.resolution + info.origin.position.x;
            p.y = (index / info.width) * info.resolution + info.origin.position.y;
            return p;
        }

        int get_MapIndex(const double x, const double y, const nav_msgs::MapMetaData& info)
        {

            double xmin = info.origin.position.x;
            double xmax = info.origin.position.x + info.width * info.resolution;

            double ymin = info.origin.position.y;
            double ymax = info.origin.position.y + info.height * info.resolution;

            if (x < xmin || x > xmax || y < ymin || y > ymax)
            {
                return 0;
            }

            double img_x = x - info.origin.position.x;
            double img_y = y - info.origin.position.y;

            int index = int(img_y / info.resolution) * info.width + int(img_x / info.resolution);

            if (index < 0)
            {
                //ROS_INFO("%f, %f, %f, %f",x,y,info.origin.position.x,info.origin.position.y);
                index = 0;
            }

            return index;
            
        }

        int get_PathIndex(const nav_msgs::Path& path, const geometry_msgs::Point& position)
        {
            double min_distance = std::numeric_limits<double>::infinity();
            int path_index = 0;
            for (int i = 0; i < path.poses.size(); i++)
            {
                double distance = get_Distance(path.poses[i].pose.position, position);
                if(distance < min_distance)
                {
                    min_distance = distance;
                    path_index = i;
                }
            }
            return path_index;
        }

        int get_PathIndex(const nav_msgs::Path& path, const geometry_msgs::Pose& position)
        {
            return get_PathIndex(path, position.position);
        }

        int get_PathIndex(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& position)
        {
            return get_PathIndex(path, position.pose.position);
        }

        int get_PathIndex(const nav_msgs::Path& path, const nav_msgs::Odometry& position)
        {
            return get_PathIndex(path, position.pose.pose.position);
        }

        double get_PathLength(nav_msgs::Path path)
        {
            double total_length = 0;
            for (int i = 1; i < path.poses.size(); i++)
            {
                total_length += get_Distance(path.poses[i].pose.position, path.poses[i-1].pose.position);
            }
            return total_length;
        }

        void Timer::start(const std::string timer_name, const double time)
        {
            if (time < 0)
            {
                times_[timer_name].begin_time = ros::Time::now().toSec();
            }
            else
            {
                times_[timer_name].begin_time = time;
            }
            times_[timer_name].running = true;
        }

        void Timer::start(const std::vector<std::string> timer_names)
        {
            double time_now = ros::Time::now().toSec();
            for (auto timer_name : timer_names)
            {
                start(timer_name, time_now);
            }
        }

        void Timer::stop(const std::string timer_name, const double time)
        {
            if (!times_[timer_name].running) return;

            if (time < 0)
            {
                times_[timer_name].end_time = ros::Time::now().toSec();
            }
            else
            {
                times_[timer_name].end_time = time;
            }
            times_[timer_name].running = false;
            times_[timer_name].duration = times_[timer_name].end_time - times_[timer_name].begin_time;
        }

        void Timer::stop(const std::vector<std::string> timer_names)
        {
            std::vector<std::string> stop_times = timer_names;
            if(stop_times.empty())
            {
                for (auto it = times_.begin(); it != times_.end(); ++it) 
                {
                    stop_times.push_back(it->first);
                }
            }

            double time_now = ros::Time::now().toSec();
            for (auto timer_name : stop_times)
            {
                stop(timer_name, time_now);
            }
        }

        void Timer::print_time(const std::vector<std::string> timer_names)
        {
            std::vector<std::string> print_times = timer_names;
            if(print_times.empty())
            {
                for (auto it = times_.begin(); it != times_.end(); ++it) 
                {
                    print_times.push_back(it->first);
                }
            }
            
            for(std::string timer_name : print_times)
            {
                std::cout<< timer_name <<": ";
                if (times_[timer_name].running)
                {
                    std::cout<< "running ";
                }
                else
                {
                    std::cout<< times_[timer_name].duration << " [s] ";
                }
            }
            std::cout<<std::endl;
        }

        void Timer::print_time(const std::string timer_name)
        {
            print_time((std::vector<std::string>){timer_name});  
        }

        void associate_obstacle(potbot_msgs::ObstacleArray& obstacle_input, const potbot_msgs::ObstacleArray& obstacle_compare, const tf2_ros::Buffer &buffer)
        {
            if (obstacle_compare.header.frame_id == "")
            {
                ROS_INFO("associate_obstacle : empty frame_id");
                for (size_t i = 0; i < obstacle_input.data.size(); i++) obstacle_input.data[i].id = i;
                return;
            }

            potbot_msgs::ObstacleArray obstacle_input_no_points;
            obstacle_input_no_points.data.resize(obstacle_input.data.size());
            for (size_t i = 0; i < obstacle_input.data.size(); i++)
            {
                obstacle_input_no_points.data[i].header = obstacle_input.data[i].header;
                obstacle_input_no_points.data[i].id     = obstacle_input.data[i].id;
                obstacle_input_no_points.data[i].pose   = obstacle_input.data[i].pose;
                obstacle_input_no_points.data[i].scale  = obstacle_input.data[i].scale;
                obstacle_input_no_points.data[i].twist  = obstacle_input.data[i].twist;
            }

            potbot_msgs::ObstacleArray obstacle_input_global;
            get_tf(buffer, obstacle_input_no_points, obstacle_compare.header.frame_id, obstacle_input_global);

            static int global_idx = 0;
            for(int i = 0; i < obstacle_input_global.data.size(); i++)
            {
                double distance_min = std::numeric_limits<double>::infinity();
                int idx = 0;
                for(int j = 0; j < obstacle_compare.data.size(); j++)
                {
                    double distance = get_Distance(obstacle_input_global.data[i].pose, obstacle_compare.data[j].pose);
                    if(distance < distance_min)
                    {
                        distance_min = distance;
                        idx = j;
                    }
                }

                if (distance_min > 1)
                {
                    obstacle_input.data[i].id = global_idx++;
                }
                else
                {
                    if (distance_min > 0.02) 
                    {
                        obstacle_input.data[i].is_moving = true;
                    }
                    obstacle_input.data[i].id = obstacle_compare.data[idx].id;
                }
            }
        }

        void find_closest_vector(const std::vector<Eigen::Vector2d>& vectors, const Eigen::Vector2d& target, Eigen::Vector2d& closest)
        {
            double minDistance = std::numeric_limits<double>::max();

            for (const auto& vec : vectors) {
                double distance = (vec - target).norm();
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = vec;
                }
            }
        }

        int get_index(const std::vector<Eigen::Vector2d>& vec, const Eigen::Vector2d& value)
        {
            auto it = std::find(vec.begin(), vec.end(), value);
            if (it != vec.end()) {
                return std::distance(vec.begin(), it);
            } else {
                return -1; // Return -1 if the element is not found
            }
        }

        void to_msg(const std::vector<Eigen::Vector2d>& vectors, nav_msgs::Path& msg)
        {
            msg.poses.clear();
            msg.header.frame_id = "map";

            for (const auto& p:vectors)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = msg.header;
                pose.pose = utility::get_Pose(p(0),p(1),0,0,0,0);
                msg.poses.push_back(pose);
            }
        }

    }

}