#include <potbot_lib/utility_ros.h>

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

        void get_rpy(const geometry_msgs::Quaternion& orientation, double &roll, double &pitch, double &yaw)
        {
            tf2::Quaternion quat;
            tf2::convert(orientation, quat);
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        }

        geometry_msgs::Quaternion get_quat(const double roll, const double pitch, const double yaw)
        {
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            geometry_msgs::Quaternion orientation;
            tf2::convert(quat, orientation);
            return orientation;
        }

        geometry_msgs::Quaternion get_quat(const Point& p)
        {
            return get_quat(p.x, p.y, p.z);
        }

        geometry_msgs::Point get_point(const double x, const double y, const double z)
        {
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            return point;
        }

        geometry_msgs::Point get_point(const Point& p)
        {
            return get_point(p.x, p.y, p.z);
        }

        void get_point(const std::vector<geometry_msgs::PoseStamped>& poses, std::vector<geometry_msgs::Point>& points)
        {
            points.resize(poses.size());
            for (size_t i = 0; i < poses.size(); i++)
            {
                points[i] = poses[i].pose.position;
            }
            
        }

        geometry_msgs::Pose get_pose(const double x, const double y, const double z, const double roll, const double pitch, const double yaw)
        {
            geometry_msgs::Pose pose;
            pose.position = get_point(x,y,z);
            pose.orientation = get_quat(roll,pitch,yaw);
            return pose;
        }

        geometry_msgs::Pose get_pose(const geometry_msgs::Point& p, const double roll, const double pitch, const double yaw)
        {
            return get_pose(p.x, p.y, p.z, roll,pitch,yaw);
        }

        geometry_msgs::Pose get_pose(const Pose& p)
        {
            return get_pose(p.position.x, p.position.y, p.position.z, p.rotation.x, p.rotation.y, p.rotation.z);
        }

        double get_distance(const geometry_msgs::Point& position1, const geometry_msgs::Point& position2)
        {
            return sqrt(pow(position2.x - position1.x,2) + pow(position2.y - position1.y,2) + pow(position2.z - position1.z,2));
        }

        double get_distance(const geometry_msgs::Pose& position1, const geometry_msgs::Pose& position2)
        {
            return get_distance(position1.position, position2.position);
        }

        double get_distance(const geometry_msgs::PoseStamped& position1, const geometry_msgs::PoseStamped& position2)
        {
            return get_distance(position1.pose.position, position2.pose.position);
        }

        double get_distance(const nav_msgs::Odometry& position1, const nav_msgs::Odometry& position2)
        {
            return get_distance(position1.pose.pose.position, position2.pose.pose.position);
        }

        void print_pose(const geometry_msgs::Pose& pose)
        {
            double r,p,y;
            get_rpy(pose.orientation,r,p,y);
            ROS_INFO("(x,y,z) = (%f, %f, %f) (r,p,y) = (%f, %f, %f)", 
                        pose.position.x, pose.position.y, pose.position.z,
                        r/M_PI*180, p/M_PI*180, y/M_PI*180);
        }

        void print_pose(const geometry_msgs::PoseStamped& pose)
        {
            print_pose(pose.pose);
        }

        void print_pose(const nav_msgs::Odometry& pose)
        {
            print_pose(pose.pose.pose);
        }

        void broadcast_frame(tf2_ros::TransformBroadcaster& bc, std::string parent_frame_id, std::string child_frame_id, const geometry_msgs::Pose& pose)
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.frame_id = parent_frame_id; // 親フレーム
            transformStamped.child_frame_id = child_frame_id; // 新しいフレーム
            transformStamped.transform.translation.x = pose.position.x; // x座標
            transformStamped.transform.translation.y = pose.position.y; // y座標
            transformStamped.transform.translation.z = pose.position.z; // z座標
            transformStamped.transform.rotation.x = pose.orientation.x; // 回転クォータニオンのx成分
            transformStamped.transform.rotation.y = pose.orientation.y; // 回転クォータニオンのy成分
            transformStamped.transform.rotation.z = pose.orientation.z; // 回転クォータニオンのz成分
            transformStamped.transform.rotation.w = pose.orientation.w; // 回転クォータニオンのw成分

            transformStamped.header.stamp = ros::Time::now();
            bc.sendTransform(transformStamped);
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
            pose_stamped_in.pose = get_pose(point_in.point,0,0,0);
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

        geometry_msgs::PoseStamped get_frame_pose(const tf2_ros::Buffer &buffer, const std::string source_frame_id, const std::string target_frame_id)
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

        geometry_msgs::Point get_map_coordinate(int index, nav_msgs::MapMetaData info)
        {
            geometry_msgs::Point p;
            p.x = (index % info.width) * info.resolution + info.origin.position.x;
            p.y = (index / info.width) * info.resolution + info.origin.position.y;
            return p;
        }

        int get_map_index(const double x, const double y, const nav_msgs::MapMetaData& info)
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

        void set_path_orientation(std::vector<geometry_msgs::PoseStamped>& path)
        {
            if (path.size() > 1)
            {
                for (size_t i = 1; i < path.size(); i++)
                {
                    double xi = path[i].pose.position.x;
                    double yi = path[i].pose.position.y;
                    double xi_1 = path[i-1].pose.position.x;
                    double yi_1 = path[i-1].pose.position.y;

                    double yaw = atan2(yi-yi_1,xi-xi_1);
                    path[i].pose.orientation = get_quat(0,0,yaw);
                }
                
            }
        }

        int get_path_index(const std::vector<geometry_msgs::PoseStamped>& path, const geometry_msgs::Point& position)
        {
            double min_distance = std::numeric_limits<double>::infinity();
            int path_index = 0;
            for (int i = 0; i < path.size(); i++)
            {
                double distance = get_distance(path[i].pose.position, position);
                if(distance < min_distance)
                {
                    min_distance = distance;
                    path_index = i;
                }
            }
            return path_index;
        }

        int get_path_index(const std::vector<geometry_msgs::PoseStamped>& path, const geometry_msgs::Pose& position)
        {
            return get_path_index(path, position.position);
        }

        int get_path_index(const std::vector<geometry_msgs::PoseStamped>& path, const geometry_msgs::PoseStamped& position)
        {
            return get_path_index(path, position.pose.position);
        }

        int get_path_index(const std::vector<geometry_msgs::PoseStamped>& path, const nav_msgs::Odometry& position)
        {
            return get_path_index(path, position.pose.pose.position);
        }

        int get_path_index(const nav_msgs::Path& path, const geometry_msgs::Point& position)
        {
            return get_path_index(path.poses, position);
        }

        int get_path_index(const nav_msgs::Path& path, const geometry_msgs::Pose& position)
        {
            return get_path_index(path.poses, position);
        }

        int get_path_index(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& position)
        {
            return get_path_index(path.poses, position);
        }

        int get_path_index(const nav_msgs::Path& path, const nav_msgs::Odometry& position)
        {
            return get_path_index(path.poses, position);
        }

        double get_path_length(const std::vector<geometry_msgs::PoseStamped>& path)
        {
            double total_length = 0;
            for (int i = 1; i < path.size(); i++)
            {
                total_length += get_distance(path[i].pose.position, path[i-1].pose.position);
            }
            return total_length;
        }

        double get_path_length(const nav_msgs::Path& path)
        {
            return get_path_length(path.poses);
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
                    double distance = get_distance(obstacle_input_global.data[i].pose, obstacle_compare.data[j].pose);
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

        void to_msg(const std::vector<Eigen::Vector2d>& vectors, std::vector<geometry_msgs::PoseStamped>& msg)
        {
            msg.clear();
            for (const auto& p:vectors)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose = utility::get_pose(p(0),p(1),0,0,0,0);
                msg.push_back(pose);
            }
        }

        void to_msg(const std::vector<Eigen::Vector2d>& vectors, nav_msgs::Path& msg)
        {
            to_msg(vectors, msg.poses);
        }

        void to_mat(const std::vector<geometry_msgs::PoseStamped>& msg, std::vector<Eigen::Vector2d>& vectors)
        {
            vectors.clear();
            for (const auto& p:msg)
            {
                vectors.push_back(Eigen::Vector2d(p.pose.position.x, p.pose.position.y));
            }
        }

        void to_mat(const nav_msgs::Path& msg, std::vector<Eigen::Vector2d>& vectors)
        {
            to_mat(msg.poses,vectors);
        }

        std_msgs::Float64MultiArray matrix_to_multiarray(const Eigen::MatrixXd& mat)
        {
            std_msgs::Float64MultiArray multiarray;

            multiarray.data.resize(mat.size());
            multiarray.layout.data_offset = 0;
            multiarray.layout.dim.resize(2);

            // multiarray(i,j) = data[data_offset + dim_stride[1]*i + j]
            
            multiarray.layout.dim[0].label = "row";
            multiarray.layout.dim[0].size = mat.rows();
            multiarray.layout.dim[0].stride = multiarray.layout.dim[0].size;

            multiarray.layout.dim[1].label = "col";
            multiarray.layout.dim[1].size = mat.cols();
            multiarray.layout.dim[1].stride = multiarray.layout.dim[1].size;

            for (size_t i = 0; i < mat.rows(); i++)
            {
                for (size_t j = 0; j < mat.cols(); j++)
                {
                    multiarray.data[multiarray.layout.data_offset + multiarray.layout.dim[1].stride*i + j] = mat(i,j);
                }
            }

            return multiarray;
        }

        Eigen::MatrixXd multiarray_to_matrix(const std_msgs::Float64MultiArray& multiarray)
        {
            size_t rows = multiarray.layout.dim[0].size;
            size_t cols = multiarray.layout.dim[1].size;
            Eigen::MatrixXd mat(rows, cols);
            for (size_t i = 0; i < rows; i++)
            {
                for (size_t j = 0; j < cols; j++)
                {
                    mat(i,j) = multiarray.data[multiarray.layout.data_offset + multiarray.layout.dim[1].stride*i + j];
                }
            }
            return mat;
        }

        void obstacle_array_to_marker_array(const potbot_msgs::ObstacleArray& obstacle_array, visualization_msgs::MarkerArray& marker_array)
        {
            marker_array.markers.clear();
            for (const auto& obs : obstacle_array.data)
            {
                // double v = obs.twist.linear.x;
                // if (abs(v) > 2.0 || abs(v) < 0.1) continue;

                visualization_msgs::Marker state_marker;
                visualization_msgs::Marker text_marker;

                state_marker.header             = obs.header;

                state_marker.ns                 = "segments/centor";
                state_marker.id                 = obs.id;
                state_marker.lifetime           = ros::Duration(1);

                state_marker.type               = visualization_msgs::Marker::ARROW;
                state_marker.action             = visualization_msgs::Marker::MODIFY;

                state_marker.pose               = obs.pose;

                state_marker.scale.x            = 0.05;
                state_marker.scale.y            = 0.1;
                state_marker.scale.z            = 0.1;
                
                state_marker.color              = potbot_lib::color::get_msg(obs.id);
                state_marker.color.a            = 1;

                text_marker = state_marker;

                geometry_msgs::Point p0, p1;
                p0.x                            = 0;
                p0.y                            = 0;
                p0.z                            = 0;
                p1.x                            = obs.twist.linear.x;
                p1.y                            = 0;
                p1.z                            = 0;
                state_marker.points.push_back(p0);
                // state_marker.colors.push_back(state_marker.color);
                state_marker.points.push_back(p1);
                // state_marker.colors.push_back(state_marker.color);
                
                marker_array.markers.push_back(state_marker);

                text_marker.id                 = obs.id + 100000;
                text_marker.text               = "id:" + std::to_string(obs.id);// + "\nlinear:" + std::to_string(obs.twist.linear.x) + ", angular:" + std::to_string(obs.twist.angular.z);
                text_marker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.action             = visualization_msgs::Marker::MODIFY;
                text_marker.pose.position.x    += 0.3;
                text_marker.scale.x            = 0.2;
                text_marker.scale.y            = 0.2;
                text_marker.scale.z            = 0.2;
                marker_array.markers.push_back(text_marker);

            }
        }

        void field_to_pcl2(std::vector<potential::FieldGrid>& field, sensor_msgs::PointCloud2& pcl_msg)
        {
            // std::vector<pcl::PointXYZ> を作成
            std::vector<pcl::PointXYZ> pointVector;
            for (auto value : field)
            {
                double x = value.x;
                double y = value.y;
                double z = value.value;
                pcl::PointXYZ point(x,y,z);
                pointVector.push_back(point);
            }

            // std::vector<pcl::PointXYZ> を pcl::PointCloud に変換
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pclPointCloud->points.resize(pointVector.size());
            for (size_t i = 0; i < pointVector.size(); ++i) {
                pclPointCloud->points[i] = pointVector[i];
            }

            // pcl::PointCloud を sensor_msgs::PointCloud2 に変換
            pcl::toROSMsg(*pclPointCloud, pcl_msg);
        }

        void to_agent(const geometry_msgs::Pose& msg, DiffDriveAgent& agent)
        {
            agent.x = msg.position.x;
            agent.y = msg.position.y;
            agent.yaw = tf2::getYaw(msg.orientation);
        }

        void to_agent(const geometry_msgs::PoseStamped& msg, DiffDriveAgent& agent)
        {
            to_agent(msg.pose, agent);
        }

        void to_agent(const geometry_msgs::Twist& msg, DiffDriveAgent& agent)
        {
            agent.v = msg.linear.x;
            agent.omega = msg.angular.z;
        }
        
        void to_agent(const nav_msgs::Odometry& msg, DiffDriveAgent& agent)
        {
            to_agent(msg.pose.pose, agent);
            to_agent(msg.twist.twist, agent);
        }

        void to_msg(DiffDriveAgent& agent, geometry_msgs::Pose& msg)
        {
            msg.position.x = agent.x;
            msg.position.y = agent.y;
            msg.orientation = get_quat(0,0,agent.yaw);
        }

        void to_msg(DiffDriveAgent& agent, geometry_msgs::Twist& msg)
        {
            msg.linear.x = agent.v;
            msg.angular.z = agent.omega;
        }

        void to_msg(DiffDriveAgent& agent, nav_msgs::Odometry& msg)
        {
            msg.header.stamp = ros::Time::now();
            to_msg(agent, msg.pose.pose);
            to_msg(agent, msg.twist.twist);
        }

    }

}