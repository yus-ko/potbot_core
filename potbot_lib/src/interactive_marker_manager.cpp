#include <potbot_lib/interactive_marker_manager.h>

namespace potbot_lib{

	InteractiveMarkerManager::InteractiveMarkerManager(std::string name)
	{
		name_space_ = name;

		ros::NodeHandle nh("~");
		imsrv_ = new interactive_markers::InteractiveMarkerServer(nh.getNamespace() + "/" + name_space_ + "/simple_marker");
		menu_handler_ = new interactive_markers::MenuHandler;

		interactive_markers::MenuHandler::EntryHandle x_entry = menu_handler_->insert("scale x");
		interactive_markers::MenuHandler::EntryHandle y_entry = menu_handler_->insert("scale y");
		interactive_markers::MenuHandler::EntryHandle xy_entry = menu_handler_->insert("scale xy");

		menu_handler_->insert( x_entry, "x2" , boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));
		menu_handler_->insert( x_entry, "x0.5" , boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));

		menu_handler_->insert( y_entry, "x2" , boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));
		menu_handler_->insert( y_entry, "x0.5" , boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));

		menu_handler_->insert( xy_entry, "x2" , boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));
		menu_handler_->insert( xy_entry, "x0.5" , boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));

		interactive_markers::MenuHandler::EntryHandle type_entry = menu_handler_->insert("marker type");
		menu_handler_->insert( type_entry, "cube" , boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));
		menu_handler_->insert( type_entry, "sphere" , boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));

		visualization_msgs::Marker move_marker;
		move_marker.type = visualization_msgs::Marker::SPHERE;
		move_marker.scale.x = 0.2;
		move_marker.scale.y = 0.2;
		move_marker.scale.z = 0.2;
		move_marker.color.r = 0.0;
		move_marker.color.g = 0.0;
		move_marker.color.b = 0.7;
		move_marker.color.a = 1.0;
		move_marker.pose = potbot_lib::utility::get_Pose(0,0,0,0,0,0);
		// move_marker.pose = potbot_lib::utility::get_Pose(0,0.5,1,0,0,0);

		visualization_msgs::InteractiveMarkerControl move_control;
		move_control.name = "move_plane";
		move_control.orientation = potbot_lib::utility::get_Quat(0,-M_PI_2,0);
		move_control.always_visible = true;
		move_control.markers.push_back(move_marker);
		move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

		visualization_msgs::InteractiveMarkerControl rotate_control;
		rotate_control.name = "rotate_yaw";
		rotate_control.orientation = potbot_lib::utility::get_Quat(0,-M_PI_2,0);
		rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

		ros::NodeHandle pnh("~/" + name_space_);
		pnh.getParam("frame_id_global", frame_id_global_);

		pub_marker_trajectory_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);

		srv_save_marker_trajectory_ = pnh.advertiseService("save_marker_tarajectory", &InteractiveMarkerManager::serviceSaveMarkerTrajectory, this);
		srv_clear_marker_trajectory_ = pnh.advertiseService("clear_marker_tarajectory", &InteractiveMarkerManager::serviceClearMarkerTrajectory, this);

		XmlRpc::XmlRpcValue markers;
		pnh.getParam("markers", markers);
		interactive_marker_num_ = markers.size();
		visual_markers_.resize(interactive_marker_num_);
		for (size_t i = 0; i < interactive_marker_num_; i++)
		{
			std::string marker_name = static_cast<std::string>(markers[i]["name"]);
			std::string marker_type = "sphere";
			std::string trajectory_marker_type = "line";
			double x=0,y=0,z=0,roll=0,pitch=0,yaw=0;
			double scale_x=0.05,scale_y=0.05,scale_z=0.05;
			double r=0.7,g=0,b=0,a=1;

			pnh.getParam(marker_name+"/type", marker_type);

			pnh.getParam(marker_name+"/trajectory_recording", visual_markers_[i].trajectory_recording);
			pnh.getParam(marker_name+"/trajectory_marker_type", trajectory_marker_type);
			pnh.getParam(marker_name+"/trajectory_interpolation_method", visual_markers_[i].trajectory_interpolation_method);
			
			pnh.getParam(marker_name+"/initial_pose/x", x);
			pnh.getParam(marker_name+"/initial_pose/y", y);
			pnh.getParam(marker_name+"/initial_pose/z", z);
			pnh.getParam(marker_name+"/initial_pose/roll", roll);
			pnh.getParam(marker_name+"/initial_pose/pitch", pitch);
			pnh.getParam(marker_name+"/initial_pose/yaw", yaw);

			pnh.getParam(marker_name+"/scale/x", scale_x);
			pnh.getParam(marker_name+"/scale/y", scale_y);
			pnh.getParam(marker_name+"/scale/z", scale_z);

			pnh.getParam(marker_name+"/color/r", r);
			pnh.getParam(marker_name+"/color/g", g);
			pnh.getParam(marker_name+"/color/b", b);
			pnh.getParam(marker_name+"/color/a", a);

			if (marker_type == "sphere")
			{
				visual_markers_[i].marker.type = visualization_msgs::Marker::SPHERE;
			}
			else if (marker_type == "cube")
			{
				visual_markers_[i].marker.type = visualization_msgs::Marker::CUBE;
			}	

			if (trajectory_marker_type == "line")
			{
				visual_markers_[i].trajectory_marker_type = visualization_msgs::Marker::LINE_STRIP;
			}
			else if (trajectory_marker_type == "points")
			{
				visual_markers_[i].trajectory_marker_type = visualization_msgs::Marker::POINTS;
			}	

			visual_markers_[i].marker.text = marker_name;

			visual_markers_[i].marker.scale.x = scale_x;
			visual_markers_[i].marker.scale.y = scale_y;
			visual_markers_[i].marker.scale.z = scale_z;

			visual_markers_[i].marker.color.r = r;
			visual_markers_[i].marker.color.g = g;
			visual_markers_[i].marker.color.b = b;
			visual_markers_[i].marker.color.a = a;

			visual_markers_[i].marker.pose = potbot_lib::utility::get_Pose();

			visualization_msgs::InteractiveMarker int_marker;
			int_marker.header.frame_id = frame_id_global_;
			int_marker.header.stamp = ros::Time::now();
			// int_marker.name = "obstacle_" + std::to_string(i);
			int_marker.name = visual_markers_[i].marker.text;
			int_marker.description = int_marker.name;
			int_marker.pose = potbot_lib::utility::get_Pose(x,y,z,roll,pitch,yaw);
			// int_marker.pose = potbot_lib::utility::get_Pose(6,0,1,0,0,0);

			move_control.markers[0] = visual_markers_[i].marker;

			int_marker.controls.push_back(move_control);
			int_marker.controls.push_back(rotate_control);

			imsrv_->insert(int_marker, boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));

			visual_markers_[i].marker.pose = int_marker.pose;
			menu_handler_->apply(*imsrv_, int_marker.name);
		}

		imsrv_->applyChanges();

		dsrv_ = new dynamic_reconfigure::Server<potbot_lib::MarkerManagerConfig>(pnh);
		dynamic_reconfigure::Server<potbot_lib::MarkerManagerConfig>::CallbackType cb = boost::bind(&InteractiveMarkerManager::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);
	}

	void InteractiveMarkerManager::reconfigureCB(const potbot_lib::MarkerManagerConfig& param, uint32_t level)
	{	
		static bool first = true;

		if (!first)
		{
			int id = param.marker_id;
			std::string visual_type = param.trajectory_marker_type;

			u_int8_t type = visualization_msgs::Marker::LINE_STRIP;
			if (visual_type == "line")
			{
				type = visualization_msgs::Marker::LINE_STRIP;
			}
			else if (visual_type == "points")
			{
				type = visualization_msgs::Marker::POINTS;
			}

			std::vector<size_t> ids;
			if (id == -1)
			{
				for (size_t i = 0; i < visual_markers_.size(); i++)
				{
					ids.push_back(id);
				}
			}
			else
			{
				ids.push_back(id);
			}
			
			for (const auto& i:ids)
			{
				if (i < visual_markers_.size())
				{
					visual_markers_[i].trajectory_recording = param.trajectory_recording;
					if (visual_markers_[i].trajectory_recording)
					{
						visual_markers_[i].trajectory_marker_type = type;
						visual_markers_[i].trajectory_interpolation_method = param.trajectory_interpolation_method;
					}
					else
					{
						visual_markers_[i].trajectory.clear();
					}
				}
			}
			publishMarkerTrajectory();
		}

		first = false;
	}

	void InteractiveMarkerManager::markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
	{
		visualization_msgs::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker)) 
		{
			int id = -1;
			for (size_t i = 0; i < interactive_marker_num_; i++)
			{
				if (int_marker.name == visual_markers_[i].marker.text)
				{
					id = i;
					break;
				}
			}
			if (id < 0)
			{
				return;
			}
			
			visual_markers_[id].marker.pose = feedback->pose;
			visual_markers_[id].marker.header = feedback->header;

			if (visual_markers_[id].trajectory_recording)
			{
				interpolateTrajectory(id);
			}

			size_t eid = feedback->menu_entry_id;
			if (eid == 4)
			{
				int_marker.controls[0].markers[0].scale.x *= 2;
			}
			else if (eid == 5)
			{
				int_marker.controls[0].markers[0].scale.x *= 0.5;
			}
			else if (eid == 6)
			{
				int_marker.controls[0].markers[0].scale.y *= 2;
			}
			else if (eid == 7)
			{
				int_marker.controls[0].markers[0].scale.y *= 0.5;
			}
			else if (eid == 8)
			{
				int_marker.controls[0].markers[0].scale.x *= 2;
				int_marker.controls[0].markers[0].scale.y *= 2;
			}
			else if (eid == 9)
			{
				int_marker.controls[0].markers[0].scale.x *= 0.5;
				int_marker.controls[0].markers[0].scale.y *= 0.5;
			}
			else if (eid == 11)
			{
				int_marker.controls[0].markers[0].type = visualization_msgs::Marker::CUBE;
			}
			else if (eid == 12)
			{
				int_marker.controls[0].markers[0].type = visualization_msgs::Marker::SPHERE;
			}

			visual_markers_[id].marker.scale = int_marker.controls[0].markers[0].scale;
			visual_markers_[id].marker.type = int_marker.controls[0].markers[0].type;

			// 変更をサーバーに反映
			imsrv_->insert(int_marker, boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));
			imsrv_->applyChanges();
		}
	}

	void InteractiveMarkerManager::interpolateTrajectory(size_t id)
	{
		if (visual_markers_[id].trajectory.empty())
		{
			geometry_msgs::PoseStamped p;
			p.header = visual_markers_[id].marker.header;
			p.pose = visual_markers_[id].marker.pose;
			visual_markers_[id].trajectory.push_back(p);
		}
		
		double distance_to_pre = utility::get_Distance(visual_markers_[id].marker.pose, visual_markers_[id].trajectory.back().pose);
		if (distance_to_pre > 0.01)
		{
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = frame_id_global_;
			pose.header.stamp = ros::Time::now();
			pose.pose = visual_markers_[id].marker.pose;

			if (distance_to_pre > 0.05)
			{
				std::vector<geometry_msgs::PoseStamped> start_end(2);
				start_end[0] = visual_markers_[id].trajectory.back();
				start_end[1] = pose;
				
				std::vector<Eigen::Vector2d> interp_vecs;
				utility::to_mat(start_end,interp_vecs);
				interpolate::linear(interp_vecs, int(distance_to_pre/0.05), interp_vecs);
				utility::to_msg(interp_vecs,start_end);

				ROS_DEBUG("distance_to_pre: %f, linear_interpolate_num: %d", distance_to_pre, (int)start_end.size());

				for (size_t i = 1; i < start_end.size(); i++)
				{
					start_end[i].header = pose.header;
					visual_markers_[id].trajectory.push_back(start_end[i]);
				}
			}
			else
			{
				visual_markers_[id].trajectory.push_back(pose);
			}

			auto* points = &visual_markers_[id].trajectory;
			size_t num = points->size();
			size_t interpolate_num_limit = 50;
			static size_t last_interpolated_index = 0;
			if (num > 1)
			{
				std::vector<geometry_msgs::PoseStamped> split_traj;
				if (num > interpolate_num_limit)
				{
					for (size_t i = num - interpolate_num_limit; i < num; i++)
					{
						split_traj.push_back((*points)[i]);
					}
				}
				else
				{
					split_traj = *points;
				}
				
				if (split_traj.size() > 1)
				{
					std::vector<Eigen::Vector2d> traj_vecs;
					utility::to_mat(split_traj,traj_vecs);
					if (visual_markers_[id].trajectory_interpolation_method == "spline")
					{
						interpolate::spline(traj_vecs, traj_vecs.size(), traj_vecs);
					}
					else if (visual_markers_[id].trajectory_interpolation_method == "bezier")
					{
						interpolate::bezier(traj_vecs, traj_vecs.size(), traj_vecs);
					}
					utility::to_msg(traj_vecs,split_traj);
				}

				if (num > interpolate_num_limit)
				{
					for (size_t i = 0; i < split_traj.size(); i++)
					{
						visual_markers_[id].trajectory[i+num-interpolate_num_limit] = split_traj[i];
					}
				}
				else
				{
					visual_markers_[id].trajectory = split_traj;
				}
				ROS_DEBUG("interpolate size: %d, trajectories_[%d]_size: %d", (int)split_traj.size(), (int)id, (int)visual_markers_[id].trajectory.size());

			}
			publishMarkerTrajectory();
		}
	}

	void InteractiveMarkerManager::publishMarkerTrajectory()
	{
		visualization_msgs::MarkerArray traj_marker;
		for (const auto& vismark:visual_markers_)
		{
			if (vismark.trajectory_recording)
			{
				visualization_msgs::Marker traj = vismark.marker;
				traj.type = vismark.trajectory_marker_type;
				traj.pose = potbot_lib::utility::get_Pose();
				traj.scale.x = 0.01;
				traj.scale.y = 0.01;
				traj.scale.z = 0.01;
				utility::get_Point(vismark.trajectory, traj.points);
				traj_marker.markers.push_back(traj);
			}
		}
		pub_marker_trajectory_.publish(traj_marker);
	}

	bool directoryExists(const std::string &path) 
	{
		struct stat info;
		if (stat(path.c_str(), &info) != 0) 
		{
			return false; // ディレクトリは存在しない
		}
		else if (info.st_mode & S_IFDIR) 
		{
			return true; // ディレクトリは存在する
		} 
		else 
		{
			return false; // パスはディレクトリではない
		}
	}

	bool createDirectory(const std::string &path) 
	{
		if (mkdir(path.c_str(), 0755) != 0) 
		{
			if (errno == EEXIST) 
			{
				return true; // ディレクトリは既に存在する
			} else 
			{
				return false; // ディレクトリの作成に失敗
			}
		}
		return true; // ディレクトリの作成に成功
	}

	bool createDirectoriesRecursively(const std::string &path) 
	{
		size_t pos = 0;
		std::string current_path;

		while ((pos = path.find_first_of('/', pos)) != std::string::npos) 
		{
			current_path = path.substr(0, pos++);
			if (!current_path.empty() && !directoryExists(current_path)) 
			{
				if (!createDirectory(current_path)) 
				{
					return false;
				}
			}
		}

		if (!directoryExists(path)) {
			if (!createDirectory(path)) 
			{
				return false;
			}
		}

		return true;
	}

	int InteractiveMarkerManager::getMarkerId(std::string marker_name)
	{
		for (size_t i = 0; i < visual_markers_.size(); i++)
		{
			if (visual_markers_[i].marker.text == marker_name)
			{
				return i;
			}
			else if (i == visual_markers_.size() - 1)
			{
				return -1;
			}
		}
	}

	bool InteractiveMarkerManager::serviceSaveMarkerTrajectory(potbot_lib::Save::Request &req, potbot_lib::Save::Response &resp)
	{
		std::string marker_name = req.save_target;
		int id = 0;

		if (marker_name == "")
		{
			marker_name = visual_markers_[0].marker.text;
		}
		else
		{
			id = getMarkerId(marker_name);
			if (id == -1)
			{
				resp.success = false;
				resp.message = "Invalid save_target: " + marker_name;
				return false;
			}
		}
		
		std::string csv_path = req.full_path;

		if (csv_path == "")
		{
			csv_path = std::string(std::getenv("HOME")) + "/.ros/marker/trajectory/" + marker_name + ".csv";
		}

		size_t last_slash_pos = csv_path.find_last_of('/');
    	std::string directory_path = csv_path.substr(0, last_slash_pos);

		// ディレクトリが存在するか確認し、存在しない場合は作成
		if (!directoryExists(directory_path)) 
		{
			if (!createDirectoriesRecursively(directory_path)) 
			{
				ROS_ERROR_STREAM("Failed to create directory: " << directory_path);
				resp.success = false;
				resp.message = "Failed to create directory: " + directory_path;
				return false;
			}
		}

		std::ofstream csv_file(csv_path);

		if (csv_file.is_open()) 
		{
			for (const auto& p : visual_markers_[id].trajectory) 
			{
				double roll,pitch,yaw;
				utility::get_RPY(p.pose.orientation, roll, pitch, yaw);

				csv_file	<< p.pose.position.x 	<< ","
							<< p.pose.position.y 	<< ","
							<< p.pose.position.z 	<< ","
							<< roll 				<< ","
							<< pitch 				<< ","
							<< yaw 					<< "\n";
			}
			csv_file.close();

			ROS_INFO_STREAM("Saved to: " << csv_path);
			resp.success = true;
			resp.message = "Saved to: " + csv_path;
			return true;
		} 
		else 
		{
			ROS_ERROR_STREAM("Failed to open file: " << csv_path);
			resp.success = false;
			resp.message = "Failed to open file: " + csv_path;
			return false;
		}
		
	}

	bool InteractiveMarkerManager::serviceClearMarkerTrajectory(potbot_lib::Save::Request &req, potbot_lib::Save::Response &resp)
	{
		std::string marker_name = req.save_target;

		if (marker_name == "")
		{
			std::string names = "";
			for (auto& vm:visual_markers_)
			{
				vm.trajectory.clear();
				names+=vm.marker.text + ", ";
			}
			names.pop_back();
			names.pop_back();
			ROS_INFO_STREAM("Clear trajectory: " << names);
			resp.success = true;
			resp.message = "Clear trajectory: " + names;
		}
		else
		{
			int id = getMarkerId(marker_name);
			if (id == -1)
			{
				ROS_INFO_STREAM("Invalid target name: " << marker_name);
				resp.success = false;
				resp.message = "Invalid target name: " + marker_name;
				return false;
			}
			else
			{
				visual_markers_[id].trajectory.clear();
				ROS_INFO_STREAM("Clear trajectory: " << visual_markers_[id].marker.text);
				resp.success = true;
				resp.message = "Clear trajectory: " + visual_markers_[id].marker.text;
			}
		}
		return true;
	}

	std::vector<VisualMarker>* InteractiveMarkerManager::getVisualMarker()
	{
		return &visual_markers_;
	}
}