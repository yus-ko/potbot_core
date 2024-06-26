#include <potbot_lib/interactive_marker_manager.h>

namespace potbot_lib{

	InteractiveMarkerManager::InteractiveMarkerManager(std::string name)
	{
		name_space_ = name;

		ros::NodeHandle nh("~/" + name_space_);
		nh.getParam("frame_id_global", frame_id_global_);
		int n = interactive_marker_num_;
		nh.getParam("marker_num", n);
		interactive_marker_num_ = n;

		pub_marker_trajectory_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
	}

	InteractiveMarkerManager::~InteractiveMarkerManager()
	{
	}

	void InteractiveMarkerManager::initInteractiveMarkers(visualization_msgs::Marker init_marker)
	{
		interactive_markers_.resize(interactive_marker_num_);
		std::fill(interactive_markers_.begin(), interactive_markers_.end(), init_marker);

		trajectories_.resize(interactive_marker_num_);
		for (size_t i = 0; i < interactive_marker_num_; i++)
		{
			visualization_msgs::Marker traj;
			traj.id = i;
			traj.type = visualization_msgs::Marker::LINE_STRIP;
			traj.header.frame_id = frame_id_global_;
			traj.pose = potbot_lib::utility::get_Pose();
			traj.scale.x = 0.02;
			traj.color = interactive_markers_[i].color;
			// traj.points.push_back(traj.pose);
			trajectory_markers_.markers.push_back(traj);
		}
	}

	void InteractiveMarkerManager::initInteractiveMarkers()
	{
		visualization_msgs::Marker default_marker;
		default_marker.type = visualization_msgs::Marker::CUBE;
		default_marker.scale.x = 0.5;
		default_marker.scale.y = 0.5;
		default_marker.scale.z = 0.2;
		default_marker.color.r = 0.5;
		default_marker.color.g = 0.5;
		default_marker.color.b = 0.5;
		default_marker.color.a = 1.0;
		default_marker.pose = potbot_lib::utility::get_Pose();
		initInteractiveMarkers(default_marker);
	}

	void InteractiveMarkerManager::initInteractiveMarkerServer()
	{
		if (interactive_markers_.size() != interactive_marker_num_)
		{
			initInteractiveMarkers();
		}
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

		for (size_t i = 0; i < interactive_marker_num_; i++)
		{
			visualization_msgs::InteractiveMarker int_marker;
			int_marker.header.frame_id = frame_id_global_;
			int_marker.header.stamp = ros::Time::now();
			int_marker.name = "obstacle_" + std::to_string(i);
			int_marker.description = int_marker.name;
			int_marker.pose = potbot_lib::utility::get_Pose(0,0,0,0,0,0);
			// int_marker.pose = potbot_lib::utility::get_Pose(6,0,1,0,0,0);

			move_control.markers[0] = interactive_markers_[i];

			int_marker.controls.push_back(move_control);
			int_marker.controls.push_back(rotate_control);

			imsrv_->insert(int_marker, boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));

			interactive_markers_[i].pose = int_marker.pose;
			menu_handler_->apply(*imsrv_, int_marker.name);
		}

		imsrv_->applyChanges();
	}

	void InteractiveMarkerManager::markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
	{
		std::stringstream ss(feedback->marker_name);
		std::string segment;
		std::vector<std::string> segments;

		// ROS_INFO_STREAM("Menu item " << feedback->menu_entry_id << " clicked in marker " << feedback->marker_name);

		// "_"で文字列を分割
		while (std::getline(ss, segment, '_'))
		{
			segments.push_back(segment);
		}

		if (segments.size() > 1)
		{
			if (segments[0] == "obstacle")
			{
				int id = std::stoi(segments[1]);
				interactive_markers_[id].pose = feedback->pose;

				if (trajectories_[id].size() == 0 || interactive_markers_[id].pose != trajectories_[id].back().pose)
				{
					geometry_msgs::PoseStamped pose;
					pose.header.frame_id = frame_id_global_;
					pose.header.stamp = ros::Time::now();
					pose.pose = interactive_markers_[id].pose;
					trajectories_[id].push_back(pose);

					trajectory_markers_.markers[id].points.push_back(pose.pose.position);
					trajectory_markers_.markers[id].header.stamp = pose.header.stamp;
					pub_marker_trajectory_.publish(trajectory_markers_);
				}

				visualization_msgs::InteractiveMarker int_marker;
				if (imsrv_->get(feedback->marker_name, int_marker)) 
				{
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

					interactive_markers_[id].scale = int_marker.controls[0].markers[0].scale;
					interactive_markers_[id].type = int_marker.controls[0].markers[0].type;

					// 変更をサーバーに反映
					imsrv_->insert(int_marker, boost::bind(&InteractiveMarkerManager::markerFeedback, this, _1));
					imsrv_->applyChanges();
				}
			}
		}
	}

	std::vector<visualization_msgs::Marker>* InteractiveMarkerManager::getMarker()
	{
		return &interactive_markers_;
	}

	std::vector<std::vector<geometry_msgs::PoseStamped>>* InteractiveMarkerManager::getMarkerTrajectories()
	{
		return &trajectories_;
	}
}