#include <potbot_ros/interactive_marker_manager.hpp>

namespace potbot_lib{

	InteractiveMarkerManager::InteractiveMarkerManager(std::string name, rclcpp::Node* node) : name_space_(name), parent_node_(node)
	{

		parent_node_->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory", 1);

		// srv_save_marker_trajectory_ = pnh.advertiseService("save_marker_tarajectory", &InteractiveMarkerManager::serviceSaveMarkerTrajectory, this);
		// srv_clear_marker_trajectory_ = pnh.advertiseService("clear_marker_tarajectory", &InteractiveMarkerManager::serviceClearMarkerTrajectory, this);
		
		initializeParameter();
		initializeController();
		initializeMarker();

		dyn_params_handler_ = parent_node_->add_on_set_parameters_callback(
			std::bind(&InteractiveMarkerManager::dynamicParametersCallback, this, std::placeholders::_1));
	}

	void InteractiveMarkerManager::initializeParameter()
	{
		parent_node_->declare_parameter("frame_id_global", rclcpp::ParameterValue("map"));
		parent_node_->declare_parameter("markers", rclcpp::ParameterValue(std::vector<std::string>{"marker"}));

		auto markers = parent_node_->get_parameter("markers").as_string_array();
		interactive_marker_num_ = markers.size();
		visual_markers_.resize(interactive_marker_num_);
		for (const auto &maker:markers)
		{
			parent_node_->declare_parameter(maker + ".type", rclcpp::ParameterValue("sphere"));

			parent_node_->declare_parameter(maker + ".trajectory_recording", rclcpp::ParameterValue(false));
			parent_node_->declare_parameter(maker + ".trajectory_marker_type", rclcpp::ParameterValue("line"));
			parent_node_->declare_parameter(maker + ".trajectory_interpolation_method", rclcpp::ParameterValue("none"));

			parent_node_->declare_parameter(maker + ".initial_pose.x", rclcpp::ParameterValue(0.0));
			parent_node_->declare_parameter(maker + ".initial_pose.y", rclcpp::ParameterValue(0.0));
			parent_node_->declare_parameter(maker + ".initial_pose.z", rclcpp::ParameterValue(0.0));
			parent_node_->declare_parameter(maker + ".initial_pose.roll", rclcpp::ParameterValue(0.0));
			parent_node_->declare_parameter(maker + ".initial_pose.pitch", rclcpp::ParameterValue(0.0));
			parent_node_->declare_parameter(maker + ".initial_pose.yaw", rclcpp::ParameterValue(0.0));

			parent_node_->declare_parameter(maker + ".scale.x", rclcpp::ParameterValue(0.05));
			parent_node_->declare_parameter(maker + ".scale.y", rclcpp::ParameterValue(0.05));
			parent_node_->declare_parameter(maker + ".scale.z", rclcpp::ParameterValue(0.05));

			parent_node_->declare_parameter(maker + ".color.r", rclcpp::ParameterValue(0.7));
			parent_node_->declare_parameter(maker + ".color.g", rclcpp::ParameterValue(0.0));
			parent_node_->declare_parameter(maker + ".color.b", rclcpp::ParameterValue(0.0));
			parent_node_->declare_parameter(maker + ".color.a", rclcpp::ParameterValue(1.0));
		}
	}

	void InteractiveMarkerManager::initializeController()
	{
		visualization_msgs::msg::Marker move_marker;
		move_marker.type = visualization_msgs::msg::Marker::SPHERE;
		move_marker.scale.x = 0.2;
		move_marker.scale.y = 0.2;
		move_marker.scale.z = 0.2;
		move_marker.color.r = 0.0;
		move_marker.color.g = 0.0;
		move_marker.color.b = 0.7;
		move_marker.color.a = 1.0;
		move_marker.pose = potbot_lib::utility::get_pose(0,0,0,0,0,0);
		// move_marker.pose = potbot_lib::utility::get_Pose(0,0.5,1,0,0,0);

		movement_controller_.name = "movement_controller_marker";
		movement_controller_.orientation = potbot_lib::utility::get_quat(0,-M_PI_2,0);
		movement_controller_.always_visible = true;
		movement_controller_.markers.push_back(move_marker);
		movement_controller_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

		rotation_controller_axis_z_.name = "rotation_controller_axis_z";
		rotation_controller_axis_z_.orientation = potbot_lib::utility::get_quat(0,-M_PI_2,0);
		rotation_controller_axis_z_.always_visible = true;
		rotation_controller_axis_z_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

		rotation_controller_ = rotation_controller_axis_z_;
		rotation_controller_.name = "rotation_controller_marker";
		rotation_controller_.markers.push_back(move_marker);

		scale_controller_axis_x_.name = "scale_controller_axis_x";
		scale_controller_axis_x_.orientation = potbot_lib::utility::get_quat(0,0,0);
		scale_controller_axis_x_.always_visible = true;
		scale_controller_axis_x_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

		scale_controller_axis_y_.name = "scale_controller_axis_y";
		scale_controller_axis_y_.orientation = potbot_lib::utility::get_quat(0,0,M_PI_2);
		scale_controller_axis_y_.always_visible = true;
		scale_controller_axis_y_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

		scale_controller_axis_z_.name = "scale_controller_axis_z";
		scale_controller_axis_z_.orientation = potbot_lib::utility::get_quat(0,M_PI_2,0);
		scale_controller_axis_z_.always_visible = true;
		scale_controller_axis_z_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

		scale_controller_.name = "scale_controller_marker";
		scale_controller_.always_visible = true;
		scale_controller_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
		scale_controller_.markers.push_back(move_marker);

	}

	void InteractiveMarkerManager::initializeMarker()
	{
		imsrv_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
			parent_node_->get_namespace() + '/' + name_space_, parent_node_);
		menu_handler_ = std::make_shared<interactive_markers::MenuHandler>();

		interactive_markers::MenuHandler::EntryHandle edit_entry = menu_handler_->insert("edit");

		menu_handler_->insert( edit_entry, "position" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->editorChangeTo(feedback, "position");});

		menu_handler_->insert( edit_entry, "rotation" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->editorChangeTo(feedback, "rotation");});

		menu_handler_->insert( edit_entry, "scale" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->editorChangeTo(feedback, "scale");});

		interactive_markers::MenuHandler::EntryHandle type_entry = menu_handler_->insert(edit_entry, "type");

		menu_handler_->insert( type_entry, "cube", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->typeChangeTo(feedback, visualization_msgs::msg::Marker::CUBE);});

		menu_handler_->insert( type_entry, "sphere", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->typeChangeTo(feedback, visualization_msgs::msg::Marker::SPHERE);});

		frame_id_global_ = parent_node_->get_parameter("frame_id_global").as_string();

		auto markers = parent_node_->get_parameter("markers").as_string_array();
		interactive_marker_num_ = markers.size();
		visual_markers_.resize(interactive_marker_num_);
		for (size_t i = 0; i < interactive_marker_num_; i++)
		{
			std::string marker_name = markers[i];

			auto marker_type = parent_node_->get_parameter(marker_name + ".type").as_string();
			auto trajectory_marker_type = parent_node_->get_parameter(marker_name + ".trajectory_marker_type").as_string();
			// visual_markers_[i].trajectory_recording = parent_node_->get_parameter(marker_name + ".trajectory_recording").as_bool();
			// visual_markers_[i].trajectory_interpolation_method = parent_node_->get_parameter(marker_name + ".trajectory_interpolation_method").as_string();

			auto x = parent_node_->get_parameter(marker_name + ".initial_pose.x").as_double();
			auto y = parent_node_->get_parameter(marker_name + ".initial_pose.y").as_double();
			auto z = parent_node_->get_parameter(marker_name + ".initial_pose.z").as_double();
			auto roll = parent_node_->get_parameter(marker_name + ".initial_pose.roll").as_double();
			auto pitch = parent_node_->get_parameter(marker_name + ".initial_pose.pitch").as_double();
			auto yaw = parent_node_->get_parameter(marker_name + ".initial_pose.yaw").as_double();

			auto scale_x = parent_node_->get_parameter(marker_name + ".scale.x").as_double();
			auto scale_y = parent_node_->get_parameter(marker_name + ".scale.y").as_double();
			auto scale_z = parent_node_->get_parameter(marker_name + ".scale.z").as_double();

			auto r = parent_node_->get_parameter(marker_name + ".color.r").as_double();
			auto g = parent_node_->get_parameter(marker_name + ".color.g").as_double();
			auto b = parent_node_->get_parameter(marker_name + ".color.b").as_double();
			auto a = parent_node_->get_parameter(marker_name + ".color.a").as_double();

			// if (marker_type == "sphere")
			// {
			// 	visual_markers_[i].marker.type = visualization_msgs::msg::Marker::SPHERE;
			// }
			// else if (marker_type == "cube")
			// {
			// 	visual_markers_[i].marker.type = visualization_msgs::msg::Marker::CUBE;
			// }	

			// if (trajectory_marker_type == "line")
			// {
			// 	visual_markers_[i].trajectory_marker_type = visualization_msgs::msg::Marker::LINE_STRIP;
			// }
			// else if (trajectory_marker_type == "points")
			// {
			// 	visual_markers_[i].trajectory_marker_type = visualization_msgs::msg::Marker::POINTS;
			// }	

			// visual_markers_[i].marker.text = marker_name;

			// visual_markers_[i].marker.scale.x = scale_x;
			// visual_markers_[i].marker.scale.y = scale_y;
			// visual_markers_[i].marker.scale.z = scale_z;

			// visual_markers_[i].marker.color.r = r;
			// visual_markers_[i].marker.color.g = g;
			// visual_markers_[i].marker.color.b = b;
			// visual_markers_[i].marker.color.a = a;

			// visual_markers_[i].marker.pose = potbot_lib::utility::get_pose();

			visualization_msgs::msg::Marker marker_msg;
			marker_msg.text = marker_name;
			marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
			marker_msg.scale.x = scale_x;
			marker_msg.scale.y = scale_y;
			marker_msg.scale.z = scale_z;
			marker_msg.color.r = r;
			marker_msg.color.g = g;
			marker_msg.color.b = b;
			marker_msg.color.a = a;

			visualization_msgs::msg::InteractiveMarker int_marker;
			int_marker.header.frame_id = frame_id_global_;
			int_marker.header.stamp = parent_node_->get_clock()->now();
			int_marker.name = marker_name;
			int_marker.description = int_marker.name;
			int_marker.pose = potbot_lib::utility::get_pose(x,y,z,roll,pitch,yaw);

			movement_controller_.markers[0] = marker_msg;
			int_marker.controls.push_back(movement_controller_);

			VisualMarker vm;
			vm.marker = int_marker;
			vm.controller = int_marker;
			
			controllable_markers_.emplace(int_marker.name, vm);

			imsrv_->insert(int_marker, 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					this->changePosition(feedback);});

			visual_markers_[i].marker.pose = int_marker.pose;
			menu_handler_->apply(*imsrv_, int_marker.name);
		}

		imsrv_->applyChanges();
	}

	rcl_interfaces::msg::SetParametersResult InteractiveMarkerManager::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
	{
		RCLCPP_INFO(parent_node_->get_logger(), "パラメータ変更");
		auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
		results->successful = true;

		// int id = param.marker_id;
		// std::string visual_type = param.trajectory_marker_type;

		// u_int8_t type = visualization_msgs::Marker::LINE_STRIP;
		// if (visual_type == "line")
		// {
		// 	type = visualization_msgs::Marker::LINE_STRIP;
		// }
		// else if (visual_type == "points")
		// {
		// 	type = visualization_msgs::Marker::POINTS;
		// }

		// std::vector<size_t> ids;
		// if (id == -1)
		// {
		// 	for (size_t i = 0; i < visual_markers_.size(); i++)
		// 	{
		// 		ids.push_back(id);
		// 	}
		// }
		// else
		// {
		// 	ids.push_back(id);
		// }
		
		// for (const auto& i:ids)
		// {
		// 	if (i < visual_markers_.size())
		// 	{
		// 		visual_markers_[i].trajectory_recording = param.trajectory_recording;
		// 		if (visual_markers_[i].trajectory_recording)
		// 		{
		// 			visual_markers_[i].trajectory_marker_type = type;
		// 			visual_markers_[i].trajectory_interpolation_method = param.trajectory_interpolation_method;
		// 		}
		// 		else
		// 		{
		// 			visual_markers_[i].trajectory.clear();
		// 		}
		// 	}
		// }
		// publishMarkerTrajectory();

		for(const auto& param : parameters)
		{

			RCLCPP_INFO_STREAM(parent_node_->get_logger(), param.get_name());

			// if(param.get_name() == "param1")
			// {
			// 	member_param1_ = param.as_double(); // double型メンバ変数に代入
			// }
			// else if(param.get_name() == "param2")
			// {
			// 	// なんか変な時．例えば重力加速度にマイナスの値を代入しようとするなど．
			// 	if(!some_considion)
			// 	{
			// 		results->successful = false;
			// 		results->reason = "Wrong operation"; 
			// 		return *results;
			// 	}
			// }
		}
		
		// initializeMarker();

		return *results;
	}

	// void InteractiveMarkerManager::reconfigureCB(const potbot_lib::MarkerManagerConfig& param, uint32_t level)
	// {	
	// 	static bool first = true;

	// 	if (!first)
	// 	{
	// 		int id = param.marker_id;
	// 		std::string visual_type = param.trajectory_marker_type;

	// 		u_int8_t type = visualization_msgs::Marker::LINE_STRIP;
	// 		if (visual_type == "line")
	// 		{
	// 			type = visualization_msgs::Marker::LINE_STRIP;
	// 		}
	// 		else if (visual_type == "points")
	// 		{
	// 			type = visualization_msgs::Marker::POINTS;
	// 		}

	// 		std::vector<size_t> ids;
	// 		if (id == -1)
	// 		{
	// 			for (size_t i = 0; i < visual_markers_.size(); i++)
	// 			{
	// 				ids.push_back(id);
	// 			}
	// 		}
	// 		else
	// 		{
	// 			ids.push_back(id);
	// 		}
			
	// 		for (const auto& i:ids)
	// 		{
	// 			if (i < visual_markers_.size())
	// 			{
	// 				visual_markers_[i].trajectory_recording = param.trajectory_recording;
	// 				if (visual_markers_[i].trajectory_recording)
	// 				{
	// 					visual_markers_[i].trajectory_marker_type = type;
	// 					visual_markers_[i].trajectory_interpolation_method = param.trajectory_interpolation_method;
	// 				}
	// 				else
	// 				{
	// 					visual_markers_[i].trajectory.clear();
	// 				}
	// 			}
	// 		}
	// 		publishMarkerTrajectory();
	// 	}

	// 	first = false;
	// }

	void InteractiveMarkerManager::markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		// visualization_msgs::msg::InteractiveMarker int_marker;
		// if (imsrv_->get(feedback->marker_name, int_marker)) 
		// {
		// 	int id = -1;
		// 	for (size_t i = 0; i < interactive_marker_num_; i++)
		// 	{
		// 		if (int_marker.name == visual_markers_[i].marker.text)
		// 		{
		// 			id = i;
		// 			break;
		// 		}
		// 	}
		// 	if (id < 0)
		// 	{
		// 		return;
		// 	}
			
		// 	visual_markers_[id].marker.pose = feedback->pose;
		// 	visual_markers_[id].marker.header = feedback->header;

		// 	if (visual_markers_[id].trajectory_recording)
		// 	{
		// 		interpolateTrajectory(id);
		// 	}

		// 	size_t eid = feedback->menu_entry_id;
		// 	if (eid == 4)
		// 	{
		// 		int_marker.controls[0].markers[0].scale.x *= 2;
		// 	}
		// 	else if (eid == 5)
		// 	{
		// 		int_marker.controls[0].markers[0].scale.x *= 0.5;
		// 	}
		// 	else if (eid == 6)
		// 	{
		// 		int_marker.controls[0].markers[0].scale.y *= 2;
		// 	}
		// 	else if (eid == 7)
		// 	{
		// 		int_marker.controls[0].markers[0].scale.y *= 0.5;
		// 	}
		// 	else if (eid == 8)
		// 	{
		// 		int_marker.controls[0].markers[0].scale.x *= 2;
		// 		int_marker.controls[0].markers[0].scale.y *= 2;
		// 	}
		// 	else if (eid == 9)
		// 	{
		// 		int_marker.controls[0].markers[0].scale.x *= 0.5;
		// 		int_marker.controls[0].markers[0].scale.y *= 0.5;
		// 	}
		// 	else if (eid == 11)
		// 	{
		// 		int_marker.controls[0].markers[0].type = visualization_msgs::msg::Marker::CUBE;
		// 	}
		// 	else if (eid == 12)
		// 	{
		// 		int_marker.controls[0].markers[0].type = visualization_msgs::msg::Marker::SPHERE;
		// 	}

		// 	visual_markers_[id].marker.scale = int_marker.controls[0].markers[0].scale;
		// 	visual_markers_[id].marker.type = int_marker.controls[0].markers[0].type;

		// 	// 変更をサーバーに反映
		// 	imsrv_->insert(int_marker, boost::bind(&InteractiveMarkerManager::markerFeedback, this, boost::placeholders::_1));
		// 	imsrv_->applyChanges();
		// }
	}

	void InteractiveMarkerManager::editorChangeTo(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string mode)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			auto viz_marker_bak = int_marker.controls.front().markers.front();
			int_marker.controls.clear();
			if (mode == "position")
			{
				movement_controller_.markers[0] = viz_marker_bak;
				int_marker.controls.push_back(movement_controller_);
				int_marker.controls[0].markers[0].color.a = 1;
				int_marker.scale = std::max(std::max(std::max(
					int_marker.controls[0].markers[0].scale.x,
					int_marker.controls[0].markers[0].scale.y),
					int_marker.controls[0].markers[0].scale.z),1.0);
				controllable_markers_[int_marker.name].marker = int_marker;
				controllable_markers_[int_marker.name].controller = int_marker;
				imsrv_->insert(int_marker, 
					[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb) {
						this->changePosition(fb);});
				imsrv_->erase(int_marker.name + "_scale_controller");
			}
			else if (mode == "rotation")
			{
				rotation_controller_.markers[0] = viz_marker_bak;
				int_marker.controls.push_back(rotation_controller_);
				int_marker.controls.push_back(rotation_controller_axis_z_);
				int_marker.controls[0].markers[0].color.a = 1;
				int_marker.scale = std::max(std::max(std::max(
					int_marker.controls[0].markers[0].scale.x,
					int_marker.controls[0].markers[0].scale.y),
					int_marker.controls[0].markers[0].scale.z),1.0);
				controllable_markers_[int_marker.name].marker = int_marker;
				controllable_markers_[int_marker.name].controller = int_marker;
				imsrv_->insert(int_marker, 
					[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb) {
						this->changeRotation(fb);});
				imsrv_->erase(int_marker.name + "_scale_controller");
			}
			else if (mode == "scale")
			{
				decltype(controllable_markers_)::iterator it = controllable_markers_.find(int_marker.name);
				if (it != controllable_markers_.end()) 
				{
					scale_controller_.markers[0] = viz_marker_bak;
					int_marker.controls.push_back(scale_controller_);
					int_marker.controls[0].markers[0].color.a = 0.5;
					int_marker.scale = 1;
					

					visualization_msgs::msg::InteractiveMarker cont_marker = int_marker;
					cont_marker.name = int_marker.name + "_scale_controller";
					cont_marker.description = "";
					cont_marker.controls.clear();
					cont_marker.controls.push_back(scale_controller_axis_x_);
					cont_marker.controls.push_back(scale_controller_axis_y_);
					cont_marker.controls.push_back(scale_controller_axis_z_);

					cont_marker.scale = std::max(std::max(std::max(
						int_marker.controls[0].markers[0].scale.x,
						int_marker.controls[0].markers[0].scale.y),
						int_marker.controls[0].markers[0].scale.z),1.0);

					controllable_markers_[int_marker.name].marker = int_marker;
					controllable_markers_[int_marker.name].controller = cont_marker;

					imsrv_->insert(int_marker,
						[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb) {
							this->changeScale(fb);});
					imsrv_->insert(cont_marker, 
						[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb) {
							this->changeScale(fb);});
				}
			}
			
			imsrv_->applyChanges();
		}
	}

	void InteractiveMarkerManager::changePosition(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			controllable_markers_[int_marker.name].marker = int_marker;
			controllable_markers_[int_marker.name].controller = int_marker;
		}
	}

	void InteractiveMarkerManager::changeRotation(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		changePosition(feedback);
	}

	void InteractiveMarkerManager::changeScale(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			std::string name = "";
			for (const auto &cm:controllable_markers_)
			{
				if (cm.second.controller.name == int_marker.name)
				{
					name = cm.first;
					break;
				}
			}

			if (name == "")
				return;

			auto &marker = controllable_markers_[name].marker;
			auto &controller = controllable_markers_[name].controller;

			marker.controls[0].markers[0].scale.x += 0.2*(int_marker.pose.position.x - marker.pose.position.x);
			marker.controls[0].markers[0].scale.y += 0.2*(int_marker.pose.position.y - marker.pose.position.y);
			marker.controls[0].markers[0].scale.z += 0.2*(int_marker.pose.position.z - marker.pose.position.z);

			marker.controls[0].markers[0].scale.x = std::max(marker.controls[0].markers[0].scale.x, 0.01);
			marker.controls[0].markers[0].scale.y = std::max(marker.controls[0].markers[0].scale.y, 0.01);
			marker.controls[0].markers[0].scale.z = std::max(marker.controls[0].markers[0].scale.z, 0.01);

			controller.pose = marker.pose;

			imsrv_->insert(marker, 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb) {
					this->changeScale(fb);});
			imsrv_->insert(controller, 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb) {
					this->changeScale(fb);});
			imsrv_->applyChanges();
		}
	}

	void InteractiveMarkerManager::typeChangeTo(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int type)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			int_marker.controls[0].markers[0].type = type;

			auto name = int_marker.controls[0].name;
			if (name == "scale_controller_marker")
			{
				imsrv_->insert(int_marker, 
					[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb) {
						this->changeScale(fb);});
			}
			else
			{
				imsrv_->insert(int_marker, 
					[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &fb) {
						this->changePosition(fb);});
			}

			// controllable_markers_[int_marker.name].marker = int_marker;
			// controllable_markers_[int_marker.name].controller = int_marker;
			imsrv_->applyChanges();
		}
	}

	void InteractiveMarkerManager::interpolateTrajectory(size_t id)
	{
		// if (visual_markers_[id].trajectory.empty())
		// {
		// 	geometry_msgs::msg::PoseStamped p;
		// 	p.header = visual_markers_[id].marker.header;
		// 	p.pose = visual_markers_[id].marker.pose;
		// 	visual_markers_[id].trajectory.push_back(p);
		// }
		
		// double distance_to_pre = utility::get_distance(visual_markers_[id].marker.pose, visual_markers_[id].trajectory.back().pose);
		// if (distance_to_pre > 0.01)
		// {
		// 	geometry_msgs::msg::PoseStamped pose;
		// 	pose.header.frame_id = frame_id_global_;
		// 	pose.header.stamp = parent_node_->get_clock()->now();
		// 	pose.pose = visual_markers_[id].marker.pose;

		// 	if (distance_to_pre > 0.05)
		// 	{
		// 		std::vector<geometry_msgs::msg::PoseStamped> start_end(2);
		// 		start_end[0] = visual_markers_[id].trajectory.back();
		// 		start_end[1] = pose;
				
		// 		std::vector<Eigen::Vector2d> interp_vecs;
		// 		utility::to_mat(start_end,interp_vecs);
		// 		interpolate::linear(interp_vecs, int(distance_to_pre/0.05), interp_vecs);
		// 		utility::to_msg(interp_vecs,start_end);

		// 		RCLCPP_DEBUG(parent_node_->get_logger(), "distance_to_pre: %f, linear_interpolate_num: %d", distance_to_pre, (int)start_end.size());

		// 		for (size_t i = 1; i < start_end.size(); i++)
		// 		{
		// 			start_end[i].header = pose.header;
		// 			visual_markers_[id].trajectory.push_back(start_end[i]);
		// 		}
		// 	}
		// 	else
		// 	{
		// 		visual_markers_[id].trajectory.push_back(pose);
		// 	}

		// 	auto* points = &visual_markers_[id].trajectory;
		// 	size_t num = points->size();
		// 	size_t interpolate_num_limit = 50;
		// 	static size_t last_interpolated_index = 0;
		// 	if (num > 1)
		// 	{
		// 		std::vector<geometry_msgs::msg::PoseStamped> split_traj;
		// 		if (num > interpolate_num_limit)
		// 		{
		// 			for (size_t i = num - interpolate_num_limit; i < num; i++)
		// 			{
		// 				split_traj.push_back((*points)[i]);
		// 			}
		// 		}
		// 		else
		// 		{
		// 			split_traj = *points;
		// 		}
				
		// 		if (split_traj.size() > 1)
		// 		{
		// 			std::vector<Eigen::Vector2d> traj_vecs;
		// 			utility::to_mat(split_traj,traj_vecs);
		// 			if (visual_markers_[id].trajectory_interpolation_method == "spline")
		// 			{
		// 				interpolate::spline(traj_vecs, traj_vecs.size(), traj_vecs);
		// 			}
		// 			else if (visual_markers_[id].trajectory_interpolation_method == "bezier")
		// 			{
		// 				interpolate::bezier(traj_vecs, traj_vecs.size(), traj_vecs);
		// 			}
		// 			utility::to_msg(traj_vecs,split_traj);
		// 		}

		// 		if (num > interpolate_num_limit)
		// 		{
		// 			for (size_t i = 0; i < split_traj.size(); i++)
		// 			{
		// 				visual_markers_[id].trajectory[i+num-interpolate_num_limit] = split_traj[i];
		// 			}
		// 		}
		// 		else
		// 		{
		// 			visual_markers_[id].trajectory = split_traj;
		// 		}
		// 		RCLCPP_DEBUG(parent_node_->get_logger(), "interpolate size: %d, trajectories_[%d]_size: %d", (int)split_traj.size(), (int)id, (int)visual_markers_[id].trajectory.size());

		// 	}
		// 	publishMarkerTrajectory();
		// }
	}

	void InteractiveMarkerManager::publishMarkerTrajectory()
	{
		// visualization_msgs::msg::MarkerArray traj_marker;
		// for (const auto& vismark:visual_markers_)
		// {
		// 	if (vismark.trajectory_recording)
		// 	{
		// 		visualization_msgs::msg::Marker traj = vismark.marker;
		// 		traj.type = vismark.trajectory_marker_type;
		// 		traj.pose = potbot_lib::utility::get_pose();
		// 		traj.scale.x = 0.01;
		// 		traj.scale.y = 0.01;
		// 		traj.scale.z = 0.01;
		// 		utility::get_point(vismark.trajectory, traj.points);
		// 		traj_marker.markers.push_back(traj);
		// 	}
		// }
		// pub_marker_trajectory_->publish(traj_marker);
	}

	// bool directoryExists(const std::string &path) 
	// {
	// 	struct stat info;
	// 	if (stat(path.c_str(), &info) != 0) 
	// 	{
	// 		return false; // ディレクトリは存在しない
	// 	}
	// 	else if (info.st_mode & S_IFDIR) 
	// 	{
	// 		return true; // ディレクトリは存在する
	// 	} 
	// 	else 
	// 	{
	// 		return false; // パスはディレクトリではない
	// 	}
	// }

	// bool createDirectory(const std::string &path) 
	// {
	// 	if (mkdir(path.c_str(), 0755) != 0) 
	// 	{
	// 		if (errno == EEXIST) 
	// 		{
	// 			return true; // ディレクトリは既に存在する
	// 		} else 
	// 		{
	// 			return false; // ディレクトリの作成に失敗
	// 		}
	// 	}
	// 	return true; // ディレクトリの作成に成功
	// }

	// bool createDirectoriesRecursively(const std::string &path) 
	// {
	// 	size_t pos = 0;
	// 	std::string current_path;

	// 	while ((pos = path.find_first_of('/', pos)) != std::string::npos) 
	// 	{
	// 		current_path = path.substr(0, pos++);
	// 		if (!current_path.empty() && !directoryExists(current_path)) 
	// 		{
	// 			if (!createDirectory(current_path)) 
	// 			{
	// 				return false;
	// 			}
	// 		}
	// 	}

	// 	if (!directoryExists(path)) {
	// 		if (!createDirectory(path)) 
	// 		{
	// 			return false;
	// 		}
	// 	}

	// 	return true;
	// }

	int InteractiveMarkerManager::getMarkerId(std::string marker_name)
	{
		// for (size_t i = 0; i < visual_markers_.size(); i++)
		// {
		// 	if (visual_markers_[i].marker.text == marker_name)
		// 	{
		// 		return i;
		// 	}
		// }
		// return -1;
	}

	// bool InteractiveMarkerManager::serviceSaveMarkerTrajectory(potbot_lib::Save::Request &req, potbot_lib::Save::Response &resp)
	// {
	// 	std::string marker_name = req.save_target;
	// 	int id = 0;

	// 	if (marker_name == "")
	// 	{
	// 		marker_name = visual_markers_[0].marker.text;
	// 	}
	// 	else
	// 	{
	// 		id = getMarkerId(marker_name);
	// 		if (id == -1)
	// 		{
	// 			resp.success = false;
	// 			resp.message = "Invalid save_target: " + marker_name;
	// 			return false;
	// 		}
	// 	}
		
	// 	std::string csv_path = req.full_path;

	// 	if (csv_path == "")
	// 	{
	// 		csv_path = std::string(std::getenv("HOME")) + "/.ros/marker/trajectory/" + marker_name + ".csv";
	// 	}

	// 	size_t last_slash_pos = csv_path.find_last_of('/');
    // 	std::string directory_path = csv_path.substr(0, last_slash_pos);

	// 	// ディレクトリが存在するか確認し、存在しない場合は作成
	// 	if (!directoryExists(directory_path)) 
	// 	{
	// 		if (!createDirectoriesRecursively(directory_path)) 
	// 		{
	// 			ROS_ERROR_STREAM("Failed to create directory: " << directory_path);
	// 			resp.success = false;
	// 			resp.message = "Failed to create directory: " + directory_path;
	// 			return false;
	// 		}
	// 	}

	// 	std::ofstream csv_file(csv_path);

	// 	if (csv_file.is_open()) 
	// 	{
	// 		for (const auto& p : visual_markers_[id].trajectory) 
	// 		{
	// 			double roll,pitch,yaw;
	// 			utility::get_rpy(p.pose.orientation, roll, pitch, yaw);

	// 			csv_file	<< p.pose.position.x 	<< ","
	// 						<< p.pose.position.y 	<< ","
	// 						<< p.pose.position.z 	<< ","
	// 						<< roll 				<< ","
	// 						<< pitch 				<< ","
	// 						<< yaw 					<< "\n";
	// 		}
	// 		csv_file.close();

	// 		ROS_INFO_STREAM("Saved to: " << csv_path);
	// 		resp.success = true;
	// 		resp.message = "Saved to: " + csv_path;
	// 		return true;
	// 	} 
	// 	else 
	// 	{
	// 		ROS_ERROR_STREAM("Failed to open file: " << csv_path);
	// 		resp.success = false;
	// 		resp.message = "Failed to open file: " + csv_path;
	// 		return false;
	// 	}
		
	// }

	// bool InteractiveMarkerManager::serviceClearMarkerTrajectory(potbot_lib::Save::Request &req, potbot_lib::Save::Response &resp)
	// {
	// 	std::string marker_name = req.save_target;

	// 	if (marker_name == "")
	// 	{
	// 		std::string names = "";
	// 		for (auto& vm:visual_markers_)
	// 		{
	// 			vm.trajectory.clear();
	// 			names+=vm.marker.text + ", ";
	// 		}
	// 		names.pop_back();
	// 		names.pop_back();
	// 		ROS_INFO_STREAM("Clear trajectory: " << names);
	// 		resp.success = true;
	// 		resp.message = "Clear trajectory: " + names;
	// 	}
	// 	else
	// 	{
	// 		int id = getMarkerId(marker_name);
	// 		if (id == -1)
	// 		{
	// 			ROS_INFO_STREAM("Invalid target name: " << marker_name);
	// 			resp.success = false;
	// 			resp.message = "Invalid target name: " + marker_name;
	// 			return false;
	// 		}
	// 		else
	// 		{
	// 			visual_markers_[id].trajectory.clear();
	// 			ROS_INFO_STREAM("Clear trajectory: " << visual_markers_[id].marker.text);
	// 			resp.success = true;
	// 			resp.message = "Clear trajectory: " + visual_markers_[id].marker.text;
	// 		}
	// 	}
	// 	return true;
	// }

	std::vector<VisualMarker>* InteractiveMarkerManager::getVisualMarker()
	{
		return &visual_markers_;
	}
}