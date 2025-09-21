#include <potbot_ros/interactive_marker_manager.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace potbot_lib{

	InteractiveMarkerManager::InteractiveMarkerManager(std::string name, std::string node_namespace) : 
		rclcpp_lifecycle::LifecycleNode(name,node_namespace)
	{
		visualization_msgs::msg::Marker marker_msg;
		marker_msg.text = "marker";
		marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
		marker_msg.scale.x = 0.05;
		marker_msg.scale.y = 0.05;
		marker_msg.scale.z = 0.05;
		marker_msg.color.r = 0.7;
		marker_msg.color.g = 0;
		marker_msg.color.b = 0;
		marker_msg.color.a = 1;
		default_visual_marker_ = marker_msg;
	}

	CallbackReturn InteractiveMarkerManager::on_configure(const rclcpp_lifecycle::State &)
	{
		initializeParameter();

		initializeController();
		initializeMenu();
		initializeMarker(marker_file_);

		dyn_params_handler_ = this->add_on_set_parameters_callback(
			std::bind(&InteractiveMarkerManager::dynamicParametersCallback, this, std::placeholders::_1));
		
		is_initialized_ = true;
		RCLCPP_INFO(this->get_logger(), "InteractiveMarkerManager initialized");
		return CallbackReturn::SUCCESS;
	}

	CallbackReturn InteractiveMarkerManager::on_activate(const rclcpp_lifecycle::State &)
	{	
		if (!is_initialized_)
		{
			RCLCPP_WARN(this->get_logger(), 
				"InteractiveMarkerManager is not initialized. Call initialize() before activateServer()");
			return CallbackReturn::FAILURE;
		}
		
		initializeMarkerServer(controllable_markers_);

		RCLCPP_INFO(this->get_logger(), "Marker server activated");
		return CallbackReturn::SUCCESS;
	}

	void InteractiveMarkerManager::initializeParameter()
	{
		this->declare_parameter("frame_id_global", rclcpp::ParameterValue("map"));
		this->declare_parameter("marker_yaml_path", rclcpp::ParameterValue("interactive_markers.yaml"));

		frame_id_global_ = this->get_parameter("frame_id_global").as_string();
		marker_file_ = this->get_parameter("marker_yaml_path").as_string();
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

	void InteractiveMarkerManager::initializeMenu()
	{
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
		
		entry_handle_add_ = menu_handler_->insert("add",
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->duplicateMarker(feedback);});

		entry_handle_delete_ = menu_handler_->insert("delete",
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->deleteMarker(feedback);});

		entry_handle_save_ = menu_handler_->insert("save");

		menu_handler_->insert( entry_handle_save_, "marker pose", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->saveMarker(feedback);});
	}

	void InteractiveMarkerManager::initializeMarker(std::string yaml_path, bool set_default)
	{
		try 
		{
			YAML::Node root = YAML::LoadFile(yaml_path);
			if (root["markers"]) 
			{
				std::vector<visualization_msgs::msg::InteractiveMarker> int_markers;
				for (const auto& node : root["markers"]) 
				{
					std::string name = node["name"].as<std::string>();
					std::string type = node["type"].as<std::string>();

					auto x = node["pose"]["position"]["x"].as<double>();
					auto y = node["pose"]["position"]["y"].as<double>();
					auto z = node["pose"]["position"]["z"].as<double>();
					auto roll = node["pose"]["rotation"]["roll"].as<double>();
					auto pitch = node["pose"]["rotation"]["pitch"].as<double>();
					auto yaw = node["pose"]["rotation"]["yaw"].as<double>();

					auto scale_x = node["scale"]["x"].as<double>();
					auto scale_y = node["scale"]["y"].as<double>();
					auto scale_z = node["scale"]["z"].as<double>();

					auto r = node["color"]["r"].as<double>();
					auto g = node["color"]["g"].as<double>();
					auto b = node["color"]["b"].as<double>();
					auto a = node["color"]["a"].as<double>();
							
					visualization_msgs::msg::Marker marker_msg;
					marker_msg.scale.x = scale_x;
					marker_msg.scale.y = scale_y;
					marker_msg.scale.z = scale_z;
					marker_msg.color.r = r;
					marker_msg.color.g = g;
					marker_msg.color.b = b;
					marker_msg.color.a = a;

					if (type == "sphere")
						marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
					else if (type == "cube")
						marker_msg.type = visualization_msgs::msg::Marker::CUBE;

					addMarker(name, marker_msg, Pose(x,y,z,roll,pitch,yaw));
				}

				RCLCPP_INFO(this->get_logger(), "Base marker loaded: %s", yaml_path.c_str());
			}
		} 
		catch (const std::exception& e) 
		{
			RCLCPP_INFO(this->get_logger(), "Failed to base marker load marker yaml: %s", e.what());

			if (set_default)
			{
				addMarker(default_visual_marker_.text);
				RCLCPP_INFO(this->get_logger(), "Set to default");
			}
		}
	}

	void InteractiveMarkerManager::initializeMarkerServer(
		const std::map<std::string, VisualMarker> &markers)
	{
		std::string marker_server_name = "markers";
		imsrv_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
			this->get_namespace() + '/' + marker_server_name, this);

		for (const auto & m:markers)
		{
			imsrv_->insert(m.second.marker, 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
					changePosition(feedback);});
			menu_handler_->apply(*imsrv_, m.second.marker.name);
		}
		imsrv_->applyChanges();
	}

	rcl_interfaces::msg::SetParametersResult InteractiveMarkerManager::dynamicParametersCallback(
		std::vector<rclcpp::Parameter> parameters)
	{
		RCLCPP_INFO(this->get_logger(), "parameter changed");
		auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
		results->successful = true;

		for(const auto& param : parameters)
		{
			RCLCPP_INFO_STREAM(this->get_logger(), param.get_name());

			if(param.get_name() == "marker_yaml_path")
			{
				marker_file_ = param.as_string();
				initializeMarker(marker_file_, false);
				initializeMarkerServer(controllable_markers_);
			}
			else if(param.get_name() == "frame_id_global")
			{
				frame_id_global_ = param.as_string();
			}
		}

		return *results;
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
					[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
						changePosition(feedback);});
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
					[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
						changeRotation(feedback);});
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

	YAML::Node InteractiveMarkerManager::saveMarker(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		YAML::Node root;
		YAML::Node node = root["markers"];

		for (const auto& cm:controllable_markers_)
		{
			const auto &marker = cm.second.marker;
			YAML::Node child;
			child["name"] = cm.first;

			auto type = marker.controls[0].markers[0].type;
			if (type == visualization_msgs::msg::Marker::SPHERE)
				child["type"] = "sphere";
			else if (type == visualization_msgs::msg::Marker::CUBE)
				child["type"] = "cube";

			child["pose"]["position"]["x"] = marker.pose.position.x;
			child["pose"]["position"]["y"] = marker.pose.position.y;
			child["pose"]["position"]["z"] = marker.pose.position.z;

			double r,p,y;
			tf2::getEulerYPR(marker.pose.orientation, r,p,y);
			child["pose"]["rotation"]["roll"] = r;
			child["pose"]["rotation"]["pitch"] = p;
			child["pose"]["rotation"]["yaw"] = y;

			child["scale"]["x"] = marker.controls[0].markers[0].scale.x;
			child["scale"]["y"] = marker.controls[0].markers[0].scale.y;
			child["scale"]["z"] = marker.controls[0].markers[0].scale.z;
			child["color"]["r"] = marker.controls[0].markers[0].color.r;
			child["color"]["g"] = marker.controls[0].markers[0].color.g;
			child["color"]["b"] = marker.controls[0].markers[0].color.b;
			child["color"]["a"] = marker.controls[0].markers[0].color.a;

			node.push_back(child);
		}

		// yamlファイル名を決定（例: marker名.yaml）
		std::string yaml_path = marker_file_;
		try {
			std::ofstream ofs(yaml_path);
			ofs << root;
			ofs.close();
			RCLCPP_INFO(this->get_logger(), "Saved base marker to %s", yaml_path.c_str());
			return root;
		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Failed to write yaml: %s", e.what());
			return YAML::Node();
		}
	}

	std::string InteractiveMarkerManager::duplicateMarker(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			auto viz_marker = int_marker.controls[0].markers[0];
			viz_marker.color.a = 1;

			std::string new_marker_name = getCopyName(int_marker.name);

			Pose new_pose = 
				utility::get_pose(getMarkerPose(int_marker.name)) + Pose(0.3,0.3);

			addMarker(new_marker_name, viz_marker, new_pose);
			initializeMarkerServer(controllable_markers_);
			RCLCPP_INFO(this->get_logger(), "[%s] added", new_marker_name.c_str());
			return new_marker_name;
		}
		return std::string();
	}

	std::string InteractiveMarkerManager::getCopyName(std::string original_name)
	{
		py::string str = original_name;
		std::vector<py::string> strs;
		str.split(strs, "_");

		std::string new_name = original_name;
		if (strs.size() < 2)
		{
			new_name += "_0";
		}
		else
		{
			try 
			{
				auto num = std::stoi(strs.back())+1;
				new_name = "";
				for (int i=0; i<strs.size()-1; i++)
				{
					new_name += strs[i] + "_";
				}
				new_name += std::to_string(num); 
			} 
			catch (const std::invalid_argument& e) 
			{
				new_name += "_0";
			}
		}

		
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(new_name, int_marker))
		{
			return getCopyName(new_name);
		}
		else
		{
			return new_name;
		}
	}

	void InteractiveMarkerManager::deleteMarker(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			controllable_markers_.erase(int_marker.name);
			imsrv_->erase(int_marker.name);
			imsrv_->applyChanges();
			RCLCPP_INFO(this->get_logger(), "[%s] deleted", int_marker.name.c_str());
		}
	}

	void InteractiveMarkerManager::registerFeedback(std::string marker_name,
		const interactive_markers::InteractiveMarkerServer::FeedbackCallback &feedbck_func)
	{
		const auto cm = controllable_markers_[marker_name];
		imsrv_->insert(cm.marker,feedbck_func);
		menu_handler_->apply(*imsrv_, cm.marker.name);
		imsrv_->applyChanges();
	}

	void InteractiveMarkerManager::addMarker(std::string name, const Pose &init_pose)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		int_marker.header.frame_id = frame_id_global_;
		int_marker.header.stamp = this->get_clock()->now();
		int_marker.name = name;
		int_marker.description = int_marker.name;
		int_marker.pose = potbot_lib::utility::get_pose(init_pose);

		visualization_msgs::msg::Marker marker_msg = default_visual_marker_;
		marker_msg.text = name;

		visualization_msgs::msg::InteractiveMarkerControl controller = movement_controller_;

		controller.markers[0] = marker_msg;
		int_marker.controls.push_back(controller);

		controllable_markers_[name].marker = int_marker;
		controllable_markers_[name].controller = int_marker;
	}

	void InteractiveMarkerManager::addMarker(std::string name,
		const visualization_msgs::msg::Marker &vis_marker, const Pose &init_pose)
	{
		addMarker(name, init_pose);
		controllable_markers_[name].marker.controls[0].markers[0] = vis_marker;
		controllable_markers_[name].controller.controls[0].markers[0] = vis_marker;
	}

	geometry_msgs::msg::Pose InteractiveMarkerManager::getMarkerPose(std::string name)
	{
		return controllable_markers_[name].marker.pose;
	}
}