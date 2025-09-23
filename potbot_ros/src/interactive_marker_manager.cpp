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
		std::string ros_distro = std::getenv("ROS_DISTRO");
		this->declare_parameter("mesh_resource_files", rclcpp::ParameterValue(std::vector<std::string>{
			"file:///opt/ros/" + ros_distro + "/share/rviz_default_plugins/test_meshes/pr2-base.dae",
			"package://rviz_default_plugins/test_meshes/pr2-base.dae"}));

		frame_id_global_ = this->get_parameter("frame_id_global").as_string();
		marker_file_ = this->get_parameter("marker_yaml_path").as_string();
		mesh_resource_files_ = this->get_parameter("mesh_resource_files").as_string_array();
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

		visualization_msgs::msg::Marker scale_arrow_x;
		scale_arrow_x.type = visualization_msgs::msg::Marker::ARROW;
		scale_arrow_x.scale.x = 1;
		scale_arrow_x.scale.y = 0.05;
		scale_arrow_x.scale.z = 0.05;
		scale_arrow_x.color = color::get_msg("red");
		scale_arrow_x.color.a = 0.3;
		scale_arrow_x.pose = potbot_lib::utility::get_pose(0,0,0,0,0,0);

		visualization_msgs::msg::Marker scale_arrow_y = scale_arrow_x;
		scale_arrow_y.color = color::get_msg("green");
		scale_arrow_y.color.a = 0.3;
		scale_arrow_y.pose = potbot_lib::utility::get_pose(0,0,0,0,0,M_PI_2);

		visualization_msgs::msg::Marker scale_arrow_z = scale_arrow_x;
		scale_arrow_z.color = color::get_msg("blue");
		scale_arrow_z.color.a = 0.3;
		scale_arrow_z.pose = potbot_lib::utility::get_pose(0,0,0,0,-M_PI_2,0);

		visualization_msgs::msg::Marker scale_arrow_xyz = scale_arrow_x;
		scale_arrow_xyz.color = color::get_msg("white");
		scale_arrow_xyz.color.a = 0.3;
		scale_arrow_xyz.pose = potbot_lib::utility::get_pose(0,0,0,0,-M_PI_4,M_PI_4);

		movement_controller_.name = "movement_controller_marker";
		movement_controller_.orientation = potbot_lib::utility::get_quat(0,-M_PI_2,0);
		movement_controller_.always_visible = true;
		movement_controller_.markers.push_back(move_marker);
		movement_controller_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

		movement_controller_axis_z_.name = "movement_controller_axis_z";
		movement_controller_axis_z_.orientation = potbot_lib::utility::get_quat(0,M_PI_2,0);
		movement_controller_axis_z_.always_visible = true;
		movement_controller_axis_z_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

		rotation_controller_axis_x_.name = "rotation_controller_axis_x";
		rotation_controller_axis_x_.orientation = potbot_lib::utility::get_quat(0,0,0);
		rotation_controller_axis_x_.always_visible = true;
		rotation_controller_axis_x_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

		rotation_controller_axis_y_.name = "rotation_controller_axis_y";
		rotation_controller_axis_y_.orientation = potbot_lib::utility::get_quat(0,0,M_PI_2);
		rotation_controller_axis_y_.always_visible = true;
		rotation_controller_axis_y_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

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
		scale_controller_axis_x_.markers.push_back(scale_arrow_x);

		scale_controller_axis_y_.name = "scale_controller_axis_y";
		scale_controller_axis_y_.orientation = potbot_lib::utility::get_quat(0,0,M_PI_2);
		scale_controller_axis_y_.always_visible = true;
		scale_controller_axis_y_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
		scale_controller_axis_y_.markers.push_back(scale_arrow_y);

		scale_controller_axis_z_.name = "scale_controller_axis_z";
		scale_controller_axis_z_.orientation = potbot_lib::utility::get_quat(0,M_PI_2,0);
		scale_controller_axis_z_.always_visible = true;
		scale_controller_axis_z_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
		scale_controller_axis_z_.markers.push_back(scale_arrow_z);

		scale_controller_axis_xyz_.name = "scale_controller_axis_xyz";
		scale_controller_axis_xyz_.orientation = potbot_lib::utility::get_quat(0,-M_PI_4,M_PI_4);
		scale_controller_axis_xyz_.always_visible = true;
		scale_controller_axis_xyz_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
		scale_controller_axis_xyz_.markers.push_back(scale_arrow_xyz);

		scale_controller_.name = "scale_controller_marker";
		scale_controller_.always_visible = true;
		scale_controller_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
		scale_controller_.markers.push_back(move_marker);

	}

	void InteractiveMarkerManager::initializeMenu()
	{
		menu_handler_ = std::make_shared<interactive_markers::MenuHandler>();

		entry_handles_["edit"] = menu_handler_->insert("edit");

		auto position_entry = menu_handler_->insert(entry_handles_["edit"], "position");

		menu_handler_->insert(position_entry, "plane" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->editorChangeTo(feedback, "position_plane");});
		
		menu_handler_->insert(position_entry, "height" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->editorChangeTo(feedback, "position_z");});

		menu_handler_->insert(position_entry, "reset height" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->resetPositionZ(feedback);});
		
		auto rotation_entry = menu_handler_->insert(entry_handles_["edit"], "rotation");

		menu_handler_->insert(rotation_entry, "yaw" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->editorChangeTo(feedback, "rotation_yaw");});

		menu_handler_->insert(rotation_entry, "3D" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->editorChangeTo(feedback, "rotation_3d");});
		
		menu_handler_->insert(rotation_entry, "reset" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->resetRotation(feedback);});
		
		auto scale_entry = menu_handler_->insert(entry_handles_["edit"], "scale");

		menu_handler_->insert(scale_entry, "3D" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->editorChangeTo(feedback, "scale");});

		menu_handler_->insert(scale_entry, "reset" , 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->resetScale(feedback);});

		auto type_entry = menu_handler_->insert(entry_handles_["edit"], "shape");

		menu_handler_->insert(type_entry, "cube", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->typeChangeTo(feedback, visualization_msgs::msg::Marker::CUBE);});

		menu_handler_->insert(type_entry, "sphere", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->typeChangeTo(feedback, visualization_msgs::msg::Marker::SPHERE);});
		
		menu_handler_->insert(type_entry, "cylinder", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->typeChangeTo(feedback, visualization_msgs::msg::Marker::CYLINDER);});

		menu_handler_->insert(type_entry, "arrow", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->typeChangeTo(feedback, visualization_msgs::msg::Marker::ARROW);});
		
		auto mesh_entry = menu_handler_->insert(type_entry, "mesh");

		for (const auto &mesh_file:mesh_resource_files_)
			menu_handler_->insert(mesh_entry, mesh_file, 
				[this,mesh_file](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					this->typeChangeTo(feedback, visualization_msgs::msg::Marker::MESH_RESOURCE, mesh_file);});

		auto color_entry = menu_handler_->insert(entry_handles_["edit"], "color");

		menu_handler_->insert(color_entry, "red", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, color::get_msg(color::RED));});

		menu_handler_->insert(color_entry, "green", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, color::get_msg(color::GREEN));});

		menu_handler_->insert(color_entry, "blue", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, color::get_msg(color::BLUE));});
		
		menu_handler_->insert(color_entry, "yellow", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, color::get_msg(color::YELLOW));});

		menu_handler_->insert(color_entry, "light blue", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, color::get_msg(color::LIGHT_BLUE));});

		menu_handler_->insert(color_entry, "purple", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, color::get_msg(color::PURPLE));});

		menu_handler_->insert(color_entry, "white", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, color::get_msg(color::WHITE));});

		menu_handler_->insert(color_entry, "black", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, color::get_msg(color::BLACK));});

		menu_handler_->insert(color_entry, "none", 
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->colorChangeTo(feedback, std_msgs::msg::ColorRGBA());});
		
		entry_handles_["add"] = menu_handler_->insert("add");

		menu_handler_->insert(entry_handles_["add"], "marker",
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->duplicateMarker(feedback);});
		
		entry_handles_["delete"] = menu_handler_->insert("delete");
				
		menu_handler_->insert(entry_handles_["delete"], "marker",
			[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
				this->deleteMarker(feedback);});

		entry_handles_["save"] = menu_handler_->insert("save");

		menu_handler_->insert( entry_handles_["save"], "marker pose", 
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
				for (const auto& node : root["markers"]) 
				{
					auto vm = getVisualMarker(node);
					addMarker(vm);
				}

				RCLCPP_INFO(this->get_logger(), "Base marker loaded: %s", yaml_path.c_str());
			}
		} 
		catch (const std::exception& e) 
		{
			RCLCPP_INFO(this->get_logger(), "Failed to base marker load. marker yaml: %s", e.what());

			if (set_default)
			{
				addMarker(default_visual_marker_.text);
				RCLCPP_INFO(this->get_logger(), "Set to default");
			}
		}
	}

	VisualMarker InteractiveMarkerManager::getVisualMarker(const YAML::Node &yaml_node)
	{
		std::string name = yaml_node["name"].as<std::string>();
		std::string type = yaml_node["type"].as<std::string>();

		auto x = yaml_node["pose"]["position"]["x"].as<double>();
		auto y = yaml_node["pose"]["position"]["y"].as<double>();
		auto z = yaml_node["pose"]["position"]["z"].as<double>();
		auto roll = yaml_node["pose"]["rotation"]["roll"].as<double>();
		auto pitch = yaml_node["pose"]["rotation"]["pitch"].as<double>();
		auto yaw = yaml_node["pose"]["rotation"]["yaw"].as<double>();

		auto scale_x = yaml_node["scale"]["x"].as<double>();
		auto scale_y = yaml_node["scale"]["y"].as<double>();
		auto scale_z = yaml_node["scale"]["z"].as<double>();

		auto r = yaml_node["color"]["r"].as<double>();
		auto g = yaml_node["color"]["g"].as<double>();
		auto b = yaml_node["color"]["b"].as<double>();
		auto a = yaml_node["color"]["a"].as<double>();
				
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
		else if (type == "cylinder")
			marker_msg.type = visualization_msgs::msg::Marker::CYLINDER;
		else if (type == "arrow")
			marker_msg.type = visualization_msgs::msg::Marker::ARROW;
		else if (type == "mesh")
		{
			marker_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
			marker_msg.mesh_resource = yaml_node["mesh_resource"].as<std::string>();
			marker_msg.mesh_use_embedded_materials = true;
		}
		
		return getVisualMarker(name, marker_msg, Pose(x,y,z,roll,pitch,yaw));
	}

	VisualMarker InteractiveMarkerManager::getVisualMarker(std::string name, const Pose &init_pose)
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

		VisualMarker vm;
		vm.marker = int_marker;
		vm.controller = int_marker;
		return vm;
	}

	VisualMarker InteractiveMarkerManager::getVisualMarker(
		std::string name, const visualization_msgs::msg::Marker &vis_marker, const Pose &init_pose)
	{
		auto vm = getVisualMarker(name, init_pose);
		vm.marker.controls[0].markers[0] = vis_marker;
		vm.controller.controls[0].markers[0] = vis_marker;
		return vm;
	}

	void InteractiveMarkerManager::initializeMarkerServer(
		const std::map<std::string, VisualMarker> &markers, const std::vector<std::string> &marker_with_controller)
	{
		std::string marker_server_name = "markers";
		imsrv_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
			this->get_namespace() + '/' + marker_server_name, this);

		for (const auto & m:markers)
		{
			imsrv_->insert(m.second.marker, 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
					changePosition(feedback);});
			
			if (utility::contains(m.first, marker_with_controller))
					imsrv_->insert(m.second.controller, 
						[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
							changeScale(feedback);});
			
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
			RCLCPP_INFO_STREAM(this->get_logger(), "\t" + param.get_name());

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
			else if(param.get_name() == "mesh_resource_files")
			{

				mesh_resource_files_ = this->get_parameter("mesh_resource_files").as_string_array();
				initializeMenu();
				initializeMarkerServer(controllable_markers_);
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
			if (mode == "position_plane")
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
			else if (mode == "position_z")
			{
				auto tmp_controller = movement_controller_axis_z_;
				tmp_controller.markers.clear();
				tmp_controller.markers.push_back(viz_marker_bak);
				int_marker.controls.push_back(tmp_controller);
				int_marker.controls.push_back(movement_controller_axis_z_);
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
			else if (mode == "rotation_yaw")
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
			else if (mode == "rotation_3d")
			{
				rotation_controller_.markers[0] = viz_marker_bak;
				auto tmp_controller = rotation_controller_;
				tmp_controller.orientation = potbot_lib::utility::get_quat(0,0,0);
				tmp_controller.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
				int_marker.controls.push_back(tmp_controller);
				int_marker.controls.push_back(rotation_controller_axis_x_);
				int_marker.controls.push_back(rotation_controller_axis_y_);
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
				if (utility::contains(int_marker.name, controllable_markers_)) 
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
					cont_marker.controls.push_back(scale_controller_axis_xyz_);

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

	void InteractiveMarkerManager::resetPositionZ(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			int_marker.pose.position.z = 0;
			controllable_markers_[int_marker.name].marker = int_marker;
			controllable_markers_[int_marker.name].controller = int_marker;
			initializeMarkerServer(controllable_markers_);
			changePosition(feedback);
		}
	}

	void InteractiveMarkerManager::changeRotation(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		changePosition(feedback);
	}

	void InteractiveMarkerManager::resetRotation(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			int_marker.pose.orientation = utility::get_quat(0,0,0);
			controllable_markers_[int_marker.name].marker = int_marker;
			controllable_markers_[int_marker.name].controller = int_marker;
			initializeMarkerServer(controllable_markers_);
			changeRotation(feedback);
		}
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

			marker.controls.front().markers.front().scale.x
				+= 0.2*(int_marker.pose.position.x - marker.pose.position.x);
			marker.controls.front().markers.front().scale.y
				+= 0.2*(int_marker.pose.position.y - marker.pose.position.y);
			marker.controls.front().markers.front().scale.z
				+= 0.2*(int_marker.pose.position.z - marker.pose.position.z);

			marker.controls.front().markers.front().scale.x
				= std::max(marker.controls.front().markers.front().scale.x, 0.01);
			marker.controls.front().markers.front().scale.y
				= std::max(marker.controls.front().markers.front().scale.y, 0.01);
			marker.controls.front().markers.front().scale.z
				= std::max(marker.controls.front().markers.front().scale.z, 0.01);

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

	void InteractiveMarkerManager::resetScale(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			int_marker.controls.front().markers.front().scale.x = 0.05;
			int_marker.controls.front().markers.front().scale.y = 0.05;
			int_marker.controls.front().markers.front().scale.z = 0.05;
			controllable_markers_[int_marker.name].marker = int_marker;
			controllable_markers_[int_marker.name].controller = int_marker;
			initializeMarkerServer(controllable_markers_);
		}
	}

	void InteractiveMarkerManager::typeChangeTo(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int type, std::string mesh_resource)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			// int_marker.controls.front().markers.front().type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
			// int_marker.controls.front().markers.front().scale.x = 0.5;
			// int_marker.controls.front().markers.front().points.push_back(utility::get_point(0,0,0));
			// int_marker.controls.front().markers.front().points.push_back(utility::get_point(cos(M_PI/3),sin(M_PI/3),0));
			// int_marker.controls.front().markers.front().points.push_back(utility::get_point(-cos(M_PI/3),sin(M_PI/3),0));
			// int_marker.controls.front().markers.front().colors.push_back(color::get_msg("green"));

			int_marker.controls.front().markers.front().type = type;
			if (type == visualization_msgs::msg::Marker::MESH_RESOURCE)
			{
				int_marker.controls.front().markers.front().mesh_use_embedded_materials = true;
				int_marker.controls.front().markers.front().mesh_resource = mesh_resource;
				int_marker.controls.front().markers.front().scale.x = 1;
				int_marker.controls.front().markers.front().scale.y = 1;
				int_marker.controls.front().markers.front().scale.z = 1;
				int_marker.controls.front().markers.front().color = std_msgs::msg::ColorRGBA();
			}

			controllable_markers_[int_marker.name].marker = int_marker;

			auto name = int_marker.controls.front().name;
			if (name == "scale_controller_marker")
				initializeMarkerServer(controllable_markers_, {int_marker.name});
			else
				initializeMarkerServer(controllable_markers_);
		}
	}

	void InteractiveMarkerManager::colorChangeTo(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std_msgs::msg::ColorRGBA color)
	{
		visualization_msgs::msg::InteractiveMarker int_marker;
		if (imsrv_->get(feedback->marker_name, int_marker))
		{
			auto name = int_marker.controls[0].name;
			if (name == "scale_controller_marker")
			{
				int_marker.controls.front().markers.front().color.r = color.r;
				int_marker.controls.front().markers.front().color.g = color.g;
				int_marker.controls.front().markers.front().color.b = color.b;

				controllable_markers_[int_marker.name].marker.controls.front().markers.front().color
					= int_marker.controls.front().markers.front().color;
				initializeMarkerServer(controllable_markers_, {int_marker.name});
			}
			else
			{
				controllable_markers_[int_marker.name].marker.controls.front().markers.front().color = color;
				initializeMarkerServer(controllable_markers_);
			}
		}
	}

	std::string InteractiveMarkerManager::getMarkerName(std::string controller_name)
	{
		for (const auto &cm:controllable_markers_)
			if (cm.second.controller.name == controller_name)
				return cm.second.marker.name;
		return std::string();
	}

	YAML::Node InteractiveMarkerManager::saveMarker(
		const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
	{
		YAML::Node root;
		YAML::Node node = root["markers"];

		for (const auto& cm:controllable_markers_)
			node.push_back(getYamlNode(cm.second));

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

	YAML::Node InteractiveMarkerManager::getYamlNode(const VisualMarker &visual_marker)
	{
		const auto &marker = visual_marker.marker;
		YAML::Node node;
		node["name"] = marker.name;

		auto type = marker.controls[0].markers[0].type;
		if (type == visualization_msgs::msg::Marker::SPHERE)
			node["type"] = "sphere";
		else if (type == visualization_msgs::msg::Marker::CUBE)
			node["type"] = "cube";
		else if (type == visualization_msgs::msg::Marker::CYLINDER)
			node["type"] = "cylinder";
		else if (type == visualization_msgs::msg::Marker::ARROW)
			node["type"] = "arrow";
		else if (type == visualization_msgs::msg::Marker::MESH_RESOURCE)
		{
			node["type"] = "mesh";
			node["mesh_resource"] = marker.controls.front().markers.front().mesh_resource;
		}

		node["pose"]["position"]["x"] = marker.pose.position.x;
		node["pose"]["position"]["y"] = marker.pose.position.y;
		node["pose"]["position"]["z"] = marker.pose.position.z;

		double r,p,y;
		tf2::getEulerYPR(marker.pose.orientation, r,p,y);
		node["pose"]["rotation"]["roll"] = r;
		node["pose"]["rotation"]["pitch"] = p;
		node["pose"]["rotation"]["yaw"] = y;

		node["scale"]["x"] = marker.controls[0].markers[0].scale.x;
		node["scale"]["y"] = marker.controls[0].markers[0].scale.y;
		node["scale"]["z"] = marker.controls[0].markers[0].scale.z;
		node["color"]["r"] = marker.controls[0].markers[0].color.r;
		node["color"]["g"] = marker.controls[0].markers[0].color.g;
		node["color"]["b"] = marker.controls[0].markers[0].color.b;
		node["color"]["a"] = marker.controls[0].markers[0].color.a;

		return node;
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

	void InteractiveMarkerManager::setMenuVisibles(bool visible)
	{
		for (auto &h:entry_handles_)
			menu_handler_->setVisible(h.second, visible);
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
		controllable_markers_[name] = getVisualMarker(name, init_pose);
	}

	void InteractiveMarkerManager::addMarker(
		std::string name, const visualization_msgs::msg::Marker &vis_marker, const Pose &init_pose)
	{
		controllable_markers_[name] = getVisualMarker(name, vis_marker, init_pose);
	}

	void InteractiveMarkerManager::addMarker(std::string name, const VisualMarker &visual_marker)
	{
		controllable_markers_[name] = visual_marker;
	}

	void InteractiveMarkerManager::addMarker(VisualMarker &visual_marker)
	{
		addMarker(visual_marker.marker.name, visual_marker);
	}

	geometry_msgs::msg::Pose InteractiveMarkerManager::getMarkerPose(std::string name)
	{
		return controllable_markers_[name].marker.pose;
	}
}