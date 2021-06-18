#include "gates_path_generator.hpp"


void GatesPathGenerator::ownSetUp()
{

	ros_utils_lib::getPrivateParam<std::string>("~namespace"					, n_space_						,"drone1");
	ros_utils_lib::getPrivateParam<std::string>("~gates_topic"	    			, gates_topic_					,"gate_detector/detected_poses");
	ros_utils_lib::getPrivateParam<std::string>("~traj_waypoints_topic"		    , traj_waypoints_topic_			,"motion_reference/waypoints");
	ros_utils_lib::getPrivateParam<std::string>("~self_localization_pose_topic" , self_localization_pose_topic_ ,"self_localization/pose");

	// gates_info_.clear();
	gates_info_.emplace_back(2);
	gates_info_.emplace_back(1);

	gates_sub_ = nh_.subscribe("/" + n_space_ + "/" + gates_topic_ ,1, &GatesPathGenerator::gatesCallback,this);
	pose_sub_ = nh_.subscribe("/" + n_space_ + "/" + self_localization_pose_topic_ ,1, &GatesPathGenerator::poseCallback,this);
	gen_traj_sub_ = nh_.subscribe("/gen_manual_traj" ,1, &GatesPathGenerator::getManualTrajCallback,this);
	traj_waypoints_pub_ = nh_.advertise <aerostack_msgs::TrajectoryWaypoints>("/" + n_space_ + "/" + traj_waypoints_topic_,1);
	
	// gates_vector_ = std::vector<geometry_msgs::PoseStamped>(N_GATES);
	gate2_pub_ = nh_.advertise <geometry_msgs::PoseStamped>("/" + n_space_ + "/" + "debug/pose_aruco2",1);
	gate1_pub_ = nh_.advertise <geometry_msgs::PoseStamped>("/" + n_space_ + "/" + "debug/pose_aruco1",1);

	tf2_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf2_buffer_);


}

geometry_msgs::PoseStamped movePose(const geometry_msgs::PoseStamped& pose, float dx,float dy, float dz){
	auto out_pose = pose;
	out_pose.pose.position.x += dx; 
	out_pose.pose.position.y += dy; 
	out_pose.pose.position.z += dz;
	return out_pose;
}

/*****************************************/

void appendSegmentToPath(segmentType& _segment,aerostack_msgs::TrajectoryWaypoints& path){
	auto time = ros::Time::now();
	for (auto& vector:_segment){
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "odom";
		pose.header.stamp = time;
		pose.pose.position.x = vector(0);
		pose.pose.position.y = vector(1);
		pose.pose.position.z = vector(2);
		pose.pose.orientation.w = 1.0f;
		path.poses.emplace_back(pose);
	}
}


void GatesPathGenerator::genTraj(){

	static ros::Time prev_time = ros::Time::now();
	static bool trajectory_has_changes = false;
	
	if (!gates_has_changes) return;
	gates_has_changes = false;

	bool generate_new_path = false;
	bool is_inside_a_gate =false; 

	for(auto& gate:gates_info_){
		generate_new_path |= gate.recalculateSegment(estimated_pose_);
		is_inside_a_gate |= gate.is_inside;
	}
	// if (!is_inside_a_gate && (ros::Time::now()-prev_time).toSec()>refresh_rate_){
	trajectory_has_changes |= generate_new_path; 
	if (trajectory_has_changes && !is_inside_a_gate && (ros::Time::now()-prev_time).toSec()>refresh_rate_){
		trajectory_has_changes = false;
		prev_time = ros::Time::now();
		waypoints_msgs_.poses.clear();
		for (auto& gate:gates_info_){
			if (!gate.has_passed){
				appendSegmentToPath(gate.gate_segment,waypoints_msgs_);
			}
		}
		if (waypoints_msgs_.poses.size()>0){
			traj_waypoints_pub_.publish(waypoints_msgs_);
			std::cout <<"WAYPOINTS SENDED" << std::endl;
		}
	}

};

/*****************************************/

void GatesPathGenerator::publishWaypoints(){
	for (auto &gate:gates_info_){
		if(gate.gate_id == 2){
			gate2_pub_.publish(gate.pose);
		}
		if(gate.gate_id == 1){
			gate1_pub_.publish(gate.pose);
		}
	}
};

void GatesPathGenerator::ownRun(){
	if (generating_traj_){
		genTraj();
	}
	publishWaypoints();

};

void GatesPathGenerator::publishTraj(){

	waypoints_msgs_.header.stamp = ros::Time::now();
	traj_waypoints_pub_.publish(waypoints_msgs_);
};


void GatesPathGenerator::getManualTrajCallback(const std_msgs::Float32& _speed){
	
	generating_traj_ = true;
	gates_has_changes = true; // for 1st time traj generation
	
	waypoints_msgs_.header.frame_id = "odom";
	waypoints_msgs_.yaw_mode = aerostack_msgs::TrajectoryWaypoints::PATH_FACING;
	
	float speed = _speed.data;
	if (speed > 0.1f && speed <5.0f)
		waypoints_msgs_.max_speed = speed;
	else
		waypoints_msgs_.max_speed = 0.5f;
	waypoints_msgs_.poses.clear();

	for (auto &gate:gates_info_){
		gate.has_passed=false;
		gate.is_inside=false;
	}

}

/**********************/

void GatesPathGenerator::poseCallback(const geometry_msgs::PoseStamped& _msg){
	estimated_pose_ <<_msg.pose.position.x,_msg.pose.position.y,_msg.pose.position.z;
};


void GatesPathGenerator::gatesCallback(const nav_msgs::Path& _msg){ 
	try{
		auto odom_to_camera_link_transform = tf2_buffer_.lookupTransform("odom","camera_link",ros::Time(0));
		for(unsigned short int id = 0; id<_msg.poses.size();id++){	
			// if (id > gates_info_.size()) break;
			// if all position components are zero -> discard this pose
			if (_msg.poses[id].pose.position.x + _msg.poses[id].pose.position.y + _msg.poses[id].pose.position.z != 0.0f){
				geometry_msgs::PoseStamped pose;
				tf2::doTransform(_msg.poses[id], pose, odom_to_camera_link_transform);
				pose.header.frame_id = "odom";
				pose.pose.orientation.w = 1.0f;
				pose.pose.orientation.x = 0.0f;
				pose.pose.orientation.y = 0.0f;
				pose.pose.orientation.z = 0.0f;
				// std::cout << "new pose for id:" << id <<std::endl;
				for (auto& gate:gates_info_){
					if(gate.gate_id == id+1){
						// std::cout << "gate with id:" << id+1<< " pose_updated" <<std::endl;
						
						gate.updatePose(pose);
						
						gates_has_changes = true;

					}
				}

			}
		}
	}
	catch (tf2::TransformException &ex) {
      	ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
	
}


