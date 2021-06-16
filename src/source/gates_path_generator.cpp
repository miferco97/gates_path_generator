#include "gates_path_generator.hpp"


void GatesPathGenerator::ownSetUp()
{

	ros_utils_lib::getPrivateParam<std::string>("~namespace"					, n_space_						,"drone1");
	ros_utils_lib::getPrivateParam<std::string>("~gates_topic"	    			, gates_topic_					,"gate_detector/detected_poses");
	ros_utils_lib::getPrivateParam<std::string>("~traj_waypoints_topic"		    , traj_waypoints_topic_			,"motion_reference/waypoints");
	ros_utils_lib::getPrivateParam<std::string>("~self_localization_pose_topic" , self_localization_pose_topic_ ,"self_localization/pose");

	gates_sub_ = nh_.subscribe("/" + n_space_ + "/" + gates_topic_ ,1, &GatesPathGenerator::gatesCallback,this);
	pose_sub_ = nh_.subscribe("/" + n_space_ + "/" + self_localization_pose_topic_ ,1, &GatesPathGenerator::poseCallback,this);
	gen_traj_sub_ = nh_.subscribe("/gen_manual_traj" ,1, &GatesPathGenerator::genManualTraj,this);
	traj_waypoints_pub_ = nh_.advertise <aerostack_msgs::TrajectoryWaypoints>("/" + n_space_ + "/" + traj_waypoints_topic_,1);
	
	gates_vector_ = std::vector<geometry_msgs::PoseStamped>(N_GATES);
	gate2_pub_ = nh_.advertise <geometry_msgs::PoseStamped>("/" + n_space_ + "/" + "debug/pose_aruco2",1);
	tf2_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf2_buffer_);

}

void filterPose(geometry_msgs::PoseStamped& pose){
	static geometry_msgs::PoseStamped prev_pose = pose;
	const float alpha = 0.1;

	pose.pose.position.x = prev_pose.pose.position.x * (1-alpha) + pose.pose.position.x* (alpha);
	pose.pose.position.y = prev_pose.pose.position.y * (1-alpha) + pose.pose.position.y* (alpha);
	pose.pose.position.z = prev_pose.pose.position.z * (1-alpha) + pose.pose.position.z* (alpha);

	prev_pose = pose;

}

void GatesPathGenerator::gatesCallback(const nav_msgs::Path& _msg){
	// std::cout<<"PATH_RECEIVED"<< std::endl; 
	try{
		auto odom_to_camera_link_transform = tf2_buffer_.lookupTransform("odom","camera_link",ros::Time(0));
		for(unsigned short int i = 0; i<_msg.poses.size();i++){	
			if (i > N_GATES) break;
			// if all position components are zero -> discard this pose
			if (_msg.poses[i].pose.position.x + _msg.poses[i].pose.position.y + _msg.poses[i].pose.position.z != 0.0f){
				geometry_msgs::PoseStamped pose;
				tf2::doTransform(_msg.poses[i], pose, odom_to_camera_link_transform);
				pose.header.frame_id = "odom";
				pose.pose.orientation.w = 1.0f;
				pose.pose.orientation.x = 0.0f;
				pose.pose.orientation.y = 0.0f;
				pose.pose.orientation.z = 0.0f;
				filterPose(pose);
				gates_vector_.at(i)  =  pose;
				gates_has_changes = true;
				// std::cout<<"index="<< i << std::endl; 
			}
		}
	}
	catch (tf2::TransformException &ex) {
      	ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
	
}


geometry_msgs::PoseStamped movePose(const geometry_msgs::PoseStamped& pose, float dx,float dy, float dz){
	auto out_pose = pose;
	out_pose.pose.position.x += dx; 
	out_pose.pose.position.y += dy; 
	out_pose.pose.position.z += dz;
	return out_pose;
}


void GatesPathGenerator::genTraj(){
	static ros::Time prev_time = ros::Time::now();
	
	if (!gates_has_changes) return;
	gates_has_changes = false;

	if ((ros::Time::now()-prev_time).toSec()>refresh_rate_){
		auto ref_pose  = gates_vector_.at(1);
		waypoints_msgs_.poses.clear();
		waypoints_msgs_.poses.reserve(3);
		waypoints_msgs_.poses.emplace_back(movePose(ref_pose,-0.5,0.0,0.0));
		waypoints_msgs_.poses.emplace_back(ref_pose);
		waypoints_msgs_.poses.emplace_back(movePose(ref_pose,+1.0,0.0,0.0));
		prev_time = ros::Time::now();
		publishTraj();
	}
};

void GatesPathGenerator::publishWaypoints(){
	gate2_pub_.publish(gates_vector_.at(1));
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

void GatesPathGenerator::genManualTraj(const std_msgs::Float32& _speed){
	

	auto ref_pose  = gates_vector_.at(1);
	generating_traj_ = true;
	gates_has_changes = true; // for 1st time traj generation
	
	aerostack_msgs::TrajectoryWaypoints manual_waypoints;
	waypoints_msgs_.header.frame_id = "odom";
	waypoints_msgs_.yaw_mode = aerostack_msgs::TrajectoryWaypoints::PATH_FACING;
	
	float speed = _speed.data;
	if (speed>0.1 && speed<5.0)
		waypoints_msgs_.max_speed = 0.5;
	else
		waypoints_msgs_.max_speed = 0.5;
	waypoints_msgs_.poses.clear();
}


void GatesPathGenerator::poseCallback(const geometry_msgs::PoseStamped& _msg){
	if(waypoints_msgs_.poses.size()>0){
		Eigen::Vector3d estimated_pose(_msg.pose.position.x,_msg.pose.position.y,_msg.pose.position.z);
		Eigen::Vector3d firstWaypointPose(	waypoints_msgs_.poses.at(0).pose.position.x,
											waypoints_msgs_.poses.at(0).pose.position.y,
											waypoints_msgs_.poses.at(0).pose.position.z);
		Eigen::Vector3d dist = estimated_pose-firstWaypointPose;
		// if drone is near the first point stop generating waypoints
		if (dist.norm()< 0.3)(generating_traj_ = false);
	}
	

};


