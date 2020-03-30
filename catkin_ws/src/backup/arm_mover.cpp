#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include "std_msgs/Float64.h"


ros::Publisher j1_pub;
ros::Publisher j2_pub;


std::vector<float> move_clamped(float j1_requested, float j2_requested)
{
	float j1_clamped = j1_requested;
	float j2_clamped = j2_requested;

	float j1_min, j1_max, j2_min, j2_max;

	ros::NodeHandle n2;

	std::string nodeName = ros::this_node::getName();
	n2.getParam(nodeName + "/min_joint_1_angle", j1_min);
	n2.getParam(nodeName + "/max_joint_1_angle", j1_max);
	n2.getParam(nodeName + "/min_joint_2_angle", j2_min);
	n2.getParam(nodeName + "/max_joint_2_angle", j2_max);


	if (j1_requested < j1_min)
	{
		j1_clamped = j1_min;
		ROS_WARN("j1 too small");
	}
	
	if (j1_requested > j1_max)
	{
		j1_clamped = j1_max;
		ROS_WARN("j1 too big");
	}

	if (j2_requested < j2_min)
	{
		j2_clamped = j2_min;
		ROS_WARN("j2 too small");
	}
	
	if (j2_requested > j2_max)
	{
		j2_clamped = j2_max;
		ROS_WARN("j2 too big");
	}

	std::vector<float> data_clamped = {j1_clamped, j2_clamped};

	return data_clamped;
}



bool move_safe(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res)
{
	std::vector<float> joint_angles = move_clamped(req.joint_1, req.joint_2);
	std_msgs::Float64 j1_angle, j2_angle;

	j1_angle.data = joint_angles[0];
	j2_angle.data = joint_angles[1];


	j1_pub.publish(j1_angle);
	j2_pub.publish(j2_angle);

    res.msg_feedback = "Joint angles set - j1: " + std::to_string(joint_angles[0]) + " , j2: " + std::to_string(joint_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);
    return true;

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    j1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10); 
    j2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10); 
    ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", move_safe);
    ROS_INFO("ready to send commmand");

    ros::spin();
    return 0;

}
