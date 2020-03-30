#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Publisher j1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10); 
    ros::Publisher j2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10); 
    int start_time, elapsed;

    while(not start_time)
    {
        start_time = ros::Time::now().toSec();
    }

    while(ros::ok())
    {
        elapsed = ros::Time::now().toSec() - start_time;
        std_msgs::Float64 j1_angle, j2_angle;
        j1_angle.data = sin(2* 0.1 * M_PI * elapsed) * (M_PI/2);
        j2_angle.data = sin(2* 0.1 * M_PI * elapsed) * (M_PI/2);
        j1_pub.publish(j1_angle);
        j2_pub.publish(j2_angle);
        loop_rate.sleep();
    }
    return 0;

}