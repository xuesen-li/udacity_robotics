#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if (!client.call(srv))
        ROS_ERROR("Failed to move");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int position = -1;

    for (int i = 0; i < (img.height * img.step-3); i++) {

        //ROS_INFO("image height, width, step %d, %d, %d", img.height, img.width, img.step);
        
        // ROS_INFO("image data %i %d",i,  img.data[i]);
        
        if ((img.data[i] + img.data[i+1] + img.data[i+2]) == white_pixel * 3) 
        {
            position = i%img.step;
            break;
        }
    }

    //ROS_INFO("position and %d", position);
    // ROS_INFO("height and width %d, %d",  img.height, img.width);

    if (position == -1 )
    {
        drive_robot(0, 0);
        ROS_INFO("CAN'T SEE THE BALL");
    }
    else if (position < 1* img.step / 3  )
    {
        drive_robot(0.1, 0.1);
        ROS_INFO("BALL IS AT LEFT SIDE");
    }
    else if (position > 2 * img.step / 3 )
    {
        drive_robot(0.1, -0.1);
        ROS_INFO("BALL IS AT RIGHT SIDE");
    }
    else
    {
        drive_robot(0, 0);
        ROS_INFO("BALL IS AT CENTER");
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 50, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}