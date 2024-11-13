#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "robot/PC2BS.h"
#include "robot/BS2PC.h"

float xTurtle, yTurtle, thetaTurtle;

void poseCallback(const turtlesim::Pose::ConstPtr &msg)
{
    xTurtle = msg->x;
    yTurtle = msg->y;
    thetaTurtle = msg->theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tes");
    ros::NodeHandle nh;

    ros::Subscriber subPose = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1, poseCallback);
    ros::Publisher pubPC2BS = nh.advertise<robot::PC2BS>("pc2bs_topic", 1);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        robot::PC2BS msg;
        msg.x = xTurtle;
        msg.y = yTurtle;
        msg.theta = thetaTurtle;

        pubPC2BS.publish(msg);

        // ROS_INFO("Turtle Position: x: [%f], y: [%f], theta: [%f]", xTurtle, yTurtle, thetaTurtle);

        ros::spinOnce();
        loop_rate.sleep();
    }
}