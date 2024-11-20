#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "robot/PC2BS.h"
#include "robot/BS2PC.h"
#include "geometry_msgs/Twist.h"

float xTurtle, yTurtle, thetaTurtle;
ros::Publisher pubTwist;

void poseCallback(const turtlesim::Pose::ConstPtr &msg)
{
    xTurtle = msg->x;
    yTurtle = msg->y;
    thetaTurtle = msg->theta;
}

void bs2pcCallback(const robot::BS2PC::ConstPtr &msg)
{
    ROS_INFO("Received: linear_velocity: [%f], angular_velocity: [%f]", msg->linear_velocity, msg->angular_velocity);

    geometry_msgs::Twist twistMsg;
    twistMsg.linear.x = msg->linear_velocity;
    twistMsg.angular.z = msg->angular_velocity;

    pubTwist.publish(twistMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tes");
    ros::NodeHandle nh;

    ros::Publisher pubPC2BS = nh.advertise<robot::PC2BS>("pc2bs_topic", 1);
    ros::Subscriber subPose = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1, poseCallback);
    ros::Subscriber subBS2PC = nh.subscribe<robot::BS2PC>("bs2pc_topic", 1, bs2pcCallback);
    pubTwist = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    ros::Rate loop_rate(10);

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