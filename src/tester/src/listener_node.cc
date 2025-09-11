#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

ros::Publisher pub;

void pingCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    pub.publish(*msg);  // 收到消息立即回发
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pong_node");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Pose>("/pong", 1);
    ros::Subscriber sub = nh.subscribe("/ping", 1, pingCallback);

    ROS_INFO("Pong node initialized successfully. Ready to receive messages.");

    ros::spin();
    return 0;
}
