#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <chrono>

ros::Publisher pub;
std::chrono::steady_clock::time_point send_time;
double last_print_time = 0.0;

void pongCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    auto recv_time = std::chrono::steady_clock::now();
    double rtt = std::chrono::duration<double, std::milli>(recv_time - send_time).count();
    double latency = rtt / 2.0;  // 等号赋值

    // 控制打印频率 0.5s
    double now = ros::Time::now().toSec();
    if (now - last_print_time >= 0.5)
    {
        last_print_time = now;
        ROS_INFO("RTT=%.3f ms, Latency=%f ms", rtt, latency);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ping_node");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Pose>("/ping", 1);
    ros::Subscriber sub = nh.subscribe("/pong", 1, pongCallback);

    ros::Rate rate(1); // 1Hz
    while (ros::ok())
    {
        geometry_msgs::Pose msg;
        msg.position.x = 1.0;
        msg.position.y = 2.0;
        msg.position.z = 0.0;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;

        send_time = std::chrono::steady_clock::now();
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
