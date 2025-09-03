#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class LineMoveController
{
public:
    LineMoveController()
    {
        // 订阅带命名空间的odom话题
        odom_sub_ = nh_.subscribe("/robot_3/odom", 10, &LineMoveController::odomCallback, this);
        // 发布带命名空间的cmd_vel话题
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 10);

        kp_ = 0.5;  // 比例控制系数，可调
        target_distance_ = 2.5; // 目标前进距离2.5米
        max_speed_ = 0.3;

        start_received_ = false;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double x = msg->pose.pose.position.x;

        if (!start_received_)
        {
            start_x_ = x;
            start_received_ = true;
            ROS_INFO("Start position recorded: x=%.3f", start_x_);
        }

        double error = target_distance_ - (x - start_x_);

        geometry_msgs::Twist cmd;

        if (error > 0.01)
        {
            cmd.linear.x = kp_ * error;
            if (cmd.linear.x > max_speed_)  // 限制最大速度
                cmd.linear.x = max_speed_;
            ROS_INFO("Moving forward, error=%.3f, cmd_vel=%.3f", error, cmd.linear.x);
        }
        else
        {
            cmd.linear.x = 0.0;
            ROS_INFO("Target reached or passed. Stopping.");
        }

        cmd_pub_.publish(cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    double kp_;
    double target_distance_;
    double max_speed_;

    double start_x_;
    bool start_received_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_move_controller");
    LineMoveController controller;
    ros::spin();
    return 0;
}
