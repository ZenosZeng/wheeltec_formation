#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class LineMoveController
{
public:
    LineMoveController()
    {
        odom_sub_ = nh_.subscribe("/robot_1/odom", 10, &LineMoveController::odomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

        kp_linear_ = 0.5;   // 线速度比例
        kp_angular_ = 1.0;  // 角速度比例

        target_x_ = 3.5;
        target_y_ = -0.5;

        max_speed_ = 0.3;
        max_angular_ = 0.5; // rad/s

        start_received_ = false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    // 控制参数
    double kp_linear_;
    double kp_angular_;
    double max_speed_;
    double max_angular_;

    // 目标点
    double target_x_;
    double target_y_;

    // 当前位姿
    double x_;
    double y_;
    double theta_;

    // 初始位姿
    double start_x_;
    double start_y_;
    double start_theta_;
    bool start_received_;

    //========================
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        UpdateRobotState(msg);
        UpdateControlCommand();
    }

    //========================
    void UpdateRobotState(const nav_msgs::Odometry::ConstPtr& msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        theta_ = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        if (!start_received_)
        {
            start_x_ = x_;
            start_y_ = y_;
            start_theta_ = theta_;
            start_received_ = true;
            ROS_INFO("Start position recorded: x=%.3f, y=%.3f, theta=%.3f", start_x_, start_y_, start_theta_);
        }
    }

    //========================
    void UpdateControlCommand()
    {
        double dx = target_x_ - (x_ - start_x_);
        double dy = target_y_ - (y_ - start_y_);
        double distance = std::sqrt(dx*dx + dy*dy);

        // 目标朝向
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = target_yaw - theta_;
        // 将误差限制到 [-pi, pi]
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));

        geometry_msgs::Twist cmd;

        // 控制线速度和角速度
        if (distance > 0.01)
        {   
            // compute control commands
            cmd.linear.x = kp_linear_ * distance;
            cmd.angular.z = kp_angular_ * yaw_error;
            
            // limit speeds
            if (cmd.linear.x > max_speed_)
                cmd.linear.x = max_speed_;
            
            if (cmd.angular.z > max_angular_)
                cmd.angular.z = max_angular_;
            else if (cmd.angular.z < -max_angular_)
                cmd.angular.z = -max_angular_;

            ROS_INFO("Distance=%.3f, Linear=%.3f, Yaw error=%.3f, Angular=%.3f",
                     distance, cmd.linear.x, yaw_error, cmd.angular.z);
        }
        else // 到达目标点
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            ROS_INFO("Target reached. Stopping.");
        }

        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot1_controller");
    LineMoveController controller;
    ros::spin();
    return 0;
}
