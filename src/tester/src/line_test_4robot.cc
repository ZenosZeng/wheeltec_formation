#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>

class LineMoveController
{
public:
    LineMoveController(const std::string& robot_name, double target_x, double target_y, double speed_limit)
        : nh_("~"), robot_name_(robot_name), target_x_(target_x), target_y_(target_y), max_speed_(speed_limit)
    {
        odom_sub_ = nh_.subscribe("/" + robot_name_ + "/odom", 10,
                                  &LineMoveController::odomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/" + robot_name_ + "/cmd_vel", 10);

        // 控制参数
        kp_linear_ = 0.5;
        kp_angular_ = 1.0;
        max_angular_ = 0.5;

        start_received_ = false;

        ROS_INFO("[%s] Controller initialized. Target=(%.2f, %.2f)",
                 robot_name_.c_str(), target_x_, target_y_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    std::string robot_name_;

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
            ROS_INFO("[%s] Start pose: x=%.3f, y=%.3f, theta=%.3f",
                     robot_name_.c_str(), start_x_, start_y_, start_theta_);
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
        yaw_error = atan2(sin(yaw_error), cos(yaw_error)); // wrap [-pi,pi]

        geometry_msgs::Twist cmd;

        if (distance > 0.01)
        {
            cmd.linear.x = kp_linear_ * distance;
            cmd.angular.z = kp_angular_ * yaw_error;

            // 限制速度
            if (cmd.linear.x > max_speed_)
                cmd.linear.x = max_speed_;
            if (cmd.angular.z > max_angular_)
                cmd.angular.z = max_angular_;
            else if (cmd.angular.z < -max_angular_)
                cmd.angular.z = -max_angular_;

            ROS_INFO("[%s] Dist=%.2f, Lin=%.2f, YawErr=%.2f, Ang=%.2f",
                     robot_name_.c_str(), distance, cmd.linear.x, yaw_error, cmd.angular.z);
        }
        else
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            ROS_INFO("[%s] Target reached.", robot_name_.c_str());
        }

        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_robot_controller");

    // 四个车，目标点可以不同
    LineMoveController robot1("robot_1", 4, 0, 0.5);
    LineMoveController robot2("robot_2", 3.5, 0, 0.4);
    LineMoveController robot3("robot_3", 3, 0, 0.3);
    LineMoveController robot4("robot_4", 2.5, 0, 0.2);

    ros::spin();
    return 0;
}
