#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

class GoToPointController
{
public:
    GoToPointController()
    {
        odom_sub_ = nh_.subscribe("/robot_1/robot_pose_ekf/odom_combined", 10, &GoToPointController::odomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

        // 控制参数
        kp_linear_ = 0.6;   // 线速度比例
        kp_angular_ = 1.5;  // 角速度比例
        max_speed_ = 0.3;   // 限制最大速度
        max_angular_ = 1.0; // 限制最大角速度

        // 目标点
        target_x_ = 5.0;
        target_y_ = 1.0;

        start_received_ = false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    double kp_linear_;
    double kp_angular_;
    double max_speed_;
    double max_angular_;

    double target_x_;
    double target_y_;

    double x_, y_, theta_;
    double start_x_, start_y_, start_theta_;
    bool start_received_;

    void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        UpdateRobotState(msg);
        UpdateControlCommand();
    }

    void UpdateRobotState(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        theta_ = tf::getYaw(msg->pose.pose.orientation);

        if (!start_received_)
        {
            start_x_ = 0.0;
            start_y_ = 0.0;
            start_theta_ = 0.0;
            start_received_ = true;
            ROS_INFO("Start recorded: x=%.3f, y=%.3f, theta=%.3f", start_x_, start_y_, start_theta_);
        }
    }

    void UpdateControlCommand()
    {
        // 目标点相对位置（以起点为原点）
        double dx = (target_x_ + start_x_) - x_;
        double dy = (target_y_ + start_y_) - y_;
        double distance = std::sqrt(dx*dx + dy*dy);

        double target_yaw = std::atan2(dy, dx);
        double yaw_error = atan2(sin(target_yaw - theta_), cos(target_yaw - theta_));

        geometry_msgs::Twist cmd;

        if (distance > 0.01)
        {
            cmd.linear.x = kp_linear_ * distance;
            cmd.angular.z = kp_angular_ * yaw_error;

            // 限制速度
            if (cmd.linear.x > max_speed_) cmd.linear.x = max_speed_;
            if (cmd.angular.z > max_angular_) cmd.angular.z = max_angular_;
            if (cmd.angular.z < -max_angular_) cmd.angular.z = -max_angular_;

            ROS_INFO_THROTTLE(1.0, "x=%.3f y=%.3f theta=%.3f", x_, y_, theta_);
            ROS_INFO_THROTTLE(1.0, "dx=%.2f dy=%.2f dist=%.2f yaw_err=%.2f v=%.2f w=%.2f",
                     dx, dy, distance, yaw_error, cmd.linear.x, cmd.angular.z);
            ROS_INFO_THROTTLE(1.0, "-------------------------------");;
            
        }
        else
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            ROS_INFO("Target reached!");
        }

        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot1_gotopoint");
    GoToPointController controller;
    ros::spin();
    return 0;
}
