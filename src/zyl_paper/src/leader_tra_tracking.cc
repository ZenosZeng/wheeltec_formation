#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class LeaderControlNode
{
public:
    LeaderControlNode()
    {
        odom_sub_ = nh_.subscribe("/robot_1/odom", 10, &LeaderControlNode::odomCallback, this);
        cmd_pub_  = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

        // 参数可调
        vx_ = 0.2;    // x方向恒定速度 (m/s)
        vy_ = 0.08;    // y方向振幅 (m/s)
        a_  = 0.8;    // 振荡频率 (rad/s)
        wheelbase_ = 0.3; // 等效轮距，用于侧向速度转角速度

        start_time_ = ros::Time::now();
        start_received_ = false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;

    // 控制参数
    double vx_, vy_, a_;
    double wheelbase_;
    ros::Time start_time_;

    // 当前位姿
    double x_, y_, theta_;
    bool start_received_;

    //=====================
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        UpdateRobotState(msg);
        UpdateControlCommand();
    }

    //=====================
    void UpdateRobotState(const nav_msgs::Odometry::ConstPtr& msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        theta_ = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        if (!start_received_)
        {
            start_received_ = true;
            ROS_INFO("Start pose recorded: x=%.3f, y=%.3f, theta=%.3f", x_, y_, theta_);
        }
    }

    //=====================
    void UpdateControlCommand()
    {
        geometry_msgs::Twist cmd;

        // 如果超过 4m，停车
        if (x_ >= 4.0)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_.publish(cmd);
            ROS_INFO_ONCE("Pe: x=%.3f >= 4.0m, stopping.", x_);
            return;
        }

        // 当前时间
        double t = (ros::Time::now() - start_time_).toSec();

        double v_body_x = 0.0;
        double v_body_y = 0.0;
        double omega = 0.0;

        // 起步阶段：沿 x 轴直走 0.2 m/s
        if (x_ < 0.5)
        {
            v_body_x = 0.2;
            v_body_y = 0.0;
            omega = 0.0;
            ROS_INFO_THROTTLE(0.5, "P0: x=%.2f < 0.5m, vx=0.2 m/s", x_);
        }
        else
        {
            // 轨迹阶段：执行 vx + vy*sin(at)
            double vx = vx_;
            double vy = vy_ * sin(a_ * (t - 0.5 / vx_)); // 延时0.5s开始振荡

            // 转换到小车坐标系
            v_body_x = cos(theta_) * vx + sin(theta_) * vy;
            v_body_y = -sin(theta_) * vx + cos(theta_) * vy;

            // 侧向速度映射成角速度
            omega = v_body_y / wheelbase_;

            ROS_INFO_THROTTLE(0.5, "P1: t=%.2f, global(vx=%.2f, vy=%.2f), "
                    "body(vx=%.2f, vy=%.2f), omega=%.2f, x=%.2f",
                    t, vx, vy, v_body_x, v_body_y, omega, x_);
        }

        // 填充指令
        cmd.linear.x  = v_body_x;
        cmd.angular.z = omega;
        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot1_controller");
    LeaderControlNode controller;
    ros::spin();
    return 0;
}
