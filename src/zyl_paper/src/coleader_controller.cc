#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class CoLeaderControlNode
{
public:
    CoLeaderControlNode()
    {
        odom_sub_ = nh_.subscribe("/robot_2/odom", 10, &CoLeaderControlNode::odomCallback, this);
        leader_odom_sub_ = nh_.subscribe("/robot_1/odom", 10, &CoLeaderControlNode::leaderOdomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 10);

        kp_x_ = 1.0;       // x方向控制增益
        kp_y_ = 1.0;       // y方向控制增益
        max_v = 0.4;
        max_omega = 1.0;

        d_desired_ = 0.3;  // 期望与 leader 的距离 (m)
        x_self_start_ = -0.8;
        y_self_start_ = 0.0;
        

        leader_received_ = false;
        self_received_ = false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber leader_odom_sub_;
    ros::Publisher cmd_pub_;

    // 控制参数
    double kp_x_, kp_y_;
    double d_desired_;

    double max_v, max_omega;

    // 位置
    double x_self_, y_self_, theta_self_;
    double x_self_start_, y_self_start_;
    double x_leader_, y_leader_, theta_leader_;

    bool leader_received_, self_received_;

    //=====================
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        x_self_ = msg->pose.pose.position.x + x_self_start_;
        y_self_ = msg->pose.pose.position.y + y_self_start_;
        theta_self_ = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        self_received_ = true;

        UpdateControlCommand();
    }

    //=====================
    void leaderOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        x_leader_ = msg->pose.pose.position.x;
        y_leader_ = msg->pose.pose.position.y;
        theta_leader_ = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        leader_received_ = true;
    }

    //=====================
    void UpdateControlCommand()
    {
        if (!(leader_received_ && self_received_)) return;

        geometry_msgs::Twist cmd;

        // 期望位置
        double x_desired = x_leader_ - d_desired_;
        double y_desired = y_leader_;

        // 全局误差
        double e_x_g = x_desired - x_self_;
        double e_y_g = y_desired - y_self_;

        // 转换到车体坐标系
        double e_x_b =  cos(theta_self_) * e_x_g + sin(theta_self_) * e_y_g;
        double e_y_b = -sin(theta_self_) * e_x_g + cos(theta_self_) * e_y_g;

        // 控制律
        double vx = kp_x_ * e_x_b;
        double omega = kp_y_ * e_y_b;

        // 限幅
        if (vx > max_v) vx = max_v;
        if (vx < -max_v) vx = -max_v;
        if (omega > max_omega) omega = max_omega;
        if (omega < -max_omega) omega = -max_omega;

        cmd.linear.x = vx;
        cmd.angular.z = omega;
        cmd_pub_.publish(cmd);

        ROS_INFO_THROTTLE(0.5, "CoLeader: e_x_b=%.2f, e_y_b=%.2f | vx=%.2f, omega=%.2f",
                          e_x_b, e_y_b, vx, omega);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot2_coleader_controller");
    CoLeaderControlNode controller;
    ros::spin();
    return 0;
}
