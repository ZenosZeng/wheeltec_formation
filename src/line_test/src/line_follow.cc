#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class FollowerController
{
public:
    FollowerController() : nh_("~")
    {
        // 参数初始化（单位：米）
        nh_.param("initial_offset_x", initial_offset_x_, -1.0); // 后方0.5m
        nh_.param("initial_offset_y", initial_offset_y_, 0.0);  // 右侧0.5m
        nh_.param("target_distance", target_distance_, 0.5);   // 目标跟随距离
        nh_.param("min_speed", min_speed_, 0.1);               // 最小线速度

        // 订阅发布
        leader_odom_sub_ = nh_.subscribe("/robot_1/odom", 10, &FollowerController::leaderOdomCallback, this);
        follower_odom_sub_ = nh_.subscribe("/robot_3/odom", 10, &FollowerController::followerOdomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 10);

        ROS_INFO("Follower initialized with physical offset: (%.2fm, %.2fm)", 
                initial_offset_x_, initial_offset_y_);
    }

private:
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat)
    {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void leaderOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 记录leader初始odom原点（仅第一次调用时记录）
        if (!got_leader_origin_)
        {
            leader_origin_x_ = msg->pose.pose.position.x;
            leader_origin_y_ = msg->pose.pose.position.y;
            leader_origin_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
            got_leader_origin_ = true;
            ROS_INFO("Leader odom origin recorded: (%.2f, %.2f)", 
                    leader_origin_x_, leader_origin_y_);
        }

        // 计算leader相对于其初始odom原点的实际位移
        leader_x_ = msg->pose.pose.position.x - leader_origin_x_;
        leader_y_ = msg->pose.pose.position.y - leader_origin_y_;
        leader_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);

        if (got_follower_odom_)
        {
            computeControlCmd();
        }
    }

    void followerOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // follower的odom读数需要补偿初始物理偏移
        follower_x_ = msg->pose.pose.position.x + initial_offset_x_;
        follower_y_ = msg->pose.pose.position.y + initial_offset_y_;
        follower_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
        got_follower_odom_ = true;

        if (got_leader_origin_)
        {
            computeControlCmd();
        }
    }

    void computeControlCmd()
    {
        geometry_msgs::Twist cmd;

        // 1. 计算目标位置（leader后方target_distance处）
        double desired_x = leader_x_ - target_distance_ * cos(leader_yaw_);
        double desired_y = leader_y_ - target_distance_ * sin(leader_yaw_);

        // 2. 计算误差（转换到follower坐标系）
        double error_x = desired_x - follower_x_;
        double error_y = desired_y - follower_y_;
        double local_error_x = cos(follower_yaw_) * error_x + sin(follower_yaw_) * error_y;
        double local_error_y = -sin(follower_yaw_) * error_x + cos(follower_yaw_) * error_y;

        // 3. P控制
        cmd.linear.x = 0.5 * local_error_x;  // kp_linear=0.5
        // cmd.angular.z = 1.0 * atan2(local_error_y, local_error_x); // kp_angular=1.0
        cmd.angular.z = 0;

        // 4. 阿克曼转向约束（v≠0才能转向）
        if (fabs(cmd.linear.x) < min_speed_ && fabs(cmd.angular.z) > 0.01)
        {
            cmd.linear.x = (cmd.linear.x >= 0) ? min_speed_ : -min_speed_;
        }

        // 5. 发布命令
        cmd_pub_.publish(cmd);

        ROS_INFO_THROTTLE(1.0, "Control: v=%.2f, w=%.2f | Leader: (%.2f,%.2f) | Follower: (%.2f,%.2f)",
                         cmd.linear.x, cmd.angular.z, leader_x_, leader_y_, follower_x_, follower_y_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber leader_odom_sub_;
    ros::Subscriber follower_odom_sub_;
    ros::Publisher cmd_pub_;

    // 初始物理偏移参数（单位：米）
    double initial_offset_x_;  // 默认后方0.5m
    double initial_offset_y_;  // 默认右侧0.5m
    double target_distance_;   // 目标跟随距离
    double min_speed_;        // 最小线速度

    // 状态变量
    double leader_origin_x_, leader_origin_y_, leader_origin_yaw_;
    double leader_x_, leader_y_, leader_yaw_;
    double follower_x_, follower_y_, follower_yaw_;
    bool got_leader_origin_ = false;
    bool got_follower_odom_ = false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot3_follower");
    FollowerController controller;
    ros::spin();
    return 0;
}