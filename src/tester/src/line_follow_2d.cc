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
        // 参数初始化
        nh_.param("initial_offset_x", initial_offset_x_, -1.0);   // follower初始相对leader的x偏移
        nh_.param("initial_offset_y", initial_offset_y_, 0.0);   // follower初始相对leader的y偏移
        nh_.param("desired_offset_x", desired_offset_x_, -0.5);  // 期望相对leader的x偏移（后方1m）
        nh_.param("desired_offset_y", desired_offset_y_, 0.0);   // 期望相对leader的y偏移（右侧0m）
        nh_.param("kp_linear", kp_linear_, 2.0);                // 纵向比例增益
        nh_.param("kp_angular", kp_angular_, 1.0);              // 横向比例增益
        nh_.param("max_v", max_v_, 0.5);                        // 最大线速度
        nh_.param("max_w", max_w_, 1.0);                        // 最大角速度
        nh_.param("min_v", min_v_, 0.05);                       // 最小线速度（防止停滞）

        // 订阅/发布
        leader_odom_sub_ = nh_.subscribe("/robot_1/odom", 10, &FollowerController::leaderOdomCallback, this);
        follower_odom_sub_ = nh_.subscribe("/robot_2/odom", 10, &FollowerController::followerOdomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 10);

        ROS_INFO("Follower initialized: initial offset (%.2f, %.2f), desired offset (%.2f, %.2f)", 
                 initial_offset_x_, initial_offset_y_, desired_offset_x_, desired_offset_y_);
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
        leader_x_ = msg->pose.pose.position.x;
        leader_y_ = msg->pose.pose.position.y;
        leader_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
        got_leader_odom_ = true;

        if (got_follower_odom_)
        {
            computeControlCmd();
        }
    }

    void followerOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // follower 的位置加上初始相对偏移
        follower_x_ = msg->pose.pose.position.x + initial_offset_x_;
        follower_y_ = msg->pose.pose.position.y + initial_offset_y_;
        follower_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
        got_follower_odom_ = true;

        if (got_leader_odom_)
        {
            computeControlCmd();
        }
    }

    void computeControlCmd()
    {
        geometry_msgs::Twist cmd;

        // 1. 期望位置（leader坐标系下的期望偏移）
        double desired_x = leader_x_ + desired_offset_x_ * cos(leader_yaw_) - desired_offset_y_ * sin(leader_yaw_);
        double desired_y = leader_y_ + desired_offset_x_ * sin(leader_yaw_) + desired_offset_y_ * cos(leader_yaw_);

        // 2. 误差（全局坐标系）
        double error_x = desired_x - follower_x_;
        double error_y = desired_y - follower_y_;

        // 3. 转换到follower坐标系
        double local_error_x = cos(follower_yaw_) * error_x + sin(follower_yaw_) * error_y;
        double local_error_y = -sin(follower_yaw_) * error_x + cos(follower_yaw_) * error_y;

        // 4. 控制律
        cmd.linear.x = kp_linear_ * local_error_x;
        cmd.angular.z = kp_angular_ * atan2(local_error_y, local_error_x);

        // 5. 限幅
        if (cmd.linear.x > max_v_) cmd.linear.x = max_v_;
        if (cmd.linear.x < -max_v_) cmd.linear.x = -max_v_;
        if (fabs(cmd.linear.x) < min_v_ && fabs(local_error_x) > 0.05)
        {
            cmd.linear.x = (cmd.linear.x >= 0) ? min_v_ : -min_v_;
        }

        if (cmd.angular.z > max_w_) cmd.angular.z = max_w_;
        if (cmd.angular.z < -max_w_) cmd.angular.z = -max_w_;

        // 6. 发布
        cmd_pub_.publish(cmd);

        ROS_INFO_THROTTLE(0.5, "Control: v=%.2f, w=%.2f | Error: (%.2f, %.2f) | Leader: (%.2f,%.2f) | Follower: (%.2f,%.2f)",
                          cmd.linear.x, cmd.angular.z, error_x, error_y,
                          leader_x_, leader_y_, follower_x_, follower_y_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber leader_odom_sub_;
    ros::Subscriber follower_odom_sub_;
    ros::Publisher cmd_pub_;

    // 参数
    double initial_offset_x_, initial_offset_y_;
    double desired_offset_x_, desired_offset_y_;
    double kp_linear_, kp_angular_;
    double max_v_, max_w_, min_v_;

    // 状态变量
    double leader_x_, leader_y_, leader_yaw_;
    double follower_x_, follower_y_, follower_yaw_;
    bool got_leader_odom_ = false;
    bool got_follower_odom_ = false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot2_follower");
    FollowerController controller;
    ros::spin();
    return 0;
}
