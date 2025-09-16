#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <string>
#include <vector>
#include <cmath>
#include <functional>
#include <boost/bind/bind.hpp>

struct Robot {
    // ros::Subscriber odom_sub;
    ros::Subscriber pose_sub;
    ros::Publisher cmd_pub;

    // 状态
    double x = 0.0, y = 0.0, yaw = 0.0;
    bool pose_ekf_ready = false;

    // 初始位置
    double init_x = 0.0, init_y = 0.0, init_yaw = 0.0;

    // 控制参数
    double max_v = 0.5;  
    double max_w = 1.8;
};

class RpControllerNode {
public:
    RpControllerNode(ros::NodeHandle& nh) : nh_(nh) {
        // leader
        robots_.push_back(Robot());
        robots_[0].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_1/robot_pose_ekf/odom_combined", 10,
            boost::bind(&RpControllerNode::poseCallback, this, _1, 0));
        robots_[0].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

        // coleader
        robots_.push_back(Robot());
        robots_[1].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_2/robot_pose_ekf/odom_combined", 10,
            boost::bind(&RpControllerNode::poseCallback, this, _1, 1));
        robots_[1].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 10);
        robots_[1].init_x = -1.0;
        robots_[1].init_y = 0.0;
        robots_[1].init_yaw = 0.0;

        // follower
        robots_.push_back(Robot());
        robots_[2].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_3/robot_pose_ekf/odom_combined", 10,
            boost::bind(&RpControllerNode::poseCallback, this, _1, 2));
        robots_[2].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 10);
        robots_[2].init_x = -1.0;
        robots_[2].init_y = 1.0;
        robots_[2].init_yaw = 0.0;

        // 定时器：统一 control loop
        timer_ = nh_.createTimer(ros::Duration(0.02), &RpControllerNode::controlLoop, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;

    std::vector<Robot> robots_;

    float akm_offset = 0.2 ;
    double k_p_ = 1.0;  // P控制增益 全局

    // ====== 回调函数 ======
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, int index) {
        float x_r = msg->pose.pose.position.x;
        float y_r = msg->pose.pose.position.y;
        float yaw = tf::getYaw(msg->pose.pose.orientation);

        // 后轮中心 -> 前轮中心
        robots_[index].x = robots_[index].init_x + x_r + akm_offset * cos(yaw);
        robots_[index].y = robots_[index].init_y + y_r + akm_offset * sin(yaw);
        robots_[index].yaw = yaw + robots_[index].init_yaw;
        robots_[index].pose_ekf_ready = true;
    }

    // ====== 控制循环 ======
    void controlLoop(const ros::TimerEvent&){
        // 确保所有pose sub已准备好
        if (!robots_[0].pose_ekf_ready || !robots_[1].pose_ekf_ready || !robots_[2].pose_ekf_ready) return;

        // 获取仿真时间
        ros::Time t_now = ros::Time::now();
        static ros::Time t_start = t_now;
        double t = (t_now - t_start).toSec();

        // 0.5Hz log 每个车的位置
        ROS_INFO_THROTTLE(1, "T:%.2f | L: (%.2f, %.2f, %.2f) | C: (%.2f, %.2f, %.2f) | F: (%.2f, %.2f, %.2f)", 
            t,
            robots_[0].x, robots_[0].y, robots_[0].yaw,
            robots_[1].x, robots_[1].y, robots_[1].yaw,
            robots_[2].x, robots_[2].y, robots_[2].yaw);

        /**
         * 主控制逻辑
         * 0-5s 静止 输出位置 检查init正确性
         * 5-10s leader不动 其他先形成队形
         * 10s leader开始动 集体跟踪 (leader先走2s直线,然后sin)
         */

        if (t < 5.0) {
            publishCmd(0, {0.0, 0.0});
            publishCmd(1, {0.0, 0.0});
            publishCmd(2, {0.0, 0.0});
            return;
        }

        if(t<10.0)
        {
            k_p_ = 0.5; 
        } else {
            k_p_ = 3.0; 
        }

        std::vector<double> u_leader = getLeaderCmd(t); // vx vy
        std::vector<double> u_coleader = getColeaderCmd();
        std::vector<double> u_follower = getFollowerCmd();

        // 转换成v omega并发布
        publishCmd(0, u_leader);
        publishCmd(1, u_coleader);
        publishCmd(2, u_follower);
    }

    // ======= Leader速度计算 =======
    std::vector<double> getLeaderCmd(double t) {
        double vx = 0.0;
        double vy = 0.0;

        // leader delay 10s
        if (t < 10.0) {
            return {vx, vy};
        } 
        
        // sin para
        double A = 0.06;     // 振幅
        double w = 0.5;     // omega
        double v_straight = 0.2; // 前直线速度

        // straight + sin (10-12 straight, 12s+ sin)
        if (t < 12.0) {
            vx = v_straight;
            vy = 0.0;
        } else { 
            vx = v_straight;  // x方向速度
            vy = -A*sin(w*(t-12.0)); // pi=3.14
        }

        // stop signal
        if ( robots_[0].x > 4.0 ) {
            vx = 0.0;
            vy = 0.0; 
        } // stop

        return {vx, vy};
    }

    // ======= Coleader速度计算 =======
    std::vector<double> getColeaderCmd(double desired_distance = 0.8) {
        // 获取leader和coleader状态
        double lx = robots_[0].x;
        double ly = robots_[0].y;
        double lyaw = robots_[0].yaw;

        double cx = robots_[1].x;
        double cy = robots_[1].y;
        double cyaw = robots_[1].yaw;

        // 相对位置误差
        double dx = lx - cx;
        double dy = ly - cy;

        // 期望位置在leader后方
        double angle_to_leader = atan2(dy, dx);
        double ex = dx - desired_distance * cos(angle_to_leader);
        double ey = dy - desired_distance * sin(angle_to_leader);
        ROS_INFO_THROTTLE(1, "err_c: (%.2f, %.2f)", ex, ey);

        double vx_global = k_p_ * ex;
        double vy_global = k_p_ * ey;

        return {vx_global, vy_global}; // 全局系下的速度
    }

    // ======= Follower速度计算 =======
    std::vector<double> getFollowerCmd(double desired_back = 0.4, double desired_left = 0.7) {
        // leader状态
        double lx = robots_[0].x;
        double ly = robots_[0].y;
        double lyaw = robots_[0].yaw;

        // follower状态
        double fx = robots_[2].x;
        double fy = robots_[2].y;
        double fyaw = robots_[2].yaw;

        // ---- 计算 target point （全局系下的期望位置）----
        // 在 leader 坐标系下: (-desired_back, +desired_left)
        double tx_local = -desired_back;
        double ty_local = +desired_left;

        // 转换到全局系
        double target_x = lx + cos(lyaw) * tx_local - sin(lyaw) * ty_local;
        double target_y = ly + sin(lyaw) * tx_local + cos(lyaw) * ty_local;

        // ---- 计算误差 (ex, ey) ----
        double ex = target_x - fx;
        double ey = target_y - fy;
        ROS_INFO_THROTTLE(1, "err_f: (%.2f, %.2f)", ex, ey);

        // ---- P 控制 (全局系速度) ----
        double vx_global = k_p_ * ex;
        double vy_global = k_p_ * ey;

        return {vx_global, vy_global}; // 全局系速度
    }


    // ======= 发布函数 =======
    void publishCmd(int index, const std::vector<double>& v) {
        geometry_msgs::Twist twist;
        
        // 转换到个体坐标系
        double vx_local =  cos(robots_[index].yaw) * v[0] + sin(robots_[index].yaw) * v[1];
        double vy_local = -sin(robots_[index].yaw) * v[0] + cos(robots_[index].yaw) * v[1];
        double omega = vy_local / akm_offset; // 侧向速度转角速度

        // 限幅
        if (vx_local > robots_[index].max_v) vx_local = robots_[index].max_v;
        // if (vx_local < -robots_[index].max_v) vx_local = -robots_[index].max_v;
        if (vx_local < 0.0) vx_local = 0.0; // 不倒车

        if (omega > robots_[index].max_w) omega = robots_[index].max_w;
        if (omega < -robots_[index].max_w) omega = -robots_[index].max_w;

        // 发布
        twist.linear.x = vx_local;
        twist.angular.z = omega;
        robots_[index].cmd_pub.publish(twist);
    }

};
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "rp_controller_node");
    ros::NodeHandle nh;
    RpControllerNode node(nh);
    ros::spin();
    return 0;
}
