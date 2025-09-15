#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <string>
#include <vector>
#include <cmath>
#include <functional>
#include <boost/bind/bind.hpp>
#include <Eigen/Dense>

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
    double max_w = 0.6;     
};

class FormationNode {
public:
    FormationNode(ros::NodeHandle& nh) : nh_(nh) {
        // leader
        robots_.push_back(Robot());
        robots_[0].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_1/robot_pose_ekf/odom_combined", 10,
            boost::bind(&FormationNode::poseCallback, this, _1, 0));
        robots_[0].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

        // coleader
        robots_.push_back(Robot());
        robots_[1].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_2/robot_pose_ekf/odom_combined", 10,
            boost::bind(&FormationNode::poseCallback, this, _1, 1));
        robots_[1].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 10);
        robots_[1].init_x = -1.0;
        robots_[1].init_y = 0.0;
        robots_[1].init_yaw = 0.0;

        // follower
        robots_.push_back(Robot());
        robots_[2].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_3/robot_pose_ekf/odom_combined", 10,
            boost::bind(&FormationNode::poseCallback, this, _1, 2));
        robots_[2].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 10);
        robots_[2].init_x = -1.0;
        robots_[2].init_y = 1.0;
        robots_[2].init_yaw = 0.0;

        // 定时器：统一 control loop
        timer_ = nh_.createTimer(ros::Duration(0.02), &FormationNode::controlLoop, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;

    std::vector<Robot> robots_;

    // 控制参数
    float akm_offset = 0.2 ;
    double k_p_ = 0.5;  // P控制增益 全局
    double alpha_ = 0.6;  // 定向增益
    double beta_ = 0.0;   // SMC

    // leader 运动参数 vy=-Asin(wt)
    double A_ = 0.1;     // 振幅
    double w_ = 0.5;     // omega
    
    double init_time_ = 5.0;
    double leader_start_time_ = 10.0;
    double go_straight_time_ = 2.0;
    double v_straight_ = 0.2; // vx

    // 定向向量
    Eigen::Vector2d p_o_;
    Eigen::Vector2d dp_o_;

    // 队形
    double d_01_ = 0.6;
    double d_02_ = 0.6;
    double d_12_ = 0.6;

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

        if (t < init_time_) {
            publishCmd(0, {0.0, 0.0});
            publishCmd(1, {0.0, 0.0});
            publishCmd(2, {0.0, 0.0});
            return;
        }

        // if(t < leader_start_time_)
        // {
        //     k_p_ = 0.5; 
        // } else {
        //     k_p_ = 1.0; 
        // }

        Eigen::Vector2d u_leader = getLeaderCmd(t); // vx vy
        Eigen::Vector2d u_coleader = getColeaderCmd(u_leader);
        Eigen::Vector2d u_follower = getFollowerCmd();

        // 转换成v omega并发布
        publishCmd(0, u_leader);
        publishCmd(1, u_coleader);
        publishCmd(2, u_follower);
    }

    // ======= Leader速度计算 =======
    // 该函数返回leader的期望速度vd，以及更新po dpo
    Eigen::Vector2d getLeaderCmd(double t) {
        Eigen::Vector2d vd;
        // leader delay 10s
        if (t < leader_start_time_) {
            p_o_ = {d_01_, 0.0};
            dp_o_ = {0.0, 0.0};
            return {0.0, 0.0};
        } else if (t < (leader_start_time_ + go_straight_time_) ) { 
            vd = {v_straight_, 0.0};
            p_o_ = {d_01_, 0.0};
            dp_o_ = {0.0, 0.0};
        } else { 
            vd = {v_straight_, -A_*sin(w_*(t-12.0))}; // pi=3.14

            // 此时po时变
            double theta = std::atan2(vd[1], vd[0]);
            double dtheta = (1.0 / std::pow(std::cos(w_ * (t-12.0)), 2)) * w_;

            p_o_ = {d_01_ * cos(theta), d_01_ * sin(theta)};
            dp_o_ = {-d_01_ * sin(theta) * dtheta, d_01_ * cos(theta) * dtheta};
        }

        // stop signal
        if ( robots_[0].x > 3.8 ) {
            vd = {0.0, 0.0};
        } // stop

        return vd;
    }

    // ======= Coleader速度计算 =======
    Eigen::Vector2d getColeaderCmd(Eigen::Vector2d vd) {
        Eigen::Vector2d u = {0.0, 0.0};

        // 定向变量
        Eigen::Vector2d p_01 = {robots_[0].x - robots_[1].x, robots_[0].y - robots_[1].y};
        Eigen::Vector2d p_o_bar = p_01 - p_o_ ;

        // 刚性变量
        Eigen::Vector2d p_10 = -p_01;
        Eigen::Vector2d p_12 = {robots_[1].x - robots_[2].x, robots_[1].y - robots_[2].y};
        double d_12 = p_12.norm();
        double d_10 = p_10.norm();
        double sigma_10 = d_10 * d_10 - d_01_ * d_01_;
        double sigma_12 = d_12 * d_12 - d_12_ * d_12_;
        Eigen::Vector2d r10 = sigma_10 * p_10;
        Eigen::Vector2d r12 = sigma_12 * p_12;
        Eigen::Vector2d r1 = r10 + r12;

        // control law
        double eta = alpha_ * (p_o_bar.dot(dp_o_)) /
                     std::pow((r1 - alpha_ * p_o_bar).norm(), 2);

        u = -(k_p_ - eta) * (r1 - alpha_ * p_o_bar) + vd;

        return u; // 全局系下的速度
    }

    Eigen::Vector2d sign(const Eigen::Vector2d& v) {
        Eigen::Vector2d res;
        for (int i = 0; i < 2; ++i) {
            if (v[i] > 0) res[i] = 1.0;
            else if (v[i] < 0) res[i] = -1.0;
            else res[i] = 0.0;
        }
        return res;
    }

    // ======= Follower速度计算 =======
    Eigen::Vector2d getFollowerCmd() {
        Eigen::Vector2d u = {0.0, 0.0};

        // 刚性变量
        Eigen::Vector2d p_20 = {robots_[2].x - robots_[0].x, robots_[2].y - robots_[0].y};
        Eigen::Vector2d p_21 = {robots_[2].x - robots_[1].x, robots_[2].y - robots_[1].y};
        double d_21 = p_21.norm();
        double d_20 = p_20.norm();
        double sigma_20 = d_20 * d_20 - d_02_ * d_02_;
        double sigma_21 = d_21 * d_21 - d_12_ * d_12_;
        Eigen::Vector2d r20 = sigma_20 * p_20;
        Eigen::Vector2d r21 = sigma_21 * p_21;
        Eigen::Vector2d r2 = r20 + r21;

        // control law
        u = -k_p_ * r2 - beta_ * sign(r2);

        return u; // 全局系下的速度
    }


    // ======= 发布函数 =======
    void publishCmd(int index, const Eigen::Vector2d& v) {
        geometry_msgs::Twist twist;
        
        // 转换到个体坐标系
        double vx_local =  cos(robots_[index].yaw) * v[0] + sin(robots_[index].yaw) * v[1];
        double vy_local = -sin(robots_[index].yaw) * v[0] + cos(robots_[index].yaw) * v[1];
        double omega = vy_local / akm_offset; // 侧向速度转角速度

        // 限幅
        if (vx_local > robots_[index].max_v) vx_local = robots_[index].max_v;
        if (vx_local < -robots_[index].max_v) vx_local = -robots_[index].max_v;
        if (omega > robots_[index].max_w) omega = robots_[index].max_w;
        if (omega < -robots_[index].max_w) omega = -robots_[index].max_w;

        // 发布
        twist.linear.x = vx_local;
        twist.angular.z = omega;
        robots_[index].cmd_pub.publish(twist);
    }

};
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "formation_node");
    ros::NodeHandle nh;
    FormationNode node(nh);
    ros::spin();
    return 0;
}
