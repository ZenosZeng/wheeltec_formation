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
#include <fstream>

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

class RigidControllerNode {
public:
    RigidControllerNode(ros::NodeHandle& nh) : nh_(nh) {
        // leader
        robots_.push_back(Robot());
        robots_[0].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_1/robot_pose_ekf/odom_combined", 10,
            boost::bind(&RigidControllerNode::poseCallback, this, _1, 0));
        robots_[0].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

        // coleader
        robots_.push_back(Robot());
        robots_[1].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_2/robot_pose_ekf/odom_combined", 10,
            boost::bind(&RigidControllerNode::poseCallback, this, _1, 1));
        robots_[1].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 10);
        robots_[1].init_x = -1.0;
        robots_[1].init_y = -0.5;
        robots_[1].init_yaw = 0.0;

        // follower
        robots_.push_back(Robot());
        robots_[2].pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_3/robot_pose_ekf/odom_combined", 10,
            boost::bind(&RigidControllerNode::poseCallback, this, _1, 2));
        robots_[2].cmd_pub  = nh_.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 10);
        robots_[2].init_x = -1.0;
        robots_[2].init_y = 1.0;
        robots_[2].init_yaw = 0.0;

        // 定时器：统一 control loop
        timer_ = nh_.createTimer(ros::Duration(0.02), &RigidControllerNode::controlLoop, this);

        // 初始化log header
        log_str_rows_.push_back("t,d01_err,d02_err,d12_err,o_error,o_error_deg");
    }

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;

    // 用于记录log csv
    std::vector<std::string> log_str_rows_;

    std::vector<Robot> robots_;

    // 控制参数
    float akm_offset = 0.2 ;

    // coleader参数
    double k_coleader_ = 0.6;
    double alpha_ = 0.6;  // 定向增益 0.6

    // follower参数
    double k_follower_ = 3.0;
    // double beta_ = 0.0;   // SMC
    double beta_soft_ = 0.37; // 0.37
    double tanh_k_ = 4.0;

    // leader 运动参数 vy=-Asin(wt)
    double A_ = 0.06;     // 振幅
    double w_ = 0.5;     // omega
    double v_straight_ = 0.2; // vx
    double init_time_ = 5.0;
    double leader_start_time_ = 5.0;
    double go_straight_time_ = 2.0;

    // 定向向量
    Eigen::Vector2d p_o_;
    Eigen::Vector2d dp_o_;

    // 队形
    double d_01_ = 0.7;
    double d_02_ = 0.7;
    double d_12_ = 0.7;

    // 控制输入
    Eigen::Vector2d u_0_, u_1_, u_2_;
    double t_global_ = 0.0; // time from node start

    // 停止信号，用于dump和stop
    bool stop_signal_ = false;
    bool log_dumped_ = false;

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
        if (!robots_[0].pose_ekf_ready || !robots_[1].pose_ekf_ready || !robots_[2].pose_ekf_ready){
            ROS_INFO_THROTTLE(1, "Waiting for all robots' pose_ekf...");
        }

        // 获取仿真时间
        ros::Time t_now = ros::Time::now();
        static ros::Time t_start = t_now;
        t_global_ = (t_now - t_start).toSec();
        ROS_INFO_ONCE("Formation control started. t=%.2f", t_global_);

        // log 每个车的位置
        // ROS_INFO_THROTTLE(1, "T:%.2f | L: (%.2f, %.2f, %.2f) | C: (%.2f, %.2f, %.2f) | F: (%.2f, %.2f, %.2f)", 
        //     t_global_,
        //     robots_[0].x, robots_[0].y, robots_[0].yaw,
        //     robots_[1].x, robots_[1].y, robots_[1].yaw,
        //     robots_[2].x, robots_[2].y, robots_[2].yaw);

        /**
         * 主控制逻辑
         * p0 静止 输出位置 检查init正确性
         * p1 leader不动 其他先形成队形
         * p2 leader开始动 集体跟踪 (leader先走2s直线,然后sin)
         */

        if ( t_global_ < init_time_) {
            // 小于init时间，静止
            u_0_ = {0.0, 0.0};
            u_1_ = {0.0, 0.0};
            u_2_ = {0.0, 0.0};
        } else {
            // 根据控制律计算控制量
            getLeaderCmd(); // vx vy
            getColeaderCmd();
            getFollowerCmd();
        }

        // 转换成v omega并发布
        publishCmd(0, u_0_);
        publishCmd(1, u_1_);
        publishCmd(2, u_2_);

        // 记录log
        record_to_log_str();
    }

    void record_to_log_str(){
        // distance err
        double e_01 = std::abs(std::hypot(robots_[0].x - robots_[1].x,
                                  robots_[0].y - robots_[1].y) - d_01_);
        double e_02 = std::abs(std::hypot(robots_[0].x - robots_[2].x,
                                  robots_[0].y - robots_[2].y) - d_02_);
        double e_12 = std::abs(std::hypot(robots_[1].x - robots_[2].x,
                                  robots_[1].y - robots_[2].y) - d_12_);
        
        // orientation err
        Eigen::Vector2d p_01 = {robots_[0].x - robots_[1].x,
                                robots_[0].y - robots_[1].y};
        double o_err = (p_01-p_o_).norm();

        double angle1 = std::atan2(p_01.y(), p_01.x());
        double angle2 = std::atan2(p_o_.y(), p_o_.x());
        double o_err_rad = std::abs(angle1 - angle2);
        double o_err_deg = o_err_rad * 180.0 / M_PI;

        // add to str
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << t_global_ << ",";
        oss << std::fixed << std::setprecision(4)
            << e_01 << ","
            << e_02 << ","
            << e_12 << ","
            << o_err << ",";
        oss << std::fixed << std::setprecision(2)
            << o_err_deg;

        log_str_rows_.push_back(oss.str());

        // print in 1Hz
        ROS_INFO_THROTTLE(1, "T:%.2f | Err | d01: %.3f | d02: %.3f | d12: %.3f | o: %.3f | o_deg: %.2f", 
            t_global_, e_01, e_02, e_12, o_err, o_err_deg);

        // 检查停止信号
        if (stop_signal_ && !log_dumped_ ) {
            dump_log("/home/wheeltec/zyl_ws/fmc.log");
            log_dumped_ = true;
        }
    }

    void dump_log(const std::string& filename){
        std::ofstream log_file(filename);
        if (!log_file.is_open()) {
            ROS_ERROR("Failed to open log file: %s", filename.c_str());
            return;
        }

        for (const auto& row : log_str_rows_) {
            log_file << row << "\n";
        }

        log_file.close();
        ROS_INFO("Log file saved: %s", filename.c_str());
    }

    // ======= Leader速度计算 =======
    // 该函数返回leader的期望速度vd，以及更新po dpo
    void getLeaderCmd() {
        // stop signal
        if ( robots_[0].x > 4.0 ) {
            u_0_ = {0.0, 0.0};
            dp_o_ = {0.0, 0.0};
            stop_signal_ = true;
            return;
        }

        // x < stop_x 
        if (t_global_ < leader_start_time_) {
            p_o_ = {d_01_, 0.0};
            dp_o_ = {0.0, 0.0};
            u_0_ = {0.0, 0.0};
            return;
        } else if (t_global_ < (leader_start_time_ + go_straight_time_) ) { 
            u_0_ = {v_straight_, 0.0};
            p_o_ = {d_01_, 0.0};
            dp_o_ = {0.0, 0.0};
            return;
        } else {
            // p2: sin运动 
            double sin_start_time = leader_start_time_ + go_straight_time_;
            double vx = v_straight_;
            double vy = -A_ * sin(w_ * (t_global_ - sin_start_time));
            double dvx = 0.0;
            double dvy = -A_ * w_ * cos(w_ * (t_global_ - sin_start_time));

            u_0_ = {vx, vy};

            double theta = std::atan2(vy, vx);
            double dtheta = (vx * dvy - vy * dvx) / (vx*vx + vy*vy);

            p_o_ = {d_01_ * cos(theta), d_01_ * sin(theta)};
            dp_o_ = {-d_01_ * sin(theta) * dtheta, d_01_ * cos(theta) * dtheta};
            return;
        }
    }

    // ======= Coleader速度计算 =======
    void getColeaderCmd() {
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
        Eigen::Vector2d r1 = r10 ; // + r12

        // control law
        double eta = alpha_ * (p_o_bar.dot(dp_o_)) /
                     std::pow((r1 - alpha_ * p_o_bar).norm(), 2);

        u_1_ = -(k_coleader_ - eta) * (r1 - alpha_ * p_o_bar) + u_0_;
    }

    // Eigen::Vector2d sign(const Eigen::Vector2d& v) {
    //     Eigen::Vector2d res;
    //     for (int i = 0; i < 2; ++i) {
    //         if (v[i] > 0) res[i] = 1.0;
    //         else if (v[i] < 0) res[i] = -1.0;
    //         else res[i] = 0.0;
    //     }
    //     return res;
    // }

    Eigen::Vector2d tanhVec(const Eigen::Vector2d& v) {
        Eigen::Vector2d res;
        for (int i = 0; i < 2; ++i) {
            res[i] = std::tanh(tanh_k_ * v[i]);
        }
        return res;
    }

    // ======= Follower速度计算 =======
    void getFollowerCmd() {
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
        double k_f = k_follower_;
        if( t_global_ < leader_start_time_ + 2.0 ) {
            k_f = 0.5;
        }
        u_2_ = -k_f * r2 - beta_soft_ * tanhVec(r2);
        return;
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
        // if (vx_local < -robots_[index].max_v) vx_local = -robots_[index].max_v;
        if (vx_local < 0.0) vx_local = 0.0; // no reverse

        if (omega > robots_[index].max_w) omega = robots_[index].max_w;
        if (omega < -robots_[index].max_w) omega = -robots_[index].max_w;

        // 发布
        twist.linear.x = vx_local;
        twist.angular.z = omega;
        robots_[index].cmd_pub.publish(twist);
    }

};
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "rigid_controller_node");
    ros::NodeHandle nh;
    RigidControllerNode node(nh);
    ros::spin();
    return 0;
}
