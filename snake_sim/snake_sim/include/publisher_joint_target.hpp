#ifndef PUBLISHER_JOINT_TARGET_HPP
#define PUBLISHER_JOINT_TARGET_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <memory>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "snake_gait.hpp"


class JointPublisher : public rclcpp::Node{
  public:
    JointPublisher();

  private:
    void timer_motion_callback();

    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    sensor_msgs::msg::JointState msg;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;
    rclcpp::Time sim_time;
    double sec;

    std::vector<std::string> jnt_names;
    std::vector<double> target_pos;
    std::vector<double> current_pos;

    void publish_target_pos();
    void sub_joint_state_callback(const sensor_msgs::msg::JointState & msg);




    std::unique_ptr<SnakeGait> snake_gait;

    rclcpp::CallbackGroup::SharedPtr callback_group_sub_1;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_2;
    rclcpp::CallbackGroup::SharedPtr callback_group_timer;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_gait_cmd;
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr sub_motion_cmd;
    rclcpp::TimerBase::SharedPtr timer_motion_pub;
    rclcpp::TimerBase::SharedPtr timer_switch_gait;

    std::string gait_mode = "Idle";
    std::string pedal_gait_mode   = "Pedal";
    std::string lateral_gait_mode = "Lateral";
    std::string helical_gait_mode = "Helical";
    std::atomic<bool> switch_gait_flag;
    double switch_gait_ratio = 0.0;
    std::vector<int8_t> motion_cmd = {0,0,0};

    std::mutex mtx_motion_cmd;
    std::mutex mtx_current_pos;



    // 値はparameter.yamlに記述
    double link_lengh;       // [m]     使用するヘビ型ロボットのリンク長
    int timer_period_ms;     // [msec]
    double s_h;              // [m]     目標曲線上の先頭部の位置
    double psi;              // [rad]　 捻転角
    double radius;           // [m]     円弧・螺旋の半径

    double d_s_h;            // [m/s]   s_hの増減速度
    double d_psi;            // [rad/s] psiの増減角速度
    double radius_threshold; // [m]     常に一定値で半径を変化させると、半径大のときは遅く半径小のときは速すぎるので閾値
    double d_radius_large;   // [m/s]   半径の増減速度
    double d_radius_small;   // [m/s]   
    double d_ratio;          // [1/s]   歩様変更の移行速度



    void gait_cmd_callback(const std_msgs::msg::String & msg);
    void motion_cmd_callback(const std_msgs::msg::Int8MultiArray & msg);
    void timer_switch_gait_callback();

    
};

#endif