#include "publisher_joint_target.hpp"
#include <mutex>
#include <atomic>

JointPublisher::JointPublisher(): Node("snake_joint_publisher"){
    // パラメータ
    this->declare_parameter<std::vector<std::string>>("jnt_names", {""});
    jnt_names = this->get_parameter("jnt_names").as_string_array();

    this->declare_parameter<double>("link_lengh", 0.090);
    link_lengh = this->get_parameter("link_lengh").as_double(); 
    
    this->declare_parameter<int>("timer_period_ms", 50);
    this->declare_parameter<double>("s_h", 0.0);
    this->declare_parameter<double>("psi", 0.0);
    this->declare_parameter<double>("radius", 2.0);
    this->declare_parameter<double>("d_s_h", 0.5);
    this->declare_parameter<double>("d_psi", 3.14);
    this->declare_parameter<double>("radius_threshold", 0.8);
    this->declare_parameter<double>("d_radius_large", 0.5);
    this->declare_parameter<double>("d_radius_small", 0.1);
    this->declare_parameter<double>("d_ratio", 0.25);
    timer_period_ms   = this->get_parameter("timer_period_ms").as_int();
    s_h               = this->get_parameter("s_h").as_double(); 
    psi               = this->get_parameter("psi").as_double(); 
    radius            = this->get_parameter("radius").as_double(); 
    d_s_h             = this->get_parameter("d_s_h").as_double(); 
    d_psi             = this->get_parameter("d_psi").as_double(); 
    radius_threshold  = this->get_parameter("radius_threshold").as_double(); 
    d_radius_large    = this->get_parameter("d_radius_large").as_double(); 
    d_radius_small    = this->get_parameter("d_radius_small").as_double(); 
    d_ratio           = this->get_parameter("d_ratio").as_double(); 

    
    snake_gait = std::make_unique<SnakeGait>(jnt_names.size(), link_lengh);

    target_pos.resize(jnt_names.size(), 0.0);
    current_pos.resize(jnt_names.size(), 0.0);
    switch_gait_flag = false;

    msg = sensor_msgs::msg::JointState();
    publisher = this->create_publisher<sensor_msgs::msg::JointState>("/target_pos", 1);

    // コールバックグループの作成
    callback_group_sub_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_timer = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options_1;
    sub_options_1.callback_group = callback_group_sub_1;
    rclcpp::SubscriptionOptions sub_options_2;
    sub_options_2.callback_group = callback_group_sub_2;

    sub_joint_state = this->create_subscription<sensor_msgs::msg::JointState>("/joint_state", 10, std::bind(&JointPublisher::sub_joint_state_callback, this, std::placeholders::_1), sub_options_1);
    sub_gait_cmd = this->create_subscription<std_msgs::msg::String>("gait_cmd", 10, std::bind(&JointPublisher::gait_cmd_callback, this, std::placeholders::_1), sub_options_2);
    sub_motion_cmd = this->create_subscription<std_msgs::msg::Int8MultiArray>("motion_cmd", 10, std::bind(&JointPublisher::motion_cmd_callback, this, std::placeholders::_1), sub_options_2);

    timer_motion_pub = rclcpp::create_timer(this->get_node_base_interface(), this->get_node_timers_interface(), this->get_clock(),
                        std::chrono::milliseconds(timer_period_ms), std::bind(&JointPublisher::timer_motion_callback, this), callback_group_timer);
    timer_motion_pub->cancel();

    timer_switch_gait = rclcpp::create_timer(this->get_node_base_interface(), this->get_node_timers_interface(), this->get_clock(),
                        std::chrono::milliseconds(timer_period_ms), std::bind(&JointPublisher::timer_switch_gait_callback, this), callback_group_timer);
    timer_switch_gait->cancel();
}


void JointPublisher::publish_target_pos(){
    sim_time = this->now();
    msg.header.stamp = sim_time;
    msg.name = jnt_names;
    msg.position = target_pos;
    publisher->publish(msg);
}

void JointPublisher::sub_joint_state_callback(const sensor_msgs::msg::JointState & msg){
    std::lock_guard<std::mutex> lock(mtx_current_pos);
    current_pos = msg.position;
}

void JointPublisher::gait_cmd_callback(const std_msgs::msg::String & msg){
    if(!switch_gait_flag){
        switch_gait_flag = true;
        switch_gait_ratio = 0.0;
        std::lock_guard<std::mutex> lock_current(mtx_current_pos);
        if(gait_mode != msg.data){
            gait_mode = msg.data;
            snake_gait->ready_next_gait(gait_mode, current_pos, s_h, psi, radius);
        } else {
            RCLCPP_INFO(this->get_logger(), "Invert shape");
            snake_gait->invert_gait(gait_mode, current_pos, s_h, psi, radius);
        }
        timer_motion_pub->cancel();
        timer_switch_gait->reset();
    }
}

void JointPublisher::motion_cmd_callback(const std_msgs::msg::Int8MultiArray & msg){
    if(!switch_gait_flag){
        std::lock_guard<std::mutex> lock(mtx_motion_cmd);
        motion_cmd = msg.data;
    }
}

void JointPublisher::timer_motion_callback(){
    if(!switch_gait_flag){
        std::lock_guard<std::mutex> lock(mtx_motion_cmd);
        s_h += (gait_mode != pedal_gait_mode) ? motion_cmd[0]*d_s_h * timer_period_ms*1e-3 : motion_cmd[0]*d_s_h*(-1) * timer_period_ms*1e-3;
        psi += motion_cmd[1]*d_psi * timer_period_ms*1e-3;
        if(radius > radius_threshold){
            radius += motion_cmd[2]*d_radius_large * timer_period_ms*1e-3;
        } else {
            radius += motion_cmd[2]*d_radius_small * timer_period_ms*1e-3;
        }
        if(radius < 0){
            radius = d_radius_small * timer_period_ms*1e-3;
        }

        RCLCPP_INFO(this->get_logger(), "s_h: %3.4f, psi: %3.4f, radius: %3.4f", s_h, psi, radius);

        if(gait_mode == pedal_gait_mode){
            target_pos = snake_gait->pedal_gait.calc_joint(s_h, psi);
        } else if(gait_mode == lateral_gait_mode){
            target_pos = snake_gait->lateral_gait.calc_joint(psi, 1/radius);
        } else if(gait_mode == helical_gait_mode){
            target_pos = snake_gait->helical_gait.calc_joint(s_h, psi, radius);
        }
        publish_target_pos();
    }
}

void JointPublisher::timer_switch_gait_callback(){
    switch_gait_ratio += d_ratio * timer_period_ms*1e-3;
    RCLCPP_INFO(this->get_logger(), "switch_gait_ratio: %f", switch_gait_ratio);

    target_pos = snake_gait->switch_or_invert_gait(switch_gait_ratio);
    publish_target_pos();

    if(switch_gait_ratio >= 1.0){
        timer_switch_gait->cancel();
        timer_motion_pub->reset();
        switch_gait_flag = false;
    }
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<JointPublisher>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
