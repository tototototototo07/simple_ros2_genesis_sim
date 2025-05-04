#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class JointPublisher : public rclcpp::Node
{
  public:
    JointPublisher(): Node("franka_joint_publisher"){
      // パラメータ
      this->declare_parameter<std::vector<std::string>>("jnt_names", {""});
      jnt_names = this->get_parameter("jnt_names").as_string_array();
      //if(jnt_names[0]==""){
      //    RCLCPP_WARN(this->get_logger(), "No joints are set to operate in 'parameter.yaml'.");
      //}
    
      publisher = this->create_publisher<sensor_msgs::msg::JointState>("/target_pos", 1);

      msg = sensor_msgs::msg::JointState();
      timer = rclcpp::create_timer(this->get_node_base_interface(), this->get_node_timers_interface(), this->get_clock(),
          std::chrono::milliseconds(100), std::bind(&JointPublisher::timer_callback, this));
    }

  private:
    void timer_callback(){
      sim_time = this->now();
      sec = sim_time.seconds();
      //std::cout << "sec: " << sec << std::endl;
      msg.header.stamp = sim_time; 
      msg.name = jnt_names;
      msg.position = {1.5*cos(sec), 1.5*sin(sec), 1.5*cos(sec), 1.5*(sin(sec)-1.0), 1.5*cos(sec), 0.0, 0.0, 0.0, 0.0}; //目標角の例
      publisher->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    sensor_msgs::msg::JointState msg;
    rclcpp::Time sim_time;

    std::vector<std::string> jnt_names;

    double sec;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPublisher>());
  rclcpp::shutdown();
  return 0;
}