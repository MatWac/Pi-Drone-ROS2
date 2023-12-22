#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/std_msgs/int8_multi_array.hpp"
using std::placeholders::_1;

int* pid(float roll, float pitch, float yaw)
{
  
}

class PIDPubSub : public rclcpp::Node
{
  public:
    PIDPubSub(): Node("pid")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "angles", 10, std::bind(&PIDPubSub::topic_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::std_msgs::Int8MultiArray>("motor", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&PIDPubSub::timer_callback, this));
    }

  private:

    void topic_callback(const geometry_msgs::msg::Vector3 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f','%f", msg.x, msg.y);
      angles[2] = {msg.x, msg.y, 0};
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::std_msgs::Int8MultiArray();
      message.data = 
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::std_msgs::Int8MultiArray>::SharedPtr publisher_;
    int angles[2] = {0,0,0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDPubSub>());
  rclcpp::shutdown();
  return 0;
}