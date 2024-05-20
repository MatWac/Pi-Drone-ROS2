#include <memory>
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
using std::placeholders::_1;

#include <iostream>
#include <chrono>

class PIDController {
  public:
      PIDController(double kp, double ki, double kd)
          : m_kp(kp), m_ki(ki), m_kd(kd), m_lastError(0), m_integral(0){}

      int calculate(double setpoint, double measured_value) {
          using namespace std::chrono;
          auto now = high_resolution_clock::now();
          double time_difference = duration<double>(now - m_lastTime).count();
          m_lastTime = now;

          double error = setpoint - measured_value;
          m_integral += error * time_difference;
          double derivative = (error - m_lastError) / time_difference;
          m_lastError = error;

          return int(m_kp * error + m_ki * m_integral + m_kd * derivative);
      }

  private:
      double m_kp;
      double m_ki;
      double m_kd;
      double m_lastError;
      double m_integral;
      std::chrono::high_resolution_clock::time_point m_lastTime;
};

class PIDPubSub : public rclcpp::Node
{
  public:
    PIDPubSub(): Node("pid_pubsub"), pid_roll(0.8, 0.0, 0.0), pid_pitch(0.8, 0.0, 0.0)
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "angles", 10, std::bind(&PIDPubSub::topic_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("motor", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&PIDPubSub::timer_callback, this));
    }

  private:

    void topic_callback(const geometry_msgs::msg::Vector3 & msg) 
    {
      angles[0] = msg.x;
      angles[1] = msg.y;
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::Int8MultiArray();
      message.data = {pid_pitch.calculate(0.0, abs(angles[0])), pid_pitch.calculate(0.0, abs(angles[1]))};
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d', '%d'", message.data[0], message.data[1]);
      publisher_->publish(message);
    }

    PIDController pid_roll;
    PIDController pid_pitch;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_;
    int angles[2] = {0,0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDPubSub>());
  rclcpp::shutdown();
  return 0;
}