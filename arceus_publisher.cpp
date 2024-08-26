#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

class RangePublisher : public rclcpp::Node
{
public:
  RangePublisher() : Node("range_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("range_topic", rclcpp::QoS(rclcpp::KeepLast(10)));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RangePublisher::publish_range, this));
  }

private:

  json us;

  void publish_range()
  {
    std::ifstream input("data.json");

    if (!input.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file data.json");
      return;
    }
    input >> us;
    input.close();

    int range_value = 0;
    try
    {
      range_value = us.at("data").get<int>();;
    }
    catch (const json::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s", e.what());
      return;
    }

    auto message = sensor_msgs::msg::Range();
    message.header.stamp = this->now();
    message.header.frame_id = "sonar_frame";
    message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    message.field_of_view = 0.262;
    message.min_range = 0.2;
    message.max_range = 4.0;
    message.range = static_cast<float>(range_value); // Ensure proper type conversion

    publisher_->publish(message);
  }

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangePublisher>());
  rclcpp::shutdown();
  return 0;
}