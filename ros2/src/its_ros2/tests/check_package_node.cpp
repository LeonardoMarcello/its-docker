#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class CheckNode : public rclcpp::Node {
public:
  CheckNode() : Node("check_package_node") {
    RCLCPP_INFO(this->get_logger(), "Check node starting up");
    pub_ = this->create_publisher<std_msgs::msg::String>("/its_check/topic", 10);
    timer_ = this->create_wall_timer(1s, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "its_ros2: alive";
      pub_->publish(msg);
      RCLCPP_DEBUG(this->get_logger(), "published check message");
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CheckNode>());
  rclcpp::shutdown();
  return 0;
}
