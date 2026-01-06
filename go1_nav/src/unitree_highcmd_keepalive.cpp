#include <rclcpp/rclcpp.hpp>
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"

using ros2_unitree_legged_msgs::msg::HighCmd;
using namespace std::chrono_literals;

class UnitreeHighCmdKeepalive : public rclcpp::Node
{
public:
  UnitreeHighCmdKeepalive()
  : Node("unitree_highcmd_keepalive")
  {
    pub_ = this->create_publisher<HighCmd>("high_cmd", 1);

    timer_ = this->create_wall_timer(
      20ms, std::bind(&UnitreeHighCmdKeepalive::publishCmd, this));

    initCmd();

    RCLCPP_INFO(this->get_logger(),
      "Unitree HighCmd keepalive running (STAND mode, 50Hz)");
  }

private:
  void initCmd()
  {
    cmd_.head[0] = 0xFE;
    cmd_.head[1] = 0xEF;

    // IMPORTANT: match teleop semantics
    cmd_.level_flag = 0;      // works because ros2_udp handles it
    cmd_.mode = 1;            // STAND
    cmd_.gait_type = 1;
    cmd_.speed_level = 0;

    cmd_.foot_raise_height = 0.08;
    cmd_.body_height = 0.0;

    cmd_.euler[0] = 0.0;
    cmd_.euler[1] = 0.0;
    cmd_.euler[2] = 0.0;

    cmd_.velocity[0] = 0.0;
    cmd_.velocity[1] = 0.0;
    cmd_.yaw_speed = 0.0;

    cmd_.reserve = 0;
  }

  void publishCmd()
  {
    pub_->publish(cmd_);
  }

  rclcpp::Publisher<HighCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  HighCmd cmd_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnitreeHighCmdKeepalive>());
  rclcpp::shutdown();
  return 0;
}
