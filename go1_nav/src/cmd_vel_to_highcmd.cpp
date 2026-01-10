#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"

using ros2_unitree_legged_msgs::msg::HighCmd;

class CmdVelToHighCmd : public rclcpp::Node
{
public:
  CmdVelToHighCmd()
  : Node("cmd_vel_to_highcmd")
  {
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelToHighCmd::cmdVelCallback, this, std::placeholders::_1));

    pub_ = create_publisher<HighCmd>("high_cmd", 1);

    initCmd();

    RCLCPP_INFO(get_logger(), "cmd_vel → HighCmd adapter (SLOW & SAFE) started");
  }

private:
  void initCmd()
  {
    cmd_.head[0] = 0xFE;
    cmd_.head[1] = 0xEF;

    cmd_.level_flag = 0;
    cmd_.mode = 2;          // WALK
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

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // SLOW & SAFE limits
    double vx = std::clamp(msg->linear.x, -0.15, 0.15);
    double wz = std::clamp(msg->angular.z, -0.5, 0.5);

    cmd_.velocity[0] = vx;
    cmd_.yaw_speed = wz;

    pub_->publish(cmd_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<HighCmd>::SharedPtr pub_;
  HighCmd cmd_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToHighCmd>());
  rclcpp::shutdown();
  return 0;
}
