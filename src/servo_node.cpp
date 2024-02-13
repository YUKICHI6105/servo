#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <servo/can_utils.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <servo/servo_node.hpp>

class ServoNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr pub_;
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  ServoArray servoArray;

public:
  ServoNode() : Node("servo_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>("can_tx", 10, std::bind(&ServoNode::joy_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<can_plugins2::msg::Frame>("servo", 10);
    this->declare_parameter("velButton", 2);
    this->declare_parameter("disButton", 1);
    // ツイスト型の調査

    this->declare_parameter("ServoButton", rclcpp::PARAMETER_INTEGER_ARRAY); // サーボのボタンのパラメーター
    std::vector<rclcpp::Parameter> servo_button_parameters{rclcpp::Parameter("ServoButton", servoArray.button)};
    this->set_parameters(servo_button_parameters);

    this->declare_parameter("ServoMode", rclcpp::PARAMETER_STRING_ARRAY); // サーボのトグルモードの選択
    std::vector<rclcpp::Parameter> servo_mode_parameters{rclcpp::Parameter("ServoMode", servoArray.mode)};
    this->set_parameters(servo_mode_parameters);
  }
  void toggle(uint32_t channel, const sensor_msgs::msg::Joy::SharedPtr msg);
  void setValue(uint32_t channel);
};

void ServoNode::setValue(uint32_t channel)
{
  if(channel<4){
    servoArray.upperValue[channel] = servoArray.value[channel];
  }else{
    servoArray.lowerValue[channel-4] = servoArray.value[channel];
  }
}

void ServoNode::toggle(uint32_t channel, const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[servoArray.button[channel]])
  {
    if (servoArray.preButton[channel] == 0)
    {
      if (servoArray.value[channel] == 58000)
      {
        servoArray.value[channel] = 58000*40/270;
      }
      else
      {
        servoArray.value[channel] = 58000;
      }
      setValue(channel);
      servoArray.preButton[channel] = 1;
    }
  }else{
    if(servoArray.preButton[channel]==1){
      servoArray.preButton[channel] = 0;
    }
  }
}

void ServoNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[this->get_parameter("velButton").as_int()])
  {
    pub_->publish(get_frame(0x300, 1));
  }
  if (msg->buttons[this->get_parameter("disButton").as_int()])
  {
    pub_->publish(get_frame(0x300, 0));
  }
  for (int i = 0; i < 8; i++)
  {
    if (servoArray.mode[i] == "Toggle")
    {
      toggle(i, msg);
    }
  }
  this->pub_->publish(get_frame(0x301, servoArray.upperValue));
  this->pub_->publish(get_frame(0x302, servoArray.lowerValue));
}

int main(int argc, char *argv[])
{
  // printf("hello world odom_check package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoNode>());
  rclcpp::shutdown();
  return 0;
}