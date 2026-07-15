#include <algorithm>
#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
class RacingControlNode : public rclcpp::Node
{
public:
  RacingControlNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
  void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
  void line_following(const ai_msgs::msg::Target &point_msg);
  void timer_callback();
  void sign_callback(const std_msgs::msg::Int32::SharedPtr msg);

  ai_msgs::msg::PerceptionTargets::SharedPtr latest_point_msg_;
  ai_msgs::msg::PerceptionTargets::SharedPtr latest_target_msg_;
  std::mutex point_msg_mutex_;
  std::mutex target_msg_mutex_;

  bool avoid_dir_qrcode_ = false;
  int y_dir_qrcode_ = 0;
  int y_avoid_dir_qrcode_ = 0;

  bool avoid_dir_p_ = false;
  int y_stop_p_ = 0;
  int y_avoid_dir_p_ = 0;

  int avoid_number = 0;
  int current_dir_ = -1;
  int end_y_ = 0;
  int is_avoid_ = 0;
  double angular_z_ = 0.0;
  double line_x_ = 0.0;
  double line_kp_ = 0.0;
  double avoid_x_ = 0.0;
  double avoid_kp_ = 0.0;
  
  double last_point_error_out_ = 0.0;
  double last_avoid_error_out_ = 0.0;
  int avoid_direction_ = -1; // 规避方向 (-1:未初始化, 0:左, 1:右)
  int avoid_counter_ = 0;
  std::string pub_control_topic_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_p_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_subscriber_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_sub_1_;
  rclcpp::TimerBase::SharedPtr timer_;
};

RacingControlNode::RacingControlNode(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name, options)
{
  declare_parameter("pub_control_topic", "cmd_vel");
  declare_parameter("end_y", end_y_);
  declare_parameter("line_x", line_x_);
  declare_parameter("line_kp", line_kp_);
  declare_parameter("avoid_x", avoid_x_);
  declare_parameter("avoid_kp", avoid_kp_);
  declare_parameter("y_stop_p", y_stop_p_);

  get_parameter("pub_control_topic", pub_control_topic_);
  get_parameter("end_y", end_y_);
  get_parameter("line_x", line_x_);
  get_parameter("line_kp", line_kp_);
  get_parameter("avoid_x", avoid_x_);
  get_parameter("avoid_kp", avoid_kp_);
  get_parameter("y_stop_p", y_stop_p_);
  
  declare_parameter("y_dir_qrcode", y_dir_qrcode_);
  get_parameter("y_dir_qrcode", y_dir_qrcode_);

  declare_parameter("y_avoid_dir_p", y_avoid_dir_p_);
  get_parameter("y_avoid_dir_p", y_avoid_dir_p_);

  declare_parameter("y_avoid_dir_qrcode", y_avoid_dir_qrcode_);
  get_parameter("y_avoid_dir_qrcode", y_avoid_dir_qrcode_);

  rclcpp::QoS qos(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  point_subscriber_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_track_center_detection", qos,
      std::bind(&RacingControlNode::subscription_callback_point, this, std::placeholders::_1));
  target_subscriber_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_obstacle_detection", qos,
      std::bind(&RacingControlNode::subscription_callback_target, this, std::placeholders::_1));

  sign_sub_1_ = this->create_subscription<std_msgs::msg::Int32>(
     "/sign4return",10,std::bind(&RacingControlNode::sign_callback, this, std::placeholders::_1));
  publisher_p_ = this->create_publisher<std_msgs::msg::String>("/p", 10);
  publisher_ = create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, qos);
  timer_ = create_wall_timer(std::chrono::milliseconds(30), std::bind(&RacingControlNode::timer_callback, this));
}

void RacingControlNode::sign_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (!msg)
      return;
  if(msg->data == 6)
  {
    avoid_dir_p_ = false;
    avoid_dir_qrcode_ = false;
  }
}

void RacingControlNode::subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(point_msg_mutex_);
  latest_point_msg_ = msg;
}
void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(target_msg_mutex_);
  latest_target_msg_ = msg;
}
void RacingControlNode::timer_callback()
{
  ai_msgs::msg::PerceptionTargets::SharedPtr point_msg;
  ai_msgs::msg::PerceptionTargets::SharedPtr target_msg;
  {
    std::unique_lock<std::mutex> lock(point_msg_mutex_);
    if (latest_point_msg_)
      point_msg = latest_point_msg_;
  }
  {
    std::unique_lock<std::mutex> lock(target_msg_mutex_);
    if (latest_target_msg_)
      target_msg = latest_target_msg_;
  }
  std::vector<ai_msgs::msg::Target> filtered_obstacles;
  std::vector<ai_msgs::msg::Target> filtered_p;
  std::vector<ai_msgs::msg::Target> filtered_qrcode;
  if (target_msg)
  {
    for (const auto &target : target_msg->targets)
    {
      if (target.type == "zt" && target.rois[0].confidence > 0.7)
      {
        filtered_obstacles.push_back(target);
      }
      if (target.type == "p" && target.rois[0].confidence > 0.7)
      {
        filtered_p.push_back(target);
      }
      if (target.type == "qrcode" && target.rois[0].confidence > 0.7)
      {
        filtered_qrcode.push_back(target);
      }
    }
  }

  double center_x_p = 0.0;

  // 停车点
  if (!filtered_p.empty())
  {
    auto max_area_target = std::max_element(
        filtered_p.begin(), filtered_p.end(),
        [](const ai_msgs::msg::Target &a, const ai_msgs::msg::Target &b)
        { return (a.rois[0].rect.width * a.rois[0].rect.height) <
                 (b.rois[0].rect.width * b.rois[0].rect.height); });
    const auto &target = *max_area_target;
    int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
    int obstacle_left = target.rois[0].rect.x_offset;
    int obstacle_right = obstacle_left + target.rois[0].rect.width;
    center_x_p = (obstacle_left + obstacle_right) / 2.0;
    RCLCPP_INFO(this->get_logger(), "end_p:%d obstacle_left:%d obstacle_right:%d center_x_p:%lf",
                bottom, obstacle_left, obstacle_right, center_x_p);
                
    avoid_dir_p_ = true;
    // if (bottom >= y_avoid_dir_p_ && bottom <= 480)
    // {
    //   avoid_dir_p_ = true;
    // }

    if (bottom >= y_stop_p_ && bottom <= 480)
    {
      auto msg_str = std_msgs::msg::String();
      msg_str.data = "1";
      publisher_p_->publish(msg_str);
      return;
    }
  }


  // 锥桶
  if (!filtered_obstacles.empty())
  {
    auto max_area_target = std::max_element(
        filtered_obstacles.begin(), filtered_obstacles.end(),
        [](const ai_msgs::msg::Target &a, const ai_msgs::msg::Target &b)
        { return (a.rois[0].rect.width * a.rois[0].rect.height) <
                 (b.rois[0].rect.width * b.rois[0].rect.height); });
    const auto &target = *max_area_target;
    int obstacle_bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
    int obstacle_left = target.rois[0].rect.x_offset;
    int obstacle_right = obstacle_left + target.rois[0].rect.width;
    double obstacle_center_x = (obstacle_left + obstacle_right) / 2.0;
    RCLCPP_INFO(this->get_logger(), "end_y:%d obstacle_left:%d obstacle_right:%d obstacle_center_x:%lf",
                obstacle_bottom, obstacle_left, obstacle_right, obstacle_center_x);

    int qrcode_bottom = 0;
    double qrcode_center_x = 0;

    if (!filtered_qrcode.empty())
    {
      auto max_area_target = std::max_element(
          filtered_qrcode.begin(), filtered_qrcode.end(),
          [](const ai_msgs::msg::Target &a, const ai_msgs::msg::Target &b)
          { return (a.rois[0].rect.width * a.rois[0].rect.height) <
                  (b.rois[0].rect.width * b.rois[0].rect.height); });
      const auto &target = *max_area_target;
      qrcode_bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
      int qrcode_left = target.rois[0].rect.x_offset;
      int qrcode_right = qrcode_left + target.rois[0].rect.width;
      qrcode_center_x = (qrcode_left + qrcode_right) / 2.0;
      RCLCPP_INFO(this->get_logger(), "end_qrcode:%d qrcode_left:%d qrcode_right:%d qrcode_center_x:%lf",
                  qrcode_bottom, qrcode_left, qrcode_right, qrcode_center_x);

      if (qrcode_bottom >= y_dir_qrcode_ && qrcode_bottom <= 480)
      {
        avoid_dir_qrcode_ = true;
      }
    }

    if (obstacle_bottom >= end_y_ && obstacle_bottom <= 480)
    {
      current_dir_ = -1;
      if (avoid_direction_ == -1 || avoid_counter_ >= 3)
      {
        if (center_x_p != 0)
        {
          current_dir_ = (center_x_p < obstacle_center_x) ? 0 : 1;
        }
        else if (qrcode_bottom != 0 && qrcode_center_x != 0 && qrcode_bottom >= y_avoid_dir_qrcode_ && qrcode_bottom <= 480)
        {
          current_dir_ = (qrcode_center_x < obstacle_center_x) ? 0 : 1;
        }
        else
        {
          current_dir_ = (obstacle_center_x > 300) ? 0 : 1;
        }
        avoid_direction_ = current_dir_;
        avoid_counter_ = 0;
      }
      else
      {
        current_dir_ = avoid_direction_;
        avoid_counter_++;
      }

      double avoid_error_now = 0.0;
      if (current_dir_ == 0)
        avoid_error_now = 640 - obstacle_center_x;
      else
        avoid_error_now = 0 - obstacle_center_x;

      double avoid_error_out = 0.7 * avoid_error_now + 0.3 * last_avoid_error_out_;
      last_avoid_error_out_ = avoid_error_out;
      angular_z_ = avoid_kp_ * avoid_error_out;
      RCLCPP_INFO(this->get_logger(), "error:%lf  avoid_z:%lf", avoid_error_out, angular_z_);
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = avoid_x_;
      twist_msg.angular.z = angular_z_;
      publisher_->publish(twist_msg);
      is_avoid_ = 3;
      return;
    }
    avoid_direction_ = -1;
    last_avoid_error_out_ = 0;
    avoid_counter_ = 0;
  }
  avoid_direction_ = -1;
  last_avoid_error_out_ = 0;
  avoid_counter_ = 0;



  //****************************************************************************************** */
  if (!point_msg || point_msg->targets.empty())
  {
    last_point_error_out_ = 0;
    is_avoid_ = 0;
    avoid_number += 1;
    if (avoid_number == 1)
    {
      if (current_dir_ == 0) // 前面左转，现在右转
      {
        angular_z_ = -0.5;
      }
      else if (current_dir_ == 1) // 前面右转，现在左转
      {
        angular_z_ = 0.5;
      }
    }
    else
    {
      angular_z_ *= 4;
    }
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = avoid_x_;
    twist_msg.angular.z = angular_z_;
    publisher_->publish(twist_msg);
  }
  /***************************************************************************************************** */
  else
  {
    avoid_number = 0;
    const auto &point_target = point_msg->targets[0];
    line_following(point_target);
  }
}

void RacingControlNode::line_following(const ai_msgs::msg::Target &point_msg)
{
  double x = point_msg.points[0].point[0].x;
  double point_error_now = 0.0;
  double point_error_out = 0.0;
  double line_z = 0.0;
  
  if(avoid_dir_p_ == true && avoid_dir_qrcode_ == true)
  {
    point_error_now = 320.0 - x; // 正常
  }
  else
  {
    point_error_now = (320.0 + 50.0) - x; // 黑线偏右
  }
  
  if (std::abs(point_error_now) < 3.0)
  {
    point_error_now = 0.0;
    last_point_error_out_ = 0.0;
  }
  point_error_out = 0.7 * point_error_now + 0.3 * last_point_error_out_;
  line_z = line_kp_ * point_error_out;
 
  /************************************************************************************************ */
  if (is_avoid_ > 0)
  {
    line_z *= 0.5;
    is_avoid_ -= 1;
  }
  /***************************************************************************************************** */

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = line_x_;
  twist_msg.angular.z = line_z;
  publisher_->publish(twist_msg);
  last_point_error_out_ = point_error_out;
}
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacingControlNode>("RacingControlNode"));
  rclcpp::shutdown();
  return 0;
}