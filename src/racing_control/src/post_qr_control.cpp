/**
 * =====================================================================
 * post_qr_control.cpp  - 后QR码控制测试节点（独立版）
 * =====================================================================
 *
 * 【用途】
 * 从racing_control.cpp中独立出来的后QR码状态机控制逻辑。
 * 专门用于调试扫描二维码之后的行为，不包含第一阶段黑线巡线。
 *
 * 【控制流程】
 *   触发条件: 订阅到/Qr_N话题消息
 *   Phase1: 开环向左行驶1.5米（左转绕过通道口）
 *   Phase2: 进入黄色道路绕圈（黄色车道线巡线）
 *   Phase3: 检测到黑线重现 → 黄色巡线结束
 *   Phase4: 黑线稳定后退出后QR模式
 *
 * 【后QR期间额外功能】
 *   黄色巡线期间持续检测锥桶(zt)，遇到锥桶执行避障
 *
 * 【订阅话题】
 *   /Qr_N                           (std_msgs/String)    - QR码触发
 *   /yellow_track_center            (ai_msgs/PerceptionTargets) - OpenCV黄色车道线
 *   /racing_track_center_detection  (ai_msgs/PerceptionTargets) - ResNet黑线(检测重现)
 *   /racing_obstacle_detection      (ai_msgs/PerceptionTargets) - YOLO障碍物(锥桶)
 *
 * 【发布话题】
 *   /cmd_vel                        (geometry_msgs/Twist) - 控制指令(直接发给底盘)
 *
 * 【调试参数(通过launch传入)】
 *   post_qr_forward_speed=0.5        Phase1左转速度(m/s)
 *   post_qr_forward_time_ms=3000     Phase1左转持续时间(ms) 约1.5米
 *   post_qr_forward_angular_z=0.3    Phase1左转角速度(rad/s)
 *   post_qr_turn_speed=0.3           Phase2右转角速度(rad/s)
 *   post_qr_turn_timeout_ms=3000     Phase2右转超时(ms)
 *   post_qr_yellow_area_threshold=0.15 黄色占比阈值≥此值认为进入通道
 *   post_qr_follow_speed=0.4         Phase3黄色巡线速度(m/s)
 *   post_qr_follow_kp=0.005          Phase3黄色巡线比例系数
 *   avoid_x=0.8                     锥桶避障速度(m/s)
 *   avoid_kp=0.0035                 锥桶避障比例系数
 *   end_y=190                       锥桶触发避障的底部y阈值
 * =====================================================================
 */

#include <algorithm>
#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

class PostQRControlNode : public rclcpp::Node
{
public:
  PostQRControlNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // ===================================================================
  // 回调函数声明
  // ===================================================================
  void qrcode_callback(const std_msgs::msg::String::SharedPtr msg);           // QR码触发
  void subscription_callback_yellow_track(const ai_msgs::msg::PerceptionTargets::SharedPtr msg); // 黄色线
  void subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);        // 黑线(检测重现)
  void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);       // 障碍物
  void timer_callback();  // 主循环(30ms)

  // 控制函数
  void yellow_line_following(const ai_msgs::msg::Target &point_msg); // 黄色车道线巡线
  void obstacle_avoidance(const ai_msgs::msg::Target &obstacle);     // 锥桶避障

  // ===================================================================
  // 后QR码状态机
  // ===================================================================
  enum PostQRPhase {
    POST_QR_IDLE = 0,          // 空闲（等待QR码触发）
    POST_QR_FORWARD_LEFT = 1,  // Phase1: 向左前方行驶1.5米（绕过通道口）
    POST_QR_TURN_RIGHT = 2,    // Phase2: 右转对准黄色道路
    POST_QR_YELLOW_FOLLOW = 3, // Phase3: 黄色车道线巡线（持续检测黑线重现 + 锥桶避障）
    POST_QR_BLACK_FOLLOW = 4,  // Phase4: 黑线重现→黑线巡线，稳定后退出
  };
  PostQRPhase post_qr_phase_ = POST_QR_IDLE;
  int post_qr_cycle_ = 0;      // 当前阶段已运行的定时器次数(30ms/cycle)

  // ===================================================================
  // 消息缓存（多线程安全）
  // ===================================================================
  ai_msgs::msg::PerceptionTargets::SharedPtr latest_yellow_track_msg_;
  ai_msgs::msg::PerceptionTargets::SharedPtr latest_point_msg_;
  ai_msgs::msg::PerceptionTargets::SharedPtr latest_target_msg_;
  std::mutex yellow_track_mutex_;
  std::mutex point_msg_mutex_;
  std::mutex target_msg_mutex_;

  // ===================================================================
  // 后QR参数（通过launch传入）
  // ===================================================================
  double post_qr_forward_speed_ = 0.5;          // Phase1左转速度(m/s)
  int post_qr_forward_time_ms_ = 3000;           // Phase1左转时间(ms) 约1.5米
  double post_qr_forward_angular_z_ = 0.3;       // Phase1左转角速度(rad/s)
  double post_qr_turn_speed_ = 0.3;              // Phase2右转角速度(rad/s)
  int post_qr_turn_timeout_ms_ = 3000;           // Phase2右转超时(ms)
  double post_qr_yellow_area_threshold_ = 0.15;  // 黄色占比阈值
  double post_qr_follow_speed_ = 0.4;            // Phase3黄色巡线速度(m/s)
  double post_qr_follow_kp_ = 0.005;             // Phase3黄色巡线比例系数

  // ===================================================================
  // 锥桶避障参数（通过launch传入，后QR期间需要避障）
  // ===================================================================
  double avoid_x_ = 0.8;       // 避障速度(m/s)
  double avoid_kp_ = 0.0035;   // 避障比例系数
  int end_y_ = 190;            // 锥桶触发避障的底部y阈值
  int current_dir_ = -1;       // 当前避障方向(-1=无, 0=左, 1=右)
  int avoid_direction_ = -1;   // 避障方向保持（连续多次同向）
  int avoid_counter_ = 0;      // 同方向避障计数
  double last_avoid_error_out_ = 0.0;  // 上一帧避障误差

  // ===================================================================
  // PID滤波变量
  // ===================================================================
  double last_point_error_out_ = 0.0;  // 上一帧巡线误差

  // ===================================================================
  // ROS话题
  // ===================================================================
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qrcode_sub_;                    // QR码
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr yellow_track_sub_;    // 黄色线
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_sub_;           // 黑线
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_sub_;          // 障碍物
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;                  // 控制指令
  rclcpp::TimerBase::SharedPtr timer_;                                                   // 30ms定时器
};

// =====================================================================
// 构造函数
// =====================================================================
PostQRControlNode::PostQRControlNode(const std::string &node_name, const rclcpp::NodeOptions &options)
  : Node(node_name, options)
{
  // ========== 声明后QR参数 ==========
  declare_parameter("post_qr_forward_speed", post_qr_forward_speed_);
  declare_parameter("post_qr_forward_time_ms", post_qr_forward_time_ms_);
  declare_parameter("post_qr_forward_angular_z", post_qr_forward_angular_z_);
  declare_parameter("post_qr_turn_speed", post_qr_turn_speed_);
  declare_parameter("post_qr_turn_timeout_ms", post_qr_turn_timeout_ms_);
  declare_parameter("post_qr_yellow_area_threshold", post_qr_yellow_area_threshold_);
  declare_parameter("post_qr_follow_speed", post_qr_follow_speed_);
  declare_parameter("post_qr_follow_kp", post_qr_follow_kp_);

  // ========== 声明锥桶避障参数 ==========
  declare_parameter("avoid_x", avoid_x_);
  declare_parameter("avoid_kp", avoid_kp_);
  declare_parameter("end_y", end_y_);

  get_parameter("post_qr_forward_speed", post_qr_forward_speed_);
  get_parameter("post_qr_forward_time_ms", post_qr_forward_time_ms_);
  get_parameter("post_qr_forward_angular_z", post_qr_forward_angular_z_);
  get_parameter("post_qr_turn_speed", post_qr_turn_speed_);
  get_parameter("post_qr_turn_timeout_ms", post_qr_turn_timeout_ms_);
  get_parameter("post_qr_yellow_area_threshold", post_qr_yellow_area_threshold_);
  get_parameter("post_qr_follow_speed", post_qr_follow_speed_);
  get_parameter("post_qr_follow_kp", post_qr_follow_kp_);

  get_parameter("avoid_x", avoid_x_);
  get_parameter("avoid_kp", avoid_kp_);
  get_parameter("end_y", end_y_);

  // 打印参数以便调试
  RCLCPP_INFO(this->get_logger(), "=== 后QR测试节点参数 ===");
  RCLCPP_INFO(this->get_logger(), "  Phase1左转: speed=%.2f z=%.2f time=%dms (≈%.2f米)",
              post_qr_forward_speed_, post_qr_forward_angular_z_,
              post_qr_forward_time_ms_,
              post_qr_forward_speed_ * post_qr_forward_time_ms_ / 1000.0);
  RCLCPP_INFO(this->get_logger(), "  Phase2右转: speed=%.2f timeout=%dms", post_qr_turn_speed_, post_qr_turn_timeout_ms_);
  RCLCPP_INFO(this->get_logger(), "  Phase3黄色巡线: speed=%.2f kp=%.4f 黄色阈值=%.2f",
              post_qr_follow_speed_, post_qr_follow_kp_, post_qr_yellow_area_threshold_);
  RCLCPP_INFO(this->get_logger(), "  锥桶避障: avoid_x=%.2f kp=%.4f end_y=%d",
              avoid_x_, avoid_kp_, end_y_);

  // ========== QoS ==========
  rclcpp::QoS qos(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // ========== 订阅 ==========
  // QR码触发信号
  qrcode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/Qr_N", 10, std::bind(&PostQRControlNode::qrcode_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /Qr_N (QR码触发后QR模式)");

  // OpenCV黄色车道线中心（后QR黄道巡线的核心输入）
  yellow_track_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/yellow_track_center", qos,
      std::bind(&PostQRControlNode::subscription_callback_yellow_track, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /yellow_track_center (OpenCV黄色车道线)");

  // ResNet黑线中心（检测黑线是否重现，用于判断是否离开黄道）
  point_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_track_center_detection", qos,
      std::bind(&PostQRControlNode::subscription_callback_point, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /racing_track_center_detection (黑线，检测重现标志)");

  // YOLO障碍物（后QR黄道绕圈时遇到锥桶也要避障）
  target_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_obstacle_detection", qos,
      std::bind(&PostQRControlNode::subscription_callback_target, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /racing_obstacle_detection (YOLO锥桶，后QR期间避障)");

  // ========== 发布 ==========
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
  RCLCPP_INFO(this->get_logger(), "[发布] /cmd_vel (控制指令)");

  // ========== 定时器(30ms ≈ 33Hz) ==========
  timer_ = create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&PostQRControlNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "=== PostQRControlNode 启动完成，等待QR码触发 ===");
}

// =====================================================================
// QR码检测回调 → 触发后QR状态机
// =====================================================================
void PostQRControlNode::qrcode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || msg->data.empty()) return;

  if (post_qr_phase_ != POST_QR_IDLE)
  {
    RCLCPP_INFO(this->get_logger(), "[QR码] 忽略(%s)，已在后QR模式(phase=%d)",
                msg->data.c_str(), post_qr_phase_);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "========== QR码检测到: '%s' → 进入后QR测试模式 ==========",
              msg->data.c_str());
  post_qr_phase_ = POST_QR_FORWARD_LEFT;
  post_qr_cycle_ = 0;
  last_point_error_out_ = 0.0;
  last_avoid_error_out_ = 0.0;
}

// =====================================================================
// 消息缓存回调
// =====================================================================
void PostQRControlNode::subscription_callback_yellow_track(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(yellow_track_mutex_);
  latest_yellow_track_msg_ = msg;
}
void PostQRControlNode::subscription_callback_point(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(point_msg_mutex_);
  latest_point_msg_ = msg;
}
void PostQRControlNode::subscription_callback_target(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(target_msg_mutex_);
  latest_target_msg_ = msg;
}

// =====================================================================
// 主定时器回调（30ms周期）
// =====================================================================
void PostQRControlNode::timer_callback()
{
  // ---- 获取所有最新消息 ----
  ai_msgs::msg::PerceptionTargets::SharedPtr yellow_track_msg;
  ai_msgs::msg::PerceptionTargets::SharedPtr point_msg;
  ai_msgs::msg::PerceptionTargets::SharedPtr target_msg;
  {
    std::unique_lock<std::mutex> lock(yellow_track_mutex_);
    if (latest_yellow_track_msg_) yellow_track_msg = latest_yellow_track_msg_;
  }
  {
    std::unique_lock<std::mutex> lock(point_msg_mutex_);
    if (latest_point_msg_) point_msg = latest_point_msg_;
  }
  {
    std::unique_lock<std::mutex> lock(target_msg_mutex_);
    if (latest_target_msg_) target_msg = latest_target_msg_;
  }

  // ---- 统一判断状态 ----
  bool black_line_available = (point_msg && !point_msg->targets.empty() &&
                                point_msg->targets[0].points.size() > 0 &&
                                point_msg->targets[0].points[0].point.size() > 0 &&
                                point_msg->targets[0].points[0].point[0].x >= 0);
  bool yellow_line_available = (yellow_track_msg && !yellow_track_msg->targets.empty() &&
                                 yellow_track_msg->targets[0].points.size() > 0 &&
                                 yellow_track_msg->targets[0].points[0].point.size() > 0 &&
                                 yellow_track_msg->targets[0].points[0].point[0].x >= 0);

  // 空闲状态 → 不做任何事
  if (post_qr_phase_ == POST_QR_IDLE) return;

  // ---- 日志 ----
  RCLCPP_INFO(this->get_logger(), "[后QR] phase=%d cycle=%d 黑线=%d 黄色=%d",
              post_qr_phase_, post_qr_cycle_, black_line_available, yellow_line_available);

  // ===================================================================
  // 状态机执行
  // ===================================================================
  switch (post_qr_phase_)
  {
    // ================================================================
    // Phase 1: 向左前方行驶1.5米（左转绕过通道口）
    //   以 post_qr_forward_speed_ 速度 + post_qr_forward_angular_z_ 角速度
    //   持续 post_qr_forward_time_ms_ 毫秒
    //   期间持续检测黄色车道线是否出现
    // ================================================================
    case POST_QR_FORWARD_LEFT:
    {
      int max_cycles = post_qr_forward_time_ms_ / 30;  // 总周期数
      post_qr_cycle_++;

      // 向左前方行驶（正向速度 + 正向角速度=左转）
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = post_qr_forward_speed_;
      twist_msg.angular.z = post_qr_forward_angular_z_;  // 正值 = 左转
      cmd_vel_pub_->publish(twist_msg);
      RCLCPP_INFO(this->get_logger(), "[后QR-左转] cycle=%d/%d v=%.2f z=%.2f 行驶≈%.2f米",
                  post_qr_cycle_, max_cycles, post_qr_forward_speed_,
                  post_qr_forward_angular_z_,
                  post_qr_forward_speed_ * post_qr_cycle_ * 0.03);

      // 提前检测到黄色道路 → 直接进入Phase3黄色巡线
      if (yellow_line_available)
      {
        RCLCPP_INFO(this->get_logger(), "[后QR-左转] 提前检测到黄色道路，进入Phase3黄色巡线");
        post_qr_phase_ = POST_QR_YELLOW_FOLLOW;
        post_qr_cycle_ = 0;
        last_point_error_out_ = 0.0;
        return;
      }

      // 黑线出现 → 直接Phase4
      if (black_line_available)
      {
        RCLCPP_INFO(this->get_logger(), "[后QR-左转] 黑线重现，直接Phase4");
        post_qr_phase_ = POST_QR_BLACK_FOLLOW;
        post_qr_cycle_ = 0;
        last_point_error_out_ = 0.0;
        return;
      }

      // 时间到 → 切换到Phase2右转
      if (post_qr_cycle_ >= max_cycles)
      {
        RCLCPP_INFO(this->get_logger(), "[后QR] 左转完成(%.2f米)，进入Phase2右转对准黄色道路",
                    post_qr_forward_speed_ * post_qr_forward_time_ms_ / 1000.0);
        post_qr_phase_ = POST_QR_TURN_RIGHT;
        post_qr_cycle_ = 0;
      }
      return;
    }

    // ================================================================
    // Phase 2: 右转对准黄色道路
    //   持续右转，实时检测黄色车道线是否出现
    //   如果黑线出现直接跳到Phase4
    // ================================================================
    case POST_QR_TURN_RIGHT:
    {
      int max_cycles = post_qr_turn_timeout_ms_ / 30;
      post_qr_cycle_++;

      // 右转（负角速度 = 右转）
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = -post_qr_turn_speed_;
      cmd_vel_pub_->publish(twist_msg);
      RCLCPP_INFO(this->get_logger(), "[后QR-右转] cycle=%d/%d z=%.2f",
                  post_qr_cycle_, max_cycles, -post_qr_turn_speed_);

      // 黑线出现 → Phase4
      if (black_line_available)
      {
        RCLCPP_INFO(this->get_logger(), "[后QR-右转] 黑线重现，跳过Phase3直接Phase4");
        post_qr_phase_ = POST_QR_BLACK_FOLLOW;
        post_qr_cycle_ = 0;
        last_point_error_out_ = 0.0;
        return;
      }

      // 检测黄色是否出现
      bool yellow_detected = false;
      double yellow_cx = -1.0;
      if (yellow_line_available)
      {
        yellow_cx = yellow_track_msg->targets[0].points[0].point[0].x;
        yellow_detected = true;
      }
      if (yellow_track_msg && !yellow_track_msg->targets.empty() &&
          yellow_track_msg->targets[0].rois.size() > 0)
      {
        double yr = yellow_track_msg->targets[0].rois[0].confidence;
        if (yr >= post_qr_yellow_area_threshold_)
        {
          yellow_detected = true;
          RCLCPP_INFO(this->get_logger(), "[后QR-右转] 黄色面积占比%.2f≥%.2f", yr, post_qr_yellow_area_threshold_);
        }
      }

      if (yellow_detected)
      {
        RCLCPP_INFO(this->get_logger(), "[后QR-右转] 黄色车道线检测到(cx=%.1f) → Phase3黄色巡线", yellow_cx);
        post_qr_phase_ = POST_QR_YELLOW_FOLLOW;
        post_qr_cycle_ = 0;
        last_point_error_out_ = 0.0;
      }
      else if (post_qr_cycle_ >= max_cycles)
      {
        RCLCPP_WARN(this->get_logger(), "[后QR] 右转超时(%dms)未检测到黄色道路，停止！", post_qr_turn_timeout_ms_);
        auto stop_msg = geometry_msgs::msg::Twist();
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_msg);
        post_qr_phase_ = POST_QR_IDLE;
      }
      return;
    }

    // ================================================================
    // Phase 3: 黄色车道线巡线（绕圈）
    //   - 沿着黄色车道线前进（进入黄道绕圈）
    //   - 持续检测黑线是否重现（离开黄道的标志）
    //   - 同时检测锥桶(zt)，遇到锥桶执行避障
    // ================================================================
    case POST_QR_YELLOW_FOLLOW:
    {
      // ---- 黑线重现 → 切Phase4离开黄道 ----
      if (black_line_available)
      {
        RCLCPP_INFO(this->get_logger(), "[后QR] 黄色巡线中检测到黑线重现 → 离开黄道，Phase4");
        post_qr_phase_ = POST_QR_BLACK_FOLLOW;
        post_qr_cycle_ = 0;
        last_point_error_out_ = 0.0;
        return;
      }

      // ---- 锥桶检测（后QR绕圈时也要避障） ----
      bool need_avoid = false;
      ai_msgs::msg::Target obstacle_target;
      if (target_msg)
      {
        for (const auto &t : target_msg->targets)
        {
          if (t.type == "zt" && t.rois[0].confidence > 0.7)
          {
            int obs_bottom = t.rois[0].rect.y_offset + t.rois[0].rect.height;
            if (obs_bottom >= end_y_ && obs_bottom <= 480)
            {
              need_avoid = true;
              obstacle_target = t;
              break;
            }
          }
        }
      }

      if (need_avoid)
      {
        // 执行锥桶避障
        obstacle_avoidance(obstacle_target);
        RCLCPP_INFO(this->get_logger(), "[后QR] 黄色巡线中遇到锥桶，执行避障");
        return;
      }

      // ---- 黄色有效 → 正常黄色巡线 ----
      if (yellow_line_available)
      {
        const auto &yellow_target = yellow_track_msg->targets[0];
        yellow_line_following(yellow_target);
      }
      else
      {
        // 黄色丢失 → 微右转搜索（不后退、不记忆搜索）
        RCLCPP_WARN(this->get_logger(), "[后QR-黄色巡线] 黄色短暂丢失，微右转搜索...");
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = post_qr_follow_speed_ * 0.3;
        twist_msg.angular.z = -post_qr_turn_speed_ * 0.5;
        cmd_vel_pub_->publish(twist_msg);
      }
      post_qr_cycle_++;
      return;
    }

    // ================================================================
    // Phase 4: 黑线重现后的黑线巡线（退出过渡）
    //   运行黑线巡线，稳定10个周期(300ms)后退出后QR模式
    //   如果黑线丢失 → 退回Phase3黄色巡线
    // ================================================================
    case POST_QR_BLACK_FOLLOW:
    {
      if (black_line_available)
      {
        // 使用简单的黑线巡线
        double x = point_msg->targets[0].points[0].point[0].x;
        double black_error_now = 370.0 - x;  // 目标370（偏右）
        double black_error_out = 0.7 * black_error_now + 0.3 * last_point_error_out_;
        double line_z = 0.006 * black_error_out;

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.8;
        twist_msg.angular.z = line_z;
        cmd_vel_pub_->publish(twist_msg);
        last_point_error_out_ = black_error_out;

        post_qr_cycle_++;
        RCLCPP_INFO(this->get_logger(), "[后QR-黑线巡线] 稳定周期%d/10 x=%.1f z=%.3f",
                    post_qr_cycle_, x, line_z);

        if (post_qr_cycle_ >= 10)
        {
          RCLCPP_INFO(this->get_logger(), "========== 后QR模式完成，退出 ==========");
          post_qr_phase_ = POST_QR_IDLE;
          post_qr_cycle_ = 0;
        }
      }
      else
      {
        // 黑线丢失 → 退回Phase3
        RCLCPP_WARN(this->get_logger(), "[后QR] Phase4黑线丢失 → 退回Phase3黄色巡线");
        post_qr_phase_ = POST_QR_YELLOW_FOLLOW;
        post_qr_cycle_ = 0;
        last_point_error_out_ = 0.0;
      }
      return;
    }

    default:
      break;
  }
}

// =====================================================================
// 黄色车道线巡线（PID控制）
// =====================================================================
void PostQRControlNode::yellow_line_following(const ai_msgs::msg::Target &point_msg)
{
  double x = point_msg.points[0].point[0].x;
  double yellow_error_now = 320.0 - x;  // 目标：画面中央320

  // 死区5像素
  if (std::abs(yellow_error_now) < 5.0)
  {
    yellow_error_now = 0.0;
    last_point_error_out_ = 0.0;
  }

  // 一阶滤波 + P控制
  double yellow_error_out = 0.5 * yellow_error_now + 0.5 * last_point_error_out_;
  double line_z = post_qr_follow_kp_ * yellow_error_out;

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = post_qr_follow_speed_;
  twist_msg.angular.z = line_z;
  cmd_vel_pub_->publish(twist_msg);
  last_point_error_out_ = yellow_error_out;

  RCLCPP_INFO(this->get_logger(), "[黄色巡线] x=%.1f err=%.1f out=%.1f z=%.3f v=%.2f",
              x, yellow_error_now, yellow_error_out, line_z, post_qr_follow_speed_);
}

// =====================================================================
// 锥桶避障（用于后QR绕圈时遇到锥桶避让）
// =====================================================================
void PostQRControlNode::obstacle_avoidance(const ai_msgs::msg::Target &obstacle)
{
  int obs_bottom = obstacle.rois[0].rect.y_offset + obstacle.rois[0].rect.height;
  int obs_left = obstacle.rois[0].rect.x_offset;
  int obs_right = obs_left + obstacle.rois[0].rect.width;
  double obs_center_x = (obs_left + obs_right) / 2.0;

  RCLCPP_INFO(this->get_logger(), "[避障] 锥桶 bottom=%d cx=%.1f", obs_bottom, obs_center_x);

  // 决定避障方向
  if (avoid_direction_ == -1 || avoid_counter_ >= 3)
  {
    current_dir_ = (obs_center_x > 300) ? 0 : 1;  // 偏右→左绕, 偏左→右绕
    avoid_direction_ = current_dir_;
    avoid_counter_ = 0;
  }
  else
  {
    current_dir_ = avoid_direction_;
    avoid_counter_++;
  }

  // 计算避障误差
  double avoid_error_now = (current_dir_ == 0)
    ? (640 - obs_center_x) : (0 - obs_center_x);
  double avoid_error_out = 0.7 * avoid_error_now + 0.3 * last_avoid_error_out_;
  last_avoid_error_out_ = avoid_error_out;
  double angular_z = avoid_kp_ * avoid_error_out;

  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = avoid_x_;
  twist.angular.z = angular_z;
  cmd_vel_pub_->publish(twist);

  RCLCPP_INFO(this->get_logger(), "[避障] dir=%s error=%.1f z=%.3f",
              current_dir_ == 0 ? "左绕" : "右绕", avoid_error_out, angular_z);
}

// =====================================================================
// 入口
// =====================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("post_qr_control"), "=== 启动PostQRControlNode ===");
  rclcpp::spin(std::make_shared<PostQRControlNode>("post_qr_control"));
  rclcpp::shutdown();
  return 0;
}