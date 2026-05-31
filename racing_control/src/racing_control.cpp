/**
 * =====================================================================
 * racing_control.cpp  - 智能小车赛控制系统主节点
 * =====================================================================
 *
 * 订阅话题：
 *   /racing_track_center_detection  (ai_msgs/PerceptionTargets) - ResNet黑线中心
 *   /racing_obstacle_detection      (ai_msgs/PerceptionTargets) - YOLO障碍物(锥桶/p/qrcode)
 *   /yellow_track_center            (ai_msgs/PerceptionTargets) - OpenCV黄色车道线中心
 *   /Qr_N                           (std_msgs/String)           - QR码内容
 *   /sign4return                    (std_msgs/Int32)            - 遥杆控制信号
 *
 * 发布话题：
 *   /racing                         (geometry_msgs/Twist)       - 控制指令
 *   /p                              (std_msgs/String)           - 停车信号
 *   /post_qr_active                 (std_msgs/Bool)             - 后QR模式激活标志
 *
 * 工作模式：
 *   1. 正常模式：黑线巡线 + 锥桶避障 + 停车点 + QR码检测
 *      - 黑线存在 → 黑线巡线
 *      - 黑线丢失但有黄色线 → 黄色线fallback巡线
 *      - 两者都丢 → 记忆搜索（反打方向盘）
 *   2. 后QR模式：QR扫码后 -> 直行 -> 右转 -> 黄色巡线 -> 黑线重现 -> IDLE
 *      - 后QR期间不触发记忆搜索
 * =====================================================================
 */

#include <algorithm>
#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

class RacingControlNode : public rclcpp::Node
{
public:
  RacingControlNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // ===================================================================
  // 回调函数声明
  // ===================================================================
  void subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);    // 黑线中心
  void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);   // 障碍物
  void subscription_callback_yellow_track(const ai_msgs::msg::PerceptionTargets::SharedPtr msg); // 黄色车道线
  void qrcode_callback(const std_msgs::msg::String::SharedPtr msg);  // QR码 → 触发后QR状态机
  void sign_callback(const std_msgs::msg::Int32::SharedPtr msg);     // 遥杆控制
  void timer_callback();  // 主循环(30ms)

  // 巡线函数
  void line_following(const ai_msgs::msg::Target &point_msg);        // 黑线巡线（PID）
  void yellow_line_following(const ai_msgs::msg::Target &point_msg); // 黄色车道线巡线（PID）

  // ===================================================================
  // 后QR码状态机
  // ===================================================================
  enum PostQRPhase {
    POST_QR_IDLE = 0,          // 空闲（未进入后QR模式）
    POST_QR_FORWARD = 1,       // Phase1: 开环直行到通道口
    POST_QR_TURN_RIGHT = 2,    // Phase2: 右转对准黄色道路
    POST_QR_YELLOW_FOLLOW = 3, // Phase3: 黄色车道线巡线（持续检测黑线是否重现）
    POST_QR_BLACK_FOLLOW = 4   // Phase4: 黑线重现→黑线巡线，稳定后退出后QR模式
  };
  PostQRPhase post_qr_phase_ = POST_QR_IDLE;  // 当前阶段
  int post_qr_cycle_ = 0;                      // 当前阶段已运行的定时器次数(30ms/cycle)
  bool post_qr_yellow_seen_ = false;           // 调试用：是否曾看到黄色道路

  // ===================================================================
  // 消息缓存（带互斥锁，多线程安全）
  // ===================================================================
  ai_msgs::msg::PerceptionTargets::SharedPtr latest_point_msg_;      // 黑线中心缓存
  ai_msgs::msg::PerceptionTargets::SharedPtr latest_target_msg_;     // 障碍物缓存
  ai_msgs::msg::PerceptionTargets::SharedPtr latest_yellow_track_msg_; // 黄色线缓存
  std::mutex point_msg_mutex_;
  std::mutex target_msg_mutex_;
  std::mutex yellow_track_mutex_;

  // ===================================================================
  // 状态标志
  // ===================================================================
  bool avoid_dir_qrcode_ = false;  // 避障方向由QR码决定
  int y_dir_qrcode_ = 0;           // QR码进入避障方向的纵向阈值(像素)
  int y_avoid_dir_qrcode_ = 0;     // QR码方向判断的纵向阈值
  bool avoid_dir_p_ = false;       // 避障方向由停车点决定
  int y_stop_p_ = 0;               // 停车点触发阈值(像素)
  int y_avoid_dir_p_ = 0;          // 停车点方向判断的纵向阈值
  int avoid_number = 0;            // 记忆搜索计数（黑线丢失后反打方向盘的次数）
  int current_dir_ = -1;           // 当前避障方向 (-1=无, 0=左, 1=右)
  int end_y_ = 0;                  // 锥桶底部y坐标阈值（触发避障）
  int is_avoid_ = 0;               // 避障后渐变恢复正常转向的计数
  double angular_z_ = 0.0;         // 当前角速度

  // ===================================================================
  // 控制参数（通过launch文件传入）
  // ===================================================================
  double line_x_ = 0.0;            // 黑线巡线速度(m/s)
  double line_kp_ = 0.0;           // 黑线巡线比例系数
  double avoid_x_ = 0.0;           // 避障速度(m/s)
  double avoid_kp_ = 0.0;          // 避障比例系数
  double last_point_error_out_ = 0.0;   // 上一帧巡线误差（一阶滤波）
  double last_avoid_error_out_ = 0.0;   // 上一帧避障误差（一阶滤波）
  int avoid_direction_ = -1;       // 避障方向保持（连续多次同向）
  int avoid_counter_ = 0;          // 同方向避障计数

  // ===================================================================
  // 后QR参数（全部通过launch可调，便于现场调试）
  // ===================================================================
  double post_qr_forward_speed_ = 0.5;         // Phase1直行速度(m/s)
  int post_qr_forward_time_ms_ = 3000;          // Phase1直行总时间(ms)，约1.5米
  double post_qr_turn_speed_ = 0.3;             // Phase2右转角速度(rad/s)
  int post_qr_turn_timeout_ms_ = 3000;          // Phase2右转超时(ms)，超时则停止
  double post_qr_yellow_area_threshold_ = 0.15; // 黄色像素占比≥15%认为进入黄色通道
  double post_qr_follow_speed_ = 0.4;           // Phase3黄色巡线速度(m/s)
  double post_qr_follow_kp_ = 0.005;            // Phase3黄色巡线比例系数

  // ===================================================================
  // ROS话题
  // ===================================================================
  std::string pub_control_topic_;  // 发布控制指令的话题名
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_p_;         // /p(停车信号)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr post_qr_active_pub_;    // /post_qr_active
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_subscriber_;        // 黑线
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;       // 障碍物
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr yellow_track_subscriber_; // 黄色线
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qrcode_sub_;      // QR码
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;     // 控制指令
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_sub_1_;       // 遥杆信号
  rclcpp::TimerBase::SharedPtr timer_;                                    // 30ms定时器
};

// =====================================================================
// 构造函数：参数声明、订阅/发布初始化、定时器启动
// =====================================================================
RacingControlNode::RacingControlNode(const std::string &node_name, const rclcpp::NodeOptions &options)
  : Node(node_name, options)
{
  // ========== 声明参数（黑线巡线 + 避障）==========
  declare_parameter("pub_control_topic", "cmd_vel");
  declare_parameter("end_y", end_y_);
  declare_parameter("line_x", line_x_);
  declare_parameter("line_kp", line_kp_);
  declare_parameter("avoid_x", avoid_x_);
  declare_parameter("avoid_kp", avoid_kp_);
  declare_parameter("y_stop_p", y_stop_p_);
  declare_parameter("y_dir_qrcode", y_dir_qrcode_);
  declare_parameter("y_avoid_dir_p", y_avoid_dir_p_);
  declare_parameter("y_avoid_dir_qrcode", y_avoid_dir_qrcode_);

  get_parameter("pub_control_topic", pub_control_topic_);
  get_parameter("end_y", end_y_);
  get_parameter("line_x", line_x_);
  get_parameter("line_kp", line_kp_);
  get_parameter("avoid_x", avoid_x_);
  get_parameter("avoid_kp", avoid_kp_);
  get_parameter("y_stop_p", y_stop_p_);
  get_parameter("y_dir_qrcode", y_dir_qrcode_);
  get_parameter("y_avoid_dir_p", y_avoid_dir_p_);
  get_parameter("y_avoid_dir_qrcode", y_avoid_dir_qrcode_);
  RCLCPP_INFO(this->get_logger(), "[参数] 黑线: line_x=%.2f kp=%.4f", line_x_, line_kp_);
  RCLCPP_INFO(this->get_logger(), "[参数] 避障: avoid_x=%.2f kp=%.4f end_y=%d", avoid_x_, avoid_kp_, end_y_);
  RCLCPP_INFO(this->get_logger(), "[参数] 停车: y_stop_p=%d  方向: y_dir_qrcode=%d y_avoid_dir_p=%d y_avoid_dir_qrcode=%d",
              y_stop_p_, y_dir_qrcode_, y_avoid_dir_p_, y_avoid_dir_qrcode_);

  // ========== 声明后QR参数 ==========
  declare_parameter("post_qr_forward_speed", post_qr_forward_speed_);
  declare_parameter("post_qr_forward_time_ms", post_qr_forward_time_ms_);
  declare_parameter("post_qr_turn_speed", post_qr_turn_speed_);
  declare_parameter("post_qr_turn_timeout_ms", post_qr_turn_timeout_ms_);
  declare_parameter("post_qr_yellow_area_threshold", post_qr_yellow_area_threshold_);
  declare_parameter("post_qr_follow_speed", post_qr_follow_speed_);
  declare_parameter("post_qr_follow_kp", post_qr_follow_kp_);

  get_parameter("post_qr_forward_speed", post_qr_forward_speed_);
  get_parameter("post_qr_forward_time_ms", post_qr_forward_time_ms_);
  get_parameter("post_qr_turn_speed", post_qr_turn_speed_);
  get_parameter("post_qr_turn_timeout_ms", post_qr_turn_timeout_ms_);
  get_parameter("post_qr_yellow_area_threshold", post_qr_yellow_area_threshold_);
  get_parameter("post_qr_follow_speed", post_qr_follow_speed_);
  get_parameter("post_qr_follow_kp", post_qr_follow_kp_);

  // 打印后QR参数以便调试
  RCLCPP_INFO(this->get_logger(), "=== 后QR参数(通过launch调节) ===");
  RCLCPP_INFO(this->get_logger(), "  直行: speed=%.2f m/s  time=%d ms (距离=%.2f m)",
              post_qr_forward_speed_, post_qr_forward_time_ms_,
              post_qr_forward_speed_ * post_qr_forward_time_ms_ / 1000.0);
  RCLCPP_INFO(this->get_logger(), "  右转: speed=%.2f rad/s  timeout=%d ms",
              post_qr_turn_speed_, post_qr_turn_timeout_ms_);
  RCLCPP_INFO(this->get_logger(), "  黄色检测阈值: %.2f  黄色巡线: speed=%.2f kp=%.4f",
              post_qr_yellow_area_threshold_, post_qr_follow_speed_, post_qr_follow_kp_);

  // ========== QoS设置（Best Effort，图像数据不需要可靠传输）==========
  rclcpp::QoS qos(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // ========== 订阅 ==========
  // 1) ResNet黑线中心检测
  point_subscriber_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_track_center_detection", qos,
      std::bind(&RacingControlNode::subscription_callback_point, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /racing_track_center_detection (ResNet黑线)");

  // 2) YOLO障碍物检测（锥桶、停车点p、QR码）
  target_subscriber_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_obstacle_detection", qos,
      std::bind(&RacingControlNode::subscription_callback_target, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /racing_obstacle_detection (YOLO障碍物)");

  // 3) OpenCV黄色车道线中心（备用巡线 + 后QR检测黄色通道）
  yellow_track_subscriber_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/yellow_track_center", qos,
      std::bind(&RacingControlNode::subscription_callback_yellow_track, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /yellow_track_center (OpenCV黄色车道线)");

  // 4) QR码内容 → 触发后QR状态机
  qrcode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/Qr_N", 10, std::bind(&RacingControlNode::qrcode_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /Qr_N (QR码内容 → 触发后QR状态机)");

  // 5) 遥杆控制信号
  sign_sub_1_ = this->create_subscription<std_msgs::msg::Int32>(
      "/sign4return", 10, std::bind(&RacingControlNode::sign_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[订阅] /sign4return (遥杆控制)");

  // ========== 发布 ==========
  publisher_p_ = this->create_publisher<std_msgs::msg::String>("/p", 10);
  publisher_ = create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, qos);
  post_qr_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/post_qr_active", 10);
  RCLCPP_INFO(this->get_logger(), "[发布] %s (控制指令)", pub_control_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "[发布] /p (停车信号)");
  RCLCPP_INFO(this->get_logger(), "[发布] /post_qr_active (后QR模式标志)");

  // ========== 定时器(30ms ≈ 33Hz) ==========
  timer_ = create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&RacingControlNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "=== RacingControlNode 启动完成(30ms周期) ===");
}

// =====================================================================
// sign4return回调
//   6 = 结束遥杆控制（恢复自动模式）
//   5 = 开启遥杆控制
//   0/-1 = 正常模式
// =====================================================================
void RacingControlNode::sign_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (!msg) return;
  RCLCPP_INFO(this->get_logger(), "[sign4return] 收到信号: %d", msg->data);
  if (msg->data == 6)
  {
    avoid_dir_p_ = false;
    avoid_dir_qrcode_ = false;
    if (post_qr_phase_ != POST_QR_IDLE)
    {
      RCLCPP_INFO(this->get_logger(), "[sign4return] 6 → 退出后QR模式");
      post_qr_phase_ = POST_QR_IDLE;
      // 通知control_master恢复控制
      auto deact_msg = std_msgs::msg::Bool();
      deact_msg.data = false;
      post_qr_active_pub_->publish(deact_msg);
    }
  }
}

// =====================================================================
// QR码检测回调 → 触发后QR状态机
// 当识别到QR码后，小车进入：直行 → 右转 → 黄色巡线 → 黑线重现 → IDLE
// =====================================================================
void RacingControlNode::qrcode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || msg->data.empty()) return;

  // 只有不在后QR模式时才触发，避免重复触发
  if (post_qr_phase_ != POST_QR_IDLE)
  {
    RCLCPP_INFO(this->get_logger(), "[QR码] 忽略(%s)，已在后QR模式(phase=%d)", msg->data.c_str(), post_qr_phase_);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "========== QR码检测到: '%s' → 进入后QR状态机 ==========", msg->data.c_str());
  post_qr_phase_ = POST_QR_FORWARD;
  post_qr_cycle_ = 0;
  post_qr_yellow_seen_ = false;
  last_point_error_out_ = 0.0;  // 重置PID滤波

  // 通知control_master：后QR模式激活，不要覆盖racing_control的cmd_vel
  auto active_msg = std_msgs::msg::Bool();
  active_msg.data = true;
  post_qr_active_pub_->publish(active_msg);
  RCLCPP_INFO(this->get_logger(), "[后QR] 已通知control_master(active=true)");
}

// =====================================================================
// 订阅消息缓存回调（简单加锁保存最新消息）
// =====================================================================
void RacingControlNode::subscription_callback_point(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(point_msg_mutex_);
  latest_point_msg_ = msg;
}
void RacingControlNode::subscription_callback_target(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(target_msg_mutex_);
  latest_target_msg_ = msg;
}
void RacingControlNode::subscription_callback_yellow_track(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(yellow_track_mutex_);
  latest_yellow_track_msg_ = msg;
}

// =====================================================================
// 主定时器回调（30ms周期，核心控制逻辑）
// =====================================================================
void RacingControlNode::timer_callback()
{
  // ---- 获取所有最新消息（一次性加锁读取） ----
  ai_msgs::msg::PerceptionTargets::SharedPtr point_msg;
  ai_msgs::msg::PerceptionTargets::SharedPtr target_msg;
  ai_msgs::msg::PerceptionTargets::SharedPtr yellow_track_msg;
  {
    std::unique_lock<std::mutex> lock(point_msg_mutex_);
    if (latest_point_msg_) point_msg = latest_point_msg_;
  }
  {
    std::unique_lock<std::mutex> lock(target_msg_mutex_);
    if (latest_target_msg_) target_msg = latest_target_msg_;
  }
  {
    std::unique_lock<std::mutex> lock(yellow_track_mutex_);
    if (latest_yellow_track_msg_) yellow_track_msg = latest_yellow_track_msg_;
  }

  // ---- 统一判断黑线和黄色线是否可用 ----
  bool black_line_available = (point_msg && !point_msg->targets.empty() &&
                                point_msg->targets[0].points.size() > 0 &&
                                point_msg->targets[0].points[0].point.size() > 0 &&
                                point_msg->targets[0].points[0].point[0].x >= 0);
  bool yellow_line_available = (yellow_track_msg && !yellow_track_msg->targets.empty() &&
                                 yellow_track_msg->targets[0].points.size() > 0 &&
                                 yellow_track_msg->targets[0].points[0].point.size() > 0 &&
                                 yellow_track_msg->targets[0].points[0].point[0].x >= 0);

  // ===================================================================
  // 【后QR状态机】
  // QR码检测到 → Phase1直行 → Phase2右转 → Phase3黄色巡线 → Phase4黑线重现 → IDLE
  // 后QR期间不触发记忆搜索（反打方向盘）
  // ===================================================================
  if (post_qr_phase_ != POST_QR_IDLE)
  {
    RCLCPP_INFO(this->get_logger(), "[后QR] phase=%d cycle=%d 黑线=%d 黄色=%d",
                post_qr_phase_, post_qr_cycle_, black_line_available, yellow_line_available);

    switch (post_qr_phase_)
    {
      // ==============================================================
      // Phase 1: 开环直行到通道口
      // 向前行走 post_qr_forward_time_ms 毫秒（默认3000ms @ 0.5m/s ≈ 1.5米）
      // 如果提前检测到黄色道路则提前切换到Phase2
      // ==============================================================
      case POST_QR_FORWARD:
      {
        int max_cycles = post_qr_forward_time_ms_ / 30;  // 总周期数
        post_qr_cycle_++;

        // 直行
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = post_qr_forward_speed_;
        twist_msg.angular.z = 0.0;
        publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "[后QR-直行] cycle=%d/%d v=%.2f 行驶距离≈%.2f米",
                    post_qr_cycle_, max_cycles, post_qr_forward_speed_,
                    post_qr_forward_speed_ * post_qr_cycle_ * 0.03);

        // 检查是否提前检测到黄色道路
        bool early_yellow = false;
        if (yellow_track_msg && !yellow_track_msg->targets.empty() &&
            yellow_track_msg->targets[0].rois.size() > 0)
        {
          double yellow_ratio = yellow_track_msg->targets[0].rois[0].confidence;
          if (yellow_ratio >= post_qr_yellow_area_threshold_)
          {
            early_yellow = true;
            RCLCPP_INFO(this->get_logger(),
                        "[后QR-直行] 提前检测到黄色道路(面积占比%.2f≥%.2f)，提前进入Phase2",
                        yellow_ratio, post_qr_yellow_area_threshold_);
          }
        }

        // 如果黑线已经出现（通道口有黑线），直接跳到Phase4
        if (black_line_available)
        {
          RCLCPP_INFO(this->get_logger(), "[后QR-直行] 黑线已出现，跳过Phase2/3直接Phase4黑线巡线");
          post_qr_phase_ = POST_QR_BLACK_FOLLOW;
          post_qr_cycle_ = 0;
          last_point_error_out_ = 0.0;
          return;
        }

        // 到达时间或提前检测到黄色 → 切Phase2
        if (post_qr_cycle_ >= max_cycles || early_yellow)
        {
          RCLCPP_INFO(this->get_logger(), "[后QR] 直行完成，进入Phase2右转对准黄色道路");
          post_qr_phase_ = POST_QR_TURN_RIGHT;
          post_qr_cycle_ = 0;
        }
        return;
      }

      // ==============================================================
      // Phase 2: 右转对准黄色道路
      // 持续右转，实时检测黄色车道线是否出现
      // 如果黑线出现（通道口有黑线），直接跳到Phase4
      // ==============================================================
      case POST_QR_TURN_RIGHT:
      {
        int max_cycles = post_qr_turn_timeout_ms_ / 30;  // 超时周期数
        post_qr_cycle_++;

        // 右转（负角速度 = 右转）
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = -post_qr_turn_speed_;
        publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "[后QR-右转] cycle=%d/%d z=%.2f",
                    post_qr_cycle_, max_cycles, -post_qr_turn_speed_);

        // 黑线出现 → 直接跳到Phase4（不用管黄色了）
        if (black_line_available)
        {
          RCLCPP_INFO(this->get_logger(), "[后QR-右转] 黑线重现，跳过Phase3直接Phase4黑线巡线");
          post_qr_phase_ = POST_QR_BLACK_FOLLOW;
          post_qr_cycle_ = 0;
          last_point_error_out_ = 0.0;
          return;
        }

        // 检测黄色是否出现（综合判断：中心点有效 + 面积占比达阈值）
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
            RCLCPP_INFO(this->get_logger(), "[后QR-右转] 黄色面积占比%.2f≥%.2f",
                        yr, post_qr_yellow_area_threshold_);
          }
        }

        if (yellow_detected)
        {
          post_qr_yellow_seen_ = true;
          RCLCPP_INFO(this->get_logger(), "[后QR-右转] 黄色车道线检测到(cx=%.1f) → 进入Phase3黄色巡线",
                      yellow_cx);
          post_qr_phase_ = POST_QR_YELLOW_FOLLOW;
          post_qr_cycle_ = 0;
          last_point_error_out_ = 0.0;
        }
        else if (post_qr_cycle_ >= max_cycles)
        {
          // 超时未检测到黄色 → 停止
          RCLCPP_WARN(this->get_logger(), "[后QR] 右转超时(%dms)未检测到黄色道路，停止！请检查黄色检测阈值",
                      post_qr_turn_timeout_ms_);
          auto stop_msg = geometry_msgs::msg::Twist();
          stop_msg.linear.x = 0.0;
          stop_msg.angular.z = 0.0;
          publisher_->publish(stop_msg);
          post_qr_phase_ = POST_QR_IDLE;
          auto deact_msg = std_msgs::msg::Bool();
          deact_msg.data = false;
          post_qr_active_pub_->publish(deact_msg);
        }
        return;
      }

      // ==============================================================
      // Phase 3: 黄色车道线巡线（持续检测黑线是否重现）
      // 沿着黄色车道线前进，实时检测黑线数据
      // 一旦黑线重现（说明到了通道口有黑线标记）→ 切Phase4黑线巡线
      // ==============================================================
      case POST_QR_YELLOW_FOLLOW:
      {
        // 黑线重现 → 切Phase4
        if (black_line_available)
        {
          RCLCPP_INFO(this->get_logger(), "[后QR] 黄色巡线中检测到黑线重现 → 切Phase4黑线巡线");
          post_qr_phase_ = POST_QR_BLACK_FOLLOW;
          post_qr_cycle_ = 0;
          last_point_error_out_ = 0.0;
          return;
        }

        // 黄色有效 → 正常巡线
        if (yellow_line_available)
        {
          const auto &yellow_target = yellow_track_msg->targets[0];
          yellow_line_following(yellow_target);
        }
        else
        {
          // 黄色短暂丢失 → 微右转搜索（不启用记忆搜索）
          RCLCPP_WARN(this->get_logger(), "[后QR] 黄色巡线中短暂丢失(cycle=%d)，微右转搜索", post_qr_cycle_);
          auto twist_msg = geometry_msgs::msg::Twist();
          twist_msg.linear.x = post_qr_follow_speed_ * 0.3;  // 低速前进
          twist_msg.angular.z = -post_qr_turn_speed_ * 0.5;  // 微右转
          publisher_->publish(twist_msg);
        }
        post_qr_cycle_++;
        return;
      }

      // ==============================================================
      // Phase 4: 黑线重现后的黑线巡线（后QR模式退出过渡）
      // 运行黑线巡线，稳定10个周期(300ms)后退出后QR模式
      // 如果黑线又丢失 → 退回Phase3黄色巡线
      // ==============================================================
      case POST_QR_BLACK_FOLLOW:
      {
        if (black_line_available)
        {
          const auto &point_target = point_msg->targets[0];
          line_following(point_target);
          post_qr_cycle_++;
          RCLCPP_INFO(this->get_logger(), "[后QR-黑线巡线] 稳定周期%d/10", post_qr_cycle_);

          // 连续10周期(300ms)黑线稳定 → 退出后QR模式
          if (post_qr_cycle_ >= 10)
          {
            RCLCPP_INFO(this->get_logger(), "========== 后QR模式完成，恢复control_master控制 ==========");
            post_qr_phase_ = POST_QR_IDLE;
            auto deact_msg = std_msgs::msg::Bool();
            deact_msg.data = false;
            post_qr_active_pub_->publish(deact_msg);
          }
        }
        else
        {
          // 黑线又丢失 → 退回Phase3黄色巡线
          RCLCPP_WARN(this->get_logger(), "[后QR] Phase4黑线巡线中丢失 → 退回Phase3黄色巡线");
          post_qr_phase_ = POST_QR_YELLOW_FOLLOW;
          post_qr_cycle_ = 0;
          last_point_error_out_ = 0.0;
        }
        return;
      }

      default:
        break;
    }
    return;  // 后QR模式处理完毕，不再执行下面的正常逻辑
  }

  // ===================================================================
  // 【正常模式】（post_qr_phase_ == POST_QR_IDLE）
  // 第一阶段赛道：黑线巡线 + 锥桶避障 + 停车点 + QR码
  //
  // 优先级：
  //   1. 停车点(p) → 发布/p=1信号让control_master停止
  //   2. 锥桶(zt)  → 避障绕行
  //   3. 黑线存在   → 黑线巡线
  //   4. 黑线丢失   → 黄色线fallback（如果有）
  //   5. 都丢失     → 记忆搜索（反打方向盘）
  // ===================================================================
  RCLCPP_INFO(this->get_logger(), "[正常] 黑线=%d 黄色=%d avoid_number=%d",
              black_line_available, yellow_line_available, avoid_number);

  // ---- 解析障碍物检测消息 ----
  std::vector<ai_msgs::msg::Target> filtered_obstacles;  // 锥桶 zt
  std::vector<ai_msgs::msg::Target> filtered_p;          // 停车点 p
  std::vector<ai_msgs::msg::Target> filtered_qrcode;     // QR码 qrcode
  if (target_msg)
  {
    for (const auto &target : target_msg->targets)
    {
      // 【调试】打印所有检测到的目标
      RCLCPP_INFO(this->get_logger(), "[YOLO] type=%s conf=%.2f rect=[%d,%d,%dx%d]",
                  target.type.c_str(), target.rois[0].confidence,
                  target.rois[0].rect.x_offset, target.rois[0].rect.y_offset,
                  target.rois[0].rect.width, target.rois[0].rect.height);

      if (target.type == "zt" && target.rois[0].confidence > 0.7)
        filtered_obstacles.push_back(target);
      if (target.type == "p" && target.rois[0].confidence > 0.7)
        filtered_p.push_back(target);
      if (target.type == "qrcode" && target.rois[0].confidence > 0.7)
        filtered_qrcode.push_back(target);
    }
  }

  double center_x_p = 0.0;  // 需要在外部声明，锥桶避障会用到

  // ---- 停车点检测 (p) ----
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
    center_x_p = (obstacle_left + obstacle_right) / 2.0;  // 移除double，使用外部声明的变量
    RCLCPP_INFO(this->get_logger(), "[停车点] bottom=%d left=%d right=%d cx=%.1f",
                bottom, obstacle_left, obstacle_right, center_x_p);
    avoid_dir_p_ = true;

    if (bottom >= y_stop_p_ && bottom <= 480)
    {
      RCLCPP_INFO(this->get_logger(), "+++++ 停车! bottom=%d >= y_stop_p=%d +++++", bottom, y_stop_p_);
      auto msg_str = std_msgs::msg::String();
      msg_str.data = "1";
      publisher_p_->publish(msg_str);
      return;
    }
  }

  // ---- 锥桶避障 (zt) ----
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
    RCLCPP_INFO(this->get_logger(), "[锥桶] bottom=%d left=%d right=%d cx=%.1f end_y=%d",
                obstacle_bottom, obstacle_left, obstacle_right, obstacle_center_x, end_y_);

    // 解析QR码位置用于避障方向选择
    int qrcode_bottom = 0;
    double qrcode_center_x = 0;
    if (!filtered_qrcode.empty())
    {
      auto max_area_target_qr = std::max_element(
          filtered_qrcode.begin(), filtered_qrcode.end(),
          [](const ai_msgs::msg::Target &a, const ai_msgs::msg::Target &b)
          { return (a.rois[0].rect.width * a.rois[0].rect.height) <
                   (b.rois[0].rect.width * b.rois[0].rect.height); });
      const auto &t = *max_area_target_qr;
      qrcode_bottom = t.rois[0].rect.y_offset + t.rois[0].rect.height;
      qrcode_center_x = (t.rois[0].rect.x_offset + t.rois[0].rect.x_offset + t.rois[0].rect.width) / 2.0;
      if (qrcode_bottom >= y_dir_qrcode_ && qrcode_bottom <= 480)
        avoid_dir_qrcode_ = true;
    }

    // 锥桶接近到阈值 → 执行避障
    if (obstacle_bottom >= end_y_ && obstacle_bottom <= 480)
    {
      // 决定避障方向（左绕=0 或 右绕=1）
      current_dir_ = -1;
      if (avoid_direction_ == -1 || avoid_counter_ >= 3)
      {
        if (center_x_p != 0)
          current_dir_ = (center_x_p < obstacle_center_x) ? 0 : 1; // 停车点在锥桶哪边就往哪边绕
        else if (qrcode_bottom != 0 && qrcode_center_x != 0 &&
                 qrcode_bottom >= y_avoid_dir_qrcode_ && qrcode_bottom <= 480)
          current_dir_ = (qrcode_center_x < obstacle_center_x) ? 0 : 1;
        else
          current_dir_ = (obstacle_center_x > 300) ? 0 : 1; // 锥桶偏右→左绕, 偏左→右绕
        avoid_direction_ = current_dir_;
        avoid_counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "[避障] 首次决定方向: %s", current_dir_ == 0 ? "左绕" : "右绕");
      }
      else
      {
        current_dir_ = avoid_direction_;
        avoid_counter_++;
        RCLCPP_INFO(this->get_logger(), "[避障] 保持方向%d次: %s", avoid_counter_, current_dir_ == 0 ? "左绕" : "右绕");
      }

      // 计算避障误差
      double avoid_error_now = (current_dir_ == 0)
        ? (640 - obstacle_center_x) : (0 - obstacle_center_x);
      double avoid_error_out = 0.7 * avoid_error_now + 0.3 * last_avoid_error_out_;
      last_avoid_error_out_ = avoid_error_out;
      angular_z_ = avoid_kp_ * avoid_error_out;
      RCLCPP_INFO(this->get_logger(), "[避障] error=%.1f out=%.1f z=%.3f v=%.2f",
                  avoid_error_now, avoid_error_out, angular_z_, avoid_x_);
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = avoid_x_;
      twist.angular.z = angular_z_;
      publisher_->publish(twist);
      is_avoid_ = 3;  // 避障后3个周期减弱转向
      return;
    }
    avoid_direction_ = -1;
    last_avoid_error_out_ = 0;
    avoid_counter_ = 0;
  }
  avoid_direction_ = -1;
  last_avoid_error_out_ = 0;
  avoid_counter_ = 0;

  // ---- 黑线巡线（优先级：黑线 > 黄色线 > 记忆搜索） ----
  if (black_line_available)
  {
    // 有黑线 → 正常黑线巡线（同时清除记忆搜索计数）
    avoid_number = 0;
    const auto &point_target = point_msg->targets[0];
    line_following(point_target);
    RCLCPP_INFO(this->get_logger(), "[正常] 黑线巡线中 x=%.1f", point_target.points[0].point[0].x);
  }
  else if (yellow_line_available)
  {
    // 黑线丢失但有黄色车道线 → 黄色fallback巡线（不开启记忆搜索反打）
    RCLCPP_WARN(this->get_logger(), "[正常-黄色fallback] 黑线丢失，切换到黄色车道线巡线");
    const auto &yellow_target = yellow_track_msg->targets[0];
    yellow_line_following(yellow_target);
  }
  else
  {
    // 两者都丢失 → 记忆搜索（反打方向盘找回黑线）
    RCLCPP_WARN(this->get_logger(), "[正常-记忆搜索] 黑线和黄色都丢失! avoid_number=%d 开始反打方向盘", avoid_number);
    last_point_error_out_ = 0;
    is_avoid_ = 0;
    avoid_number += 1;
    if (avoid_number == 1)
    {
      // 第一次丢失：根据之前的方向反打
      angular_z_ = (current_dir_ == 0) ? -0.5 : 0.5;
      RCLCPP_INFO(this->get_logger(), "[记忆搜索] 第1次: 反打 z=%.2f (基于dir=%d)", angular_z_, current_dir_);
    }
    else
    {
      // 后续：加大转向幅度
      angular_z_ *= 4;
      RCLCPP_INFO(this->get_logger(), "[记忆搜索] 第%d次: 加大幅度 z=%.2f", avoid_number, angular_z_);
    }
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = avoid_x_;
    twist.angular.z = angular_z_;
    publisher_->publish(twist);
  }
}

// =====================================================================
// 黑线巡线（PID控制，一阶滤波）
// 目标：将黑线中心控制在图像(320+50)=370像素处（偏右）
//       当ping和qrcode同时存在时控制在320（画面中心）
// =====================================================================
void RacingControlNode::line_following(const ai_msgs::msg::Target &point_msg)
{
  double x = point_msg.points[0].point[0].x;
  double point_error_now = 0.0;
  double point_error_out = 0.0;
  double line_z = 0.0;

  // 根据标志位选择目标中心
  if (avoid_dir_p_ && avoid_dir_qrcode_)
    point_error_now = 320.0 - x;       // 正常：控制到画面中央
  else
    point_error_now = (320.0 + 50.0) - x;  // 偏右：控制到370，让车身偏右

  // 死区：3像素内忽略误差（防抖动）
  if (std::abs(point_error_now) < 3.0)
  {
    point_error_now = 0.0;
    last_point_error_out_ = 0.0;
  }

  // 一阶低通滤波 + P控制
  point_error_out = 0.7 * point_error_now + 0.3 * last_point_error_out_;
  line_z = line_kp_ * point_error_out;

  // 避障后的渐变恢复
  if (is_avoid_ > 0)
  {
    line_z *= 0.5;
    is_avoid_ -= 1;
  }

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = line_x_;
  twist_msg.angular.z = line_z;
  publisher_->publish(twist_msg);
  last_point_error_out_ = point_error_out;

  RCLCPP_INFO(this->get_logger(), "[黑线巡线] x=%.1f target=%.0f err=%.1f out=%.1f z=%.3f",
              x, (avoid_dir_p_ && avoid_dir_qrcode_) ? 320.0 : 370.0,
              point_error_now, point_error_out, line_z);
}

// =====================================================================
// 黄色车道线巡线（PID控制）
// 目标：将黄色车道线中心控制在图像中央(320)
// 两种使用场景：
//   A) 后QR Phase3：使用 post_qr_follow_kp_ 和 post_qr_follow_speed_
//   B) 第一阶段fallback：使用 line_kp_*0.8 和 line_x_*0.7（降速降增益）
// =====================================================================
void RacingControlNode::yellow_line_following(const ai_msgs::msg::Target &point_msg)
{
  double x = point_msg.points[0].point[0].x;
  double yellow_error_now = 320.0 - x;  // 目标：画面中央320

  // 死区5像素（OpenCV检测噪声比ResNet大）
  if (std::abs(yellow_error_now) < 5.0)
  {
    yellow_error_now = 0.0;
    last_point_error_out_ = 0.0;
  }

  // 判断是后QR模式还是第一阶段fallback → 使用不同参数
  bool is_post_qr = (post_qr_phase_ == POST_QR_YELLOW_FOLLOW);
  double kp = is_post_qr ? post_qr_follow_kp_ : (line_kp_ * 0.8);
  double speed = is_post_qr ? post_qr_follow_speed_ : (line_x_ * 0.7);
  double filter_now = is_post_qr ? 0.5 : 0.6;      // 后QR模式更平滑(0.5/0.5)
  double filter_last = is_post_qr ? 0.5 : 0.4;     // fallback略微增大响应(0.6/0.4)

  double yellow_error_out = filter_now * yellow_error_now + filter_last * last_point_error_out_;
  double line_z = kp * yellow_error_out;

  if (is_avoid_ > 0)
  {
    line_z *= 0.5;
    is_avoid_ -= 1;
  }

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = speed;
  twist_msg.angular.z = line_z;
  publisher_->publish(twist_msg);
  last_point_error_out_ = yellow_error_out;

  // 始终打印黄色巡线信息以便调试
  std::string tag = is_post_qr ? "[黄色巡线-后QR]" : "[黄色巡线-fallback]";
  RCLCPP_INFO(this->get_logger(), "%s x=%.1f target=320 err=%.1f out=%.1f z=%.3f v=%.2f kp=%.4f",
              tag.c_str(), x, yellow_error_now, yellow_error_out, line_z, speed, kp);
}

// =====================================================================
// 入口
// =====================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("racing_control"), "=== 启动RacingControlNode ===");
  rclcpp::spin(std::make_shared<RacingControlNode>("RacingControlNode"));
  rclcpp::shutdown();
  return 0;
}