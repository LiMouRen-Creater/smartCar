// racing_control_fsm.cpp
// Finite State Machine for racing control
// States: IDLE, LINE_FOLLOW, QRCODE_SCAN, REVERSE_TURN, TRACK_CHANNEL,
//         YELLOW_ENTER, YELLOW_FOLLOW, GET_PICTURE, TRACK_CHANNEL_END,
//         EXIT_CHANNEL, P_TRACK, P_STOP, FINISH

#include <algorithm>
#include <mutex>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include "origincar_msg/msg/sign.hpp"

// ─────────────────────────────────────────────
// State definitions
// ─────────────────────────────────────────────
enum class State {
    IDLE,               // 待机
    LINE_FOLLOW,        // 巡黑线
    QRCODE_SCAN,        // 扫描二维码，获得 CW(3)/CCW(4)
    REVERSE_TURN,       // 边倒车边右转约90度，IMU判断角度
    TRACK_CHANNEL,      // 追踪"通"字（入口）
    YELLOW_ENTER,       // 根据CW/CCW转向进入黄色通道
    YELLOW_FOLLOW,      // 黄色通道巡线
    GET_PICTURE,        // 触发拍照发给大模型
    TRACK_CHANNEL_END,  // 再次追踪"通"字（出口，近距离阈值）
    EXIT_CHANNEL,       // 出通道，向右前方走，优先追P点否则巡黑线
    P_TRACK,            // 追踪P点
    P_STOP,             // 停车
    FINISH              // 完成
};

// 状态名称，用于info日志
static const char* state_name(State s) {
    switch (s) {
        case State::IDLE:               return "IDLE";
        case State::LINE_FOLLOW:        return "LINE_FOLLOW";
        case State::QRCODE_SCAN:        return "QRCODE_SCAN";
        case State::REVERSE_TURN:       return "REVERSE_TURN";
        case State::TRACK_CHANNEL:      return "TRACK_CHANNEL";
        case State::YELLOW_ENTER:       return "YELLOW_ENTER";
        case State::YELLOW_FOLLOW:      return "YELLOW_FOLLOW";
        case State::GET_PICTURE:        return "GET_PICTURE";
        case State::TRACK_CHANNEL_END:  return "TRACK_CHANNEL_END";
        case State::EXIT_CHANNEL:       return "EXIT_CHANNEL";
        case State::P_TRACK:            return "P_TRACK";
        case State::P_STOP:             return "P_STOP";
        case State::FINISH:             return "FINISH";
        default:                        return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────
// Node
// ─────────────────────────────────────────────
class RacingControlFSM : public rclcpp::Node
{
public:
    RacingControlFSM(const std::string &node_name,
                     const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node(node_name, options)
    {
        // ── 参数声明 ──────────────────────────────
        // 巡黑线
        declare_parameter("line_x",   0.8);
        declare_parameter("line_kp",  0.006);
        // 避障（黑线/黄线通用）
        declare_parameter("avoid_x",  0.8);
        declare_parameter("avoid_kp", 0.0035);
        // 锥桶触发bottom阈值
        declare_parameter("end_y",    190);
        // P点停车bottom阈值
        declare_parameter("y_stop_p", 460);
        // 二维码方向判断bottom阈值
        declare_parameter("y_dir_qrcode",       155);
        declare_parameter("y_avoid_dir_qrcode", 170);
        // 追踪"通"字 bottom阈值（入口/出口）
        declare_parameter("channel_enter_y", 400);  // 入口：到达脚下
        declare_parameter("channel_end_y",   300);  // 出口：近距离才追
        // REVERSE_TURN 目标偏转角度（弧度，约90度）
        declare_parameter("reverse_turn_angle", 1.57);
        // REVERSE_TURN 速度
        declare_parameter("reverse_linear_x",  -0.3); // 倒车
        declare_parameter("reverse_angular_z",  0.5); // 右转
        // YELLOW_ENTER 转向速度
        declare_parameter("yellow_enter_angular_z", 0.5);
        // YELLOW_FOLLOW
        declare_parameter("yellow_x",  0.5);
        declare_parameter("yellow_kp", 0.005);
        // GET_PICTURE 触发一次后自动继续，无需等待
        // EXIT_CHANNEL
        declare_parameter("exit_linear_x",   0.5);
        declare_parameter("exit_angular_z", -0.3); // 向右前方
        // P_TRACK
        declare_parameter("p_track_x",  0.5);
        declare_parameter("p_track_kp", 0.004);

        // ── 获取参数 ──────────────────────────────
        get_parameter("line_x",   line_x_);
        get_parameter("line_kp",  line_kp_);
        get_parameter("avoid_x",  avoid_x_);
        get_parameter("avoid_kp", avoid_kp_);
        get_parameter("end_y",    end_y_);
        get_parameter("y_stop_p", y_stop_p_);
        get_parameter("y_dir_qrcode",       y_dir_qrcode_);
        get_parameter("y_avoid_dir_qrcode", y_avoid_dir_qrcode_);
        get_parameter("channel_enter_y",    channel_enter_y_);
        get_parameter("channel_end_y",      channel_end_y_);
        get_parameter("reverse_turn_angle", reverse_turn_angle_);
        get_parameter("reverse_linear_x",   reverse_linear_x_);
        get_parameter("reverse_angular_z",  reverse_angular_z_);
        get_parameter("yellow_enter_angular_z", yellow_enter_angular_z_);
        get_parameter("yellow_x",  yellow_x_);
        get_parameter("yellow_kp", yellow_kp_);
        get_parameter("exit_linear_x",  exit_linear_x_);
        get_parameter("exit_angular_z", exit_angular_z_);
        get_parameter("p_track_x",  p_track_x_);
        get_parameter("p_track_kp", p_track_kp_);

        // ── QoS ──────────────────────────────────
        rclcpp::QoS qos_be(1);
        qos_be.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        rclcpp::QoS qos_re(1);
        qos_re.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        // ── 订阅 ──────────────────────────────────
        // 黑线中心点（ResNet）
        point_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
            "/racing_track_center_detection", qos_be,
            std::bind(&RacingControlFSM::point_callback, this, std::placeholders::_1));

        // YOLO检测结果（锥桶/P点/二维码/通道）
        target_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
            "/racing_obstacle_detection", qos_be,
            std::bind(&RacingControlFSM::target_callback, this, std::placeholders::_1));

        // 黄色通道error（OpenCV节点）
        yellow_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/yellow_track_center", qos_re,
            std::bind(&RacingControlFSM::yellow_callback, this, std::placeholders::_1));

        // sign_switch：CW(3)/CCW(4)方向指令，以及状态控制
        sign_sub_ = create_subscription<origincar_msg::msg::Sign>(
            "/sign_switch", 10,
            std::bind(&RacingControlFSM::sign_callback, this, std::placeholders::_1));

        // IMU：用于REVERSE_TURN角度判断
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", qos_re,
            std::bind(&RacingControlFSM::imu_callback, this, std::placeholders::_1));

        // ── 发布 ──────────────────────────────────
        racing_pub_  = create_publisher<geometry_msgs::msg::Twist>("/racing", qos_be);
        get_pic_pub_ = create_publisher<std_msgs::msg::Int32>("/get_picture", 10);
        sign4return_pub_ = create_publisher<std_msgs::msg::Int32>("/sign4return", 10);

        // ── 定时器 30ms ───────────────────────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&RacingControlFSM::timer_callback, this));

        RCLCPP_INFO(get_logger(), "RacingControlFSM started, state=IDLE");
    }

private:
    // ─────────────────────────────────────────
    // 状态变量
    // ─────────────────────────────────────────
    State current_state_ = State::IDLE;
    int   yellow_dir_    = 0;    // 3=CW(顺时针), 4=CCW(逆时针)

    // 避障方向记忆
    int    avoid_direction_   = -1; // -1:未初始化, 0:左, 1:右
    int    avoid_counter_     = 0;
    double last_avoid_error_  = 0.0;

    // 巡黑线误差滤波
    double last_line_error_   = 0.0;
    int    avoid_number_      = 0;  // 丢线计数
    int    current_dir_       = -1; // 上次避障方向

    // 黄色通道误差滤波
    double last_yellow_error_ = 0.0;

    // P点追踪误差滤波
    double last_p_error_      = 0.0;

    // REVERSE_TURN：IMU积分
    double imu_yaw_accum_   = 0.0;   // 累计偏转角度（弧度）
    double last_imu_time_   = -1.0;  // 上次IMU时间戳

    // GET_PICTURE：触发标志
    bool get_picture_sent_ = false;

    // 二维码方向标志
    bool avoid_dir_qrcode_ = false;

    // P点方向标志
    bool avoid_dir_p_ = false;

    // ─────────────────────────────────────────
    // 参数
    // ─────────────────────────────────────────
    double line_x_,  line_kp_;
    double avoid_x_, avoid_kp_;
    int    end_y_;
    int    y_stop_p_;
    int    y_dir_qrcode_, y_avoid_dir_qrcode_;
    int    channel_enter_y_, channel_end_y_;
    double reverse_turn_angle_;
    double reverse_linear_x_, reverse_angular_z_;
    double yellow_enter_angular_z_;
    double yellow_x_, yellow_kp_;
    double exit_linear_x_,  exit_angular_z_;
    double p_track_x_, p_track_kp_;

    // ─────────────────────────────────────────
    // 最新消息缓存
    // ─────────────────────────────────────────
    ai_msgs::msg::PerceptionTargets::SharedPtr latest_point_;
    ai_msgs::msg::PerceptionTargets::SharedPtr latest_target_;
    double latest_yellow_error_ = 0.0;
    std::mutex point_mtx_, target_mtx_, yellow_mtx_;

    // ─────────────────────────────────────────
    // ROS接口
    // ─────────────────────────────────────────
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_sub_;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           yellow_sub_;
    rclcpp::Subscription<origincar_msg::msg::Sign>::SharedPtr         sign_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           racing_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                get_pic_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                sign4return_pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;

    // ─────────────────────────────────────────
    // 回调
    // ─────────────────────────────────────────
    void point_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(point_mtx_);
        latest_point_ = msg;
    }
    void target_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(target_mtx_);
        latest_target_ = msg;
    }
    void yellow_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(yellow_mtx_);
        latest_yellow_error_ = msg->data;
    }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!msg) return;
        // 只在REVERSE_TURN时积分角速度
        if (current_state_ != State::REVERSE_TURN) {
            last_imu_time_ = -1.0;
            return;
        }
        double now = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (last_imu_time_ < 0.0) {
            last_imu_time_ = now;
            return;
        }
        double dt = now - last_imu_time_;
        last_imu_time_ = now;
        // angular_velocity.z 右转为负，取绝对值累计
        imu_yaw_accum_ += std::abs(msg->angular_velocity.z) * dt;
        RCLCPP_INFO(get_logger(), "[REVERSE_TURN] imu_yaw_accum=%.3f / target=%.3f",
                    imu_yaw_accum_, reverse_turn_angle_);
    }
    void sign_callback(const origincar_msg::msg::Sign::SharedPtr msg) {
        if (!msg) return;
        int val = msg->sign_data;
        RCLCPP_INFO(get_logger(), "sign_switch received: %d", val);

        if (val == 3 || val == 4) {
            // CW(3) / CCW(4) 二维码方向指令
            yellow_dir_ = val;
            RCLCPP_INFO(get_logger(), "yellow_dir set to %d (%s)",
                        val, val == 3 ? "CW(顺时针)" : "CCW(逆时针)");
        }
        if (val == 1) {
            // 外部触发：开始巡线
            switch_state(State::LINE_FOLLOW);
        }
        if (val == 5) {
            // 复位
            avoid_dir_qrcode_ = false;
            avoid_dir_p_      = false;
            RCLCPP_INFO(get_logger(), "sign=5 reset flags");
        }
        if (val == 10) {
         // 测试用：直接切换到YELLOW_FOLLOW
         RCLCPP_INFO(get_logger(), "sign=10 test: -> YELLOW_FOLLOW");
         switch_state(State::YELLOW_FOLLOW);
        }
    }

    // ─────────────────────────────────────────
    // 状态切换
    // ─────────────────────────────────────────
    void switch_state(State next) {
        RCLCPP_INFO(get_logger(), "State: %s -> %s",
                    state_name(current_state_), state_name(next));
        current_state_ = next;
        // 进入新状态时重置相关变量
        if (next == State::REVERSE_TURN) {
            imu_yaw_accum_ = 0.0;
            last_imu_time_ = -1.0;
        }
        if (next == State::GET_PICTURE) {
            get_picture_sent_ = false;
        }
        if (next == State::LINE_FOLLOW || next == State::YELLOW_FOLLOW) {
            last_line_error_   = 0.0;
            last_yellow_error_ = 0.0;
            avoid_direction_   = -1;
            avoid_counter_     = 0;
        }
    }

    // ─────────────────────────────────────────
    // 发布Twist
    // ─────────────────────────────────────────
    void publish_twist(double linear_x, double angular_z) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x  = linear_x;
        msg.angular.z = angular_z;
        racing_pub_->publish(msg);
    }

    void stop() {
        publish_twist(0.0, 0.0);
    }

    // ─────────────────────────────────────────
    // 辅助：过滤目标
    // ─────────────────────────────────────────
    struct FilteredTargets {
        std::vector<ai_msgs::msg::Target> zt;       // 锥桶
        std::vector<ai_msgs::msg::Target> p;        // P点
        std::vector<ai_msgs::msg::Target> qrcode;   // 二维码
        std::vector<ai_msgs::msg::Target> channel;  // 通道文字
    };

    FilteredTargets filter_targets(
        const ai_msgs::msg::PerceptionTargets::SharedPtr &target_msg)
    {
        FilteredTargets ft;
        if (!target_msg) return ft;
        for (const auto &t : target_msg->targets) {
            if (t.rois.empty() || t.rois[0].confidence < 0.7) continue;
            if (t.type == "zt")      ft.zt.push_back(t);
            else if (t.type == "p")  ft.p.push_back(t);
            else if (t.type == "qrcode") ft.qrcode.push_back(t);
            else if (t.type == "channel") ft.channel.push_back(t);
        }
        return ft;
    }

    // 取面积最大的目标
    static const ai_msgs::msg::Target* max_area_target(
        const std::vector<ai_msgs::msg::Target> &targets)
    {
        if (targets.empty()) return nullptr;
        return &(*std::max_element(targets.begin(), targets.end(),
            [](const ai_msgs::msg::Target &a, const ai_msgs::msg::Target &b) {
                return (a.rois[0].rect.width * a.rois[0].rect.height) <
                       (b.rois[0].rect.width * b.rois[0].rect.height);
            }));
    }

    // ─────────────────────────────────────────
    // 避障公共逻辑
    // 参数 next_target_x: 下一个目标在画面中的x坐标（用于预判方向）
    //      -1 表示不知道，按锥桶位置判断
    // 返回 true 表示正在避障，false 表示无需避障
    // ─────────────────────────────────────────
    bool do_avoid_zt(const FilteredTargets &ft, double linear_x,
                     double next_target_x = -1.0)
    {
        const auto *zt = max_area_target(ft.zt);
        if (!zt) {
            avoid_direction_ = -1;
            last_avoid_error_ = 0.0;
            avoid_counter_   = 0;
            return false;
        }

        int    bottom   = zt->rois[0].rect.y_offset + zt->rois[0].rect.height;
        double center_x = zt->rois[0].rect.x_offset + zt->rois[0].rect.width / 2.0;

        if (bottom < end_y_ || bottom > 480) {
            avoid_direction_ = -1;
            last_avoid_error_ = 0.0;
            avoid_counter_   = 0;
            return false;
        }

        // 确定避障方向
        if (avoid_direction_ == -1 || avoid_counter_ >= 3) {
            if (next_target_x >= 0) {
                // 预判：优先从靠近下一个目标的那侧绕过
                avoid_direction_ = (next_target_x < center_x) ? 0 : 1;
                RCLCPP_INFO(get_logger(),
                    "[AVOID_ZT] next_target_x=%.1f center_x=%.1f -> dir=%s",
                    next_target_x, center_x,
                    avoid_direction_ == 0 ? "LEFT" : "RIGHT");
            } else {
                // 无预判：锥桶在右侧从左绕，在左侧从右绕
                avoid_direction_ = (center_x > 300) ? 0 : 1;
                RCLCPP_INFO(get_logger(),
                    "[AVOID_ZT] center_x=%.1f -> dir=%s",
                    center_x, avoid_direction_ == 0 ? "LEFT" : "RIGHT");
            }
            avoid_counter_ = 0;
        } else {
            avoid_counter_++;
        }

        double error = (avoid_direction_ == 0) ?
                       (640.0 - center_x) : (0.0 - center_x);
        double error_filtered = 0.7 * error + 0.3 * last_avoid_error_;
        last_avoid_error_ = error_filtered;
        double angular_z = avoid_kp_ * error_filtered;

        RCLCPP_INFO(get_logger(),
            "[AVOID_ZT] bottom=%d center_x=%.1f dir=%d error=%.1f angular_z=%.3f",
            bottom, center_x, avoid_direction_, error_filtered, angular_z);

        publish_twist(linear_x, angular_z);
        return true;
    }

    // ─────────────────────────────────────────
    // 主定时器
    // ─────────────────────────────────────────
    void timer_callback()
    {
        // 拷贝最新消息
        ai_msgs::msg::PerceptionTargets::SharedPtr point_msg, target_msg;
        double yellow_error;
        {
            std::lock_guard<std::mutex> lk(point_mtx_);
            point_msg = latest_point_;
        }
        {
            std::lock_guard<std::mutex> lk(target_mtx_);
            target_msg = latest_target_;
        }
        {
            std::lock_guard<std::mutex> lk(yellow_mtx_);
            yellow_error = latest_yellow_error_;
        }

        FilteredTargets ft = filter_targets(target_msg);

        switch (current_state_) {
            case State::IDLE:
                state_idle();
                break;
            case State::LINE_FOLLOW:
                state_line_follow(ft, point_msg);
                break;
            case State::QRCODE_SCAN:
                state_qrcode_scan(ft);
                break;
            case State::REVERSE_TURN:
                state_reverse_turn();
                break;
            case State::TRACK_CHANNEL:
                state_track_channel(ft, false);
                break;
            case State::YELLOW_ENTER:
                state_yellow_enter(yellow_error);
                break;
            case State::YELLOW_FOLLOW:
                state_yellow_follow(ft, yellow_error);
                break;
            case State::GET_PICTURE:
                state_get_picture(ft, yellow_error);
                break;
            case State::TRACK_CHANNEL_END:
                state_track_channel(ft, true);
                break;
            case State::EXIT_CHANNEL:
                state_exit_channel(ft, point_msg);
                break;
            case State::P_TRACK:
                state_p_track(ft);
                break;
            case State::P_STOP:
                state_p_stop();
                break;
            case State::FINISH:
                stop();
                break;
        }
    }

    // ─────────────────────────────────────────
    // 各状态实现
    // ─────────────────────────────────────────

    // IDLE: 等待外部触发（sign=1）
    void state_idle() {
        stop();
    }

    // LINE_FOLLOW: 巡黑线，内部处理避障
    // 避障预判：二维码偏右侧 -> 优先从右侧绕
    void state_line_follow(
        const FilteredTargets &ft,
        const ai_msgs::msg::PerceptionTargets::SharedPtr &point_msg)
    {
        // 检测到二维码，记录位置并切换
        const auto *qr = max_area_target(ft.qrcode);
        if (qr) {
            int bottom     = qr->rois[0].rect.y_offset + qr->rois[0].rect.height;
            double center_x = qr->rois[0].rect.x_offset + qr->rois[0].rect.width / 2.0;
            if (bottom >= y_dir_qrcode_ && bottom <= 480) {
                avoid_dir_qrcode_ = true;
                RCLCPP_INFO(get_logger(),
                    "[LINE_FOLLOW] qrcode detected bottom=%d center_x=%.1f -> QRCODE_SCAN",
                    bottom, center_x);
                switch_state(State::QRCODE_SCAN);
                return;
            }
        }

        // 避障（预判：二维码在右侧，next_target_x=640）
        double next_x = avoid_dir_qrcode_ ? 640.0 : -1.0;
        if (do_avoid_zt(ft, avoid_x_, next_x)) return;

        // 巡黑线
        if (!point_msg || point_msg->targets.empty()) {
            // 丢线处理
            avoid_number_++;
            if (avoid_number_ == 1) {
                double angular_z = (current_dir_ == 0) ? -0.5 : 0.5;
                publish_twist(avoid_x_, angular_z);
            } else {
                publish_twist(avoid_x_, last_line_error_ * 4.0);
            }
            RCLCPP_INFO(get_logger(), "[LINE_FOLLOW] lost line, avoid_number=%d", avoid_number_);
            return;
        }
        avoid_number_ = 0;
        const auto &target = point_msg->targets[0];
        double x = target.points[0].point[0].x;
        double error_now = (avoid_dir_p_ && avoid_dir_qrcode_) ?
                           (320.0 - x) : (370.0 - x);
        if (std::abs(error_now) < 3.0) { error_now = 0.0; last_line_error_ = 0.0; }
        double error_filtered = 0.7 * error_now + 0.3 * last_line_error_;
        last_line_error_ = error_filtered;
        double angular_z = line_kp_ * error_filtered;
        RCLCPP_INFO(get_logger(),
            "[LINE_FOLLOW] x=%.1f error=%.1f angular_z=%.3f", x, error_filtered, angular_z);
        publish_twist(line_x_, angular_z);
    }

    // QRCODE_SCAN: 等待/sign_switch收到3或4
    void state_qrcode_scan(const FilteredTargets &ft) {
        // 等待yellow_dir_被sign_callback设置
        if (yellow_dir_ == 3 || yellow_dir_ == 4) {
            RCLCPP_INFO(get_logger(),
                "[QRCODE_SCAN] got direction %d (%s) -> REVERSE_TURN",
                yellow_dir_, yellow_dir_ == 3 ? "CW(顺时针)" : "CCW(逆时针)");
            // 通知yellow_track_opencv节点方向
            auto sign_msg = origincar_msg::msg::Sign();
            sign_msg.sign_data = yellow_dir_;
            // 通过sign_switch发布（yellow_track_opencv会订阅）
            switch_state(State::REVERSE_TURN);
            return;
        }
        // 停车等待二维码扫描结果
        stop();
        RCLCPP_INFO(get_logger(), "[QRCODE_SCAN] waiting for direction...");
        (void)ft;
    }

    // REVERSE_TURN: 边倒车边右转，IMU判断约90度
    void state_reverse_turn() {
        if (imu_yaw_accum_ >= reverse_turn_angle_) {
            RCLCPP_INFO(get_logger(),
                "[REVERSE_TURN] reached %.3f rad -> TRACK_CHANNEL", imu_yaw_accum_);
            stop();
            switch_state(State::TRACK_CHANNEL);
            return;
        }
        // 边倒车边右转
        publish_twist(reverse_linear_x_, reverse_angular_z_);
    }

    // TRACK_CHANNEL: 追踪"通"字
    // is_end=false 入口（底部阈值channel_enter_y_）
    // is_end=true  出口（近距离阈值channel_end_y_）
    void state_track_channel(const FilteredTargets &ft, bool is_end) {
        int    threshold = is_end ? channel_end_y_ : channel_enter_y_;
        State  next_state = is_end ? State::EXIT_CHANNEL : State::YELLOW_ENTER;
        const char *tag   = is_end ? "TRACK_CHANNEL_END" : "TRACK_CHANNEL";

        // 避障（预判：通道在正前方，next_target_x=320）
        if (do_avoid_zt(ft, avoid_x_, 320.0)) return;

        const auto *ch = max_area_target(ft.channel);
        if (!ch) {
            // 找不到通道文字，慢速前进
            publish_twist(avoid_x_ * 0.5, 0.0);
            RCLCPP_INFO(get_logger(), "[%s] channel not found, moving forward", tag);
            return;
        }

        int    bottom   = ch->rois[0].rect.y_offset + ch->rois[0].rect.height;
        double center_x = ch->rois[0].rect.x_offset + ch->rois[0].rect.width / 2.0;

        RCLCPP_INFO(get_logger(),
            "[%s] channel bottom=%d center_x=%.1f threshold=%d",
            tag, bottom, center_x, threshold);

        // 到达阈值，切换状态
        if (bottom >= threshold && bottom <= 480) {
            RCLCPP_INFO(get_logger(), "[%s] reached threshold -> %s",
                        tag, state_name(next_state));
            stop();
            switch_state(next_state);
            return;
        }

        // 追踪：根据目标位置控制转向
        double error = 320.0 - center_x;
        double angular_z = 0.004 * error;
        publish_twist(avoid_x_ * 0.6, angular_z);
    }

    // YELLOW_ENTER: 根据CW/CCW转向，直到检测到黄色边线
    void state_yellow_enter(double yellow_error) {
        // yellow_error != 0 说明yellow_track_opencv已经检测到边线
        if (std::abs(yellow_error) > 1.0) {
            RCLCPP_INFO(get_logger(),
                "[YELLOW_ENTER] yellow edge detected error=%.1f -> YELLOW_FOLLOW",
                yellow_error);
            switch_state(State::YELLOW_FOLLOW);
            return;
        }
        // 根据方向转向
        // CCW(4)逆时针：右转进入
        // CW(3)顺时针：左转进入
        double angular_z = (yellow_dir_ == 4) ?
                           -yellow_enter_angular_z_ : yellow_enter_angular_z_;
        publish_twist(avoid_x_ * 0.5, angular_z);
        RCLCPP_INFO(get_logger(),
            "[YELLOW_ENTER] dir=%d angular_z=%.3f", yellow_dir_, angular_z);
    }

    // YELLOW_FOLLOW: 黄色通道巡线，内部处理避障
    void state_yellow_follow(const FilteredTargets &ft, double yellow_error) {
        // 检测到出口"通"字（需要近距离阈值）
        const auto *ch = max_area_target(ft.channel);
        if (ch) {
            int bottom = ch->rois[0].rect.y_offset + ch->rois[0].rect.height;
            if (bottom >= channel_end_y_ && bottom <= 480) {
                RCLCPP_INFO(get_logger(),
                    "[YELLOW_FOLLOW] channel_end detected -> TRACK_CHANNEL_END");
                switch_state(State::TRACK_CHANNEL_END);
                return;
            }
        }

        // TODO: GET_PICTURE触发条件（第二个弯前的图文标识牌）
        // 当YOLO检测到图文标识牌时触发，目前留flag位置
        // if (ft.sign_board detected) { switch_state(State::GET_PICTURE); return; }

        // 避障（预判：图文标识牌方向，暂时无预判）
        if (do_avoid_zt(ft, yellow_x_)) return;

        // 黄色通道巡线
        double error_filtered = 0.7 * yellow_error + 0.3 * last_yellow_error_;
        last_yellow_error_ = error_filtered;
        double angular_z = yellow_kp_ * error_filtered;
        RCLCPP_INFO(get_logger(),
            "[YELLOW_FOLLOW] dir=%d(%s) yellow_error=%.1f angular_z=%.3f",
            yellow_dir_, yellow_dir_ == 3 ? "CW" : "CCW",
            error_filtered, angular_z);
        publish_twist(yellow_x_, angular_z);
    }

    // GET_PICTURE: 触发拍照，立即回YELLOW_FOLLOW
    void state_get_picture(const FilteredTargets &ft, double yellow_error) {
        if (!get_picture_sent_) {
            auto msg = std_msgs::msg::Int32();
            msg.data = 1;
            get_pic_pub_->publish(msg);
            get_picture_sent_ = true;
            RCLCPP_INFO(get_logger(), "[GET_PICTURE] /get_picture sent -> YELLOW_FOLLOW");
            switch_state(State::YELLOW_FOLLOW);
        }
        (void)ft; (void)yellow_error;
    }

    // EXIT_CHANNEL: 出通道后向右前方走
    // 优先追P点，否则巡黑线
    void state_exit_channel(
        const FilteredTargets &ft,
        const ai_msgs::msg::PerceptionTargets::SharedPtr &point_msg)
    {
        // 优先追P点
        const auto *p = max_area_target(ft.p);
        if (p) {
            RCLCPP_INFO(get_logger(), "[EXIT_CHANNEL] P detected -> P_TRACK");
            switch_state(State::P_TRACK);
            return;
        }

        // 避障
        if (do_avoid_zt(ft, avoid_x_)) return;

        // 有黑线则巡线
        if (point_msg && !point_msg->targets.empty()) {
            const auto &target = point_msg->targets[0];
            double x = target.points[0].point[0].x;
            double error_now = 370.0 - x;
            if (std::abs(error_now) < 3.0) { error_now = 0.0; last_line_error_ = 0.0; }
            double error_filtered = 0.7 * error_now + 0.3 * last_line_error_;
            last_line_error_ = error_filtered;
            double angular_z = line_kp_ * error_filtered;
            RCLCPP_INFO(get_logger(),
                "[EXIT_CHANNEL] line_follow x=%.1f error=%.1f angular_z=%.3f",
                x, error_filtered, angular_z);
            publish_twist(line_x_, angular_z);
            return;
        }

        // 无黑线也无P点，向右前方慢走
        RCLCPP_INFO(get_logger(), "[EXIT_CHANNEL] no target, moving right-forward");
        publish_twist(exit_linear_x_, exit_angular_z_);
    }

    // P_TRACK: 追踪P点
    void state_p_track(const FilteredTargets &ft) {
        // 避障（预判：P点方向）
        const auto *p = max_area_target(ft.p);
        double next_x = p ?
            (p->rois[0].rect.x_offset + p->rois[0].rect.width / 2.0) : -1.0;
        if (do_avoid_zt(ft, p_track_x_, next_x)) return;

        if (!p) {
            // P点丢失，慢速前进
            publish_twist(p_track_x_ * 0.5, 0.0);
            RCLCPP_INFO(get_logger(), "[P_TRACK] P not found, moving forward");
            return;
        }

        int    bottom   = p->rois[0].rect.y_offset + p->rois[0].rect.height;
        double center_x = p->rois[0].rect.x_offset + p->rois[0].rect.width / 2.0;

        RCLCPP_INFO(get_logger(),
            "[P_TRACK] bottom=%d center_x=%.1f y_stop_p=%d",
            bottom, center_x, y_stop_p_);

        // 到达停车阈值
        if (bottom >= y_stop_p_ && bottom <= 480) {
            RCLCPP_INFO(get_logger(), "[P_TRACK] reached y_stop_p -> P_STOP");
            switch_state(State::P_STOP);
            return;
        }

        // 追踪P点
        double error = 320.0 - center_x;
        double error_filtered = 0.7 * error + 0.3 * last_p_error_;
        last_p_error_ = error_filtered;
        double angular_z = p_track_kp_ * error_filtered;
        publish_twist(p_track_x_, angular_z);
    }

    // P_STOP: 停车
    void state_p_stop() {
        stop();
        // 通知control_master停车
        auto msg = std_msgs::msg::String();
        RCLCPP_INFO(get_logger(), "[P_STOP] stopped -> FINISH");
        switch_state(State::FINISH);
    }
};

// ─────────────────────────────────────────────
// main
// ─────────────────────────────────────────────
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacingControlFSM>("RacingControlFSM"));
    rclcpp::shutdown();
    return 0;
}