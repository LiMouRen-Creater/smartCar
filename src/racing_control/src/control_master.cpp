#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include "origincar_msg/msg/sign.hpp"
class ChassisController : public rclcpp::Node
{
public:
    ChassisController() : Node("control_master"), sign_value_(0), qrcode_detected_(false), p_detected_(false)
    {
        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qrcode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/Qr_N",10,std::bind(&ChassisController::qrcode_callback, this, std::placeholders::_1));
        p_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/p",10,std::bind(&ChassisController::p_callback, this, std::placeholders::_1));
        sign_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sign4return",10,std::bind(&ChassisController::sign_callback, this, std::placeholders::_1));
        racing_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/racing",qos,std::bind(&ChassisController::racing_callback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30),std::bind(&ChassisController::timer_callback, this));
    }
private:
    void qrcode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        qrcode_detected_ = true;
    }
    void p_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        p_detected_ = true;
    }
    void sign_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        sign_value_ = msg->data;
    }
    void racing_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        racing_cmd_ = *msg;
    }
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        geometry_msgs::msg::Twist output_cmd;
        if (sign_value_ == 5)
        {
            qrcode_detected_ = false;
            p_detected_ = false;
            return;
        }
        if (qrcode_detected_ || p_detected_)
        {
            output_cmd.linear.x = 0.0;
            output_cmd.angular.z = 0.0;
        }
        else if (sign_value_ == 0 || sign_value_ == 6)
        {
            output_cmd = racing_cmd_;
        }
        cmd_vel_pub_->publish(output_cmd);
    }
    std::mutex mutex_;
    int sign_value_;
    geometry_msgs::msg::Twist racing_cmd_ = geometry_msgs::msg::Twist();
    bool qrcode_detected_;
    bool p_detected_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr p_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qrcode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr racing_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisController>());
    rclcpp::shutdown();
    return 0;
}