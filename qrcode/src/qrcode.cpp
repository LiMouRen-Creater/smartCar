#include <rclcpp/rclcpp.hpp>

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <string>
#include <stdexcept>


class Qrcode : public rclcpp::Node
{
public:
  Qrcode() : Node("qrcode"), number_i_(0)
  {
    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    subscriber_hbmem_ =
        this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            "/nv12_img",
            qos,
            std::bind(&Qrcode::subscription_callback, this, std::placeholders::_1));

    // 发布转换后的方向编号：3 顺时针，4 逆时针
    qrcode_number_publisher_ =
        this->create_publisher<std_msgs::msg::Int32>("/qrcode_number", 10);

    // 新增：发布二维码原始识别结果
    qrcode_result_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/qrcode_result", 10);

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    RCLCPP_INFO(this->get_logger(), "qrcode node started");
  }

private:
  void subscription_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    if (!msg)
      return;

    int height = msg->height;
    int width = msg->width;
    size_t step = msg->step;

    if (height <= 0 || width <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid image size: width=%d, height=%d", width, height);
      return;
    }

    if (step < static_cast<size_t>(width))
    {
      RCLCPP_WARN(this->get_logger(), "Invalid image step: step=%zu, width=%d", step, width);
      return;
    }

    if (msg->data.size() < step * static_cast<size_t>(height))
    {
      RCLCPP_WARN(this->get_logger(), "Invalid image data size");
      return;
    }

    cv::Mat y_plane(height, width, CV_8UC1, msg->data.data(), step);

    cv::Mat gray;
    if (step == static_cast<size_t>(width) && y_plane.isContinuous())
    {
      gray = y_plane;
    }
    else
    {
      gray = y_plane.clone();
    }

    zbar::Image zbar_image(
        width,
        height,
        "Y800",
        gray.data,
        width * height);

    int result = scanner_.scan(zbar_image);

    if (result <= 0)
      return;

    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end();
         ++symbol)
    {
      std::string qr_data = symbol->get_data();

      std_msgs::msg::Int32 qrcode_number_msg;
      std_msgs::msg::String qrcode_result_msg;

      bool valid_qrcode = true;

      if (qr_data == "ClockWise")
      {
        // 顺时针
        qrcode_number_msg.data = 3;
      }
      else if (qr_data == "AntiClockWise")
      {
        // 逆时针
        qrcode_number_msg.data = 4;
      }
      else
      {
        try
        {
          int number = std::stoi(qr_data);

          if (number >= 1 && number <= 9999)
          {
            // 奇数 -> 顺时针 3
            // 偶数 -> 逆时针 4
            qrcode_number_msg.data = (number % 2 == 0) ? 4 : 3;
          }
          else
          {
            RCLCPP_WARN(
                this->get_logger(),
                "Recognized number out of range (1-9999): %d",
                number);

            valid_qrcode = false;
          }
        }
        catch (const std::invalid_argument &e)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Unrecognized content: %s",
              qr_data.c_str());

          valid_qrcode = false;
        }
        catch (const std::out_of_range &e)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Number out of int range: %s",
              qr_data.c_str());

          valid_qrcode = false;
        }
      }

      if (!valid_qrcode)
        continue;

      // 发布二维码原始结果
      qrcode_result_msg.data = qr_data;
      qrcode_result_publisher_->publish(qrcode_result_msg);

      // 发布转换后的编号结果
      qrcode_number_publisher_->publish(qrcode_number_msg);

      RCLCPP_INFO(
          this->get_logger(),
          "QR raw: %s, number result: %d",
          qr_data.c_str(),
          qrcode_number_msg.data);
    }
  }

private:
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr subscriber_hbmem_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr qrcode_number_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qrcode_result_publisher_;

  zbar::ImageScanner scanner_;

  int number_i_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Qrcode>());
  rclcpp::shutdown();
  return 0;
}