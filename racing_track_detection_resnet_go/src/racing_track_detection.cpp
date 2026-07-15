// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "racing_track_detection/racing_track_detection.h"

#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include "dnn_node/util/image_proc.h"
#include "hobot_cv/hobotcv_imgproc.h"

// =========================================================================
// --- 可以修改的参数区 (兼容 1920x1080 / 640x480 及任意区域裁剪) ---
// =========================================================================
// 左上角为坐标原点，水平向右x轴增大，竖直向下y轴增大
// 左边第一个点是0，右边是640-1
// ResNet可以推理任意尺寸，不一定是224x224

constexpr int IMG_WIDTH = 640;     // 图片的宽 (供参考，实际图像尺寸将自动从 msg 获取)
constexpr int IMG_HEIGHT = 480;    // 图片的高 (供参考，实际图像尺寸将自动从 msg 获取)

constexpr int CROP_LEFT = 0;       // 图片裁剪的左边
constexpr int CROP_RIGHT = 640-1;    // 图片裁剪的右边
constexpr int CROP_TOP = 120-1;      // 图片裁剪的顶部
constexpr int CROP_BOTTOM = 480-1;   // 图片裁剪的底部

// =========================================================================
// 1920x1080参数
//  constexpr int IMG_WIDTH = 1920;     // 图片的宽 (供参考，实际图像尺寸将自动从 msg 获取)
//  constexpr int IMG_HEIGHT = 1080;    // 图片的高 (供参考，实际图像尺寸将自动从 msg 获取)

//  constexpr int CROP_LEFT = 0;       // 图片裁剪的左边
//  constexpr int CROP_RIGHT = 1920-1;    // 图片裁剪的右边
//  constexpr int CROP_TOP = 400-1;      // 图片裁剪的顶部
//  constexpr int CROP_BOTTOM = 1080-1;   // 图片裁剪的底部

// =========================================================================

void prepare_nv12_tensor_without_padding(const char *image_data,
                                         int image_height,
                                         int image_width,
                                         hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_NV12;
  properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = 3;
  valid_shape.dimensionSize[2] = image_height;
  valid_shape.dimensionSize[3] = image_width;

  auto &aligned_shape = properties.alignedShape;
  aligned_shape = valid_shape;

  int32_t image_length = image_height * image_width * 3 / 2;

  hbSysAllocCachedMem(&tensor->sysMem[0], image_length);
  memcpy(tensor->sysMem[0].virAddr, image_data, image_length);

  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
}

void prepare_nv12_tensor_without_padding(int image_height,
                                         int image_width,
                                         hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_NV12;
  properties.tensorLayout = HB_DNN_LAYOUT_NCHW;

  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = 3;
  valid_shape.dimensionSize[2] = image_height;
  valid_shape.dimensionSize[3] = image_width;

  auto &aligned_shape = properties.alignedShape;
  int32_t w_stride = ALIGN_16(image_width);
  aligned_shape.numDimensions = 4;
  aligned_shape.dimensionSize[0] = 1;
  aligned_shape.dimensionSize[1] = 3;
  aligned_shape.dimensionSize[2] = image_height;
  aligned_shape.dimensionSize[3] = w_stride;

  int32_t image_length = image_height * w_stride * 3 / 2;
  hbSysAllocCachedMem(&tensor->sysMem[0], image_length);
}

TrackDetectionNode::TrackDetectionNode(const std::string& node_name,
                                       const NodeOptions& options)
  : DnnNode(node_name, options) {
  this->declare_parameter<std::string>("model_path", model_path_);
  this->declare_parameter<std::string>("sub_img_topic", sub_img_topic_);

  this->get_parameter("model_path", model_path_);
  this->get_parameter("sub_img_topic", sub_img_topic_);

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TrackDetectionNode"), "Init failed!");
  }

  rclcpp::QoS qos(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  publisher_ =
    this->create_publisher<ai_msgs::msg::PerceptionTargets>("racing_track_center_detection_go", 1);
  subscriber_hbmem_ =
    this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
      sub_img_topic_,
      qos,
      std::bind(&TrackDetectionNode::subscription_callback,
      this,
      std::placeholders::_1)); 
}

TrackDetectionNode::~TrackDetectionNode() {

}

int TrackDetectionNode::SetNodePara() {
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("TrackDetectionNode"), "path:%s\n", model_path_.c_str());
  dnn_node_para_ptr_->model_file = model_path_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 4;
  return 0;
}

int TrackDetectionNode::PostProcess(
  const std::shared_ptr<DnnNodeOutput> &outputs) {
  std::shared_ptr<LineCoordinateParser> line_coordinate_parser =
      std::make_shared<LineCoordinateParser>();
  std::shared_ptr<LineCoordinateResult> result =
      std::make_shared<LineCoordinateResult>();
  line_coordinate_parser->Parse(result, outputs->output_tensors[0]);
  float x = result->x;
  float y = result->y;
  RCLCPP_INFO(rclcpp::get_logger("TrackDetectionNode"),
               "post coor x: %d    y:%d", int(x), int(y));
  ai_msgs::msg::PerceptionTargets::UniquePtr msg(
        new ai_msgs::msg::PerceptionTargets());
  msg->set__header(*outputs->msg_header);
  ai_msgs::msg::Target target;
  target.set__type("track_center");
  ai_msgs::msg::Point track_center;

  geometry_msgs::msg::Point32 pt;
  pt.set__x(x);
  pt.set__y(y);
  track_center.point.emplace_back(pt);
  //To display
  track_center.point.emplace_back(pt);
  std::vector<ai_msgs::msg::Point> tar_points;
  tar_points.push_back(track_center);
  target.set__points(tar_points);
  msg->targets.emplace_back(target);
  publisher_->publish(std::move(msg));
  return 0;
}

void TrackDetectionNode::subscription_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg) {
  int ret = 0;
  if (!msg || !rclcpp::ok()) {
    return;
  }
  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(msg->encoding.data()))
     << ", h: " << msg->height << ", w: " << msg->width
     << ", step: " << msg->step << ", index: " << msg->index
     << ", stamp: " << msg->time_stamp.sec << "_"
     << msg->time_stamp.nanosec << ", data size: " << msg->data_size;
  RCLCPP_DEBUG(rclcpp::get_logger("TrackDetectionNode"), "%s", ss.str().c_str());

  auto model_manage = GetModel();
  if (!model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("TrackDetectionNode"), "Invalid model");
    return;
  }

  hbDNNRoi roi;
  roi.left = CROP_LEFT;
  roi.top = CROP_TOP;
  roi.right = CROP_RIGHT - 1;
  roi.bottom = CROP_BOTTOM - 1;
  hbDNNTensor input_tensor;
  // resize
  cv::Mat img_mat(msg->height * 3 / 2, msg->width, CV_8UC1, (void*)(msg->data.data()));
  cv::Range rowRange(CROP_TOP, CROP_BOTTOM);
  cv::Range colRange(CROP_LEFT, CROP_RIGHT);
  cv::Mat crop_img_mat = hobot_cv::hobotcv_crop(img_mat, msg->height, msg->width, 224, 224, rowRange, colRange);

  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(crop_img_mat.data),
      224,
      224,
      224,
      224);
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("TrackDetectionNode"), "Get Nv12 pym fail!");
    return;
  }
  std::vector<std::shared_ptr<DNNInput>> inputs;
  auto rois = std::make_shared<std::vector<hbDNNRoi>>();
  // 注意：这里传给模型的 roi 是相对于裁剪并 resize 后 224x224 的坐标
  roi.left = 0;
  roi.top = 0;
  roi.right = 224;
  roi.bottom = 224;
  rois->push_back(roi);

  for (size_t i = 0; i < rois->size(); i++) {
    for (int32_t j = 0; j < model_manage->GetInputCount(); j++) {
      inputs.push_back(pyramid);
    }
  }

  auto dnn_output = std::make_shared<DnnNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(msg->index));
  dnn_output->msg_header->set__stamp(msg->time_stamp);
  ret = Predict(inputs, dnn_output, rois);
}

int TrackDetectionNode::Predict(
  std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
  const std::shared_ptr<DnnNodeOutput> &output,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois) {
  RCLCPP_INFO(rclcpp::get_logger("TrackDetectionNode"), "input size:%d roi size:%d", dnn_inputs.size(), rois->size());
  return Run(dnn_inputs,
             output,
             rois,
             false);
}

int32_t LineCoordinateParser::Parse(
    std::shared_ptr<LineCoordinateResult> &output,
    std::shared_ptr<DNNTensor> &output_tensor) {
  if (!output_tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("TrackDetectionNode"), "invalid out tensor");
    rclcpp::shutdown();
  }
  std::shared_ptr<LineCoordinateResult> result;
  if (!output) {
    result = std::make_shared<LineCoordinateResult>();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<LineCoordinateResult>(output);
  }
  DNNTensor &tensor = *output_tensor;
  const int32_t *shape = tensor.properties.validShape.dimensionSize;
  RCLCPP_DEBUG(rclcpp::get_logger("TrackDetectionNode"),
               "PostProcess shape[1]: %d shape[2]: %d shape[3]: %d",
               shape[1],
               shape[2],
               shape[3]);
  hbSysFlushMem(&(tensor.sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  float x = reinterpret_cast<float *>(tensor.sysMem[0].virAddr)[0];
  float y = reinterpret_cast<float *>(tensor.sysMem[0].virAddr)[1];
  
  // =========================================================================
  // --- 动态映射回原图坐标 ---
  // 模型输出的 x, y 是 [0, 1] 之间的归一化比例值
  // 映射逻辑: 原图坐标 = 比例 * 裁剪区域的宽度/高度 + 裁剪区域的左/上偏移量
  // =========================================================================
  result->x = x * (CROP_RIGHT - CROP_LEFT) + CROP_LEFT;
  result->y = y * (CROP_BOTTOM - CROP_TOP) + CROP_TOP;
  
  RCLCPP_INFO(rclcpp::get_logger("TrackDetectionNode"),
               "coor rawx: %f,  rawy:%f, x: %f    y:%f", x, y, result->x, result->y);
  return 0;
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TrackDetectionNode>("GetLineCoordinate_go"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("TrackDetectionNode"), "Pkg exit.");
  return 0;
}
