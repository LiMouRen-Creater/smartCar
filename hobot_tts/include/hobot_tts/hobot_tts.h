// Copyright (c) 2024，D-Robotics.
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

#ifndef HOBOT_TTS_INCLUDE_H_
#define HOBOT_TTS_INCLUDE_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tts_api.h"
#include "utils/alsa_device.h"

namespace hobot_tts {

class HobotTTSNode {
 public:
  HobotTTSNode(rclcpp::Node::SharedPtr& nh);
  ~HobotTTSNode();

  void OnGetText(const std_msgs::msg::String::SharedPtr msg);

 private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_subscription_;
  std::string topic_subscription_name_ = "/tts_text";
  std::string playback_device_name_ = "hw:0,1";
  double volume_gain_ = 1.0;

  void MessageCallback(const std_msgs::msg::String::SharedPtr msg);

  void ProcessMessages();

  void PlaybackMessages();

  void StopPlayback();

  int ConvertToPCM(const std::string& msg, std::unique_ptr<float[]>& pcm_data,
                   int& pcm_size);

  int SynthesizePCM(const std::string& msg, std::unique_ptr<float[]>& pcm_data,
                    int& pcm_size);

  void WarmupQRCodeCache();

  void WarmupCommonCharacterDiskCache();

  int CachePCM(const std::string& msg);

  int EnsurePCMOnDisk(const std::string& msg, bool& generated);

  bool EnsureCacheDirectory();

  std::string CacheFilePath(const std::string& msg) const;

  bool LoadPCMFromDisk(const std::string& msg, std::vector<float>& pcm_data);

  bool SavePCMToDisk(const std::string& msg,
                     const std::vector<float>& pcm_data);

  bool BuildCachedQRCodePCM(const std::string& msg,
                            std::unique_ptr<float[]>& pcm_data, int& pcm_size);

  void EnqueuePCM(std::unique_ptr<float[]> pcm_data, int pcm_size);

  std::queue<std_msgs::msg::String::SharedPtr> message_queue_;
  std::mutex mutex_;
  std::condition_variable cv_;

  std::queue<std::pair<std::unique_ptr<float[]>, int>> playback_queue_;
  std::mutex playback_mutex_;
  std::condition_variable cv_playback_;

  std::atomic<bool> stop_playback_{false};
  std::thread processing_thread_;
  std::thread playback_thread_;
  std::thread common_chars_cache_thread_;

  static constexpr size_t kMaxMessageQueueSize = 10;
  static constexpr size_t kMaxPlaybackQueueSize = 5;

  void* tts_ = nullptr;
  alsa_device_t* speaker_device_ = nullptr;
  char* pcm_data_ = nullptr;
  // 二维码短片段会在启动时预热；磁盘缓存用于跨进程复用 PCM。
  bool warmup_enabled_ = true;
  bool disk_cache_enabled_ = true;
  // 3500 常用字缓存比较耗时，默认关闭。按需开启后会逐字写入磁盘。
  // 当前普通整句播报不会自动按字拼接，这些 PCM 主要用于单字复用和后续扩展。
  bool common_chars_cache_enabled_ = false;
  std::string pcm_cache_dir_;
  std::string common_chars_file_;
  // 仅保存本次进程经常使用的二维码短片段，避免每次播报都访问磁盘。
  std::unordered_map<std::string, std::vector<float>> pcm_cache_;
  // wetts 推理和磁盘缓存可能被不同线程访问，需要分别串行化。
  std::mutex tts_mutex_;
  std::mutex disk_cache_mutex_;
};

}  // namespace hobot_tts

#endif
