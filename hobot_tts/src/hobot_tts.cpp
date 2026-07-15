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

#include "hobot_tts/hobot_tts.h"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace hobot_tts {

namespace {

constexpr char kDiskCacheMagic[] = "HTTSPCM1";
constexpr size_t kDiskCacheMagicSize = sizeof(kDiskCacheMagic) - 1;
constexpr uint32_t kMaxCachedTextBytes = 1024 * 1024;
constexpr uint32_t kMaxCachedPCMElements = 10 * 1000 * 1000;
constexpr size_t kProgressBarWidth = 30;

std::string FormatProgressBar(size_t current, size_t total) {
  const auto bounded_current = std::min(current, total);
  const auto filled =
      total == 0 ? kProgressBarWidth
                 : bounded_current * kProgressBarWidth / total;
  const auto percent = total == 0 ? 100 : bounded_current * 100 / total;

  std::ostringstream stream;
  stream << "[" << std::string(filled, '#')
         << std::string(kProgressBarWidth - filled, '-') << "] "
         << bounded_current << "/" << total << " (" << percent << "%)";
  return stream.str();
}

size_t UTF8CharacterLength(unsigned char lead_byte) {
  if ((lead_byte & 0x80) == 0) {
    return 1;
  }
  if ((lead_byte & 0xE0) == 0xC0) {
    return 2;
  }
  if ((lead_byte & 0xF0) == 0xE0) {
    return 3;
  }
  if ((lead_byte & 0xF8) == 0xF0) {
    return 4;
  }
  return 0;
}

}  // namespace

HobotTTSNode::HobotTTSNode(rclcpp::Node::SharedPtr& nh) : nh_(nh) {
  nh_->declare_parameter<std::string>("playback_device", playback_device_name_);
  nh_->get_parameter<std::string>("playback_device", playback_device_name_);
  nh_->declare_parameter<double>("volume_gain", volume_gain_);
  nh_->get_parameter<double>("volume_gain", volume_gain_);

  speaker_device_ = alsa_device_allocate();
  if (!speaker_device_) {
    RCLCPP_ERROR(nh_->get_logger(), "alloc speaker device error!");
    throw std::runtime_error("HobotTTSNode allocate alsa device failed");
  }
  speaker_device_->name = const_cast<char*>(playback_device_name_.c_str());
  speaker_device_->format = SND_PCM_FORMAT_S16;
  speaker_device_->direct = SND_PCM_STREAM_PLAYBACK;
  speaker_device_->rate = 16000;
  speaker_device_->channels = 2;
  speaker_device_->buffer_time = 0;  // use default buffer time
  speaker_device_->nperiods = 4;
  speaker_device_->period_size = 512;  // 1 period including 1024 frames

  auto ret = alsa_device_init(speaker_device_);
  if (ret < 0) {
    alsa_device_free(speaker_device_);
    speaker_device_ = nullptr;
    RCLCPP_ERROR(nh_->get_logger(), "alsa_device_init speaker failed. ret = %d",
                 ret);
    throw std::runtime_error("HobotTTSNode initialize alsa device failed");
  }

  // Register before slow model loading and PCM cache construction so startup
  // announcements remain queued until playback workers are ready.
  nh_->declare_parameter<std::string>("topic_sub", topic_subscription_name_);
  nh_->get_parameter<std::string>("topic_sub", topic_subscription_name_);
  text_subscription_ = nh_->create_subscription<std_msgs::msg::String>(
      topic_subscription_name_, 10,
      std::bind(&HobotTTSNode::MessageCallback, this, std::placeholders::_1));

  int err_code = 0;
  std::string tros_distro
      = std::string(std::getenv("TROS_DISTRO")? std::getenv("TROS_DISTRO") : "");
  tts_ =
      wetts_init(std::string("/opt/tros/" + tros_distro + "/lib/hobot_tts/tts_model").c_str(),
      "tts.flags", &err_code);
  if (!tts_) {
    RCLCPP_ERROR(nh_->get_logger(), "wetts_init failed. err_code = %d",
                 err_code);
    alsa_device_deinit(speaker_device_);
    alsa_device_free(speaker_device_);
    speaker_device_ = nullptr;
    throw std::runtime_error("HobotTTSNode initialize tts model failed");
  }

  struct audio_info info = wetts_audio_info(tts_);
  pcm_data_ = new char[info.max_len];

  RCLCPP_INFO_STREAM(nh_->get_logger(), "Sample rate: " << info.sample_rate);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "Bit depth: " << info.bit_depth);
  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "Num of channels: " << info.num_channels);
  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "Max seconds of audio: " << info.max_dur_ms / 1000);

  // PCM 磁盘缓存可跨进程复用。节点重启后，已经生成过的内容可以直接读取，
  // 不需要再次调用 wetts 推理。默认保存在源码功能包内，启动参数仍可覆盖。
  pcm_cache_dir_ = std::string(HOBOT_TTS_SOURCE_DIR) + "/pcm_cache";
  try {
    const auto package_share_directory =
        ament_index_cpp::get_package_share_directory("hobot_tts");
    common_chars_file_ =
        package_share_directory + "/resources/common_3500_chars.txt";
  } catch (const std::exception& ex) {
    RCLCPP_WARN(nh_->get_logger(),
                "Failed to resolve hobot_tts package share directory: %s",
                ex.what());
  }

  // disk_cache_enabled:
  //   保存二维码短片段和普通文本的 PCM。二维码完整播报不落盘，
  //   每次都使用内存中的短片段拼接，避免随机二维码产生大量缓存文件。
  // common_chars_cache_enabled:
  //   可选地在后台逐字生成 3500 常用字 PCM。该任务较重，默认关闭。
  //   当前普通整句不会自动拆字拼接；常用字缓存主要供单字复用和后续扩展。
  nh_->declare_parameter<bool>("disk_cache_enabled", disk_cache_enabled_);
  nh_->get_parameter<bool>("disk_cache_enabled", disk_cache_enabled_);
  nh_->declare_parameter<std::string>("pcm_cache_dir", pcm_cache_dir_);
  nh_->get_parameter<std::string>("pcm_cache_dir", pcm_cache_dir_);
  nh_->declare_parameter<bool>("common_chars_cache_enabled",
                               common_chars_cache_enabled_);
  nh_->get_parameter<bool>("common_chars_cache_enabled",
                           common_chars_cache_enabled_);
  nh_->declare_parameter<std::string>("common_chars_file", common_chars_file_);
  nh_->get_parameter<std::string>("common_chars_file", common_chars_file_);

  if (disk_cache_enabled_ && !EnsureCacheDirectory()) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Failed to create PCM disk cache directory: %s",
                 pcm_cache_dir_.c_str());
    disk_cache_enabled_ = false;
  }
  if (disk_cache_enabled_) {
    RCLCPP_INFO(nh_->get_logger(), "PCM disk cache directory: %s",
                pcm_cache_dir_.c_str());
  }

  // 二维码短片段预热会优先从磁盘读取；首次不存在时才合成并写盘。
  nh_->declare_parameter<bool>("warmup_enabled", warmup_enabled_);
  nh_->get_parameter<bool>("warmup_enabled", warmup_enabled_);
  WarmupQRCodeCache();

  processing_thread_ = std::thread(&HobotTTSNode::ProcessMessages, this);
  playback_thread_ = std::thread(&HobotTTSNode::PlaybackMessages, this);
  if (disk_cache_enabled_ && common_chars_cache_enabled_) {
    common_chars_cache_thread_ =
        std::thread(&HobotTTSNode::WarmupCommonCharacterDiskCache, this);
  } else {
    RCLCPP_INFO(nh_->get_logger(),
                "Common character PCM disk cache warmup disabled");
  }
}

HobotTTSNode::~HobotTTSNode() {
  StopPlayback();

  if (pcm_data_) {
    delete[] pcm_data_;
  }

  if (tts_) {
    wetts_free(tts_);
  }

  if (speaker_device_) {
    alsa_device_deinit(speaker_device_);
    alsa_device_free(speaker_device_);
    speaker_device_ = nullptr;
  }
}

void HobotTTSNode::MessageCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (message_queue_.size() >= kMaxMessageQueueSize) {
    // Discard the oldest message if the queue size exceeds the limit
    message_queue_.pop();
  }
  message_queue_.push(msg);
  cv_.notify_one();
}

int HobotTTSNode::ConvertToPCM(const std::string& msg,
                               std::unique_ptr<float[]>& pcm_data,
                               int& pcm_size) {
  // PCM 查找顺序：
  // 1. 本次进程的内存缓存；
  // 2. 上次运行留下的磁盘缓存；
  // 3. 首次调用 wetts 合成，并将结果写入磁盘。
  auto cached_pcm = pcm_cache_.find(msg);
  if (cached_pcm != pcm_cache_.end()) {
    pcm_size = static_cast<int>(cached_pcm->second.size());
    pcm_data.reset(new float[pcm_size]);
    std::copy(cached_pcm->second.begin(), cached_pcm->second.end(),
              pcm_data.get());
    return 0;
  }

  std::vector<float> disk_pcm;
  if (LoadPCMFromDisk(msg, disk_pcm)) {
    pcm_size = static_cast<int>(disk_pcm.size());
    pcm_data.reset(new float[pcm_size]);
    std::copy(disk_pcm.begin(), disk_pcm.end(), pcm_data.get());
    RCLCPP_INFO(nh_->get_logger(), "Loaded PCM disk cache: %s", msg.c_str());
    return 0;
  }

  auto ret = SynthesizePCM(msg, pcm_data, pcm_size);
  if (ret != 0) {
    return ret;
  }

  if (disk_cache_enabled_) {
    const std::vector<float> synthesized_pcm(pcm_data.get(),
                                             pcm_data.get() + pcm_size);
    if (!SavePCMToDisk(msg, synthesized_pcm)) {
      RCLCPP_WARN(nh_->get_logger(), "Failed to save PCM disk cache: %s",
                  msg.c_str());
    } else {
      RCLCPP_INFO(nh_->get_logger(), "Saved PCM disk cache: %s", msg.c_str());
    }
  }
  return 0;
}

int HobotTTSNode::SynthesizePCM(const std::string& msg,
                               std::unique_ptr<float[]>& pcm_data,
                               int& pcm_size) {
  std::lock_guard<std::mutex> lock(tts_mutex_);
  auto err_code = wetts_synthesis(tts_, msg.c_str(), 1, pcm_data_, &pcm_size);
  if (err_code != ERRCODE_TTS_SUCC) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(),
                        "Synthesis failed with error code: " << err_code);
    return -1;
  }

  pcm_data.reset(new float[pcm_size]);
  memcpy(pcm_data.get(), pcm_data_, pcm_size * sizeof(float));

  return 0;
}

bool HobotTTSNode::EnsureCacheDirectory() {
  if (pcm_cache_dir_.empty()) {
    return false;
  }

  size_t position = pcm_cache_dir_[0] == '/' ? 1 : 0;
  while (position <= pcm_cache_dir_.size()) {
    position = pcm_cache_dir_.find('/', position);
    const auto directory = pcm_cache_dir_.substr(0, position);
    if (!directory.empty()) {
      struct stat status {};
      if (stat(directory.c_str(), &status) == 0) {
        if (!S_ISDIR(status.st_mode)) {
          return false;
        }
      } else if (mkdir(directory.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
      }
    }
    if (position == std::string::npos) {
      break;
    }
    ++position;
  }
  return true;
}

std::string HobotTTSNode::CacheFilePath(const std::string& msg) const {
  uint64_t hash = 1469598103934665603ULL;
  for (unsigned char byte : msg) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }

  std::ostringstream stream;
  stream << pcm_cache_dir_ << "/" << std::hex << std::setw(16)
         << std::setfill('0') << hash << ".pcm";
  return stream.str();
}

bool HobotTTSNode::LoadPCMFromDisk(const std::string& msg,
                                   std::vector<float>& pcm_data) {
  if (!disk_cache_enabled_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(disk_cache_mutex_);
  std::ifstream input(CacheFilePath(msg), std::ios::binary);
  if (!input) {
    return false;
  }

  char magic[kDiskCacheMagicSize] = {};
  uint32_t text_size = 0;
  uint32_t pcm_size = 0;
  input.read(magic, sizeof(magic));
  input.read(reinterpret_cast<char*>(&text_size), sizeof(text_size));
  if (!input || memcmp(magic, kDiskCacheMagic, sizeof(magic)) != 0 ||
      text_size > kMaxCachedTextBytes) {
    return false;
  }

  std::string cached_text(text_size, '\0');
  if (text_size > 0) {
    input.read(&cached_text[0], text_size);
  }
  input.read(reinterpret_cast<char*>(&pcm_size), sizeof(pcm_size));
  if (!input || cached_text != msg || pcm_size > kMaxCachedPCMElements) {
    return false;
  }

  pcm_data.resize(pcm_size);
  input.read(reinterpret_cast<char*>(pcm_data.data()),
             pcm_data.size() * sizeof(float));
  return static_cast<bool>(input);
}

bool HobotTTSNode::SavePCMToDisk(const std::string& msg,
                                 const std::vector<float>& pcm_data) {
  if (!disk_cache_enabled_ || msg.size() > kMaxCachedTextBytes ||
      pcm_data.size() > kMaxCachedPCMElements) {
    return false;
  }

  std::lock_guard<std::mutex> lock(disk_cache_mutex_);
  const auto cache_path = CacheFilePath(msg);
  // 先写临时文件，再原子替换正式缓存，避免异常退出留下半个 PCM 文件。
  const auto temporary_path = cache_path + ".tmp";
  std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
  if (!output) {
    return false;
  }

  const auto text_size = static_cast<uint32_t>(msg.size());
  const auto pcm_size = static_cast<uint32_t>(pcm_data.size());
  output.write(kDiskCacheMagic, kDiskCacheMagicSize);
  output.write(reinterpret_cast<const char*>(&text_size), sizeof(text_size));
  output.write(msg.data(), msg.size());
  output.write(reinterpret_cast<const char*>(&pcm_size), sizeof(pcm_size));
  output.write(reinterpret_cast<const char*>(pcm_data.data()),
               pcm_data.size() * sizeof(float));
  output.close();
  if (!output || std::rename(temporary_path.c_str(), cache_path.c_str()) != 0) {
    std::remove(temporary_path.c_str());
    return false;
  }
  return true;
}

int HobotTTSNode::EnsurePCMOnDisk(const std::string& msg, bool& generated) {
  // 常用字后台预热使用此函数：磁盘中已有 PCM 时直接跳过，
  // 只有首次遇到的汉字才会执行推理并保存。
  generated = false;
  std::vector<float> disk_pcm;
  if (LoadPCMFromDisk(msg, disk_pcm)) {
    return 0;
  }

  std::unique_ptr<float[]> pcm_data;
  int pcm_size = 0;
  auto ret = SynthesizePCM(msg, pcm_data, pcm_size);
  if (ret != 0) {
    return ret;
  }

  const std::vector<float> synthesized_pcm(pcm_data.get(),
                                           pcm_data.get() + pcm_size);
  if (!SavePCMToDisk(msg, synthesized_pcm)) {
    return -1;
  }
  generated = true;
  return 0;
}

int HobotTTSNode::CachePCM(const std::string& msg) {
  // 二维码短片段会同时进入内存缓存和磁盘缓存：
  // 本次运行直接从内存拼接；下次启动先从磁盘读取，再装入内存。
  std::unique_ptr<float[]> pcm_data;
  int pcm_size = 0;
  auto ret = ConvertToPCM(msg, pcm_data, pcm_size);
  if (ret != 0) {
    return ret;
  }

  pcm_cache_[msg] = std::vector<float>(pcm_data.get(), pcm_data.get() + pcm_size);
  return 0;
}

void HobotTTSNode::WarmupQRCodeCache() {
  if (!warmup_enabled_) {
    RCLCPP_WARN(nh_->get_logger(),
                "QR code PCM cache disabled; QR announcements use synthesis");
    return;
  }

  RCLCPP_INFO(nh_->get_logger(), "Building QR code PCM fragment cache");
  // 数字二维码播报由短片段拼接完成。例如“二维码8888逆时针”会复用
  // “二维码”、“8”和“逆时针”的 PCM，避免扫码后临时执行完整推理。
  const std::vector<std::string> prompts = {
      "二维码", "0", "1", "2", "3", "4", "5", "6", "7",
      "8", "9", "顺时针", "逆时针", "clockwise", "anticlockwise"};
  size_t processed = 0;
  for (const auto& prompt : prompts) {
    if (CachePCM(prompt) != 0) {
      throw std::runtime_error("HobotTTSNode build QR code PCM cache failed");
    }
    ++processed;
    const auto progress = FormatProgressBar(processed, prompts.size());
    RCLCPP_INFO(nh_->get_logger(),
                "QR code PCM fragment disk cache progress: %s, fragment=%s",
                progress.c_str(), prompt.c_str());
  }
  RCLCPP_INFO(nh_->get_logger(), "QR code PCM fragment cache ready");
}

void HobotTTSNode::WarmupCommonCharacterDiskCache() {
  // 仅在 common_chars_cache_enabled=true 时启动。逐字检查磁盘缓存，
  // 已存在的汉字直接复用，缺失的汉字才合成并写入磁盘。
  // 注意：普通整句播报目前仍按整句读取或合成，不会从这里自动逐字拼接。
  std::ifstream input(common_chars_file_);
  if (!input) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Failed to open common character list: %s",
                 common_chars_file_.c_str());
    return;
  }

  const std::string contents((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
  size_t processed = 0;
  size_t generated = 0;
  size_t reused = 0;
  size_t failed = 0;

  auto progress = FormatProgressBar(0, 3500);
  RCLCPP_INFO(nh_->get_logger(),
              "Building 3500 common character PCM disk cache in background: %s",
              progress.c_str());
  size_t offset = 0;
  while (offset < contents.size() && !stop_playback_) {
    const auto lead_byte = static_cast<unsigned char>(contents[offset]);
    if (std::isspace(lead_byte)) {
      ++offset;
      continue;
    }

    const auto character_size = UTF8CharacterLength(lead_byte);
    if (character_size == 0 || offset + character_size > contents.size()) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Skipping invalid UTF-8 byte in common character list");
      ++offset;
      continue;
    }

    const auto character = contents.substr(offset, character_size);
    bool was_generated = false;
    if (EnsurePCMOnDisk(character, was_generated) == 0) {
      if (was_generated) {
        ++generated;
      } else {
        ++reused;
      }
    } else {
      ++failed;
      RCLCPP_WARN(nh_->get_logger(),
                  "Failed to cache common character: %s",
                  character.c_str());
    }
    ++processed;
    offset += character_size;

    if (processed % 100 == 0) {
      progress = FormatProgressBar(processed, 3500);
      RCLCPP_INFO(nh_->get_logger(),
                  "Common character PCM disk cache progress: %s, "
                  "generated=%zu, reused=%zu, failed=%zu",
                  progress.c_str(), generated, reused, failed);
    }
  }

  if (stop_playback_) {
    progress = FormatProgressBar(processed, 3500);
    RCLCPP_WARN(nh_->get_logger(),
                "Common character PCM disk cache stopped: %s",
                progress.c_str());
    return;
  }
  progress = FormatProgressBar(processed, 3500);
  RCLCPP_INFO(nh_->get_logger(),
              "Common character PCM disk cache ready: %s, "
              "generated=%zu, reused=%zu, failed=%zu",
              progress.c_str(), generated, reused, failed);
}

bool HobotTTSNode::BuildCachedQRCodePCM(const std::string& msg,
                                        std::unique_ptr<float[]>& pcm_data,
                                        int& pcm_size) {
  if (!warmup_enabled_) {
    return false;
  }

  const std::string prefix = "二维码";
  std::string direction;
  if (msg.size() >= std::string("顺时针").size() &&
      msg.compare(msg.size() - std::string("顺时针").size(),
                  std::string("顺时针").size(), "顺时针") == 0) {
    direction = "顺时针";
  } else if (msg.size() >= std::string("逆时针").size() &&
             msg.compare(msg.size() - std::string("逆时针").size(),
                         std::string("逆时针").size(), "逆时针") == 0) {
    direction = "逆时针";
  } else {
    return false;
  }

  if (msg.rfind(prefix, 0) != 0 ||
      msg.size() <= prefix.size() + direction.size()) {
    return false;
  }

  const std::string content =
      msg.substr(prefix.size(), msg.size() - prefix.size() - direction.size());

  // 二维码数字范围为 1-9999，组合较多。完整播报不读写磁盘缓存，
  // 每次都使用启动阶段装入内存的短片段拼接，避免生成大量小文件。
  std::vector<std::string> fragments = {prefix};
  if (std::all_of(content.begin(), content.end(),
                  [](unsigned char ch) { return std::isdigit(ch); })) {
    size_t qrcode_value = 0;
    for (char digit : content) {
      qrcode_value = qrcode_value * 10 + static_cast<size_t>(digit - '0');
      if (qrcode_value > 9999) {
        return false;
      }
      fragments.emplace_back(1, digit);
    }
    if (qrcode_value < 1) {
      return false;
    }
  } else if (content == "ClockWise") {
    fragments.push_back("clockwise");
  } else if (content == "AntiClockWise") {
    fragments.push_back("anticlockwise");
  } else {
    return false;
  }
  fragments.push_back(direction);

  size_t total_size = 0;
  for (const auto& fragment : fragments) {
    auto cached_pcm = pcm_cache_.find(fragment);
    if (cached_pcm == pcm_cache_.end()) {
      return false;
    }
    total_size += cached_pcm->second.size();
  }

  pcm_size = static_cast<int>(total_size);
  pcm_data.reset(new float[pcm_size]);
  auto output = pcm_data.get();
  for (const auto& fragment : fragments) {
    const auto& cached_pcm = pcm_cache_.at(fragment);
    output = std::copy(cached_pcm.begin(), cached_pcm.end(), output);
  }
  return true;
}

void HobotTTSNode::EnqueuePCM(std::unique_ptr<float[]> pcm_data, int pcm_size) {
  std::lock_guard<std::mutex> playback_lock(playback_mutex_);
  if (playback_queue_.size() >= kMaxPlaybackQueueSize) {
    // Discard the oldest PCM data if the queue size exceeds the limit
    playback_queue_.pop();
  }
  playback_queue_.push(std::make_pair(std::move(pcm_data), pcm_size));
  cv_playback_.notify_one();
}

void HobotTTSNode::ProcessMessages() {
  auto splitString = [](std::string& input,
                        std::vector<std::string>& segments) {
    std::string segment;

    size_t startPos = 0;
    size_t index = 0;

    auto isChinesePunctuation = [](const std::string& str, size_t index) {
      if (index + 2 >= str.size()) {
        return false;
      }
      return (str[index] == '\xEF' && str[index + 1] == '\xBC' &&
              (str[index + 2] == '\x8C' || str[index + 2] == '\x9F' ||
               str[index + 2] == '\x9A' || str[index + 2] == '\x81')) ||
             (str[index] == '\xE3' && str[index + 1] == '\x80' &&
              (str[index + 2] == '\x82' || str[index + 2] == '\x81'));
    };

    while (index < input.length()) {
      if (isChinesePunctuation(input, index)) {
        segment = input.substr(startPos, index - startPos);
        if (!segment.empty()) {
          segments.push_back(segment);
          segment.clear();
        }
        startPos = index + 3;
        index += 3;
        continue;
      }
      auto current_char = static_cast<unsigned char>(input[index]);
      if (std::ispunct(current_char)) {
        segment = input.substr(startPos, index - startPos);
        if (!segment.empty()) {
          segments.push_back(segment);
          segment.clear();
        }
        startPos = index + 1;
      }
      if (std::isspace(current_char)) {
        // input[index] = ' ';
        segment = input.substr(startPos, index - startPos);
        if (!segment.empty()) {
          segments.push_back(segment);
          segment.clear();
        }
        startPos = index + 1;
      }
      if (std::isupper(current_char)) {
        input[index] = static_cast<char>(std::tolower(current_char));
      }
      ++index;
    }

    segment = input.substr(startPos);
    if (!segment.empty()) {
      segments.push_back(segment);
    }
  };

  while (rclcpp::ok()) {
    std_msgs::msg::String::SharedPtr message;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock,
               [this] { return !message_queue_.empty() || stop_playback_; });

      if (stop_playback_) {
        break;
      }

      message = message_queue_.front();
      message_queue_.pop();
    }

    std::unique_ptr<float[]> cached_qrcode_pcm;
    int cached_qrcode_pcm_size = 0;
    if (BuildCachedQRCodePCM(message->data, cached_qrcode_pcm,
                             cached_qrcode_pcm_size)) {
      EnqueuePCM(std::move(cached_qrcode_pcm), cached_qrcode_pcm_size);
      RCLCPP_INFO(nh_->get_logger(), "Queued cached QR announcement: %s",
                  message->data.c_str());
      continue;
    }

    std::vector<std::string> message_vector;
    splitString(message->data, message_vector);
    for (auto msg : message_vector) {
      std::unique_ptr<float[]> pcm_data;
      int pcm_size;
      auto ret = ConvertToPCM(msg, pcm_data, pcm_size);
      if (!ret) {
        EnqueuePCM(std::move(pcm_data), pcm_size);
      }
    }
  }
}

void HobotTTSNode::PlaybackMessages() {
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lock(playback_mutex_);
    cv_playback_.wait(
        lock, [this] { return !playback_queue_.empty() || stop_playback_; });

    if (stop_playback_ && playback_queue_.empty()) {
      break;
    }

    while (!playback_queue_.empty()) {
      auto pcm_data = std::move(playback_queue_.front().first);
      auto pcm_size = playback_queue_.front().second;
      playback_queue_.pop();

      std::vector<int16_t> pcm_int16;
      auto pcm_float = pcm_data.get();
      for (int i = 0; i < pcm_size; i++) {
        pcm_int16.push_back(*pcm_float);
        pcm_int16.push_back(*pcm_float);
        pcm_float++;
      }

      // 保留已经在设备上验证可播报的原始 ALSA 播放顺序。
      if (speaker_device_) {
        snd_pcm_sframes_t frames = snd_pcm_bytes_to_frames(
            speaker_device_->handle, pcm_int16.size() * sizeof(int16_t));
        snd_pcm_prepare(speaker_device_->handle);
        alsa_device_write(speaker_device_, pcm_int16.data(), frames);
        snd_pcm_drop(speaker_device_->handle);
      }
    }
  }
}

void HobotTTSNode::StopPlayback() {
  if (!stop_playback_) {
    stop_playback_ = true;
    cv_.notify_one();
    cv_playback_.notify_one();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (playback_thread_.joinable()) {
      playback_thread_.join();
    }
    if (common_chars_cache_thread_.joinable()) {
      common_chars_cache_thread_.join();
    }
  }
}

}  // namespace hobot_tts
