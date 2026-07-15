[English](./README.md) | 简体中文

# Hobot TTS

**hobot_tts**提供将文本转化为语音播放功能，订阅文本消息，然后调用TTS软件接口，将文本转化为PCM数据，最后调用ALSA接口播放。

## 环境搭建

运行hobot_tts，需要确认音频设备设置正确，具体设置方法参考RDK用户手册[音频转接板](https://developer.horizon.cc/documents_rdk/hardware_development/rdk_x3/audio_board)章节。

可使用如下命令检查是否设置正确：

```bash
root@ubuntu:~# ls /dev/snd/
by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
```

如果出现例如`pcmC0D1p`音频设备则表示设置正确。

## 运行方式

首次运行需要下载模型文件解压，详细命令如下：

```bash
wget http://archive.d-robotics.cc/tts-model/tts_model.tar.gz
sudo tar -xf tts_model.tar.gz -C /opt/tros/${TROS_DISTRO}/lib/hobot_tts/
```

启动程序：

```bash
source /opt/tros/setup.bash

# 屏蔽调式打印信息
export GLOG_minloglevel=1

ros2 run hobot_tts hobot_tts
```

运行成功后，程序订阅topic "/tts_text"（消息类型为std_msgs/msg/String），然后转化为语音信号播放。

当`warmup_enabled`为`true`时，如果要求首次二维码播报也立即播放，请等待日志出现`QR code PCM fragment cache ready`后再启动导航。预热期间发布的播报消息会排队，并在预热完成后播放。

二维码短片段预热会打印文本进度条，例如`QR code PCM fragment disk cache progress: [##----------------------------] 1/15 (6%)`。普通文本首次生成并写入磁盘时会打印`Saved PCM disk cache`，下次直接从磁盘复用时会打印`Loaded PCM disk cache`。当 3500 常用字后台缓存开启时，每完成 100 字会打印一次文本进度条。

当`disk_cache_enabled`为`true`时，二维码短片段和其他普通文本在首次生成后会保存到本地 PCM 缓存目录，重启后可直接复用。二维码数字为完全随机的`1-9999`，因此完整二维码播报不会写入磁盘，而是每次使用内存中的短片段拼接，避免随机二维码产生大量缓存文件。默认缓存目录为源码功能包`hobot_tts-develop`内的`pcm_cache`。

当`common_chars_cache_enabled`为`true`时，程序会在二维码短缓存可用后，于后台逐步生成《通用规范汉字表》一级字表中的 3500 个常用字。首次生成预计需要较长时间，日志每完成 100 字会输出一次进度；再次启动时，已经存在的缓存文件会被跳过。

注意：加载音频驱动时，若新增音频设备不是`pcmC0D1p`，例如`pcmC1D1p`，则需要使用参数`playback_device`指定播放音频设备。

## 参数列表

| 参数名          | 解释             | 类型        | 是否必须 | 默认值      |
| --------------- | ---------------- | ----------- | -------- | ----------- |
| topic_sub       | 订阅的文本topic  | std::string | 否       | "/tts_text" |
| playback_device | 语音播放音频设备 | std::string | 否       | "hw:0,1"    |
| volume_gain | PCM 播放音量倍数 | double | 否 | 1.0 |
| warmup_enabled  | 启动时预生成“二维码”、数字和方向词 PCM 片段，使随机二维码扫码播报不再执行 TTS 推理 | bool | 否 | true |
| disk_cache_enabled | 启用磁盘缓存，保存二维码短片段、普通文本和可选常用字 PCM | bool | 否 | true |
| pcm_cache_dir | PCM 磁盘缓存目录 | std::string | 否 | 源码功能包 hobot_tts-develop 内的 pcm_cache |
| common_chars_cache_enabled | 是否在后台生成 3500 个常用字 PCM 缓存 | bool | 否 | false |
| common_chars_file | 3500 常用字列表资源文件 | std::string | 否 | 安装目录内的 resources/common_3500_chars.txt |
