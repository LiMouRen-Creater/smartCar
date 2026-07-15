English| [简体中文](./README_cn.md)

# Hobot TTS

**hobot_tts** provides the function to convert text into speech for audio playback. It subscribes to text messages, then calls the TTS software interface to convert the text into PCM data, and finally uses the ALSA interface for playback.

## Environment Setup

To run hobot_tts, it is necessary to confirm that the audio device is set up correctly. Refer to the RDK User Manual [Audio Adapter](https://developer.horizon.cc/documents_rdk/hardware_development/rdk_x3/audio_board) section for specific setup methods.

You can use the following command to check if the settings are correct:

```bash
root@ubuntu:~# ls /dev/snd/
by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
```

If an audio device such as `pcmC0D1p` appears, it means the settings are correct.

## How to Run

To run for the first time, download and extract the model file. The detailed commands are as follows:

```bash
wget http://archive.d-robotics.cc/tts-model/tts_model.tar.gz
sudo tar -xf tts_model.tar.gz -C /opt/tros/${TROS_DISTRO}/lib/hobot_tts/
```

Start the program:

```bash
source /opt/tros/setup.bash

# Suppress debug printing information
export GLOG_minloglevel=1

ros2 run hobot_tts hobot_tts
```

After successful execution, the program subscribes to the topic "/tts_text" (message type is std_msgs/msg/String) and converts it into speech signal for playback.

When `warmup_enabled` is `true`, wait for the `QR code PCM fragment cache ready` log before starting navigation if the first QR announcement must play immediately. Announcements published during startup are queued and played after warmup completes.

QR fragment warmup logs a text progress bar such as `QR code PCM fragment disk cache progress: [##----------------------------] 1/15 (6%)`. When ordinary text is generated and saved for the first time, the node logs `Saved PCM disk cache`; later disk reuse logs `Loaded PCM disk cache`. When the 3500 common-character background cache is enabled, a text progress bar is logged every 100 characters.

When `disk_cache_enabled` is `true`, QR fragments and other text are saved to the local PCM cache directory after their first generation and reused after restart. QR values are fully random in the `1-9999` range, so complete numeric QR announcements are not written to disk; they are assembled from in-memory fragments each time to avoid creating many cache files. The default cache directory is `pcm_cache` inside the `hobot_tts-develop` source package.

When `common_chars_cache_enabled` is `true`, the node generates local PCM files for the 3500 level-1 characters from the Common Standard Chinese Characters Table in the background after the short QR cache is ready. Initial generation takes a while. Progress is logged every 100 characters, and existing files are skipped on later starts.

Note: When loading the audio driver, if the new audio device is not `pcmC0D1p`, for example `pcmC1D1p`, you need to use the `playback_device` parameter to specify the playback audio device.

## Parameter List

| Parameter Name  | Explanation              | Type         | Required | Default Value |
| --------------- | ------------------------ | ------------ | -------- | ------------- |
| topic_sub       | Subscribed text topic    | std::string  | No       | "/tts_text"   |
| playback_device | Audio playback device     | std::string  | No       | "hw:0,1"      |
| volume_gain | PCM playback volume multiplier | double | No | 1.0 |
| warmup_enabled  | Pre-generate the QR prefix, digit, and direction PCM fragments at startup so random QR playback does not run TTS inference | bool | No | true |
| disk_cache_enabled | Enable disk caching for QR fragments, ordinary text, and optional common-character PCM | bool | No | true |
| pcm_cache_dir | PCM disk cache directory | std::string | No | pcm_cache inside the hobot_tts-develop source package |
| common_chars_cache_enabled | Generate local PCM cache files for 3500 common Chinese characters in the background | bool | No | false |
| common_chars_file | 3500 common Chinese character list resource file | std::string | No | Installed resources/common_3500_chars.txt |
