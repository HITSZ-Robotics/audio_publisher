# Audio Publisher Node
![whisper_ros1_logo](https://github.com/user-attachments/assets/477981a7-4ef6-4ee9-8a5b-2f877dbeec6e)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

The **Audio Publisher Node** is a ROS (Robot Operating System) node designed to capture audio data from a microphone, publish it as a ROS topic, and optionally save it to a WAV file. It also supports playing the received audio in real time.

This package contains a ROS node that subscribes to audio data and optionally saves it to a WAV file. It also plays the received audio in real time.

## Demo

https://github.com/user-attachments/assets/3086f7b6-1980-4c05-b58a-1c9ff978e7ce

---

## About

This code is part of the **Whisper ROS1** project, which integrates OpenAI's Whisper speech recognition model with ROS1 for real-time audio processing and transcription. For more details about the project, please visit the [Whisper ROS1 repository](https://github.com/zzl410/whisper_ros1).

---

## Prerequisites

Before running the node, you need to install the following dependencies:

**PortAudio**: This library is used for audio capture. Install it using your system's package manager:
   - On Ubuntu:
     ```bash
     sudo apt-get install portaudio19-dev
     ```

---

## Installation

1. Clone this repository into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone 
   ```

2. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

---

## Usage

### Running the Node

1. Launch the `audio_publisher` node:
   ```bash
   roslaunch audio_publisher audio_publisher.launch
   ```

2. By default, the node publishes audio data to the `/audio_stream` topic. You can change the topic name and enable/disable WAV file saving by modifying the launch file or passing parameters.

### Launch File Parameters

The `audio_publisher.launch` file supports the following parameters:

- `save_wav` (default: `false`): Set to `true` to save the captured audio to a WAV file.
- `topic_name` (default: `audio_stream`): Set the name of the ROS topic to publish audio data.

Example:
```xml
<launch>
    <node
        name="audio_publisher"
        pkg="audio_publisher"
        type="audio_publisher_node_main"
        output="screen">
        
        <param name="save_wav" value="true" />
        <param name="topic_name" value="custom_audio_topic" />
    </node>
</launch>
```

---

## Configuration

- **Sample Rate**: The default sample rate is 16 kHz. Modify the `sample_rate` variable in the code if needed.
- **Chunk Size**: The default chunk size is 1024 samples. Modify the `chunk_size` variable in the code if needed.
- **Channels**: The default number of audio channels is 1 (mono). Modify the `channels` variable in the code if needed.

---

## Troubleshooting

- **PortAudio Initialization Failed**: Ensure that the PortAudio library is installed and your microphone is properly connected.
- **No Input Device Found**: Check your audio input devices and ensure they are recognized by your system.
- **Failed to Open WAV File**: Ensure you have write permissions in the directory where the WAV file is being saved.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.


