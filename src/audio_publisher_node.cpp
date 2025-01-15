#include "audio_publisher/audio_publisher_node.h"

AudioPublisher::AudioPublisher(const std::string &topic_name, bool save_wav) : 
    chunk_size(1024), 
    sample_rate(16000), 
    channels(1), 
    sample_width(sizeof(float)),
    save_wav_file(save_wav),
    topic_name(topic_name) { // Initialize topic name
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Float32MultiArray>(topic_name, 1000); // Use custom topic name

    PaError err = Pa_Initialize();
    if (err != paNoError) {
        ROS_ERROR("PortAudio initialization failed: %s", Pa_GetErrorText(err));
        throw std::runtime_error("PortAudio initialization failed");
    }

    inputParameters.device = Pa_GetDefaultInputDevice();
    if (inputParameters.device == paNoDevice) {
        ROS_ERROR("No input device found");
        throw std::runtime_error("No input device found");
    }

    inputParameters.channelCount = channels;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream, &inputParameters, nullptr, sample_rate, chunk_size, paClipOff, nullptr, nullptr);
    if (err != paNoError) {
        ROS_ERROR("Failed to open audio stream: %s", Pa_GetErrorText(err));
        throw std::runtime_error("Failed to open audio stream");
    }

    err = Pa_StartStream(stream);
    if (err != paNoError) {
        ROS_ERROR("Failed to start audio stream: %s", Pa_GetErrorText(err));
        throw std::runtime_error("Failed to start audio stream");
    }

    if (save_wav_file) {
        wav_file.open("output_publisher.wav", std::ios::binary);
        if (!wav_file.is_open()) {
            ROS_ERROR("Failed to create WAV file");
            throw std::runtime_error("Failed to open WAV file");
        }
        writeWavHeader();
    }

    ROS_INFO("Audio publisher node initialized, publishing to topic: %s", topic_name.c_str());
}

AudioPublisher::~AudioPublisher() {
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    if (save_wav_file && wav_file.is_open()) {
        updateWavHeader();
        wav_file.close();
        ROS_INFO("WAV file saved");
    }
}

void AudioPublisher::publishAudio() {
    ros::Rate rate(sample_rate / chunk_size);

    ROS_INFO("Start publishing audio data...");
    std::vector<float> buffer(chunk_size);

    while (ros::ok()) {
        PaError err = Pa_ReadStream(stream, buffer.data(), chunk_size);
        if (err && err != paInputOverflowed) {
            ROS_ERROR("Failed to read audio stream: %s", Pa_GetErrorText(err));
            break;
        }

        std_msgs::Float32MultiArray audio_msg;
        audio_msg.data.assign(buffer.begin(), buffer.end());
        pub.publish(audio_msg);

        if (save_wav_file) {
            wav_file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size() * sample_width);
            if (!wav_file) {
                ROS_ERROR("Failed to write to WAV file");
                break;
            }
        }

        rate.sleep();
    }

    Pa_StopStream(stream);
}

void AudioPublisher::writeWavHeader() {
    wav_file.write("RIFF", 4);
    wav_file.write("\0\0\0\0", 4);
    wav_file.write("WAVE", 4);
    wav_file.write("fmt ", 4);
    int subchunk1_size = 16;
    wav_file.write(reinterpret_cast<const char*>(&subchunk1_size), 4);
    short audio_format = 3;
    wav_file.write(reinterpret_cast<const char*>(&audio_format), 2);
    wav_file.write(reinterpret_cast<const char*>(&channels), 2);
    wav_file.write(reinterpret_cast<const char*>(&sample_rate), 4);
    int byte_rate = sample_rate * channels * sample_width;
    wav_file.write(reinterpret_cast<const char*>(&byte_rate), 4);
    short block_align = channels * sample_width;
    wav_file.write(reinterpret_cast<const char*>(&block_align), 2);
    short bits_per_sample = sample_width * 8;
    wav_file.write(reinterpret_cast<const char*>(&bits_per_sample), 2);
    wav_file.write("data", 4);
    wav_file.write("\0\0\0\0", 4);
}

void AudioPublisher::updateWavHeader() {
    std::streamoff file_size = wav_file.tellp();
    int data_size = static_cast<int>(file_size - 44);
    wav_file.seekp(4);
    int file_size_minus_8 = static_cast<int>(file_size - 8);
    wav_file.write(reinterpret_cast<const char*>(&file_size_minus_8), 4);
    wav_file.seekp(40);
    wav_file.write(reinterpret_cast<const char*>(&data_size), 4);
}