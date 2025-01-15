#ifndef AUDIO_PUBLISHER_NODE_H
#define AUDIO_PUBLISHER_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <portaudio.h>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <string>

class AudioPublisher {
public:
    AudioPublisher(const std::string &topic_name, bool save_wav);
    ~AudioPublisher();

    void publishAudio();

private:
    ros::Publisher pub;
    PaStream *stream;
    PaStreamParameters inputParameters;
    std::ofstream wav_file;

    const int chunk_size;
    const int sample_rate;
    const int channels;
    const int sample_width;
    const bool save_wav_file;

    std::string topic_name; 

    void writeWavHeader();
    void updateWavHeader();
};

#endif // AUDIO_PUBLISHER_NODE_H
