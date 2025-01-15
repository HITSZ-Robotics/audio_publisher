#include "audio_publisher/audio_publisher_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "audio_publisher");

    ros::NodeHandle nh_private("~");

    // Get parameters
    bool save_wav = true;
    std::string topic_name = "audio_stream"; // Default topic name
    nh_private.param("save_wav", save_wav, true);
    nh_private.param("topic_name", topic_name, std::string("audio_stream")); // Get topic name from parameter server

    try {
        AudioPublisher audio_pub(topic_name, save_wav); // Pass custom topic name
        audio_pub.publishAudio();
    } catch (const std::exception &e) {
        ROS_ERROR("An error occurred: %s", e.what());
    }

    return 0;
}