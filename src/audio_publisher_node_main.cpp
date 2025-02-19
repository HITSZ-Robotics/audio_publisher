#include "audio_publisher/audio_publisher_node.h"
/*
 * Copyright (c) 2025 zzl410
 *
 * This software is licensed under the MIT License. 
 * You may obtain a copy of the license at:
 * https://opensource.org/licenses/MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * For more details, visit: https://github.com/zzl410/whisper_ros1
 */

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