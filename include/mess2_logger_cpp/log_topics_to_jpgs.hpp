#ifndef MESS2_LOGGER_LOG_TOPICS_TO_JPGS_HPP
#define MESS2_LOGGER_LOG_TOPICS_TO_JPGS_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include </usr/include/boost/filesystem.hpp>
#include </usr/include/opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace fs = boost::filesystem;
namespace mess2_logger_cpp
{
class LogTopicsToJPGs : public rclcpp::Node
{
public:
    /**
     * @brief A class to log multiple ROS2 topics of image msgs to log .jpg files.
     * 
     * This class initializes logger instances for specific ROS2 topics with sensor_msgs::msg::Image msg types. It supports dynamic subscription creation from a parameter list of topics.
     */
    LogTopicsToJPGs();

private:
    /**
     * @brief Callback function to process incoming image messages and save them as .jpg files.
     * 
     * @param name_actor The name of the actor associated with the topic.
     * @param msg The image message received from the subscription.
     */
    void callback(const std::string& name_actor, const std::shared_ptr<sensor_msgs::msg::Image> msg);

    std::string dir_logs_;
    std::vector<std::string> dirs_sub_;
    std::vector<std::string> names_actors_;
    std::vector<std::string> topics_;
    fs::path path_log_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> loggers_;
};
} // namespace mess2_logger_cpp

#endif // MESS2_LOGGER_LOG_TOPICS_TO_JPGS_HPP
