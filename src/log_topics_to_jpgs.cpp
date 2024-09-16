
#include "mess2_logger_cpp/log_topics_to_jpgs.hpp"

namespace fs = boost::filesystem;
namespace mess2_logger_cpp
{
/**
 * @brief A class to log multiple ROS2 topics of image msgs to log .jpg files.
 * 
 * This class initializes logger instances for specific ROS2 topics with sensor_msgs::msg::Image msg types. It supports dynamic subscription creation from a parameter list of topics.
 */
LogTopicsToJPGs::LogTopicsToJPGs() : Node("log_topics_to_jpgs")
{
    // declare parameters
    this->declare_parameter("dir_logs", "Projets/testing");
    this->declare_parameter("dirs_sub", std::vector<std::string>{"actor"});
    this->declare_parameter("names_actors", std::vector<std::string>{"flir1", "flir2"});
    this->declare_parameter("topics", std::vector<std::string>{"/topic1", "/topic2"});

    // retrieve parameters
    this->get_parameter("dir_logs", dir_logs_);
    this->get_parameter("dirs_sub", dirs_sub_);
    this->get_parameter("names_actors", names_actors_);
    this->get_parameter("topics", topics_);

    // create log directories if they do not exist
    const auto& dir_home = std::getenv("HOME");
    path_log_ = fs::path(dir_home) / fs::path(dir_logs_);
    for (const auto& dir_sub : dirs_sub_) {
        path_log_ /= dir_sub;
    }

    for (const auto& name_actor : names_actors_) {
        auto path_jpg = path_log_ / name_actor / "images";
        if (!fs::exists(path_jpg)) {
            RCLCPP_INFO(this->get_logger(), "%s", path_jpg.c_str());
            fs::create_directories(path_jpg);
        }

        // empty log directories if they contain files
        fs::directory_iterator iter;
        for (fs::directory_iterator jter(path_jpg); jter != iter; ++jter) {
            const fs::path& path_file = jter->path();
            if (fs::is_regular_file(path_file) || fs::is_symlink(path_file)) {
                fs::remove(path_file);
            } else if (fs::is_directory(path_file)) {
                fs::remove_all(path_file);
            }
        }
    }    

    // ensure topics_ and names_actors_ have the same size
    if (topics_.size() != names_actors_.size()) {
        throw std::runtime_error("the number of topics must match the number of actors");
    }

    // create subscription for each topic
    for (size_t iter = 0; iter < topics_.size(); ++iter) {
        const auto& topic = topics_[iter];
        const auto& name_actor = names_actors_[iter];
        auto subscription = create_subscription<sensor_msgs::msg::Image>(
            topic, 10, [this, name_actor](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->callback(msg, name_actor);
            }
        );
        loggers_[topic] = subscription;
    }
};

/**
 * 
 */
void LogTopicsToJPGs::callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& name_actor){
    try {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

        rclcpp::Time stamp_time = msg->header.stamp;
        std::ostringstream name_file;
        name_file << path_log_.string() << "/" << name_actor << "/" << "images" << "/";
        name_file << name_actor << "_" << std::setw(10) << std::setfill('0') << stamp_time.nanoseconds() << ".jpg";

        cv::imwrite(name_file.str(), cv_ptr->image);

        RCLCPP_INFO(this->get_logger(), "Saved image file: %s", name_file.str().c_str());

    } catch (const cv_bridge::Exception& e) {
        //
    }
}

} // namespace mess2_logger_cpp

/**
 * 
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mess2_logger_cpp::LogTopicsToJPGs>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
