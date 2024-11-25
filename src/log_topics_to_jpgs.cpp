
#include "mess2_logger_cpp/log_topics_to_jpgs.hpp"

namespace fs = boost::filesystem;
namespace mess2_logger_cpp
{
LogTopicsToJPGs::LogTopicsToJPGs() : Node("log_topics_to_jpgs")
{
    // declare parameters
    this->declare_parameter("dir_logs", "Desktop/logs");
    this->declare_parameter("dirs_sub", std::vector<std::string>{"flir"});
    this->declare_parameter("names_actors", std::vector<std::string>{"flir2"});
    this->declare_parameter("topics", std::vector<std::string>{"/flir2/image_raw"});

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
    for (int64_t iter = 0; iter < static_cast<int64_t>(topics_.size()); ++iter) {
        const std::string& topic = topics_[iter];
        const std::string& name_actor = names_actors_[iter];

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription = this->create_subscription<sensor_msgs::msg::Image>(topic, 10, [this, name_actor](const std::shared_ptr<sensor_msgs::msg::Image> msg) { this->callback(name_actor, msg); });

        loggers_[topic] = subscription;
    }
};

void LogTopicsToJPGs::callback(const std::string& name_actor, const std::shared_ptr<sensor_msgs::msg::Image> msg){
    try {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);

        rclcpp::Time stamp_time = msg->header.stamp;
        std::ostringstream name_file;
        name_file << path_log_.string() << "/" << name_actor << "/" << "images" << "/";
        name_file << name_actor << "_" << std::setw(10) << std::setfill('0') << stamp_time.nanoseconds() << ".tiff";

        cv::imwrite(name_file.str(), cv_ptr->image);

    } catch (const cv_bridge::Exception& e) {
        //
    }
}

} // namespace mess2_logger_cpp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mess2_logger_cpp::LogTopicsToJPGs>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
