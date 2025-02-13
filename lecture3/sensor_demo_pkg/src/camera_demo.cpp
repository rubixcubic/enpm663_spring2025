#include <camera_demo_interface.hpp>

void CameraDemoInterface::publish_camera_data()
{
    auto message = sensor_msgs::msg::Image();

    // Set header information
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "camera_frame";

    // Set image parameters
    const uint32_t width = 640;  // Image width
    const uint32_t height = 480; // Image height
    const uint32_t channels = 3; // RGB
    const uint32_t step = width * channels;

    message.height = height;
    message.width = width;
    message.encoding = "rgb8";
    message.is_bigendian = false;
    message.step = step;

    // Generate random pixel data
    std::vector<uint8_t> data(height * step);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> pixel_dist(0, 255);

    for (auto &pixel : data)
    {
        pixel = static_cast<uint8_t>(pixel_dist(gen));
    }

    message.data = data;

    // Publish the message
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published random camera image");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDemoInterface>());
    rclcpp::shutdown();
}