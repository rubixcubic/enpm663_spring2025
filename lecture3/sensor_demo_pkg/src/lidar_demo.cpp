#include <lidar_demo_interface.hpp>

void LidarDemoInterface::publish_lidar_data()
{
    auto message = sensor_msgs::msg::LaserScan();

    // Set header information
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "laser_frame";

    // Set the angle and range parameters
    message.angle_min = -M_PI;
    message.angle_max = M_PI;
    message.angle_increment = M_PI / 180; // 1 degree increments
    message.time_increment = 0.0;
    message.scan_time = 0.1;
    message.range_min = 0.2;
    message.range_max = 10.0;

    // Number of readings based on angle range
    size_t num_readings = static_cast<size_t>((message.angle_max - message.angle_min) / message.angle_increment);

    // Generate random ranges and intensities
    std::vector<float> ranges(num_readings);
    std::vector<float> intensities(num_readings);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> range_dist(message.range_min, message.range_max);
    std::uniform_real_distribution<float> intensity_dist(0.0, 1.0);

    for (size_t i = 0; i < num_readings; ++i)
    {
        ranges[i] = range_dist(gen);
        intensities[i] = intensity_dist(gen);
    }

    message.ranges = ranges;
    message.intensities = intensities;

    // Publish the message
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published lidar data");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarDemoInterface>());
    rclcpp::shutdown();
}