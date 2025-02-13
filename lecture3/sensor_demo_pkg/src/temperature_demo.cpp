#include <temperature_demo_interface.hpp>

void TemperatureDemoInterface::publish_temperature_data() {
        // Create a Temperature message
        auto message = sensor_msgs::msg::Temperature();

        // Set header information
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "temperature_frame";

        // Generate random temperature data
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> temp_dist(-10.0, 40.0);  // Random temperature between -10°C and 40°C

        // Set temperature data
        message.temperature = temp_dist(gen);  // Random temperature
        message.variance = 0.5;  // Variance in the measurement

        // Publish the message
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published temperature: %.2f°C", message.temperature);
    }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureDemoInterface>());
    rclcpp::shutdown();
}