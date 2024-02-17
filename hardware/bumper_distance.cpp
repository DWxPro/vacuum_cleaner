#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>

class BumperDistance : public rclcpp::Node
{
public:
    BumperDistance() : Node("bumper_distance")
    {
        settings_left = {"field_of_view": 30.0, "min_range": 0.0, "max_range": 1.0, "frame_id": "distance_sensor_left"};
        settings_front = {"field_of_view": 30.0, "min_range": 0.0, "max_range": 1.0, "frame_id": "distance_sensor_front"};
        settings_right = {"field_of_view": 30.0, "min_range": 0.0, "max_range": 1.0, "frame_id": "distance_sensor_right"};

        scale_factor = 0.0001;

        distance_left_pub = this->create_publisher<sensor_msgs::msg::Range>("/bumper/distance/left", 10);
        distance_front_pub = this->create_publisher<sensor_msgs::msg::Range>("/bumper/distance/front", 10);
        distance_right_pub = this->create_publisher<sensor_msgs::msg::Range>("/bumper/distance/right", 10);

        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(9600);
        ser.setTimeout(1000);
        ser.open();

        RCLCPP_INFO(this->get_logger(), "node \"bumper_distance\" has been started");
    }

    void read_serial_data()
    {
        while (rclcpp::ok())
        {
            if (ser.available())
            {
                std::string serial_data = ser.readline();
                std::istringstream iss(serial_data);
                std::string item;
                std::vector<std::string> tokens;
                while (std::getline(iss, item, ' '))
                {
                    tokens.push_back(item);
                }

                float range_left = std::stof(tokens[1]) * scale_factor;
                float range_front = std::stof(tokens[3]) * scale_factor;
                float range_right = std::stof(tokens[5]) * scale_factor;

                publish_distance(distance_left_pub, settings_left, range_left);
                publish_distance(distance_front_pub, settings_front, range_front);
                publish_distance(distance_right_pub, settings_right, range_right);

                RCLCPP_INFO(this->get_logger(), "left: %f front: %f right: %f", range_left, range_front, range_right);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

private:
    void publish_distance(rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub, std::map<std::string, float> settings, float range)
    {
        auto msg = std::make_shared<sensor_msgs::msg::Range>();
        msg->radiation_type = 1;
        msg->field_of_view = settings["field_of_view"];
        msg->min_range = settings["min_range"];
        msg->max_range = settings["max_range"];
        msg->header.frame_id = settings["frame_id"];
        msg->range = range;

        pub->publish(*msg);
    }

    std::map<std::string, float> settings_left;
    std::map<std::string, float> settings_front;
    std::map<std::string, float> settings_right;
    float scale_factor;
    serial::Serial ser;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr distance_left_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr distance_front_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr distance_right_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BumperDistance>();

    try
    {
        node->read_serial_data();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
