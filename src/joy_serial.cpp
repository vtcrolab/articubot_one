#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <serial/serial.h>
#include <string>
#include <sstream>

class JoyToSerial : public rclcpp::Node {
public:
    JoyToSerial() : Node("joy_to_serial"), current_speed(0) {
        // Initialize Serial Connection
        try {
            serial_port.setPort("/dev/ttyAMA0"); // Ensure this is the correct port
            serial_port.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port.setTimeout(timeout);
            serial_port.open();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        }

        // Subscribe to joystick topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToSerial::joy_callback, this, std::placeholders::_1));

        // Steering and Speed parameters
        servo_min = 60;   // Left max
        servo_max = 120;  // Right max
        servo_center = 90; // Straight

        speed_min = 0;    // Min speed
        speed_max = 255;  // Max speed
    }

private:
    serial::Serial serial_port;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    int servo_min, servo_max, servo_center;
    int speed_min, speed_max, current_speed;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::stringstream serial_command;

        // Get steering input from the Left Stick X-Axis
        float steering_input = msg->axes[0];  // Left (-1) to Right (+1)
        int servo_angle = static_cast<int>(servo_center + (steering_input * (servo_max - servo_center)));
        serial_command << "S" << servo_angle;

        // Get motor speed input from RT & LT buttons (Triggers)
        bool rt_pressed = msg->buttons[7];  // RT increases speed
        bool lt_pressed = msg->buttons[6];  // LT decreases speed

        if (rt_pressed) {
            current_speed = std::min(current_speed + 20, speed_max);  // Increase speed
        } else if (lt_pressed) {
            current_speed = std::max(current_speed - 20, speed_min);  // Decrease speed
        }

        serial_command << "M" << current_speed;

        // Button actions
        bool button_y = msg->buttons[4]; // Y for Forward
        bool button_a = msg->buttons[0]; // A for Backward
        bool button_x = msg->buttons[3]; // X for Halt
        // bool button_b = msg->buttons[2];

        if (button_y) {
            serial_command << "F";
        } else if (button_a) {
            serial_command << "B";
        } else if (button_x) {
            serial_command << "H"; // Halt
        }

        serial_command << "\n";

        // Send the command via Serial
        if (serial_port.isOpen()) {
            serial_port.write(serial_command.str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open. Failed to send command.");
        }

        // Debugging logs
        RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", serial_command.str().c_str());
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyToSerial>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
