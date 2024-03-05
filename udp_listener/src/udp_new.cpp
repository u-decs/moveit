#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  // Replace with the appropriate message type for new_data
#include "std_msgs/msg/float32.hpp" // Replace with the appropriate message type for joint_states

#include <sstream>

class ASCIIToFloatNode : public rclcpp::Node
{
public:
    ASCIIToFloatNode()
        : Node("ascii_to_float_node")
    {
        // Subscribe to the "new_data" topic
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "new_data", 10, std::bind(&ASCIIToFloatNode::newDataCallback, this, std::placeholders::_1));

        // Publish on the "joint_states" topic
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("joint_states", 10);
    }

private:
    void newDataCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            // Assuming ASCII values are space-separated in the received message
            std::istringstream iss(msg->data);
            std::string token;

            while (iss >> token)
            {
                // Convert each ASCII value to float and publish
                float value = std::stof(token);
                auto float_msg = std_msgs::msg::Float32();
                float_msg.data = value;
                publisher_->publish(float_msg);
            }
        }
        catch (const std::invalid_argument &e)
        {
            // Handle invalid conversions
            RCLCPP_ERROR(this->get_logger(), "Invalid argument: %s", e.what());
        }
        catch (const std::out_of_range &e)
        {
            // Handle out of range conversions
            RCLCPP_ERROR(this->get_logger(), "Out of range: %s", e.what());
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ASCIIToFloatNode>());
    rclcpp::shutdown();
    return 0;
}

