#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>

class UDPPublisherNode : public rclcpp::Node
{
public:
  UDPPublisherNode() : Node("udp_publisher_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("joint_states", 10);
    startUDPServer();
  }

private:
  void startUDPServer()
  {
    const char *udp_ip = "0.0.0.0";  // Listen on any available network interface
    int udp_port = 5500;

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

    memset(&udp_server_address_, 0, sizeof(udp_server_address_));
    udp_server_address_.sin_family = AF_INET;
    udp_server_address_.sin_addr.s_addr = inet_addr(udp_ip);
    udp_server_address_.sin_port = htons(udp_port);

    bind(sockfd_, (const struct sockaddr *)&udp_server_address_, sizeof(udp_server_address_));

    rclcpp::spin_some(this->get_node_base_interface()); // Allow ROS to process any callbacks

    rclcpp::create_timer(
        this,
        rclcpp::Duration(1),
        [this]() -> void {
          char buffer[1024];  // Adjust buffer size as needed

          ssize_t len = recvfrom(sockfd_, buffer, sizeof(buffer), 0, NULL, NULL);
          if (len > 0)
          {
            buffer[len] = '\0';

            auto joint_states_msg = std_msgs::msg::String();
            joint_states_msg.data = buffer;

            publisher_->publish(joint_states_msg);
            RCLCPP_INFO(this->get_logger(), "Received UDP data: %s", buffer);
          }
        });
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int sockfd_;
  struct sockaddr_in udp_server_address_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UDPPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

