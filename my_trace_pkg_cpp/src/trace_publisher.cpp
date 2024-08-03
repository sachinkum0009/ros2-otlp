#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TracePublisher : public rclcpp::Node
{
public:
    TracePublisher() : Node("trace_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);

        message_.data = "Hello, world!";

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TracePublisher::publishMessage, this));
    }

private:
    void publishMessage()
    {
        publisher_->publish(message_);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std_msgs::msg::String message_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TracePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}