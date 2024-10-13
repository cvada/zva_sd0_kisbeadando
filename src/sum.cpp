#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SumData : public rclcpp::Node
{
public:
    SumData(): Node("sum_node"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Summing numbers");
        pub1_ = this->create_publisher<std_msgs::msg::Float32>("out", 10);
        sub1_ = this->create_subscription<std_msgs::msg::Float32>("sine", 10, std::bind(&SumData::sine_callback, this, _1));
        sub2_ = this->create_subscription<std_msgs::msg::Float32>("rand", 10, std::bind(&SumData::rand_callback, this, _1));
        sub3_ = this->create_subscription<std_msgs::msg::Float32>("in", 10, std::bind(&SumData::in_callback, this, _1));
        timer_ = this->create_wall_timer(50ms, std::bind(&SumData::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message_sum = std_msgs::msg::Float32();
        message_sum.data = sine_ + rand_ + in_;
        pub1_->publish(message_sum);
    }
    void sine_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Sine: '%f'", msg->data);
        sine_ = msg->data;
    }
    void rand_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Rand: '%f'", msg->data);
        rand_ = msg->data;
    }
    void in_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "In: '%f'", msg->data);
        in_ = msg->data;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub1_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub1_, sub2_, sub3_;
    double sum_, sine_, rand_, in_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumData>());
    rclcpp::shutdown();
    return 0;
}
