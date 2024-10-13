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
    SumData(): Node("sum_node"), sine_(0.0), rand_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Summing sine and random numbers");

        // Publikáló az "out" topicra
        pub1_ = this->create_publisher<std_msgs::msg::Float32>("out", 10);

        // Előfizetések a "sine" és "rand" topicokra
        sub1_ = this->create_subscription<std_msgs::msg::Float32>("sine", 10, std::bind(&SumData::sine_callback, this, _1));
        sub2_ = this->create_subscription<std_msgs::msg::Float32>("rand", 10, std::bind(&SumData::rand_callback, this, _1));

        // Timer, ami 50ms-enként futtatja az összegző callbacket
        timer_ = this->create_wall_timer(50ms, std::bind(&SumData::timer_callback, this));
    }

private:
    // Az időzítő callbackje, ami publikálja az összegzett adatokat
    void timer_callback()
    {
        auto message_sum = std_msgs::msg::Float32();
        message_sum.data = sine_ + rand_;
        RCLCPP_INFO(this->get_logger(), "Publishing sum: %f", message_sum.data);
        pub1_->publish(message_sum);
    }

    // A szinusz adatokat fogadó callback
    void sine_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received sine: %f", msg->data);
        sine_ = msg->data;
    }

    // A random adatokat fogadó callback
    void rand_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received rand: %f", msg->data);
        rand_ = msg->data;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub1_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub1_, sub2_;
    double sine_, rand_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumData>());
    rclcpp::shutdown();
    return 0;
}
