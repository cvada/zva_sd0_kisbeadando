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
    SumData(): Node("sum_node"), sine_(0.0), rand_(0.0), in_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Summing numbers");

        // Publisher létrehozása az "out" topicra
        pub1_ = this->create_publisher<std_msgs::msg::Float32>("out", 10);

        // Subscriber-ek létrehozása a "sine", "rand" és "in" topicokra
        sub1_ = this->create_subscription<std_msgs::msg::Float32>("sine", 10, std::bind(&SumData::sine_callback, this, _1));
        sub2_ = this->create_subscription<std_msgs::msg::Float32>("rand", 10, std::bind(&SumData::rand_callback, this, _1));
        sub3_ = this->create_subscription<std_msgs::msg::Float32>("in", 10, std::bind(&SumData::in_callback, this, _1));

        // Timer létrehozása 50 ms periódussal
        timer_ = this->create_wall_timer(50ms, std::bind(&SumData::timer_callback, this));
    }

private:
    // Timer callback, amely összeadja az értékeket és publikálja
    void timer_callback()
    {
        auto message_sum = std_msgs::msg::Float32();
        message_sum.data = sine_ + rand_ + in_;  // Az adatok összeadása
        pub1_->publish(message_sum);  // Az eredmény publikálása az "out" topicra
    }

    // "sine" topic callback
    void sine_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        sine_ = msg->data;  // A beérkező szinusz érték tárolása
    }

    // "rand" topic callback
    void rand_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        rand_ = msg->data;  // A beérkező véletlenszám tárolása
    }

    // "in" topic callback (külső adat forrás)
    void in_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        in_ = msg->data;  // A beérkező "in" érték tárolása
    }

    // Publikáló, subscriber-ek és timer tárolása
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub1_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub1_, sub2_, sub3_;

    // Értékek tárolása
    double sine_, rand_, in_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumData>());  // Node futtatása
    rclcpp::shutdown();
    return 0;
}