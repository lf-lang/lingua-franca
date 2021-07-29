#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#define QUEUE_DEPTH 1000

using namespace std::chrono_literals;

class TimeLimitClock : public rclcpp::Node {
    public:
        TimeLimitClock()
        : Node("TimeLimit_Clock") {
            clock_ = this->create_publisher<std_msgs::msg::Int32>("TimeLimit_y", QUEUE_DEPTH);
            timer_ = this->create_wall_timer(15us, std::bind(&TimeLimitClock::timer_callback, this));
            count_ = 0;
        }
    
    private:
        void timer_callback() {
            auto message = std_msgs::msg::Int32();
            message.data = ++count_;
            clock_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr clock_;
        int count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::chrono::duration<int> ten_seconds = 10s; // Timeout to be enforced if the following promise is not fullfilled
    std::promise<void> false_promise; // Never fullfilled
    rclcpp::spin_until_future_complete(std::make_shared<TimeLimitClock>(),
                                       false_promise.get_future().share(),
                                       ten_seconds);
    rclcpp::shutdown();
    return 0;
}