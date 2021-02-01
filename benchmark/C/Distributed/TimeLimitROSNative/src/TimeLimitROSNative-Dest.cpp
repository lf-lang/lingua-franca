#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#define QUEUE_DEPTH 1000

using namespace std::chrono_literals;

class TimeLimitDestination;

std::shared_ptr<TimeLimitDestination> destination;

class TimeLimitDestination : public rclcpp::Node {
    public:
        TimeLimitDestination()
        : Node("TimeLimit_Destination") {
            destination_ = this->create_subscription<std_msgs::msg::Int32>("TimeLimit_y", 
                                                                           QUEUE_DEPTH,
                                                                           std::bind(&TimeLimitDestination::x_callback,
                                                                           this,
                                                                           std::placeholders::_1));
            start_time = std::chrono::high_resolution_clock::now();
            s_ = 1;
            received_messages_ = 0;
            dropped_messages_ = 0;
        }

        void shutdown() {            
            RCLCPP_INFO(this->get_logger(), "**** shutdown invoked.");
            if (s_ != 10000002) {
                 RCLCPP_INFO(this->get_logger(), "Expected 10000002 but got %d.", s_);
            }
            auto physical_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = physical_time.time_since_epoch() - start_time.time_since_epoch();
            RCLCPP_INFO(this->get_logger(), "Approx. time per message: %lldns" 
                " Dropped messages: %d",
                elapsed_time/(received_messages_+1),
                dropped_messages_);
        }

    private:
        void x_callback(const std_msgs::msg::Int32::SharedPtr msg) const {
            destination->update(msg->data);
        }

        void update(int data) {
            received_messages_++;
            if (data != s_) {
                RCLCPP_INFO(this->get_logger(), "Expected %d and got %d. " 
                    "Jumping my expectation to %d in order to catch up with the sender.", 
                    s_, data, data);
                dropped_messages_ += data - s_;
                s_ = data;
            }
            s_++;            
        }

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr destination_;
        int s_;
        int received_messages_;
        int dropped_messages_;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    destination = std::make_shared<TimeLimitDestination>();
    std::chrono::duration<int> ten_seconds = 10s; // Timeout to be enforced if the following promise is not fullfilled
    std::promise<void> false_promise; // Never fullfilled
    rclcpp::spin_until_future_complete(destination,
                                       false_promise.get_future().share(),
                                       ten_seconds);
    destination->shutdown();
    rclcpp::shutdown();
    return 0;
 }