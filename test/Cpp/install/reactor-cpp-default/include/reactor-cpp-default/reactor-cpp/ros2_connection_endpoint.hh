#ifndef REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH
#define REACTOR_CPP_ROS2_CONNECTION_ENDPOINT_HH

#include "connection_endpoint.hh"
#include <type_traits>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

extern rclcpp::Node* lf_node;

namespace reactor {

template <class T>
class ROS2PubEndpoint : public UpstreamEndpoint<T> {
    private:
        std::conditional_t< std::is_same<T,void>::value, 
                            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr, 
                            std::shared_ptr<rclcpp::Publisher<T>> > pub_;

    protected:
        PortCallback set_cb() override{
            return [this](const BasePort& port) {
                auto& typed_port = reinterpret_cast<const Port<T>&>(port); 
                if constexpr (std::is_same<T, void>::value) {
                    std_msgs::msg::Empty msg;
                    pub_->publish(msg);
                } else {
                    pub_->publish(*typed_port.get());
                }
            };
        }

    public:
        ROS2PubEndpoint(const std::string& topic_name) : UpstreamEndpoint<T>() { 
            if constexpr (std::is_same<T, void>::value)
                pub_ = lf_node->create_publisher<std_msgs::msg::Empty>(topic_name, 10);
            else 
                pub_ = lf_node->create_publisher<T>(topic_name, 10);
        }

};

template <class T>
class ROS2SubEndpoint : public DownstreamEndpoint<T> {
    private:
        std::conditional_t< std::is_same<T,void>::value, 
                            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr, 
                            std::shared_ptr<rclcpp::Subscription<T>> > sub_;

    protected:
        void schedule_this(std::shared_ptr<T> p) {
            if constexpr (std::is_same<T, void>::value) {
                this->schedule();
            } else {
                this->schedule(*p.get());
            }
        }

    public:
        ROS2SubEndpoint(const std::string& topic_name, const std::string& name, Environment* environment, bool is_logical, Duration min_delay) 
        : DownstreamEndpoint<T>(name, environment, is_logical, min_delay){
            while (!lf_node->count_publishers(topic_name));
            if constexpr (std::is_same<T, void>::value) {
                std::function<void(std::shared_ptr<std_msgs::msg::Empty>)> f = 
                    std::bind(&ROS2SubEndpoint::schedule_this, this, std::placeholders::_1);
                sub_ = lf_node->create_subscription<std_msgs::msg::Empty>(topic_name, 10,
                            f);
            } else {
                std::function<void(std::shared_ptr<T>)> f = 
                    std::bind(&ROS2SubEndpoint::schedule_this, this, std::placeholders::_1);
                sub_ = lf_node->create_subscription<T>(topic_name, 10,
                            f);
            }
        }
};

} // reactor ns

#endif