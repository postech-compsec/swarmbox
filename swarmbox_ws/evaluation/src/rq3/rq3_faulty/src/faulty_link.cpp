#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <random>
#include <stdexcept> // Required for std::runtime_error

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

// Message types are still needed for the mapping
#include "sb_base/msg/heartbeat.hpp"
#include "sb_base/msg/report.hpp"
#include "sb_base/msg/task_command.hpp"
#include "sb_base/msg/prox_alert.hpp"

using namespace std::chrono_literals;

struct DelayedMsg {
    rclcpp::Time publish_time;
    rclcpp::SerializedMessage serialized_msg;
};

class FaultyLinkNode : public rclcpp::Node {
public:
    int target_id_;
    FaultyLinkNode() : Node("faulty_link_node") { // Node name can be anything, launch file will override
        this->declare_parameter<std::string>("topic", "task_cmd");
        this->declare_parameter<int>("target", 0);
        this->declare_parameter<double>("delay_ms", 100.0);
        this->declare_parameter<double>("loss_rate", 0.5);

        std::string topic_name = this->get_parameter("topic").as_string();
        target_id_ = this->get_parameter("target").as_int();
        delay_ms_ = this->get_parameter("delay_ms").as_double();
        loss_rate_ = this->get_parameter("loss_rate").as_double();

        // target_id_ = std::stoi(target_name.substr(target_name.find("_") + 1));
        // size_t pos = target_name.find("_");

        // if (pos != std::string::npos) {
        //     std::string id_str = target_name.substr(pos + 1);

        //     try {
        //         this->target_id_ = std::stoi(id_str);
        //         RCLCPP_INFO(this->get_logger(), "Successfully parsed target_id: %d from target_name: %s", target_id_, target_name.c_str());
        //     } catch (const std::invalid_argument& e) {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to parse target_id: '%s' is not a valid number. (%s)", id_str.c_str(), e.what());
        //     } catch (const std::out_of_range& e) {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to parse target_id: '%s' is out of range. (%s)", id_str.c_str(), e.what());
        //     }
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "target_name '%s' does not contain '_', cannot parse target_id.", target_name.c_str());
        // }

        RCLCPP_INFO(this->get_logger(), "Fault Injection Node Initialized for topic '%s' on target '%d'", topic_name.c_str(), target_id_);
        RCLCPP_INFO(this->get_logger(), "Delay: %.2f ms, Loss: %.2f%%", delay_ms_, loss_rate_ * 100.0);

        // --- THIS IS THE FIX ---
        // Map the short topic name to the full ROS2 message type name.
        std::string message_type;
        if (topic_name == "report") {
            message_type = "sb_base/msg/Report";
        } else if (topic_name == "heartbeat") {
            message_type = "sb_base/msg/Heartbeat";
        } else if (topic_name == "task_cmd") {
            message_type = "sb_base/msg/TaskCommand";
        } else if (topic_name == "prox_alert") {
            message_type = "sb_base/msg/ProxAlert";
        } else {
            // If we get an unknown topic, throw an error to prevent crashing later.
            RCLCPP_FATAL(this->get_logger(), "Unsupported topic type: %s", topic_name.c_str());
            throw std::runtime_error("Unsupported topic type provided");
        }
        RCLCPP_INFO(this->get_logger(), "Mapping topic '%s' to message type '%s'", topic_name.c_str(), message_type.c_str());
        // --- END OF FIX ---

        rng_ = std::mt19937(rd_());
        dist_ = std::uniform_real_distribution<double>(0.0, 1.0);


        std::string target_name = (message_type == "sb_base/msg/Report" || message_type == "sb_base/msg/Heartbeat") ? "gcs" : "drone_" + std::to_string(target_id_);
        std::string input_topic = "/" + target_name + "/" + topic_name;
        std::string output_topic = input_topic + "_faulty";
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: {%s} -> {%s}", input_topic.c_str(), output_topic.c_str());

        // Now, pass the correct message_type string instead of ""
        publisher_ = this->create_generic_publisher(output_topic, message_type, rclcpp::QoS(10));

        auto generic_callback = [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            this->topic_callback(msg);
        };
        subscription_ = this->create_generic_subscription(input_topic, message_type, rclcpp::QoS(10), generic_callback);
        
        timer_ = this->create_wall_timer(1ms, std::bind(&FaultyLinkNode::timer_callback, this));
    }

private:
    void topic_callback(const std::shared_ptr<rclcpp::SerializedMessage>& msg) {
        // if I am reading 'report' of gcs, I should selectively delay or drop only to the ones with orig_id == target_id. else: just keep and replay as is.
        RCLCPP_INFO_ONCE(this->get_logger(), "Message received on topic %s", subscription_->get_topic_name());
        std::string tp = subscription_->get_topic_name();
        if (tp.find("report") != std::string::npos) {
            rclcpp::Serialization<sb_base::msg::Report> serializer;
            sb_base::msg::Report report_msg;
            serializer.deserialize_message(msg.get(), &report_msg);
            // Check the original ID in the message
            // If it matches the target ID, apply the fault injection
            // Otherwise, just replay the message as is
            if (report_msg.orig_id != this->target_id_) {
                // RCLCPP_INFO(this->get_logger(), "Replaying non-target(%d) message on topic %s without delay/loss", report_msg.orig_id, subscription_->get_topic_name());
                publisher_->publish(*msg);
                return;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Applying fault injection to target(%d) message on topic %s", report_msg.orig_id, subscription_->get_topic_name());
        if (dist_(rng_) < loss_rate_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Message lost on topic %s", subscription_->get_topic_name());
            return;
        }
        DelayedMsg delayed_msg;
        delayed_msg.serialized_msg = *msg;
        delayed_msg.publish_time = this->get_clock()->now() + std::chrono::milliseconds(static_cast<long>(delay_ms_));
        msg_queue_.push_back(std::move(delayed_msg));
    }

    void timer_callback() {
        auto now = this->get_clock()->now();
        msg_queue_.erase(
            std::remove_if(msg_queue_.begin(), msg_queue_.end(),
                [&](const DelayedMsg& delayed_msg) {
                    if (now >= delayed_msg.publish_time) {
                        publisher_->publish(delayed_msg.serialized_msg);
                        return true;
                    }
                    return false;
                }),
            msg_queue_.end());
    }

    rclcpp::GenericPublisher::SharedPtr publisher_;
    rclcpp::GenericSubscription::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double delay_ms_;
    double loss_rate_;

    std::random_device rd_;
    std::mt19937 rng_;
    std::uniform_real_distribution<> dist_;

    std::vector<DelayedMsg> msg_queue_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto faulty_link_node = std::make_shared<FaultyLinkNode>();
    rclcpp::spin(faulty_link_node);
    rclcpp::shutdown();
    return 0;
}