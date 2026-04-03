#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <type_traits>
#include <eigen3/Eigen/Eigen>


// Stage definitions
#define STAGE_ABRT -1
#define STAGE_IDLE  0
#define STAGE_ICEB  1
#define STAGE_PREP  2
#define STAGE_EXEC  3
#define STAGE_FINI  4
#define STAGE_TERM  5
#define LOST_THRESH 20*1

struct setpoint{ // setpoints should be created as world frame.
    double north, east, down, yaw;
    uint8_t type = 0;
};

class SBNode : public rclcpp::Node {
public:
    SBNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : rclcpp::Node(node_name, options) {}
    
protected:
    /**
     * @brief User-defined marker for logging purposes.
     * 
     * This function uses RCLCPP_INFO to log a user-defined marker with a unique identifier and a formatted message.
     * example: marker(101, "What you want to log: %s", "Hello World");
     * 
     * @tparam Args variable argument types for format string
     * @param marker_id Unique identifier for the marker.
     * @param format_str C-style format string 
     * @param args Variable arguments to format the string.
     */
    template<typename... Args>
    void marker(int marker_id, const char* format_str, Args... args) {
        std::string new_format = "MARKER [%d]: " + std::string(format_str);
        RCLCPP_INFO(this->get_logger(), new_format.c_str(), marker_id, args...);
    }
    template<typename... Args>
    void marker_once(int marker_id, const char* format_str, Args... args) {
        std::string new_format = "MARKER [%d]: " + std::string(format_str);
        RCLCPP_INFO_ONCE(this->get_logger(), new_format.c_str(), marker_id, args...);
    }
    template<typename... Args>
    void marker_debug(const char* format_str, Args... args) {
        std::string new_format = "DEBUG_MARKER: " + std::string(format_str);
        RCLCPP_DEBUG(this->get_logger(), new_format.c_str(), args...);
    }
    template<typename... Args>
    void marker_debug_once(const char* format_str, Args... args) {
        std::string new_format = "DEBUG_MARKER: " + std::string(format_str);
        RCLCPP_DEBUG_ONCE(this->get_logger(), new_format.c_str(), args...);
    }


    template<typename T, typename... Args,
             typename = std::enable_if_t<!std::is_integral<std::decay_t<T>>::value>>
    void marker(T&& format_str, Args&&... args) {
        marker(-1, std::forward<T>(format_str), std::forward<Args>(args)...);
    }
    
    template<typename T, typename... Args,
             typename = std::enable_if_t<!std::is_integral<std::decay_t<T>>::value>>
    void marker_once(T&& format_str, Args&&... args) {
        marker_once(-1, std::forward<T>(format_str), std::forward<Args>(args)...);
    }
    // void marker(int marker_id, const char* format_str, Args... args);
};