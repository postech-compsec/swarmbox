#pragma once

// C++ Standard Libraries
#include <string>
#include <fstream>

// base node
#include "sb_base/sb_node.hpp"

// Custom Messages from this package
#include "sb_base/msg/heartbeat.hpp"
#include "sb_base/msg/report.hpp"
#include "sb_base/msg/task_command.hpp"
#include "sb_base/msg/prox_alert.hpp"


class Ground : public SBNode {
public:
    //================================================================
    // Public Interface
    //================================================================
    Ground(const rclcpp::NodeOptions & options);
    virtual ~Ground();
    void parse_config_file(const std::string& config_path);

protected:
    //================================================================
    // Protected Interface for derived classes
    //================================================================
    
    // --- Stage-specific virtual functions for customization ---
    virtual void prep_once();
    virtual void prep_loop();
    virtual void exec_once();
    virtual void exec_loop();

    // --- Logger ---
    std::ofstream log_file_;

    // --- Current State Variables ---
    int stage;
    int total_swarm_size;
    int conn_total;
    int conn_alive;
    // bool leader_found;
    uint tick;

    std::string mission_type_;
    std::map<int, std::vector<double>> values_;


    // --- Direct Inferior Info ---
    // int leader_id;
    int inf_alive;
    int inf_count;
    int inf_complete_count;
    std::vector<int> inf_list = {};
    std::map<int, Eigen::Vector3d> inf_pos;
    std::map<int, bool> inf_completes = {};
    std::map<int, sb_base::msg::Report> inf_reports = {};
    std::map<int, int> inf_stage = {};
    std::map<int, int> inf_lost_timer = {};
    std::map<int, std::vector<uint8_t>> inf_descs = {}; // descendants of each inferior, indexed by their id in inf_list
    std::map<int, int> inf_mapper = {}; // inverse mapping inferior
    // discrepancy handling
    std::map<int, Eigen::Vector3d> inf_pos_dis;   // position discrepancies of inferiors
    // network delay handling
    std::map<int, double> inf_delays;
    // loss handling
    std::map<int, int> inf_packets_received;
    std::map<int, double> inf_loss_ratio_;
    const int loss_window_ticks = 100; // 2.5s


    std::map<int, std::vector<int>> proximity_alerts; // list of drones that are likely to collide: [my_id][prox_ids]
    
    // --- Publishers ---
    std::map<int, rclcpp::Publisher<sb_base::msg::TaskCommand>::SharedPtr> pub_task_cmds_;
    std::map<int, rclcpp::Publisher<sb_base::msg::ProxAlert>::SharedPtr> pub_prox_alerts_;

    void box_publish_task(int target_id, const sb_base::msg::TaskCommand &task_cmd) {
        if (pub_task_cmds_.find(target_id) == pub_task_cmds_.end()) {
            RCLCPP_WARN(this->get_logger(), "Invalid target_id: %d", target_id);
            return;
        }
        pub_task_cmds_[target_id]->publish(task_cmd);
    }

private:
    //================================================================
    // Private Implementation
    //================================================================
    void timer_callback();
    void report_callback(const sb_base::msg::Report::SharedPtr report);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sb_base::msg::Heartbeat>::SharedPtr pub_heartbeat_;
    rclcpp::Subscription<sb_base::msg::Report>::SharedPtr sub_reports_;

    void stage_changes(); // Handle stage changes and logging
    void prox_detector(int dist, int timewindow); // proximity alert system
    std::optional<std::tuple<double, double>> iqr_outlier_detector(std::vector<double> data, double threshold);


    // int shake_lost_count;
};
