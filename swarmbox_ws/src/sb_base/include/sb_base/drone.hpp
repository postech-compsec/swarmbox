#pragma once

// C++ Standard Libraries
#include <vector>
#include <string>
#include <cmath>
#include <optional>

// base node
#include "sb_base/sb_node.hpp"

// PX4 Messages
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

// Custom Messages from this package
#include "sb_base/msg/heartbeat.hpp"
#include "sb_base/msg/report.hpp"
#include "sb_base/msg/task_command.hpp"
#include "sb_base/msg/prox_alert.hpp"


const double EARTH_RADIUS = 6378137.0;
const double METER_PER_DEG_LAT = 111320.0;


class Drone : public SBNode {
public:
    //================================================================
    // Public Interface: Accessible from anywhere
    //================================================================
    Drone(const rclcpp::NodeOptions & options);
    Drone(int id, int superior, float location[3]);
    virtual ~Drone();

    rclcpp::Node::SharedPtr get_px4_node() {return px4_node_;}

protected:
    //================================================================
    // Protected Interface: Accessible by this class and derived classes (e.g., UserDrone)
    //================================================================

    bool hasReached(setpoint target_pos,
                   const float d_threshold = 0.7f, const float v_threshold = 0.7f);
    
    // --- Connection Resilience functions ---
    virtual std::vector<int16_t> assign_successors();
    virtual bool update_superior();

    // --- Stage-specific virtual functions for customization ---
    // These functions can be overridden by a derived class to change behavior.
    virtual void prep_once();
    virtual void prep_loop();
    virtual bool prep_complete();
    virtual void exec_once();
    virtual std::optional<setpoint> exec_loop();
    virtual bool exec_complete();

    std::mutex state_mutex_;


    // --- Current State Variables ---
    // Derived classes might need to read these to make decisions.
    int             identity;
    int             stage;
    bool            complete;
    Eigen::Vector3d init_pos;       // initial relative location (spawn point NED)
    Eigen::Vector3d init_gpos = {NAN, NAN, NAN};
    Eigen::Vector3d discrepancy; // discrepancy between NED and GPS position
    Eigen::Vector3d world_pos;      // current world position (NED)
    Eigen::Vector3d velocity;       // current velocity (NED)
    double          z_bias;
    bool            armed;
    uint tick;

    // Superior info
    int                     sup_id;
    std::vector<int16_t>    successors;
    int                     success_no; // id of current successor choice. if the connection is still alive, the value is -1.
    std::vector<int16_t>    emergency_contacts;

    // Descendant info
    int             total_count; // represents how many of my descendants are there.
    int             total_alive; // represents how many of my descendants are alive.
    
    // Inferior info
    int                 inf_alive; // represents how many of my DIRECT inferiors are alive.
    int                 inf_count; // represents how many of my DIRECT inferiors are there.
    int                 inf_complete_count;
    std::vector<int>    inf_list;
    std::map<int, bool>   inf_completes;
    std::map<int, sb_base::msg::Report> inf_reports;
    std::map<int, int>    inf_stage;
    std::map<int, int>    inf_lost_timer;
    std::map<int, std::vector<uint8_t>> inf_descs; // descendants of each inferior, indexed by their id in inf_list
    std::map<int, int> inf_mapper; // inverse mapping inferior

    // mission info
    std::string              report_data;
    

    // PX4 and Task Data
    px4_msgs::msg::VehicleGlobalPosition    px4_global_pos_;
    px4_msgs::msg::SensorGps                px4_sensorgps_;
    bool gps_updated = false;

    px4_msgs::msg::VehicleLocalPosition     px4_local_pos_;
    px4_msgs::msg::VehicleCommand           px4_vehicle_cmd_;

    sb_base::msg::TaskCommand               box_rcvd_task_cmd_;
    sb_base::msg::ProxAlert                 box_prox_alert_;

    // --- Publishers ---
    // Derived classes might need to use these publishers directly.
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_pub_traj_setpoint_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr     px4_pub_command_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr px4_sub_land_detected_;
    std::map<int, rclcpp::Publisher<sb_base::msg::TaskCommand>::SharedPtr> pub_task_cmds_;

    px4_msgs::msg::TrajectorySetpoint last_valid_traj_;

    void box_publish_task(int target_id, const sb_base::msg::TaskCommand &task_cmd) {
        if (pub_task_cmds_.find(target_id) == pub_task_cmds_.end()) {
            RCLCPP_WARN(this->get_logger(), "Invalid target_id: %d", target_id);
            return;
        }
        pub_task_cmds_[target_id]->publish(task_cmd);
    }


private:
    //================================================================
    // Private Implementation: Internal details, not accessible by derived classes
    //================================================================

    // --- Private Methods ---
    void timer_callback();
    void box_stage_changes();
    void box_report_publisher();
    void sup_hb_callback(const sb_base::msg::Heartbeat::SharedPtr msg);

    Eigen::Vector3d position_discrepancy(Eigen::Vector3d& ned, Eigen::Vector3d& gps);

    // --- Private Member Variables ---
    rclcpp::TimerBase::SharedPtr timer_;
    std::string sup_ns, px4_ns;
    
    // Lost connection counters
    uint sup_lost_count;
    // std::vector<int> inf_lost_timer;

    // PX4 Messages
    px4_msgs::msg::OffboardControlMode  px4_offboard_hb_{};
    px4_msgs::msg::VehicleCommandAck    px4_vehicle_cmd_ack_{};

    // SwarmBox Messages
    sb_base::msg::Report    box_report_;
    sb_base::msg::Heartbeat box_heartbeat_;

    // --- ROS2 Publishers & Subscribers ---
    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr    px4_pub_offboard_;
    rclcpp::Publisher<sb_base::msg::Report>::SharedPtr                  box_pub_report_;
    rclcpp::Publisher<sb_base::msg::Heartbeat>::SharedPtr               box_pub_heartbeat_;
    
    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr       px4_sub_command_ack_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr   px4_sub_global_pos_;
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr               px4_sub_sensorgps_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr    px4_sub_local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr           px4_sub_vehiclestat_;
    rclcpp::Subscription<sb_base::msg::Heartbeat>::SharedPtr                box_sub_heartbeat_;
    rclcpp::Subscription<sb_base::msg::TaskCommand>::SharedPtr              box_sub_task_cmd_;
    rclcpp::Subscription<sb_base::msg::ProxAlert>::SharedPtr                box_sub_prox_alert_;
    rclcpp::Subscription<sb_base::msg::Report>::SharedPtr                   box_sub_reports_;

    rclcpp::Context::SharedPtr px4_context_;
    rclcpp::Node::SharedPtr px4_node_;
};
