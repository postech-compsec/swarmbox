#include "sb_base/drone.hpp"
#include <iostream>
#include <algorithm>

using namespace std::chrono_literals;
using namespace sb_base::msg;

//=============================================
// Constructor & Destructor
//=============================================
// make a no-argument constructor for Drone node: defaults: id=0, superior=-1, location={0.0f, 0.0f, 0.0f}

static float default_loc[3] = {0.0f, 0.0f, 0.0f};

Drone::Drone(const rclcpp::NodeOptions & options) : SBNode("drone", options) {
    this->declare_parameter<int>("id", 0);
    this->declare_parameter<int>("superior", -1);
    this->declare_parameter<std::vector<double>>("relative_pos", {default_loc[0], default_loc[1], default_loc[2]});
    
    this->get_parameter("id", this->identity);
    this->get_parameter("superior", this->sup_id);
    std::vector<double> loc = {0.0, 0.0, 0.0};
    this->get_parameter("relative_pos", loc);
    this->init_pos.x() = loc[0];
    this->init_pos.y() = loc[1];
    this->init_pos.z() = loc[2];

    // Initialize QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
    uint tick = 0;

    this->report_data = "";
    
    sup_ns = (this->sup_id == -1 ? "/gcs" : "/drone_" + std::to_string(sup_id));
    px4_ns = (this->identity == 0 ? "" : "/px4_"+std::to_string(this->identity));
    RCLCPP_INFO(this->get_logger(), "Drone node %d initialized, superior: %d", this->identity, this->sup_id);
    // std::cout << "Superior namespace: " << sup_ns << std::endl;
    
    // print out parameter parsing results
    RCLCPP_INFO_ONCE(this->get_logger(), "ID: %d, Superior: %d, Location(NED): (%.2f, %.2f, %.2f)", 
                this->identity, this->sup_id, this->init_pos.x(), this->init_pos.y(), this->init_pos.z());
    
    // Initialize states
    this->stage = STAGE_ICEB;
    this->complete = false;
    this->sup_lost_count = 0;
    this->z_bias = 0.0;
    
    // Initialize inferior list
    this->inf_list = {};
    this->inf_count = 0;
    this->inf_stage = {};
    this->inf_lost_timer = {};
    this->inf_reports = {};
    this->inf_completes = {};
    this->inf_complete_count = 0;
    this->inf_descs = {};
    this->init_gpos = Eigen::Vector3d(NAN, NAN, NAN); // Initialize to NaN

    // Initialize PX4 offboard heartbeat
    this->px4_offboard_hb_.position = true;
    this->px4_offboard_hb_.velocity = false;
    this->px4_offboard_hb_.acceleration = false;
    this->px4_offboard_hb_.attitude = false;
    this->px4_offboard_hb_.body_rate = false;
    this->px4_offboard_hb_.thrust_and_torque = false;
    this->px4_offboard_hb_.direct_actuator = false;

    // Initialize PX4 vehicle command
    px4_msgs::msg::VehicleCommand vehicle_cmd_template;
    vehicle_cmd_template.target_system = this->identity + 1;
    vehicle_cmd_template.target_component = 1;
    vehicle_cmd_template.source_system = 255; // MAV_SYS_ID_ALL
    vehicle_cmd_template.source_component = 1;
    vehicle_cmd_template.from_external = true;
    this->px4_vehicle_cmd_ = vehicle_cmd_template;

    // --- ROS2 Publisher/Subscriber Setup ---
    // PX4 Publishers
    px4_pub_offboard_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(this->px4_ns+"/fmu/in/offboard_control_mode", 10);
    px4_pub_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(this->px4_ns+"/fmu/in/vehicle_command", 10);
    px4_pub_traj_setpoint_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(this->px4_ns+"/fmu/in/trajectory_setpoint", 10);
    
    
    // PX4 Subscribers
    px4_sub_global_pos_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                            this->px4_ns+"/fmu/out/vehicle_global_position", qos, 
                            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg){ 
        this->px4_global_pos_ = *msg;
        // this->gps_updated = true;
    });
    px4_sub_sensorgps_ = this->create_subscription<px4_msgs::msg::SensorGps>(
                            this->px4_ns+"/fmu/out/sensor_gps", qos, 
                            [this](const px4_msgs::msg::SensorGps::SharedPtr msg){ 
        this->px4_sensorgps_ = *msg;
        // update only if it's in exec stage: to prevent bias caused from initializing error.
        if (this->stage >= STAGE_EXEC) {
            if (std::isnan(this->init_gpos.x()) || std::isnan(this->init_gpos.y()) || std::isnan(this->init_gpos.z())) {
                this->init_gpos = Eigen::Vector3d(
                msg->latitude_deg, 
                msg->longitude_deg, 
                msg->altitude_msl_m);
            } else {
                Eigen::Vector3d gps_(msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m);
                Eigen::Vector3d lps_ = this->world_pos - this->init_pos;
                this->discrepancy = this->position_discrepancy(lps_, gps_);
            }
            this->gps_updated = true;
        }
    });
    px4_sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(this->px4_ns+"/fmu/out/vehicle_local_position", qos, [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){ 
        this->px4_local_pos_ = *msg;
        this->world_pos = Eigen::Vector3d(
            this->px4_local_pos_.x + this->init_pos.x(), 
            this->px4_local_pos_.y + this->init_pos.y(), 
            this->px4_local_pos_.z + this->init_pos.z());
        this->velocity = Eigen::Vector3d(
            this->px4_local_pos_.vx, 
            this->px4_local_pos_.vy, 
            this->px4_local_pos_.vz);
    });
    px4_sub_command_ack_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(this->px4_ns+"/fmu/out/vehicle_command_ack", qos, [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg){ this->px4_vehicle_cmd_ack_ = *msg; });
    px4_sub_vehiclestat_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(this->px4_ns+"/fmu/out/vehicle_status", qos, [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg){
        // FAILSAFE ABORTION TURNED OFF FOR SITL TESTING
        if (msg->failsafe) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Failsafe activated.");
        //     this->stage = STAGE_ABRT;
        //     this->box_report_publisher();
        //     this->box_stage_changes();
        }
        this->armed = (msg->arming_state - 1);
    });
    px4_sub_land_detected_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(this->px4_ns+"/fmu/out/vehicle_land_detected", qos, [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(), "Ground contact: %d, Maybe landed: %d", msg->ground_contact, msg->maybe_landed);
        RCLCPP_INFO_ONCE(this->get_logger(), "PX4 timestamp: %ld", msg->timestamp);
        if (this->stage == STAGE_FINI && msg->landed) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Landed successfully.");
            this->complete = true;
            this->box_report_publisher();
            this->box_stage_changes();
        }
    });


    // SwarmBox Publishers
    box_pub_report_ = this->create_publisher<sb_base::msg::Report>(sup_ns+"/report", 10);
    box_pub_heartbeat_ = this->create_publisher<sb_base::msg::Heartbeat>("heartbeat", 10);

    // SwarmBox Subscribers
    box_sub_heartbeat_ = this->create_subscription<sb_base::msg::Heartbeat>(sup_ns+"/heartbeat", qos, std::bind(&Drone::sup_hb_callback, this, std::placeholders::_1));
    box_sub_task_cmd_ = this->create_subscription<sb_base::msg::TaskCommand>("task_cmd", qos, [this](const sb_base::msg::TaskCommand::SharedPtr msg){
        // calculate delay
        double delay = (int)((this->get_clock()->now().nanoseconds() / 1000) - msg->timestamp); // us
        if (delay > 200*1000) {
            RCLCPP_WARN(this->get_logger(), "Task command delay from [%d]: %.03f ms", msg->orig_id, delay / 1000);
        }

        this->box_rcvd_task_cmd_ = *msg; // Update my task command
    });
    box_sub_prox_alert_ = this->create_subscription<sb_base::msg::ProxAlert>("prox_alert", qos, [this](const sb_base::msg::ProxAlert::SharedPtr msg){
        this->box_prox_alert_ = *msg; // Update my proximity alert
        // for (size_t i = 0; i < msg->prox_ids.size(); i++) {
        //     RCLCPP_INFO(this->get_logger(), "Proximity alert: close to Drone %d at position (%.2f, %.2f, %.2f)", 
        //                 msg->prox_ids[i], msg->prox_data[i*3], msg->prox_data[i*3+1], msg->prox_data[i*3+2]);
        // }
    });
    box_sub_reports_ = this->create_subscription<sb_base::msg::Report>("report", qos, [this](const sb_base::msg::Report::SharedPtr msg){
        // auto it = std::find(inf_list.begin(), inf_list.end(), msg->orig_id);
        auto it = this->inf_reports.find(msg->orig_id);
        if (it == inf_reports.end()) {
            RCLCPP_INFO(this->get_logger(), "[inferior %d] New handshake received", msg->orig_id);
            this->inf_list.push_back(msg->orig_id);
            this->inf_lost_timer[msg->orig_id] = 0;
            this->inf_reports[msg->orig_id] = *msg;
            this->inf_stage[msg->orig_id] = msg->stage;
            this->inf_completes[msg->orig_id] = false;
            this->inf_descs[msg->orig_id] = msg->descendants;
            this->inf_count++;
            this->pub_task_cmds_[msg->orig_id] = this->create_publisher<sb_base::msg::TaskCommand>("/drone_"+std::to_string(msg->orig_id)+"/task_cmd", 10);
            // RCLCPP_INFO(this->get_logger(), "[inferior %d] Descendants: ", msg->orig_id);
            // for (const auto& desc : msg->descendants) {
            //     RCLCPP_INFO(this->get_logger(), "%d ", desc);
            // }

        } else {
            // int index = std::distance(inf_list.begin(), it);
            this->inf_lost_timer[msg->orig_id] = 0;
            this->inf_reports[msg->orig_id] = *msg;
            this->inf_stage[msg->orig_id] = msg->stage;
            this->inf_completes[msg->orig_id] = msg->complete;
            this->inf_descs[msg->orig_id] = msg->descendants; // Update descendants
        }
    });
        
    // Main timer
    timer_ = this->create_wall_timer(25ms, std::bind(&Drone::timer_callback, this));
}


Drone::~Drone() {
    RCLCPP_WARN(this->get_logger(), "Drone node terminated.");
}

bool Drone::hasReached(setpoint target_setpoint, const float d_threshold, const float v_threshold) {
    Eigen::Vector3d target_pos(target_setpoint.north, target_setpoint.east, target_setpoint.down);
    // Eigen::Vector3d current_pos(world_pos.x(), world_pos.y(), world_pos.z());
    // Eigen::Vector3d current_vel(px4_local_pos_.vx, px4_local_pos_.vy, px4_local_pos_.vz);

    float distance = (target_pos - world_pos).norm();
    float speed    = velocity.norm();

    // check boundary condition
    return (distance <= d_threshold && speed <= v_threshold);
}

void Drone::sup_hb_callback(const sb_base::msg::Heartbeat::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Heartbeat received from superior %d OK", msg->identity);
    this->emergency_contacts = msg->successors; // update emergency contacts
    if (!this->emergency_contacts.empty()) RCLCPP_DEBUG_ONCE(this->get_logger(), "Emergency contact #1: %d", this->emergency_contacts[0]);
    this->sup_lost_count = 0;
    this->success_no = -1; // we've found our superior, so we're not using successors.
    if (msg->stage != this->stage) {
        RCLCPP_INFO(this->get_logger(), "[superior %d] Stage changed from %d to %d", this->sup_id, this->stage, msg->stage);
        this->stage = msg->stage;
        this->complete = false;
        this->box_stage_changes();
    }
}

//=============================================
// Superior lost handling
//=============================================

bool Drone::update_superior() {
    // will be called when superior needs to be updated
    // first let's check if who was the last superior in superior's last heartbeat->successors
    if (this->sup_id == -1) {
        // my superior is ground, so unfortunately I am left alone: superior update failed.
        RCLCPP_ERROR(this->get_logger(), "Ground connection lost.");
        return false; // return true if succeeded, false if failed
    } else {
        // try connecting with last heartbeat->successors[success_no+1];
        this->success_no++;
        if (this->success_no >= this->emergency_contacts.size()) {
            // no more successors to try
            RCLCPP_ERROR(this->get_logger(), "No more successors to try. Superior update failed.");
            return false; // return true if succeeded, false if failed
        }
        int new_superior = this->emergency_contacts[this->success_no];
        // update the superior namespace
        this->sup_id = new_superior;
        this->sup_ns = (this->sup_id == -1 ? "/gcs" : "/drone_" + std::to_string(sup_id));
        RCLCPP_INFO(this->get_logger(), "Superior updated to %d", this->sup_id);
        // change sub/pubs
        this->box_sub_heartbeat_ = this->create_subscription<sb_base::msg::Heartbeat>(this->sup_ns+"/heartbeat", 10, std::bind(&Drone::sup_hb_callback, this, std::placeholders::_1));
        this->box_pub_report_ = this->create_publisher<sb_base::msg::Report>(this->sup_ns+"/report", 10);
        return true;
    }
}

// Tells how to assign successors.
std::vector<int16_t> Drone::assign_successors() { 
    // default behavior is to assign my superior as my only successor.
    std::vector<int16_t> successor_list = {static_cast<int16_t>(this->sup_id)};
    // RCLCPP_WARN_ONCE(this->get_logger(), "There's no successor rule implemented: %d has been assigned. Make sure you implement your overriding [std::vector<int> assign_successors()] function", this->sup_id);
    return successor_list;
}


//=============================================
// Stage-specific Virtual Functions
//=============================================
// This function is called once at the beginning of the preparation stage.
void Drone::prep_once() { 
    RCLCPP_WARN_ONCE(this->get_logger(), "Make sure you implement your overriding [void prep_once()] function");
}

// This function is called repeatedly during the preparation stage.
void Drone::prep_loop() { 
    RCLCPP_WARN_ONCE(this->get_logger(), "Make sure you implement your overriding [void prep_loop()] function");
}

bool Drone::prep_complete() {
    // Default behavior is to return true, indicating preparation is complete.
    // This can be overridden by derived classes to implement custom logic.
    RCLCPP_WARN_ONCE(this->get_logger(), "Make sure you implement your overriding [bool prep_complete()] function");
    if (this->armed){
        return true; 
    } else {
        return false;
    }
}

// This function is called once at the beginning of the execution stage.
void Drone::exec_once() { 
    RCLCPP_WARN_ONCE(this->get_logger(), "Make sure you implement your overriding [void exec_once()] function");
}

// This function is called repeatedly during the execution stage.
std::optional<setpoint> Drone::exec_loop(){
    RCLCPP_WARN_ONCE(this->get_logger(), "Make sure you implement your overriding [std::optional<setpoint> exec_loop()] function");
    setpoint sp;
    sp.north = 0.0f;
    sp.east = 0.0f;
    sp.down = -10.0f; // default takeoff height
    sp.yaw = 0.0f; // default yaw
    return sp;
}

bool Drone::exec_complete() {
    // Default behavior is to return true, indicating execution is complete.
    // This can be overridden by derived classes to implement custom logic.
    RCLCPP_WARN_ONCE(this->get_logger(), "Make sure you implement your overriding [bool exec_complete()] function");
    if (this->world_pos.z() < -9.5 + this->z_bias) {
        RCLCPP_INFO(this->get_logger(), "Takeoff completed.");
        return true;
    }
    return false;
}


//=============================================
// Private Methods
//=============================================

void Drone::timer_callback() {
    this->tick++;
    // Superior connection check
    // RCLCPP_DEBUG(this->get_logger(), "Timeout watchdog: lost %d stage %d", this->sup_lost_count, this->stage);
    if (this->sup_lost_count > 40 && this->stage >= STAGE_EXEC) {
        RCLCPP_WARN(this->get_logger(), "Superior %d connection lost.", this->sup_id);
        if (!this->update_superior()) {
            // If superior update failed, abort the mission.
            RCLCPP_ERROR(this->get_logger(), "Failed to update superior. Aborting mission.");
            this->stage = STAGE_ABRT;
            this->box_stage_changes();
            return;
        }
        return;
    }
    this->sup_lost_count++;

    // Inferior computations
    inf_alive = 0;
    inf_complete_count = 0;
    this->total_count = 0;
    this->total_alive = 0;
    for (int i : this->inf_list) {
        if (this->inf_stage[i] != STAGE_ABRT) {
            inf_alive++;
            this->total_alive += this->inf_reports[i].total_alive;
            this->total_count += this->inf_reports[i].total_count;
        }
        if (this->inf_completes[i]) {
            inf_complete_count++;
        }
        if (this->inf_lost_timer[i] > LOST_THRESH && this->inf_stage[i] != STAGE_ABRT) {
            RCLCPP_WARN(this->get_logger(), "Inferior %d connection lost.", i);
            this->inf_stage[i] = STAGE_ABRT;
        } else {
            this->inf_lost_timer[i]++;
        }
    }
    
    this->successors = this->assign_successors();
    
    // Publish heartbeat to myself
    this->box_heartbeat_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->box_heartbeat_.identity = this->identity;
    this->box_heartbeat_.stage = this->stage;
    this->box_heartbeat_.successors = this->successors;
    box_pub_heartbeat_->publish(this->box_heartbeat_);
    
    // Publish report to superior
    this->box_report_publisher();

    // Publish PX4 offboard heartbeat
    this->px4_offboard_hb_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    if (this->stage < STAGE_FINI) {
        this->px4_pub_offboard_->publish(this->px4_offboard_hb_);
    }
    

    // Run stage-specific tasks
    if (this->stage == STAGE_ICEB) {
        this->complete = true;
    } else if (this->stage == STAGE_PREP) {
        px4_vehicle_cmd_.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        px4_vehicle_cmd_.param1 = 1.0;
        px4_vehicle_cmd_.param2 = 6.0; // Offboard mode
        px4_vehicle_cmd_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        this->px4_pub_command_->publish(px4_vehicle_cmd_);
        // if (this->complete == true) {
        // before waiting for user confirmation, send arm command ONLY IF xy_valid and z_valid
        if (this->px4_local_pos_.xy_valid && this->px4_local_pos_.z_valid && (this->tick % 20 == 0)) {
            px4_vehicle_cmd_.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            px4_vehicle_cmd_.param1 = 1.0; // arm
            px4_vehicle_cmd_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            this->px4_pub_command_->publish(px4_vehicle_cmd_);
        }
        // }
        this->successors = this->assign_successors();
        this->prep_loop(); // user-defined preparation loop
        this->complete = this->prep_complete(); // user-defined preparation completion check
    } else if (this->stage == STAGE_EXEC) {
        // error handling: if the stage has been approved but the drone is not armed, send arming command
        if (!this->armed) {
            px4_vehicle_cmd_.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            px4_vehicle_cmd_.param1 = 1.0; // arm
            px4_vehicle_cmd_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            this->px4_pub_command_->publish(px4_vehicle_cmd_);
        } else {
            px4_msgs::msg::TrajectorySetpoint traj;
            traj.position = {NAN, NAN, NAN};
            traj.velocity = {NAN, NAN, NAN};
            traj.acceleration = {NAN, NAN, NAN};
            traj.jerk = {NAN, NAN, NAN};
            traj.yaw = NAN;
            traj.yawspeed = NAN;
            traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            std::optional<setpoint> traj_setpoint = this->exec_loop(); // user-defined execution loop
            // process optional
            if (traj_setpoint.has_value()) {
                setpoint rcvd = traj_setpoint.value();
                // feature: now supports pos, vel, and acc
                if (rcvd.type == 0) {
                    // // faulty value check
                    // if (abs(rcvd.north - this->world_pos.x()) > 1000.0f || 
                    //     abs(rcvd.east - this->world_pos.y()) > 1000.0f || 
                    //     abs(rcvd.down - this->world_pos.z()) > 1000.0f) {
                    //     RCLCPP_WARN(this->get_logger(), "Received setpoint that might be faulty: (%.2f, %.2f, %.2f)", 
                    //                 rcvd.north, rcvd.east, rcvd.down);
                    //     return; // skip publishing faulty setpoint
                    // }
                    /* Process to drone's bodyframe */
                    rcvd.north -= this->init_pos.x();
                    rcvd.east  -= this->init_pos.y();
                    rcvd.down  -= this->init_pos.z();

                    traj.position[0] = rcvd.north;
                    traj.position[1] = rcvd.east;
                    traj.position[2] = rcvd.down + this->z_bias;
                } else if (rcvd.type == 1) {
                    float velocity_norm = sqrt(pow(rcvd.north, 2) + pow(rcvd.east, 2) + pow(rcvd.down, 2));
                    if (velocity_norm > 20.0) {
                        RCLCPP_WARN(this->get_logger(), "Received velocity setpoint that might be faulty: (%.2f, %.2f, %.2f)", 
                                    rcvd.north, rcvd.east, rcvd.down);
                    }
                    traj.velocity[0] = rcvd.north;
                    traj.velocity[1] = rcvd.east;
                    traj.velocity[2] = rcvd.down;
                } else {
                    // acceleration setpoint. skip faulty value check for now
                    traj.acceleration[0] = rcvd.north;
                    traj.acceleration[1] = rcvd.east;
                    traj.acceleration[2] = rcvd.down;
                }
                traj.yaw = rcvd.yaw;
                traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                // this->last_valid_traj_ = traj;
                this->px4_pub_traj_setpoint_->publish(traj);
            } else {
                // 
                // this->last_valid_traj_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                // this->px4_pub_traj_setpoint_->publish(this->last_valid_traj_);
            }
        }
        this->complete = this->exec_complete(); // user-defined execution completion check
        if (this->complete) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Execution completed!");
        }
    } else if (this->stage == STAGE_FINI) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Landing stage: waiting for landing completion.");
    } else if (this->stage == STAGE_TERM) {
        rclcpp::shutdown();
    }
    
    if (this->complete && inf_complete_count == this->inf_count && this->inf_count > 0) {
        // this->complete = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Stage %d completed for me and my %d inferiors", this->stage, this->inf_count);
    }
}

void Drone::box_stage_changes() {
    px4_msgs::msg::VehicleCommand msg;
    px4_msgs::msg::TrajectorySetpoint traj;
    switch (this->stage) {
        case STAGE_ABRT:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to ABORTION.");
            RCLCPP_ERROR(this->get_logger(), "Stage Update: Mission Aborted. Shutting down.");
            rclcpp::shutdown();
            break;
        case STAGE_IDLE:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to IDLE.");
            break;
        case STAGE_ICEB:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to ICEBREAK.");
            break;
        case STAGE_PREP:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to PREPARATION.");
            // RCLCPP_INFO(this->get_logger(), "timestamp from ros: %ld", this->get_clock()->now().nanoseconds() / 1000);
            for (int i = 0; i < inf_list.size(); i++) {
                this->inf_mapper[inf_list[i]] = i; // inverse mapping
            }
            this->prep_once();
            break;
        case STAGE_EXEC:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to MISSION EXECUTION.");
            px4_vehicle_cmd_.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            px4_vehicle_cmd_.param1 = 1.0; // arm
            px4_vehicle_cmd_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            this->px4_pub_command_->publish(px4_vehicle_cmd_);
            RCLCPP_INFO_ONCE(this->get_logger(), "Arming vehicle...");
            // this->complete = false;
            this->tick = 0; // reset tick for execution stage
            this->exec_once();
            break;
        case STAGE_FINI:
            RCLCPP_INFO_ONCE(this->get_logger(), "Stage Update: set to LANDING (completion).");
            msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
            msg.param4 = 0.0; // reset to zero yaw attitude
            msg.target_system = this->identity + 1;
            msg.target_component = 1;
            msg.source_system = this->identity + 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            this->px4_pub_command_->publish(msg);
            RCLCPP_INFO_ONCE(this->get_logger(), "Publishing vehicle land...");
            break;
        case STAGE_TERM:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to TERMINATION.");
            rclcpp::shutdown();
            break;
        default:
            break;
    }
}

void Drone::box_report_publisher() {
    this->box_report_.orig_id = this->identity;
    this->box_report_.stage = this->stage;
    this->box_report_.complete = (this->inf_complete_count == this->inf_alive && this->complete);
    this->box_report_.total_count = this->total_count + 1;
    this->box_report_.total_alive = this->total_alive + ((this->stage == STAGE_ABRT) ? 0 : 1);
    // marker("Publishing box report: total [%d / %d]", this->box_report_.total_count, this->box_report_.total_alive);
    this->box_report_.data = this->report_data; // add report data if any
    // NOTE: currently, the centroid position value is set to single drone's position.
    // we should consider the relative positions of the drones
    this->box_report_.pos_x = this->world_pos.x(); 
    this->box_report_.pos_y = this->world_pos.y();
    this->box_report_.pos_z = this->world_pos.z();
    // this->box_report_.gps_lat = this->px4_sensorgps_.latitude_deg;
    // this->box_report_.gps_lon = this->px4_sensorgps_.longitude_deg;
    // this->box_report_.gps_alt = this->px4_sensorgps_.altitude_msl_m;
    if (this->gps_updated) {
        // Eigen::Vector3d discrep = this->position_discrepancy(this->world_pos, gps_);
        this->box_report_.discrep_x = this->discrepancy.x();
        this->box_report_.discrep_y = this->discrepancy.y();
        this->box_report_.discrep_z = this->discrepancy.z();
        this->gps_updated = false;
    } else {
        this->box_report_.discrep_x = NAN;
        this->box_report_.discrep_y = NAN;
        this->box_report_.discrep_z = NAN;
    }
    this->box_report_.vel_x = this->velocity.x();
    this->box_report_.vel_y = this->velocity.y();
    this->box_report_.vel_z = this->velocity.z();
    this->box_report_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->box_pub_report_->publish(this->box_report_);
}

Eigen::Vector3d Drone::position_discrepancy(Eigen::Vector3d& ned, Eigen::Vector3d& gps) {
    // RCLCPP_INFO(this->get_logger(), "NED: (%.2f, %.2f, %.2f), GPS: (%.2f, %.2f, %.2f), init: (%.2f, %.2f, %.2f)",
    //     ned.x(), ned.y(), ned.z(),
    //     gps.x(), gps.y(), gps.z(),
    //     this->init_gpos.x(), this->init_gpos.y(), this->init_gpos.z());
    Eigen::Vector3d gps_delta = gps - this->init_gpos;
    double north = gps_delta.x() * METER_PER_DEG_LAT;
    
    double ref_lat_rad = (this->init_gpos.x() * M_PI) / 180.0; // Convert latitude to radians
    double meters_per_deg_lon = (M_PI / 180.0) * EARTH_RADIUS * cos(ref_lat_rad);
    double east = gps_delta.y() * meters_per_deg_lon;

    double down = -gps_delta.z(); // down is negative in NED
    Eigen::Vector3d discrepancy = Eigen::Vector3d(north, east, down) - ned;

    return discrepancy;
}