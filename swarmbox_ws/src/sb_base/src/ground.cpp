#include "sb_base/ground.hpp"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <cstdlib>
#include <string>
// #include <sstream>
#include <filesystem>

using namespace std::chrono_literals;
using namespace sb_base::msg;

//=============================================
// Constructor & Destructor
//=============================================

Ground::Ground(const rclcpp::NodeOptions& options) : SBNode("ground", options) {
    // Initialize QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
    
    // read configuration file
    this->declare_parameter<std::string>("config_file", "");
    std::string config_path = this->get_parameter("config_file").as_string();


    if (config_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Configuration file path is not set. Please set the 'config_file' parameter.");
        return;
    }

    parse_config_file(config_path);
    

    // Initialize variables
    // this->total_swarm_size = swarm_size;
    this->stage = STAGE_ICEB;
    this->tick = 0;
    this->conn_total = 0;
    this->conn_alive = 0;

    // Initialize direct inferiors
    this->inf_list = {};
    this->inf_count = 0;
    this->inf_stage = {};
    this->inf_lost_timer = {};
    this->inf_reports = {};
    this->inf_completes = {};
    this->inf_complete_count = 0;
    this->inf_pos = {};
    this->inf_descs = {};
    this->inf_pos_dis = {};
    this->inf_delays = {};

    // setup logger
    // std::stringstream ss;
    // ss << "sb_log/ground" << (this->get_clock()->now().nanoseconds() / 1000000000) << ".csv";

    // log_file_.open(ss.str(), std::ios::out | std::ios::trunc);
    // if (log_file_.is_open()) {
    //     log_file_ << "ts_sent,ts_received,log_type,drone_id,val1,val2,val3\n";
    //     RCLCPP_INFO(this->get_logger(), "Log file created: %s", ss.str().c_str());
    // } else {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to open log file!");
    // }

    std::string ws_root = "."; // Fallback
    const char* ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");

    if (ament_prefix_path != nullptr) {
        std::string path_str(ament_prefix_path);
        std::stringstream path_stream(path_str);
        std::string token;
        
        while (std::getline(path_stream, token, ':')) {
            size_t pos = token.find("swarmbox_ws");
            if (pos != std::string::npos) {
                ws_root = token.substr(0, pos + std::string("swarmbox_ws").length());
                break;
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "AMENT_PREFIX_PATH not found. Using current directory.");
    }

    std::filesystem::path log_dir = std::filesystem::path(ws_root) / "sb_log";

    if (!std::filesystem::exists(log_dir)) {
        std::filesystem::create_directories(log_dir);
    }

    std::stringstream ss;
    ss << (log_dir / "ground").string() 
    << (this->get_clock()->now().nanoseconds() / 1000000000) 
    << ".csv";

    log_file_.open(ss.str(), std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
        log_file_ << "ts_sent,ts_received,log_type,drone_id,val1,val2,val3\n";
        RCLCPP_INFO(this->get_logger(), "Log file created safely at: %s", ss.str().c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open log file at %s!", ss.str().c_str());
    }


    // Create publishers
    pub_heartbeat_ = this->create_publisher<sb_base::msg::Heartbeat>("heartbeat", 10);

    // Create subscribers
    sub_reports_ = this->create_subscription<sb_base::msg::Report>("report", qos, [this](const sb_base::msg::Report::SharedPtr msg) {
        // calculate delay
        double delay = ((this->get_clock()->now().nanoseconds() / 1000) - msg->timestamp); // us
        // RCLCPP_INFO(this->get_logger(), "Report delay from [%d]: %.03f ms", msg->orig_id, delay / 1000);

        auto it = inf_reports.find(msg->orig_id);
        Eigen::Vector3d pos = {msg->pos_x, msg->pos_y, msg->pos_z};
        if (it == inf_reports.end()) {
            RCLCPP_INFO(this->get_logger(), "[inferior %d] New handshake received", msg->orig_id);
            this->inf_list.push_back(msg->orig_id);
            this->inf_lost_timer[msg->orig_id] = 0;
            this->inf_reports[msg->orig_id] = *msg;
            this->inf_stage[msg->orig_id] = msg->stage;
            this->inf_completes[msg->orig_id] = false;
            this->inf_descs[msg->orig_id] = msg->descendants;
            this->inf_pos[msg->orig_id] = pos; // Initialize position to (0, 0, 0)
            this->inf_delays[msg->orig_id] = delay;

            this->inf_packets_received[msg->orig_id] = 0;
            this->inf_loss_ratio_[msg->orig_id] = 0.0;

            this->inf_pos_dis[msg->orig_id] = Eigen::Vector3d(0.0, 0.0, 0.0); // Initialize position discrepancy
            this->inf_count++;
            this->pub_task_cmds_[msg->orig_id]      = this->create_publisher<sb_base::msg::TaskCommand>("/drone_"+std::to_string(msg->orig_id)+"/task_cmd", 10);
            this->pub_prox_alerts_[msg->orig_id]    = this->create_publisher<sb_base::msg::ProxAlert>("/drone_"+std::to_string(msg->orig_id)+"/prox_alert", 10);
        } else {
            // int index = it->first;
            this->inf_lost_timer[msg->orig_id] = 0;
            this->inf_reports[msg->orig_id] = *msg;
            this->inf_stage[msg->orig_id] = msg->stage;
            this->inf_completes[msg->orig_id] = msg->complete;
            this->inf_pos[msg->orig_id] = pos; // Update position
            this->inf_descs[msg->orig_id] = msg->descendants; // Update descendants
            this->inf_delays[msg->orig_id] = delay;
            this->inf_packets_received[msg->orig_id]++;
        }

        if (!std::isnan(msg->discrep_x) && !std::isnan(msg->discrep_y) && !std::isnan(msg->discrep_z)) {
            // RCLCPP_INFO(this->get_logger(), "Log: %d, %.4f, %.4f, %.4f", (int)msg->orig_id, msg->discrep_x, msg->discrep_y, msg->discrep_z);
            this->inf_pos_dis[msg->orig_id] = Eigen::Vector3d(msg->discrep_x, msg->discrep_y, msg->discrep_z);
            uint64_t ts_rcvd = this->get_clock()->now().nanoseconds() / 1000;
            log_file_ << std::fixed << std::setprecision(4)
                        << msg->timestamp << ","
                        << ts_rcvd << "," 
                        << "Discrepancy,"
                        << msg->orig_id << ","
                        << msg->discrep_x << ","
                        << msg->discrep_y << ","
                        << msg->discrep_z << "\n";
        }
        // this->report_callback(msg);
    });
    
    // Main timer callback
    timer_ = this->create_wall_timer(25ms, std::bind(&Ground::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Ground node initialized.");
}

Ground::~Ground(){
    if (log_file_.is_open()) {
        log_file_.close();
    }
    RCLCPP_INFO(this->get_logger(), "Ground node terminated.");
}

void Ground::parse_config_file(const std::string& config_path) {
    RCLCPP_DEBUG(this->get_logger(), "Parsing configuration file: %s", config_path.c_str());

    std::ifstream f(config_path.c_str());
    if (!f.good()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open configuration file: %s", config_path.c_str());
        return;
    }

    try {
        YAML::Node root = YAML::LoadFile(config_path);
        const YAML::Node& mission_config = root["mission_config"];
        if (mission_config) {
            this->mission_type_ = mission_config["type"].as<std::string>();

            this->total_swarm_size = mission_config["swarm_size"].as<int>();

            const YAML::Node& mission_values = mission_config["values"];
            for (const auto& value : mission_values) {
                int id = value.first.as<int>();
                std::vector<double> vals = value.second.as<std::vector<double>>();
                this->values_[id] = vals;
                RCLCPP_DEBUG(this->get_logger(), "values %d: (%.2f, %.2f)", 
                            id, vals[0], vals[1]);
            }
        }

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse configuration file: %s", e.what());
    }

}

//=============================================
// Stage-specific Virtual Functions
//=============================================
void Ground::prep_once() {
    RCLCPP_INFO_ONCE(this->get_logger(), "Preparation stage started.");
}

void Ground::prep_loop() {
    // Default behavior is to do nothing. Can be overridden.
    // (void)tick; // Unused parameter
}

void Ground::exec_once() {
    RCLCPP_INFO_ONCE(this->get_logger(), "Execution stage started.");
}

void Ground::exec_loop() {
    RCLCPP_INFO_ONCE(this->get_logger(), "Execution stage loop called.");
}

//=============================================
// Private Methods
//=============================================

void Ground::prox_detector(int dist, int timewindow) {
    // proximity alert function. detects possible proximity issues within dist meters in timewindow.
    // for now, use distance only.
    proximity_alerts.clear();
    // proximity_alerts.resize(this->total_swarm_size);
    for (int i : this->inf_list) {
        for (int j : this->inf_list) {
            if (i <= j) continue;
            float dx = this->inf_pos[i].x() - this->inf_pos[j].x();
            float dy = this->inf_pos[i].y() - this->inf_pos[j].y();
            float dz = this->inf_pos[i].z() - this->inf_pos[j].z();
            float distance = sqrt(dx * dx + dy * dy + dz * dz);
            if (distance < dist) {
                // RCLCPP_WARN(this->get_logger(), "Proximity alert: Drone %d and Drone %d are within %d meters.", 
                //             i, j, dist);
                // add to proximity alerts
                proximity_alerts[i].push_back(j);
                proximity_alerts[j].push_back(i);
            }
        }
    }
    // Publish proximity alerts
    for (int i : this->inf_list) {
        if (!proximity_alerts[i].empty()) {
            sb_base::msg::ProxAlert alert;
            alert.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            alert.orig_id = -1; // Ground node does not have an ID
            alert.dest_id = i;
            alert.prox_ids = {};
            alert.prox_data = {}; // Fill with relevant proximity data
            for (int prox_id : proximity_alerts[i]) {
                alert.prox_ids.push_back(prox_id);
                Eigen::Vector3d pos = this->inf_pos[prox_id];
                alert.prox_data.push_back(pos.x());
                alert.prox_data.push_back(pos.y());
                alert.prox_data.push_back(pos.z());
            }
            this->pub_prox_alerts_[i]->publish(alert);
            // RCLCPP_INFO(this->get_logger(), "Proximity alert published for Drone %d", this->inf_list[i]);
        }
    }
}

void Ground::timer_callback() {
    this->tick++;

    // Publish ground heartbeat
    sb_base::msg::Heartbeat msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.identity = -1;
    msg.stage = this->stage;
    msg.successors = {}; // The ground does not have successors
    pub_heartbeat_->publish(msg);

    // check proximity
    this->prox_detector(2, 5); // for test.

    //> Network Latency Outlier detection
    if (!this->inf_delays.empty()) {
        std::vector<double> delay_vals = {};
        for (const auto& [id, delay] : this->inf_delays) {
            delay_vals.push_back(delay);
        }
        // get optional value
        auto outlier = this->iqr_outlier_detector(delay_vals, 1000.0);
        if (!outlier.has_value()) {
            RCLCPP_INFO_ONCE(this->get_logger(), "No outlier detected.");
        } else {
            // find who is outlier
            for (const auto& [id, delay] : this->inf_delays) {
                if (delay == std::get<0>(*outlier)) {
                    RCLCPP_WARN_ONCE(this->get_logger(), "Outlier detected: Drone %d with Network Latency %.03f ms (score: %.03f)", id, delay / 1000, std::get<1>(*outlier));
                    log_file_ << std::fixed << std::setprecision(4)
                        << this->get_clock()->now().nanoseconds() / 1000 << ","
                        << 0 << "," 
                        << "Latency,"
                        << -1 << ","
                        << id << ","
                        << delay / 1000 << ","
                        << std::get<1>(*outlier) << "\n";
                    break;
                }
            }
            // RCLCPP_WARN(this->get_logger(), "Outlier detected: Network Latency %.03f ms (score: %.03f)", std::get<0>(*outlier) / 1000, std::get<1>(*outlier));
        }
    }

    //> Message Loss Calculation & Outlier detection
    if (this->tick > 0 && this->tick % this->loss_window_ticks == 0) {
        // update loss ratio
        for (const auto& [id, packets] : this->inf_packets_received) {
            int expected_packets = this->loss_window_ticks; // since timer_callback is called every 25ms
            int received_packets = packets;
            if (expected_packets > 0) {
                this->inf_loss_ratio_[id] = 1.0 - (static_cast<double>(received_packets) / expected_packets);
            } else {
                this->inf_loss_ratio_[id] = 0.0;
            }
            this->inf_packets_received[id] = 0; // reset counter
        }
    }
    if (!this->inf_loss_ratio_.empty()) {
        std::vector<double> loss_vals = {};
        for (const auto& [id, loss] : this->inf_loss_ratio_) {
            loss_vals.push_back(loss);
        }

        auto outlier_loss = this->iqr_outlier_detector(loss_vals, 0.05);
        if (outlier_loss.has_value()) {
            // find who is outlier
            for (const auto& [id, loss] : this->inf_loss_ratio_) {
                if (loss == std::get<0>(*outlier_loss)) {
                    RCLCPP_WARN_ONCE(this->get_logger(), "Outlier detected: Drone %d with Packet Loss %.03f (score: %.03f)", id, loss * 100, std::get<1>(*outlier_loss));
                    log_file_ << std::fixed << std::setprecision(4)
                        << this->get_clock()->now().nanoseconds() / 1000 << ","
                        << 0 << ","
                        << "Loss,"
                        << -1 << ","
                        << id << ","
                        << loss * 100 << ","
                        << std::get<1>(*outlier_loss) << "\n";
                    break;
                }
            }
        }
    }


    //> Discrepancy Outlier detection
    if (!this->inf_pos_dis.empty()) {
        std::vector<double> dis_vals_x = {};
        std::vector<double> dis_vals_y = {};
        std::vector<double> dis_vals_z = {};
        for (const auto& [id, dis] : this->inf_pos_dis) {
            dis_vals_x.push_back(dis.x());
            dis_vals_y.push_back(dis.y());
            dis_vals_z.push_back(dis.z());
        }
        auto outlier_x = this->iqr_outlier_detector(dis_vals_x, 1.5);
        auto outlier_y = this->iqr_outlier_detector(dis_vals_y, 1.5);
        // auto outlier_z = this->iqr_outlier_detector(dis_vals_z, 1.5);
        if (outlier_x.has_value()) {
            // find who is outlier
            for (const auto& [id, dis] : this->inf_pos_dis) {
                if (dis.x() == std::get<0>(*outlier_x)) {
                    RCLCPP_WARN_ONCE(this->get_logger(), "Outlier detected: #%d Discrepancy (N) %.03f m (score: %.03f)", id, std::get<0>(*outlier_x), std::get<1>(*outlier_x));
                    log_file_ << std::fixed << std::setprecision(4)
                        << this->get_clock()->now().nanoseconds() / 1000 << ","
                        << 0 << "," 
                        << "Discrepancy_N,"
                        << -1 << ","
                        << id << ","
                        << std::get<0>(*outlier_x) << ","
                        << std::get<1>(*outlier_x) << "\n";
                    break;
                }
            }
        }
        if (outlier_y.has_value()) {
            for (const auto& [id, dis] : this->inf_pos_dis) {
                if (dis.y() == std::get<0>(*outlier_y)) {
                    RCLCPP_WARN_ONCE(this->get_logger(), "Outlier detected: #%d Discrepancy (E) %.03f m (score: %.03f)", id, std::get<0>(*outlier_y), std::get<1>(*outlier_y));
                    log_file_ << std::fixed << std::setprecision(4)
                        << this->get_clock()->now().nanoseconds() / 1000 << ","
                        << 0 << "," 
                        << "Discrepancy_E,"
                        << -1 << ","
                        << id << ","
                        << std::get<0>(*outlier_y) << ","
                        << std::get<1>(*outlier_y) << "\n";
                    break;
                }
            }
        }
        // if (outlier_z.has_value()) {
        //     RCLCPP_WARN(this->get_logger(), "Outlier detected: Position Discrepancy (D) %.03f m (score: %.03f)", std::get<0>(*outlier_z), std::get<1>(*outlier_z));
        // }
    }



    // Stage-specific loop functions
    if (this->stage == STAGE_PREP) {
        this->prep_loop();
    } else if (this->stage == STAGE_EXEC) {
        // if (this->tick == 1) this->exec_once(); // Call once on first tick of exec stage
        this->exec_loop();
    }

    // inferior computations
    inf_alive = 0;
    inf_complete_count = 0;
    int total_alive = 0;
    int total_count = 0;
    for (int i : this->inf_list) {
        if (this->inf_stage[i] != STAGE_ABRT) {
            inf_alive++;
        }
        if (this->inf_completes[i] && this->inf_stage[i] == this->stage) {
            // difference with drone: check if the inferior's stage completion report matches the current stage
            inf_complete_count++;
        }
        if (this->inf_lost_timer[i] > LOST_THRESH && this->inf_stage[i] != STAGE_ABRT) {
            RCLCPP_WARN(this->get_logger(), "Inferior %d connection lost.", i);
            this->inf_stage[i] = STAGE_ABRT;

        } else {
            this->inf_lost_timer[i]++;
        }
        // Count total connections
        if (this->inf_stage[i] != STAGE_ABRT) {
            // add inferior's alive count to total alive count
            total_alive += this->inf_reports[i].total_alive;
            // add inferior's total count to total count
            total_count += this->inf_reports[i].total_count;

            // RCLCPP_DEBUG(this->get_logger(), "Inferior %d: alive %d, total %d", i, this->inf_reports[i].alive, this->inf_reports[i].total_count);
        }
    }

    if (total_alive != this->conn_alive || total_count != this->conn_total) {
        RCLCPP_INFO(this->get_logger(), "Report Updated: [%d alive out of %d total]", total_alive, total_count);
    }
    if (this->stage == STAGE_TERM) {
        // if inf_count gets 0: (all inferiors have completed and terminated their nodes)
        if (total_count == 0) {
            RCLCPP_INFO(this->get_logger(), "All inferiors have completed.");
            // this->stage = STAGE_TERM;
            // this->stage_changes();
            rclcpp::shutdown();
        }
    }

    this->conn_total = total_count;
    this->conn_alive = total_alive;

    // check if complete
    if (this->inf_complete_count == this->inf_count && this->inf_count > 0) {
        // this->complete = true;
        // state change happens here.
        if (this->stage == STAGE_ICEB){
            if (total_count == this->total_swarm_size) {
                RCLCPP_INFO(this->get_logger(), "All drones are connected. Proceeding to PREPARATION stage.");
                this->stage = STAGE_PREP;
                this->stage_changes();
            }
        } else if (this->stage == STAGE_PREP) {
            RCLCPP_INFO(this->get_logger(), ">> All drones are ready.");
            this->stage = STAGE_EXEC;
            this->stage_changes();
        } else if (this->stage > STAGE_PREP) {
            RCLCPP_INFO(this->get_logger(), "Stage %d completed for the whole swarm!: %d, %d", this->stage, this->inf_complete_count, this->inf_count);
            this->stage += 1; // Move to next stage
            this->stage_changes();
        }
    }
}

std::optional<std::tuple<double, double>> Ground::iqr_outlier_detector(std::vector<double> data, double threshold) {
    std::vector<double> vals = data;
    std::sort(vals.begin(), vals.end());
    double q1, q2, q3;

    if (vals.size() % 2 == 1) {
        // if vals.size() is odd
        q2 = vals[vals.size() / 2];
        if (vals.size() / 2 % 2 == 1) {
            q1 = vals[vals.size() / 4];
            q3 = vals[3 * vals.size() / 4];
        } else {
            q1 = (vals[vals.size() / 4 - 1] + vals[vals.size() / 4]) / 2.0;
            q3 = (vals[3 * vals.size() / 4 - 1] + vals[3 * vals.size() / 4]) / 2.0;
        }
    } else {
        q2 = (vals[vals.size() / 2 - 1] + vals[vals.size() / 2]) / 2.0;
        if (vals.size() / 2 % 2 == 1) {
            q1 = vals[vals.size() / 4];
            q3 = vals[3 * vals.size() / 4];
        } else {
            q1 = (vals[vals.size() / 4 - 1] + vals[vals.size() / 4]) / 2.0;
            q3 = (vals[3 * vals.size() / 4 - 1] + vals[3 * vals.size() / 4]) / 2.0;
        }
    }

    double iqr = q3 - q1;
    double lower_bound = q1 - 2.5 * iqr;
    double upper_bound = q3 + 2.5 * iqr;

    double outlier = 0;
    double o_score = 0;
    for (double val : vals) {
        if (val > upper_bound || val < lower_bound) {
            // abnormal value detected!
            double score = std::min(fabs(val - lower_bound), fabs(val - upper_bound)) / iqr;
            if (val > threshold){
                // RCLCPP_WARN(this->get_logger(), "Abnormal value detected: %.03f ms (score: %.03f)", val / 1000, score);
                if (o_score < score){
                    o_score = score;
                    outlier = val;
                }
            }
        }
    }
    if (o_score == 0) return std::nullopt; // if no outlier found
    return std::make_tuple(outlier, o_score);
}

void Ground::stage_changes() {
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
            // inf_mapper.resize(inf_list.size(), 0); // Initialize inf_mapper vector with size inf_list.size()
            for (int i = 0; i < inf_list.size(); i++) {
                this->inf_mapper[inf_list[i]] = i; // inverse mapping
            }
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to PREPARATION.");
            this->prep_once();
            break;
        case STAGE_EXEC:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to MISSION EXECUTION.");
            log_file_ << std::fixed << std::setprecision(4) // stage mark in log
                        << this->get_clock()->now().nanoseconds() / 1000 << ","
                        << 0 << "," 
                        << "EXEC_STAGE,"
                        << 0 << ","
                        << 0 << ","
                        << 0 << ","
                        << 0 << "\n";
            this->exec_once();
            break;
        case STAGE_FINI:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to LANDING (completion).");
            break;
        case STAGE_TERM:
            RCLCPP_INFO(this->get_logger(), "Stage Update: set to TERMINATION.");
            // rclcpp::shutdown();
            break;
        default:
            break;
    }
}