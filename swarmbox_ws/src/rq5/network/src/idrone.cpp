#include "sb_base/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <Eigen/Dense>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

// Helper function to parse a string of delimited values into a vector of doubles
std::vector<double> parse_string(const std::string& s, char delimiter) {
    std::vector<double> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(std::stod(token));
    }
    return tokens;
}

class IDrone : public Drone {
    public:
        // IDrone(int id, int superior, float loc_s_m[3]);
        IDrone(const rclcpp::NodeOptions & options);
        ~IDrone();

    protected:
        // implement your own flight logic in following methods
        void        prep_once() override;
        void        prep_loop() override;
        bool        prep_complete() override;
        void        exec_once() override;
        std::optional<setpoint> exec_loop() override;
        bool        exec_complete() override;

        bool rel_set = false; // whether relative position is received from leader
};


IDrone::IDrone(const rclcpp::NodeOptions & options) : Drone(options) {
    marker_once("Inherited Drone (IDrone) node initialized!");
}

// ====================================
// Override Functions
// ====================================
void IDrone::prep_once() {
    marker_once("IDrone prep_once called!");
    // Add any one-time preparation logic here
    this->rel_set = true;
}

void IDrone::prep_loop() {
}

bool IDrone::prep_complete() {
    if (this->rel_set & this->armed
    ) {
        marker_once("IDrone prep stage completed!");
        return true;
    } else {
        return false;
    }
}

void IDrone::exec_once() {}

std::optional<setpoint> IDrone::exec_loop() {
    // --- Constants for Behavior Logic ---
    const float ATTRACTION_STRENGTH = 5.0f;
    const float REPULSION_STRENGTH = 500.0f;
    const float DAMPING_FACTOR = 20.0f;
    const float MAX_SPEED = 5.0f; // m/s
    const float STEP_SIZE = 0.1f; // How far to project the target position per calculation


    if (tick < 200) {
        setpoint sp;
        sp.north = this->init_pos.x();
        sp.east = this->init_pos.y();
        sp.down = -10.0f; // Maintain constant altitude
        sp.yaw = 0.0f; // No specific yaw control
        return sp;
    } else if (tick % 40 == 0) { // run every 1/4 second
        // --- 1. Parse Incoming Data from GCS ---
        std::string task_data = this->box_rcvd_task_cmd_.data;
        
        // Define headers and separators
        const std::string heatmap_header = "heatmap:\n";
        const std::string positions_separator = "\npositions:\n";

        // Find the separator to split heatmap and position data
        size_t separator_pos = task_data.find(positions_separator);
        if (separator_pos == std::string::npos) {
            // marker("Invalid task_data format: separator not found.");
            return std::nullopt;
        }

        // Extract raw heatmap and position strings
        std::string heatmap_part = task_data.substr(0, separator_pos);
        std::string positions_str = task_data.substr(separator_pos + positions_separator.length());
        std::string heatmap_str = "";
        if (heatmap_part.rfind(heatmap_header, 0) == 0) {
            heatmap_str = heatmap_part.substr(heatmap_header.length());
        }

        // Parse heatmap string into a 2D vector (10x10)
        std::vector<std::vector<float>> heatmap(10, std::vector<float>(10, 0.0f));
        std::stringstream ss_heatmap(heatmap_str);
        std::string row_str;
        int row_idx = 0;
        // marker("Parsing heatmap string: %s", heatmap_str.c_str());
        while (std::getline(ss_heatmap, row_str, '\n') && row_idx < 10) {
            auto row_values = parse_string(row_str, ',');
            // marker("Parsed heatmap row: %s", row_str);
            if (row_values.size() == 10) {
                for (int col_idx = 0; col_idx < 10; ++col_idx) {
                    heatmap[row_idx][col_idx] = static_cast<float>(row_values[col_idx]);
                }
            }
            row_idx++;
        }

        // marker("Positions string: %s", positions_str.c_str());
        std::vector<Eigen::Vector2d> other_drones;
        std::stringstream ss_positions(positions_str);
        std::string pos_pair_str;
        int current_index = 0;
        while(std::getline(ss_positions, pos_pair_str, ';')) {
            if (current_index != this->identity) {
                auto coords = parse_string(pos_pair_str, ',');
                if(coords.size() == 2) {
                    other_drones.push_back(Eigen::Vector2d(coords[0], coords[1]));
                }
            }
            current_index++;
        }
        // marker("Parsed %zu other drones' positions.", other_drones.size());

        // --- 2. Calculate Forces based on Parsed Data ---
        Eigen::Vector2d current_pos(this->world_pos.x(), this->world_pos.y());
        Eigen::Vector2d current_vel(this->velocity.x(), this->velocity.y());

        // Attraction Force: Move towards the weighted centroid of the heatmap
        Eigen::Vector2d attraction_force(0.0, 0.0);
        float total_heat = 0.0f;
        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 10; ++j) {
                float heat = heatmap[i][j];
                if (heat > 0) {
                    // Map grid cell (i,j) to world coordinates [-50, 50]
                    Eigen::Vector2d cell_center(i * 10.0 - 45.0, j * 10.0 - 45.0);
                    attraction_force += (cell_center - current_pos) * (heat * heat);
                    total_heat += heat;
                }
            }
        }
        if (total_heat > 0) {
            attraction_force /= total_heat; // Get the average direction
        } else {
            marker("Total heat is 0!");
        }

        // Repulsion Force: Move away from other nearby drones
        Eigen::Vector2d repulsion_force(0.0, 0.0);
        for (const auto& other_pos : other_drones) {
            Eigen::Vector2d vec_to_other = current_pos - other_pos;
            double dist = vec_to_other.norm();
            if (dist > 0.01 && dist < 15.0) { // Repel only if within 15 meters
                repulsion_force += vec_to_other.normalized() / (dist);
            }
        }

        // Damping Force: Stabilize movement by resisting current velocity
        Eigen::Vector2d damping_force = -current_vel * DAMPING_FACTOR;

        // --- 3. Combine Forces and Generate Setpoint ---
        Eigen::Vector2d total_force = (attraction_force * ATTRACTION_STRENGTH) + 
                                    (repulsion_force * REPULSION_STRENGTH) +
                                    damping_force;

        // The total force is our desired velocity vector
        Eigen::Vector2d desired_velocity = total_force;
        if (desired_velocity.norm() > MAX_SPEED) {
            desired_velocity = desired_velocity.normalized() * MAX_SPEED;
        }

        // Calculate a target position by projecting from the current position
        // This is more stable for PX4 Offboard mode than a pure velocity setpoint
        Eigen::Vector2d target_pos = current_pos + desired_velocity.normalized() * STEP_SIZE;

        
        setpoint target_setpoint;
        // target_setpoint.north = target_pos.x();
        // target_setpoint.east = target_pos.y();
        // target_setpoint.down = -10.0f; // Maintain constant altitude
        target_setpoint.north = desired_velocity.x();
        target_setpoint.east = desired_velocity.y();
        // calculate velocity to maintain altitude
        target_setpoint.down = (this->world_pos.z() + 10.0f) * -0.5f;
        target_setpoint.type = 1; // velocity control
        target_setpoint.yaw = 0.0f; // No specific yaw control
        // target_setpoint.yaw = atan2(desired_velocity.y(), desired_velocity.x());

        // marker("Target Position: (%.2f, %.2f)", target_pos.x(), target_pos.y());
        // marker("Generated setpoint: (attr: %.2f, rep: %.2f, damping: %.2f), %.2f m/s", 
        //     attraction_force.norm(), repulsion_force.norm(), damping_force.norm(), desired_velocity.norm());
        return target_setpoint;
    }
    return std::nullopt; // No setpoint to return, can be modified as needed
}


bool IDrone::exec_complete() {
    if (this->tick >= 4200) {
        return true;
    }
    return false;
}

IDrone::~IDrone() {
    marker("Inherited Drone (IDrone) node destroyed!");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto idrone_node = std::make_shared<IDrone>(options);

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(idrone_node);
    executor.add_node(idrone_node->get_px4_node());

    executor.spin();

    rclcpp::shutdown();
    return 0;
}