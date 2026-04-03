#include "sb_base/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <cstring>
// #include <bitset>
#include <vector>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

class IDrone : public Drone {
    public:
        IDrone(const rclcpp::NodeOptions & options);
        ~IDrone();

        std::vector<Eigen::Vector2i> path;
    protected:
        // implement your own flight logic in following methods
        void        prep_once() override;
        void        prep_loop() override;
        bool        prep_complete() override;
        void        exec_once() override;
        std::optional<setpoint> exec_loop() override;
        bool        exec_complete() override;

        int path_count = 0;

        std::vector<Eigen::Vector2i> generate_boustrophedon(int x1, int y1, int x2, int y2, int sweep_width);

        // std::bitset<90000> local_coverage; // 0: not covered, 1: covered(visited)
};


IDrone::IDrone(const rclcpp::NodeOptions & options) : Drone(options) {
    marker_once("Inherited Drone (IDrone) node initialized!");
}

// ====================================
// Custom Mission Functions
// ====================================
std::vector<Eigen::Vector2i> IDrone::generate_boustrophedon(int x1, int y1, int x2, int y2, int sweep_width) {
    // Generate a boustrophedon path from (x1, y1) to (x2, y2) with given sweep width, starting at (sx, sy)
    std::vector<Eigen::Vector2i> path;
    int x = x1 + (sweep_width/2);
    int y = y1;
    bool dir_up = true; // Start sweeping to the right
    // initial push
    path.push_back({x,y}); // initial position

    while (x <= x2) {
        int target_y;

        if (dir_up) {
            target_y = y2; // Move to the top of the sweep
        } else {
            target_y = y1; // Move to the bottom of the sweep
        }
        // Add the current position to the path
        path.push_back({x, target_y});
        // next sweep
        y = target_y;
        x += sweep_width; // Move to the next sweep position
        if (x <= x2) {
            path.push_back({x,y});
        }

        dir_up = !dir_up; // Toggle direction
    }
    // lastly, go to origin.
    path.push_back({(int)this->init_pos.x(), (int)this->init_pos.y()}); // Return to the starting position
    return path;
}


// ====================================
// Override Functions
// ====================================
void IDrone::prep_once() {
    this->marker_once("IDrone prep_once called!");
    // Add any one-time preparation logic here
}

void IDrone::prep_loop() {
    if (this->path.empty() && this->box_rcvd_task_cmd_.data != "") {
        // Parse the task command data
        std::string data = this->box_rcvd_task_cmd_.data;
        size_t pos = 0;
        std::vector<int> bounds;
        while ((pos = data.find(':')) != std::string::npos) {
            bounds.push_back(std::stoi(data.substr(0, pos)));
            data.erase(0, pos + 1);
        }
        bounds.push_back(std::stoi(data)); // Last value after the last colon

        // Generate boustrophedon path based on received bounds
        this->path = generate_boustrophedon(bounds[0], bounds[1], bounds[2], bounds[3], 10);
        marker("Boustrophedon path generated: (%d, %d), ..., (%d, %d)", 
                        this->path[0].x(), this->path[0].y(),
                        this->path.back().x(), this->path.back().y());
    }
    marker_once("IDrone prep_loop called!");
    // Add any loop preparation logic here
}

bool IDrone::prep_complete() {
    // Add logic to determine if preparation is complete
    // check if vehicle is armed
    if (this->armed && !this->path.empty()) {
        marker_once("IDrone prep stage completed!");
        return true; 
    } else {
        return false;
    }
}

void IDrone::exec_once() {
    marker_once("IDrone exec_once called!");
    // Add any one-time execution logic here
}

std::optional<setpoint> IDrone::exec_loop() {
    marker_once("IDrone exec_loop called!");
    setpoint home = {this->init_pos.x(), this->init_pos.y(), -10.0f, 0.0f, 0}; // Default setpoint to start at (0, 0, -10)
    if (this->path.empty()) {
        marker_once("No path available for execution. Please ensure prep stage is complete.");
        // return {0.0f, 0.0f, -10.0f, 0.0f}; // Default setpoint
        return home;
    }


    // if I am near path[path_count], path_count += 1;
    if (this->world_pos.x() >= this->path[path_count].x() - 0.3 &&
        this->world_pos.x() <= this->path[path_count].x() + 0.3 &&
        this->world_pos.y() >= this->path[path_count].y() - 0.3 &&
        this->world_pos.y() <= this->path[path_count].y() + 0.3) {
        path_count++;
        float yaw = atan2(
            this->path[path_count].y() - this->world_pos.y(),
            this->path[path_count].x() - this->world_pos.x()
        );
        if (path_count >= path.size()) {
            marker_once("Reached the end of the path. Returning to start.");
            path_count = -1; // Reset to start
            home.yaw = yaw;
            return home; // Return to start setpoint
        } else {
            // todo: sp - relative position
            setpoint sp = {(float)(path[path_count].x()), (float)(path[path_count].y()), -10.0f, yaw, 0}; // Return the next setpoint
            return sp;
        }
        // return path[path_count]
    } else if (path_count == 0 & this->tick % 10 == 0) { // takeoff command: but throttled to every 10 ticks
            float yaw = atan2(
                this->path[path_count].y() - this->world_pos.y(),
                this->path[path_count].x() - this->world_pos.x()
            );
            setpoint sp = {(float)(path[path_count].x()), (float)(path[path_count].y()), -10.0f, yaw, 0}; // Return the next setpoint
            return sp;
    }
    return std::nullopt; // if there's no setpoint change, we don't have to return any setpoint.
}

bool IDrone::exec_complete() {
    // if I have visited all points in the path
    if (path_count == -1) {
        return true; // Execution is complete
    } else {
        // marker_debug_once("IDrone exec stage not yet completed. Current path count: %d", path_count);
        return false;
    }
}

IDrone::~IDrone() {
    marker("Inherited Drone (IDrone) node destroyed!");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto idrone_node = std::make_shared<IDrone>(rclcpp::NodeOptions());

    rclcpp::spin(idrone_node);

    rclcpp::shutdown();
    return 0;
}