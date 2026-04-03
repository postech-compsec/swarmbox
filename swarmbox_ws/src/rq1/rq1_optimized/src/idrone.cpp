#include "sb_base/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <optional>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

class IDrone : public Drone {
public:
    IDrone(const rclcpp::NodeOptions & options);
    ~IDrone() override;

protected:
    void prep_once() override;
    bool prep_complete() override;
    void exec_once() override;
    std::optional<setpoint> exec_loop() override;
    bool exec_complete() override;

private:
    float cruise_altitude_ = -10.0f;
};
    
IDrone::IDrone(const rclcpp::NodeOptions & options) : Drone(options) {
    marker_once("Inherited Drone (IDrone) node initialized!");
}

IDrone::~IDrone() {
    marker("Inherited Drone (IDrone) node destroyed!");
}

void IDrone::prep_once() {
    marker_once("IDrone prep_once called!");
}

bool IDrone::prep_complete() {
    if (this->armed) {
        marker_once("IDrone prep stage completed!");
        return true; 
    }
    return false;
}

void IDrone::exec_once() {
    this->tick = 0;
}

std::optional<setpoint> IDrone::exec_loop() {
    setpoint sp;

    // During preparation, command takeoff to cruise altitude
    if (tick < 40*10){
        sp.type = 0; // Position setpoint for takeoff
        sp.north = this->init_pos.x();
        sp.east = this->init_pos.y();
        sp.down = cruise_altitude_;
        sp.yaw = 0.0f;
        return sp;
    }

    // After preparation, follow velocity commands from GCS
    if (box_rcvd_task_cmd_.timestamp > 0 && box_rcvd_task_cmd_.code == 1) { // Check for velocity command
        sp.type = 1; // Set setpoint type to VELOCITY
        sp.north = box_rcvd_task_cmd_.wp_n; 
        sp.east = box_rcvd_task_cmd_.wp_e;
        // target_setpoint.down = (this->world_pos.z() + 10.0f) * -0.5f;
        sp.down = (this->world_pos.z() - cruise_altitude_) * -0.5f;
        sp.yaw = box_rcvd_task_cmd_.wp_yaw; // Use yaw from the command
        return sp;
    }

    // If no command, hold current position by sending a velocity of zero.
    sp.north = 0.0f;
    sp.east = 0.0f;
    sp.down = 0.0f;
    sp.yaw = 0.0f;
    sp.type = 1; // Velocity setpoint
    return sp;
}

bool IDrone::exec_complete() {
    // The flocking mission runs continuously
    // completion after 40*2*60 ticks (2 minutes)
    if (this->tick > 40*2*60) {
        return true; // Execution is complete
    }
    return false;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto idrone_node = std::make_shared<IDrone>(rclcpp::NodeOptions());
    rclcpp::spin(idrone_node);
    rclcpp::shutdown();
    return 0;
}