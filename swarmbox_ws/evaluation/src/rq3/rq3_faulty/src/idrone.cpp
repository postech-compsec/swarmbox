#include "sb_base/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

struct Formation {
    double n; 
    double e;
    double d;
};

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

void IDrone::prep_loop() {}

bool IDrone::prep_complete() {
    if (this->armed & this->rel_set) {
        marker_once("IDrone prep stage completed!");
        return true;
    } else {
        return false;
    }
}

void IDrone::exec_once() {}

std::optional<setpoint> IDrone::exec_loop() {
    setpoint traj_setpoint;    
    float leader_n = this->box_rcvd_task_cmd_.wp_n;
    float leader_e = this->box_rcvd_task_cmd_.wp_e;
    Formation relative_formation;
    
    if (this->tick < 40*5) {
        // first, goto takeoff(0,0,-10) (for first 5 seconds: to prevent ground crashings)
        traj_setpoint.north = this->init_pos.x();
        traj_setpoint.east  = this->init_pos.y();
        traj_setpoint.down  = -10.0f + this->z_bias; // z_bias is added to prevent ground crashings.
        traj_setpoint.yaw   = 0.0f; // no yaw control for now.
    } else {
        double formation_yaw = this->box_rcvd_task_cmd_.wp_yaw; // use the yaw from the task command
        relative_formation.n =   this->init_pos.x() * cos(formation_yaw) - this->init_pos.y() * sin(formation_yaw);
        relative_formation.e =   this->init_pos.x() * sin(formation_yaw) + this->init_pos.y() * cos(formation_yaw);
        traj_setpoint.north = leader_n + relative_formation.n; // 
        traj_setpoint.east  = leader_e + relative_formation.e; // 
        traj_setpoint.down  = this->box_rcvd_task_cmd_.wp_d + this->z_bias;
        traj_setpoint.yaw = formation_yaw;
    }

    return traj_setpoint;
}

bool IDrone::exec_complete() {
    // if I got back to the initial position, then return true.
    if (this->tick > 1000 &&
        abs(this->world_pos.x() - this->init_pos.x()) < 1 && 
        abs(this->world_pos.y() - this->init_pos.y()) < 1) {
        // if I am near
        return true;
    } else {
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