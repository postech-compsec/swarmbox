#include "sb_base/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

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
    return std::nullopt; // No setpoint to return, can be modified as needed
}

bool IDrone::exec_complete() {
    if (true) { // set completion condition!
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