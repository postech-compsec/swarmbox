#include "sb_base/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <optional>

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

    setpoint traj_setpoint;    

    traj_setpoint.north = this->box_rcvd_task_cmd_.wp_n;
    traj_setpoint.east  = this->box_rcvd_task_cmd_.wp_e;
    traj_setpoint.down  = this->box_rcvd_task_cmd_.wp_d;
    traj_setpoint.type  = 0;

    return traj_setpoint;
}

bool IDrone::exec_complete() {
    if (this->box_rcvd_task_cmd_.code == 1) {
        marker_once("IDrone exec stage completed!");
        return true; 
    }
    return false;
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