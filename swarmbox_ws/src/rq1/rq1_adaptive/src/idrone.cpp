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
    float leader_n = this->box_rcvd_task_cmd_.wp_n;
    float leader_e = this->box_rcvd_task_cmd_.wp_e;
    Formation relative_formation;
    
    // goto received task command
    if (this->identity == 0) { // Leader Drone
        if (this->tick % 40 == 0) {
            marker("I am at position (%.2f, %.2f, %.2f)", 
                   this->world_pos.x(), this->world_pos.y(), this->world_pos.z());
        }
        traj_setpoint.north = leader_n;
        traj_setpoint.east  = leader_e;
        traj_setpoint.down  = this->box_rcvd_task_cmd_.wp_d + this->z_bias;
        if (this->box_rcvd_task_cmd_.yaw_control) {
            traj_setpoint.yaw = this->box_rcvd_task_cmd_.wp_yaw; // use the yaw from the task command
        } else {
            traj_setpoint.yaw = atan2(
                leader_e - this->world_pos.y(), 
                leader_n - this->world_pos.x()
            ); 
        }

        sb_base::msg::TaskCommand task_cmd;
        task_cmd.orig_id = this->identity;
        task_cmd.code = 1; // whatever...
        task_cmd.wp_n = this->box_rcvd_task_cmd_.wp_n;
        task_cmd.wp_e = this->box_rcvd_task_cmd_.wp_e;
        task_cmd.wp_d = this->box_rcvd_task_cmd_.wp_d;
            if (this->box_rcvd_task_cmd_.code == 1 && this->hasReached(
            setpoint{
                .north = this->box_rcvd_task_cmd_.wp_n,
                .east  = this->box_rcvd_task_cmd_.wp_e,
                .down  = this->box_rcvd_task_cmd_.wp_d,
                .yaw   = 0.0f,
                .type  = 0
            },
            1.0f, 1.0f)) {
        task_cmd.code = 1;
    } else {
        task_cmd.code = 0;
    }
        if (this->box_rcvd_task_cmd_.yaw_control) {
            task_cmd.wp_yaw = this->box_rcvd_task_cmd_.wp_yaw;
        } else {
            task_cmd.wp_yaw = traj_setpoint.yaw;
        }
        task_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        // for every inferior, 
        for (int i : this->inf_list) {
            task_cmd.dest_id = i;
            this->box_publish_task(i, task_cmd);
            marker_debug_once("Task command sent to drone %d: wp_n=%.2f, wp_e=%.2f, wp_d=%.2f", 
                              i, task_cmd.wp_n, task_cmd.wp_e, task_cmd.wp_d);
        }

    } else { // Follower Drone
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
    if (this->box_rcvd_task_cmd_.code == 1) {
        // if I am the leader, check if I have reached the target
        if (this->identity == 0) {
            if (this->hasReached(setpoint{
                        .north = this->box_rcvd_task_cmd_.wp_n,
                        .east  = this->box_rcvd_task_cmd_.wp_e,
                        .down  = this->box_rcvd_task_cmd_.wp_d,
                        .yaw   = 0.0f,
                        .type  = 0
                    }, 1.0f, 1.0f)) {
                marker_once("IDrone exec stage completed!");
                return true; 
            } else return false; // if I am leader but not reached yet
        } else return true; // if I am follower, I am complete if the leader declares completion.
    } else {
        return false; // not even the last waypoint
    }
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