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
        int sector_id;
        Eigen::Vector3d standby;
        bool was_active = false;
};


IDrone::IDrone(const rclcpp::NodeOptions & options) : Drone(options) {
    marker_once("Inherited Drone (IDrone) node initialized!");
    // read values from options
}

// ====================================
// Override Functions
// ====================================
void IDrone::prep_once() {
    marker_once("IDrone prep_once called!");
    // Add any one-time preparation logic here
    
}

void IDrone::prep_loop() {
    if (!this->rel_set){
        if (this->box_rcvd_task_cmd_.data != "") {
            // read and understand my role.
            this->sector_id = this->box_rcvd_task_cmd_.code; // sector id
            if (this->sector_id == -1){
                // I am assigned as active tracker.
                marker("I am assigned as active tracker!");
                this->standby.x() = 0;
                this->standby.y() = 0;
                // this->active = true;
                this->rel_set = true;
            } else if (this->sector_id < 4 && this->sector_id >= 0) {
                // I am assigned as sector sentinel.
                marker("I am assigned as sector sentinel for sector %d", this->sector_id);
                // set standby position. 
                this->standby.x() = pow(-1, (this->sector_id % 2)) * -50 + pow(-1, (this->identity % 2)) * -20;
                this->standby.y() = pow(-1, (this->sector_id / 2)) * -50 + pow(-1, (this->identity % 2)) * -20;
                marker("Standby position set to (%.2f, %.2f)", this->standby.x(), this->standby.y());
                this->rel_set = true;
            } else {
                // invalid sector code assigned.
            }
        }
    }
}

bool IDrone::prep_complete() {
    if (
        this->armed &
        this->rel_set) {
        marker_once("IDrone prep stage completed!");
        return true;
    } else {
        return false;
    }
}

void IDrone::exec_once() {}

std::optional<setpoint> IDrone::exec_loop() {
    setpoint startpoint{this->init_pos.x(), this->init_pos.y(), -10.0f, 0.0, 0};
    if (this->tick < 400) {
        return startpoint;
    }

    // read taskcommand.
    if (this->box_rcvd_task_cmd_.code == 1) {
        // if I received 1 as code, I am active tracker now.
        setpoint tracking_setpt{};
        tracking_setpt.north = this->box_rcvd_task_cmd_.wp_n;
        tracking_setpt.east  = this->box_rcvd_task_cmd_.wp_e;
        
        // Tracking Formation: keep Y shape 
        // (equilateral triangle formation, where tracking obj is at center)
        if (this->sector_id != -1) {
            tracking_setpt.north += 2.5; 
            tracking_setpt.east  += pow(-1, (this->identity % 2)) * -2.5 * sqrt(3.0f);
        } else {
            tracking_setpt.north -= 5.0;
        }

        tracking_setpt.down = -10.0f;
        tracking_setpt.yaw = atan2(
            tracking_setpt.east - this->world_pos.y(), 
            tracking_setpt.north - this->world_pos.x()
        );
        if (!this->was_active) {
            marker("changed to active tracker");
            this->was_active = true;
        }
        return tracking_setpt; // Return tracking_setpt setpoint
    } else {
        // else (I received 0 as code), I am standby mode: get back to my standby position.
        setpoint standby_setpoint{};
        standby_setpoint.north = this->standby.x();
        standby_setpoint.east  = this->standby.y();
        standby_setpoint.down  = -10.0f;
        standby_setpoint.yaw   = atan2(
            standby_setpoint.east - this->world_pos.y(), 
            standby_setpoint.north - this->world_pos.x()
        );
        if (this->was_active) {
            marker("changed to standby mode");
            this->was_active = false;
        }
        return standby_setpoint; // Return standby setpoint
    }

    return std::nullopt; // No setpoint to return, can be modified as needed
}

bool IDrone::exec_complete() {
    if (this->tick > 60*2*40 + 400) {
        // mark as complete after 2 minutes of execution stage.
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

    rclcpp::NodeOptions options;
    auto idrone_node = std::make_shared<IDrone>(options);

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(idrone_node);
    executor.add_node(idrone_node->get_px4_node());

    executor.spin();

    rclcpp::shutdown();
    return 0;
}