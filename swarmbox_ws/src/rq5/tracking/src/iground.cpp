#include "sb_base/ground.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <cstring>
// #include <bitset>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

class IGround : public Ground {
    public:
        std::vector<std::vector<int>> positions; // 2D vector to hold positions of drones
        IGround(const rclcpp::NodeOptions & options);
        int swarm_size;

        ~IGround() {
            std::cout << ("IGround node terminated.") << std::endl;
        }

        void prep_once() override;

        void prep_loop() override;
        // bool prep_complete() override;
        void exec_once() override;
        void exec_loop() override;
        // bool exec_complete() override;

        // roles
        std::vector<int> roles; // roles of drones
        Eigen::Vector3d target; // arbitrary target position
};

IGround::IGround(const rclcpp::NodeOptions & options) : Ground(options) {
    // int positions[swarm_size][2];
    marker_once("Inherited Ground (IGround) node initialized!");
    // read roles from values
    for (const auto& [id, vals] : values_) {
        // value for id means the sector.
        if (vals.size() >= 1) {
            int role = vals[0]; // role of the drone
            this->roles.push_back(role);
            marker("Drone %d assigned role %d", id, role);
        } else {
            marker("Drone %d has no role assigned.", id);
        }
    }
}

void IGround::prep_once() {
    marker("IGround prep_once called!");
    // send roles to each drones, using taskcommand
    for (int i : this->inf_list) {
        sb_base::msg::TaskCommand task{};

        task.code = this->roles[i];
        task.orig_id = -1;
        task.dest_id = i; // Assign to the drone
        task.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        task.data = std::to_string(i % 2);
        marker("Sending role %d to drone %d", this->roles[i], i);
        this->box_publish_task(i, task);
    }
}

void IGround::prep_loop() {

}

void IGround::exec_once() {
    this->tick = 0; // Reset tick for execution stage
    
}

void IGround::exec_loop() {
    // calculate arbitrary target position
    target.x() = 50.0f * sin(((float)tick-400) / 600.0f);
    target.y() = 50.0f * cos(((float)tick-400) / 200.0f);
    // target.z = -10.0f;
    //
    // decide which sector the target is in.
    int target_is_in = 0;
    if (target.x() > 0) {
        target_is_in += 1;
    }
    if (target.y() > 0) {
        target_is_in += 2;
    }
    // marker("Target is at sector %d: (%.02f, %.02f)", target_is_in, target.x(), target.y());

    for (int i : this->inf_list) {
        // now we should publish the target position to the relevant drones, using taskcommand.
        // send the target position to active sector & active tracker (whose role is -1)
        sb_base::msg::TaskCommand task{};
        if (this->roles[i] == -1 || this->roles[i] == target_is_in) {
            // send target's position as wp_n/wp_e value.
            task.code = 1; // active
            task.wp_n = target.x();
            task.wp_e = target.y();
            // marker("Drone %d is set as active tracker", did);
        } else {
            // else, then the drones should standby, send 0 as data.
            task.code = 0; // standby
        }
        task.orig_id = -1;
        task.dest_id = i;
        task.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        this->box_publish_task(i, task);
    }
}


int main(int argc, char *argv[]) {
    // ROS2 initialization
    // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IGround>(rclcpp::NodeOptions()));
    rclcpp::shutdown();

    return 0;
}
