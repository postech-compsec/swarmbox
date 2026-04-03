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
        void exec_once() override;
        void exec_loop() override;
};

IGround::IGround(const rclcpp::NodeOptions & options) : Ground(options) {
    // int positions[swarm_size][2];
    marker_once("Inherited Ground (IGround) node initialized!");
    // this->swarm_size = swarm_size;
}

void IGround::prep_once() {
    marker("IGround prep_once called!");
}

void IGround::prep_loop() {
}

void IGround::exec_once() {
    this->tick = 0; // Reset tick for execution stage
}

void IGround::exec_loop() {

    
    sb_base::msg::TaskCommand task{};
    task.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    task.orig_id = -1; // ground
    task.dest_id = 0; // leader.
    
    task.code = 0;
    if (tick < 200 + (6.28 * 300)) {
        task.wp_n = 50.0f * sin(((float)tick-200) / 300.0f);
        task.wp_e = 50.0f * cos(((float)tick-200) / 300.0f) - 50.0f;
        task.yaw_control = true;
        double yaw = (200 - (float)tick) / 300.0f;
        task.wp_yaw = atan2(
            sin(yaw), cos(yaw)
        );

    } else {
        task.wp_n = 0.0f;
        task.wp_e = 0.0f;
        task.yaw_control = true;
        task.wp_yaw = 0.0f;
    }
    task.wp_d = -10.0f;

    for (int i : this->inf_list) {
        // Publish task command to each inferior
        this->box_publish_task(i, task);
        // marker_debug("Task command sent to drone %d: wp_x=%.2f, wp_y=%.2f, wp_z=%.2f", 
        //             i, task.wp_x, task.wp_y, task.wp_z);
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
