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
};

IGround::IGround(const rclcpp::NodeOptions & options) : Ground(options) {
    // int positions[swarm_size][2];
    marker_once("Inherited Ground (IGround) node initialized!");
    // read roles from values
    for (const auto& [id, vals] : values_) {
    }
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

    // 
}


int main(int argc, char *argv[]) {
    // ROS2 initialization
    // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IGround>(rclcpp::NodeOptions()));
    rclcpp::shutdown();

    return 0;
}
