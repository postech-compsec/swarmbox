#include "sb_base/ground.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <bitset>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

// use per_row int as define
#define DRONES_PER_ROW 4

class IGround : public Ground {
    public:
        IGround(const rclcpp::NodeOptions & options);
        // int swarm_size; instead use this->total_swarm_size.

        ~IGround() {
            std::cout << ("IGround node terminated.") << std::endl;
        }

        void prep_once() override;

        // void prep_loop() override;
        // bool prep_complete() override;
        // void exec_once() override;
        void exec_loop() override;
        // bool exec_complete() override;
        std::bitset<10000 * DRONES_PER_ROW * DRONES_PER_ROW> global_coverage; // 0: not covered, 1: covered(visited)
        // int boundpoint[swarm_size][4];
        int globalbound[4];
};

IGround::IGround(const rclcpp::NodeOptions & options) : Ground(options) {
    marker_once("Inherited Ground (IGround) node initialized!");
    // this->swarm_size = swarm_size;
    // read from values
    for (const auto& [id, vals] : values_) {
        if (id == 0) {
            globalbound[0] = vals[0];
            globalbound[2] = vals[1];
        } else if (id == 1) {
            globalbound[1] = vals[0];
            globalbound[3] = vals[1];
        }
    }
}

void IGround::prep_once() {
    marker("IGround prep_once called!");
    // initial positions: (x,y) for each drone
    // int positions[swarm_size][2]; relative positions processing should be done in drones.
    // for (int i = 0; i < swarm_size; i++) {
    //     positions[i][0] = 5 * (i % 3); // x position
    //     positions[i][1] = 5 * (i / 3); // y position
    // }
    
    // from initial positions, assign drones' partitions: *100/3. for example, (0,0) -> (0,0), (3,0) -> (100,0)
    int boundpoint[this->total_swarm_size][4];
    for (int i : inf_list) {
        boundpoint[i][0] = 100 * (i % DRONES_PER_ROW);
        boundpoint[i][1] = 100 * (i / DRONES_PER_ROW);
        boundpoint[i][2] = 100 * ((i % DRONES_PER_ROW) + 1);
        boundpoint[i][3] = 100 * ((i / DRONES_PER_ROW) + 1);
        // send data to drones
        // for now, just print (x1, y1, x2, y2) for each drone
        marker("Drone %d Boundaries: (%d, %d, %d, %d)", 
                    i, boundpoint[i][0], boundpoint[i][1], 
                    boundpoint[i][2], boundpoint[i][3]);

        TaskCommand task_cmd;
        task_cmd.data = std::to_string(boundpoint[i][0]) + ":" + 
                                    std::to_string(boundpoint[i][1]) + ":" + 
                                    std::to_string(boundpoint[i][2]) + ":" + 
                                    std::to_string(boundpoint[i][3]);
        task_cmd.orig_id = -1; // ground
        task_cmd.code = 0; // code 0 for partitioning
        task_cmd.dest_id = i;
        task_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        this->box_publish_task(i, task_cmd);
        marker("Task command sent to drone %d: %s", 
                    i, task_cmd.data.c_str());
    }
   
}

void IGround::exec_loop() {
    // calculate global coverage map
    for (int i : inf_list) {
        // get the inferior's location
        Eigen::Vector3d pos = this->inf_pos[i];
        // make local coverage, as 11*11, where the center is the inferior's position
        std::bitset<10000 * DRONES_PER_ROW * DRONES_PER_ROW> local_coverage; // 0: not covered, 1: covered(visited)
        // calculate the range of coverage
        int x_start = std::max(0, (int)(pos.x() - 5)); // 5 is half of the coverage range
        int y_start = std::max(0, (int)(pos.y() - 5));
        int x_end = std::min(DRONES_PER_ROW * 100 - 1, (int)(pos.x() + 5)); // 899 is the max index for 90000 bits
        int y_end = std::min(DRONES_PER_ROW * 100 - 1, (int)(pos.y() + 5));

        // fill the local coverage map
        for (int x = x_start; x <= x_end; x++) {
            for (int y = y_start; y <= y_end; y++) {
                // calculate the index in the bitset
                int index = y * DRONES_PER_ROW * 100 + x; // 300 is the width of the coverage map
                local_coverage.set(index); // set the bit to 1 (covered)
            }
        }

        // update global coverage map
        this->global_coverage |= local_coverage;
    }
    if (tick % 40 == 0) {
        // print global coverage (%)
        int covered_count = this->global_coverage.count();
        int total_count = this->global_coverage.size();
        float coverage_percentage = (float)covered_count / total_count * 100.0f;
        marker("Global coverage: %.2f%% (%d / %d)", 
                    coverage_percentage, covered_count, total_count);
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
