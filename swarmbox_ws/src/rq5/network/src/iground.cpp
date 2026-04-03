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
}

void IGround::prep_once() {
    marker("IGround prep_once called!");
    // sector positions / role initialization
}

void IGround::prep_loop() {
}

void IGround::exec_once() {
    this->tick = 0; // Reset tick for execution stage
}

void IGround::exec_loop() {

    int heatmap_id = 0;
    if (this->tick % (40) == 0) {
        // create random heatmap every 10s
        std::vector<std::vector<float>> heatmap(10, std::vector<float>(10, 0.0f));
        
        
        // // RANDOMIZED HEATMAP
        // for (size_t i = 0; i < 10; i++) {
        //     for (size_t j = 0; j < 10; j++) {
        //         heatmap[i][j] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        //     }
        // }

        std::string heatmap_name = "";

        // BIASED HEATMAP filter
        for (size_t i = 0; i < 10; i++) {
            for (size_t j = 0; j < 10; j++) {
                // BIASED HEATMAP
                // option 1. fill 1 for i < 5
                if ((tick-200) / 800 < 1) { // bottom-left quadrant
                    if (i < 5 && j < 5) {
                        heatmap[i][j] = 1;
                    } else {
                        heatmap[i][j] = 0;
                    }
                    heatmap_name = "Bottom Left Quadrant";
                    // this->marker_once("Heatmap 1");
                    heatmap_id = 1;
                } else if ((tick-200) / 800 < 2) { // bottom-right quadrant
                    if (i < 5 && j > 5) {
                        heatmap[i][j] = 1;
                    } else {
                        heatmap[i][j] = 0;
                    }
                    heatmap_name = "Bottom Right Quadrant";
                    // this->marker_once("Heatmap 2");
                    heatmap_id = 2;
                } else if ((tick-200) / 800 < 3) { // top-right quadrant
                    if (i > 5 && j > 5) {
                        heatmap[i][j] = 1;
                    } else {
                        heatmap[i][j] = 0;
                    }
                    heatmap_name = "Top Right Quadrant";
                    // this->marker_once("Heatmap 3");
                    heatmap_id = 3;
                } else if ((tick-200) / 800 < 4) { // top-left quadrant
                    if (i > 5 && j < 5) {
                        heatmap[i][j] = 1;
                    } else {
                        heatmap[i][j] = 0;
                    }
                    heatmap_name = "Top Left Quadrant";
                    // this->marker_once("Heatmap 4");
                    heatmap_id = 4;
                } else { // full heatmap
                    heatmap[i][j] = 1;
                    // this->marker_once("Heatmap 5");
                    heatmap_name = "Full Heatmap";
                    heatmap_id = 5;
                }
            }
        }

        // change heatmap to string data
        std::string heatmap_str = "";
        for (const auto& row : heatmap) {
            for (const auto& val : row) {
                heatmap_str += std::to_string(val) + ",";
            }
            heatmap_str += "\n";
        }
        // marker("Heatmap string: %s", heatmap_str.c_str());

        // create position list
        std::string positions = "";
        for (const auto& [i, report] : this->inf_reports) {
            positions += std::to_string(report.pos_x) + "," +
                        std::to_string(report.pos_y);
            if (i < this->total_swarm_size - 1) {
                positions += ";"; // separate with semicolon
            }
        }

        // join heatmap and positions
        std::string task_data = "heatmap:\n" + heatmap_str + "positions:\n" + positions;

        // publish heatmap to all drones using TaskCommand
        for (const auto& [i, _] : this->inf_mapper) {
            sb_base::msg::TaskCommand task{};
            task.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            task.orig_id = -1; // ground
            task.dest_id = i; // drone ID
            task.code = 0;
            task.data = task_data;

            this->box_publish_task(i, task);
        }
        this->marker("Heatmap [%s] Active.", heatmap_name.c_str());
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
