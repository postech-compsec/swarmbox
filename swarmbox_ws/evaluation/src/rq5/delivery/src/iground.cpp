#include "sb_base/ground.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <bitset>
#include <vector>
// #include <fstream>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

struct delivery{
    int north, east;
    bool delivered;
    uint64_t time_of_delivery;
    int drone_id;
    int package_id;

    delivery(double n, double e, int pid)
        : north(n), east(e), delivered(false), time_of_delivery(0), drone_id(-1), package_id(pid) {}
};

class IGround : public Ground {
    public:
        IGround(const rclcpp::NodeOptions & options);

        ~IGround() {
            std::cout << ("IGround node terminated.") << std::endl;
        }

        void prep_once() override;
        void prep_loop() override;
        void exec_loop() override;

        std::vector<delivery> deliver_list = {}; // list of delivery points loaded from file
        std::string filename = "run/deliverypoints.txt";
        int deliver_num = 0; // next delivery point to assign
        std::map<int, std::vector<int>> drone_deliveries; // deliveries assigned to each drone
        int last_count = 0;
        bool preplanned = false;
        std::vector<int> current_deliveries;
};

IGround::IGround(const rclcpp::NodeOptions & options) : Ground(options) {
    marker_once("Inherited Ground (IGround) node initialized!");
    // this->swarm_size = swarm_size;

    // read from values
    for (const auto& [id, vals] : values_) {
        if (vals.size() >= 2) {
            double north = vals[0];
            double east = vals[1];
            deliver_list.emplace_back(north, east, id);
        }
    }

    marker("Loaded %zu deliveries from parsed values", deliver_list.size());
}
            
void IGround::prep_once() {
    marker("IGround prep_once called!");
    // create 5 clusters of deliverypoints
    for (int i : this->inf_list) {
        this->drone_deliveries[i] = {}; // initialize empty delivery list for each drone
    }
    // plan 0: default: no sort.

    // plan A: clustered delivery points: sort by angle
    std::sort(this->deliver_list.begin(), this->deliver_list.end(), [](const delivery& a, const delivery& b) {
        return std::atan2(a.east, a.north) < std::atan2(b.east, b.north);
    });
    
    // plan B: deliver from closest to farthest
    // std::sort(this->deliver_list.begin(), this->deliver_list.end(), [](const delivery& a, const delivery& b) {
    //     return std::sqrt(a.north * a.north + a.east * a.east) < std::sqrt(b.north * b.north + b.east * b.east);
    // });
    
    // plan C: deliver from farthest to closest
    // std::sort(this->deliver_list.begin(), this->deliver_list.end(), [](const delivery& a, const delivery& b) {
    //     return std::sqrt(a.north * a.north + a.east * a.east) > std::sqrt(b.north * b.north + b.east * b.east);
    // });

    if (preplanned){
        // // // // // default: bulk assign deliveries to drones // // // // //
        // (each drone gets deliver_list.size() / swarm_size deliveries at once)
        for (int i = 0; i < this->deliver_list.size(); i++) {
            int drone_id = i / (this->deliver_list.size() / this->inf_list.size());
            // should we reassign package id?
            this->drone_deliveries[drone_id].push_back(i);
        }

        // // plan B: round-robin randomly sorted deliveries
        // for (int i = 0; i < this->deliver_list.size(); i++) {
        //     int drone_id = i % this->deliver_list.size(); // Assign deliveries in round-robin fashion
        //     this->drone_deliveries[drone_id].push_back(i);
        // }

        marker_debug("drone_deliveries size: %zu", this->drone_deliveries.size());
        for (const auto& [i, _] : this->inf_mapper) {
            // change to actual ids of drones, not just indices
            marker("Drone %d has %zu deliveries assigned.", 
                        i, this->drone_deliveries[i].size());
            // send as taskcommand, format: n1:e1,n2:e2,n3:e3,...
            // task_cmd.data = "";
            string points;
            for (int delivery_id : this->drone_deliveries[i]) {
                if (!points.empty()) {
                    points += ",";
                }
                points += std::to_string(deliver_list[delivery_id].north) + ":" + 
                    std::to_string(deliver_list[delivery_id].east) + ":" + 
                    std::to_string(deliver_list[delivery_id].package_id);
            }
            // // Log the assigned deliveries for each drone
            // // Publish the task command to the drone
            TaskCommand task_cmd;
            task_cmd.orig_id = -1;
            task_cmd.dest_id = i; // Assign to the drone
            task_cmd.code = 0; // this stands for preplanned delivery
            task_cmd.yaw_control = false;
            task_cmd.data = points;
            // // wp datas are not needed now, so i'll not set them.
            task_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            this->box_publish_task(i, task_cmd);
            marker("Drone %d: [%s]", i, task_cmd.data.c_str());
        }
    } else { // dynamic assignment
        // marker("Using dynamic assignment of deliveries to drones.");
        this->marker(0, "Using dynamic assignment of deliveries to drones.");
        TaskCommand task_cmd;
        // tell drones that this is a dynamic assignment
        task_cmd.orig_id = -1;
        task_cmd.code = 1;
        task_cmd.yaw_control = false;
        task_cmd.data = "";
        for (int i = 0; i < this->total_swarm_size; i++) {
            task_cmd.dest_id = i; // Assign to the drone
            task_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            this->box_publish_task(i, task_cmd);
            // marker("Drone %d: [%s]", i, task_cmd.data.c_str());
        }
    }
}

void IGround::prep_loop() {

}

void IGround::exec_loop() {
    int completed_count = 0;

    if (!preplanned) {
        // for dynamic assignment: check if any drone has completed its task
        // then give the next delivery point [deliver_num] to the drone
        for (const auto& [i, _] : this->inf_mapper) {
            std::string data = this->inf_reports[i].data;
            if (this->drone_deliveries[i].empty() && data.empty()) { 
                // don't have to wait: just assign while they takeoff.
                marker("Assigning first delivery point to Drone %d", i);
                if (this->deliver_num < this->deliver_list.size()) {
                    int next_delivery_id = this->deliver_num++;
                    // push to drone_deliveries
                    this->drone_deliveries[i].push_back(next_delivery_id);
                    TaskCommand task_cmd;
                    task_cmd.orig_id = -1; // ground station
                    task_cmd.dest_id = i; // drone id
                    task_cmd.code = 1; // dynamic assignment
                    task_cmd.yaw_control = false;
                    task_cmd.data = std::to_string(this->deliver_list[next_delivery_id].north) + ":" + 
                                    std::to_string(this->deliver_list[next_delivery_id].east) + ":" + 
                                    std::to_string(this->deliver_list[next_delivery_id].package_id);
                    task_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                    this->box_publish_task(i, task_cmd);
                    marker("Published task command to drone %d: [%s]", 
                                i, task_cmd.data.c_str());
                }
            } else {
                completed_count = 0;
                std::stringstream ss(data);
                std::string token;
                char delimiter = ',';
                while (std::getline(ss, token, delimiter)) {
                    if (!token.empty()) {
                        int package_id = std::stoi(token);
                        // check if this package_id is already delivered, and if not, mark it as delivered AND publish new task command
                        if (package_id >= this->deliver_list.size() || package_id < 0) {
                            marker("Received package_id %d, but only %zu deliveries are available.", 
                                        package_id, this->deliver_list.size());
                            continue; // skip this package_id
                        } else if (this->deliver_list[package_id].delivered) {
                            // marker_debug("Package %d already delivered by drone %d.", package_id, i);
                            // continue; // already delivered
                        } else {
                            // new completion: assign next delivery point
                            // marker("Drone %d completed delivery of package %d.", 
                            //             i, package_id);
                            // this->marker(2, "Drone %d completed delivery of package %d.", i, package_id);
                            this->marker(2, "Drone %d completed delivery of package %d.", i, package_id);

                            // mark as delivered
                            deliver_list[package_id].delivered = true;
                            deliver_list[package_id].time_of_delivery = this->get_clock()->now().nanoseconds() / 1000;
                            deliver_list[package_id].drone_id = i;

                            // assign next delivery point
                            if (this->deliver_num < this->deliver_list.size()) {
                                int next_delivery_id = this->deliver_num++;
                                TaskCommand task_cmd;
                                task_cmd.orig_id = -1; // ground station
                                task_cmd.dest_id = i; // drone id
                                task_cmd.code = 1; // dynamic assignment
                                task_cmd.yaw_control = false;
                                task_cmd.data = std::to_string(this->deliver_list[next_delivery_id].north) + ":" + 
                                                std::to_string(this->deliver_list[next_delivery_id].east) + ":" + 
                                                std::to_string(this->deliver_list[next_delivery_id].package_id);
                                task_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                                this->box_publish_task(i, task_cmd);
                                marker("Published task command to drone %d: [%s]", 
                                            i, task_cmd.data.c_str());
                            } else {
                                marker("No more delivery points to assign to drone %d.", i);
                                // if no more delivery points, send END command
                                TaskCommand end_cmd;
                                end_cmd.orig_id = -1; // ground station
                                end_cmd.dest_id = i; // drone id
                                end_cmd.code = 1; // dynamic assignment
                                end_cmd.yaw_control = false;
                                end_cmd.data = "END"; // signal end of dynamic assignment
                                end_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                                this->box_publish_task(i, end_cmd);
                                this->marker(99, "Published END command to drone %d.", i);
                            }
                        }
                        // completed_count++;
                    }
                }
            }
        }
    } else {
        for (const auto& [i, _] : this->inf_mapper) {
            std::string data = this->inf_reports[i].data;
            std::stringstream ss(data);
            std::string token;
            char delimiter = ',';
            while (std::getline(ss, token, delimiter)) {
                if (!token.empty()) {
                    int package_id = std::stoi(token);
                    if (package_id >= this->deliver_list.size() || package_id < 0) {
                        // marker("Received package_id %d, but only %zu deliveries are available.", 
                        //             package_id, this->deliver_list.size());
                        continue; // skip this package_id
                    } else if (this->deliver_list[package_id].delivered) {
                        // marker("Package %d already delivered by drone %d.", package_id, i);
                        continue; // already delivered
                    } else {
                        this->marker(2, "Drone %d completed delivery of package %d.", i, package_id);
                        deliver_list[package_id].delivered = true;
                        deliver_list[package_id].time_of_delivery = this->get_clock()->now().nanoseconds() / 1000;
                        deliver_list[package_id].drone_id = i;
                    }
                    // completed_count++;
                }
            }
        }
    }

    // new logic: completed_count should be counted with deliver_list
    completed_count = 0;
    for (const auto& delivery : this->deliver_list) {
        if (delivery.delivered) {
            completed_count++;
        }
    }

    if (completed_count != this->last_count) {
        marker("Total completed deliveries: \t%d/%d", completed_count, 
                (int)this->deliver_list.size());
        this->last_count = completed_count;
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
