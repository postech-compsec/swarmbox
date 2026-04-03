#include "sb_base/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

struct delivery{
    float north, east;
    bool delivered;
    uint64_t time_of_delivery;
    int drone_id;
    int package_id;

    delivery(float n, float e, int pid)
        : north(n), east(e), delivered(false), time_of_delivery(0), drone_id(-1), package_id(pid) {}
};

class IDrone : public Drone {
    public:
        IDrone(const rclcpp::NodeOptions & options);
        ~IDrone();

    protected:
        // implement your own flight logic in following methods
        void                    prep_once() override;
        void                    prep_loop() override;
        bool                    prep_complete() override;
        void                    exec_once() override;
        std::optional<setpoint> exec_loop() override;
        bool                    exec_complete() override;

        std::map<int, delivery> deliver_list = {};
        int deliver_count = -1;
        bool to_home = true;
        int current_delivery_id = -1;
    float cruise_alt;
};

IDrone::IDrone(const rclcpp::NodeOptions & options) : Drone(options) {
   
    // // check result
    // marker("ID: %d, Superior: %d, Location: (%.2f, %.2f, %.2f)", 
    //             this->identity, this->sup_id, loc[0], loc[1], loc[2]);
    marker_once("Inherited Drone (IDrone) node initialized!");
    // marker_once("This is RQ4 Evaluation Drone Node #%d!", this->identity);
    this->cruise_alt = -10.0f + (-2.0f) * this->identity;
}

// ====================================
// Custom Mission Functions
// ====================================



// ====================================
// Override Functions
// ====================================
void IDrone::prep_once() {
    marker_once("IDrone prep_once called!");
    // Add any one-time preparation logic here
    marker_debug("received delivery points: %s", this->box_rcvd_task_cmd_.data.c_str());
    if (this->box_rcvd_task_cmd_.code == 0) {
        // preplanned delivery
        if (this->deliver_list.empty() && this->box_rcvd_task_cmd_.data != "") {
            marker_once("Parsing delivery points...");
            // received data looks like: n1:e1:pid1,n2:e2:pid2,n3:e3:pid3,...
            // parse the data and fill the deliver_list
            std::string data = this->box_rcvd_task_cmd_.data;
            std::stringstream ss(data);
            std::string token;
            char delimiter = ',';
    
            while (std::getline(ss, token, delimiter)) {
                size_t colon_pos1 = token.find(":");
                size_t colon_pos2 = token.rfind(":");
                if (colon_pos1 != std::string::npos && colon_pos2 != std::string::npos && colon_pos1 != colon_pos2) {
                    // change coordinate and interpret to relative position
                    float north = std::stof(token.substr(0, colon_pos1));
                    float east = std::stof(token.substr(colon_pos1 + 1, colon_pos2 - colon_pos1 - 1));
                    int package_id = std::stoi(token.substr(colon_pos2 + 1));
                    this->deliver_list.emplace(package_id, delivery(north, east, package_id));
                    marker_debug("Parsed delivery point: %d, (%.2f, %.2f, %d)",
                                this->deliver_list.size(), north, east, package_id);
                }
            }
            marker_once("Parsed %zu delivery points from task command.", this->deliver_list.size());
        }
    } else {
        // dynamic delivery assignment
        marker_once("Dynamic delivery assignment mode.");
        // now we receive no delivery points data. we'll have to deal with dynamically assigned deliveries in exec_loop.
    }
}

void IDrone::prep_loop() {
    marker_once("IDrone prep_loop called!");
}

bool IDrone::prep_complete() {
    // Add logic to determine if preparation is complete
    // check if vehicle is armed
    if (this->armed && (!this->deliver_list.empty() || this->box_rcvd_task_cmd_.code == 1)) {
        marker_once("IDrone prep stage completed!");
        return true; 
    } else {
        return false;
    }
    // return false;
}

void IDrone::exec_once() {
    // Add any one-time execution logic here
}

std::optional<setpoint> IDrone::exec_loop() {
    setpoint home = {this->init_pos.x(), this->init_pos.y(), this->cruise_alt, 0.0f}; // Default setpoint to start at (0, 0, -10)
    setpoint deliverypoint;
    this->report_data = "";
    // prepare report data
    // report completed delivery package ids as: #,#,#,#,#,...
    for (const auto& [i, delivery] : this->deliver_list) {
        if (delivery.delivered) {
            if (!this->report_data.empty()) {
                this->report_data += ",";
            }
            this->report_data += std::to_string(i);
        }
    }

    if (this->box_rcvd_task_cmd_.code == 1) {
        // get the task assigned, add to deliver_list
        marker_once("Dynamic delivery assignment mode.");
        // parse the command. this time, there will be only one delivery point "n:e:pid"
        if (this->box_rcvd_task_cmd_.data != "") {
            size_t colon_pos1 = this->box_rcvd_task_cmd_.data.find(":");
            size_t colon_pos2 = this->box_rcvd_task_cmd_.data.rfind(":");
            if (colon_pos1 != std::string::npos && colon_pos2 != std::string::npos && colon_pos1 != colon_pos2) {
                // change coordinate and interpret to relative position
                float north = std::stof(this->box_rcvd_task_cmd_.data.substr(0, colon_pos1));
                float east = std::stof(this->box_rcvd_task_cmd_.data.substr(colon_pos1 + 1, colon_pos2 - colon_pos1 - 1));
                int package_id = std::stoi(this->box_rcvd_task_cmd_.data.substr(colon_pos2 + 1));
                // check if this package_id is already in the list, and if not, add it. check the last package_id
                if (this->deliver_list.find(package_id) == this->deliver_list.end()) { // if package with pacakge_id is not in deliver_list
                    this->deliver_list.emplace(package_id, delivery(north, east, package_id)); // add to the list
                    marker("Parsed dynamic delivery point: %d, (%.2f, %.2f, %d)",
                                this->deliver_list.size(), north, east, package_id);
                    // deliverypoint.north = north;
                    // deliverypoint.east = east;
                    // deliverypoint.down = this->cruise_alt; // default down position
                    // deliverypoint.yaw = atan2(
                    //     deliverypoint.east  - this->world_pos.y(),
                    //     deliverypoint.north - this->world_pos.x()
                    // );
                    // return deliverypoint; // Return the new delivery point setpoint
                }

            } else if (this->box_rcvd_task_cmd_.data == "END") {
                // end of dynamic assignment
                marker_once("Received END command for dynamic assignment.");
                // return std::nullopt; // no setpoint change
            } else {
                marker_once("Invalid dynamic delivery command format: %s", 
                                  this->box_rcvd_task_cmd_.data.c_str());
                // return std::nullopt; // no setpoint change
            }
        } else {
            // takeoff does not have any data.
            marker_once("No dynamic delivery point assigned yet.");
        }
    }

    // takeoff: first goto (0, 0, -10)
    if (this->current_delivery_id == -1) {
        if (hasReached(home)
            ) {
            if (this->deliver_list.empty()) {
                marker_once("Reached Home, but delivery list is empty. Waiting for tasks.");
                return std::nullopt;
            }
            
            deliver_count++;
            this->to_home = false;
            for (const auto& [i, delivery] : this->deliver_list) {
                // find delivery that is not done.
                if (!delivery.delivered) {
                    this->current_delivery_id = i; // set current delivery id
                    marker_once("Reached Home. Starting delivery (%.02f, %.02f, %.02f)", 
                                    delivery.north, 
                                    delivery.east,
                                    // delivery.down
                                    this->cruise_alt
                                );
                    deliverypoint.north = delivery.north;
                    deliverypoint.east = delivery.east;
                    deliverypoint.down = this->cruise_alt; // default down position
                    deliverypoint.yaw = atan2(
                        deliverypoint.east  - this->world_pos.y(), 
                        deliverypoint.north - this->world_pos.x() 
                    );
                    return deliverypoint; // Return first delivery point setpoint
                }
            }
            // the script shouldn't reach here.
        } else {
            return home; // Default setpoint to start at (0, 0, -10)
        }
    }

    if (this->deliver_count >= (int)this->deliver_list.size()) {
        marker_once("All deliveries seem to be completed. Staying at home.");
        return home;
    }

    if (this->current_delivery_id >= 0 && !this->deliver_list.empty()) {
        // if there's a current delivery
        auto it = this->deliver_list.find(this->current_delivery_id);
        if (it == this->deliver_list.end()) {
            marker("Current delivery ID %d not found in delivery list.", this->current_delivery_id);
            return std::nullopt;
        }
        // delivery item = it->second;
        if (it->second.delivered && hasReached(home)) {
            // if this package is delivered and drone returned to home
            it->second.time_of_delivery = this->get_clock()->now().nanoseconds() / 1000;
            it->second.drone_id = this->identity;
            marker("The drone has come back home from delivery %d", it->second.package_id);
            // increment deliver_count?
            if (this->deliver_count >= (int)this->deliver_list.size()) {
                this->to_home = true; // set to home
                return home; // Return home setpoint
            } else {
                this->deliver_count++;
                marker("Delivered package #%d/%d: (%.02f, %.02f) and returned", 
                this->deliver_count, (int)(this->deliver_list.size()), 
                it->second.north, 
                it->second.east);
                this->to_home = false;
                // find the next target from map
                for (const auto& [i, delivery] : this->deliver_list) {
                    if (!delivery.delivered) {
                        // if there's any not completed delivery, go for it.
                        this->current_delivery_id = i; // set current delivery id
                        deliverypoint.north = delivery.north;
                        deliverypoint.east = delivery.east;
                        deliverypoint.down = this->cruise_alt; // default down position
                        deliverypoint.yaw = atan2(
                            deliverypoint.east  - this->world_pos.y(), 
                            deliverypoint.north - this->world_pos.x()
                        );
                        return deliverypoint; // Return next delivery point setpoint
                    }
                }
                return std::nullopt; // no more deliveries: will be cared in next iteration.
            }
        } else if (!this->to_home && !(it->second.delivered) 
            && hasReached(setpoint{it->second.north, it->second.east, this->cruise_alt, 0.0f })) {
            // if drone reached the delivery point
            it->second.delivered = true;
            this->marker(101, "Reached delivery point #%d: (%.02f, %.02f)", 
                it->second.package_id,
                it->second.north,
                it->second.east);
            // if drone is near the delivery point, return to home
            this->to_home = true; // set to home
            home.yaw = atan2(
                home.east  - this->world_pos.y(), 
                home.north - this->world_pos.x()
            );
            return home;
        }
    } 

    return std::nullopt; // if there's no setpoint change, we don't have to return any setpoint.
}

bool IDrone::exec_complete() {
    if ((this->box_rcvd_task_cmd_.code == 0 && this->deliver_count == (int)this->deliver_list.size())) { // completion for preplanned delivery
        marker_once("All deliveries are done.");
        return true; // Execution is complete
    } else if (this->box_rcvd_task_cmd_.code == 1 && this->box_rcvd_task_cmd_.data == "END"){ // completion for dynamic assignment){
        // check if home
        if (hasReached(setpoint{this->init_pos.x(), this->init_pos.y(), this->cruise_alt, 0.0f})
        ) {
            marker_once("All dynamic deliveries are done.");
            return true; // Execution is complete
        } else {
            return false; // Execution is not complete
        }
    } else {
        return false;
    }
    // return false;
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