#include "sb_base/ground.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <random>
#include <bitset>


using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

struct SocraticParams {
    double w_goal = 2.0;    
    double w_sep = 10.0;    
    double w_vel = 0.5;     
    double safe_dist = 1.5; 
    double sensing_range = 5.0; 
    int sample_count = 50;  
    double step_size = 1.0; 
};

double calculate_cost(const Eigen::Vector2d& candidate, 
                      const Eigen::Vector2d& current,
                      const Eigen::Vector2d& goal,
                      const std::vector<Eigen::Vector2d>& neighbors,
                      const SocraticParams& p) {
    double cost = 0.0;

    // 1. Goal Cost
    cost += p.w_goal * (candidate - goal).norm();

    // 2. Separation Cost
    for (const auto& n : neighbors) {
        double dist = (candidate - n).norm();
        if (dist < p.sensing_range) {
            double separation = 1.0 / (dist - 0.2 + 0.001); 
            if (dist < p.safe_dist) separation *= 10.0;
            
            cost += p.w_sep * separation;
        }
    }

    cost += p.w_vel * (candidate - current).norm();

    return cost;
}

Eigen::Vector2d optimize_position(Eigen::Vector2d curr, Eigen::Vector2d goal, 
                                  const std::vector<Eigen::Vector2d>& neighbors,
                                  const SocraticParams& params) {
    
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-params.step_size, params.step_size);

    Eigen::Vector2d best_pos = curr;
    double min_cost = std::numeric_limits<double>::max();

    double current_cost = calculate_cost(curr, curr, goal, neighbors, params);
    if (current_cost < min_cost) {
        min_cost = current_cost;
    }

    for (int i = 0; i < params.sample_count; ++i) {
        Eigen::Vector2d candidate;
        candidate.x() = curr.x() + dis(gen);
        candidate.y() = curr.y() + dis(gen);

        double cost = calculate_cost(candidate, curr, goal, neighbors, params);

        if (cost < min_cost) {
            min_cost = cost;
            best_pos = candidate;
        }
    }

    return best_pos;
}

class IGround : public Ground {
public:
    IGround(const rclcpp::NodeOptions & options);

    ~IGround() {
        std::cout << ("IGround node terminated.") << std::endl;
    }

    void prep_once() override;
    // void prep_loop() override;
    // bool prep_complete() override;
    // void exec_once() override;
    void exec_loop() override;
    // void exec_complete() override;

private:

    static const int MAX_GRID = 2500;
    std::bitset<2500> global_coverage; // 0: not covered, 1: covered(visited)
    std::bitset<2500> global_assigned; // 0: not assigned, 1: assigned
    double GRID_RES = 10.0; // each grid is 10mx10m
    static const int GRID_COLS = 50; // 500mx500m area

    std::map<int, int> drone_task_map;

    SocraticParams params_;

    int pos_to_idx(double x, double y) {
        int c = static_cast<int>((x + 250.0) / GRID_RES);
        int r = static_cast<int>((y + 250.0) / GRID_RES);
        // (Safety check)
        if (c < 0 || c >= GRID_COLS || r < 0 || r >= GRID_COLS) return -1;
        return r * GRID_COLS + c;
    }

    Eigen::Vector2d idx_to_pos(int idx) {
        int r = idx / GRID_COLS;
        int c = idx % GRID_COLS;
        
        double x = (c * GRID_RES) - 250.0 + (GRID_RES / 2.0);
        double y = (r * GRID_RES) - 250.0 + (GRID_RES / 2.0);
        return Eigen::Vector2d(x, y);
    }
};

IGround::IGround(const rclcpp::NodeOptions & options) : Ground(options) {
}

void IGround::prep_once() {
    // reset bitsets
    this->global_coverage.reset();
    this->global_assigned.reset();
}


void IGround::exec_loop() {

    int live_covered = this->global_coverage.count();
    if (this->tick % 40 == 0)  RCLCPP_INFO(this->get_logger(), "Current coverage: %d / %d", live_covered, MAX_GRID);

    // if the coverage is complete, send code 1 to all drones.
    if (live_covered >= MAX_GRID) {
        for (int droneid : this->inf_list) {
            TaskCommand task{};
            task.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            task.orig_id = -1; // ground
            task.dest_id = droneid;
            task.code = 1; // mission complete
            this->pub_task_cmds_[droneid]->publish(task);
        }
        return;
    }

    // check and update coverage
    for (auto it = drone_task_map.begin(); it != drone_task_map.end();) {
        int drone_id = it->first;
        int assigned_idx = it->second;

        Eigen::Vector3d pos = this->inf_pos[drone_id];
        Eigen::Vector2d current_xy(pos.x(), pos.y());
        Eigen::Vector2d target = idx_to_pos(assigned_idx);

        if ((current_xy - target).norm() < 3.0) {
            // reached target
            this->global_coverage.set(assigned_idx);
            marker("Explored tile %d", assigned_idx);
            it = drone_task_map.erase(it);
        } else {
            ++it;
        }
    }

    // for (int i : this->inf_reports)
    // for each report
    for (int droneid : this->inf_list) {
        // bidding

        // if already has task, skip
        if (drone_task_map.find(droneid) != drone_task_map.end()) continue;

        Eigen::Vector2d current_pos(this->inf_pos[droneid].x(), this->inf_pos[droneid].y());
        double best_score = -1.0;
        int best_idx = -1;

        for (int i = 0; i < MAX_GRID; ++i) {
            if (global_assigned.test(i) || global_coverage.test(i)) continue;

            Eigen::Vector2d tile_pos = idx_to_pos(i);
            double dist = (current_pos - tile_pos).norm();

            double score = 1.0 / (dist + 0.1); // simple inverse distance scoring

            if (score > best_score) {
                best_score = score;
                best_idx = i;
            }
        }

        if (best_idx != -1) {
            drone_task_map[droneid] = best_idx;
            global_assigned.set(best_idx);
            marker("Drone %d assigned to tile %d", droneid, best_idx);
        }
    }

    for (auto const& [droneid, tile_idx] : drone_task_map) {
        Eigen::Vector2d target = idx_to_pos(tile_idx);

        Eigen::Vector2d current_pos(this->inf_pos[droneid].x(), this->inf_pos[droneid].y());

        std::vector<Eigen::Vector2d> neighbors;
        for (auto const& [other_id, other_pos] : inf_pos) {
            if (other_id != droneid) {
                neighbors.push_back(Eigen::Vector2d(other_pos.x(), other_pos.y()));
            }
        }

        Eigen::Vector2d best_next = optimize_position(current_pos, target, neighbors, params_);

        TaskCommand task{};

        if ((best_next - current_pos).norm() > 0.1) {
            task.wp_yaw = atan2(best_next.y() - current_pos.y(), best_next.x() - current_pos.x());
        }
        task.dest_id = droneid;
        task.wp_n = best_next.x();
        task.wp_e = best_next.y();
        task.wp_d = -10.0f;
        task.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        this->pub_task_cmds_[droneid]->publish(task);
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IGround>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}