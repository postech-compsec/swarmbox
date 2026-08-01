#include "sb_base/ground.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Eigen>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

// Flocking parameters from robotsim, adapted for real-world units (m, m/s).
struct FlockingParams {
    // repulsion term
    double r_0_rep = 5.0;
    double p_rep = 0.13;

    // alignment term
    double r_0_frict = 85.3;
    double C_frict = 0.05;
    double v_frict = 0.63;
    double p_frict = 5.20;
    double a_frict = 4.16;

    // wall term
    double arena_bound = 125.0;
    double r_0_shill = 0.3;
    double v_wall = 13.6;
    double p_shill = 3.55;
    double a_shill = 3.02;

    // flock term
    double v_flock = 8.0;

    // max cutoff
    double v_max = 12.0;
};

class IGround : public Ground {
public:
    IGround(const rclcpp::NodeOptions & options);

    ~IGround() {
        std::cout << ("IGround node terminated.") << std::endl;
    }

    // void prep_once() override;

    // void prep_loop() override;
    // bool prep_complete() override;
    // void exec_once() override;
    void exec_loop() override;
    // void prep_complete() override;

private:
    Eigen::Vector3d calculate_desired_velocity(int drone_idx);
    Eigen::Vector3d repulsion(int drone_idx);
    double braking_d(double r, double a, double p);
    Eigen::Vector3d alignment(int drone_idx);
    Eigen::Vector3d interaction_with_walls(int drone_idx);
    Eigen::Vector3d self_propell(int drone_idx);

    FlockingParams params_;

};

IGround::IGround(const rclcpp::NodeOptions & options) : Ground(options) {
    marker_once("Inherited Ground (IGround) ready to send VELOCITY setpoints.");
}


void IGround::exec_loop() {
    if (inf_mapper.empty() || inf_mapper.size() != inf_pos.size() || inf_mapper.size() != inf_reports.size()) {
        return; // Wait until all data is available
    }

    for (const auto& [i, _] : this->inf_reports) {
        // Calculate the ideal velocity vector in the NE plane
        // marker("calculate for Drone ID %d", index);
        Eigen::Vector3d preferred_velocity = calculate_desired_velocity(i);

        // Create and publish the TaskCommand with velocity setpoints
        TaskCommand task_cmd;
        task_cmd.orig_id = -1;
        task_cmd.dest_id = i;
        task_cmd.code = 1; // Set code to 1 for VELOCITY command
        task_cmd.yaw_control = true;

        // Populate the velocity fields (renamed from wp_ fields in msg)
        task_cmd.wp_n = preferred_velocity.x();
        task_cmd.wp_e = preferred_velocity.y();
        task_cmd.wp_d = 0.0; // No vertical velocity command from GCS

        // Calculate a simple yaw rate to align with velocity direction
        double target_yaw = atan2(preferred_velocity.y(), preferred_velocity.x());
        task_cmd.wp_yaw = target_yaw;

        task_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        if (i < pub_task_cmds_.size()) {
            this->box_publish_task(i, task_cmd);
        }
    }
}

Eigen::Vector3d IGround::calculate_desired_velocity(int drone_idx) {
    Eigen::Vector3d v_flock = self_propell(drone_idx);
    Eigen::Vector3d v_rep   = repulsion(drone_idx);
    Eigen::Vector3d v_frict = alignment(drone_idx);
    Eigen::Vector3d v_wall  = interaction_with_walls(drone_idx);

    Eigen::Vector3d desired_vel = v_flock + v_rep + v_frict + v_wall;

    if (desired_vel.norm() > params_.v_max) {
        // cutoff
        desired_vel = desired_vel.normalized() * params_.v_max;
    }

    marker("Velocity terms: (desired %.2f) = flock(%.2f) + rep(%.2f) + frict(%.2f) + wall(%.2f)",
           desired_vel.norm(), v_flock.norm(), v_rep.norm(), v_frict.norm(), v_wall.norm());
    return desired_vel;
}

Eigen::Vector3d IGround::repulsion(int drone_idx) {
    Eigen::Vector3d r_i(inf_reports[drone_idx].pos_x, inf_reports[drone_idx].pos_y, inf_reports[drone_idx].pos_z);
    Eigen::Vector3d v_i_rep(0.0, 0.0, 0.0);
    for (int j = 0; j < inf_reports.size(); j++) {
        if (drone_idx == j) continue; // skip if i==j
        Eigen::Vector3d r_j(inf_reports[j].pos_x, inf_reports[j].pos_y, inf_reports[j].pos_z);
        Eigen::Vector3d r_ij = r_i - r_j; // r_i - r_j
        Eigen::Vector3d v_ij_rep(0.0, 0.0, 0.0);
        if (r_ij.norm() < params_.r_0_rep) {
            // Apply repulsion force
            v_ij_rep = params_.p_rep * (params_.r_0_rep - r_ij.norm()) * r_ij.normalized();
        }
        v_i_rep += v_ij_rep;
    }
    return v_i_rep;
}

double IGround::braking_d(double r, double a, double p) {
    // Calculate the braking distance based on the formula
    if (r <= 0) {
        return 0.0;
    } else if (r * p > 0 && r * p < a / p) {
        return r * p;
    } else {
        return sqrt(2 * a * r - pow(a, 2) / pow (p, 2));
    }
}

Eigen::Vector3d IGround::alignment(int drone_idx) {
    Eigen::Vector3d v_i_frict(0.0, 0.0, 0.0);
    Eigen::Vector3d v_i(inf_reports[drone_idx].vel_x, inf_reports[drone_idx].vel_y, inf_reports[drone_idx].vel_z);
    Eigen::Vector3d r_i(inf_reports[drone_idx].pos_x, inf_reports[drone_idx].pos_y, inf_reports[drone_idx].pos_z);
    for (int j = 0; j < inf_reports.size(); j++){
        if (drone_idx == j) continue; // skip if i==j
        Eigen::Vector3d v_j(inf_reports[j].vel_x, inf_reports[j].vel_y, inf_reports[j].vel_z);
        Eigen::Vector3d r_j(inf_reports[j].pos_x, inf_reports[j].pos_y, inf_reports[j].pos_z);
        
        Eigen::Vector3d v_ij = v_i - v_j;
        Eigen::Vector3d r_ij = r_i - r_j;

        double v_ij_frictmax = max(
            params_.v_frict, 
            braking_d((r_ij.norm() - params_.r_0_frict), params_.a_frict, params_.p_frict)
        );
        Eigen::Vector3d v_ij_frict(0.0, 0.0, 0.0);
        if (v_ij.norm() > v_ij_frictmax) {
            v_ij_frict = params_.C_frict * (v_ij.norm() - v_ij_frictmax) * v_ij.normalized();
        }
        v_i_frict -= v_ij_frict;
    }
    return v_i_frict;
}

Eigen::Vector3d IGround::interaction_with_walls(int drone_idx) {
    // square arena: (-100,-100)x(100,100)
    // Position p_i(inf_reports[drone_idx].pos_x, inf_reports[drone_idx].pos_y, inf_reports[drone_idx].pos_z);
    Eigen::Vector3d p_i(inf_reports[drone_idx].pos_x, inf_reports[drone_idx].pos_y, inf_reports[drone_idx].pos_z);
    Eigen::Vector3d v_i(inf_reports[drone_idx].vel_x, inf_reports[drone_idx].vel_y, inf_reports[drone_idx].vel_z);
    // 2 wall terms: NS, WE.

    int direction = 0;
    Eigen::Vector3d wall_rep_n(0.0, 0.0, 0.0);
    // find closest wall
    double dist_ns = min(abs(p_i.x() + params_.arena_bound), abs(p_i.x() - params_.arena_bound));
    double v_ns_shillmax = braking_d(dist_ns - params_.r_0_shill, params_.a_shill, params_.p_shill);
    // if north wall is closer, repulsion direction should be (-1,0,0).
    // else, repulsion direction should be (1,0,0).
    if (p_i.x() < 0) {
        direction = -1;
    } else {
        direction = 1;
    }
    Eigen::Vector3d vel_s_ns = params_.v_wall * Eigen::Vector3d(direction, 0.0, 0.0);
    Eigen::Vector3d vel_is_ns = v_i - vel_s_ns;
    if (vel_is_ns.norm() > v_ns_shillmax) {
        wall_rep_n = (vel_is_ns.norm() - v_ns_shillmax) * vel_is_ns.normalized();
    }
    
    Eigen::Vector3d wall_rep_e(0.0, 0.0, 0.0);
    // find closest wall
    double dist_ew = min(abs(p_i.y() + params_.arena_bound), abs(p_i.y() - params_.arena_bound));
    double v_ew_shillmax = braking_d(dist_ew - params_.r_0_shill, params_.a_shill, params_.p_shill);
    if (p_i.y() < 0) {
        direction = -1;
    } else {
        direction = 1;
    }
    Eigen::Vector3d vel_s_ew = params_.v_wall * Eigen::Vector3d(0.0, direction, 0.0);
    Eigen::Vector3d vel_is_ew = v_i - vel_s_ew;
    if (vel_is_ew.norm() > v_ew_shillmax) {
        wall_rep_e = (vel_is_ew.norm() - v_ew_shillmax) * vel_is_ew.normalized();
    }

    Eigen::Vector3d wall_rep = wall_rep_n + wall_rep_e;
    return wall_rep;
}

Eigen::Vector3d IGround::self_propell(int drone_idx) {
    // get current velocity
    Eigen::Vector3d v_i(inf_reports[drone_idx].vel_x, inf_reports[drone_idx].vel_y, inf_reports[drone_idx].vel_z);
    if (v_i.size() < 1.0) {
        // just give +x
        v_i = Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    return v_i.normalized() * params_.v_flock;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IGround>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}