#include "sb_base/ground.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <random>

using namespace sb_base::msg;
using namespace std;
using namespace std::chrono_literals;

struct RRTNode {
    Eigen::Vector2d p = Eigen::Vector2d::Zero();
    int i   = 0;
    int iprev = 0;
};

struct RRTParams {
    double minDistGoal = 5.0;    
    double extension = 10.0;     
    double goal_prob = 0.05;     
    
    
    double world_bounds_x[2] = {-400.0, 150.0}; 
    double world_bounds_y[2] = {-400.0, 150.0}; 
    
    bool animate_rrt = false;
    double takeoffheight = 10.0; 
    
    
    double drone_vel = 10.0;      
    double influence_radius = 30.0; 
    double max_sp_dist = 5.0;    
};

bool isPointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) {
    bool inside = false;
    int n = polygon.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
            (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) / 
            (polygon[j].y() - polygon[i].y()) + polygon[i].x())) {
            inside = !inside;
        }
    }
    return inside;
}

bool isCollisionFreeVertex(const std::vector<std::vector<Eigen::Vector2d>>& obstacles, const Eigen::Vector2d& xy) {
    for (const auto& obstacle : obstacles) {
        if (isPointInPolygon(xy, obstacle)) return false;
    }
    return true;
}

bool isCollisionFreeEdge(const std::vector<std::vector<Eigen::Vector2d>>& obstacles, const Eigen::Vector2d& closest_vert, const Eigen::Vector2d& xy) {
    double l = (closest_vert - xy).norm();
    double map_resolution = 0.01;
    
    // M = int(l / map_resolution)
    int M = static_cast<int>(l / map_resolution);
    if (M <= 2) M = 20;

    // Linearly interpolate M points and check collision
    for (int i = 1; i < M; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(M - 1); 
        Eigen::Vector2d p = (1.0 - t) * closest_vert + t * xy;
        
        if (!isCollisionFreeVertex(obstacles, p)) return false;
    }
    return true;
}

std::vector<Eigen::Vector2d> rrt_path(
    const std::vector<std::vector<Eigen::Vector2d>>& obstacles, 
    Eigen::Vector2d xy_start, 
    Eigen::Vector2d xy_goal, 
    RRTParams params) {    
    
    std::vector<RRTNode> rrt;
    rrt.reserve(5000);

    RRTNode start_node;
    start_node.p = xy_start;
    start_node.i = 0;
    start_node.iprev = 0;
    rrt.push_back(start_node);

    bool near_goal = false;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    std::cout << "Configuring space sampling started ..." << std::endl;

    int iters = 0;

    while (!near_goal) {
        Eigen::Vector2d xy_sample;
        double rnd = dis(gen);
        if (rnd < params.goal_prob) {
            xy_sample = xy_goal;
        } else {
            double r1 = dis(gen);
            double r2 = dis(gen);
            double width_x = params.world_bounds_x[1] - params.world_bounds_x[0];
            double width_y = params.world_bounds_y[1] - params.world_bounds_y[0];
            xy_sample = Eigen::Vector2d(params.world_bounds_x[0] + r1 * width_x,
                                        params.world_bounds_y[0] + r2 * width_y);
        }

        if (!isCollisionFreeVertex(obstacles, xy_sample)) {
            iters++;
            continue;
        }

        int closest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < rrt.size(); ++i) {
            double d = (rrt[i].p - xy_sample).norm();
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }


        if (closest_idx == -1) continue;
        const RRTNode& closest_node = rrt[closest_idx];

        Eigen::Vector2d direction = xy_sample - closest_node.p;
        double dist_to_sample = direction.norm();

        if (dist_to_sample <= 1e-3) continue;

        Eigen::Vector2d new_pos = closest_node.p + params.extension * direction.normalized();

        RRTNode new_node;
        new_node.p = new_pos;
        new_node.i = rrt.size();
        new_node.iprev = closest_idx;

        if (!isCollisionFreeEdge(obstacles, closest_node.p, new_pos)) {
            iters++;
            continue;
        }

        rrt.push_back(new_node);

        if ((xy_goal - new_node.p).norm() < params.minDistGoal) {
            RRTNode goal_node;
            goal_node.p = xy_goal;
            goal_node.i = rrt.size();
            goal_node.iprev = new_node.i;

            if (isCollisionFreeEdge(obstacles, new_node.p, xy_goal)) {
                rrt.push_back(goal_node);
                near_goal = true;
            }
        }

        iters++;
    }

    std::vector<Eigen::Vector2d> path;

    int curr_idx = rrt.size() - 1; // index of goal node
    while (true) {
        path.push_back(rrt[curr_idx].p);
        if (curr_idx == 0) break; // reached start node
        if (rrt[curr_idx].i == 0) break; // safety check

        curr_idx = rrt[curr_idx].iprev;
    }

    std::reverse(path.begin(), path.end());

    return path;
};


std::vector<Eigen::Vector2d> ShortenPath(std::vector<Eigen::Vector2d> P, 
                                         const std::vector<std::vector<Eigen::Vector2d>>& obstacles, 
                                         int smoothiters = 10) 
{
    // Random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    
    int m = P.size();
    std::vector<double> l(m, 0.0);

    // Initial cumulative length calculation
    for (int k = 1; k < m; ++k) {
        l[k] = (P[k] - P[k - 1]).norm() + l[k - 1];
    }

    int iters = 0;
    while (iters < smoothiters) {
        // Dynamic distribution based on current total length
        std::uniform_real_distribution<> dis(0.0, l[m - 1]);
        
        double s1 = dis(gen);
        double s2 = dis(gen);

        if (s2 < s1) std::swap(s1, s2);

        // Find segment indices i and j
        int i = 0;
        for (int k = 1; k < m; ++k) {
            if (s1 < l[k]) {
                i = k - 1;
                break;
            }
        }

        int j = 0;
        for (int k = i; k < m; ++k) {
            if (s2 < l[k]) {
                j = k - 1;
                break;
            }
        }

        if (j <= i) {
            iters++;
            continue;
        }

        // Interpolate points gamma1 and gamma2
        double t1 = (s1 - l[i]) / (l[i + 1] - l[i]);
        Eigen::Vector2d gamma1 = (1.0 - t1) * P[i] + t1 * P[i + 1];

        double t2 = (s2 - l[j]) / (l[j + 1] - l[j]);
        Eigen::Vector2d gamma2 = (1.0 - t2) * P[j] + t2 * P[j + 1];

        // Check collision for shortcut
        if (!isCollisionFreeEdge(obstacles, gamma1, gamma2)) {
            iters++;
            continue;
        }

        // Construct new path: P[:i+1] + gamma1 + gamma2 + P[j+1:]
        std::vector<Eigen::Vector2d> new_P;
        
        // 1. Add P[0] ... P[i]
        for(int k = 0; k <= i; ++k) new_P.push_back(P[k]);
        
        // 2. Add gamma1, gamma2
        new_P.push_back(gamma1);
        new_P.push_back(gamma2);
        
        // 3. Add P[j+1] ... P[end]
        for(int k = j + 1; k < m; ++k) new_P.push_back(P[k]);

        // Update P
        P = new_P;
        m = P.size();
        
        // Recalculate lengths
        l.assign(m, 0.0);
        for (int k = 1; k < m; ++k) {
            l[k] = (P[k] - P[k - 1]).norm() + l[k - 1];
        }

        iters++;
    }

    return P;
}

std::vector<Eigen::Vector2d> waypts2setpts(std::vector<Eigen::Vector2d> Path, RRTParams params) {
    double V = params.drone_vel * 1.3;
    double freq = 1;
    double dt = 1.0 / freq;
    double dx = V * dt;
	std::vector<Eigen::Vector2d> traj_global = {Path.front()};
    for (size_t i = 0; i < Path.size() - 1; ++i) {
        Eigen::Vector2d A = Path[i];
        Eigen::Vector2d B = Path[i + 1];
        
        Eigen::Vector2d diff = B - A;
        if (diff.norm() < 1e-6) continue;

        Eigen::Vector2d n = diff.normalized();
        Eigen::Vector2d delta = n * dx;
        
        int N = static_cast<int>(diff.norm() / delta.norm());
        
        Eigen::Vector2d sp = A;
        
        for (int j = 0; j < N; ++j) {
            sp += delta;
            traj_global.push_back(sp);
        }
        
        sp = B;
        traj_global.push_back(sp);
    }
    return traj_global;
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
    std::vector<Eigen::Vector2d> Path;

    std::vector<std::vector<Eigen::Vector2d>> obstacles;
    
    Eigen::Vector2d xy_start = Eigen::Vector2d(0.0, 0.0);
    Eigen::Vector2d xy_goal  = Eigen::Vector2d(30.0, -240.0);
    
    RRTParams params;
    
    int sp_idx = 0;
    std::vector<setpoint> route = {};
    double t_goal = -1;
    std::vector<Eigen::Vector2d> traj_global;

};

IGround::IGround(const rclcpp::NodeOptions & options) : Ground(options) {
}

void IGround::prep_once() {
    marker_once("Inherited Ground (IGround) node initialized!");
    // Eigen::Vector2d xy_start(0.0, 0.0);
    // Eigen::Vector2d xy_goal(2.0, 2.0);

    double scale = 100.0;
    // double passage_width = 0.3 * scale; 
    // double passage_location = 0.0 * scale;

    // this->obstacles.push_back({
    //     Eigen::Vector2d(-2.5 * scale + 150, -0.3 * scale + 150),
    //     Eigen::Vector2d((-passage_location - passage_width/2.0) + 150, -0.3 * scale + 150),
    //     Eigen::Vector2d((-passage_location - passage_width/2.0) + 150, 0.3 * scale + 150), 
    //     Eigen::Vector2d(-2.5 * scale + 150, 0.3 * scale + 150)
    // });

    // this->obstacles.push_back({
    //     Eigen::Vector2d((-passage_location + passage_width/2.0) + 150, -0.3 * scale + 150),
    //     Eigen::Vector2d(2.5 * scale + 150, -0.3 * scale + 150),
    //     Eigen::Vector2d(2.5 * scale + 150, 0.3 * scale + 150),
    //     Eigen::Vector2d((-passage_location + passage_width/2.0) + 150, 0.3 * scale + 150)  
    // });

    // bugtrap
    this->obstacles.push_back({
        Eigen::Vector2d( 0.5  * scale - 120, 0 * scale - 100),
        Eigen::Vector2d( 2.5  * scale - 120, 0 * scale - 100),
        Eigen::Vector2d( 2.5  * scale - 120, 0.3 * scale - 100),
        Eigen::Vector2d( 0.5  * scale - 120, 0.3 * scale - 100)
    });
    this->obstacles.push_back({
        Eigen::Vector2d( 0.5  * scale - 120, 0.3 * scale - 100),
        Eigen::Vector2d( 0.8  * scale - 120, 0.3 * scale - 100),
        Eigen::Vector2d( 0.8  * scale - 120, 1.5 * scale - 100),
        Eigen::Vector2d( 0.5  * scale - 120, 1.5 * scale - 100)
    });
    this->obstacles.push_back({
        Eigen::Vector2d( 0.5  * scale - 120, 1.5 * scale - 100),
        Eigen::Vector2d( 1.5  * scale - 120, 1.5 * scale - 100),
        Eigen::Vector2d( 1.5  * scale - 120, 1.8 * scale - 100),
        Eigen::Vector2d( 0.5  * scale - 120, 1.8 * scale - 100)
    });
    // angle
    this->obstacles.push_back({
        Eigen::Vector2d(-2    * scale - 120, -2 * scale - 100),
        Eigen::Vector2d(-0.5  * scale - 120, -2 * scale - 100),
        Eigen::Vector2d(-0.5  * scale - 120, -1.8 * scale - 100),
        Eigen::Vector2d(-2    * scale - 120, -1.8 * scale - 100)
    });
    this->obstacles.push_back({
        Eigen::Vector2d(-0.7  * scale - 120, -1.8 * scale - 100),
        Eigen::Vector2d(-0.5  * scale - 120, -1.8 * scale - 100),
        Eigen::Vector2d(-0.5  * scale - 120, -0.8 * scale - 100),
        Eigen::Vector2d(-0.7  * scale - 120, -0.8 * scale - 100)
    });
    // walls
    this->obstacles.push_back({
        Eigen::Vector2d(-2.5  * scale - 120, -2.5 * scale - 100),
        Eigen::Vector2d( 2.5  * scale - 120, -2.5 * scale - 100),
        Eigen::Vector2d( 2.5  * scale - 120, -2.47 * scale - 100),
        Eigen::Vector2d(-2.5  * scale - 120, -2.47 * scale - 100)
    });
    this->obstacles.push_back({
        Eigen::Vector2d(-2.5  * scale - 120, 2.47 * scale - 100),
        Eigen::Vector2d( 2.5  * scale - 120, 2.47 * scale - 100),
        Eigen::Vector2d( 2.5  * scale - 120, 2.5 * scale - 100),
        Eigen::Vector2d(-2.5  * scale - 120, 2.5 * scale - 100)
    });
    this->obstacles.push_back({
        Eigen::Vector2d(-2.5  * scale - 120, -2.47 * scale - 100),
        Eigen::Vector2d(-2.47 * scale - 120, -2.47 * scale - 100),
        Eigen::Vector2d(-2.47 * scale - 120, 2.47 * scale - 100),
        Eigen::Vector2d(-2.5  * scale - 120, 2.47 * scale - 100)
    });
    this->obstacles.push_back({
        Eigen::Vector2d( 2.47 * scale - 120, -2.47 * scale - 100),
        Eigen::Vector2d( 2.5  * scale - 120, -2.47 * scale - 100),
        Eigen::Vector2d( 2.5  * scale - 120, 2.47 * scale - 100),
        Eigen::Vector2d( 2.47 * scale - 120, 2.47 * scale - 100)
    });

    for (auto& poly : this->obstacles) {
        std::cout << "Obstacle:" << std::endl;
        for (const auto& p : poly) {
            std::cout << "(" << p.x() << ", " << p.y() << ")" << std::endl;
        }
    }

    this->Path = rrt_path(this->obstacles, this->xy_start, this->xy_goal, params);
    this->Path = ShortenPath(this->Path, this->obstacles, 30);

    traj_global = waypts2setpts(Path, params);

    for (size_t i = 0; i < traj_global.size(); ++i) {
        // print traj_global
        std::cout << "Setpoint " << i << ": (" << traj_global[i].x() << ", " << traj_global[i].y() << ")" << std::endl;
    }

    // add {traj_global[0].x(), traj_global[0].y(), -params.takeoffheight, 0.0, 0.0} setpoint to the route
    setpoint sp_takeoff;
    sp_takeoff.north = traj_global[0].x();
    sp_takeoff.east = traj_global[0].y();
    sp_takeoff.down = -params.takeoffheight;
    sp_takeoff.yaw = 0.0;
    route.push_back(sp_takeoff);
}


void IGround::exec_loop() {
    auto leader_at = this->inf_pos[0];
    Eigen::Vector2d leader_xy(leader_at.x(), leader_at.y());

    Eigen::Vector2d final_goal = this->traj_global.back();
    
    // check if reached final goal
    // -> this is done on each agent, so don't care here.
    // double dist_to_goal = (final_goal - leader_xy).norm();
    // if (dist_to_goal < params.goal_tolerance) {
    //     if (t_goal < 0) {
    //         // goal reached
    //         t_goal = this->tick;
    //         RCLCPP_INFO(this->get_logger(), "Final goal reached at tick %d", t_goal);
    //     }
    // }

    if (sp_idx >= this->traj_global.size()) sp_idx = this->traj_global.size() - 1;
    Eigen::Vector2d global_sp = this->traj_global[sp_idx];

    Eigen::Vector2d force_att = global_sp - leader_xy;
    if (force_att.norm() > 1.0) force_att.normalize();

    Eigen::Vector2d force_rep(0.0, 0.0);

    for (const auto& obs_poly : this->obstacles) {
        
        Eigen::Vector2d obs_center(0,0); 
        for(auto& p : obs_poly) obs_center += p;
        obs_center /= (double)obs_poly.size();

        Eigen::Vector2d diff = leader_xy - obs_center;
        double dist = diff.norm();
        double radius = 0.5;
        
        if (dist < params.influence_radius + radius) {
             double rep_strength = 1.0 / (dist - radius + 0.01); 
             force_rep += diff.normalized() * rep_strength * 0.5;
        }
    }

    Eigen::Vector2d next_pos = leader_xy + (force_att + force_rep);

    TaskCommand cmd;
    cmd.dest_id = 0; // leader only.
    cmd.wp_type = 0;
    cmd.wp_n = next_pos.x();
    cmd.wp_e  = next_pos.y();
    cmd.wp_d  = -params.takeoffheight;
    cmd.wp_yaw   = 0.0;
    // send code 1 if the command is the final goal
    cmd.code = sp_idx >= this->traj_global.size() - 1 ? 1 : 0;
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    this->pub_task_cmds_[0]->publish(cmd);

    if (sp_idx < this->traj_global.size() - 1) {
        if ((leader_xy - global_sp).norm() < params.max_sp_dist) {
            sp_idx++;
            marker("Advancing to next setpoint: %d [%.2f, %.2f]", sp_idx, global_sp.x(), global_sp.y());
        }
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IGround>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}