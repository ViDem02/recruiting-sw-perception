#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

// Helper to wrap angle to [-pi, pi]
static double wrapToPi(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

int main() {
    using Eigen::MatrixXd; using Eigen::Vector2d; using std::cout; using std::endl;

    // Cones matrix (Nx2)
    MatrixXd Cones(21, 2);
    Cones <<
        3.883, -3.466,
        3.774, -2.478,
        3.693, -1.193,
        3.436, -0.084,
        2.922,  1.080,
        2.259,  2.067,
        1.975, -3.574,
        1.542, -2.411,
        1.502, -1.856,
        1.245, -1.261,
        0.974, -0.814,
        0.257, -0.138,
       -1.163,  0.484,
        1.488,  2.649,
        0.122,  3.596,
       -1.055,  3.920,
        0.825,  3.014,
        2.043,  2.297,
       -0.257,  0.214,
       -1.583,  0.660,
       -1.975,  0.836;

    // Parameters
    Vector2d robot_pos(2.8, -6.0);
    double robot_heading = M_PI/2.0; // facing upward
    const double max_angle = M_PI/2.0; // unused (we use 150deg)
    const int step_limit = 200;

    // Output paths (store points)
    std::vector<Vector2d> path_center; path_center.push_back(robot_pos);
    std::vector<Vector2d> path_left;   
    std::vector<Vector2d> path_right;  

    for (int step = 1; step <= step_limit; ++step) {
        if (Cones.rows() == 0) break;

        // relative vectors
        MatrixXd rel = Cones.rowwise() - robot_pos.transpose(); // Nx2
        Eigen::VectorXd dists = rel.rowwise().norm();

        // angles from robot heading
        Eigen::VectorXd angles(Cones.rows());
        for (int i=0;i<Cones.rows();++i) {
            double a = std::atan2(rel(i,1), rel(i,0)) - robot_heading;
            angles(i) = wrapToPi(a);
        }

        // Cones within +/-150deg
        std::vector<int> visible_idx;
        visible_idx.reserve(Cones.rows());
        for (int i=0;i<angles.size();++i) {
            if (std::abs(angles(i)) < (150.0*M_PI/180.0)) visible_idx.push_back(i);
        }
        if (visible_idx.empty()) break;

        // Split to left and right candidates
        std::vector<int> left_candidates, right_candidates;
        for (int idx : visible_idx) {
            if (angles(idx) > 0) left_candidates.push_back(idx);
            else if (angles(idx) < 0) right_candidates.push_back(idx);
        }
        if (left_candidates.empty() || right_candidates.empty()) break;

        double best_score = std::numeric_limits<double>::infinity();
        int best_L = -1, best_R = -1; Vector2d best_mid(NAN, NAN);

        for (int Li : left_candidates) {
            for (int Ri : right_candidates) {
                Vector2d L = Cones.row(Li);
                Vector2d R = Cones.row(Ri);
                Vector2d mid = 0.5*(L+R);
                double width = (L-R).norm();
                double dist_mid = (mid - robot_pos).norm();

                if (width < 0.2 || width > 3.0) continue;

                double heading_to_mid = std::atan2(mid.y()-robot_pos.y(), mid.x()-robot_pos.x());
                double dtheta = std::abs(wrapToPi(heading_to_mid - robot_heading));

                double corridor_dir = std::atan2((L-R).y(), (L-R).x()) + M_PI/2.0;
                double d_corridor = std::abs(wrapToPi(corridor_dir - robot_heading));

                double heading_penalty = 0.5 * dtheta + 0.5 * d_corridor;
                double dist_penalty = std::abs(dist_mid - 0.5);

                double score = 1.0*dist_penalty + 1.0*heading_penalty + 0.3*std::abs(width - 1.0);

                if (score < best_score) {
                    best_score = score; best_L = Li; best_R = Ri; best_mid = mid;
                }
            }
        }

        if (best_L < 0 || best_R < 0) {
            std::cout << "No valid corridor found — stopping." << std::endl;
            break;
        }

        Vector2d left_cone = Cones.row(best_L);
        Vector2d right_cone = Cones.row(best_R);
        Vector2d mid = best_mid;

        path_left.push_back(left_cone);
        path_right.push_back(right_cone);
        path_center.push_back(mid);

        double desired_heading = std::atan2(mid.y()-robot_pos.y(), mid.x()-robot_pos.x());
        double delta_heading = wrapToPi(desired_heading - robot_heading);
        robot_heading = robot_heading + 0.6 * delta_heading;

        double turn_factor = 1 - std::min(std::abs(delta_heading) / M_PI, 1.0);
        double step_size = 0.5 * turn_factor + 0.15; // 0.65 straight → 0.15 tight turn

        robot_pos += step_size * Vector2d(std::cos(robot_heading), std::sin(robot_heading));

        // Remove nearby cones (< 0.25)
        std::vector<int> keep;
        keep.reserve(Cones.rows());
        for (int i=0;i<Cones.rows();++i) {
            Vector2d diff = Cones.row(i).transpose() - robot_pos;
            if (diff.norm() >= 0.25) keep.push_back(i);
        }
        MatrixXd newCones(keep.size(), 2);
        for (size_t i=0;i<keep.size();++i) newCones.row(i) = Cones.row(keep[i]);
        Cones = newCones;
    }

    // Print results
    auto print_vec = [](const std::string& name, const std::vector<Vector2d>& v){
        std::cout << name << " (" << v.size() << "):\n";
        for (const auto& p : v) std::cout << p.x() << ", " << p.y() << "\n";
        std::cout << std::endl;
    };

    std::cout << "\u2705 Simulation complete." << std::endl;
    print_vec("Left path", path_left);
    print_vec("Right path", path_right);
    print_vec("Centerline", path_center);

    return 0;
}
