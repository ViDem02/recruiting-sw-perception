#include <nlopt.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

/*
 Transliteration of pathplanning/racingline.py using NLOPT.
 
 We minimize curvature^2 + 0.1 * sum (d[i+1]-d[i])^2 subject 
 to -half_width[i] <= d[i] <= half_width[i].
*/

struct TrackData {
    std::vector<std::vector<double>> left {{0,0},{0,3},{2,5},{3,5}};
    std::vector<std::vector<double>> right{{0,0},{1,2},{3,4},{4,4}};
    int N = left.size();
    std::vector<std::vector<double>> mid; // midpoint
    std::vector<double> half_width;
    std::vector<std::vector<double>> tangent; // unit tangent
    std::vector<std::vector<double>> normal;  // normal (rotated tangent)

    TrackData() {
        mid.resize(N, std::vector<double>(2));
        half_width.resize(N);
        for (int i=0;i<N;++i) {
            mid[i][0] = 0.5*(left[i][0] + right[i][0]);
            mid[i][1] = 0.5*(left[i][1] + right[i][1]);
            double dx = right[i][0] - left[i][0];
            double dy = right[i][1] - left[i][1];
            half_width[i] = 0.5 * std::sqrt(dx*dx + dy*dy);
        }
        tangent.resize(N, std::vector<double>(2,0.0));
        // central differences for interior, forward/backward at ends
        for (int i=1;i<N-1;++i) {
            tangent[i][0] = mid[i+1][0] - mid[i-1][0];
            tangent[i][1] = mid[i+1][1] - mid[i-1][1];
        }
        tangent[0][0] = mid[1][0] - mid[0][0];
        tangent[0][1] = mid[1][1] - mid[0][1];
        tangent[N-1][0] = mid[N-1][0] - mid[N-2][0];
        tangent[N-1][1] = mid[N-1][1] - mid[N-2][1];
        normal.resize(N, std::vector<double>(2,0.0));
        for (int i=0;i<N;++i) {
            double norm = std::sqrt(tangent[i][0]*tangent[i][0] + tangent[i][1]*tangent[i][1]);
            if (norm < 1e-12) norm = 1e-12;
            tangent[i][0] /= norm;
            tangent[i][1] /= norm;
            // rotate 90 degrees ccw
            normal[i][0] = -tangent[i][1];
            normal[i][1] = tangent[i][0];
        }
    }
};



// Objective callback for NLOPT C API
// x: vector d[0..N-1]
// returns curvature cost + smoothness cost.
static double objective_func(unsigned n, const double* x, double* grad, void* data) {
    TrackData* td = static_cast<TrackData*>(data);
    int N = td->N;
    const auto& mid = td->mid;
    const auto& normal = td->normal;

    // Precompute racing line points
    std::vector<std::vector<double>> pts(N, std::vector<double>(2));
    for (int i=0;i<N;++i) {
        pts[i][0] = mid[i][0] + x[i]*normal[i][0];
        pts[i][1] = mid[i][1] + x[i]*normal[i][1];
    }
    double cost = 0.0;
    // curvature squared
    for (int i=1;i<N-1;++i) {
        double dx1 = pts[i][0] - pts[i-1][0];
        double dy1 = pts[i][1] - pts[i-1][1];
        double dx2 = pts[i+1][0] - pts[i][0];
        double dy2 = pts[i+1][1] - pts[i][1];
        double denom = std::pow(dx1*dx1 + dy1*dy1 + 1e-6, 1.5);
        double curvature = (dx1*dy2 - dy1*dx2) / denom;
        cost += curvature * curvature;
    }
    // smoothness term
    for (int i=0;i<N-1;++i) {
        double diff = x[i+1]-x[i];
        cost += 0.1 * diff * diff;
    }
    // No analytical gradient provided; derivative-free algorithm ignores this.
    if (grad) {
        for (unsigned i = 0; i < n; ++i) grad[i] = 0.0;
    }
    return cost;
}

int main() {
    TrackData td;
    int N = td.N;

    // Choose an algorithm: LN_BOBYQA (derivative-free, bound-constrained)
    nlopt_opt opt = nlopt_create(NLOPT_LN_BOBYQA, N);

    // Bounds
    std::vector<double> lb(N), ub(N);
    for (int i=0;i<N;++i) {
        lb[i] = -td.half_width[i];
        ub[i] =  td.half_width[i];
    }
    nlopt_set_lower_bounds(opt, lb.data());
    nlopt_set_upper_bounds(opt, ub.data());

    nlopt_set_min_objective(opt, objective_func, &td);
    nlopt_set_xtol_rel(opt, 1e-6);
    nlopt_set_maxeval(opt, 1000);

    std::vector<double> x(N, 0.0); // initial guess centered
    double minf;
    nlopt_result result = nlopt_optimize(opt, x.data(), &minf);
    if (result < 0) {
        std::cerr << "Optimization failed with code: " << result << std::endl;
        nlopt_destroy(opt);
        return 1;
    }

    std::cout << "Status: " << result << "\nMinimum cost: " << minf << "\nOptimal d:";
    for (double v : x) std::cout << " " << v;
    std::cout << "\nRacing line (x,y):\n";
    for (int i=0;i<N;++i) {
        double X = td.mid[i][0] + x[i]*td.normal[i][0];
        double Y = td.mid[i][1] + x[i]*td.normal[i][1];
        std::cout << X << ", " << Y << "\n";
    }
    nlopt_destroy(opt);
    return 0;
}
