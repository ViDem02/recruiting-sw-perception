#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <numeric>

namespace cones{

struct Result {
    pcl::PointCloud<pcl::PointXYZ>::Ptr left;
    pcl::PointCloud<pcl::PointXYZ>::Ptr right;
};

static inline double wrapToPi(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

static double deg(double rad) { return rad * 180.0 / M_PI; }

// Compute signed turn angle between prev->curr and curr->next (degrees)
static double turn_angle_deg(const pcl::PointXYZ& prev, const pcl::PointXYZ& curr, const pcl::PointXYZ& next) {
    const double v1x = curr.x - prev.x; const double v1y = curr.y - prev.y;
    const double v2x = next.x - curr.x; const double v2y = next.y - curr.y;
    double a1 = std::atan2(v1y, v1x);
    double a2 = std::atan2(v2y, v2x);
    return deg(wrapToPi(a2 - a1));
}

Result distinguishLeftRight(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& src_cones,
    double robot_x, double robot_y,
    double angle_low_deg = -45.0, double angle_high_deg = 45.0)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cones(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < src_cones->size(); ++i)
    {
        pcl::PointXYZ p;
        p.x = src_cones->points[i].z;
        p.z = src_cones->points[i].x;
        p.y = 0;
        cones->push_back(p);
    }

    Result res{pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(),
               pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};

    const int N = static_cast<int>(cones->size());
    if (N < 2) return res;

    // pick two closest cones to robot
    std::vector<int> idx(N);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b){
        const auto &pa = cones->points[a];
        const auto &pb = cones->points[b];
        double da = (pa.x-robot_x)*(pa.x-robot_x) + (pa.y-robot_y)*(pa.y-robot_y);
        double db = (pb.x-robot_x)*(pb.x-robot_x) + (pb.y-robot_y)*(pb.y-robot_y);
        return da < db;
    });
    int first_idx = idx[0];
    int second_idx = idx[1];

    std::vector<char> remaining(N, 1);
    remaining[first_idx] = remaining[second_idx] = 0;

    std::vector<int> path_l_idx{first_idx};
    std::vector<int> path_r_idx{second_idx};

    auto get_point = [&](int i)->const pcl::PointXYZ& { return cones->points[i]; };

    for (int step = 0; step < N-2; ++step) {
        std::vector<int> rem_idx;
        for (int i=0;i<N;++i) if (remaining[i]) rem_idx.push_back(i);
        if (rem_idx.empty()) break;

        // odd case: one cone left -> assign to side with least angle
        if (rem_idx.size() == 1) {
            int last = rem_idx[0];
            double dL = 0.0, dR = 0.0;
            if (path_l_idx.size() >= 2) {
                dL = turn_angle_deg(get_point(path_l_idx[path_l_idx.size()-2]),
                                    get_point(path_l_idx.back()),
                                    get_point(last));
            }
            if (path_r_idx.size() >= 2) {
                dR = turn_angle_deg(get_point(path_r_idx[path_r_idx.size()-2]),
                                    get_point(path_r_idx.back()),
                                    get_point(last));
            }
            if (std::abs(dL) <= std::abs(dR)) path_l_idx.push_back(last);
            else path_r_idx.push_back(last);
            remaining[last] = 0;
            break;
        }

        // compute nearest candidates to current L/R
        auto nearest_to = [&](int current)->int{
            int best = -1; double bestd = std::numeric_limits<double>::infinity();
            const auto &pc = get_point(current);
            for (int j : rem_idx) {
                const auto &p = get_point(j);
                double d = (p.x-pc.x)*(p.x-pc.x)+(p.y-pc.y)*(p.y-pc.y);
                if (d < bestd) { bestd = d; best = j; }
            }
            return best;
        };

        int candL = nearest_to(path_l_idx.back());
        int candR = nearest_to(path_r_idx.back());

        // if the same candidate, give to the closer one and pick 2nd for the other
        if (candL == candR) {
            auto distance_sq = [&](int from, int to){
                const auto &a = get_point(from); const auto &b = get_point(to);
                return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
            };
            double dl = distance_sq(path_l_idx.back(), candL);
            double dr = distance_sq(path_r_idx.back(), candR);
            if (dl < dr) {
                // find second best for R
                int second = -1; double bestd = std::numeric_limits<double>::infinity();
                const auto &pr = get_point(path_r_idx.back());
                for (int j : rem_idx) if (j != candL) {
                    const auto &p = get_point(j);
                    double d = (p.x-pr.x)*(p.x-pr.x)+(p.y-pr.y)*(p.y-pr.y);
                    if (d < bestd) { bestd = d; second = j; }
                }
                candR = second;
            } else {
                int second = -1; double bestd = std::numeric_limits<double>::infinity();
                const auto &pl = get_point(path_l_idx.back());
                for (int j : rem_idx) if (j != candR) {
                    const auto &p = get_point(j);
                    double d = (p.x-pl.x)*(p.x-pl.x)+(p.y-pl.y)*(p.y-pl.y);
                    if (d < bestd) { bestd = d; second = j; }
                }
                candL = second;
            }
        }

        // compute turn angles
        auto safe_turn = [&](const std::vector<int>& path, int cand)->double{
            if (path.size() >= 2 && cand >= 0) {
                return turn_angle_deg(get_point(path[path.size()-2]),
                                      get_point(path.back()),
                                      get_point(cand));
            }
            return 0.0;
        };
        double deltaL = safe_turn(path_l_idx, candL);
        double deltaR = safe_turn(path_r_idx, candR);

        auto bad = [&](double a){ return !std::isnan(a) && (a < angle_low_deg || a > angle_high_deg); };
        bool badL = bad(deltaL);
        bool badR = bad(deltaR);

        if (badL || badR) {
            std::swap(candL, candR);
            std::swap(deltaL, deltaR);
            badL = bad(deltaL);
            badR = bad(deltaR);
            if (badL) candL = -1;
            if (badR) candR = -1;
        }

        if (candL >= 0) { path_l_idx.push_back(candL); remaining[candL] = 0; }
        if (candR >= 0) { path_r_idx.push_back(candR); remaining[candR] = 0; }
    }

    res.left->reserve(path_l_idx.size());
    res.right->reserve(path_r_idx.size());
    for (int i : path_l_idx) res.left->push_back(cones->points[i]);
    for (int i : path_r_idx) res.right->push_back(cones->points[i]);
    return res;
}

}


/*
int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<std::pair<float,float>> pts = {
        {0,0},{0,3},{2,5},{3,5},{1,0},{1,2},{3,4}
    };
    for (auto &p: pts){ pcl::PointXYZ q; q.x=p.first; q.y=p.second; q.z=0; cloud->push_back(q);}
    auto res = cones::distinguishLeftRight(cloud, 0.5, -0.5);
    std::cout << "Left path:\n"; for (auto &p: res.left->points) std::cout<<p.x<<","<<p.y<<"\n";
    std::cout << "Right path:\n"; for (auto &p: res.right->points) std::cout<<p.x<<","<<p.y<<"\n";
    return 0;
}
*/