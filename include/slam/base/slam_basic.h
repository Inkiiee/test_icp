#ifndef __RCL_SLAM_BASIC_H__
#define __RCL_SLAM_BASIC_H__

#include <vector>
#include "nanoflann.h"

namespace rcl_slam_basic_type{
    struct Point{
        double x, y;
        
        Point(double xx=0, double yy=0);
        Point(const Point& p);
        bool operator<(const Point& t) const;
    };

    struct PointCloud{
        std::vector<Point> pts;

        inline size_t kdtree_get_point_count() const {return pts.size();}
        inline double kdtree_get_pt(const size_t idx, int dim) const {
            return (dim == 0) ? pts[idx].x : pts[idx].y;
        }
        inline double kdtree_distance(const double* p, const size_t idx_p2, size_t /*dim*/) const {
            return (p[0] - pts[idx_p2].x) * (p[0] - pts[idx_p2].x) + (p[1] - pts[idx_p2].y) * (p[1] - pts[idx_p2].y);
        }
        template<typename BBOX> bool kdtree_get_bbox(BBOX&) const {return false;}
    };

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2> KDTree;

    struct RobotBasePose{
        double tx, ty, theta;
        RobotBasePose(double x=0, double y=0, double t=0);
        RobotBasePose(const RobotBasePose& p);
    };
}

namespace rcl_slam_basic_transform{
    using rcl_slam_basic_type::RobotBasePose;
    
    void rotationAndTranslation(double tx, double ty, double theta, std::vector<double>& x, std::vector<double>& y);
    double normalizeAngle(double a);
    RobotBasePose identityPose();
    RobotBasePose inversePose(const RobotBasePose& p);
    RobotBasePose relativePose(const RobotBasePose& from, const RobotBasePose& to);
}

#endif // __RCL_SLAM_BASIC_H__