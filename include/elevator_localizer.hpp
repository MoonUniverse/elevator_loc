#ifndef ELEVATOR_LOCALIZER_H_
#define ELEVATOR_LOCALIZER_H_

#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <thread>

#include "geometry_msgs/PoseArray.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "tf/transform_listener.h"
using namespace std;
using namespace Eigen;

typedef union {
    unsigned char cv[4];
    float fv;
} float_union;
namespace elevator_localizer {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
typedef Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, 1> List4DPoints;

class ElevatorLocalizer {
private:
    void runBehavior(void);
    void laserscancallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void refpointfusion(List4DPoints elevator_vertex);
    PM::TransformationParameters parseTranslation(string &translation, const int cloudDimension);
    PM::TransformationParameters parseRotation(string &rotation, const int cloudDimension);
    std::thread *run_behavior_thread_;

    ros::Subscriber laser_scan_sub;
    ros::Publisher ref_point_pub;

    List4DPoints positions_of_markers_on_object;

    double elevator_length_, elevator_width_;
    double inflation_coefficient_;

    sensor_msgs::PointCloud2 ref_point;

    string initTranslation_, initRotation_;

public:
    // Create the default ICP algorithm
    PM::ICP icp;

    ElevatorLocalizer();
    ~ElevatorLocalizer();

    tf::TransformListener tfListener;
};

} // namespace elevator_localizer

#endif