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
using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, 1> List4DPoints;

namespace elevator_localizer {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

class ElevatorLocalizer {
private:
    Pointcloud::Ptr loadPointcloudFromPcd(const std::string &filename);
    void publishCloud(Pointcloud::Ptr cloud, const ros::Publisher &pub, const std::string &frameId);
    void runBehavior(void);
    // DP fromPCL(const Pointcloud &pcl);

    Pointcloud::Ptr mapCloud;
    std::thread *run_behavior_thread_;

    ros::Publisher cloudPub;
    ros::Publisher laser_filtered_point_pub;
    ros::Publisher laser_icp_point_pub;
    ros::Publisher box_legs_array_pub;
    ros::Subscriber cloudSub;

    double detect_up_, detect_down_, detect_right_, detect_left_, lidar_intensity_;

    unsigned it_since_initialized_;

public:
    ElevatorLocalizer();
    ~ElevatorLocalizer();
};

} // namespace elevator_localizer

#endif