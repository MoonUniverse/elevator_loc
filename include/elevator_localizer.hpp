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
#include <sstream>
#include <thread>

#include "angles/angles.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

using namespace std;
using namespace Eigen;

typedef union {
    unsigned char cv[4];
    float fv;
} float_union;

enum MOVING_TO_TARGET_STATE {
    MOVING_TO_TARGET_IDLE = 0,
    FIRST_STAGE_MOVE,
    SECOND_STAGE_MOVE,
    THIRD_STAGE_MOVE,
    NEAR_WALL_MOVE,
    LEAVE_STAGE_MOVE
};

namespace elevator_localizer {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
typedef Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, 1> List4DPoints;

class ElevatorLocalizer {
private:
    void runBehavior(void);
    void lidarpointcallback(const sensor_msgs::PointCloud2::ConstPtr &pointMsgIn);
    void refpointfusion(List4DPoints elevator_vertex);
    void cmdcallback(const std_msgs::String::ConstPtr &msg);
    inline geometry_msgs::Twist calcVelocity(double linear_vel, double angle_vel);

    PM::TransformationParameters parseTranslation(string &translation, const int cloudDimension);
    PM::TransformationParameters parseRotation(string &rotation, const int cloudDimension);
    std::thread *run_behavior_thread_;

    ros::Subscriber cur_point_sub;
    ros::Subscriber start_cmd_sub;

    ros::Publisher filtered_point_pub;
    ros::Publisher compute_point_pub;
    ros::Publisher ref_point_pub;
    ros::Publisher elevator_coordinate_pub;
    ros::Publisher vehicle_vel_pub;

    List4DPoints positions_of_markers_on_object;

    double elevator_length_, elevator_width_;
    double inflation_coefficient_;
    double detect_up_, detect_down_, detect_right_, detect_left_, lidar_intensity_;
    double base_sick_link_;

    PM::TransformationParameters translation;
    PM::TransformationParameters rotation;
    PM::TransformationParameters initTransfo;

    sensor_msgs::PointCloud2 ref_point;

    string initTranslation_, initRotation_;
    geometry_msgs::PoseStamped elevator_coordinate;

    MOVING_TO_TARGET_STATE moving_state_;

    double ki_integrator_a;
    bool it_since_initialized_;

public:
    // Create the default ICP algorithm
    PM::ICP icp;

    ElevatorLocalizer();
    ~ElevatorLocalizer();

    tf::TransformListener tfListener;
};

} // namespace elevator_localizer

#endif