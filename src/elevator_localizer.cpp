#include "elevator_localizer.hpp"
namespace elevator_localizer {
ElevatorLocalizer::ElevatorLocalizer() : it_since_initialized_(0) {
    ros::NodeHandle nh("~");
    const std::string pclFilename = nh.param<std::string>("pcd_filename", "");
    mapCloud = loadPointcloudFromPcd(pclFilename);

    nh.param("detect_up", detect_up_, 2.0);
    nh.param("detect_down", detect_down_, -1.0);
    nh.param("detect_right", detect_right_, 1.0);
    nh.param("detect_left", detect_left_, 1.0);
    nh.param("lidar_intensity", lidar_intensity_, 35000.0);

    // Create the marker positions from the test points
    List4DPoints positions_of_markers_on_object;
    // Read in the marker positions from the YAML parameter file
    XmlRpc::XmlRpcValue points_list;
    if (!nh.getParam("marker_positions", points_list)) {
        ROS_ERROR(
            "%s: No reference file containing the marker positions, or the "
            "file is improperly formatted. Use the 'marker_positions_file' "
            "parameter in the launch file.",
            ros::this_node::getName().c_str());
        ros::shutdown();
    } else {
        positions_of_markers_on_object.resize(points_list.size());
        for (int i = 0; i < points_list.size(); i++) {
            Eigen::Matrix<double, 4, 1> temp_point;
            temp_point(0) = points_list[i]["x"];
            temp_point(1) = points_list[i]["y"];
            temp_point(2) = points_list[i]["z"];
            temp_point(3) = 1;
            positions_of_markers_on_object(i) = temp_point;
        }
    }
    // publish
    cloudPub = nh.advertise<sensor_msgs::PointCloud2>("icp_map", 10, true);
    laser_filtered_point_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_filtered_point", 10);
    laser_icp_point_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_icp_point_pub", 10);
    box_legs_array_pub = nh.advertise<geometry_msgs::PoseArray>("box_legs", 10);

    // subscribe

    run_behavior_thread_ = new std::thread(std::bind(&ElevatorLocalizer::runBehavior, this));
}

Pointcloud::Ptr ElevatorLocalizer::loadPointcloudFromPcd(const std::string &filename) {
    Pointcloud::Ptr cloud(new Pointcloud);
    pcl::PCLPointCloud2 cloudBlob;
    pcl::io::loadPCDFile(filename, cloudBlob);
    pcl::fromPCLPointCloud2(cloudBlob, *cloud);
    return cloud;
}

void ElevatorLocalizer::publishCloud(Pointcloud::Ptr cloud, const ros::Publisher &pub, const std::string &frameId) {
    cloud->header.frame_id = frameId;
    cloud->header.seq = 0;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
}

void ElevatorLocalizer::runBehavior(void) {
    ros::NodeHandle nh;
    ros::Rate rate(1.0);
    while (nh.ok()) {
        // publishCloud(mapCloud, cloudPub, "laser_sick_tim551");
        rate.sleep();
    }
}
// DP TransportBoxLocalizer::fromPCL(const Pointcloud &pcl) {
//     // todo this can result in data loss???
//     sensor_msgs::PointCloud2 ros;
//     pcl::toROSMsg(pcl, ros);
//     return PointMatcher_ros::rosMsgToPointMatcherCloud<float>(ros);
// }

ElevatorLocalizer::~ElevatorLocalizer() {}
} // namespace elevator_localizer
