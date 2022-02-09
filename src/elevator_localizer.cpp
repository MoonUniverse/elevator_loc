#include "elevator_localizer.hpp"
namespace elevator_localizer {
ElevatorLocalizer::ElevatorLocalizer() : ki_integrator_a(0.0) {
    ros::NodeHandle nh("~");

    nh.param("detect_up", detect_up_, 3.0);
    nh.param("detect_down", detect_down_, 0.1);
    nh.param("detect_right", detect_right_, 1.5);
    nh.param("detect_left", detect_left_, 1.5);
    nh.param("lidar_intensity", lidar_intensity_, 30000.0);

    nh.param("base_sick_link", base_sick_link_, 0.0);

    nh.param("inflation_coefficient", inflation_coefficient_, 0.01);
    nh.param<std::string>("initTranslation", initTranslation_, "0,0");
    nh.param<std::string>("initRotation", initRotation_, "1,0;0,1");

    ref_point_pub = nh.advertise<sensor_msgs::PointCloud2>("ref_point", 10);
    filtered_point_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point", 10);
    compute_point_pub = nh.advertise<sensor_msgs::PointCloud2>("compute_point", 10);
    elevator_coordinate_pub = nh.advertise<geometry_msgs::PoseStamped>("elevator_coordinate", 10);
    vehicle_vel_pub = nh.advertise<geometry_msgs::Twist>("/shelf_lifter/velocity", 10);

    // subscribe
    cur_point_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/tim551_cloud", 10, boost::bind(&ElevatorLocalizer::lidarpointcallback, this, _1));
    start_cmd_sub =
        nh.subscribe<std_msgs::String>("/shelf_lifter/cmd", 10, boost::bind(&ElevatorLocalizer::cmdcallback, this, _1));

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

    refpointfusion(positions_of_markers_on_object);

    // See the implementation of setDefault() to create a custom ICP algorithm
    icp.setDefault();

    run_behavior_thread_ = new std::thread(std::bind(&ElevatorLocalizer::runBehavior, this));
}
inline geometry_msgs::Twist ElevatorLocalizer::calcVelocity(double linear_vel, double angle_vel) {
    geometry_msgs::Twist cmd_vel;

    if (linear_vel > 0.2) {
        linear_vel = 0.2;
    }
    if (angle_vel > 0.1) {
        angle_vel = 0.1;
    }
    if (angle_vel < -0.1) {
        angle_vel = -0.1;
    }

    cmd_vel.linear.x = linear_vel;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = angle_vel;

    return cmd_vel;
}

void ElevatorLocalizer::runBehavior(void) {
    ros::NodeHandle nh;
    ros::Rate rate(50);
    geometry_msgs::Twist pub_cmd_vel;
    while (nh.ok()) {
        switch (moving_state_) {
            case MOVING_TO_TARGET_IDLE: {
                ROS_DEBUG("moving_state is Move to target idle!");
                break;
            }
            case FIRST_STAGE_MOVE: {
                /* code */
                double error_dist = hypot(elevator_coordinate.pose.position.x, elevator_coordinate.pose.position.y);
                double error_angle = 0 - angles::normalize_angle(tf::getYaw(elevator_coordinate.pose.orientation));

                ki_integrator_a += (0 - elevator_coordinate.pose.position.y);

                double rot_vel = 3.0 * (0 - elevator_coordinate.pose.position.y) + 0.0 * ki_integrator_a;

                ROS_ERROR("error_dist:%f,error_angle:%f,rot_vel:%f", error_dist, error_angle, rot_vel);

                double move_vel = 0.5 * error_dist;
                if (error_dist >= 0.05) {
                    pub_cmd_vel = calcVelocity(move_vel, rot_vel + error_angle); //
                } else {
                    pub_cmd_vel = calcVelocity(0.0, error_angle);
                    if (fabs(error_angle) < 0.01) {
                        moving_state_ = MOVING_TO_TARGET_IDLE;
                        ki_integrator_a = 0;
                    }
                }

                vehicle_vel_pub.publish(pub_cmd_vel);
                break;
            }

            case SECOND_STAGE_MOVE: {
                /* code */
                break;
            }
        }
        rate.sleep();
    }
}

void ElevatorLocalizer::refpointfusion(List4DPoints elevator_vertex) {
    if (elevator_vertex.size() < 2) {
        ROS_ERROR("elevator vertex input error");
        return;
    }
    vector<unsigned> tmp_data;
    int point_width = 0;
    float_union inflation_point_x, inflation_point_y, inflation_point_z, inflation_intensity;
    for (int i = 0; i < elevator_vertex.size() - 1; i++) {
        double inflation_dist =
            hypot(elevator_vertex(i + 1)(0) - elevator_vertex(i)(0), elevator_vertex(i + 1)(1) - elevator_vertex(i)(1));

        double inflation_angle =
            atan2(elevator_vertex(i + 1)(0) - elevator_vertex(i)(0), elevator_vertex(i + 1)(1) - elevator_vertex(i)(1));
        int inflation_count = inflation_dist / inflation_coefficient_;
        point_width += inflation_count;

        for (int index = 0; index < inflation_count; index++) {
            inflation_point_x.fv = sin(inflation_angle) * (index * inflation_coefficient_) + elevator_vertex(i)(0);
            inflation_point_y.fv = cos(inflation_angle) * (index * inflation_coefficient_) + elevator_vertex(i)(1);
            inflation_point_z.fv = 1.0;
            inflation_intensity.fv = 0;
            for (int j = 0; j < 4; j++) {
                tmp_data.push_back(inflation_point_x.cv[j]);
            }
            for (int j = 0; j < 4; j++) {
                tmp_data.push_back(inflation_point_y.cv[j]);
            }
            for (int j = 0; j < 4; j++) {
                tmp_data.push_back(inflation_point_z.cv[j]);
            }
            for (int j = 0; j < 4; j++) {
                tmp_data.push_back(inflation_intensity.cv[j]);
            }
        }
    }

    const int numChannels = 4;

    ref_point.header.stamp = ros::Time::now();
    ref_point.header.frame_id = "base_link";
    ref_point.header.seq = 0;

    ref_point.height = 1;
    ref_point.width = point_width;

    ref_point.is_bigendian = false;
    ref_point.is_dense = true;
    ref_point.point_step = numChannels * sizeof(float);
    ref_point.row_step = ref_point.point_step * ref_point.width;

    ref_point.fields.resize(numChannels);
    for (int i = 0; i < numChannels; i++) {
        std::string channelId[] = {"x", "y", "z", "intensity"};
        ref_point.fields[i].name = channelId[i];
        ref_point.fields[i].offset = i * sizeof(float);
        ref_point.fields[i].count = 1;
        ref_point.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    ref_point.data.resize(ref_point.row_step * ref_point.height);

    for (int i = 0; i < ref_point.row_step * ref_point.height; ++i) {
        ref_point.data[i] = tmp_data[i];
    }
    ROS_INFO("ref_point.data:%ld", ref_point.data.size());
    tmp_data.clear();
    ref_point_pub.publish(ref_point);
}

void ElevatorLocalizer::lidarpointcallback(const sensor_msgs::PointCloud2::ConstPtr& pointMsgIn) {
    if (pointMsgIn == nullptr) {
        return;
    }
    ref_point_pub.publish(ref_point);
    sensor_msgs::PointCloud2 filtered_point;
    float_union point_x, point_y, point_z, intensity;
    vector<uint8_t> tmp_data;
    int point_count = 0;
    for (int i = 0; i < pointMsgIn->width; i++) {
        for (int j = 0; j < 4; j++) {
            point_x.cv[j] = pointMsgIn->data[i * 16 + j];
            point_y.cv[j] = pointMsgIn->data[i * 16 + 4 + j];
            point_z.cv[j] = 0;
            intensity.cv[j] = pointMsgIn->data[i * 16 + 12 + j];
        }
        if (point_x.fv > detect_down_ && point_x.fv < detect_up_ && fabs(point_y.fv) < detect_right_ &&
            intensity.fv < lidar_intensity_) {
            point_x.fv = point_x.fv + base_sick_link_;
            point_count++;
            for (int j = 0; j < 4; j++) {
                tmp_data.push_back(point_x.cv[j]);
            }
            for (int j = 0; j < 4; j++) {
                tmp_data.push_back(point_y.cv[j]);
            }
            for (int j = 0; j < 4; j++) {
                tmp_data.push_back(point_z.cv[j]);
            }
            for (int j = 0; j < 4; j++) {
                tmp_data.push_back(intensity.cv[j]);
            }
        }
    }

    const int numChannels = 4;

    filtered_point.header.stamp = ros::Time::now();
    filtered_point.header.frame_id = "base_link";
    filtered_point.header.seq = 0;

    filtered_point.height = 1;
    filtered_point.width = point_count;

    filtered_point.is_bigendian = false;
    filtered_point.is_dense = true;
    filtered_point.point_step = numChannels * sizeof(float);
    filtered_point.row_step = filtered_point.point_step * filtered_point.width;

    filtered_point.fields.resize(numChannels);
    for (int i = 0; i < numChannels; i++) {
        std::string channelId[] = {"x", "y", "z", "intensity"};
        filtered_point.fields[i].name = channelId[i];
        filtered_point.fields[i].offset = i * sizeof(float);
        filtered_point.fields[i].count = 1;
        filtered_point.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    filtered_point.data.resize(filtered_point.row_step * filtered_point.height);

    for (int i = 0; i < filtered_point.row_step * filtered_point.height; ++i) {
        filtered_point.data[i] = tmp_data[i];
    }
    tmp_data.clear();
    filtered_point_pub.publish(filtered_point);

    DP ref_cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(ref_point);
    DP input_cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(filtered_point);

    int cloudDimension = ref_cloud.getEuclideanDim();
    // ROS_ERROR("cloudDimension:%d", cloudDimension);
    if (!(cloudDimension == 2 || cloudDimension == 3)) {
        cerr << "Invalid input point clouds dimension" << endl;
        return;
    }

    PM::TransformationParameters translation = parseTranslation(initTranslation_, cloudDimension);
    PM::TransformationParameters rotation = parseRotation(initRotation_, cloudDimension);
    PM::TransformationParameters initTransfo = translation * rotation;

    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
    if (!rigidTrans->checkParameters(initTransfo)) {
        cerr << endl << "Initial transformation is not rigid, identiy will be used" << endl;
        initTransfo = PM::TransformationParameters::Identity(cloudDimension + 1, cloudDimension + 1);
    }

    const DP initializedData = rigidTrans->compute(input_cloud, initTransfo);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(initializedData, ref_cloud);

    // Transform data to express it in ref
    DP data_out(initializedData);
    icp.transformations.apply(data_out, T);

    sensor_msgs::PointCloud2 compute_point;

    compute_point = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_out, "base_link", ros::Time::now());

    compute_point_pub.publish(compute_point);

    // initTranslation_ = std::to_string(T(0, 2)) + std::to_string(T(1, 2));

    // cout << "ICP transformation:" << endl << T << endl;

    elevator_coordinate.header.frame_id = "base_link";
    elevator_coordinate.header.stamp = ros::Time::now();

    elevator_coordinate.pose = PointMatcher_ros::eigenMatrixToPoseMsg<float>(T);

    elevator_coordinate_pub.publish(elevator_coordinate);
}

PM::TransformationParameters ElevatorLocalizer::parseTranslation(string& translation, const int cloudDimension) {
    PM::TransformationParameters parsedTranslation;
    parsedTranslation = PM::TransformationParameters::Identity(cloudDimension + 1, cloudDimension + 1);

    translation.erase(std::remove(translation.begin(), translation.end(), '['), translation.end());
    translation.erase(std::remove(translation.begin(), translation.end(), ']'), translation.end());
    std::replace(translation.begin(), translation.end(), ',', ' ');
    std::replace(translation.begin(), translation.end(), ';', ' ');

    float translationValues[3] = {0};
    stringstream translationStringStream(translation);
    for (int i = 0; i < cloudDimension; i++) {
        if (!(translationStringStream >> translationValues[i])) {
            cerr << "An error occured while trying to parse the initial "
                 << "translation." << endl
                 << "No initial translation will be used" << endl;
            return parsedTranslation;
        }
    }
    float extraOutput = 0;
    if ((translationStringStream >> extraOutput)) {
        cerr << "Wrong initial translation size" << endl << "No initial translation will be used" << endl;
        return parsedTranslation;
    }

    for (int i = 0; i < cloudDimension; i++) {
        parsedTranslation(i, cloudDimension) = translationValues[i];
        // ROS_ERROR("translationValues[i]:%f", translationValues[i]);
    }

    return parsedTranslation;
}

PM::TransformationParameters ElevatorLocalizer::parseRotation(string& rotation, const int cloudDimension) {
    PM::TransformationParameters parsedRotation;
    parsedRotation = PM::TransformationParameters::Identity(cloudDimension + 1, cloudDimension + 1);

    rotation.erase(std::remove(rotation.begin(), rotation.end(), '['), rotation.end());
    rotation.erase(std::remove(rotation.begin(), rotation.end(), ']'), rotation.end());
    std::replace(rotation.begin(), rotation.end(), ',', ' ');
    std::replace(rotation.begin(), rotation.end(), ';', ' ');

    float rotationMatrix[9] = {0};
    stringstream rotationStringStream(rotation);
    for (int i = 0; i < cloudDimension * cloudDimension; i++) {
        if (!(rotationStringStream >> rotationMatrix[i])) {
            cerr << "An error occured while trying to parse the initial "
                 << "rotation." << endl
                 << "No initial rotation will be used" << endl;
            return parsedRotation;
        }
    }
    float extraOutput = 0;
    if ((rotationStringStream >> extraOutput)) {
        cerr << "Wrong initial rotation size" << endl << "No initial rotation will be used" << endl;
        return parsedRotation;
    }

    for (int i = 0; i < cloudDimension * cloudDimension; i++) {
        parsedRotation(i / cloudDimension, i % cloudDimension) = rotationMatrix[i];
    }

    return parsedRotation;
}

void ElevatorLocalizer::cmdcallback(const std_msgs::String::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    std_msgs::String cmd_str = *msg;

    if (cmd_str.data.find("move_back") != std::string::npos) {
        // it_since_initialized_ = 0;
        moving_state_ = FIRST_STAGE_MOVE;
        std::cout << "alignment start!" << std::endl;
    }
}

ElevatorLocalizer::~ElevatorLocalizer() {}
} // namespace elevator_localizer
