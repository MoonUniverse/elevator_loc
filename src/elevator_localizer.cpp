#include "elevator_localizer.hpp"
namespace elevator_localizer {
ElevatorLocalizer::ElevatorLocalizer() {
    ros::NodeHandle nh("~");

    nh.param("inflation_coefficient", inflation_coefficient_, 0.01);
    nh.param<std::string>("initTranslation", initTranslation_, "0,0,0");
    nh.param<std::string>("initRotation", initRotation_, "1,0,0;0,1,0;0,0,1");

    ref_point_pub = nh.advertise<sensor_msgs::PointCloud2>("ref_point", 10);

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

    // See the implementation of setDefault() to create a custom ICP algorithm
    icp.setDefault();

    // publish

    // subscribe
    laser_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/sick_tim551_scan", 10,
                                                          boost::bind(&ElevatorLocalizer::laserscancallback, this, _1));

    run_behavior_thread_ = new std::thread(std::bind(&ElevatorLocalizer::runBehavior, this));
}

void ElevatorLocalizer::runBehavior(void) {
    ros::NodeHandle nh;
    ros::Rate rate(0.1);
    while (nh.ok()) {
        refpointfusion(positions_of_markers_on_object);
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
            inflation_point_z.fv = 0;
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
    ref_point.header.frame_id = "world";
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

void ElevatorLocalizer::laserscancallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    if (scan_msg == nullptr) {
        return;
    }
    DP input_cloud =
        PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*scan_msg, &tfListener, scan_msg->header.frame_id);
    DP ref_cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(ref_point);
    int cloudDimension = ref_cloud.getEuclideanDim();
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

    cout << "ICP transformation:" << endl << T << endl;
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

ElevatorLocalizer::~ElevatorLocalizer() {}
} // namespace elevator_localizer
