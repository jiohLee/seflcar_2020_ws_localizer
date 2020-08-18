#include "localizer/localizer.h"

const double Localizer::imuUpdateRate = 0.016; // second
const double Localizer::gpsUpdateRate = 0.05; // second

Localizer::Localizer()
    :nh_("")
    ,pnh_("~")
    ,erp(nh_, pnh_)
{
    std::string imu_subscribe_topic_name;
    std::string gps_subscribe_topic_name;
    std::string gps_bestvel_subscribe_topic_name;
    std::string gps_bestpos_subscribe_topic_name;
    bool gps_velocity_estimate;

    // param - imu
    pnh_.param<std::string>("imu_subscribe_topic_name",imu_subscribe_topic_name,"/ebimu_");
    pnh_.param<int>("calibration_frame_count", calibrationFrameCount, 40);
    frameCount = 0;

    // param - gps
    pnh_.param<std::string>("gps_subscribe_topic_name",gps_subscribe_topic_name,"/novatel_fix");
    pnh_.param<std::string>("gps_bestvel_subscribe_topic_name",gps_bestvel_subscribe_topic_name,"/bestvel");
    pnh_.param<std::string>("gps_bestpos_subscribe_topic_name",gps_bestpos_subscribe_topic_name,"/bestpos");
    pnh_.param<int>("covariance_sample_num", sampleNum, 3);
    pnh_.param<bool>("gps_velocity_estimate",gps_velocity_estimate,true);

    // param - map
    pnh_.param<double>("utmOffset_x",utmoffsetX,361875);
    pnh_.param<double>("utmOffset_y",utmoffsetY,4124215.34631);

    // ros - subscriber
    subIMU = nh_.subscribe(imu_subscribe_topic_name,1,&Localizer::imuCallback,this);
    subGPS = nh_.subscribe(gps_subscribe_topic_name,1,&Localizer::gpsCallback,this);
    subBestVel = nh_.subscribe(gps_bestvel_subscribe_topic_name,1,&Localizer::bestvelCallback,this);
    subBestPos = nh_.subscribe(gps_bestpos_subscribe_topic_name,1,&Localizer::bestposCallback,this);

    // ros - publisher
    pubPose = nh_.advertise<geometry_msgs::PoseWithCovariance>("Perception/Localization/LocalPose",1);
    pubMarker = nh_.advertise<visualization_msgs::Marker>("Perception/Localization/marker",1);
    pubMarker_filtered = nh_.advertise<visualization_msgs::Marker>("Perception/Localization/marker_filtered",1);

    // imu - data
    localHeading = 0;
    headings = 0;
    qYawBias.setRPY(0,0,0);

    // imu = flag
    bCalibrationDone = false;

    // gps - data
    gpsSample = Eigen::MatrixXd::Zero(sampleNum, dim_x);
    gpsData = Eigen::MatrixXd::Zero(dim_x, 1);
    gpsCov = Eigen::MatrixXd::Zero(dim_x, dim_x);

    // gps - flag
    bInitgps = true;
    bInitgpsSample = false;
    bInitvelSample = false;
    isGPSstop = true;
    isGPSavailable = false;

    // visualizer
    markerId = 0;
    markerId_filtered = 0;

    // kalman filter
    X_prev = Eigen::MatrixXd::Zero(dim_x, 1);
    bKalmanInit = true;

    //other
    seq = 0;

    std::cout.precision(15);
}

void Localizer::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu = *msg;
    localHeading = localHeading + imu.angular_velocity.z * imuUpdateRate;
    localHeading = std::atan2(sin(localHeading), cos(localHeading));
}

void Localizer::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gps = *msg;
    double utm_x_meas = 0;
    double utm_y_meas = 0;
    std::string utm_zone;
    RobotLocalization::NavsatConversions::LLtoUTM(gps.latitude, gps.longitude,utm_y_meas, utm_x_meas, utm_zone);

    if(bInitgps)
    {
        gpsData(IDX::X) = utm_x_meas;
        gpsData(IDX::Y) = utm_y_meas;
        gpsData(IDX::YAW) = 0;
        gpsData(IDX::V) = 0;
        bInitgpsSample = true;
        bInitvelSample = true;
        bInitgps = false;
        return;
    }

    double theta = std::atan2(utm_y_meas - gpsData(IDX::Y), utm_x_meas - gpsData(IDX::X));
    gpsData(IDX::X) = utm_x_meas;
    gpsData(IDX::Y) = utm_y_meas;
    gpsData(IDX::YAW) = theta;

    if(bInitgpsSample)
    {
        gpsSample.block(0,IDX::X,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,gpsData(IDX::X));
        gpsSample.block(0,IDX::Y,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,gpsData(IDX::Y));
        gpsSample.block(0,IDX::YAW,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,gpsData(IDX::YAW));
        bInitgpsSample = false;
    }

    Eigen::MatrixXd data = gpsData.transpose();
    getCovariance(gpsSample, data, gpsCov);
    isGPSavailable = true;

    filter();
    visualizerCallback();

    isGPSavailable = false;
}

void Localizer::bestvelCallback(const novatel_gps_msgs::NovatelVelocity::ConstPtr &msg)
{
    bestvel = *msg;
    gpsData(IDX::V) = bestvel.horizontal_speed;
    if(bestvel.horizontal_speed > 0.8)
    {
        isGPSstop = false;
    }
    else
    {
        isGPSstop = true;
    }
}

void Localizer::bestposCallback(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg)
{
    bestpos = *msg;
}

void Localizer::filter()
{
    if (isGPSavailable)
    {
        ROS_INFO("GET GPS DATA, SEQ : %lu", seq++);

        Eigen::MatrixXd X_curr = Eigen::MatrixXd::Zero(dim_x,1);
        Eigen::MatrixXd X_next = Eigen::MatrixXd::Zero(dim_x,1);
        Eigen::MatrixXd P_curr = Eigen::MatrixXd::Zero(dim_x,dim_x);

        if(bKalmanInit)
        {
            X_curr(IDX::X) = gpsData(IDX::X);
            X_curr(IDX::Y) = gpsData(IDX::Y);
            X_curr(IDX::YAW) = gpsData(IDX::YAW);
            X_curr(IDX::V) = gpsData(IDX::V);
            P_curr(IDX::X, IDX::X) = gps.position_covariance[0];
            P_curr(IDX::Y, IDX::Y) = gps.position_covariance[4];
            P_curr(IDX::YAW, IDX::YAW) = gpsCov(IDX::YAW);
            P_curr(IDX::V, IDX::V) = gpsCov(IDX::V);
            kf_.init(X_curr, P_curr);
            bKalmanInit = false;
            ROS_INFO("Initialized Kalman Filter");
            return;
        }

        bool fStop = isGPSstop;
        if(erp.isERPavailable())
        {
            fStop = erp.isStop();
            ROS_INFO("ERP MODE : %s", erp.getAorM().c_str());
            ROS_INFO("ERP ESTOP : %s", erp.getEstop().c_str());
            ROS_INFO("ERP GEAR : %s", erp.getGear().c_str());
            ROS_INFO("ERP VEL(m/s) : %f", erp.getVelocity());
            ROS_INFO("ERP ENC : %d", erp.getEncoder());
            ROS_INFO("ERP STATE : %s", erp.getState().c_str());
        }

        if (!fStop && !bCalibrationDone)
        {
            ROS_WARN("GPS/IMU Calibration");
            headings += gpsData(2);
            frameCount++;
            qYawBias.setRPY(0,0, headings / frameCount);
            qYawBias.normalize();

            if (frameCount >= calibrationFrameCount)
            {
                bCalibrationDone = true;
                frameCount = 0;
                headings = 0;
                ROS_WARN("GPS/IMU Calibration DONE");
            }
        }

        // predict
        predict();
        // update
        updateWithGate();

        Eigen::MatrixXd result;
        Eigen::MatrixXd P_result;

        kf_.getX(result);
        kf_.getP(P_result);

        std::cout << "X(+) : "
                  << result(IDX::X) << " "
                  << result(IDX::Y) << " "
                  << result(IDX::YAW) * 180 / M_PI << " "
                  << result(IDX::V) << " "
                  << "\n\n";

        tf2::Quaternion q_filtered;
        q_filtered.setRPY(0,0,result(IDX::YAW));
        q_filtered.normalize();

        tf2::Quaternion q_local, q_new;
        q_local.setRPY(0,0,localHeading);
        q_new = q_local * qYawBias;
        q_new.normalize();

        geometry_msgs::PoseWithCovariance pos;
        pos.pose.position.x = result(IDX::X);
        pos.pose.position.y = result(IDX::Y);
        pos.pose.position.z = 0;
        pos.pose.orientation = tf2::toMsg(q_new);
        pos.covariance[6*0 + 0] = gpsCov(IDX::X);
        pos.covariance[6*1 + 1] = gpsCov(IDX::Y);
        pos.covariance[6*2 + 2] = 0;
        pos.covariance[6*3 + 3] = imu.orientation_covariance[0];
        pos.covariance[6*4 + 4] = imu.orientation_covariance[4];
        pos.covariance[6*5 + 5] = imu.orientation_covariance[8];
        pubPose.publish(pos);
    }
    else
    {
        ROS_WARN("NO GPS DATA");
        return;
    }
}

void Localizer::predict()
{
    /*
     * [ MY MODEL]
     * x_k+1 = x_k + v_k * cos(yaw_k) * dt;
     * y_k+1 = y_k + v_k * sin(yaw_k) * dt;
     * yaw_k+1 = yaw_k
     * v_k+1 = v_k
    */

    /*
     * [MY LINEARLISED MODEL]
     * A =  [ 1, 0, -v * sin(yaw) * dt, cos(yaw) * dt]
     *      [ 0, 1, -v * cos(yaw) * dt, sin(yaw) * dt]
     *      [ 0, 0, 1, 0]
     *      [ 0, 0, 0, 1]
    */

    Eigen::MatrixXd X_curr = Eigen::MatrixXd::Zero(dim_x,1);
    Eigen::MatrixXd X_next = Eigen::MatrixXd::Zero(dim_x,1);
    Eigen::MatrixXd P_curr = Eigen::MatrixXd::Zero(dim_x,dim_x);

    kf_.getX(X_curr);
    kf_.getP(P_curr);
    X_prev = X_curr;
    P_prev = P_curr;

    double yaw = X_curr(IDX::YAW);
    double v = X_curr(IDX::V);
    double dt = gpsUpdateRate;
    double heading = std::atan2(std::sin(localHeading + tf2::getYaw(qYawBias)), std::cos(localHeading + tf2::getYaw(qYawBias)));

    X_next(IDX::X) = X_curr(IDX::X) + v * std::cos(yaw) * dt;
    X_next(IDX::Y) = X_curr(IDX::Y) + v * std::sin(yaw) * dt;
    X_next(IDX::YAW) = heading;
    X_next(IDX::V) = erp.getVelocity();

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x,dim_x);
    A(IDX::X, IDX::YAW) = -1 * v * dt * std::sin(yaw);
    A(IDX::X, IDX::V) = dt * std::cos(yaw);
    A(IDX::Y, IDX::YAW) = -1 * v * dt * std::cos(yaw);
    A(IDX::Y, IDX::V) = dt * std::sin(yaw);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x,dim_x);

    const double dvx = std::sqrt(P_curr(IDX::V, IDX::V));
    const double dyaw = std::sqrt(P_curr(IDX::YAW, IDX::YAW));

    if (dvx < 10.0 && dyaw < 1.0)
    {
      // auto covariance calculate for x, y assuming vx & yaw estimation covariance is small

      /* Set covariance matrix Q for process noise. Calc Q by velocity and yaw angle covariance :
       dx = Ax + Jp*w -> Q = Jp*w_cov*Jp'          */
      Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(2, 2);  // coeff of deviation of vx & yaw
      Jp << cos(yaw), -v * sin(yaw), sin(yaw), v * cos(yaw);
      Eigen::MatrixXd Q_vx_yaw = Eigen::MatrixXd::Zero(2, 2);  // cov of vx and yaw

      Q_vx_yaw(0, 0) = P_curr(IDX::V, IDX::V) * dt;        // covariance of vx - vx
      Q_vx_yaw(1, 1) = P_curr(IDX::YAW, IDX::YAW) * dt;      // covariance of yaw - yaw
      Q_vx_yaw(0, 1) = P_curr(IDX::V, IDX::YAW) * dt;       // covariance of vx - yaw
      Q_vx_yaw(1, 0) = P_curr(IDX::YAW, IDX::V) * dt;       // covariance of yaw - vx
      Q.block(0, 0, 2, 2) = Jp * Q_vx_yaw * Jp.transpose();  // for pos_x & pos_y
    }
    else
    {
      // vx & vy is not converged yet, set constant value.
      Q(IDX::X, IDX::X) = 0.05;
      Q(IDX::Y, IDX::Y) = 0.05;
    }

    Q(IDX::YAW, IDX::YAW) = 0.005;         // for yaw
    Q(IDX::V, IDX::V) = 0.1;            // for v

    if (kf_.predict(X_next, A, Q))
    {
        ROS_INFO("PREDICT DONE");
    }
    else
    {
        ROS_WARN("PREDICT FAIL");
    }
}

void Localizer::updateWithGate()
{
    Eigen::MatrixXd X_next = Eigen::MatrixXd::Zero(dim_x,1);
    Eigen::MatrixXd P_next = Eigen::MatrixXd::Zero(dim_x,dim_x);

    kf_.getX(X_next);
    kf_.getP(P_next);

    // measurement
    Eigen::MatrixXd Z_ = Eigen::MatrixXd::Zero(4, 1);
    Z_ << gpsData(IDX::X), gpsData(IDX::Y), gpsData(IDX::YAW), gpsData(IDX::V);

    std::string postype = bestpos.position_type;
    ROS_INFO("POSTYPE : %s", postype.c_str());

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(2, dim_x);
    C(IDX::X, IDX::X) = 1.0;
    C(IDX::Y, IDX::Y) = 1.0;

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
    R(IDX::X, IDX::Y) = gpsCov(IDX::X, IDX::Y);
    R(IDX::Y, IDX::X) = gpsCov(IDX::Y, IDX::X);
    R(IDX::X, IDX::X) = gpsCov(IDX::X, IDX::X);
    R(IDX::Y, IDX::Y) = gpsCov(IDX::Y, IDX::Y);
    R *= 500;

    double yawVariance = gpsCov(IDX::YAW, IDX::YAW); // to degree
    double gate_yaw = 0.02;
    if (bCalibrationDone)
    {
        if (yawVariance < gate_yaw * gate_yaw) // gps yaw estimation stable
        {
            ROS_INFO("GOOD GPS YAW : %f, var : %f",gpsData(IDX::YAW) * 180 / M_PI, yawVariance);
            headings += gpsData(IDX::YAW);
            frameCount++;
            const int recal = 5;
            if(frameCount >= recal) // get yaw bias
            {
                qYawBias.setRPY(0,0,headings / recal);
                qYawBias.normalize();
                headings = 0;
                frameCount = 0;
                localHeading = 0;
                ROS_WARN("GET YAW BIAS");
            }
        }
        else
        {
            headings = 0;
            frameCount = 0;
            ROS_WARN("SUCK GPS YAW : %f, var : %f",gpsData(IDX::YAW) * 180 / M_PI, yawVariance);
        }
    }

    Eigen::MatrixXd mahalanobis = (Z_ - X_next).transpose() * P_next.inverse() * (Z_ - X_next);
    const double distance = mahalanobis(0);
    const double gate = 0.05;

    if(distance < gate)
    {
        ROS_INFO("distance : %f", distance);
    }
    else
    {
        ROS_WARN("distance : %f", distance);
    }

    const int dim_update = 2;
    Eigen::MatrixXd Z_update = Z_.block(0,0,dim_update,1);
    Eigen::MatrixXd C_update = C.block(0,0,dim_update,dim_x);
    Eigen::MatrixXd R_update = R.block(0,0,dim_update,dim_update);
    if (kf_.update(Z_update, C_update, R_update))
    {
        ROS_INFO("UPDATE ESTIMATION DONE");
    }
    else
    {
        ROS_WARN("UPDATE ESTIMATION FAIL");
    }
}

void Localizer::visualizerCallback()
{
    Eigen::MatrixXd result(dim_x,1);
    kf_.getX(result);

    tf2::Quaternion q_local, q_new;
    q_local.setRPY(0,0,localHeading);
    q_new = q_local * qYawBias;
    q_new.normalize();

    tf2::Quaternion q_filtered;
    q_filtered.setRPY(0,0,result(IDX::YAW));
    q_filtered.normalize();

    tf2::Quaternion q_gps;
    q_gps.setRPY(0,0,gpsData(IDX::YAW));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "baisic_shapes";
    marker.id = markerId++;
//    if (markerId >= 100) markerId = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = gpsData(IDX::X) - utmoffsetX;
    marker.pose.position.y = gpsData(IDX::Y) - utmoffsetY;
    marker.pose.position.z = 0;
    marker.pose.orientation = tf2::toMsg(q_gps);
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    pubMarker.publish(marker);


    marker_filtered.header.frame_id = "map";
    marker_filtered.header.stamp = ros::Time::now();
    marker_filtered.ns = "baisic_shapes";
    marker_filtered.id = markerId_filtered++;
    marker_filtered.type =
 visualization_msgs::Marker::ARROW;
    marker_filtered.action = visualization_msgs::Marker::ADD;
    marker_filtered.pose.position.x = result(IDX::X) - utmoffsetX;
    marker_filtered.pose.position.y = result(IDX::Y) - utmoffsetY;
    marker_filtered.pose.position.z = 0;
    marker_filtered.pose.orientation = tf2::toMsg(q_filtered);
    marker_filtered.scale.x = 0.2;
    marker_filtered.scale.y = 0.1;
    marker_filtered.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_filtered.color.r = 1.0f;
    marker_filtered.color.g = 0.0f;
    marker_filtered.color.b = 0.0f;
    marker_filtered.color.a = 1.0;
    pubMarker_filtered.publish(marker_filtered);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tStamp;
    tStamp.header.stamp = ros::Time::now();
    tStamp.header.frame_id = "map";
    tStamp.child_frame_id = "gps";
    tStamp.transform.translation.x = result(IDX::X) - utmoffsetX;
    tStamp.transform.translation.y = result(IDX::Y) - utmoffsetY;
    tStamp.transform.translation.z = 0;
    tStamp.transform.rotation = tf2::toMsg(q_filtered);
    br.sendTransform(tStamp);
}
