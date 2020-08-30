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
    wz_dt = 0;
    wzdtSample = Eigen::MatrixXd(sampleNum, 1);
    wzdtCov = Eigen::MatrixXd(1, 1);

    // imu = flag
    bCalibrationDone = false;
    bIMUavailable = false;

    // gps - data
    gpsSample = Eigen::MatrixXd::Zero(sampleNum, dim_gps);
    gpsData = Eigen::MatrixXd::Zero(dim_gps, 1);
    gpsCov = Eigen::MatrixXd::Zero(dim_gps, dim_gps);

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
    X_prev = Eigen::MatrixXd::Zero(dim_kf, 1);
    bKalmanInit = true;

    //other
    seq = 0;

    std::cout.precision(15);
}

void Localizer::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ros::Time time_current = ros::Time::now();
    timeIMUelapsed = time_current.toSec() - timeIMUprev.toSec();
    double dt = timeIMUelapsed;

    imu = *msg;

    localHeading = localHeading + imu.angular_velocity.z * dt;
    localHeading = std::atan2(sin(localHeading), cos(localHeading));

    wz_dt = wz_dt + imu.angular_velocity.z * dt;
    wz_dt = std::atan2(sin(wz_dt), cos(wz_dt));
    Eigen::MatrixXd data = Eigen::MatrixXd::Zero(1, 1);
    data << wz_dt;
    updateSample(wzdtSample,data);

    timeIMUprev = time_current;
    bIMUavailable = true;
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
        gpsData(GPSIDX::X) = utm_x_meas;
        gpsData(GPSIDX::Y) = utm_y_meas;
        gpsData(GPSIDX::YAW) = 0;
        gpsData(GPSIDX::V) = 0;
        bInitgpsSample = true;
        bInitvelSample = true;
        bInitgps = false;
        timeGPSprev = ros::Time::now();
        return;
    }

    double theta = std::atan2(utm_y_meas - gpsData(GPSIDX::Y), utm_x_meas - gpsData(GPSIDX::X));
    if (erp.getState() == "BACKWARD") theta += M_PI;
    theta = std::atan2(sin(theta), cos(theta));
    gpsData(GPSIDX::X) = utm_x_meas;
    gpsData(GPSIDX::Y) = utm_y_meas;
    gpsData(GPSIDX::YAW) = theta;

    if(bInitgpsSample)
    {
        gpsSample.block(0,GPSIDX::X,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,gpsData(GPSIDX::X));
        gpsSample.block(0,GPSIDX::Y,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,gpsData(GPSIDX::Y));
        gpsSample.block(0,GPSIDX::YAW,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,gpsData(GPSIDX::YAW));
        bInitgpsSample = false;
    }

    Eigen::MatrixXd data = gpsData.transpose();
    getCovariance(gpsSample, data, gpsCov);
    isGPSavailable = true;

    filter();
    visualizerCallback();

    isGPSavailable = false;
    bIMUavailable = false;
}

void Localizer::bestvelCallback(const novatel_gps_msgs::NovatelVelocity::ConstPtr &msg)
{
    bestvel = *msg;
    gpsData(GPSIDX::V) = bestvel.horizontal_speed;
    if (erp.getState() == "BACKWARD") gpsData(GPSIDX::V) = -1 * bestvel.horizontal_speed;
    else if (erp.getState() == "STOP") gpsData(GPSIDX::V) = 0;
}

void Localizer::bestposCallback(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg)
{
    bestpos = *msg;
}

void Localizer::filter()
{
    // rostime display
    ros::Time time_current = ros::Time::now();
    timeGPSelapsed = time_current.toSec() - timeGPSprev.toSec();
    ROS_INFO("TIME ELAPSED : %f", timeGPSelapsed);
    if (isGPSavailable)
    {
        ROS_INFO("GET GPS DATA, SEQ : %lu", seq++);

        Eigen::MatrixXd X_curr = Eigen::MatrixXd::Zero(dim_kf,1);
        Eigen::MatrixXd X_next = Eigen::MatrixXd::Zero(dim_kf,1);
        Eigen::MatrixXd P_curr = Eigen::MatrixXd::Zero(dim_kf,dim_kf);

        if(bKalmanInit)
        {
            X_curr(KFIDX::X) = gpsData(GPSIDX::X);
            X_curr(KFIDX::Y) = gpsData(GPSIDX::Y);
            X_curr(KFIDX::YAW) = gpsData(GPSIDX::YAW);
            X_curr(KFIDX::VX) = gpsData(GPSIDX::V);
            P_curr(KFIDX::X, KFIDX::X) = gps.position_covariance[0];
            P_curr(KFIDX::Y, KFIDX::Y) = gps.position_covariance[4];
            P_curr(KFIDX::YAW, KFIDX::YAW) = gpsCov(GPSIDX::YAW);
            P_curr(KFIDX::VX, KFIDX::VX) = gpsCov(GPSIDX::V);
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
            ROS_INFO("ERP ENC : %d", erp.getEncoder());
            ROS_INFO("ERP STATE : %s", erp.getState().c_str());
            ROS_INFO("ERP STEER : %d", erp.getSteer());
            ROS_INFO("ERP VEL(m/s) : %f", erp.getVelocity());
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
                  << result(KFIDX::X) << " "
                  << result(KFIDX::Y) << " "
                  << result(KFIDX::YAW) * 180 / M_PI << " "
                  << result(KFIDX::DYAW) * 180 / M_PI << " "
                  << result(KFIDX::VX) << " "
                  << "\n\n";

        tf2::Quaternion q_filtered;
        q_filtered.setRPY(0,0,result(KFIDX::YAW));
        q_filtered.normalize();

        tf2::Quaternion q_local, q_new;
        q_local.setRPY(0,0,localHeading);
        q_new = q_local * qYawBias;
        q_new.normalize();

        geometry_msgs::PoseWithCovariance pos;
        pos.pose.position.x = result(KFIDX::X);
        pos.pose.position.y = result(KFIDX::Y);
        pos.pose.position.z = 0;
        pos.pose.orientation = tf2::toMsg(q_filtered);
        pos.covariance[6*0 + 0] = gpsCov(GPSIDX::X);
        pos.covariance[6*1 + 1] = gpsCov(GPSIDX::Y);
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
    timeGPSprev = time_current;
}

void Localizer::predict()
{
    Eigen::MatrixXd X_curr = Eigen::MatrixXd::Zero(dim_kf,1);
    Eigen::MatrixXd X_next = Eigen::MatrixXd::Zero(dim_kf,1);
    Eigen::MatrixXd P_curr = Eigen::MatrixXd::Zero(dim_kf,dim_kf);

    kf_.getX(X_curr);
    kf_.getP(P_curr);
    X_prev = X_curr;

    const double yaw = X_curr(KFIDX::YAW);
    const double yaw_dt = X_curr(KFIDX::DYAW);
    const double vx = X_curr(KFIDX::VX);
    const double dt = timeGPSelapsed;

    X_next(KFIDX::X) = X_curr(KFIDX::X) + vx * cos(yaw) * dt;  // dx = v * cos(yaw)
    X_next(KFIDX::Y) = X_curr(KFIDX::Y) + vx * sin(yaw) * dt;  // dy = v * sin(yaw)
    X_next(KFIDX::YAW) = X_curr(KFIDX::YAW) + yaw_dt;                    // dyaw = omega + omega_bias
    X_next(KFIDX::DYAW) = yaw_dt;
    X_next(KFIDX::VX) = vx;

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_kf,dim_kf);
    A(KFIDX::X, KFIDX::YAW) = -1 * vx * dt * std::sin(yaw);
    A(KFIDX::X, KFIDX::VX) = dt * std::cos(yaw);
    A(KFIDX::Y, KFIDX::YAW) = -1 * vx * dt * std::cos(yaw);
    A(KFIDX::Y, KFIDX::VX) = dt * std::sin(yaw);
    A(KFIDX::YAW, KFIDX::DYAW) = 1;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_kf,dim_kf);

    const double dvx = std::sqrt(P_curr(KFIDX::VX, KFIDX::VX));
    const double dyaw = std::sqrt(P_curr(KFIDX::YAW, KFIDX::YAW));

    if (dvx < 10.0 && dyaw < 1.0)
    {
      // auto covariance calculate for x, y assuming vx & yaw estimation covariance is small

      /* Set covariance matrix Q for process noise. Calc Q by velocity and yaw angle covariance :
       dx = Ax + Jp*w -> Q = Jp*w_cov*Jp'          */
      Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(2, 2);  // coeff of deviation of vx & yaw
      Jp << cos(yaw), -vx * sin(yaw), sin(yaw), vx * cos(yaw);
      Eigen::MatrixXd Q_vx_yaw = Eigen::MatrixXd::Zero(2, 2);  // cov of vx and yaw

      Q_vx_yaw(0, 0) = P_curr(KFIDX::VX, KFIDX::VX) * dt;        // covariance of vx - vx
      Q_vx_yaw(1, 1) = P_curr(KFIDX::YAW, KFIDX::YAW) * dt;      // covariance of yaw - yaw
      Q_vx_yaw(0, 1) = P_curr(KFIDX::VX, KFIDX::YAW) * dt;       // covariance of vx - yaw
      Q_vx_yaw(1, 0) = P_curr(KFIDX::YAW, KFIDX::VX) * dt;       // covariance of yaw - vx
      Q.block(0, 0, 2, 2) = Jp * Q_vx_yaw * Jp.transpose();  // for pos_x & pos_y
    }
    else
    {
      // vx & vy is not converged yet, set constant value.
      Q(KFIDX::X, KFIDX::X) = 0.05;
      Q(KFIDX::Y, KFIDX::Y) = 0.05;
    }

    Q(KFIDX::YAW, KFIDX::YAW) = 0.05;         // for yaw
    Q(KFIDX::DYAW, KFIDX::DYAW) = 0.1;         // for dyaw
    Q(KFIDX::VX, KFIDX::VX) = 0.1;            // for v

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
    Eigen::MatrixXd X_next = Eigen::MatrixXd::Zero(dim_kf,1);
    Eigen::MatrixXd P_next = Eigen::MatrixXd::Zero(dim_kf,dim_kf);

    kf_.getX(X_next);
    kf_.getP(P_next);

    // measurement
    int dim_meas = 7;
    Eigen::MatrixXd Z_ = Eigen::MatrixXd::Zero(dim_meas, 1);
    double heading = std::atan2(std::sin(localHeading + tf2::getYaw(qYawBias)), std::cos(localHeading + tf2::getYaw(qYawBias)));
    double erpwzdt = erp.getVelocity() / 1.04 * std::tan(((-1 * erp.getSteer() + 75) / 71.0) * M_PI / 180) * timeGPSelapsed;
    Z_ << gpsData(GPSIDX::X)
        , gpsData(GPSIDX::Y)
        , heading
        , wz_dt
        , erpwzdt
        , gpsData(GPSIDX::V)
        , erp.getVelocity();
    std::cout << Z_ << std::endl;
    wz_dt = 0;

    getCovariance(wzdtSample, wzdtCov);


    std::string postype = bestpos.position_type;
    ROS_INFO("POSTYPE : %s", postype.c_str());
    ROS_INFO("WZDTCOV : %f", wzdtCov(0,0));
    ROS_INFO("ERPSTEERCOV : %f", erp.getSteerCovariance());
    ROS_INFO("ERPVELCOV : %f", erp.getVelocityCovariance());

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_meas, dim_kf);
    C(KFIDX::X, KFIDX::X) = 1.0;
    C(KFIDX::Y, KFIDX::Y) = 1.0;
    C(2, KFIDX::YAW) = 1.0;
    C(3, KFIDX::DYAW) = 1.0;
    C(4, KFIDX::DYAW) = 1.0;
    C(5, KFIDX::VX) = 1.0;
    C(6, KFIDX::VX) = 1.0;

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_meas, dim_meas);
//    R(KFIDX::X, KFIDX::Y) = gpsCov(GPSIDX::X, GPSIDX::Y);
//    R(KFIDX::Y, KFIDX::X) = gpsCov(GPSIDX::Y, GPSIDX::X);
    R(KFIDX::X, KFIDX::X) = gpsCov(GPSIDX::X, GPSIDX::X);
    R(KFIDX::Y, KFIDX::Y) = gpsCov(GPSIDX::Y, GPSIDX::Y);
    R(2, 2) = wzdtCov(0,0) * 50;  // imu heading
    R(3, 3) = wzdtCov(0,0) * 50;  // wz*dt imu
//    R(4, 4) = 0.0005;   // wz*dt erp
    R(4, 4) = erp.getSteerCovariance();
    R(5, 5) = gpsCov(GPSIDX::V, GPSIDX::V);   // vel gps
    R(6, 6) = 0.005;   // vel erp

    if (!bIMUavailable)
    {
        ROS_ERROR("IMU FALSE");
    }

    ROS_INFO("R X :\t%f", R(0,0));
    ROS_INFO("R Y :\t%f", R(1,1));
    ROS_INFO("R YAWIMU :\t%f", R(2,2));
    ROS_INFO("R WZDTIMU :\t%f", R(3,3));
    ROS_INFO("R WZDTERP :\t%f", R(4,4));
    ROS_INFO("R VELGPS :\t%f", R(5,5));
    ROS_INFO("R VELERP :\t%f", R(6,6));

    double yawVariance = gpsCov(GPSIDX::YAW, GPSIDX::YAW); // to degree
    double gate_yaw = 0.025;
    if (bCalibrationDone)
    {
        if (yawVariance < gate_yaw * gate_yaw) // gps yaw estimation stable
        {
            ROS_INFO("GOOD GPS YAW : %f, var : %f",gpsData(GPSIDX::YAW) * 180 / M_PI, yawVariance);
            headings += gpsData(GPSIDX::YAW);
            frameCount++;
            const int recal = 5;
            if(frameCount >= recal && erp.getState() == "FORWARD") // get yaw bias
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
            ROS_WARN("SUCK GPS YAW : %f, var : %f",gpsData(GPSIDX::YAW) * 180 / M_PI, yawVariance);
        }
    }

//    Eigen::MatrixXd mahalanobis = (Z_ - X_next).transpose() * P_next.inverse() * (Z_ - X_next);
//    const double distance = mahalanobis(0);
//    const double gate = 0.05;

//    if(distance < gate)
//    {
//        ROS_INFO("distance : %f", distance);
//    }
//    else
//    {
//        ROS_WARN("distance : %f", distance);
//    }

    const int dim_update = 4;
    Eigen::MatrixXd Z_update = Z_.block(0,0,dim_update,1);
    Eigen::MatrixXd C_update = C.block(0,0,dim_update,dim_kf);
    Eigen::MatrixXd R_update = R.block(0,0,dim_update,dim_update);

    if (kf_.update(Z_, C, R))
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
    Eigen::MatrixXd result(dim_kf,1);
    kf_.getX(result);

    tf2::Quaternion q_local, q_new;
    q_local.setRPY(0,0,localHeading);
    q_new = q_local * qYawBias;
    q_new.normalize();

    tf2::Quaternion q_filtered;
    q_filtered.setRPY(0,0,result(KFIDX::YAW));
    q_filtered.normalize();

    tf2::Quaternion q_gps;
    q_gps.setRPY(0,0,gpsData(KFIDX::YAW));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "baisic_shapes";
    marker.id = markerId++;
//    if (markerId >= 100) markerId = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = gpsData(KFIDX::X) - utmoffsetX;
    marker.pose.position.y = gpsData(KFIDX::Y) - utmoffsetY;
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
    marker_filtered.pose.position.x = result(KFIDX::X) - utmoffsetX;
    marker_filtered.pose.position.y = result(KFIDX::Y) - utmoffsetY;
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
    tStamp.transform.translation.x = result(KFIDX::X) - utmoffsetX;
    tStamp.transform.translation.y = result(KFIDX::Y) - utmoffsetY;
    tStamp.transform.translation.z = 0;
    tStamp.transform.rotation = tf2::toMsg(q_filtered);
    br.sendTransform(tStamp);
}
