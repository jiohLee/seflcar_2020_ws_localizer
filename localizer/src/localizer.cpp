#include "localizer/localizer.h"

#include "selfcar_lib/trigonometric.h"

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
    std::string initialpose_topic_name;
    bool gps_velocity_estimate;

    // param - imu
    pnh_.param<std::string>("imu_subscribe_topic_name",imu_subscribe_topic_name,"/ebimu_");
    pnh_.param<int>("calibration_frame_count", calibrationFrameCount, 40);
    frameCount = 0;

    // param - gps
    pnh_.param<std::string>("gps_subscribe_topic_name",gps_subscribe_topic_name,"/novatel_fix");
    pnh_.param<std::string>("gps_bestvel_subscribe_topic_name",gps_bestvel_subscribe_topic_name,"/bestvel");
    pnh_.param<std::string>("gps_bestpos_subscribe_topic_name",gps_bestpos_subscribe_topic_name,"/bestpos");
    pnh_.param<std::string>("initialpose_topic_name",initialpose_topic_name,"/initialpose");
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
    subInitialpose = nh_.subscribe(initialpose_topic_name,1,&Localizer::initialposeCallback,this);

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
    bKalmanInit = false;
    stable = ros::Time::now();

    std::cout.precision(15);

}

void Localizer::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ros::Time time_current = ros::Time::now();
    timeIMUelapsed = time_current.toSec() - timeIMUprev.toSec();
    double dt = timeIMUelapsed;

    imu = *msg;

    localHeading = normalize(localHeading + imu.angular_velocity.z * dt);
    wz_dt = normalize(wz_dt + imu.angular_velocity.z * dt);

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

    if(bKalmanInit)
    {
        filter();
    }

    if(erp.isERPavailable())
    {
        ROS_INFO("ERP MODE : %s", erp.getAorM().c_str());
        ROS_INFO("ERP ESTOP : %s", erp.getEstop().c_str());
        ROS_INFO("ERP GEAR : %s", erp.getGear().c_str());
        ROS_INFO("ERP ENC : %d", erp.getEncoder());
        ROS_INFO("ERP STATE : %s", erp.getState().c_str());
        ROS_INFO("ERP STEER : %d", erp.getSteer());
        ROS_INFO("ERP VEL(m/s) : %f\n", erp.getVelocity());
    }

    tf2::Quaternion q_gps;
    q_gps.setRPY(0,0,gpsData(KFIDX::YAW));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "raw";
    marker.id = markerId++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = gpsData(KFIDX::X) - utmoffsetX;
    marker.pose.position.y = gpsData(KFIDX::Y) - utmoffsetY;
    marker.pose.position.z = 0;
    marker.pose.orientation = tf2::toMsg(q_gps);
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    if(markerId > 200) markerId = 0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    pubMarker.publish(marker);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tStamp;
    tStamp.header.stamp = ros::Time::now();
    tStamp.header.frame_id = "map";
    tStamp.child_frame_id = "gps_raw";
    tStamp.transform.translation.x = gpsData(KFIDX::X) - utmoffsetX;
    tStamp.transform.translation.y = gpsData(KFIDX::Y) - utmoffsetY;
    tStamp.transform.translation.z = 0;
    tStamp.transform.rotation = tf2::toMsg(q_gps);
    br.sendTransform(tStamp);

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

void Localizer::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    double roll, pitch, yaw;

    tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
                );
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    std::cout << "RPY : " << roll << ' ' << pitch << ' ' << yaw << '\n';

    Eigen::MatrixXd X_curr = Eigen::MatrixXd::Zero(dim_kf,1);
    Eigen::MatrixXd P_curr = Eigen::MatrixXd::Zero(dim_kf,dim_kf);

    X_curr(KFIDX::X) = msg->pose.pose.position.x + utmoffsetX;
    X_curr(KFIDX::Y) = msg->pose.pose.position.y + utmoffsetY;;
    X_curr(KFIDX::YAW) = yaw;
    X_curr(KFIDX::VX) = 0;
    P_curr(KFIDX::X, KFIDX::X) = 1.0;
    P_curr(KFIDX::Y, KFIDX::Y) = 1.0;
    P_curr(KFIDX::YAW, KFIDX::YAW) = 0.0;
    P_curr(KFIDX::VX, KFIDX::VX) = 0.0;
    kf_.init(X_curr, P_curr);
    bKalmanInit = true;

    localHeading = yaw;

    ROS_INFO("Kalman Filter Initialized");
}

void Localizer::filter()
{
    // rostime display
    ros::Time time_current = ros::Time::now();
    timeGPSelapsed = time_current.toSec() - timeGPSprev.toSec();
    ROS_INFO("TIME ELAPSED : %f", timeGPSelapsed);

    // predict && update
    predict();
    update();

    Eigen::MatrixXd result;
    Eigen::MatrixXd P_result;

    kf_.getX(result);
    kf_.getP(P_result);

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

    marker_filtered.header.frame_id = "map";
    marker_filtered.header.stamp = ros::Time::now();
    marker_filtered.ns = "filtered";
    marker_filtered.id = markerId++;
    marker_filtered.type = visualization_msgs::Marker::ARROW;
    marker_filtered.action = visualization_msgs::Marker::ADD;
    marker_filtered.pose.position.x = result(KFIDX::X) - utmoffsetX;
    marker_filtered.pose.position.y = result(KFIDX::Y) - utmoffsetY;
    marker_filtered.pose.position.z = 0;
    marker_filtered.pose.orientation = tf2::toMsg(q_filtered);
    marker_filtered.scale.x = 0.2;
    marker_filtered.scale.y = 0.1;
    marker_filtered.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!

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

    // motion model
    X_next(KFIDX::X) = X_curr(KFIDX::X) + vx * cos(yaw) * dt;  // dx = v * cos(yaw)
    X_next(KFIDX::Y) = X_curr(KFIDX::Y) + vx * sin(yaw) * dt;  // dy = v * sin(yaw)
    X_next(KFIDX::YAW) = X_curr(KFIDX::YAW) + yaw_dt;
    X_next(KFIDX::DYAW) = yaw_dt;
    X_next(KFIDX::VX) = vx;

    if(erp.getState() == "STOP")
    {
        X_next(KFIDX::YAW) = X_curr(KFIDX::YAW);
        X_next(KFIDX::DYAW) = 0;
        X_next(KFIDX::VX) = 0;
    }

    X_next(KFIDX::YAW) = normalize(X_next(KFIDX::YAW));

    // jacobian of motion model
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_kf,dim_kf);
    A(KFIDX::X, KFIDX::YAW) = -1 * vx * dt * std::sin(yaw);
    A(KFIDX::X, KFIDX::VX) = dt * std::cos(yaw);
    A(KFIDX::Y, KFIDX::YAW) = -1 * vx * dt * std::cos(yaw);
    A(KFIDX::Y, KFIDX::VX) = dt * std::sin(yaw);
    A(KFIDX::YAW, KFIDX::DYAW) = 1;

    // process noise
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

    Q(KFIDX::YAW, KFIDX::YAW) = 0.0005;         // for yaw
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

void Localizer::update()
{

    Eigen::MatrixXd X_next = Eigen::MatrixXd::Zero(dim_kf,1);
    Eigen::MatrixXd P_next = Eigen::MatrixXd::Zero(dim_kf,dim_kf);

    kf_.getX(X_next);
    kf_.getP(P_next);

    // measurement
    int dim_meas = 8;
    Eigen::MatrixXd Z_ = Eigen::MatrixXd::Zero(dim_meas, 1);

    double heading = normalize(localHeading + tf2::getYaw(qYawBias));
    double erpwzdt = erp.getVelocity() / 1.04 * std::tan(toRadian(((-1 * erp.getSteer() + 75) / 71.0))) * timeGPSelapsed;
    double gpsheading = gpsData(KFIDX::YAW);

    // yaw error
    double yawError = heading - X_next(KFIDX::YAW);
    yawError = toDegree(std::abs(yawError));

    if(yawError > 300)
    {
        ROS_ERROR("YAW ERROR : %f, heading : %f, next : %f", yawError , toDegree(heading), toDegree(X_next(KFIDX::YAW)));
        // normalize again
        if (heading < 0) heading += 2*M_PI;
        else if (heading > 0) heading -= 2*M_PI;
    }
    else
    {
        ROS_INFO("YAW ERROR : %f", toDegree(yawError));
    }

    double gpsyawError = gpsheading - X_next(KFIDX::YAW);
    gpsyawError = toDegree(std::abs(gpsyawError));

    if(gpsyawError > 300)
    {
        ROS_ERROR("GPSYAW ERROR : %f, gpsheading : %f, next : %f", gpsyawError , toDegree(gpsheading), toDegree(X_next(KFIDX::YAW)));
        // normalize again
        if (gpsheading < 0) gpsheading += 2*M_PI;
        else if (gpsheading > 0) gpsheading -= 2*M_PI;
    }
    else
    {
        ROS_INFO("gpsYAW ERROR : %f", toDegree(gpsyawError));
    }

    if (wz_dt == 0) wz_dt = X_next(KFIDX::DYAW);

    Z_ << gpsData(GPSIDX::X)
          , gpsData(GPSIDX::Y)
            , gpsheading
            , heading
            , wz_dt
            , erpwzdt
            , gpsData(GPSIDX::V)
            , erp.getVelocity();

    if (erp.getState() == "STOP")
    {
        Z_(4) = 0;
        Z_(5) = 0;
        Z_(6) = 0;
        Z_(7) = 0;
    }

    std::cout << "Z : \t"
              << Z_(0) << " "
              << Z_(1) << "\n\t"
              << toDegree(Z_(2)) << " "
              << toDegree(Z_(3)) << "\n\t"
              << toDegree(Z_(4)) << " "
              << toDegree(Z_(5)) << "\n\t"
              << Z_(6) << " "
              << Z_(7) << " "
              << "\n";

    std::cout << "\t" << toDegree(tf2::getYaw(qYawBias)) << " " << toDegree(localHeading) << std::endl;

    wz_dt = 0;

    getCovariance(wzdtSample, wzdtCov);

    std::string postype = bestpos.position_type;
    ROS_INFO("POSTYPE : %s", postype.c_str());
    ROS_INFO("WZDTCOV : %f", wzdtCov(0,0));
    ROS_INFO("ERPSTEERCOV : %f", erp.getSteerCovariance());
    ROS_INFO("ERPVELCOV : %f", erp.getVelocityCovariance());

    // measurement model
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_meas, dim_kf);
    C(KFIDX::X, KFIDX::X) = 1.0;
    C(KFIDX::Y, KFIDX::Y) = 1.0;
    C(2, KFIDX::YAW) = 1.0;
    C(3, KFIDX::YAW) = 1.0;
    C(4, KFIDX::DYAW) = 1.0;
    C(5, KFIDX::DYAW) = 1.0;
    C(6, KFIDX::VX) = 1.0;
    C(7, KFIDX::VX) = 1.0;

    // measurement noise
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_meas, dim_meas);
    R(KFIDX::X, KFIDX::X) = gps.position_covariance[0] / 10.0;
    R(KFIDX::Y, KFIDX::Y) = gps.position_covariance[4] / 10.0;
    R(KFIDX::YAW, KFIDX::YAW) = gpsCov(GPSIDX::YAW, GPSIDX::YAW);
    R(3, 3) = wzdtCov(0, 0) * 50;  // imu heading
    R(4, 4) = wzdtCov(0, 0) * 50;  // wz*dt imu
    R(5, 5) = 0.005;
    R(6, 6) = gpsCov(GPSIDX::V, GPSIDX::V);   // vel gps
    R(7, 7) = 0.005;   // vel erp

    // imu fail
    if (!bIMUavailable)
    {
        ROS_ERROR("IMU DISABLE");
        R(3, 3) = 10000;
        R(4, 4) = 10000;
    }
    else
    {
        ROS_INFO("IMU ENABLE");
    }

    ROS_INFO("R \tX :\t%f", R(0,0));
    ROS_INFO("R \tY :\t%f", R(1,1));
    ROS_INFO("R YAWGPS :\t%f", R(2, 2));
    ROS_INFO("R YAWIMU :\t%f", R(3, 3));
    ROS_INFO("R WZDTIMU :\t%f", R(4, 4));
    ROS_INFO("R WZDTERP :\t%f", R(5, 5));
    ROS_INFO("R VELGPS :\t%f", R(6, 6));
    ROS_INFO("R VELERP :\t%f", R(7, 7));

    // fix yaw bias
    double yawVariance = gpsCov(GPSIDX::YAW, GPSIDX::YAW); // to degree
    double gate_yaw = 0.025;
    if (yawVariance < gate_yaw * gate_yaw) // gps yaw estimation stable
    {
        ROS_INFO("GOOD GPS YAW : %f, var : %f",toDegree(gpsData(GPSIDX::YAW)), yawVariance);
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
        ROS_WARN("SUCK GPS YAW : %f, var : %f",toDegree(gpsData(GPSIDX::YAW)), yawVariance);
    }

    // update kalman filter

    // default
    marker_filtered.color.r = 1.0f;
    marker_filtered.color.g = 0.0f;
    marker_filtered.color.b = 0.0f;
    marker_filtered.color.a = 1.0;

    if (erp.getState() == "STOP")
    {
        // gps has big variance when stop
        stable = ros::Time::now();
        R(KFIDX::X, KFIDX::X) = 10000;
        R(KFIDX::Y, KFIDX::Y) = 10000;
    }
    else
    {
        double tmp_time = ros::Time::now().toSec() - stable.toSec();
        if (tmp_time < 1)
        {
            // gps has big variance when depart
            R(KFIDX::X, KFIDX::X) = 1000;
            R(KFIDX::Y, KFIDX::Y) = 1000;
            marker_filtered.color.r = 0.0f;
            marker_filtered.color.g = 0.0f;
            marker_filtered.color.b = 1.0f;
            marker_filtered.color.a = 1.0;
        }
        else if(tmp_time < 2)
        {
            // gps has big variance when depart
            R(KFIDX::X, KFIDX::X) = 100;
            R(KFIDX::Y, KFIDX::Y) = 100;
            marker_filtered.color.r = 0.0f;
            marker_filtered.color.g = 1.0f;
            marker_filtered.color.b = 1.0f;
            marker_filtered.color.a = 1.0;
        }
        else if(tmp_time < 3)
        {
            // gps has big variance when depart
            R(KFIDX::X, KFIDX::X) = 10;
            R(KFIDX::Y, KFIDX::Y) = 10;
            marker_filtered.color.r = 1.0f;
            marker_filtered.color.g = 0.0f;
            marker_filtered.color.b = 1.0f;
            marker_filtered.color.a = 1.0;
        }
    }
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
    marker_filtered.type = visualization_msgs::Marker::ARROW;
    marker_filtered.action = visualization_msgs::Marker::ADD;
    marker_filtered.pose.position.x = result(KFIDX::X) - utmoffsetX;
    marker_filtered.pose.position.y = result(KFIDX::Y) - utmoffsetY;
    marker_filtered.pose.position.z = 0;
    marker_filtered.pose.orientation = tf2::toMsg(q_filtered);
    marker_filtered.scale.x = 0.2;
    marker_filtered.scale.y = 0.1;
    marker_filtered.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!

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
