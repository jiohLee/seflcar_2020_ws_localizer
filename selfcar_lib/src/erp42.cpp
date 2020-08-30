#include "selfcar_lib//erp42.h"

ERP42::ERP42(ros::NodeHandle &node, ros::NodeHandle &prv_node)
    :nh_(node)
    ,pnh_(prv_node)
{
    std::string aormTopicName   = "/MSG_CON/Rx_AorM";
    std::string estopTopicName  = "/MSG_CON/Rx_Estop";
    std::string gearTopicName   = "/MSG_CON/Rx_Gear";
    std::string breakTopicName  = "/MSG_CON/Rx_Break";
    std::string velTopicName    = "/MSG_CON/Rx_Vel";
    std::string steerTopicName  = "/MSG_CON/Rx_Steer";
    std::string encTopicName    = "/MSG_CON/Rx_Enc";

    subAorM = nh_.subscribe(aormTopicName, 1, &ERP42::aormCallback, this);
    subEstop = nh_.subscribe(estopTopicName, 1, &ERP42::estopCallback, this);
    subGear = nh_.subscribe(gearTopicName, 1, &ERP42::gearCallback, this);
    subBreak = nh_.subscribe(breakTopicName, 1, &ERP42::breakCallback, this);
    subVel = nh_.subscribe(velTopicName, 1, &ERP42::velCallback, this);
    subSteer = nh_.subscribe(steerTopicName, 1, &ERP42::steerCallback, this);
    subEnc = nh_.subscribe(encTopicName, 1, &ERP42::encCallback, this);

    aormEnable = false;
    estopEnable = false;
    gearEnable = false;
    breakEnable = false;
    velEnable = false;
    steerEnable = false;
    encEbable = false;
    state = IDX::STOP;

    steerSample = Eigen::MatrixXd::Zero(10,1);
    steerCov = Eigen::MatrixXd::Zero(1,1);
    velSample = Eigen::MatrixXd::Zero(10,1);
    velCov = Eigen::MatrixXd::Zero(1,1);
}

bool ERP42::isStop()
{
    if (vel.data < 6) return true;
    return false;
}

bool ERP42::isERPavailable() { return velEnable; }

const std::string ERP42::getAorM()
{
    if(aormEnable)
    {
        std::string data = aorm.data;
        if( data[0] == 0x00) return "MANUAL";
        else if (data[0] == 0x01) return "AUTO";
        else return "INVALIDDATA";
    }
    else
    {
        return "NODATA";
    }
}

const std::string ERP42::getEstop()
{
    if(estopEnable)
    {
        std::string data = estop.data;
        if( data[0] == 0x00) return "ESTOPOFF";
        else if (data[0] == 0x01) return "ESTOPON";
        else return "INVALIDDATA";
    }
    else
    {
        return "NODATA";
    }
}

const std::string ERP42::getGear()
{
    if(gearEnable)
    {
        std::string data = gear.data;
        if( data[0] == 0x00) return "FORWARD";
        else if (data[0] == 0x01) return "NEUTRAL";
        else if (data[0] == 0x02) return "BACKWARD";
        else return "INVALIDDATA";
    }
    else
    {
        return "NODATA";
    }
}

const int ERP42::getBreak()
{
    if(breakEnable)
    {
        int data = brk.data;
        return data;
    }
    else
    {
        return 200;
    }
}

const double ERP42::getVelocity()
{
    if(velEnable)
    {
        if( state == IDX::STOP) return 0;
        else if( state == IDX::BACKWARD) return vel.data / 10.0 * 1000.0 / 3600.0 * -1;
        else return vel.data / 10.0 * 1000.0 / 3600.0;
    }
    else
    {
        return 0;
    }
}

const double ERP42::getVelocityCovariance()
{
    if (velCov(0) != 0 ) return velCov(0);
    return 0.0005;
}

const int ERP42::getSteer() { return steer.data; }

const double ERP42::getSteerCovariance()
{
    if (steerCov(0) != 0) return steerCov(0);
    return 0.0005;
}

const int ERP42::getEncoder()
{
    if (encEbable) return enc.data;
    else return 0;
}

const std::string ERP42::getState()
{
    switch (state)
    {
    case IDX::FORWARD: return "FORWARD";
    case IDX::STOP: return "STOP";
    case IDX::BACKWARD: return "BACKWARD";
    case IDX::GO: return "GO";
    default:
        break;
    }
}

void ERP42::aormCallback(const std_msgs::String::ConstPtr &msg)
{
    aormEnable = true;
    aorm = *msg;
}

void ERP42::estopCallback(const std_msgs::String::ConstPtr &msg)
{
    estopEnable = true;
    estop = *msg;
}

void ERP42::gearCallback(const std_msgs::String::ConstPtr &msg)
{
    gearEnable = true;
    gear = *msg;
}

void ERP42::breakCallback(const std_msgs::Int8::ConstPtr &msg)
{
    breakEnable = true;
    brk = *msg;
}

void ERP42::velCallback(const std_msgs::Int16::ConstPtr &msg)
{
    vel = *msg;
    if(!velEnable)
    {
        velSample = Eigen::MatrixXd::Constant(10,1,vel.data / 10 * 1000 / 3600);
    }

    if (vel.data == 0)
    {
        state = IDX::STOP;
    }
    else
    {
        if(!encEbable)
        {
            state = IDX::GO;
        }
    }

    Eigen::MatrixXd Z_ = Eigen::MatrixXd::Zero(1,1);
    Z_ << vel.data / 10 * 1000.0 / 3600.0;
    getCovariance(velSample, Z_, velCov);

    velEnable = true;
}

void ERP42::steerCallback(const std_msgs::Int16::ConstPtr &msg)
{
    steer = *msg;
    if (!steerEnable)
    {
        steerSample = Eigen::MatrixXd::Constant(10,1,steer.data / 71);
    }

    Eigen::MatrixXd Z_ = Eigen::MatrixXd::Zero(1,1);
    Z_ << steer.data / 71;
    getCovariance(steerSample, Z_, steerCov);

    steerEnable = true;
}

void ERP42::encCallback(const std_msgs::Int32::ConstPtr &msg)
{
    encEbable = true;
    std_msgs::Int32 tmp = *msg;
    if(tmp.data > 3000)
    {
        if(0 < tmp.data - enc.data  && tmp.data - enc.data < 100)
        {
            state = IDX::FORWARD;
        }
        else if(-100 < tmp.data - enc.data  && tmp.data - enc.data < 0)
        {
            state = IDX::BACKWARD;
        }
    }
    enc = *msg;
}
