#include "selfcar_lib//erp42.h"

ERP42::ERP42(ros::NodeHandle &node, ros::NodeHandle &prv_node)
    :nh_(node)
    ,pnh_(prv_node)
{
    std::string serial_vel_subscribe_topic_name;
    std::string serial_enc_subscribe_topic_name;
    pnh_.param<std::string>("serial_vel_subscribe_topic_name", serial_vel_subscribe_topic_name, "/MSG_CON/Rx_Vel");
    pnh_.param<std::string>("serial_enc_subscribe_topic_name", serial_enc_subscribe_topic_name, "/MSG_CON/Rx_Enc");

    subVel = nh_.subscribe(serial_vel_subscribe_topic_name, 1, &ERP42::velCallback, this);
    subEnc = nh_.subscribe(serial_enc_subscribe_topic_name, 1, &ERP42::encCallback, this);

    velEnable = false;
    encEbable = false;
    state = IDX::STOP;
}

bool ERP42::isStop()
{
    if (vel.data < 6) return true;
    return false;
}

bool ERP42::isERPavailable() { return velEnable; }

const double ERP42::getVelocity()
{
    if( state == IDX::STOP) return 0;
    else if( state == IDX::BACKWARD) return vel.data / 10.0 * 1000.0 / 3600.0 * -1;
    else return vel.data / 10.0 * 1000.0 / 3600.0;
}


const int ERP42::getEncoderValue() { return enc.data; }

const int ERP42::getState() { return state; }

void ERP42::velCallback(const std_msgs::Int16::ConstPtr &msg)
{
    velEnable = true;
    std_msgs::Int16 tmp = *msg;
    if (tmp.data == 0)
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
    vel = *msg;
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
