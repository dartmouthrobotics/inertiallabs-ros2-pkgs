#include <iostream>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

//Inertial Labs source header
#include "ILDriver.h"

//adding message type headers
#include <inertiallabs_msgs/msg/sensor_data.hpp>
#include <inertiallabs_msgs/msg/ins_data.hpp>
#include <inertiallabs_msgs/msg/gps_data.hpp>
#include <inertiallabs_msgs/msg/gnss_data.hpp>
#include <inertiallabs_msgs/msg/marine_data.hpp>

//publish as sensor_msgs/msg/Imu
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

//Publishers

struct Context {
    rclcpp::Node::SharedPtr node;

    rclcpp::Publisher<inertiallabs_msgs::msg::GnssData>::SharedPtr gnssDataPub;
    rclcpp::Publisher<inertiallabs_msgs::msg::GpsData>::SharedPtr gpsDataPub;
    rclcpp::Publisher<inertiallabs_msgs::msg::InsData>::SharedPtr insDataPub;
    rclcpp::Publisher<inertiallabs_msgs::msg::MarineData>::SharedPtr marineDataPub;
    rclcpp::Publisher<inertiallabs_msgs::msg::SensorData>::SharedPtr sensorDataPub;

    //publish as sensor_msgs/msg/Imu
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuDataPub;

    std::string imuFrameId;
};

tf2::Quaternion rpyToQuaternion(double r, double p, double y)
{
  double roll  = r * M_PI / 180.0;
  double pitch = p * M_PI / 180.0;
  double yaw   = y * M_PI / 180.0;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);  // roll, pitch, yaw

  return q;
}

void publishDevice(IL::INSDataStruct* data, void* contextPtr)
{
    Context* context = reinterpret_cast<Context*>(contextPtr);

    const rclcpp::Time timestamp = context->node->now();

    if (context->sensorDataPub->get_subscription_count() > 0)
    {
        inertiallabs_msgs::msg::SensorData msg_sensor_data;
        msg_sensor_data.header.stamp = timestamp;
        msg_sensor_data.header.frame_id = context->imuFrameId;
        msg_sensor_data.mag.x = data->Mag[0];
        msg_sensor_data.mag.y = data->Mag[1];
        msg_sensor_data.mag.z = data->Mag[2];
        msg_sensor_data.accel.x = data->Acc[0];
        msg_sensor_data.accel.y = data->Acc[1];
        msg_sensor_data.accel.z = data->Acc[2];
        msg_sensor_data.gyro.x = data->Gyro[0];
        msg_sensor_data.gyro.y = data->Gyro[1];
        msg_sensor_data.gyro.z = data->Gyro[2];
        msg_sensor_data.temp = data->Temp;
        msg_sensor_data.vinp = data->VSup;
        msg_sensor_data.pressure = data->hBar;
        msg_sensor_data.barometric_height = data->pBar;
        context->sensorDataPub->publish(msg_sensor_data);
    }

    if (context->insDataPub->get_subscription_count() > 0)
    {
        inertiallabs_msgs::msg::InsData msg_ins_data;
        msg_ins_data.header.stamp = timestamp;
        msg_ins_data.header.frame_id = context->imuFrameId;
        msg_ins_data.ypr.x = data->Heading;
        msg_ins_data.ypr.y = data->Pitch;
        msg_ins_data.ypr.z = data->Roll;
        msg_ins_data.ori_quat.w = data->Quat[0];
        msg_ins_data.ori_quat.x = data->Quat[1];
        msg_ins_data.ori_quat.y = data->Quat[2];
        msg_ins_data.ori_quat.z = data->Quat[3];
        msg_ins_data.llh.x = data->Latitude;
        msg_ins_data.llh.y = data->Longitude;
        msg_ins_data.llh.z = data->Altitude;
        msg_ins_data.vel_enu.x = data->VelENU[0];
        msg_ins_data.vel_enu.y = data->VelENU[1];
        msg_ins_data.vel_enu.z = data->VelENU[2];
        msg_ins_data.gps_ins_time = data->GPS_INS_Time;
        msg_ins_data.gps_imu_time = data->GPS_IMU_Time;
        msg_ins_data.gps_msow.data = data->ms_gps;
        msg_ins_data.solution_status.data = data->INSSolStatus;
        msg_ins_data.usw = data->USW;
        msg_ins_data.pos_std.x = data->KFLatStd;
        msg_ins_data.pos_std.y = data->KFLonStd;
        msg_ins_data.pos_std.z = data->KFAltStd;
        msg_ins_data.heading_std = data->KFHdgStd;
        context->insDataPub->publish(msg_ins_data);
    }

    if (context->gpsDataPub->get_subscription_count() > 0)
    {
        inertiallabs_msgs::msg::GpsData msg_gps_data;
        msg_gps_data.header.stamp = timestamp;
        msg_gps_data.header.frame_id = context->imuFrameId;
        msg_gps_data.llh.x = data->LatGNSS;
        msg_gps_data.llh.y = data->LonGNSS;
        msg_gps_data.llh.z = data->AltGNSS;
        msg_gps_data.hor_speed = data->V_Hor;
        msg_gps_data.speed_dir = data->Trk_gnd;
        msg_gps_data.ver_speed = data->V_ver;
        context->gpsDataPub->publish(msg_gps_data);
    }

    if (context->gnssDataPub->get_subscription_count() > 0)
    {
        inertiallabs_msgs::msg::GnssData msg_gnss_data;
        msg_gnss_data.header.stamp = timestamp;
        msg_gnss_data.header.frame_id = context->imuFrameId;
        msg_gnss_data.gnss_info_1 = data->GNSSInfo1;
        msg_gnss_data.gnss_info_2 = data->GNSSInfo2;
        msg_gnss_data.number_sat = data->SVsol;
        msg_gnss_data.gnss_velocity_latency = data->GNSSVelLatency;
        msg_gnss_data.gnss_angles_position_type = data->AnglesType;
        msg_gnss_data.gnss_heading = data->Heading_GNSS;
        msg_gnss_data.gnss_pitch = data->Pitch_GNSS;
        msg_gnss_data.gnss_gdop = data->GDOP;
        msg_gnss_data.gnss_pdop = data->PDOP;
        msg_gnss_data.gnss_hdop = data->HDOP;
        msg_gnss_data.gnss_vdop = data->VDOP;
        msg_gnss_data.gnss_tdop = data->TDOP;
        msg_gnss_data.new_gnss_flags = data->NewGPS;
        msg_gnss_data.diff_age = data->DiffAge;
        msg_gnss_data.pos_std.x = data->LatGNSSStd;
        msg_gnss_data.pos_std.y = data->LonGNSSStd;
        msg_gnss_data.pos_std.z = data->AltGNSSStd;
        msg_gnss_data.heading_std = data->HeadingGNSSStd;
        msg_gnss_data.pitch_std = data->PitchGNSSStd;
        context->gnssDataPub->publish(msg_gnss_data);
    }

    if (context->marineDataPub->get_subscription_count() > 0)
    {
        inertiallabs_msgs::msg::MarineData msg_marine_data;
        msg_marine_data.header.stamp = timestamp;
        msg_marine_data.header.frame_id = context->imuFrameId;
        msg_marine_data.heave = data->Heave;
        msg_marine_data.surge = data->Surge;
        msg_marine_data.sway = data->Sway;
        msg_marine_data.heave_velocity = data->Heave_velocity;
        msg_marine_data.surge_velocity = data->Surge_velocity;
        msg_marine_data.sway_velocity = data->Sway_velocity;
        msg_marine_data.significant_wave_height = data->significant_wave_height;
        context->marineDataPub->publish(msg_marine_data);
    }

    if (context->imuDataPub->get_subscription_count() > 0)
    {
        sensor_msgs::msg::Imu imu_msg;
        // header
        imu_msg.header.stamp = timestamp;
        imu_msg.header.frame_id = context->imuFrameId;

        // orientation 
        tf2::Quaternion q = rpyToQuaternion(data->Roll, data->Pitch, data->Heading);
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        // linear_acceleration (m/s^2) 
        imu_msg.linear_acceleration.x = data->Acc[0];
        imu_msg.linear_acceleration.y = data->Acc[1];
        imu_msg.linear_acceleration.z = data->Acc[2];

        // angular_velocity (rad/s)
        imu_msg.angular_velocity.x = data->Gyro[0] * M_PI / 180.0;
        imu_msg.angular_velocity.y = data->Gyro[1] * M_PI / 180.0;
        imu_msg.angular_velocity.z = data->Gyro[2] * M_PI / 180.0;

        context->imuDataPub->publish(imu_msg);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("il_ins", options);

    std::string port("serial:/dev/ttyUSB0:460800");
    int insOutputFormat = 82; // 0x52

    Context context;
    IL::Driver ins;

    // Command line parameters
    if (node->has_parameter("ins_url")) {
        port = node->get_parameter("ins_url").as_string();
    }
    if (node->has_parameter("ins_output_format")) {
        insOutputFormat = node->get_parameter("ins_output_format").as_int();
    }

    context.node = node;

    // Initializing Publishers
    context.sensorDataPub = node->create_publisher<inertiallabs_msgs::msg::SensorData>("Inertial_Labs/sensor_data", 10);
    context.insDataPub = node->create_publisher<inertiallabs_msgs::msg::InsData>("Inertial_Labs/ins_data", 10);
    context.gpsDataPub = node->create_publisher<inertiallabs_msgs::msg::GpsData>("Inertial_Labs/gps_data", 10);
    context.gnssDataPub = node->create_publisher<inertiallabs_msgs::msg::GnssData>("Inertial_Labs/gnss_data", 10);
    context.marineDataPub = node->create_publisher<inertiallabs_msgs::msg::MarineData>("Inertial_Labs/marine_data", 10);
    context.imuDataPub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // Communication with the device
    RCLCPP_INFO(node->get_logger(), "Connecting to INS at URL %s\n", port.c_str());

    int il_err = ins.connect(port.c_str());
    if (il_err)
    {
        RCLCPP_FATAL(node->get_logger(), "Could not connect to the INS on this URL %s\n", port.c_str());
        exit(EXIT_FAILURE);
    }

    if (ins.isStarted())
    {
        ins.stop();
    }

    const IL::INSDeviceInfo devInfo = ins.getDeviceInfo();
    const IL::INSDevicePar devParams = ins.getDeviceParams();

    const std::string serialNumber(reinterpret_cast<const char *>(devInfo.IDN), 8);
    RCLCPP_INFO(node->get_logger(), "Found INS S/N %s\n", serialNumber.c_str());
    context.imuFrameId = serialNumber;

    #ifdef __linux__
    bool enableRealtime = node->declare_parameter<bool>("enable_realtime_priority", false);
    if (enableRealtime) {
        // Optional: attempt real-time priority for low-latency sensor handling
        struct sched_param sp;
        sp.sched_priority = 99;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
            RCLCPP_WARN(node->get_logger(), "Could not set realtime priority: %s", strerror(errno));
        }
    }
    #endif

    il_err = ins.start(insOutputFormat);
    if (il_err)
    {
        RCLCPP_FATAL(node->get_logger(), "Could not start the INS: %i\n", il_err);
        ins.disconnect();
        exit(EXIT_FAILURE);
    }

    ins.setCallback(&publishDevice, &context);

    RCLCPP_INFO(node->get_logger(), "Publishing at %d Hz\n", devParams.dataRate);
    RCLCPP_INFO(node->get_logger(), "Run \"rostopic list\" to see all topics.\nRun \"rostopic echo <topic-name>\" to see the data from sensor");

    rclcpp::spin(node);
    std::cout << "Stopping INS... " << std::flush;
    ins.stop();
    std::cout << "Disconnecting... " << std::flush;
    ins.disconnect();
    std::cout << "Done." << std::endl;
    return 0;
}
