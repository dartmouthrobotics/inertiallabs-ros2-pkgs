#include <chrono>
#include <cstring>
#include <iostream>
#include <math.h>
#include <sched.h>
#include <stdlib.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Inertial Labs source header
#include "ILDriver.h"

// Message type headers
#include <inertiallabs_msgs/msg/gnss_data.hpp>
#include <inertiallabs_msgs/msg/gps_data.hpp>
#include <inertiallabs_msgs/msg/ins_data.hpp>
#include <inertiallabs_msgs/msg/marine_data.hpp>
#include <inertiallabs_msgs/msg/sensor_data.hpp>

// Publish as sensor_msgs/msg/Imu
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

class IL_INS : public rclcpp::Node
{
public:
    explicit IL_INS(const rclcpp::NodeOptions & options)
        : Node("il_ins", options)
    {
        // Keep parameters aligned with launch files and standalone usage.
        this->declare_parameter("ins_url", "serial:/dev/ttyUSB0:921600");
        this->declare_parameter("ins_output_format", 149);
        this->declare_parameter("frame_id", "");
        this->declare_parameter("enable_realtime_priority", false);
        this->declare_parameter("use_device_time", true);
        this->declare_parameter("publisher_queue_depth", 200);

        insUrl_ = this->get_parameter("ins_url").as_string();
        insOutputFormat_ = this->get_parameter("ins_output_format").as_int();
        frameIdParam_ = this->get_parameter("frame_id").as_string();
        enableRealtimePriority_ = this->get_parameter("enable_realtime_priority").as_bool();
        useDeviceTime_ = this->get_parameter("use_device_time").as_bool();
        publisherQueueDepth_ = this->get_parameter("publisher_queue_depth").as_int();

        if (publisherQueueDepth_ < 1) {
            RCLCPP_WARN(this->get_logger(), "publisher_queue_depth must be >= 1. Using 1.");
            publisherQueueDepth_ = 1;
        }

        auto pub_qos = rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(publisherQueueDepth_));

        sensorDataPub_ = this->create_publisher<inertiallabs_msgs::msg::SensorData>(
            "Inertial_Labs/sensor_data", pub_qos);
        insDataPub_ = this->create_publisher<inertiallabs_msgs::msg::InsData>(
            "Inertial_Labs/ins_data", pub_qos);
        gpsDataPub_ = this->create_publisher<inertiallabs_msgs::msg::GpsData>(
            "Inertial_Labs/gps_data", pub_qos);
        gnssDataPub_ = this->create_publisher<inertiallabs_msgs::msg::GnssData>(
            "Inertial_Labs/gnss_data", pub_qos);
        marineDataPub_ = this->create_publisher<inertiallabs_msgs::msg::MarineData>(
            "Inertial_Labs/marine_data", pub_qos);
        imuDataPub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", pub_qos);

        if (!initializeIns()) {
            // Keep container alive and retry periodically if device is unavailable at startup.
            reconnectTimer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                [this]() {
                    if (connected_) {
                        reconnectTimer_->cancel();
                        return;
                    }
                    RCLCPP_WARN(this->get_logger(), "Retrying INS connection...");
                    if (initializeIns()) {
                        reconnectTimer_->cancel();
                    }
                });
        }
    }

    ~IL_INS() override
    {
        if (connected_) {
            RCLCPP_INFO(this->get_logger(), "Stopping INS...");
            ins_.stop();
            RCLCPP_INFO(this->get_logger(), "Disconnecting...");
            ins_.disconnect();
            RCLCPP_INFO(this->get_logger(), "Done.");
        }
    }

private:
    static tf2::Quaternion rpyToQuaternion(double r, double p, double y)
    {
        double roll = r * M_PI / 180.0;
        double pitch = p * M_PI / 180.0;
        double yaw = y * M_PI / 180.0;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return q;
    }

    bool initializeIns()
    {
        RCLCPP_INFO(this->get_logger(), "Connecting to INS at URL %s", insUrl_.c_str());

        int il_err = ins_.connect(insUrl_.c_str());
        if (il_err) {
            RCLCPP_ERROR(
                this->get_logger(), "Could not connect to the INS on this URL %s", insUrl_.c_str());
            connected_ = false;
            return false;
        }

        if (ins_.isStarted()) {
            ins_.stop();
        }

        const IL::INSDeviceInfo devInfo = ins_.getDeviceInfo();
        const IL::INSDevicePar devParams = ins_.getDeviceParams();

        const std::string serialNumber(reinterpret_cast<const char *>(devInfo.IDN), 8);
        RCLCPP_INFO(this->get_logger(), "Found INS S/N %s", serialNumber.c_str());
        imuFrameId_ = frameIdParam_.empty() ? serialNumber : frameIdParam_;

#ifdef __linux__
        if (enableRealtimePriority_) {
            struct sched_param sp;
            sp.sched_priority = 99;
            if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
                RCLCPP_WARN(this->get_logger(), "Could not set realtime priority: %s", strerror(errno));
            }
        }
#endif

        il_err = ins_.start(insOutputFormat_);
        if (il_err) {
            RCLCPP_ERROR(this->get_logger(), "Could not start the INS: %i", il_err);
            ins_.disconnect();
            connected_ = false;
            return false;
        }

        timeInitialized_ = false;
        ins_.setCallback(&IL_INS::staticPublishDevice, this);

        RCLCPP_INFO(this->get_logger(), "Publishing at %d Hz", devParams.dataRate);
        connected_ = true;
        return true;
    }

    static void staticPublishDevice(IL::INSDataStruct * data, void * contextPtr)
    {
        IL_INS * self = reinterpret_cast<IL_INS *>(contextPtr);
        self->publishDevice(data);
    }

    void publishDevice(IL::INSDataStruct * data)
    {
        rclcpp::Time timestamp;
        if (useDeviceTime_) {
            if (!timeInitialized_) {
                rosTimeStart_ = this->now();
                deviceTimeStart_ = data->GPS_IMU_Time;
                timeInitialized_ = true;
                timestamp = rosTimeStart_;
            } else {
                const double device_time_delta = data->GPS_IMU_Time - deviceTimeStart_;
                timestamp = rosTimeStart_ + rclcpp::Duration::from_seconds(device_time_delta);
            }
        } else {
            timestamp = this->now();
        }

        if (sensorDataPub_->get_subscription_count() > 0) {
            inertiallabs_msgs::msg::SensorData msg_sensor_data;
            msg_sensor_data.header.stamp = timestamp;
            msg_sensor_data.header.frame_id = imuFrameId_;
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
            sensorDataPub_->publish(msg_sensor_data);
        }

        if (insDataPub_->get_subscription_count() > 0) {
            inertiallabs_msgs::msg::InsData msg_ins_data;
            msg_ins_data.header.stamp = timestamp;
            msg_ins_data.header.frame_id = imuFrameId_;
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
            insDataPub_->publish(msg_ins_data);
        }

        if (gpsDataPub_->get_subscription_count() > 0) {
            inertiallabs_msgs::msg::GpsData msg_gps_data;
            msg_gps_data.header.stamp = timestamp;
            msg_gps_data.header.frame_id = imuFrameId_;
            msg_gps_data.llh.x = data->LatGNSS;
            msg_gps_data.llh.y = data->LonGNSS;
            msg_gps_data.llh.z = data->AltGNSS;
            msg_gps_data.hor_speed = data->V_Hor;
            msg_gps_data.speed_dir = data->Trk_gnd;
            msg_gps_data.ver_speed = data->V_ver;
            gpsDataPub_->publish(msg_gps_data);
        }

        if (gnssDataPub_->get_subscription_count() > 0) {
            inertiallabs_msgs::msg::GnssData msg_gnss_data;
            msg_gnss_data.header.stamp = timestamp;
            msg_gnss_data.header.frame_id = imuFrameId_;
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
            gnssDataPub_->publish(msg_gnss_data);
        }

        if (marineDataPub_->get_subscription_count() > 0) {
            inertiallabs_msgs::msg::MarineData msg_marine_data;
            msg_marine_data.header.stamp = timestamp;
            msg_marine_data.header.frame_id = imuFrameId_;
            msg_marine_data.heave = data->Heave;
            msg_marine_data.surge = data->Surge;
            msg_marine_data.sway = data->Sway;
            msg_marine_data.heave_velocity = data->Heave_velocity;
            msg_marine_data.surge_velocity = data->Surge_velocity;
            msg_marine_data.sway_velocity = data->Sway_velocity;
            msg_marine_data.significant_wave_height = data->significant_wave_height;
            marineDataPub_->publish(msg_marine_data);
        }

        if (imuDataPub_->get_subscription_count() > 0) {
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = timestamp;
            imu_msg.header.frame_id = imuFrameId_;

            tf2::Quaternion q = rpyToQuaternion(data->Roll, data->Pitch, data->Heading);
            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            imu_msg.linear_acceleration.x = data->Acc[0];
            imu_msg.linear_acceleration.y = data->Acc[1];
            imu_msg.linear_acceleration.z = data->Acc[2];

            imu_msg.angular_velocity.x = data->Gyro[0] * M_PI / 180.0;
            imu_msg.angular_velocity.y = data->Gyro[1] * M_PI / 180.0;
            imu_msg.angular_velocity.z = data->Gyro[2] * M_PI / 180.0;

            imuDataPub_->publish(imu_msg);
        }
    }

    rclcpp::Publisher<inertiallabs_msgs::msg::SensorData>::SharedPtr sensorDataPub_;
    rclcpp::Publisher<inertiallabs_msgs::msg::InsData>::SharedPtr insDataPub_;
    rclcpp::Publisher<inertiallabs_msgs::msg::GpsData>::SharedPtr gpsDataPub_;
    rclcpp::Publisher<inertiallabs_msgs::msg::GnssData>::SharedPtr gnssDataPub_;
    rclcpp::Publisher<inertiallabs_msgs::msg::MarineData>::SharedPtr marineDataPub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuDataPub_;

    IL::Driver ins_;

    std::string imuFrameId_;
    std::string insUrl_;
    std::string frameIdParam_;
    int insOutputFormat_{149};
    int publisherQueueDepth_{200};
    bool enableRealtimePriority_{false};
    bool useDeviceTime_{true};
    bool connected_{false};

    rclcpp::Time rosTimeStart_;
    double deviceTimeStart_{0.0};
    bool timeInitialized_{false};

    rclcpp::TimerBase::SharedPtr reconnectTimer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(IL_INS)

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IL_INS>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}