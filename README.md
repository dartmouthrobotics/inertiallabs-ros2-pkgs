# inertiallabs_ros2_pkgs
Linux ROS driver for [Inertial Labs](https://inertiallabs.com/) products.
Supported devices: INS, IMU-P, AHRS, AHRS-10.

The package is developed based on the official `SDK v0.2` for Linux.

> **Note:** This repository contains local modifications relative to the [upstream repository](https://us.inertiallabs.com:31443/scm/ins/inertiallabs-ros2-pkgs.git). See [Changes from upstream](#changes-from-upstream) for details.

The package is tested on:
- [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) with Ubuntu 22.04 LTS
- [ROS2 Iron Irwini](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html) with Ubuntu 22.04 LTS

## Changes from upstream

- **Node architecture:** refactored from a free-function/main-driven node to a ROS2 composable component (`IL_INS`); supports both composable and standalone execution.
- **New parameters:** `frame_id`, `enable_realtime_priority`, `use_device_time`, `publisher_queue_depth`.
- **New topic:** `imu/data` (`sensor_msgs/msg/Imu`) with orientation, linear acceleration, and angular velocity.
- **Reconnect retry:** node retries connection every 5 seconds if the device is unavailable at startup instead of exiting.
- **Removed polling loop:** `rclcpp::Rate rate(100)` busy-wait loop removed; driver is now purely callback-driven.
- **Publisher queue depth:** increased from `1` to `200` (configurable via `publisher_queue_depth`).
- **Realtime priority:** optional `SCHED_FIFO` scheduling via `enable_realtime_priority` parameter (requires `sudo setcap cap_sys_nice+ep`).
- **Magnetometer indexing fix:** `Mag[0]`, `Mag[1]`, `Mag[2]` now correctly mapped to x, y, z (upstream used `Mag[0]` for all three).
- **Gyro unit conversion:** gyroscope values converted from deg/s to rad/s for `imu/data`.
- **SDK low-latency serial reads:** `SerialPort.cpp` sets `VMIN=0`, `VTIME=0` for non-blocking, low-latency serial reads.

## License
* The license for the official SDK is the MIT license.
* The license for the other codes is Apache 2.0 whenever not specified.

## Build
It builds by colcon build-system.
Make sure that your folder with ROS2 packages included in environment variable `ROS_PACKAGE_PATH`.
Clone your package in `<your_work_space>/src`.
And build with the Colcon.

```bash
cd <your_work_space>/src
git clone https://us.inertiallabs.com:31443/scm/ins/inertiallabs-ros2-pkgs.git
cd <your_work_space>
colcon build
```

## Run

### Standalone node
```bash
sudo chmod 666 /dev/ttyUSB0
source install/setup.bash
ros2 run inertiallabs_ins il_ins_node --ros-args -p ins_url:=serial:/dev/ttyUSB0:921600 -p ins_output_format:=149
```

For ins OPVT2AHR packet via USB serial port:
```bash
ros2 run inertiallabs_ins il_ins_node --ros-args -p ins_url:=serial:/dev/ttyUSB0:921600 -p ins_output_format:=88
```
For ins OPVT packet via UDP (INS hostname is used):
```bash
ros2 run inertiallabs_ins il_ins_node --ros-args -p ins_url:=udp:INS-F2001234:23 -p ins_output_format:=82
```
For ins OPVT packet via UDP (INS IP address is used):
```bash
ros2 run inertiallabs_ins il_ins_node --ros-args -p ins_url:=udp:192.168.0.249:23 -p ins_output_format:=82
```

### Composable node (loaded into an existing component container)
```bash
source install/setup.bash
ros2 launch inertiallabs_ins ins_composable.launch.py \
  container_name:=my_container \
  ins_url:=serial:/dev/ttyUSB0:921600 \
  ins_output_format:=149
```

### Launch file (standalone)
```bash
source install/setup.bash
ros2 launch inertiallabs_ins ins.launch
```

**Parameters**

`ins_url` (`string`, `default: serial:/dev/ttyUSB0:921600`)
Port the device is connected to. Can be:
- serial:[path to device]:[baudrate]
- tcp:[hostname or address]:[tcp server port]
- udp:[hostname or address]:[udp server port]

Inertial Labs Driver supports serial connection.

`ins_output_format` (`int`, `default: 149`)
The output data format of the INS data according to IL INS ICD.
```
 IL_IMU_Orientation         51  (0x33)
 IL_SENSOR_DATA             80  (0x50)
 IL_OPVT                    82  (0x52)
 IL_MINIMAL_DATA            83  (0x53)
 IL_QPVT                    86  (0x56)
 IL_OPVT2A                  87  (0x57)
 IL_OPVT2AHR                88  (0x58)
 IL_OPVT2AW                 89  (0x59)
 IL_OPVTAD                  97  (0x61)
 MRU_OPVTHSSHR              100 (0x64)
 IL_OPVT_RAWIMU_DATA        102 (0x66)
 IL_OPVT_GNSSEXT_DATA       103 (0x67)
 IL_USER_DEFINED_DATA       149 (0x95)
```

`frame_id` (`string`, `default: ""`)
Frame ID used in message headers. If empty, the device serial number is used automatically.

`enable_realtime_priority` (`bool`, `default: false`)
If `true`, the driver thread will attempt to acquire SCHED_FIFO realtime scheduling priority (Linux only).
Requires the executable to hold the `cap_sys_nice` capability. After each build, run:
```bash
sudo setcap cap_sys_nice+ep \
    install/inertiallabs_ins/lib/inertiallabs_ins/il_ins_node
```
Without this capability the driver will log a warning and continue without realtime priority.

`use_device_time` (`bool`, `default: true`)
If `true`, timestamps are derived from device-reported GPS/IMU time synchronized to the ROS clock at startup. If `false`, the ROS wall clock is used.

`publisher_queue_depth` (`int`, `default: 200`)
QoS history depth for all published topics.

**Published Topics**

Feel free to modify using fields from IL::INSDataStruct

`/Inertial_Labs/sensor_data`  
Gyro(x,y,z) , Accelation(x,y,z) , Magnetic (x,y,z) , Temprature , Input Voltage , Pressure , Barometric height.

`/Inertial_Labs/ins_data`  
GPS INS Time, GPS IMU Time, Millisecond of the week, Latitude, Longitude, Altitude, Heading , Pitch , Roll, Orientation quaternion, East Velocity, North Velocity, Up Velocity values, Solution status, Position STD, Heading STD, Unit Status.

`/Inertial_Labs/gps_data`  
Latitude, Longitude , Altitude , Ground Speed , Track Direction,  Vertical Speed values .

`/Inertial_Labs/gnss_data`  
GNSS service Info 1, Info 2, Satellites Used, Velocity Latency, Heading status, Heading, Pitch, GDOP, PDOP, HDOP, VDOP, TDOP, New GNSS Flag, Age of differenctiol correction, Position STD, Heading STD, Pitch STD.

`/Inertial_Labs/marine_data`  
Heave, Surge, Sway, Heave Velocity, Surge Velocity, Sway Velocity, Significant Wave Height.

`/imu/data` (`sensor_msgs/msg/Imu`)  
Orientation quaternion (from Roll/Pitch/Heading), linear acceleration (m/s²), angular velocity (rad/s).


## FAQ
1. **I use WSL2 with Linux and can't receive data from sensor.**\
You need to use [usbipd](https://learn.microsoft.com/en-us/windows/wsl/connect-usb#install-the-usbipd-win-project). It allows to transfer data from USB to WSL2. And `/dev/ttyUSBx` devices will appear in Linux.
Run PowerShell as Administrator and make something like:
```powershell
# Mount USB:
usbipd list
usbipd bind --busid 1-1
usbipd attach --wsl --busid 1-1

# Unmount USB:
usbipd detach --busid 1-1
```

2. **The driver can't open my serial device?**\
Make sure you have enough access to `/dev`.
```bash
sudo chmod 666 /dev/ttyUSB0
```

3. **Why I have permission error during the initialization process of the driver?**\
Most often, this is because the baud rate you set does not match the package size to be received. Try increase the baud rate.
```bash
sudo stty -F /dev/ttyUSB0 460800
ros2 run inertiallabs_ins il_ins_node --ros-args -p ins_url:=serial:/dev/ttyUSB0:460800 -p ins_output_format:=88
```

4. **How can I check data from sensor?**\
Be sure, that Inertial Labs node has the topic-subscribers. Because messages will not send with no topic subscribers!

Print topic example:
In the separate windows run
```bash
source install/setup.bash
ros2 topic list
ros2 topic echo /Inertial_Labs/sensor_data
```

5. **Why is the IMU data output rate much lower than what is set?**\
This may be due to a recent change in the FTDI USB-Serial driver in the Linux kernel, the following shell script might help:
```bash
# Reduce latency in the FTDI serial-USB kernel driver to 1ms
# This is required due to https://github.com/torvalds/linux/commit/c6dce262
for file in $(ls /sys/bus/usb-serial/devices/); do
  value=`cat /sys/bus/usb-serial/devices/$file/latency_timer`
  if [ $value -gt 1 ]; then
    echo "Setting low_latency mode for $file"
    sudo sh -c "echo 1 > /sys/bus/usb-serial/devices/$file/latency_timer"
  fi
done
```

6. **Why a field value is always zero?**\
Most likely, because this field is not provided in the selected INS data packet. The most versatile data packet is User-Defined Data, which allows to order any set of fields

## Bug Report
Prefer to open an issue. You can also send an E-mail to support@inertiallabs.com.
