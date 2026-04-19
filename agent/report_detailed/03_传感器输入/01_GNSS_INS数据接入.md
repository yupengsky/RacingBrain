# GNSS/INS 数据接入

## 1. 模块位置

GNSS/INS 相关代码位于：

```text
gnss/
├── gnss_ins_msg/
└── cpp_pubsub/
```

其中：

- `gnss_ins_msg` 定义消息。
- `cpp_pubsub` 从串口读取并发布消息。

核心文件：

```text
gnss/gnss_ins_msg/msg/Gnssins.msg
gnss/gnss_ins_msg/msg/Gnssins64.msg
gnss/cpp_pubsub/src/publisher_member_function.cpp
gnss/cpp_pubsub/src/serial_stream.cpp
gnss/cpp_pubsub/include/cpp_pubsub/stream.h
```

## 2. 为什么有 Gnssins 和 Gnssins64 两种消息

`Gnssins.msg` 中经纬度是：

```text
float32 latitude
float32 longitude
```

`Gnssins64.msg` 中经纬度是：

```text
float64 latitude
float64 longitude
```

SLAM 使用的是 `Gnssins64`，因为经纬度非常敏感，`float32` 会带来明显精度损失。对于以 UTM 米级坐标建图的系统，几厘米到几十厘米误差都会影响锥桶地图。

## 3. SLAM 实际使用的字段

SLAM 节点包含头文件：

```cpp
#include "gnss_ins_msg/msg/gnssins64.hpp"
```

实际读取字段：

```text
header
latitude
longitude
roll
pitch
yaw
vel_e
vel_n
imu_gyro_z
```

字段用途：

- `header.stamp`：同步和发布 TF/Odom/Path。
- `latitude`、`longitude`：经纬度转 UTM，得到车辆位置。
- `roll`、`pitch`、`yaw`：构造车辆姿态。
- `vel_e`、`vel_n`：计算速度，并发布车体系速度。
- `imu_gyro_z`：估计 yaw rate，用于动态噪声和阿克曼 ROI。

## 4. 串口桥接节点

节点源码：

```text
gnss/cpp_pubsub/src/publisher_member_function.cpp
```

构建出的可执行程序：

```text
talker
```

主函数中硬编码：

```cpp
std::string serial_dev = "/dev/ttyUSB0";
int baudrate = 460800;
Serial_stream = Stream::create_serial(serial_dev.c_str(), baudrate);
```

也就是说，在线使用时默认读取：

```text
/dev/ttyUSB0
```

波特率：

```text
460800
```

## 5. 串口底层封装

串口抽象接口在：

```text
gnss/cpp_pubsub/include/cpp_pubsub/stream.h
```

实现位于：

```text
gnss/cpp_pubsub/src/serial_stream.cpp
```

核心接口：

```cpp
virtual bool Connect() = 0;
virtual bool Disconnect() = 0;
virtual size_t read(uint8_t *buffer, size_t max_length) = 0;
virtual size_t write(const uint8_t *buffer, size_t length) = 0;
```

`Stream::create_serial` 根据波特率创建 `SerialStream`。`SerialStream` 内部使用 Linux termios 配置串口：

- 非阻塞打开。
- raw mode。
- 8 位数据。
- 1 位停止位。
- 无校验。
- 无流控。

## 6. 数据帧查找

惯导二进制帧头定义：

```cpp
#define HEAD "AA44AA45"
#define HEADLEN 8
```

但代码实际找的是字节序列：

```cpp
0xAA 0x44 0xAA 0x45
```

函数：

```cpp
char *findhead(char *src)
```

逻辑：

1. 在缓冲区中逐字节扫描。
2. 如果连续发现 `-86, 68, -86, 69`，返回帧头位置。
3. 找不到则返回 `NULL`。

因为 `char` 可能是有符号类型，所以 `0xAA` 会显示为 `-86`。

## 7. 数据帧长度和 CRC

代码判断完整帧：

```cpp
if (end_ptr - beg_ptr == 172)
```

也就是说每帧长度为：

```text
172 字节
```

CRC 校验：

```cpp
if (Calculate_Crc8(crc_p, 171) == *(uint8_t *)(beg_ptr + 171))
```

也就是说：

- 前 171 字节参与 CRC。
- 第 172 字节是 CRC 校验位。

## 8. 字段解析

代码直接根据偏移读取字段，例如：

```cpp
tInfo.gnss_lati = *(int32_t *)(beg_ptr + 4) * (1e-7);
tInfo.gnss_long = *(int32_t *)(beg_ptr + 8) * (1e-7);
tInfo.gnss_height = *(int32_t *)(beg_ptr + 12) * (1e-3);
tInfo.gnss_vel_e = *(int16_t *)(beg_ptr + 16) * (1e2 / pow(2, 15));
tInfo.gnss_vel_n = *(int16_t *)(beg_ptr + 18) * (1e2 / pow(2, 15));
tInfo.pitch = *(int16_t *)(beg_ptr + 22) * (180 / pow(2, 15));
tInfo.roll  = *(int16_t *)(beg_ptr + 24) * (180 / pow(2, 15));
tInfo.yaw   = *(int16_t *)(beg_ptr + 26) * (180 / pow(2, 15));
```

惯导状态：

```cpp
tInfo.ins_status = *(uint8_t *)(beg_ptr + 28);
tInfo.gnss_week = *(uint16_t *)(beg_ptr + 29);
tInfo.gnss_second = *(uint32_t *)(beg_ptr + 31);
tInfo.GNSS_Status = *(uint8_t *)(beg_ptr + 35);
```

IMU：

```cpp
tInfo.online_ax = *(int16_t *)(beg_ptr + 38) * (8 / pow(2, 15));
tInfo.online_ay = *(int16_t *)(beg_ptr + 40) * (8 / pow(2, 15));
tInfo.online_az = *(int16_t *)(beg_ptr + 42) * (8 / pow(2, 15));
tInfo.online_gx = *(int16_t *)(beg_ptr + 44) * (1e2 / pow(2, 15));
tInfo.online_gy = *(int16_t *)(beg_ptr + 46) * (1e2 / pow(2, 15));
tInfo.online_gz = *(int16_t *)(beg_ptr + 48) * (1e2 / pow(2, 15));
```

精度信息：

```cpp
tInfo.accuracy_horizon = *(uint16_t *)(beg_ptr + 52) * (100 / pow(2, 16));
tInfo.accuracy_height = *(uint16_t *)(beg_ptr + 54) * (100 / pow(2, 16));
...
```

## 9. ROS 时间戳计算

原始帧提供：

```text
gnss_week
gnss_second
```

代码定义 GPS 起点：

```cpp
const static double gpst0[] = {1980, 1, 6, 0, 0, 0};
#define LEAPS 18
```

转换流程：

```text
GPS week + GPS second
  -> GPS time
  -> 减去闰秒
  -> UTC time
  -> ROS header.stamp
```

对应函数：

```cpp
epoch2time
gpst2time
timeadd
GPSTime2UTCTime
```

最终：

```cpp
gnss_ins_data.header.stamp.sec = (int)GPSTime.time;
gnss_ins_data.header.stamp.nanosec = (uint)(GPSTime.sec * 1e9);
gnss_ins_data_64.header = gnss_ins_data.header;
```

## 10. 发布消息

旧消息：

```cpp
publisher_ = this->create_publisher<gnss_ins_msg::msg::Gnssins>(
    "gongji_gnss_ins", 10);
```

新消息：

```cpp
publisher_64_ = this->create_publisher<gnss_ins_msg::msg::Gnssins64>(
    "gongji_gnss_ins_64", 10);
```

IMU：

```cpp
imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
```

车体系速度：

```cpp
vbody_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "body_velocity", 10);
```

## 11. 车体系速度计算

输入速度：

```text
vel_e
vel_n
vel_u
```

代码先构造导航系速度：

```cpp
Eigen::Vector3d v_n(vel_e, vel_n, vel_u);
```

然后根据 yaw、pitch、roll 构造旋转矩阵，把速度转入 IMU 系，再通过一个 -90 度旋转转入车体系：

```cpp
Eigen::Vector3d v_imu = C_n_imu * v_n;
Eigen::Matrix3d C_imu_b = Eigen::AngleAxisd(-90deg, Z);
Eigen::Vector3d v_b = C_imu_b * v_imu;
```

发布：

```cpp
vbody_msg.vector.x = v_b.x();
vbody_msg.vector.y = v_b.y();
vbody_msg.vector.z = v_b.z();
```

## 12. IMU 消息发布

姿态：

```cpp
q_tf.setRPY(roll, pitch, yaw);
imu_msg.orientation = tf2::toMsg(q_tf);
```

角速度轴向修正：

```cpp
angular_velocity.x = imu_gyro_y;
angular_velocity.y = -imu_gyro_x;
angular_velocity.z = imu_gyro_z;
```

线加速度轴向修正：

```cpp
linear_acceleration.x = imu_acc_y;
linear_acceleration.y = -imu_acc_x;
linear_acceleration.z = imu_acc_z;
```

## 13. 离线复现方式

如果没有真实惯导，可以直接用 rosbag 播放：

```bash
ros2 bag play <bag_dir> --topics /gongji_gnss_ins_64
```

只要 `/gongji_gnss_ins_64` 的字段完整，SLAM 不关心它来自串口还是 rosbag。

## 14. 最小复现伪代码

```cpp
while (running) {
    bytes = serial.read(buffer);
    append_to_ring_buffer(bytes);

    while (has_complete_frame(buffer)) {
        frame = extract_172_bytes(buffer);
        if (!crc_ok(frame)) continue;

        info.latitude = read_i32(frame, 4) * 1e-7;
        info.longitude = read_i32(frame, 8) * 1e-7;
        info.height = read_i32(frame, 12) * 1e-3;
        info.vel_e = read_i16(frame, 16) * scale_vel;
        info.vel_n = read_i16(frame, 18) * scale_vel;
        info.pitch = read_i16(frame, 22) * scale_angle;
        info.roll = read_i16(frame, 24) * scale_angle;
        info.yaw = read_i16(frame, 26) * scale_angle;

        msg64.header.stamp = gps_week_second_to_utc_ros_time(info);
        msg64.latitude = info.latitude;
        msg64.longitude = info.longitude;
        msg64.roll = info.roll;
        msg64.pitch = info.pitch;
        msg64.yaw = info.yaw;
        msg64.vel_e = info.vel_e;
        msg64.vel_n = info.vel_n;
        msg64.imu_gyro_z = info.gyro_z;

        pub_gnss64.publish(msg64);
    }
}
```

## 15. 实现注意点

1. 真实工程中不要长期硬编码 `/dev/ttyUSB0`，应改成 ROS 参数。
2. 二进制字段直接 `reinterpret_cast` 有大小端和内存对齐风险，不过在当前平台可工作。
3. `Gnssins64` 是 SLAM 推荐接口。
4. 离线数据回放可以绕过串口模块。
5. 如果地图出现整体跳变，应先检查 GNSS 时间戳、经纬度精度和 yaw 定义。

