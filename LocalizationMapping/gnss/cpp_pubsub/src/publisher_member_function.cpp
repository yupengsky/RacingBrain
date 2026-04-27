// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <fcntl.h>
#include <math.h>
#include <cmath>
#include <linux/netlink.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <ctime>
#include <thread>
#include <iostream>
#include "stream.h"
#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"
#include "gnss_ins_msg/msg/gnssins.hpp"
#include "gnss_ins_msg/msg/gnssins64.hpp"     // double精度消息头文件
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#define RMC_PREFIX "$"
#define RMC_MAX_LEN (1024)
#define RMC_BUF_SIZ (RMC_MAX_LEN * 8)
#define MAX_NG_COUNT (20)

#define HEAD "AA44AA45"
#define HEADLEN 8

using namespace std;

using namespace std::chrono_literals;
Stream *Serial_stream = NULL;

typedef struct
{

  int GNSS_Status;
  int NSV1;
  int NSV2;
  float online_ax;
  float online_ay;
  float online_az;
  float online_gx;
  float online_gy;
  float online_gz;
  float IMU_T;
  float gnss_height;
  float gnss_vel_n;
  float gnss_vel_e;
  float gnss_vel_u;
  float gnss_heading;
  float gnss_course;
  float accuracy_horizon;
  float accuracy_height;
  float accuracy_horizon_velocity;
  float accuracy_vertical_velocity;
  float accuracy_horizon_posture;
  float accuracy_yaw;
  float gnss_pos_delay;
  float gnss_vel_delay;
  float gnss_heading_delay;
  double gnss_long;
  double gnss_lati;
  double gnss_height_double;
  float gnss_pos_time;
  float pitch;
  float roll;
  float yaw;
  uint8_t ins_status;
  uint16_t gnss_week;
  uint32_t gnss_second;


} info_t;

bool nVerbose = false;
uint32_t ng_cnt = 0;
uint32_t ok_cnt = 0;

// 定义GPS起始时间为1980年1月6日0时0分0秒
const static double gpst0[] = {1980, 1, 6, 0, 0, 0}; 
// 定义闰秒数
#define LEAPS 18

// 定义时间结构体
typedef struct { 
    time_t time; 
    double sec; 
} gtime_t;

// 根据给定的日期和时间计算标准时间_t表示的时间
gtime_t epoch2time(const double* ep) {
    // 每个月的累计天数（考虑闰年）
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}; 
    gtime_t time = {0};
    int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

    // 检查输入的日期是否在有效范围内
    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) {
        return time; 
    }

    // 计算从1970年1月1日到给定日期的天数
    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3? 1 : 0);
    sec = (int)floor(ep[5]);
    time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
    time.sec = ep[5] - sec;
    return time;
}

// 将GPS周和周内秒转换为标准时间_t表示的时间
gtime_t gpst2time(int week, double sec) {
    gtime_t t = epoch2time(gpst0);

    // 检查周内秒是否在合理范围内
    if (sec < -1E9 || 1E9 < sec) {
        sec = 0.0;
    }

    t.time += 86400 * 7 * week + (int)sec;
    t.sec = sec - (int)sec;
    return t;
}

// 对时间进行秒数的加减操作
gtime_t timeadd(gtime_t t, double sec) {
    double tt;
    t.sec += sec;
    tt = floor(t.sec);
    t.time += (int)tt;
    t.sec -= tt;
    return t;
}

// 将GPS时间转换为UTC时间
gtime_t GPSTime2UTCTime(int week, double sec, double leapsec) {
    gtime_t gpst = gpst2time(week, sec);
    return timeadd(gpst, -leapsec);
}

static uint8_t crc8(unsigned char data, unsigned int crc)
{
  uint8_t crc8 = (0x107 & 0xFF);
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (crc & 0x80)
    {
      crc <<= 1;
      crc ^= crc8;
    }
    else
    {
      crc <<= 1;
    }
  }
  return crc;
}

uint8_t Calculate_Crc8(unsigned char *buf, uint16_t len)
{
  uint8_t crc = 0x00;
  while (len--)
  {
    crc = crc8(*buf, crc);
    if (len > 0x00)
    {
      buf++;
    }
  }
  return crc;
}

char *findhead(char *src)
{
  char *str = src - 1;
  char *end = src + strlen(src) - 1;
  while (end - str > 0)
  {

    str++;
    if ((int)(*str) == -86)
    {
      str++;
      if ((int)(*str) == 68)
      {
        str++;
        if ((int)(*str) == -86)
        {
          str++;
          if ((int)(*str) == 69)
          {
            return str - 3;
          }
        }
      }
    }
  }
  return NULL;
}

void supersplit(const std::string &s, std::vector<std::string> &v, const std::string &c)
{
  std::string::size_type pos1, pos2;
  size_t len = s.length();
  pos2 = s.find(c);
  pos1 = 0;
  while (std::string::npos != pos2)
  {
    if ("" == s.substr(pos1, pos2 - pos1))
    {

      v.emplace_back("0");
    }
    else
    {

      v.emplace_back(s.substr(pos1, pos2 - pos1));
    }

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if (pos1 != len)
    v.emplace_back(s.substr(pos1));
}

#define DEG_TO_RAD_LOCAL (M_PI / 180.0)
#define WGS84_TEXT "+proj=latlong +ellps=WGS84"
const double PI = 3.14159265358979323846;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<gnss_ins_msg::msg::Gnssins>("gongji_gnss_ins", 10);
    // 新增发布包含double精度经纬度消息
    publisher_64_ = this->create_publisher<gnss_ins_msg::msg::Gnssins64>("gongji_gnss_ins_64", 10);
    
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    vbody_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("body_velocity", 10);
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    //std::cout<<"test1"<<std::endl;
    char *RMC_buf = new char[RMC_BUF_SIZ];
    char *RMC_ptr = RMC_buf;
    char *RMC_end = RMC_buf + RMC_BUF_SIZ - 1;
    info_t tInfo;
    int num = 0;
    while(1){
      ssize_t r = Serial_stream->read((uint8_t *)RMC_ptr, 172);


            if (r <= 0)
            { ///
            //std::cout<<"test2"<<std::endl;
                continue;
            }




            RMC_ptr[r] = 0;
            RMC_ptr += r;
            char *find_ptr = RMC_buf;
            char *beg_ptr;

//std::cout<<"test3"<<std::endl;
            while (RMC_ptr - find_ptr > 0)
            {
                beg_ptr = findhead(find_ptr);
                // beg_ptr = strst/r(find_ptr,HEAD);
                if (beg_ptr)
                {
                  //std::cout<<"test4"<<std::endl;
                    break;
                }
                find_ptr++;
            }

//std::cout<<"test5"<<std::endl;

            if (!beg_ptr)
            {//std::cout<<"test6"<<std::endl;
                if (nVerbose)
                {
                    printf("####find_ptr,%s####\n", find_ptr);
                    printf("[gpsd] no begin string, continue.\n");
                }

                if ((RMC_ptr - RMC_buf) > sizeof(HEAD))
                {
                    memmove(RMC_buf, RMC_ptr - sizeof(HEAD), sizeof(HEAD));
                    RMC_ptr = RMC_buf + sizeof(HEAD);
                    *RMC_ptr = 0;
                }

                continue;
            }



            find_ptr = beg_ptr + 8;
            char *end_ptr;
            while (RMC_ptr - find_ptr > 0)
            {//std::cout<<"test7"<<std::endl;
                end_ptr = findhead(find_ptr);
                if (end_ptr)
                {
                    break;
                }
                find_ptr++;
            }



            if (end_ptr)
            {//std::cout<<"test88888888"<<std::endl;
                if (end_ptr - beg_ptr == 172)
                {//std::cout<<"test9999999999"<<std::endl;
                    uint8_t *crc_p = (uint8_t *)beg_ptr;
                    if (Calculate_Crc8(crc_p, 171) == *(uint8_t *)(beg_ptr + 171))
                    {
                        //std::cout<<"test1010101010"<<std::endl;
                        tInfo.GNSS_Status = *(uint8_t *)(beg_ptr + 35);
                        //std::cout<<"GNSS_Status"<<std::endl;
                        tInfo.NSV1 = *(uint8_t *)(beg_ptr + 36);
                        //std::cout<<"NSV1"<<std::endl;
                        tInfo.NSV2 = *(uint8_t *)(beg_ptr + 37);
                        //std::cout<<"NSV2"<<std::endl;
                        tInfo.online_ax = *(int16_t *)(beg_ptr + 38) * (8 / pow(2, 15));
                        //std::cout<<"online_ax"<<std::endl;
                        tInfo.online_ay = *(int16_t *)(beg_ptr + 40) * (8 / pow(2, 15));
                        //std::cout<<"online_ay"<<std::endl;
                        tInfo.online_az = *(int16_t *)(beg_ptr + 42) * (8 / pow(2, 15));
                        //std::cout<<"online_az"<<std::endl;
                        tInfo.online_gx = *(int16_t *)(beg_ptr + 44) * (1e2 / pow(2, 15));
                        //std::cout<<"online_gx"<<std::endl;
                        tInfo.online_gy = *(int16_t *)(beg_ptr + 46) * (1e2 / pow(2, 15));
                        //std::cout<<"online_gy"<<std::endl;
                        tInfo.online_gz = *(int16_t *)(beg_ptr + 48) * (1e2 / pow(2, 15));
                        //std::cout<<"cout"<<std::endl;
                        tInfo.IMU_T = *(int16_t *)(beg_ptr + 50) * (150 / pow(2, 15));
                        //std::cout<<"cout"<<std::endl;
                        tInfo.gnss_vel_e = *(int16_t *)(beg_ptr + 16) * (1e2 / pow(2, 15));
                        //std::cout<<"gnss_vel_e"<<std::endl;
                        tInfo.gnss_vel_n = *(int16_t *)(beg_ptr + 18) * (1e2 / pow(2, 15));
                        tInfo.gnss_vel_u = *(int16_t *)(beg_ptr + 20) * (1e2 / pow(2, 15));
                        tInfo.gnss_height = *(int32_t *)(beg_ptr + 12) * (1e-3);
                        tInfo.gnss_lati = *(int32_t *)(beg_ptr + 4) * (1e-7);
                        tInfo.gnss_long = *(int32_t *)(beg_ptr + 8) * (1e-7);
                        tInfo.pitch = *(int16_t *)(beg_ptr + 22) * (180 / pow(2, 15));
                        tInfo.roll = *(int16_t *)(beg_ptr + 24) * (180 / pow(2, 15));
                        tInfo.yaw = *(int16_t *)(beg_ptr + 26) * (180 / pow(2, 15));
                        tInfo.ins_status = *(uint8_t *)(beg_ptr + 28);
                        tInfo.gnss_week = *(uint16_t *)(beg_ptr + 29);
                        tInfo.gnss_second = *(uint32_t *)(beg_ptr + 31);
                        tInfo.accuracy_horizon = *(uint16_t *)(beg_ptr + 52) * (100 / pow(2, 16));
                        tInfo.accuracy_height = *(uint16_t *)(beg_ptr + 54) * (100 / pow(2, 16));
                        tInfo.accuracy_horizon_velocity = *(uint16_t *)(beg_ptr + 56) * (100 / pow(2, 16));
                        tInfo.accuracy_vertical_velocity = *(uint16_t *)(beg_ptr + 58) * (100 / pow(2, 16));
                        tInfo.accuracy_horizon_posture = *(uint16_t *)(beg_ptr + 60) * (100 / pow(2, 16));
                        tInfo.accuracy_yaw = *(uint16_t *)(beg_ptr + 62) * (100 / pow(2, 16));
                        // parseGONGJI(&tInfo);
                        //std::cout<<"test1010101010wwwwwwwwwwwwwwwwwwwwwwwwww"<<std::endl;
                        parseGONGJI(&tInfo);
                    }
                }
                else
                {
                    std::cout << "It is not a valid info" << std::endl;
                }
                size_t s = RMC_ptr - end_ptr;
                memmove(RMC_buf, end_ptr, s);
                RMC_ptr = RMC_buf + s;
            }
            else
            {
                size_t s = RMC_ptr - beg_ptr;
                memmove(RMC_buf, beg_ptr, s);
                RMC_ptr = RMC_buf + s;
            }
      
      }


     //timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::parseGONGJI, this, std::placeholders::_1));


  }

private:
  void parseGONGJI(const info_t *info)//  void timer_callback()
  {
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    
    // publisher_->publish(message);

    // 旧消息
    gnss_ins_msg::msg::Gnssins gnss_ins_data;
    gnss_ins_data.latitude  = info->gnss_lati;
    gnss_ins_data.longitude = info->gnss_long;
    gnss_ins_data.height    = info->gnss_height;
    gnss_ins_data.vel_e     = info->gnss_vel_e;
    gnss_ins_data.vel_n     = info->gnss_vel_n;
    gnss_ins_data.vel_u     = info->gnss_vel_u;
    gnss_ins_data.pitch     = info->pitch;
    gnss_ins_data.roll      = info->roll;
    gnss_ins_data.yaw       = info->yaw;
    gnss_ins_data.ins_status     = info->ins_status;
    gnss_ins_data.gnss_week = info->gnss_week;
    gnss_ins_data.gnss_second = info->gnss_second;
    gnss_ins_data.gnss_status = info->GNSS_Status;
    gnss_ins_data.satellite_main = info->NSV1;
    gnss_ins_data.satellite_sub = info->NSV2;
    gnss_ins_data.imu_acc_x = info->online_ax;
    gnss_ins_data.imu_acc_y = info->online_ay;
    gnss_ins_data.imu_acc_z = info->online_az;
    gnss_ins_data.imu_gyro_x = info->online_gx;
    gnss_ins_data.imu_gyro_y = info->online_gy;
    gnss_ins_data.imu_gyro_z = info->online_gz;
    gnss_ins_data.imu_temp   = info->IMU_T;
    gnss_ins_data.accuracy_horizon = info->accuracy_horizon;
    gnss_ins_data.accuracy_height  = info->accuracy_height;
    gnss_ins_data.accuracy_horizon_velocity  = info->accuracy_horizon_velocity;
    gnss_ins_data.accuracy_vertical_velocity  = info->accuracy_vertical_velocity;
    gnss_ins_data.accuracy_horizon_posture  = info->accuracy_horizon_posture;
    gnss_ins_data.accuracy_yaw  = info->accuracy_yaw;

    //新消息
    gnss_ins_msg::msg::Gnssins64 gnss_ins_data_64;
    gnss_ins_data_64.latitude  = info->gnss_lati; 
    gnss_ins_data_64.longitude = info->gnss_long; 
    gnss_ins_data_64.height    = info->gnss_height; 
    gnss_ins_data_64.vel_e     = info->gnss_vel_e;
    gnss_ins_data_64.vel_n     = info->gnss_vel_n;
    gnss_ins_data_64.vel_u     = info->gnss_vel_u;
    gnss_ins_data_64.pitch     = info->pitch;
    gnss_ins_data_64.roll      = info->roll;
    gnss_ins_data_64.yaw       = info->yaw;
    gnss_ins_data_64.ins_status     = info->ins_status;
    gnss_ins_data_64.gnss_week = info->gnss_week;
    gnss_ins_data_64.gnss_second = info->gnss_second;
    gnss_ins_data_64.gnss_status = info->GNSS_Status;
    gnss_ins_data_64.satellite_main = info->NSV1;
    gnss_ins_data_64.satellite_sub = info->NSV2;
    gnss_ins_data_64.imu_acc_x = info->online_ax;
    gnss_ins_data_64.imu_acc_y = info->online_ay;
    gnss_ins_data_64.imu_acc_z = info->online_az;
    gnss_ins_data_64.imu_gyro_x = info->online_gx;
    gnss_ins_data_64.imu_gyro_y = info->online_gy;
    gnss_ins_data_64.imu_gyro_z = info->online_gz;
    gnss_ins_data_64.imu_temp   = info->IMU_T;
    gnss_ins_data_64.accuracy_horizon = info->accuracy_horizon;
    gnss_ins_data_64.accuracy_height  = info->accuracy_height;
    gnss_ins_data_64.accuracy_horizon_velocity  = info->accuracy_horizon_velocity;
    gnss_ins_data_64.accuracy_vertical_velocity  = info->accuracy_vertical_velocity;
    gnss_ins_data_64.accuracy_horizon_posture  = info->accuracy_horizon_posture;
    gnss_ins_data_64.accuracy_yaw  = info->accuracy_yaw;

    unsigned short gpsweek = info->gnss_week;
    double gpssec = ((double)info->gnss_second) * 1e-3;
    gtime_t GPSTime = GPSTime2UTCTime(gpsweek, gpssec, LEAPS);

    gnss_ins_data.header.stamp.sec  = (int)GPSTime.time;
    gnss_ins_data.header.stamp.nanosec  = (uint)(GPSTime.sec * 1e9);

    // 设置新消息时间 (与旧消息一致)
    gnss_ins_data_64.header = gnss_ins_data.header;

    publisher_->publish(gnss_ins_data);
    publisher_64_->publish(gnss_ins_data_64);
    // std::cout<<"testqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq"<<std::endl;
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", gnss_ins_data.imu_acc_x);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%u", gnss_ins_data.gnss_status);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%u'", gnss_ins_data.gnss_week);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%u'", gnss_ins_data.gnss_second);
    // time_t ttt = GPSTime.time - 28800;
    // struct tm* gpstimeinfo = localtime(&ttt);
    // RCLCPP_INFO(this->get_logger(),"timestamp sec = %d", gnss_ins_data.header.stamp.sec);
    // RCLCPP_INFO(this->get_logger(),"timestamp nanosec = %d", gnss_ins_data.header.stamp.nanosec);
    // RCLCPP_INFO(this->get_logger(),"year=%d,month=%d,day=%d,hour=%d,min=%d,sec=%d\n", 
    //        gpstimeinfo->tm_year + 1900, gpstimeinfo->tm_mon + 1, gpstimeinfo->tm_mday, 
    //        gpstimeinfo->tm_hour, gpstimeinfo->tm_min, gpstimeinfo->tm_sec); 

    Eigen::Matrix3d C_yaw;
    C_yaw = Eigen::AngleAxisd(-(double)gnss_ins_data.yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d C_pitch;
    C_pitch = Eigen::AngleAxisd(-(double)gnss_ins_data.pitch * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d C_roll;
    C_roll = Eigen::AngleAxisd(-(double)gnss_ins_data.roll * M_PI / 180.0, Eigen::Vector3d::UnitY());

    Eigen::Matrix3d C_n_imu = C_roll * C_pitch * C_yaw;
    Eigen::Vector3d v_n((double)gnss_ins_data.vel_e, (double)gnss_ins_data.vel_n, (double)gnss_ins_data.vel_u);
    Eigen::Vector3d v_imu = C_n_imu * v_n;

    double angle = -90.0 * M_PI / 180.0;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ(); // UnitZ() 返回一个 (0, 0, 1) 的向量
    Eigen::Matrix3d C_imu_b;
    C_imu_b = Eigen::AngleAxisd(angle, axis);
    Eigen::Vector3d v_b = C_imu_b * v_imu;

    geometry_msgs::msg::Vector3Stamped vbody_msg;
    vbody_msg.header = gnss_ins_data.header;
    vbody_msg.vector.x = v_b.x();
    vbody_msg.vector.y = v_b.y();
    vbody_msg.vector.z = v_b.z();
    vbody_pub_->publish(vbody_msg);

    tf2::Quaternion q_tf;
    q_tf.setRPY((double)gnss_ins_data.roll * M_PI / 180.0, (double)gnss_ins_data.pitch * M_PI / 180.0, (double)gnss_ins_data.yaw * M_PI / 180.0);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg = tf2::toMsg(q_tf); // 使用 tf2_geometry_msgs 中的转换函数

    geometry_msgs::msg::Vector3 angular_velocity;
    angular_velocity.x = (double)gnss_ins_data.imu_gyro_y;
    angular_velocity.y = -(double)gnss_ins_data.imu_gyro_x;
    angular_velocity.z = (double)gnss_ins_data.imu_gyro_z;

    geometry_msgs::msg::Vector3 linear_acceleration;
    linear_acceleration.x = (double)gnss_ins_data.imu_acc_y;
    linear_acceleration.y = -(double)gnss_ins_data.imu_acc_x;
    linear_acceleration.z = (double)gnss_ins_data.imu_acc_z;

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header = gnss_ins_data.header;
    imu_msg.orientation = q_msg;
    imu_msg.angular_velocity = angular_velocity;
    imu_msg.linear_acceleration = linear_acceleration;
    imu_pub_->publish(imu_msg);

  }


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<gnss_ins_msg::msg::Gnssins>::SharedPtr publisher_;
  // 新增的高精度发布者
  rclcpp::Publisher<gnss_ins_msg::msg::Gnssins64>::SharedPtr publisher_64_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vbody_pub_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  int32_t ret = 0;
  int module_type = 0;
  std::string serial_dev = "/dev/ttyUSB0";
  int baudrate = 460800;
  
  Serial_stream = Stream::create_serial(serial_dev.c_str(), baudrate);
  while (1)
  {

    bool ret = Serial_stream->Connect();
    if (ret)
    {
      break;
    }
    sleep(1);
  }
  if (1 == module_type)
  {
  }

  


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
