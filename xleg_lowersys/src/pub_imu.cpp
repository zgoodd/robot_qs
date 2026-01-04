/*
HI219出厂默认输出协议接收:
输出 sum = 41
0x5A+0xA5+LEN_LOW+LEN_HIGH+CRC_LOW+CRC_HIGH+ 0x90+ID(1字节) + 0xA0+Acc(加速度6字节) + 0xB0+Gyo(角速度6字节) + 0xC0+Mag(地磁6字节) + 0xD0 +AtdE(欧拉角6字节) + 0xF0+Pressure(压力4字节)
*/
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include "xleg_msgs/BodyState.h"
#include <serial/serial.h>
//读取连接到机器人的IMU（Inertial Measurement Unit）向串口发送的数据
using namespace std;
using namespace boost::asio;

#define MAX_PACKET_LEN          (41)// length of the data


float Eular[3];
serial::Serial ser_imu;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bodystate_read_node");
    ros::NodeHandle n;
    ros::Publisher bodystate_pub = n.advertise<xleg_msgs::BodyState>("xleg/bodystate", 10);
    xleg_msgs::BodyState bodySta;
//设置节点的发布者

    try
    {
        ser_imu.setPort("/dev/ttyUSB0");
        ser_imu.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_imu.setTimeout(to);
        ser_imu.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open IMU port!");
        return 0;
    }

    if(ser_imu.isOpen())
    {
        ROS_INFO("Serial Port for IMU_Sensor has been initialized.");
    }


    int count = 0;
    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        // 向串口读数据
        uint8_t buf_tmp[1];
        uint8_t buf[MAX_PACKET_LEN-1];

        ser_imu.read(buf_tmp, 1);

        if(buf_tmp[0] == 0x5A )//首字节为0x5A时开始读取,判断表头
        {
            ser_imu.read(buf, MAX_PACKET_LEN-1);

            int i=0;
            if(buf[i] == 0xA5) /* user ID */
            {                            
                i+=5;//moving right 5bit to 0x90

                if(buf[i+23] == 0xD0)
                {
                    Eular[0] = ((float)(int16_t)(buf[i+24] + (buf[i+25]<<8)))/100;
                    Eular[1] = ((float)(int16_t)(buf[i+26] + (buf[i+27]<<8)))/100;
                    Eular[2] = ((float)(int16_t)(buf[i+28] + (buf[i+29]<<8)))/10;

                    bodySta.roll = Eular[0];
                    bodySta.pitch = Eular[1];
                    bodySta.yaw = Eular[2];

//                    Center_rel_EXP[3] = -roll;
//                    Center_rel_EXP[4] = -pitch;
//                    Center_rel_EXP[5] = yaw;

                }

                
                //debug
                /*
                printf("ID: %d \r\n", ID);
                printf("AccRaw: %d %d %d\r\n", AccRaw[0], AccRaw[1], AccRaw[2]);
                printf("GyoRaw: %f %f %f\r\n", GyoRaw[0], GyoRaw[1], GyoRaw[2]);
                printf("MagRaw: %d %d %d\r\n", MagRaw[0], MagRaw[1], MagRaw[2]);
                printf("Eular: %0.2f %0.2f %0.2f\r\n", Eular[1], Eular[0], Eular[2]);
                printf("Pressure: %d Pa\r\n\n", Pressure);
                */

                bodystate_pub.publish(bodySta);

                //ros::spinOnce();
                loop_rate.sleep();

                count++;
            }
        }
    }//while end

    return 0;
}
