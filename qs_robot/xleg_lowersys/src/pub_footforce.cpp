#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>

#include "xleg_msgs/FootsForce.h"
#include <serial/serial.h>
//#include <QTimer>

using namespace std;
serial::Serial ser;

unsigned char   getforce[33];    //接收缓存   8通道25字节   12通道33字节
double LANDED_FORCE = 10;   //判断角触地阈值

/*
 * 循环超过30Hz,貌似力传感器采集速率就跟不上了
 *
 *
*/
//读取连接到串口的力传感器的读数
// ****************************************************************************************
// 主函数
int main(int argc, char *argv[])
{
    // 初始化ROS
    ros::init(argc, argv, "pub_footforce_node");
    ros::NodeHandle n;

    try
    {
        ser.setPort("/dev/ttyUSB0");//打开串口
        ser.setBaudrate(19200);//以19200的波特率来读取该串口
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);//设定一个计时器
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        //         qDebug()<<("串口未打开！没有可用串口！");
        ROS_ERROR_STREAM("Unable to open port!");
        return 0;
    }

    if(ser.isOpen())
    {
        ROS_INFO("Serial Port for Force_Sensor has been initialized.");
    }

    // 发布topic
    ros::Publisher footForce_pub = n.advertise<xleg_msgs::FootsForce>("xleg/footForce",1);    //FootForce_info

    xleg_msgs::FootsForce footsForce;

//设置 "xleg/footForce" topic，发布力传感器的数据

    // 设置循环
    ros::Rate loop_rate(50);      // 单位 Hz,每秒钟循环50次
    unsigned char togetF[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0E, 0xC4, 0x0E};
    //待发送的8字节数据包
    while(ros::ok())//持续进行的循环，只要ROS环境仍在运行，就会继续
    {
        ser.write(togetF,  8);

//        unsigned int dn = ser.available();     //串口缓存区有多少数，就读多少
        unsigned int dn = 33;     //等待读取33字节
        float force[12]={0};
        ser.read( getforce, dn);//读取设备返回的33个字节的数据
        ser.flush();//清空串口的读写缓冲区

//        ROS_INFO("读取字节数： dn=  %d . ", dn);
        //返回33字节，第8-31共24字节是力值
        if ( dn==33 )
        {
//            ROS_INFO("dn=33");
            // 解析力数据
            for (int j=0; j<12; j++)
            {
                force[j] = ( (signed short int) ( getforce[2*(j+4)-1]*256+getforce[2*(j+4)] )  ) /10.0 ;      // 有符号数
            }

            // 发布 足端力 消息, 腿编号1234（不是0 1 2 3）
            for(int i=0;i<4;i++)
            {
                footsForce.foots_force[i].num = i+1;
                footsForce.foots_force[i].X = force[3*i];
                footsForce.foots_force[i].Y = force[3*i+1];
                footsForce.foots_force[i].Z = force[3*i+2];

                if((double)footsForce.foots_force[i].Z <LANDED_FORCE)
                // 如果每只脚的Z方向的力低于设定的阈值，则假设该脚没有接触地面
                    footsForce.foots_force[i].isLanded = false;
                else
                    footsForce.foots_force[i].isLanded = true;

            }
            // 发布 质心坐标 消息
            footsForce.massCenterX = 1 ;     // Center_rel_massxyz[0]
            footsForce.massCenterY = 2 ;
            footsForce.massCenterZ = 3 ;
            footForce_pub.publish(footsForce);


        } // dn==25

        loop_rate.sleep();

    } //while

    return 0;
}
