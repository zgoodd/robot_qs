#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#include <fstream>  //c++ wen jian cao zuo
#include <iomanip>  //she zhi shu chu ge shi

//#include "xleg_msgs/JointState.h"
#include "xleg_msgs/JointsState.h"
//#include "xleg_msgs/FootForce.h"
#include "xleg_msgs/FootsForce.h"
#include "xleg_msgs/BodyState.h"

#include "xleg_msgs/servoControl.h"
#include "xleg_msgs/motorCode.h"           //测试用
#include "xleg_msgs/gaitControl.h"
#include "xleg_msgs/bodyControl.h"
#include "xleg_msgs/dataSave.h"

#include "robot_function.h"
#include "robot_thread.h"
#include <serial/serial.h>
#include <thread>
//#include <pthread.h>
#include <mutex>
#include <semaphore.h>

using namespace std;


//视觉
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h> // ROS下对图像进行处理
#include <image_transport/image_transport.h> // 用来发布和订阅图像信息
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
#include <stdlib.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>


//setlocale (LC_CTYPE, "zh_CN.utf8");
//setlocale(LC_ALL, ""); // 我用的是这一句，亲测ok

//#define LOOPHZ 40       //定义循环周期，每秒多少次

#define CONTROLER_INTERVAL 40   // 定义控制周期/ms
#define MOTORSNUM 12    // 总共12个   先期调试6

//如果没接，则定义; 如果接了，则取消定义
//#define NO_dxl
//#define NO_FORCE
//#define NO_IMU
//#define NONO

DynamixelWorkbench dxl_wb;
serial::Serial ser;
serial::Serial ser_imu;

float Center_ZERO_Z=0;
float Center_dZ=0;
float Center_dxy_ZERO[2]={0};
float Center_dw_ZERO[2]={0};
int climb_stateflag=1;

int gaitType;
int locomotionMode;
bool climbMode;
bool stopFlag = true;    //控制电机停止 全局
//float direction[3];
float movedir=0;
float bodydir=0;
float g_v[3] = {0,0,0}, g_w[3] = {0,0,0}, g_foot1[3], g_foot2[3], g_foot3[3], g_foot4[3]; // g_footholdDistance;
//g_foot 当前四条腿末端的绝对三维坐标 （相对于原点。 本体中心(0,0,H)在地面的投影位置为原点 ）。因此，g_foot[2]（即z坐标）肯定为正。
//a[3],b[3],c[3],d[3];  // a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm。因此，落地时a[2]（即z坐标）为负（足一般在腿起点下方）。
float  k, b;
int stepHeight, stepDistance;
double T;   //一个步进周期，四腿和机身走完
unsigned int ticks;
float Leg_present_angle[12]={0};   //存储实时舵机角度
float Foot_present_forces[12]={0};   //存储实时足底力  全局
float foot_present_reposition[12]={0};  //存储足端相对位置（abcd）, xyz*4
unsigned int showCode[8]={0};
int showValue[8]={0};
int showMOVE=0;  
int bodyadjX=0; int bodyadjY=0;
bool noGravity = false;
bool gaitReadDxldataNow = false;          //当线程读写舵机端口，主函数不读，用线程数据（只更新角度）
bool NO_force_port;    //线程中判断能否进入越障模式

bool bizhang;
float zhangaiwuD = 1.0;  //正前方障碍物距离
int qianfLukuang[4] = {0,0,0,0};  //前方路况0123：障碍物，台阶，斜坡

//int testFd[40];
//string filename[40]={"wx.txt","wy.txt","wz.txt","x.txt","y.txt","z.txt","joint0.txt","joint1.txt","joint2.txt","joint3.txt","joint4.txt","joint5.txt","joint6.txt","joint7.txt","joint8.txt","joint9.txt","joint10.txt","joint11.txt","joint12.txt","joint13.txt",
//                     "force1x.txt","force1y.txt","force1z.txt","force2x.txt","force2y.txt","force2z.txt","force3x.txt","force3y.txt","force3z.txt","force4x.txt","force4y.txt","force4z.txt"};
string dataname;
bool stopSaveData = true;

//给线程用
void hello();
void gait_thread();
void showMove_thread();
void readForce_thread();
void startVision_thread();

//mutex m;
sem_t sem_id;


//======================  舵机 torque on/off       ====================================================================
bool service_servoControl(xleg_msgs::servoControl::Request  &req,
                          xleg_msgs::servoControl::Response &res)
{
    //    ROS_INFO("servo service request:\ncmd:%d, allServos:%d, servoID:%d",req.servoCmd,req.allServos,req.servoID);
    const char *log;
    if(req.servoCmd == req.STOP_ROBOT)
    {
        stopFlag = true;
        showMOVE = 0;
        ROS_INFO("STOP: showMOVE==%d,stopFalg=%d", showMOVE, stopFlag);

        ROS_WARN("All motors is ready STOP.\n\n");
    }
    else if(req.servoCmd == req.TORQUE_ON)
    {
        if(req.allServos)
        {
            for (int i=0; i < MOTORSNUM; i++)
            {
                dxl_wb.torqueOn(i+1, &log);//通过dxl_wb.torqueOn()函数实现的，传入的参数是伺服器的ID
            }
            ROS_INFO("All motors is Torque ON.");
        }
        //        else
        //             dxl_wb.torqueOn(req.servoID, &log);
    }

    else if(req.servoCmd == req.TORQUE_OFF)
    {
        if(req.allServos)
        {
            for (char i=0; i < MOTORSNUM; i++)
            {
                dxl_wb.torqueOff(i+1, &log);
            }
            ROS_INFO("All motors is Torque OFF.");
        }
    }

    else if(req.servoCmd == req.CLEAR_ERROR)
    {
        if(req.allServos)
        {
            for(int i=0;i<MOTORSNUM;i++)
            {
                dxl_wb.reboot(i+1, &log);
            }
            ROS_INFO("All motors is ReBoot.");
        }
        else
        {
            bool res = dxl_wb.reboot(req.servoID, &log);
            dxl_wb.torqueOn(req.servoID, &log);
            if (res==false)
                ROS_ERROR("Failed to reboot Motor %d.",  req.servoID);
            else
                ROS_INFO("Succeed to reboot Motor %d.",  req.servoID);
        }
    }

    return true;
}


// ============================     单腿舵机控制 服务程序 (只是测试)    ====================================
bool service_motorCode(xleg_msgs::motorCode::Request  &req,
                       xleg_msgs::motorCode::Response &res)
{
    //    ROS_INFO("servo service request:\n angle=%d \n velocity=%d",  req.positionR, req.speedR );
    //    ROS_INFO("servo service reques: service_motorCode" );

    boost::array<int32_t, 3> dxl_id = req.servoID;
    boost::array<float, 3> dxl_v =req.speedR;          //    v[°/s]
    boost::array<uint16_t, 3>  dxl_time = req.playtime;   //  ms
    boost::array<float, 3> dxl_p = req.angleR;
//提取了请求中的伺服ID(dxl_id), 速度(dxl_v), 时间(dxl_time), 以及角度(dxl_p)。
    ROS_INFO("dxl_p == %f, %f, %f." ,dxl_p[0], dxl_p[1],dxl_p[2] );

    if (req.drive_alllegs == 1)
    {
        //        ROS_INFO("req.drive_alllegs == 1");
        if ( dxl_time[0]>0 )
        {
            for (int j=0; j<4; j++ )     // int j=0; j<4; j++   int j=3; j>-1; j--
            {
                //循环中的每个步骤对应于腿的一个关节
                Send_Servo_Position(3*j+1, dxl_time[0],  dxl_p[0]);   //joint 1
                Send_Servo_Position(3*j+2, dxl_time[1],  dxl_p[1]);   //joint 2
                Send_Servo_Position(3*j+3, dxl_time[2],  dxl_p[2]);   //joint 3
            }
        }
        else if ( dxl_v[0]>0 )
        {
            //            ROS_INFO("接收到速度控制模式指令 - 位置控制");
            for (int j=0; j<4; j++)
            {
                Send_Servo_Position_speed(3*j+1, dxl_v[0],  dxl_p[0]);   //joint 1
                Send_Servo_Position_speed(3*j+2, dxl_v[1],  dxl_p[1]);   //joint 2
                Send_Servo_Position_speed(3*j+3, dxl_v[2],  dxl_p[2]);   //joint 3
            //具体的调整操作通过函数Send_Servo_Position()和Send_Servo_Position_speed()完成。
            }
        }
    }
    else
    {
        //        ROS_INFO("req.drive_alllegs == 0");
        if ( dxl_time[0]>0 )
        {
            //            ROS_INFO("dxl_time[0]>0");
            Send_Servo_Position(dxl_id[0], dxl_time[0],  dxl_p[0]);
            Send_Servo_Position(dxl_id[1], dxl_time[1],  dxl_p[1]);
            Send_Servo_Position(dxl_id[2], dxl_time[2],  dxl_p[2]);
        }
        else if ( dxl_v[0]>0 )
        {
            //            ROS_INFO("接收到速度控制模式指令");
            Send_Servo_Position_speed(dxl_id[0], dxl_v[0], dxl_p[0]);
            Send_Servo_Position_speed(dxl_id[1], dxl_v[1], dxl_p[1]);
            Send_Servo_Position_speed(dxl_id[2], dxl_v[2], dxl_p[2]);
        }
    }
    return true;

}


//============================     步态控制 服务程序   ==========================================================
bool service_gaitControl(xleg_msgs::gaitControl::Request  &req, xleg_msgs::gaitControl::Response &res)
//根据收到的请求req来更新机器人的运动参数，然后返回一个响应res
{
    ROS_INFO("GaitControl service in... ");


    // 越障模式
    if(req.climbMode == true)
        climbMode = true;
    else
        climbMode = false;

    if(req.bizhangMode == true)
        bizhang = true;
    else
        bizhang = false;

    // 步态及特有参数
    if(req.gaitType == req.GAIT_TRANS)
    {
        gaitType = xleg_msgs::gaitControl::Request::GAIT_TRANS;
        k= req.k;
        ROS_INFO("req.gaitType == req.GAIT_TRANS");
    }
    else if(req.gaitType == req.GAIT_TRANS_FAST)
    {
        gaitType = xleg_msgs::gaitControl::Request::GAIT_TRANS_FAST;
        k= req.k;
        ROS_INFO("req.gaitType == req.GAIT_TRANS_FAST");
    }
    else if(req.gaitType == req.GAIT_TROT)
    {
        gaitType = xleg_msgs::gaitControl::Request::GAIT_TROT;
        b = req.b;
        ROS_INFO("req.gaitType == req.GAIT_TROT");

        //            Center_ZERO_Z=Center_rel_EXP[5];
        //            Center_dxy_ZERO[0]=Center_rel_massxyz[0];
        //            Center_dxy_ZERO[1]=Center_rel_massxyz[1];
        //            ss<<"gaitType:GAIT_TROT"<<endl<<"b:"<<(double)req.b<<",";
    }
    else if(req.gaitType == req.GAIT_TROT_FAST)
    {
        gaitType = xleg_msgs::gaitControl::Request::GAIT_TROT_FAST;
        b = req.b;
        ROS_INFO("req.gaitType == req.GAIT_TROT_FAST");

        //            Center_ZERO_Z=Center_rel_EXP[5];
        //            Center_dxy_ZERO[0]=Center_rel_massxyz[0];
        //            Center_dxy_ZERO[1]=Center_rel_massxyz[1];
        //            ss<<"gaitType:GAIT_TROT_FAST"<<endl<<"b:"<<(double)req.b<<",";
    }

    noGravity = req.isNoGravity;
    if (noGravity == true)
        ROS_INFO("不考虑重力，即无重心调整。");
    else
        ROS_INFO("考虑重力，即行走过程有重心调整。");


    // 步长和步高
    stepDistance = req.stepDistance;
    stepHeight = req.stepHeight;

    // 运动方向
    movedir = req.move_direction;
    bodydir = req.body_direction;

    // 初始姿态
    double n_x = req.footholdDistanceX;//提供了足部接触地面位置的初始信息
    double n_y = req.footholdDistanceY;
    g_v[0] = req.x;     g_v[1] = req.y;     g_v[2] = req.z;//机器人的线速度信息
    g_w[0] = req.Wx;    g_w[1] = req.Wy;    g_w[2] = req.Wz;//机器人的角速度信息
    g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;  //XIE 在调整姿态时调整了
    g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);   g_foot2[2] =0;
    g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);   g_foot3[2] =0;
    g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;
    T = req.T;
//四个足部的初始位置信息。中心位置（CENTER_L和CENTER_B）被用于计算机器人初始的足部位置。
//这些位置在X，Y平面内的偏移量通过footholdDistanceX和footholdDistanceY控制



    if(req.locomotionMode == req.STOP)
    {
        stopFlag = true;
        //            ROS_WARN("Gait service request: STOP");
    }
    else if(req.locomotionMode ==req.MOVEONCE)
    {
        locomotionMode = xleg_msgs::gaitControl::Request::MOVEONCE;
        ROS_INFO("Gait service request: MOVEONCE");
        stopFlag = false;
    }
    else if(req.locomotionMode ==req.MOVE)
    {
        locomotionMode = xleg_msgs::gaitControl::Request::MOVE;
        ROS_INFO("Gait service request: MOVE");
        stopFlag = false;
    }
    climbMode = req.climbMode;
    if (climbMode == true)
        ROS_INFO("climbMode is ON.");
    else
        ROS_INFO("climbmode is OFF.");

    showMOVE = req.showMOVE;   //0: stop;   1: 按序列运动演示move,  2 gait线程（非运动演示）
    stopFlag = !showMOVE;   //1: move, 0: stop
    boost::array<int32_t, 8> showCode_tmp = req.showCode;   //0无，1前进，2平移，3转向
    boost::array<int32_t, 8> showValue_tmp = req.showValue;
    bodyadjX = req.bodyadjustX;
    bodyadjY = req.bodyadjustY;
    for(int i=0; i<8; i++)
    {
        showCode[i] = showCode_tmp[i];//这两个数组中获取的运动指令和值
        showValue[i] = showValue_tmp[i];
                    ROS_INFO("showCode[%d]=%d, showValue[%d]=%d",i,showCode[i], i,showValue[i]);
    }

    ROS_INFO("showMOVE==%d,stopFalg=%d", showMOVE, stopFlag);

    if (stopFlag==false )
    {
        if (showMOVE == 2)//启动名为 gait 的线程来执行
        {
            std::thread gait(gait_thread);
            gait.detach();
            //    gait.join();  //阻塞
        }
        else if ( (showMOVE == 1) || (showMOVE ==3) )//sm_thread的线程来执行
        {
            std::thread sm_thread(showMove_thread);
            sm_thread.detach();
        }
    }
    else
        ROS_WARN("STOP!!   stopFlag=%d, showMOVE=%d",stopFlag,showMOVE);



    ROS_WARN("service_gaitControl END");
    return true;
}





// ===================================== 机身姿态控制 服务例程 ==========================================================
//根据请求中的参数来控制机器人的姿态和行为

bool service_bodyControl(xleg_msgs::bodyControl::Request  &req,
                         xleg_msgs::bodyControl::Response &res)
{
    gaitReadDxldataNow = true;

    usleep( int(req.waittime) *1000);
    uint16_t pt = req.playtime;
    if (pt < 500)  pt = 500;

    float n_x = req.footholdDistanceX;   //足相对于腿起点的相对坐标
    float n_y = req.footholdDistanceY;

    int get_Ready = req.get_robotReady;
    ROS_INFO("posready=%d", get_Ready);
    //    机器人抬起到初始状态
    if (get_Ready == 1)
    {
        ROS_INFO("Robot PosReady");

        n_y=220;
        g_v[0] = 0;    g_v[1] = 0;    g_v[2] = -10; //body down, foot up
        g_w[0] = 0;    g_w[1] = 0;    g_w[2] = 0;
        float a[3],b[3],c[3],d[3];
        // 足端绝对坐标
        g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;
        g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
        g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
        g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;

        //腿统一到达位置
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
        res.error_codes=Set_ALL_Leg_Positon(a,b,c,d);
        ROS_INFO("body service response: error code[%d]", res.error_codes);
        if(res.error_codes == 0 )
            Send_ALL_Leg_Positon( pt );  //0x3c  2000ms//Send_ALL_Leg_Position 来实际控制所有机器人腿的位置，否则直接返回
        else
        {
            gaitReadDxldataNow = false;     return false;
        }
        usleep( pt*1000);
        //腿统一落下
        n_y=150;
        g_v[0] = 0;    g_v[1] = 0;    g_v[2] = 30; //body down, foot up
        g_w[0] = 0;    g_w[1] = 0;    g_w[2] = 0;
        // 足端绝对坐标
        g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;
        g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
        g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
        g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
        res.error_codes=Set_ALL_Leg_Positon(a,b,c,d);
        ROS_INFO("body service response: error code[%d]", res.error_codes);
        if(res.error_codes == 0 )
            Send_ALL_Leg_Positon( pt );  //0x3c  2000ms
        else
        {
            gaitReadDxldataNow = false;     return false;
        }
        usleep( pt*1000);

        //body up
        //        g_v[2] = 100;
        g_v[2] = req.z;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        res.error_codes = Set_ALL_Leg_Positon(a,b,c,d);
        if(res.error_codes == 0)
        {
            Send_ALL_Leg_Positon( uint16_t (1.2*pt) );
        }
        else
        {
            gaitReadDxldataNow = false;     return false;
        }

        ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",a[0], a[1],a[2], b[0], b[1],b[2] ,
                c[0], c[1],c[2] ,d[0], d[1],d[2]);

        gaitReadDxldataNow = false;
        return true;
    }
    else if (get_Ready == 2)
    {
        ROS_INFO("Robot PosDown");

        g_v[0] = 0;    g_v[1] = 0;    g_v[2] = 35; //body down, foot up
        g_w[0] = 0;    g_w[1] = 0;    g_w[2] = 0;
        float a[3],b[3],c[3],d[3];
        // 足端绝对坐标
        g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;
        g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
        g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
        g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;

        //腿统一到达位置
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
        res.error_codes=Set_ALL_Leg_Positon(a,b,c,d);
        ROS_INFO("body service response: error code[%d]", res.error_codes);
        if(res.error_codes == 0 )
            Send_ALL_Leg_Positon( uint16_t (1.2*pt) );  //0x3c  2000ms
        else
        {
            gaitReadDxldataNow = false;     return false;
        }
        usleep( pt*1000);

        //body return zeor
        g_v[2] = -1;   // 抬的太高，释放力矩时候冲击太大   原先-10
        n_y = 220;
        g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;
        g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
        g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
        g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        res.error_codes = Set_ALL_Leg_Positon(a,b,c,d);
        if(res.error_codes == 0)
        {
            Send_ALL_Leg_Positon( pt );
        }
        else
        {
            gaitReadDxldataNow = false;     return false;
        }

        ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",a[0], a[1],a[2], b[0], b[1],b[2] ,
                c[0], c[1],c[2] ,d[0], d[1],d[2]);


        gaitReadDxldataNow = false;
        return true;
    }


    n_x = req.footholdDistanceX;   //足相对于腿起点的相对坐标
    n_y = req.footholdDistanceY;
    g_v[0] = req.x;    g_v[1] = req.y;    g_v[2] = req.z;
    g_w[0] = req.Wx;    g_w[1] = req.Wy;    g_w[2] = req.Wz;

    ROS_INFO("Body service request:\n\
             x:%d, y:%d, z:%d,\n\
             Wx:%f, Wy:%f, Wz:%f,\n  footholdDistanceX:%d, footholdDistanceY:%d\n",
             (int)req.x,(int)req.y,(int)req.z,(float)req.Wx,(float)req.Wy,(float)req.Wz,(int)req.footholdDistanceX,(int)req.footholdDistanceY);

    float a[3],b[3],c[3],d[3];  // 定义a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm
    if(req.initPosMode == req.DIRECT)
    {
        if(req.gaitType == req.GAIT_TRANS_FAST || req.gaitType == req.GAIT_TROT_FAST)
        {
            // 足端绝对坐标
            g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;
            g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
            g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
            g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;
        }
        else
        {
            g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;
            g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
            g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
            g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;
        }
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        res.error_codes=Set_ALL_Leg_Positon(a,b,c,d);
        ROS_INFO("body service response: error code[%d]", res.error_codes);

        if(res.error_codes == 0)
            Send_ALL_Leg_Positon( pt );  //0x3c  2000ms
    }
    else if(req.initPosMode == req.INDIRECT)
    {
        if(req.gaitType == req.GAIT_TRANS_FAST || req.gaitType == req.GAIT_TROT_FAST)
        {
            // 足端绝对坐标
            g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;
            g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
            g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
            g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;

            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
            res.error_codes = Set_ALL_Leg_Positon(a,b,c,d);

            int stepDistance = req.stepDistance;
            int stepHeight = req.stepHeight;
            ROS_INFO("stepDistance=%d,  stepHeight=%d",stepDistance, stepHeight);

            //调到初始状态
            Make_One_Leg_Runonce(1, a, (int)(0.5*stepDistance), 0, stepHeight, pt);
            g_foot1[0]=g_foot1[0] + 0.5*stepDistance;   //后来移动机身的参数
            usleep(int (1000*pt));

            Make_One_Leg_Runonce(3, c, (int)(-0.5*stepDistance), 0, stepHeight, pt);
            g_foot3[0]=g_foot3[0] - 0.5*stepDistance;   //后来移动机身的参数
            usleep(int (1000*pt));

        }
        else
        {
            g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =0;
            g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
            g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
            g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;

            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
            res.error_codes=Set_ALL_Leg_Positon(a,b,c,d);
            ROS_INFO("body service response: error code[%d]", res.error_codes);

            Make_One_Leg_toPositon(1, a, 50, pt);
            usleep(int (1000*pt));
            Make_One_Leg_toPositon(3, c, 50, pt);
            usleep(int (1000*pt));
            Make_One_Leg_toPositon(2, b, 50, pt);
            usleep(int (1000*pt));
            Make_One_Leg_toPositon(4, d, 50, pt);
            usleep(int (1000*pt*1));
        }
    }

    ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",a[0], a[1],a[2], b[0], b[1],b[2] ,
            c[0], c[1],c[2] ,d[0], d[1],d[2]);

    ROS_INFO("此时：\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);

    gaitReadDxldataNow = false;
    return true;
}






/* ===============================             ===================================
 *
 *            线程
 * ===============================================================================
*/

void hello()
{
    int i=0;
    while(1)
    {
        i++;
        ROS_INFO("hello %d",i );
        sleep(1);
    }
}


void gait_thread()//步态线程的线程
{
    ROS_WARN("into gait_thread()");

    uint16_t playtime = floor(T*1000/5);  //5步，每步时间ms
    if(gaitType == xleg_msgs::gaitControl::Request::GAIT_TRANS_FAST)     playtime = floor(T*1000/4);  //4步，每步时间ms
    if (playtime<500)  playtime=500;

    int ii = 0;  //循环次数
    int error_back=1;
    float a[3],b[3],c[3],d[3];  // 定义a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm
    char legID;
    int bodyceyi = 20; //重心侧移  稳定

    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);//CCenterToLeg()函数计算
    error_back = Set_ALL_Leg_Positon(a,b,c,d);//Set_ALL_Leg_Positon(a,b,c,d）函数会将这些坐标用于设定步态

    ROS_INFO("进入步态后初始：");
    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
             g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

    ROS_INFO("\n stepDistance=%d, stepHeight=%d, T=%lfs", stepDistance, stepHeight, T);
    ROS_INFO("重心调整bodyadjX=%d,bodyadjY=%d",bodyadjX,bodyadjY);

    float vw_foot_abcd[30] = {0};   //
    for (int i=0;i<3;i++)
    {
        vw_foot_abcd[i] = g_v[i];
        vw_foot_abcd[i+3] = g_w[i];
        vw_foot_abcd[i+6] = g_foot1[i];
        vw_foot_abcd[i+9] = g_foot2[i];
        vw_foot_abcd[i+12] = g_foot3[i];
        vw_foot_abcd[i+15] = g_foot4[i];
        vw_foot_abcd[i+18] = a[i];
        vw_foot_abcd[i+21] = b[i];
        vw_foot_abcd[i+24] = c[i];
        vw_foot_abcd[i+27] = d[i];
    }
    float *vwfa_backdata = new float[30];

    gaitReadDxldataNow = true;

    while ((!stopFlag) && (showMOVE==2))
    {
        if (noGravity == true)   // 不考虑重力，即无重心调整
        {
            ROS_WARN("不考虑重力，即无重心调整。");

            if(gaitType == xleg_msgs::gaitControl::Request::GAIT_TRANS)   //3+1
            {
                ROS_INFO("IN GAIT_TRANS....");
                if(climbMode==true)     //越障模式
                {
                    ROS_WARN("越障模式还没完成...");
                    stopFlag = true;


                }
                else      //不考虑避障越障
                {
                    if ( ii==0 )
                    {
                        if ( abs(bodydir)>0 )//检查身体方向偏移角 bodydir
                        {
                            sem_wait(&sem_id);

                            int eachRoa = 20;  //每30度转一次
                            ROS_WARN("bodydir=%f，需要调整机身方向。", bodydir);
                            float aa[3],bb[3],cc[3],dd[3];  // 定义a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm

                            int kkk = floor( abs(bodydir)/eachRoa);
                            if (kkk>0)
                            {
                                for (int k=0; k<kkk; k++)
                                {
                                    ROS_WARN("第%d/%d次调整机身方向。", k+1, kkk+1);
                                    g_w[2] = eachRoa;
                                    if (bodydir<0) g_w[2] = -eachRoa;

                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,aa,bb,cc,dd);
                                    int error_codes = Set_ALL_Leg_Positon(aa,bb,cc,dd);

                                    if(error_codes == 0 && !stopFlag)
                                        Send_ALL_Leg_Positon( 1200 );
                                    Send_ALL_Leg_Positon( 1200 )
                                     //这部分是向所有腿发送位置指令，参数1200是时间单位，意味着在1200毫秒内，腿完成从当前位置到目标位置的运动
                                    else
                                        ROS_ERROR("第%d次调整机身方向出错！", k+1);
                                    usleep( 1200*1000);

                                    //
                                    int pt=4400;
                                    Make_One_Leg_toPositon(1, a, 50, (uint16_t) (pt/4));    //用原abcd
                                    //为一只腿制定运动路径，其中a是腿的目标位置，50是速度，pt/4是运动所需时间
                                    usleep(int (1000*pt/4));
                                    Make_One_Leg_toPositon(3, c, 50, (uint16_t) (pt/4));
                                    usleep(int (1000*pt/4));
                                    Make_One_Leg_toPositon(2, b, 50, (uint16_t) (pt/4));
                                    usleep(int (1000*pt/4));
                                    Make_One_Leg_toPositon(4, d, 50, (uint16_t) (pt/4));
                                    usleep(int (1000*pt/4));
                                }
                            }

                            ROS_WARN("第%d/%d次调整机身方向。", kkk+1, kkk+1);
                            g_w[2] = fmod (bodydir, eachRoa);//余数
                            ROS_INFO("fmod (bodydir, 30)=%f", fmod (bodydir, 30));
                            if ( abs(fmod(bodydir, eachRoa)) > 2)  //2度内就不调整了
                            {
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,aa,bb,cc,dd);
                                int error_codes = Set_ALL_Leg_Positon(aa,bb,cc,dd);
                                if(error_codes == 0  && !stopFlag)
                                    Send_ALL_Leg_Positon( 1200 );
                                else
                                    ROS_ERROR("旋转机身出错！");
                                usleep( 1200*1000);

                                int pt=4400;
                                Make_One_Leg_toPositon(1, a, 50, (uint16_t) (pt/4));    //用原abcd
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(3, c, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(2, b, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(4, d, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                            }
                            else
                                ROS_WARN("第%d/%d次调整机身方向角度过小，不再调整。", kkk+1, kkk+1);

                            sem_post(&sem_id);
                        }

                    }
                    //向前走 1234
                    if ( movedir == 0)
                    {
                        sem_wait(&sem_id);
                        if (ii==0) ROS_WARN("向前行走%d：", ii);





                        sem_post(&sem_id);
                        //以上步骤结束后，四条腿回归原位置
                    }

                    //向左走 1423
                    else if ( movedir == 90)
                    {
                        sem_wait(&sem_id);//等待信号量，如果信号量的值小于等于0，那么就等待，否则将信号量的值减1后继续执行。
                        //这实际上是一种互斥锁的机制，用来保证同一时刻只有一个线程运行
                        if (ii==0) ROS_WARN("向左行走：");

                        ROS_WARN("1. leg1+d");
                        legID=1;//**********
                        error_back = Make_One_Leg_Runonce(legID, a, 0, stepDistance, stepHeight, playtime);
                        //调用Make_One_Leg_Runonce函数使腿运动一次
                        g_foot1[1]=g_foot1[1]+1*stepDistance;   //后来移动机身的参数
                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        ROS_WARN("2. leg4+d");//用来提示当前行走状态的输出信息
                        legID=4;
                        error_back = Make_One_Leg_Runonce(legID, d, 0, stepDistance, stepHeight, playtime);
                        g_foot4[1]=g_foot4[1]+1*stepDistance;   //后来移动机身的参数
                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        ROS_WARN("3. body+%dmm", int(stepDistance));
                        g_v[1] =  stepDistance;
                        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                        // //机身的运动参数转化为各腿的运动参数，并调用Set_ALL_Leg_Positon函数设置各腿的位置
                        error_back = Set_ALL_Leg_Positon(a,b,c,d);//设置各腿的位置
                        if(error_back == 0 && !stopFlag)
                        {
                            Send_ALL_Leg_Positon( playtime );
                        }
                        else
                        {
                            ROS_INFO("机身进出错。 error_codes=%d",error_back );
                            sem_post(&sem_id);   return;
                        }
                        usleep(playtime*1000);

                        ROS_WARN("4. leg2+d");
                        legID=2;
                        error_back = Make_One_Leg_Runonce(legID, b, 0, stepDistance, stepHeight, playtime);
                        g_foot2[1]=g_foot2[1]+1*stepDistance;   //后来移动机身的参数
                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        ROS_WARN("5. leg3+d");
                        legID=3;
                        error_back = Make_One_Leg_Runonce(legID, c, 0, stepDistance, stepHeight, playtime);
                        g_foot3[1]=g_foot3[1]+1*stepDistance;   //后来移动机身的参数
                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        sem_post(&sem_id);
                    }

                    //向右走 2314
                    else if ( movedir == -90)
                    {
                        sem_wait(&sem_id);
                        if (ii==0) ROS_WARN("向右行走：");

                        ROS_WARN("1. leg2+d");
                        legID=2;

                        error_back = Make_One_Leg_Runonce(legID, b, 0, -stepDistance, stepHeight, playtime);
                        g_foot2[1]=g_foot2[1] - 1*stepDistance;   //后来移动机身的参数

                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        ROS_WARN("2. leg3+d");
                        legID=3;

                        error_back = Make_One_Leg_Runonce(legID, c, 0, -stepDistance, stepHeight, playtime);
                        g_foot3[1]=g_foot3[1] - 1*stepDistance;   //后来移动机身的参数

                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        ROS_WARN("3. body+%dmm", int(stepDistance));
                        g_v[1] =  -stepDistance;
                        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                        error_back = Set_ALL_Leg_Positon(a,b,c,d);
                        if(error_back == 0 && !stopFlag)
                        {

                            Send_ALL_Leg_Positon( playtime );

                        }
                        else
                        {
                            ROS_INFO("机身进出错。 error_codes=%d",error_back );
                            sem_post(&sem_id);   return;
                        }
                        usleep(playtime*1000);

                        ROS_WARN("4. leg1+d");
                        legID=1;

                        error_back = Make_One_Leg_Runonce(legID, a, 0, -stepDistance, stepHeight, playtime);
                        g_foot1[1]=g_foot1[1] - 1*stepDistance;   //后来移动机身的参数

                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        ROS_WARN("5. leg4+d");
                        legID=4;

                        error_back = Make_One_Leg_Runonce(legID, d, 0, -stepDistance, stepHeight, playtime);
                        g_foot4[1]=g_foot4[1] - 1*stepDistance;   //后来移动机身的参数

                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }
                        sem_post(&sem_id);

                    }

                    //向后走
                    else if ( movedir == 180)
                    {
                        sem_wait(&sem_id);
                        if (ii==0) ROS_WARN("向后行走：");

                        //腿3进
                        ROS_WARN("1.leg3");
                        legID=3;

                        error_back = Make_One_Leg_Runonce(legID, c, -stepDistance, 0, stepHeight, playtime);
                        g_foot3[0]=g_foot3[0] - 1*stepDistance;   //后来移动机身的参数

                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        //腿1进
                        ROS_WARN("2.leg1");
                        legID=1;

                        error_back = Make_One_Leg_Runonce(legID, a, -stepDistance, 0, stepHeight, playtime);
                        g_foot1[0]=g_foot1[0] - 1*stepDistance;   //后来移动机身的参数

                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }



                        //机身进
                        ROS_WARN("3. body+%dmm", int(stepDistance));
                        g_v[0] = -stepDistance;
                        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                        error_back = Set_ALL_Leg_Positon(a,b,c,d);
                        if(error_back == 0 && !stopFlag)
                        {

                            Send_ALL_Leg_Positon( playtime );

                        }
                        else
                        {
                            ROS_INFO("机身进出错。 error_codes=%d",error_back );
                            sem_post(&sem_id);   return;
                        }
                        usleep(playtime*1000);

                        //腿4进 1d
                        ROS_WARN("4.leg4");
                        legID=4;

                        error_back = Make_One_Leg_Runonce(legID, d, -stepDistance, 0, stepHeight, playtime);

                        g_foot4[0]=g_foot4[0]-stepDistance;   //后来移动机身的参数
                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }

                        //腿2进 1d
                        ROS_WARN("5.leg2");
                        legID=2;

                        error_back = Make_One_Leg_Runonce(legID, b, -stepDistance, 0, stepHeight, playtime);

                        g_foot2[0]=g_foot2[0] - stepDistance;   //后来移动机身的参数
                        if (error_back == 1 || stopFlag)
                        {
                            stopFlag = true;
                            sem_post(&sem_id);   return;
                        }
                        sem_post(&sem_id);
                        //以上步骤结束后，四条腿回归原位置
                    }





                    if(locomotionMode == xleg_msgs::gaitControl::Request::MOVEONCE)
                    {
                        //回原位
                        ROS_INFO("MoveOnce");
                        //                        ROS_WARN("back to zero");
                        //
                        //                        Make_One_Leg_Runonce(1, a, int(0.25*stepDistance), 0, int(0.5*stepHeight), uint16_t(0.3*playtime));
                        //                        Make_One_Leg_Runonce(2, b, int(0.25*stepDistance), 0, int(0.5*stepHeight), uint16_t(0.3*playtime));
                        //                        Make_One_Leg_Runonce(3, c, int(-0.25*stepDistance), 0, int(0.5*stepHeight), uint16_t(0.3*playtime));
                        //                        Make_One_Leg_Runonce(4, d, int(-0.25*stepDistance), 0, int(0.5*stepHeight), uint16_t(0.3*playtime));
                        //                        sem_post(&sem_id);
                        ii=0;
                        stopFlag = true;
                    }
                    bodydir=0;
                    ii++;
                    ROS_INFO("已完成%d次步进周期，累计行进距离%dmm。",ii, (ii*stepDistance));
                    ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);
                }

            }


            // 快速步态
            else if(gaitType == xleg_msgs::gaitControl::Request::GAIT_TRANS_FAST)
            {
                sem_wait(&sem_id);

                ROS_INFO("IN GAIT_TRANS_FAST....");

                if (ii==0)
                {
                    if ( abs(bodydir)>0 )
                    {
                        int eachRoa = 20;  //每30度转一次
                        ROS_WARN("bodydir=%f，需要调整机身方向。",bodydir);
                        float aa[3],bb[3],cc[3],dd[3];  // 定义a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm

                        int kkk = floor( abs(bodydir)/eachRoa);
                        if (kkk>0)
                        {
                            for (int k=0; k<kkk; k++)
                            {
                                ROS_WARN("第%d/%d次调整机身方向。", k+1, kkk+1);
                                g_w[2] = eachRoa;
                                if (bodydir<0) g_w[2] = -eachRoa;

                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,aa,bb,cc,dd);
                                int error_codes = Set_ALL_Leg_Positon(aa,bb,cc,dd);
                                if(error_codes == 0)
                                    Send_ALL_Leg_Positon( 1200 );
                                else
                                    ROS_ERROR("第%d次调整机身方向出错！", k+1);
                                usleep( 1200*1000);

                                int pt=4400;
                                Make_One_Leg_toPositon(1, a, 50, (uint16_t) (pt/4));    //用原abcd
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(3, c, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(2, b, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(4, d, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                            }
                        }

                        ROS_WARN("第%d/%d次调整机身方向。", kkk+1, kkk+1);
                        g_w[2] = fmod (bodydir, eachRoa);//余数
                        ROS_INFO("fmod (bodydir, 30)=%f", fmod (bodydir, 30));
                        if ( abs(fmod(bodydir, eachRoa)) > 2)  //2度内就不调整了
                        {
                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,aa,bb,cc,dd);
                            int error_codes = Set_ALL_Leg_Positon(aa,bb,cc,dd);
                            if(error_codes == 0)
                                Send_ALL_Leg_Positon( 1200 );
                            else
                                ROS_ERROR("旋转机身出错！");

                            usleep( 1200*1000);

                            int pt=4400;

                            Make_One_Leg_toPositon(1, a, 50, (uint16_t) (pt/4));    //用原abcd

                            usleep(int (1000*pt/4));

                            Make_One_Leg_toPositon(3, c, 50, (uint16_t) (pt/4));

                            usleep(int (1000*pt/4));

                            Make_One_Leg_toPositon(2, b, 50, (uint16_t) (pt/4));

                            usleep(int (1000*pt/4));

                            Make_One_Leg_toPositon(4, d, 50, (uint16_t) (pt/4));

                            usleep(int (1000*pt/4));
                        }
                        else
                            ROS_WARN("第%d/%d次调整机身方向角度过小，不再调整。", kkk+1, kkk+1);
                    }

                }

                //腿3进
                ROS_WARN("1.3");
                g_v[0] =   0.25*stepDistance;
                g_v[1] = bodyceyi;
                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，

                ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                if(error_back == 0 && !stopFlag)
                {

                    Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );

                }
                else
                {
                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                    sem_post(&sem_id);   return;
                }
                legID=3;

                error_back = Make_One_Leg_Runonce(legID, c, stepDistance, 0, stepHeight, playtime);

                g_foot3[0]=g_foot3[0]+stepDistance;   //后来移动机身的参数
                if (error_back == 1 || stopFlag)
                {
                    ROS_WARN("error_back == 1");

                    stopFlag = true;
                    sem_post(&sem_id);   return;
                }

                ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);


                //腿2+机身进
                ROS_WARN("2.2");
                g_v[0] =  + 0.25*stepDistance;

                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                if(error_back == 0 && !stopFlag)
                {

                    Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );

                }
                else
                {
                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                    sem_post(&sem_id);   return;
                }
                legID=2;

                error_back = Make_One_Leg_Runonce(legID, b, stepDistance, 0, stepHeight, playtime);
                g_foot2[0]=g_foot2[0]+1*stepDistance;   //后来移动机身的参数

                if (error_back == 1 || stopFlag)
                {
                    //                    ROS_ERROR("腿%d 抬 解算出错。",legID);
                    stopFlag = true;
                    sem_post(&sem_id);   return;
                }

                ROS_INFO("\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                         g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);


                //腿1+机身进
                ROS_INFO("3.1 腿1+d,机身+0.25d.  d=%dmm", int(stepDistance));
                g_v[0] = 0.25*stepDistance;
                g_v[1] = -bodyceyi;
                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                if(error_back == 0 && !stopFlag)
                {

                    Send_ALL_Leg_Positon( (uint16_t) (0.4*playtime) );

                }
                else
                {
                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                    sem_post(&sem_id);   return;
                }

                legID=1;

                error_back = Make_One_Leg_Runonce(legID, a, stepDistance, 0, stepHeight, playtime);
                g_foot1[0]=g_foot1[0] + stepDistance;   //后来移动机身的参数

                if (error_back == 1 || stopFlag)
                {
                    stopFlag = true;
                    sem_post(&sem_id);   return;
                }

                ROS_INFO("\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                         g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

                //腿4进
                ROS_WARN("4.4");
                g_v[0] =  + 0.25*stepDistance;
                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                if(error_back == 0 && !stopFlag)
                {

                    Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );

                }
                else
                {
                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                    sem_post(&sem_id);   return;
                }
                legID=4;

                error_back = Make_One_Leg_Runonce(legID, d, stepDistance, 0, stepHeight, playtime);

                g_foot4[0]=g_foot4[0]+stepDistance;   //后来移动机身的参数
                if (error_back == 1 || stopFlag)
                {
                    stopFlag = true;
                    sem_post(&sem_id);   return;
                }
                //以上步骤结束后，四条腿回归原位置
                sem_post(&sem_id);

                if(locomotionMode == xleg_msgs::gaitControl::Request::MOVEONCE)
                {
                    //回原位
                    ROS_INFO("MoveOnce");
                    ii=0;
                    stopFlag = true;
                }

                ii++;
                ROS_INFO("已完成%d次步进周期，累计行进距离%dmm。",ii, (ii*stepDistance));
                ROS_INFO("\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                         g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);
            }

        }


        else
        {
            ROS_INFO("考虑重力，即行走过程有重心调整。");

            if(gaitType == xleg_msgs::gaitControl::Request::GAIT_TRANS)   //3+1
            {
                ROS_INFO("IN GAIT_TRANS....");
                if (climbMode==true  && bizhang==true)     //越障模式（非平整地面，考虑足力）+视觉避障
                {
                    ROS_WARN("越障模式还没完成...");
                    stopFlag = true;


                    std::thread sm_thread(showMove_thread);
                    sm_thread.detach();


                }
                else if (climbMode==true  && bizhang==false)      //考虑地面不平（足力），不考虑避障
                {




                }


                else if (climbMode==false  && bizhang==true)     //只考虑视觉避障
                {
                    if ( movedir == 0)  //向前
                    {
                        ROS_WARN("障碍物距离 = %f", zhangaiwuD);

                        if (zhangaiwuD>10.2) // 每前进一步判断一次
                        {
                            ROS_INFO("zhangaiwuD>0.5");
                            if (ii==0) ROS_WARN("向前行走%d：", ii);
                            error_back = Make_Runonce_withGravity(vw_foot_abcd,  stepDistance, 0, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);
                        }
                        else
                        {

                            int ssf_D = zhangaiwuD;
                            int ssf_H = 100;

                            error_back = Climb_stepsurface_withGravity( vw_foot_abcd,  ssf_D, ssf_H,  stepHeight, bodyadjX, bodyadjY, playtime, vwfa_backdata);

                        }



                    }
                    else
                    {
                        ROS_WARN("视觉避障只能向前走");
                    }



                }
                else  //越障+避障 都不考虑
                {

                   if ( ii==0 )
                    {
                        if ( abs(bodydir)>0 )
                        {
                            sem_wait(&sem_id);

                            int eachRoa = 20;  //每30度转一次
                            ROS_WARN("bodydir=%f，需要调整机身方向。", bodydir);
                            float aa[3],bb[3],cc[3],dd[3];  // 定义a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm

                            int kkk = floor( abs(bodydir)/eachRoa);
                            if (kkk>0)
                            {
                                for (int k=0; k<kkk; k++)
                                {
                                    ROS_WARN("第%d/%d次调整机身方向。", k+1, kkk+1);
                                    g_w[2] = eachRoa;
                                    if (bodydir<0) g_w[2] = -eachRoa;

                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,aa,bb,cc,dd);
                                    int error_codes = Set_ALL_Leg_Positon(aa,bb,cc,dd);

                                    if(error_codes == 0 && !stopFlag)
                                        Send_ALL_Leg_Positon( 1200 );
                                    else
                                        ROS_ERROR("第%d次调整机身方向出错！", k+1);
                                    usleep( 1200*1000);

                                    int pt=4400;
                                    Make_One_Leg_toPositon(1, a, 50, (uint16_t) (pt/4));    //用原abcd
                                    usleep(int (1000*pt/4));
                                    Make_One_Leg_toPositon(3, c, 50, (uint16_t) (pt/4));
                                    usleep(int (1000*pt/4));
                                    Make_One_Leg_toPositon(2, b, 50, (uint16_t) (pt/4));
                                    usleep(int (1000*pt/4));
                                    Make_One_Leg_toPositon(4, d, 50, (uint16_t) (pt/4));
                                    usleep(int (1000*pt/4));
                                }
                            }

                            ROS_WARN("第%d/%d次调整机身方向。", kkk+1, kkk+1);
                            g_w[2] = fmod (bodydir, eachRoa);//余数
                            ROS_INFO("fmod (bodydir, 30)=%f", fmod (bodydir, 30));
                            if ( abs(fmod(bodydir, eachRoa)) > 2)  //2度内就不调整了
                            {
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,aa,bb,cc,dd);
                                int error_codes = Set_ALL_Leg_Positon(aa,bb,cc,dd);
                                if(error_codes == 0  && !stopFlag)
                                    Send_ALL_Leg_Positon( 1200 );
                                else
                                    ROS_ERROR("旋转机身出错！");
                                usleep( 1200*1000);

                                int pt=4400;
                                Make_One_Leg_toPositon(1, a, 50, (uint16_t) (pt/4));    //用原abcd
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(3, c, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(2, b, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                                Make_One_Leg_toPositon(4, d, 50, (uint16_t) (pt/4));
                                usleep(int (1000*pt/4));
                            }
                            else
                                ROS_WARN("第%d/%d次调整机身方向角度过小，不再调整。", kkk+1, kkk+1);

                            sem_post(&sem_id);
                        }

                    }


                   if ( movedir == 0)  //向前
                   {
                       if (ii==0) ROS_WARN("向前行走%d：", ii);
                       error_back = Make_Runonce_withGravity(vw_foot_abcd,  stepDistance, 0, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);
                   }
                   else if ( movedir == 90)  //左
                   {
                       if (ii==0) ROS_WARN("向左行走%d：", ii);
                       error_back = Make_Runonce_withGravity(vw_foot_abcd,  0, stepDistance, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);
                   }
                   else if ( movedir == -90)  //右
                   {
                       if (ii==0) ROS_WARN("向右行走%d：", ii);
                       error_back = Make_Runonce_withGravity(vw_foot_abcd,  0, -stepDistance, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);
                   }
                   else if ( movedir == 180)  //后
                   {
                       if (ii==0) ROS_WARN("向后行走%d：", ii);
                       error_back = Make_Runonce_withGravity(vw_foot_abcd,  -stepDistance, 0, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);
                   }

                }


            }   //3+1 end


        }




        if(locomotionMode == xleg_msgs::gaitControl::Request::MOVEONCE)
        {
            //回原位
            ROS_INFO("MoveOnce DONE!");
            //                        ROS_WARN("back to zero");
            //
            //                        Make_One_Leg_Runonce(1, a, int(0.25*stepDistance), 0, int(0.5*stepHeight), uint16_t(0.3*playtime));
            //                        Make_One_Leg_Runonce(2, b, int(0.25*stepDistance), 0, int(0.5*stepHeight), uint16_t(0.3*playtime));
            //                        Make_One_Leg_Runonce(3, c, int(-0.25*stepDistance), 0, int(0.5*stepHeight), uint16_t(0.3*playtime));
            //                        Make_One_Leg_Runonce(4, d, int(-0.25*stepDistance), 0, int(0.5*stepHeight), uint16_t(0.3*playtime));
            //                        sem_post(&sem_id);
            ii=0;
            stopFlag = true;
        }
        bodydir=0;
        ii++;
        ROS_INFO("已完成%d次步进周期，累计行进距离%dmm。",ii, (ii*stepDistance));
        ROS_INFO("此时：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                 a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);


    } //while (!stopFlag)
    ROS_WARN("已退出步态控制循环。");
}


/* ==========================================================================================================================
 *
=============================================================================================================================







==========================================================================================================================





==========================================================================================================================





/* =================================   运动演示    =============================================================================
 *
 */
void showMove_thread()
{

    ROS_WARN("into showMove_thread()");
    uint16_t playtime = floor(T*1000/5);  //5步，每步时间ms

    int error_back=1;
    float a[3],b[3],c[3],d[3];  // 定义a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm
    char legID;

    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
    error_back = Set_ALL_Leg_Positon(a,b,c,d);

    ROS_INFO("进入步态后初始：");
    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
             g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

    ROS_INFO("\n stepDistance=%d, stepHeight=%d, T=%lfs", stepDistance, stepHeight, T);
    ROS_INFO("重心调整bodyadjX=%d,bodyadjY=%d",bodyadjX,bodyadjY);

    float vw_foot_abcd[30] = {0};   //
    for (int i=0;i<3;i++)
    {
        vw_foot_abcd[i] = g_v[i];
        vw_foot_abcd[i+3] = g_w[i];
        vw_foot_abcd[i+6] = g_foot1[i];
        vw_foot_abcd[i+9] = g_foot2[i];
        vw_foot_abcd[i+12] = g_foot3[i];
        vw_foot_abcd[i+15] = g_foot4[i];
        vw_foot_abcd[i+18] = a[i];
        vw_foot_abcd[i+21] = b[i];
        vw_foot_abcd[i+24] = c[i];
        vw_foot_abcd[i+27] = d[i];
    }
    float *vwfa_backdata = new float[30];
    
    gaitReadDxldataNow = true;

    if (showMOVE == 3)     // 一直前进，遇到障碍就左右移动，避障判断，有无重心判断，步态就不扩展对角步态了
    {
        ROS_WARN("进入前进演示。");

        int iii=0; int jjj=0;
        while( (!stopFlag) && (showMOVE==3) )
        {
            iii++;
            if( bizhang == true)
            {
                if (iii < 5  && iii>0)  ROS_INFO("进入前进+避障演示。");
                if (noGravity == false)
                {
                    //                                ROS_INFO("考虑重力，即行走过程有重心调整。");

                    if (gaitType == xleg_msgs::gaitControl::Request::GAIT_TRANS)//步态
                    {
                        ROS_INFO("当前为3+1步态。");
                        ROS_WARN("障碍物距离 = %f", zhangaiwuD);

                        if (zhangaiwuD>0.5) // 每前进一步判断一次
                        {
                            ROS_INFO("zhangaiwuD>0.5");

                            sem_wait(&sem_id);
                            g_v[0] =0; g_v[1]=0;g_w[2]=0;
                            ROS_INFO("\n\
                                     g_v=%f, %f, %f \n\
                                     g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);
                            // 前进
                            int stepD_tmp= stepDistance;

                            ROS_INFO("stepD_tmp=%d",stepD_tmp);
//                            ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
//                                     g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
//                                    g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
//                            ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
//                                     a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

                            error_back = Make_Runonce_withGravity(vw_foot_abcd,  stepD_tmp, 0, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);
                            sem_post(&sem_id);


                            //以上步骤结束后，四条腿回归原位置

                            sem_post(&sem_id);
                        }
                        else  // 右移动避障 500mm
                        {
                            //move right
                            ROS_WARN("zhangaiwuD<0.5");
                            sem_wait(&sem_id);
                            g_v[0] =0; g_v[1]=0;g_w[2]=0;

                            int stepDistance_H = stepDistance;
                            if (stepDistance_H>100) stepDistance_H=100;  //左移/右移步长最高为120mm。

                            int numb=floor(100/ stepDistance_H)+1; //定义要move步数  movejulishi 500mm
                            ROS_INFO("右移步数%d来避障",numb);
                            int stepD_tmp = stepDistance_H;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次右移。", i, numb);

                                //2-3-4-1
                                //腿2进,调整重心
                                ROS_INFO("1.body+leg2");
                                g_v[0] = -bodyadjX;   //相对前一时刻，相对值
                                g_v[1] = +bodyadjY;
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0 && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (0.3*playtime) );
                                    usleep( 0.3*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }

                                error_back = Make_One_Leg_Runonce(2, b, 0, -stepD_tmp, stepHeight, playtime);
                                g_foot2[1]=g_foot2[1]-stepD_tmp;   //后来移动机身的参数
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                //腿3进,调整重心
                                ROS_INFO("2.body+leg3");
                                g_v[0] = +2*bodyadjX;   //相对前一时刻，相对值
                                g_v[1] = 0;   //-Y
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (0.5*playtime) );
                                    usleep( 0.5*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }

                                error_back = Make_One_Leg_Runonce(3, c, 0, -stepD_tmp, stepHeight, playtime);
                                g_foot3[1]=g_foot3[1]-stepD_tmp;   //后来移动机身的参数
                                if (error_back == 1  || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }


                                //body move + 腿4进,调整重心
                                ROS_INFO("3+4.body+leg4");
                                g_v[0] = 0;   //相对前一时刻，相对值
                                g_v[1] = -stepD_tmp - 2*bodyadjY;//步长临时值和机身调整量
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (1*playtime) );
                                    usleep( 1*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }

                                error_back = Make_One_Leg_Runonce(4, d, 0, -stepD_tmp, stepHeight, playtime);
                                g_foot4[1]=g_foot4[1]-stepD_tmp;   //后来移动机身的参数
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                //腿1进,调整重心
                                ROS_INFO("5.body+leg1");
                                g_v[0] = -2*bodyadjX;   //相对前一时刻，相对值
                                g_v[1] = 0;
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (0.5*playtime) );
                                    usleep( 0.5*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }

                                error_back = Make_One_Leg_Runonce(1, a, 0, -stepD_tmp, stepHeight, playtime);
                                g_foot1[1]=g_foot1[1]-stepD_tmp;   //后来移动机身的参数
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                //5,调整重心,回归初始姿态
                                ROS_INFO("6.body return zero");
                                g_v[0] = +bodyadjX;   //相对前一时刻，相对值
                                g_v[1] = +bodyadjY;   //y
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，

                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (0.3*playtime) );
                                    usleep( 0.25*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }
                                //以上步骤结束后，四条腿回归原位置
                            }

                            sem_post(&sem_id);
                            // move right end
                        }


                    }
                    else if (gaitType == xleg_msgs::gaitControl::Request::GAIT_TROT)
                    {
                        ROS_INFO("当前为对角步态 ————  还没编写，有重力，重心调整复杂");
                    }

                }
                else if (noGravity == true)
                {

                }

            }
            else
            {
                jjj++;
                if (jjj<5 && jjj>0)   ROS_INFO("进入前进(不避障)演示。。。待编写%d", jjj);

            }

            if (showMOVE!=3) return;

        }

    } //  前进演示结束




    else if (showMOVE == 1)  //按照序列运动，重力判断，步态判断
    {

        for (int k=0; k<8; k++)     ROS_INFO("showCode[%d]=%d, showValue[%d]=%d",k,showCode[k], k,showValue[k]);//这是个什么东西

        if (noGravity == false)
        {
            ROS_INFO("考虑重力，即行走过程有重心调整。");

            if (gaitType != xleg_msgs::gaitControl::Request::GAIT_TROT)         // gaitType == xleg_msgs::gaitControl::Request::GAIT_TRANS
            {
                ROS_INFO("当前为3+1步态。");

                for (int k=0; k<8; k++)
                {
                    if (showCode[k]==1)   //前进后退  //1-3-2-4
                    {
                        sem_wait(&sem_id);
                        g_v[0] =0; g_v[1]=0;g_w[2]=0;
                        ROS_INFO("\n\
                                 g_v=%f, %f, %f \n\
                                 g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);
                        // 前进
                        if (showValue[k]>0)
                        {
                            int numb=0; //定义要前进的步数
                            int shang = floor( abs(showValue[k])/stepDistance);
                            int yushu = fmod(abs(showValue[k]), stepDistance);
                            if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要前进步数%d",shang,yushu,numb);

                            int stepD_tmp;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次前进。", i, numb);

                                stepD_tmp = stepDistance;
                                if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                ROS_INFO("stepD_tmp=%d",stepD_tmp);


//                                ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
//                                         g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
//                                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
//                                ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
//                                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);



                                error_back = Make_Runonce_withGravity(vw_foot_abcd,  stepD_tmp, 0, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);


//                                for (int i=0; i<3; i++)
//                                {
//                                    g_v[i] = vwfa_backdata[i];     g_w[i] = vwfa_backdata[i+3];

//                                    g_foot1[i] = vwfa_backdata[i+6];
//                                    g_foot2[i] = vwfa_backdata[i+9];
//                                    g_foot3[i] = vwfa_backdata[i+12];
//                                    g_foot4[i] = vwfa_backdata[i+15];

//                                    a[i] = vwfa_backdata[i+18];
//                                    b[i] = vwfa_backdata[i+21];
//                                    c[i] = vwfa_backdata[i+24];
//                                    d[i] = vwfa_backdata[i+27];
//                                }

//                                ROS_INFO("单步后");
//                                ROS_INFO("g_v = %f,%f,%f",vwfa_backdata[0], vwfa_backdata[1], vwfa_backdata[2]);
//                                ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）：\n g_foot1[x,y,z] = %f,%f,%f \n g_foot2[x,y,z] = %f,%f,%f \n g_foot3[x,y,z] = %f,%f,%f \n g_foot4[x,y,z] = %f,%f,%f",
//                                         vwfa_backdata[6],vwfa_backdata[7],vwfa_backdata[8],
//                                        vwfa_backdata[9],vwfa_backdata[10],vwfa_backdata[11],
//                                        vwfa_backdata[12],vwfa_backdata[13],vwfa_backdata[14],
//                                        vwfa_backdata[15],vwfa_backdata[16],vwfa_backdata[17]);
//                                ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
//                                         vwfa_backdata[18],vwfa_backdata[19],vwfa_backdata[20],
//                                        vwfa_backdata[21],vwfa_backdata[22],vwfa_backdata[23],
//                                        vwfa_backdata[24],vwfa_backdata[25],vwfa_backdata[26],
//                                        vwfa_backdata[27],vwfa_backdata[28],vwfa_backdata[29]);


                                //以上步骤结束后，四条腿回归原位置
                            }
                        }

                        // 后退
                        else if (showValue[k]<0)
                        {
                            int numb=0; //定义要后退的步数
                            int shang = floor( abs(showValue[k])/stepDistance);
                            int yushu = fmod(abs(showValue[k]), stepDistance);
                            if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要后退步数%d",shang,yushu,numb);

                            int stepD_tmp;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次后退。", i, numb);

                                stepD_tmp = stepDistance;
                                if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                ROS_INFO("stepD_tmp=%d",stepD_tmp);



                                error_back = Make_Runonce_withGravity(vw_foot_abcd,  -stepD_tmp, 0, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);
                                //以上步骤结束后，四条腿回归原位置
                            }
                        }//hou tui

                        sem_post(&sem_id);
                    }// forward+backward end

                    else if (showCode[k]==2)   //左移右移
                    {
                        sem_wait(&sem_id);
                        g_v[0] =0; g_v[1]=0;g_w[2]=0;

                        int stepDistance_H = stepDistance;
                        if (stepDistance_H>100) stepDistance_H=100;  //左移/右移步长最高为120mm。
                        //move left
                        if (showValue[k]>0)
                        {
                            int numb=0; //定义要move步数
                            int shang = floor( abs(showValue[k])/stepDistance_H);
                            int yushu = fmod(abs(showValue[k]), stepDistance_H);
                            if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要左移步数%d",shang,yushu,numb);

                            int stepD_tmp;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次左移。", i, numb);

                                stepD_tmp = stepDistance_H;
                                if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                ROS_INFO("stepD_tmp=%d",stepD_tmp);



                                error_back = Make_Runonce_withGravity(vw_foot_abcd, 0, stepD_tmp, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);

                                //以上步骤结束后，四条腿回归原位置
                            }

                        }// move left end


                        //move right
                        else if (showValue[k]<0)
                        {
                            int numb=0; //定义要move步数
                            int shang = floor( abs(showValue[k])/stepDistance_H);
                            int yushu = fmod(abs(showValue[k]), stepDistance_H);
                            if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要右移步数%d",shang,yushu,numb);

                            int stepD_tmp;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次右移。", i, numb);

                                stepD_tmp = stepDistance_H;
                                if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                ROS_INFO("stepD_tmp=%d",stepD_tmp);



                                error_back = Make_Runonce_withGravity(vw_foot_abcd, 0, -stepD_tmp, stepHeight,bodyadjX,bodyadjY, playtime, vwfa_backdata);

                            }
                            sem_post(&sem_id);

                        }// move right end

                        sem_post(&sem_id);
                    }//else if (showCode[k]==2) //左移右移 jieshu

                    else if (showCode[k]==3)   //左转右转
                    {
                        sem_wait(&sem_id);
                        g_v[0] =0; g_v[1]=0;g_w[2]=0;
                        int eachRoa = 15;  //每20度转一次
                        int eachRoa_tmp;
                        // 定义aa,bb,cc,dd 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm, 为目标; abcd为实时当前
                        float aa[3],bb[3],cc[3],dd[3];
                        float g_foot1_tmp[3],g_foot2_tmp[3],g_foot3_tmp[3],g_foot4_tmp[3];
                        for (int ii=0; ii<3; ii++)
                        {
                            aa[ii]=a[ii]; bb[ii]=b[ii]; cc[ii]=c[ii]; dd[ii]=d[ii];
                            g_foot1_tmp[ii] = g_foot1[ii];   g_foot2_tmp[ii] = g_foot2[ii];
                            g_foot3_tmp[ii] = g_foot3[ii];   g_foot4_tmp[ii] = g_foot4[ii];
                        }
                        ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                 g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                        ROS_INFO("\n aa[x,y,z] = %f,%f,%f \n bb[x,y,z] = %f,%f,%f \n cc[x,y,z] = %f,%f,%f \n dd[x,y,z] = %f,%f,%f",
                                 aa[0], aa[1],aa[2], bb[0], bb[1],bb[2] ,cc[0], cc[1],cc[2] ,dd[0], dd[1],dd[2]);


                        //turn left
                        if (showValue[k]>0)
                        {
                            int numb=0; //定义要turn cishu
                            int shang = floor( abs(showValue[k])/eachRoa);
                            int yushu = fmod(abs(showValue[k]), eachRoa);
                            if (yushu<1) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要左转步数%d",shang,yushu,numb);

                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次左转。", i, numb);

                                eachRoa_tmp = eachRoa;
                                if ( (numb>shang) && (i==numb))  eachRoa_tmp=yushu;
                                ROS_INFO("eachRoa_tmp=%d",eachRoa_tmp);

                                //2-1-4-3
                                //body turn + adj
                                ROS_INFO("1.body");

                                ROS_INFO("\n\
                                         g_v=%f, %f, %f \n\
                                         g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                                         g_v[0] = -bodyadjX;
                                g_w[2] = eachRoa_tmp;
                                ROS_INFO("\n\
                                         g_v=%f, %f, %f \n\
                                         g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                                         ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                                  g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.8*playtime );
                                    usleep( 0.9*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }


                                //leg
                                ROS_INFO("2.leg2");
                                int dx, dy;
                                dx = bb[0]-b[0]+bodyadjX;   dy = bb[1]-b[1];
                                error_back = Make_One_Leg_Runonce(2, b, dx, dy, stepHeight, playtime);
                                g_foot2[0]=g_foot2[0]+ dx;   //后来移动机身的参数
                                g_foot2[1]=g_foot2[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }



                                ROS_INFO("3.leg1");
                                dx = aa[0]-a[0]+bodyadjX;   dy = aa[1]-a[1];
                                error_back = Make_One_Leg_Runonce(1, a, dx, dy, stepHeight, playtime);
                                g_foot1[0]=g_foot1[0]+ dx;   //后来移动机身的参数
                                g_foot1[1]=g_foot1[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }


                                ROS_INFO("4.body");
                                //                            g_v[0] = +2*bodyadjX;
                                //                            g_w[2] = 0;
                                //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                //上面方法结果有误，下面的思路更直接,机身前进，相当于足底相对坐标均后退
                                a[0]=a[0]-2*bodyadjX; b[0]=b[0]-2*bodyadjX; c[0]=c[0]-2*bodyadjX; d[0]=d[0]-2*bodyadjX;
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.4*playtime );
                                    usleep( 0.5*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }



                                ROS_INFO("5.leg4");
                                dx = dd[0]-d[0]-bodyadjX;   dy = dd[1]-d[1];
                                error_back = Make_One_Leg_Runonce(4, d, dx, dy, stepHeight, playtime);
                                g_foot4[0]=g_foot4[0]+ dx;   //后来移动机身的参数
                                g_foot4[1]=g_foot4[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }



                                ROS_INFO("6.leg3");
                                dx = cc[0]-c[0]-bodyadjX;   dy = cc[1]-c[1];
                                error_back = Make_One_Leg_Runonce(3, c, dx, dy, stepHeight, playtime);
                                g_foot3[0]=g_foot3[0]+ dx;   //后来移动机身的参数
                                g_foot3[1]=g_foot3[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }



                                ROS_INFO("7.body to zero");
                                //                            g_v[0] = -1*bodyadjX;
                                //                            g_w[2] = 0;
                                //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);

                                a[0]=a[0]+bodyadjX;  b[0]=b[0]+bodyadjX;  c[0]=c[0]+bodyadjX;  d[0]=d[0]+bodyadjX;
                                int error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.4*playtime );
                                    usleep( 0.4*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }

                                for (int ii=0; ii<3; ii++)
                                {
                                    g_foot1[ii] = g_foot1_tmp[ii];  g_foot2[ii] = g_foot2_tmp[ii];
                                    g_foot3[ii] = g_foot3_tmp[ii];  g_foot4[ii] = g_foot4_tmp[ii];
                                }

                                //to zero
                            }
                        }


                        //turn right
                        else if (showValue[k]<0)
                        {
                            int numb=0; //定义要turn cishu
                            int shang = floor( abs(showValue[k])/eachRoa);
                            int yushu = fmod(abs(showValue[k]), eachRoa);
                            if (yushu<1) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要右转步数%d",shang,yushu,numb);

                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次右转。", i, numb);

                                eachRoa_tmp = eachRoa;
                                if ( (numb>shang) && (i==numb))  eachRoa_tmp=yushu;
                                ROS_INFO("eachRoa_tmp=%d",eachRoa_tmp);

                                //1-2-3-4
                                //body turn + adj
                                ROS_INFO("1.body");
                                g_v[0] = -bodyadjX;
                                g_w[2] = -eachRoa_tmp;
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.8*playtime );
                                    usleep( 0.9*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }

                                //leg
                                ROS_INFO("2.leg1");
                                int dx, dy;
                                dx = aa[0]-a[0]+bodyadjX;   dy = aa[1]-a[1];
                                error_back = Make_One_Leg_Runonce(1, a, dx, dy, stepHeight, playtime);
                                g_foot1[0]=g_foot1[0]+ dx;   //后来移动机身的参数
                                g_foot1[1]=g_foot1[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("3.leg2");
                                dx = bb[0]-b[0]+bodyadjX;   dy = bb[1]-b[1];
                                error_back = Make_One_Leg_Runonce(2, b, dx, dy, stepHeight, playtime);
                                g_foot2[0]=g_foot2[0]+ dx;   //后来移动机身的参数
                                g_foot2[1]=g_foot2[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("4.body");
                                //                            g_v[0] = +2*bodyadjX;
                                //                            g_w[2] = 0;
                                //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                //上面方法结果有误，下面的思路更直接,机身hou，相当于足底相对坐标均qian
                                a[0]=a[0]-2*bodyadjX; b[0]=b[0]-2*bodyadjX; c[0]=c[0]-2*bodyadjX; d[0]=d[0]-2*bodyadjX;
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.4*playtime );
                                    usleep( 0.5*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("5.leg3");
                                dx = cc[0]-c[0]-bodyadjX;   dy = cc[1]-c[1];
                                error_back = Make_One_Leg_Runonce(3, c, dx, dy, stepHeight, playtime);
                                g_foot3[0]=g_foot3[0]+ dx;   //后来移动机身的参数
                                g_foot3[1]=g_foot3[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("6.leg4");
                                dx = dd[0]-d[0]-bodyadjX;   dy = dd[1]-d[1];
                                error_back = Make_One_Leg_Runonce(4, d, dx, dy, stepHeight, playtime);
                                g_foot4[0]=g_foot4[0]+ dx;   //后来移动机身的参数
                                g_foot4[1]=g_foot4[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("7.body to zero");
                                //                            g_v[0] = -1*bodyadjX;
                                //                            g_w[2] = 0;
                                //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);

                                a[0]=a[0]+bodyadjX;  b[0]=b[0]+bodyadjX;  c[0]=c[0]+bodyadjX;  d[0]=d[0]+bodyadjX;
                                int error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.4*playtime );
                                    usleep( 0.35*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }

                                for (int ii=0; ii<3; ii++)
                                {
                                    g_foot1[ii] = g_foot1_tmp[ii];  g_foot2[ii] = g_foot2_tmp[ii];
                                    g_foot3[ii] = g_foot3_tmp[ii];  g_foot4[ii] = g_foot4_tmp[ii];
                                }
                                //to zero
                            }
                        }

                        ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                 g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                        ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                 a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);


                        sem_post(&sem_id);
                    }// turn left+right end

                    else if (showCode[k]==4)   //爬台阶
                    {
                        int ssf_D = zhangaiwuD;
                        int ssf_H = showValue[k];
                        ROS_INFO("台阶高度%d", ssf_H);

                        error_back = Climb_stepsurface_withGravity( vw_foot_abcd,  ssf_D, ssf_H,  stepHeight, bodyadjX, bodyadjY, playtime, vwfa_backdata);

                    }

                }

            }
            else if (gaitType == xleg_msgs::gaitControl::Request::GAIT_TROT)
            {
                ROS_INFO("当前为对角步态 ————  还没编写，有重力，重心调整复杂");
            }


        }

        else   // 不考虑重力，即无重心调整
        {
            ROS_INFO("不考虑重力，即无重心调整。");
            if (gaitType == xleg_msgs::gaitControl::Request::GAIT_TRANS)
            {
                ROS_INFO("当前为3+1步态。");
                if (climbMode == true)
                {
                    if(NO_force_port)
                    {
                        ROS_ERROR("力传感器未连接，不能进入越障模式！");
                        //                                return;
                    }

                    ROS_INFO("climbMode is ON. 力传感器在线，进入越障模式。");
                    for (int k=0; k<8; k++)
                    {
                        if (showCode[k]==1)   //前进后退  //1-3-2-4
                        {
                            sem_wait(&sem_id);
                            g_v[0] =0; g_v[1]=0;g_w[2]=0;
                            ROS_INFO("\n\
                                     g_v=%f, %f, %f \n\
                                     g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                            // 前进
                            if (showValue[k]>0)
                            {
                                int numb=0; //定义要前进的步数
                                int shang = floor( abs(showValue[k])/stepDistance);
                                int yushu = fmod(abs(showValue[k]), stepDistance);
                                if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要前进步数%d",shang,yushu,numb);

                                int stepD_tmp;
                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次前进。", i, numb);

                                    stepD_tmp = stepDistance;
                                    if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                    ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                    //1-3-2-4
                                    //腿1进
                                    ROS_INFO("1.leg1");
                                    float *response5;
                                    response5 = Make_One_Leg_Runonce_withForce(1, a, stepD_tmp, 0, stepHeight, playtime);
                                    ROS_WARN("response5= %f %f %f %f %f",response5[0],response5[1],response5[2],response5[3],response5[4]);
                                    g_foot1[0]=g_foot1[0]+stepD_tmp;   //后来移动机身的参数
                                    if (response5[4] == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }
                                    else if ( response5[4]==0 )
                                    {
                                        g_foot1[2]=g_v[2]+response5[3];   //后来移动机身的参数
                                    }

                                    ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                             g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

                                    //腿3进
                                    ROS_INFO("2.leg3");
                                    response5 = Make_One_Leg_Runonce_withForce(3, c, stepD_tmp, 0, stepHeight, playtime);
                                    g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
                                    if (response5[4] == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }
                                    else if ( response5[4]==0 )
                                    {
                                        g_foot3[2]=g_v[2]+response5[3];   //后来移动机身的参数
                                    }

                                    ROS_WARN("response5= %f %f %f %f %f",response5[0],response5[1],response5[2],response5[3],response5[4]);



                                    //body
                                    ROS_INFO("3.body move");
                                    g_v[0] = stepD_tmp;
                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0 && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
                                        usleep( 0.8*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                        sem_post(&sem_id);   return;
                                    }


                                    //腿2进
                                    ROS_INFO("4.leg2");
                                    response5 = Make_One_Leg_Runonce_withForce(2, b, stepD_tmp, 0, stepHeight, playtime);
                                    g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
                                    if (response5[4] == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }
                                    else if ( response5[4]==0 )
                                    {
                                        g_foot2[2]=g_v[2]+response5[3];   //后来移动机身的参数
                                    }

                                    ROS_WARN("response5= %f %f %f %f %f",response5[0],response5[1],response5[2],response5[3],response5[4]);



                                    //腿4进
                                    ROS_INFO("5.leg4");
                                    response5 = Make_One_Leg_Runonce_withForce(4, d, stepD_tmp, 0, stepHeight, playtime);
                                    g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
                                    if (response5[4] == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }
                                    else if ( response5[4]==0 )
                                    {
                                        g_foot4[2]=g_v[2]+response5[3];   //后来移动机身的参数
                                    }
                                    //以上步骤结束后，四条腿回归原位置

                                    ROS_WARN("response5= %f %f %f %f %f",response5[0],response5[1],response5[2],response5[3],response5[4]);



                                }
                            }

                            // 后退
                            else if (showValue[k]<0)
                            {
                                int numb=0; //定义要后退的步数
                                int shang = floor( abs(showValue[k])/stepDistance);
                                int yushu = fmod(abs(showValue[k]), stepDistance);
                                if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要后退步数%d",shang,yushu,numb);

                                int stepD_tmp;
                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次后退。", i, numb);

                                    stepD_tmp = stepDistance;
                                    if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                    ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                    //4-2-3-1
                                    //腿4后退
                                    ROS_INFO("1.leg4");
                                    legID=4;
                                    error_back = Make_One_Leg_Runonce(legID, d, -stepD_tmp, 0, stepHeight, playtime);
                                    g_foot4[0]=g_foot4[0]-stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    //腿2后退
                                    ROS_INFO("2.leg2");
                                    legID=2;
                                    error_back = Make_One_Leg_Runonce(legID, b, -stepD_tmp, 0, stepHeight, playtime);
                                    g_foot2[0]=g_foot2[0]-stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    //body
                                    ROS_INFO("3.body move");
                                    g_v[0] = -stepD_tmp;
                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0 && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
                                        usleep( 0.8*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                        sem_post(&sem_id);   return;
                                    }

                                    //腿3后退
                                    ROS_INFO("4.leg3");
                                    legID=3;
                                    error_back = Make_One_Leg_Runonce(legID, c, -stepD_tmp, 0, stepHeight, playtime);
                                    g_foot3[0]=g_foot3[0]-stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }


                                    //腿1后退
                                    ROS_INFO("5.leg1");
                                    legID=1;
                                    error_back = Make_One_Leg_Runonce(legID, a, -stepD_tmp, 0, stepHeight, playtime);
                                    g_foot1[0]=g_foot1[0]-stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }
                                    //以上步骤结束后，四条腿回归原位置
                                }
                            }//hou tui

                            sem_post(&sem_id);
                        }// forward+backward end

                        else if (showCode[k]==2)   //左移右移
                        {
                            sem_wait(&sem_id);
                            g_v[0] =0; g_v[1]=0;g_w[2]=0;

                            int stepDistance_H = stepDistance;
                            if (stepDistance_H>100) stepDistance_H=100;  //左移/右移步长最高为120mm。
                            //move left
                            if (showValue[k]>0)
                            {
                                int numb=0; //定义要move步数
                                int shang = floor( abs(showValue[k])/stepDistance_H);
                                int yushu = fmod(abs(showValue[k]), stepDistance_H);
                                if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要左移步数%d",shang,yushu,numb);

                                int stepD_tmp;
                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次左移。", i, numb);

                                    stepD_tmp = stepDistance_H;
                                    if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                    ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                    //1-4-3-2
                                    //腿1进
                                    ROS_INFO("1.leg1");
                                    error_back = Make_One_Leg_Runonce(1, a, 0, stepD_tmp, stepHeight, playtime);
                                    g_foot1[1]=g_foot1[1]+stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    //腿4进
                                    ROS_INFO("2.leg4");
                                    error_back = Make_One_Leg_Runonce(4, d, 0, stepD_tmp, stepHeight, playtime);
                                    g_foot4[1]=g_foot4[1]+stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1  || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }


                                    //body move + 腿3进,调整重心
                                    ROS_INFO("3+4.body+leg3");
                                    g_v[1] = stepD_tmp;
                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon(  (uint16_t) (1*playtime) );
                                        usleep( 1*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                        sem_post(&sem_id);   return;
                                    }

                                    error_back = Make_One_Leg_Runonce(3, c, 0, stepD_tmp, stepHeight, playtime);
                                    g_foot3[1]=g_foot3[1]+stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    //腿2进
                                    ROS_INFO("5.leg2");
                                    error_back = Make_One_Leg_Runonce(2, b, 0, stepD_tmp, stepHeight, playtime);
                                    g_foot2[1]=g_foot2[1]+stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }
                                    //以上步骤结束后，四条腿回归原位置
                                }

                            }// move left end


                            //move right
                            else if (showValue[k]<0)
                            {
                                int numb=0; //定义要move步数
                                int shang = floor( abs(showValue[k])/stepDistance_H);
                                int yushu = fmod(abs(showValue[k]), stepDistance_H);
                                if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要右移步数%d",shang,yushu,numb);

                                int stepD_tmp;
                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次右移。", i, numb);

                                    stepD_tmp = stepDistance_H;
                                    if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                    ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                    //2-3-4-1
                                    //腿2进
                                    ROS_INFO("1.leg2");
                                    error_back = Make_One_Leg_Runonce(2, b, 0, -stepD_tmp, stepHeight, playtime);
                                    g_foot2[1]=g_foot2[1]-stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    //腿3进
                                    ROS_INFO("2.leg3");
                                    error_back = Make_One_Leg_Runonce(3, c, 0, -stepD_tmp, stepHeight, playtime);
                                    g_foot3[1]=g_foot3[1]-stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1  || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }


                                    //body move + 腿4进,调整重心
                                    ROS_INFO("3+4.body+leg4");
                                    g_v[1] = -stepD_tmp;
                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon(  (uint16_t) (1*playtime) );
                                        usleep( 1*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                        sem_post(&sem_id);   return;
                                    }

                                    error_back = Make_One_Leg_Runonce(4, d, 0, -stepD_tmp, stepHeight, playtime);
                                    g_foot4[1]=g_foot4[1]-stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    //腿1进
                                    ROS_INFO("5.leg1");
                                    error_back = Make_One_Leg_Runonce(1, a, 0, -stepD_tmp, stepHeight, playtime);
                                    g_foot1[1]=g_foot1[1]-stepD_tmp;   //后来移动机身的参数
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }
                                    //以上步骤结束后，四条腿回归原位置
                                }

                            }// move right end

                            sem_post(&sem_id);
                        }//else if (showCode[k]==2) //左移右移 jieshu

                        else if (showCode[k]==3)   //左转右转
                        {
                            sem_wait(&sem_id);
                            g_v[0] =0; g_v[1]=0;g_w[2]=0;


                            int eachRoa = 15;  //每20度转一次
                            int eachRoa_tmp;
                            // 定义aa,bb,cc,dd 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm, 为目标; abcd为实时当前
                            float aa[3],bb[3],cc[3],dd[3];
                            float g_foot1_tmp[3],g_foot2_tmp[3],g_foot3_tmp[3],g_foot4_tmp[3];
                            for (int ii=0; ii<3; ii++)
                            {
                                aa[ii]=a[ii]; bb[ii]=b[ii]; cc[ii]=c[ii]; dd[ii]=d[ii];
                                g_foot1_tmp[ii] = g_foot1[ii];   g_foot2_tmp[ii] = g_foot2[ii];
                                g_foot3_tmp[ii] = g_foot3[ii];   g_foot4_tmp[ii] = g_foot4[ii];
                            }
                            ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                     g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                    g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                            ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                     a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);
                            ROS_INFO("\n aa[x,y,z] = %f,%f,%f \n bb[x,y,z] = %f,%f,%f \n cc[x,y,z] = %f,%f,%f \n dd[x,y,z] = %f,%f,%f",
                                     aa[0], aa[1],aa[2], bb[0], bb[1],bb[2] ,cc[0], cc[1],cc[2] ,dd[0], dd[1],dd[2]);


                            //turn left
                            if (showValue[k]>0)
                            {
                                int numb=0; //定义要turn cishu
                                int shang = floor( abs(showValue[k])/eachRoa);
                                int yushu = fmod(abs(showValue[k]), eachRoa);
                                if (yushu<1) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要左转步数%d",shang,yushu,numb);

                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次左转。", i, numb);

                                    eachRoa_tmp = eachRoa;
                                    if ( (numb>shang) && (i==numb))  eachRoa_tmp=yushu;
                                    ROS_INFO("eachRoa_tmp=%d",eachRoa_tmp);

                                    //2-1-4-3
                                    //body turn + adj
                                    ROS_INFO("1.body");

                                    ROS_INFO("\n\
                                             g_v=%f, %f, %f \n\
                                             g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                                             g_v[0] = -bodyadjX;
                                    g_w[2] = eachRoa_tmp;
                                    ROS_INFO("\n\
                                             g_v=%f, %f, %f \n\
                                             g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);



                                             CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                            error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.8*playtime );
                                        usleep( 0.9*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }


                                    //leg
                                    ROS_INFO("2.leg2");
                                    int dx, dy;
                                    dx = bb[0]-b[0]+bodyadjX;   dy = bb[1]-b[1];
                                    error_back = Make_One_Leg_Runonce(2, b, dx, dy, stepHeight, playtime);
                                    g_foot2[0]=g_foot2[0]+ dx;   //后来移动机身的参数
                                    g_foot2[1]=g_foot2[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }


                                    ROS_INFO("3.leg1");
                                    dx = aa[0]-a[0]+bodyadjX;   dy = aa[1]-a[1];
                                    error_back = Make_One_Leg_Runonce(1, a, dx, dy, stepHeight, playtime);
                                    g_foot1[0]=g_foot1[0]+ dx;   //后来移动机身的参数
                                    g_foot1[1]=g_foot1[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }
                                    ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                             g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

                                    ROS_INFO("4.body");
                                    //                            g_v[0] = +2*bodyadjX;
                                    //                            g_w[2] = 0;
                                    //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                    //上面方法结果有误，下面的思路更直接,机身前进，相当于足底相对坐标均后退
                                    a[0]=a[0]-2*bodyadjX; b[0]=b[0]-2*bodyadjX; c[0]=c[0]-2*bodyadjX; d[0]=d[0]-2*bodyadjX;
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.4*playtime );
                                        usleep( 0.5*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }



                                    ROS_INFO("5.leg4");
                                    dx = dd[0]-d[0]-bodyadjX;   dy = dd[1]-d[1];
                                    error_back = Make_One_Leg_Runonce(4, d, dx, dy, stepHeight, playtime);
                                    g_foot4[0]=g_foot4[0]+ dx;   //后来移动机身的参数
                                    g_foot4[1]=g_foot4[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }


                                    ROS_INFO("6.leg3");
                                    dx = cc[0]-c[0]-bodyadjX;   dy = cc[1]-c[1];
                                    error_back = Make_One_Leg_Runonce(3, c, dx, dy, stepHeight, playtime);
                                    g_foot3[0]=g_foot3[0]+ dx;   //后来移动机身的参数
                                    g_foot3[1]=g_foot3[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }



                                    ROS_INFO("7.body to zero");
                                    //                            g_v[0] = -1*bodyadjX;
                                    //                            g_w[2] = 0;
                                    //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);

                                    a[0]=a[0]+bodyadjX;  b[0]=b[0]+bodyadjX;  c[0]=c[0]+bodyadjX;  d[0]=d[0]+bodyadjX;
                                    int error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.4*playtime );
                                        usleep( 0.4*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }




                                    for (int ii=0; ii<3; ii++)
                                    {
                                        g_foot1[ii] = g_foot1_tmp[ii];  g_foot2[ii] = g_foot2_tmp[ii];
                                        g_foot3[ii] = g_foot3_tmp[ii];  g_foot4[ii] = g_foot4_tmp[ii];
                                    }


                                    //to zero
                                }
                            }


                            //turn right
                            else if (showValue[k]<0)
                            {
                                int numb=0; //定义要turn cishu
                                int shang = floor( abs(showValue[k])/eachRoa);
                                int yushu = fmod(abs(showValue[k]), eachRoa);
                                if (yushu<1) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要右转步数%d",shang,yushu,numb);

                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次右转。", i, numb);

                                    eachRoa_tmp = eachRoa;
                                    if ( (numb>shang) && (i==numb))  eachRoa_tmp=yushu;
                                    ROS_INFO("eachRoa_tmp=%d",eachRoa_tmp);

                                    //1-2-3-4
                                    //body turn + adj
                                    ROS_INFO("1.body");
                                    g_v[0] = -bodyadjX;
                                    g_w[2] = -eachRoa_tmp;
                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.8*playtime );
                                        usleep( 0.9*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }

                                    //leg
                                    ROS_INFO("2.leg1");
                                    int dx, dy;
                                    dx = aa[0]-a[0]+bodyadjX;   dy = aa[1]-a[1];
                                    error_back = Make_One_Leg_Runonce(1, a, dx, dy, stepHeight, playtime);
                                    g_foot1[0]=g_foot1[0]+ dx;   //后来移动机身的参数
                                    g_foot1[1]=g_foot1[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("3.leg2");
                                    dx = bb[0]-b[0]+bodyadjX;   dy = bb[1]-b[1];
                                    error_back = Make_One_Leg_Runonce(2, b, dx, dy, stepHeight, playtime);
                                    g_foot2[0]=g_foot2[0]+ dx;   //后来移动机身的参数
                                    g_foot2[1]=g_foot2[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("4.body");
                                    //                            g_v[0] = +2*bodyadjX;
                                    //                            g_w[2] = 0;
                                    //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                    //上面方法结果有误，下面的思路更直接,机身hou，相当于足底相对坐标均qian
                                    a[0]=a[0]-2*bodyadjX; b[0]=b[0]-2*bodyadjX; c[0]=c[0]-2*bodyadjX; d[0]=d[0]-2*bodyadjX;
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.4*playtime );
                                        usleep( 0.5*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("5.leg3");
                                    dx = cc[0]-c[0]-bodyadjX;   dy = cc[1]-c[1];
                                    error_back = Make_One_Leg_Runonce(3, c, dx, dy, stepHeight, playtime);
                                    g_foot3[0]=g_foot3[0]+ dx;   //后来移动机身的参数
                                    g_foot3[1]=g_foot3[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("6.leg4");
                                    dx = dd[0]-d[0]-bodyadjX;   dy = dd[1]-d[1];
                                    error_back = Make_One_Leg_Runonce(4, d, dx, dy, stepHeight, playtime);
                                    g_foot4[0]=g_foot4[0]+ dx;   //后来移动机身的参数
                                    g_foot4[1]=g_foot4[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("7.body to zero");
                                    //                            g_v[0] = -1*bodyadjX;
                                    //                            g_w[2] = 0;
                                    //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);

                                    a[0]=a[0]+bodyadjX;  b[0]=b[0]+bodyadjX;  c[0]=c[0]+bodyadjX;  d[0]=d[0]+bodyadjX;
                                    int error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.4*playtime );
                                        usleep( 0.35*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }

                                    for (int ii=0; ii<3; ii++)
                                    {
                                        g_foot1[ii] = g_foot1_tmp[ii];  g_foot2[ii] = g_foot2_tmp[ii];
                                        g_foot3[ii] = g_foot3_tmp[ii];  g_foot4[ii] = g_foot4_tmp[ii];
                                    }
                                    //to zero
                                }
                            }

                            ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                     g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                    g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                            ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                     a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);


                            sem_post(&sem_id);
                        }// turn left+right end

                    }



                }
                else  // climbMode is OFF.
                {
                    for (int k=0; k<8; k++)
                    {
                        if (showCode[k]==1)   //前进后退  //1-3-2-4
                        {
                            sem_wait(&sem_id);
                            g_v[0] =0; g_v[1]=0;g_w[2]=0;
                            ROS_INFO("\n\
                                     g_v=%f, %f, %f \n\
                                     g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                            // 前进
                            if (showValue[k]>0)
                            {
                                int numb=0; //定义要前进的步数
                                int shang = floor( abs(showValue[k])/stepDistance);
                                int yushu = fmod(abs(showValue[k]), stepDistance);
                                if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要前进步数%d",shang,yushu,numb);

                                int stepD_tmp;
                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次前进。", i, numb);

                                    stepD_tmp = stepDistance;
                                    if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                    ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                    ROS_INFO("单步前，各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                             g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);



                                    error_back = Make_Runonce_noGravity(vw_foot_abcd,  stepD_tmp, 0, stepHeight,  playtime, vwfa_backdata);
                                    sem_post(&sem_id);


                                    ROS_INFO("单步后：g_v = %f,%f,%f",vwfa_backdata[0], vwfa_backdata[1], vwfa_backdata[2]);
                                    ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）：\n g_foot1[x,y,z] = %f,%f,%f \n g_foot2[x,y,z] = %f,%f,%f \n g_foot3[x,y,z] = %f,%f,%f \n g_foot4[x,y,z] = %f,%f,%f",
                                             vwfa_backdata[6],vwfa_backdata[7],vwfa_backdata[8],
                                            vwfa_backdata[9],vwfa_backdata[10],vwfa_backdata[11],
                                            vwfa_backdata[12],vwfa_backdata[13],vwfa_backdata[14],
                                            vwfa_backdata[15],vwfa_backdata[16],vwfa_backdata[17]);
                                    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                             vwfa_backdata[18],vwfa_backdata[19],vwfa_backdata[20],
                                            vwfa_backdata[21],vwfa_backdata[22],vwfa_backdata[23],
                                            vwfa_backdata[24],vwfa_backdata[25],vwfa_backdata[26],
                                            vwfa_backdata[27],vwfa_backdata[28],vwfa_backdata[29]);


//                                    int ssf_D = 80, ssf_H=80, dz=stepHeight;
//                                    error_back = Climb_stepsurface_noGravity( vw_foot_abcd, ssf_D, ssf_H, dz, playtime, vwfa_backdata );

                                }
                            }

                            // 后退
                            else if (showValue[k]<0)
                            {
                                int numb=0; //定义要后退的步数
                                int shang = floor( abs(showValue[k])/stepDistance);
                                int yushu = fmod(abs(showValue[k]), stepDistance);
                                if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要后退步数%d",shang,yushu,numb);

                                int stepD_tmp;
                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次后退。", i, numb);

                                    stepD_tmp = stepDistance;
                                    if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                    ROS_INFO("stepD_tmp=%d",stepD_tmp);


                                    error_back = Make_Runonce_noGravity(vw_foot_abcd,  -stepD_tmp, 0, stepHeight,  playtime, vwfa_backdata);
                                    sem_post(&sem_id);
                                    //以上步骤结束后，四条腿回归原位置
                                }
                            }//hou tui

                            sem_post(&sem_id);
                        }// forward+backward end

                        else if (showCode[k]==2)   //左移右移
                        {
                            sem_wait(&sem_id);
                            g_v[0] =0; g_v[1]=0;g_w[2]=0;

                            int stepDistance_H = stepDistance;
                            if (stepDistance_H>100) stepDistance_H=100;  //左移/右移步长最高为120mm。
                            //move left
                            if (showValue[k]>0)
                            {
                                int numb=0; //定义要move步数
                                int shang = floor( abs(showValue[k])/stepDistance_H);
                                int yushu = fmod(abs(showValue[k]), stepDistance_H);
                                if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要左移步数%d",shang,yushu,numb);

                                int stepD_tmp;
                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次左移。", i, numb);

                                    stepD_tmp = stepDistance_H;
                                    if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                    ROS_INFO("stepD_tmp=%d",stepD_tmp);


                                    error_back = Make_Runonce_noGravity(vw_foot_abcd, 0, stepD_tmp, stepHeight,  playtime, vwfa_backdata);
                                    sem_post(&sem_id);
                                    //以上步骤结束后，四条腿回归原位置
                                }

                            }// move left end


                            //move right
                            else if (showValue[k]<0)
                            {
                                int numb=0; //定义要move步数
                                int shang = floor( abs(showValue[k])/stepDistance_H);
                                int yushu = fmod(abs(showValue[k]), stepDistance_H);
                                if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要右移步数%d",shang,yushu,numb);

                                int stepD_tmp;
                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次右移。", i, numb);

                                    stepD_tmp = stepDistance_H;
                                    if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                    ROS_INFO("stepD_tmp=%d",stepD_tmp);


                                    error_back = Make_Runonce_noGravity(vw_foot_abcd, 0, -stepD_tmp, stepHeight,  playtime, vwfa_backdata);
                                    sem_post(&sem_id);
                                    //以上步骤结束后，四条腿回归原位置

                                    //以上步骤结束后，四条腿回归原位置
                                }

                            }// move right end

                            sem_post(&sem_id);
                        }//else if (showCode[k]==2) //左移右移 jieshu

                        else if (showCode[k]==3)   //左转右转
                        {
                            sem_wait(&sem_id);
                            g_v[0] =0; g_v[1]=0;g_w[2]=0;

                            int eachRoa = 15;  //每20度转一次
                            int eachRoa_tmp;
                            // 定义aa,bb,cc,dd 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm, 为目标; abcd为实时当前
                            float aa[3],bb[3],cc[3],dd[3];
                            float g_foot1_tmp[3],g_foot2_tmp[3],g_foot3_tmp[3],g_foot4_tmp[3];
                            for (int ii=0; ii<3; ii++)
                            {
                                aa[ii]=a[ii]; bb[ii]=b[ii]; cc[ii]=c[ii]; dd[ii]=d[ii];
                                g_foot1_tmp[ii] = g_foot1[ii];   g_foot2_tmp[ii] = g_foot2[ii];
                                g_foot3_tmp[ii] = g_foot3[ii];   g_foot4_tmp[ii] = g_foot4[ii];
                            }

                            ROS_INFO("\n aa[x,y,z] = %f,%f,%f \n bb[x,y,z] = %f,%f,%f \n cc[x,y,z] = %f,%f,%f \n dd[x,y,z] = %f,%f,%f",
                                     aa[0], aa[1],aa[2], bb[0], bb[1],bb[2] ,cc[0], cc[1],cc[2] ,dd[0], dd[1],dd[2]);


                            //turn left
                            if (showValue[k]>0)
                            {
                                int numb=0; //定义要turn cishu
                                int shang = floor( abs(showValue[k])/eachRoa);
                                int yushu = fmod(abs(showValue[k]), eachRoa);
                                if (yushu<1) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要左转步数%d",shang,yushu,numb);

                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次左转。", i, numb);

                                    eachRoa_tmp = eachRoa;
                                    if ( (numb>shang) && (i==numb))  eachRoa_tmp=yushu;
                                    ROS_INFO("eachRoa_tmp=%d",eachRoa_tmp);

                                    //2-1-4-3
                                    //body turn + adj
                                    ROS_INFO("1.body");

                                    ROS_INFO("\n\
                                             g_v=%f, %f, %f \n\
                                             g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                                             g_v[0] = -bodyadjX;
                                    g_w[2] = eachRoa_tmp;
                                    ROS_INFO("\n\
                                             g_v=%f, %f, %f \n\
                                             g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                                             ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                                      g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.8*playtime );
                                        usleep( 0.9*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }


                                    //leg
                                    ROS_INFO("2.leg2");
                                    int dx, dy;
                                    dx = bb[0]-b[0]+bodyadjX;   dy = bb[1]-b[1];
                                    error_back = Make_One_Leg_Runonce(2, b, dx, dy, stepHeight, playtime);
                                    g_foot2[0]=g_foot2[0]+ dx;   //后来移动机身的参数
                                    g_foot2[1]=g_foot2[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }


                                    ROS_INFO("3.leg1");
                                    dx = aa[0]-a[0]+bodyadjX;   dy = aa[1]-a[1];
                                    error_back = Make_One_Leg_Runonce(1, a, dx, dy, stepHeight, playtime);
                                    g_foot1[0]=g_foot1[0]+ dx;   //后来移动机身的参数
                                    g_foot1[1]=g_foot1[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }


                                    ROS_INFO("4.body");
                                    //                            g_v[0] = +2*bodyadjX;
                                    //                            g_w[2] = 0;
                                    //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                    //上面方法结果有误，下面的思路更直接,机身前进，相当于足底相对坐标均后退
                                    a[0]=a[0]-2*bodyadjX; b[0]=b[0]-2*bodyadjX; c[0]=c[0]-2*bodyadjX; d[0]=d[0]-2*bodyadjX;
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.4*playtime );
                                        usleep( 0.5*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }




                                    ROS_INFO("5.leg4");
                                    dx = dd[0]-d[0]-bodyadjX;   dy = dd[1]-d[1];
                                    error_back = Make_One_Leg_Runonce(4, d, dx, dy, stepHeight, playtime);
                                    g_foot4[0]=g_foot4[0]+ dx;   //后来移动机身的参数
                                    g_foot4[1]=g_foot4[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }



                                    ROS_INFO("6.leg3");
                                    dx = cc[0]-c[0]-bodyadjX;   dy = cc[1]-c[1];
                                    error_back = Make_One_Leg_Runonce(3, c, dx, dy, stepHeight, playtime);
                                    g_foot3[0]=g_foot3[0]+ dx;   //后来移动机身的参数
                                    g_foot3[1]=g_foot3[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }



                                    ROS_INFO("7.body to zero");
                                    //                            g_v[0] = -1*bodyadjX;
                                    //                            g_w[2] = 0;
                                    //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);

                                    a[0]=a[0]+bodyadjX;  b[0]=b[0]+bodyadjX;  c[0]=c[0]+bodyadjX;  d[0]=d[0]+bodyadjX;
                                    int error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.4*playtime );
                                        usleep( 0.4*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }



                                    for (int ii=0; ii<3; ii++)
                                    {
                                        g_foot1[ii] = g_foot1_tmp[ii];  g_foot2[ii] = g_foot2_tmp[ii];
                                        g_foot3[ii] = g_foot3_tmp[ii];  g_foot4[ii] = g_foot4_tmp[ii];
                                    }



                                    //to zero
                                }
                            }


                            //turn right
                            else if (showValue[k]<0)
                            {
                                int numb=0; //定义要turn cishu
                                int shang = floor( abs(showValue[k])/eachRoa);
                                int yushu = fmod(abs(showValue[k]), eachRoa);
                                if (yushu<1) numb=shang;     //余数太小，最后一步就不走了
                                else numb=shang+1;
                                ROS_INFO("商%d,余数%d,需要右转步数%d",shang,yushu,numb);

                                for (int i=1; i<numb+1; i++)
                                {
                                    ROS_WARN("第%d/%d次右转。", i, numb);

                                    eachRoa_tmp = eachRoa;
                                    if ( (numb>shang) && (i==numb))  eachRoa_tmp=yushu;
                                    ROS_INFO("eachRoa_tmp=%d",eachRoa_tmp);

                                    //1-2-3-4
                                    //body turn + adj
                                    ROS_INFO("1.body");
                                    g_v[0] = -bodyadjX;
                                    g_w[2] = -eachRoa_tmp;
                                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.8*playtime );
                                        usleep( 0.9*playtime*1000);
                                    }
                                    //调整机身方向
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }

                                    //leg
                                    ROS_INFO("2.leg1");
                                    int dx, dy;
                                    dx = aa[0]-a[0]+bodyadjX;   dy = aa[1]-a[1];
                                    error_back = Make_One_Leg_Runonce(1, a, dx, dy, stepHeight, playtime);
                                    g_foot1[0]=g_foot1[0]+ dx;   //后来移动机身的参数
                                    g_foot1[1]=g_foot1[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("3.leg2");
                                    dx = bb[0]-b[0]+bodyadjX;   dy = bb[1]-b[1];
                                    error_back = Make_One_Leg_Runonce(2, b, dx, dy, stepHeight, playtime);
                                    g_foot2[0]=g_foot2[0]+ dx;   //后来移动机身的参数
                                    g_foot2[1]=g_foot2[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("4.body");
                                    //                            g_v[0] = +2*bodyadjX;
                                    //                            g_w[2] = 0;
                                    //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                    //上面方法结果有误，下面的思路更直接,机身hou，相当于足底相对坐标均qian
                                    a[0]=a[0]-2*bodyadjX; b[0]=b[0]-2*bodyadjX; c[0]=c[0]-2*bodyadjX; d[0]=d[0]-2*bodyadjX;
                                    error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.4*playtime );
                                        usleep( 0.5*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("5.leg3");
                                    dx = cc[0]-c[0]-bodyadjX;   dy = cc[1]-c[1];
                                    error_back = Make_One_Leg_Runonce(3, c, dx, dy, stepHeight, playtime);
                                    g_foot3[0]=g_foot3[0]+ dx;   //后来移动机身的参数
                                    g_foot3[1]=g_foot3[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("6.leg4");
                                    dx = dd[0]-d[0]-bodyadjX;   dy = dd[1]-d[1];
                                    error_back = Make_One_Leg_Runonce(4, d, dx, dy, stepHeight, playtime);
                                    g_foot4[0]=g_foot4[0]+ dx;   //后来移动机身的参数
                                    g_foot4[1]=g_foot4[1]+ dy;
                                    if (error_back == 1 || !showMOVE)
                                    {
                                        ROS_WARN("error_back == 1");
                                        sem_post(&sem_id);   return;
                                    }

                                    ROS_INFO("7.body to zero");
                                    //                            g_v[0] = -1*bodyadjX;
                                    //                            g_w[2] = 0;
                                    //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);

                                    a[0]=a[0]+bodyadjX;  b[0]=b[0]+bodyadjX;  c[0]=c[0]+bodyadjX;  d[0]=d[0]+bodyadjX;
                                    int error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                    if(error_back == 0  && showMOVE)
                                    {
                                        Send_ALL_Leg_Positon( 0.4*playtime );
                                        usleep( 0.35*playtime*1000);
                                    }
                                    else
                                    {
                                        ROS_ERROR("第%d次调整机身方向出错！", i);
                                        sem_post(&sem_id);   return;
                                    }

                                    for (int ii=0; ii<3; ii++)
                                    {
                                        g_foot1[ii] = g_foot1_tmp[ii];  g_foot2[ii] = g_foot2_tmp[ii];
                                        g_foot3[ii] = g_foot3_tmp[ii];  g_foot4[ii] = g_foot4_tmp[ii];
                                    }
                                    //to zero
                                }
                            }

                            ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                     g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                    g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                            ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                     a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);


                            sem_post(&sem_id);
                        }// turn left+right end

                        else if (showCode[k]==4)   //爬台阶
                        {
                            int ssf_D = zhangaiwuD;
                            int ssf_H = showValue[k];

                           int dz = stepHeight;
                            ROS_INFO("台阶高度%d", ssf_H);

                            error_back = Climb_stepsurface_noGravity( vw_foot_abcd, ssf_D, ssf_H, dz, playtime, vwfa_backdata );
//爬台阶
                        }


                    }
                }
            } 

            else if (gaitType == xleg_msgs::gaitControl::Request::GAIT_TROT)
            {
                ROS_INFO("当前为对角步态。");
                for (int k=0; k<8; k++)
                {
                    if (showCode[k]==1)   //前进后退  //1-3-2-4
                    {
                        sem_wait(&sem_id);
                        g_v[0] =0; g_v[1]=0;g_w[2]=0;
                        ROS_INFO("\n\
                                 g_v=%f, %f, %f \n\
                                 g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                        // 前进
                        if (showValue[k]>0)
                        {
                            int numb=0; //定义要前进的步数
                            int shang = floor( abs(showValue[k])/stepDistance);
                            int yushu = fmod(abs(showValue[k]), stepDistance);
                            if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要前进步数%d",shang,yushu,numb);

                            int stepD_tmp;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次前进。", i, numb);
                                stepD_tmp = stepDistance;
                                if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                //13-body-24
                                //腿1,3进
                                ROS_INFO("1.leg1+leg3");
                                error_back = Make_Two_Legs_Runonce(1, 3, a, c,stepD_tmp, 0, stepHeight,
                                                                   stepD_tmp, 0, stepHeight, playtime);
                                g_foot1[0]=g_foot1[0]+stepD_tmp;   //后来移动机身的参数
                                g_foot3[0]=g_foot3[0]+stepD_tmp;
                                if (error_back >0 || !showMOVE)
                                {
                                    ROS_WARN("error_back = %d", error_back);
                                    sem_post(&sem_id);   return;
                                }

                                //body
                                ROS_INFO("2.body move");
                                g_v[0] = stepD_tmp;
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0 && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
                                    usleep( 0.8*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }

                                //腿2,4进
                                ROS_INFO("3.leg2+leg4");
                                error_back = Make_Two_Legs_Runonce(2, 4, b, d,stepD_tmp, 0, stepHeight,
                                                                   stepD_tmp, 0, stepHeight, playtime);
                                g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
                                g_foot4[0]=g_foot4[0]+stepD_tmp;
                                if (error_back >0 || !showMOVE)
                                {
                                    ROS_WARN("error_back = %d", error_back);
                                    sem_post(&sem_id);   return;
                                }
                                //以上步骤结束后，四条腿回归原位置
                            }
                        }

                        // 后退
                        else if (showValue[k]<0)
                        {
                            int numb=0; //定义要后退的步数
                            int shang = floor( abs(showValue[k])/stepDistance);
                            int yushu = fmod(abs(showValue[k]), stepDistance);
                            if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要后退步数%d",shang,yushu,numb);

                            int stepD_tmp;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次后退。", i, numb);

                                stepD_tmp = stepDistance;
                                if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                //42-body-31
                                //腿2,4 back
                                ROS_INFO("1.leg2+leg4");
                                error_back = Make_Two_Legs_Runonce(2, 4, b, d,-stepD_tmp, 0, stepHeight,
                                                                   -stepD_tmp, 0, stepHeight, playtime);
                                g_foot2[0]=g_foot2[0]-stepD_tmp;   //后来移动机身的参数
                                g_foot4[0]=g_foot4[0]-stepD_tmp;
                                if (error_back >0 || !showMOVE)
                                {
                                    ROS_WARN("error_back = %d", error_back);
                                    sem_post(&sem_id);   return;
                                }

                                //body
                                ROS_INFO("2.body move");
                                g_v[0] = -stepD_tmp;
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0 && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
                                    usleep( 0.8*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }

                                //腿2,4 back
                                ROS_INFO("3.leg1+leg3");
                                error_back = Make_Two_Legs_Runonce(1, 3, a, c,-stepD_tmp, 0, stepHeight,
                                                                   -stepD_tmp, 0, stepHeight, playtime);
                                g_foot1[0]=g_foot1[0]-stepD_tmp;   //后来移动机身的参数
                                g_foot3[0]=g_foot3[0]-stepD_tmp;
                                if (error_back >0 || !showMOVE)
                                {
                                    ROS_WARN("error_back = %d", error_back);
                                    sem_post(&sem_id);   return;
                                }
                                //以上步骤结束后，四条腿回归原位置

                                ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                         g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);
                            }
                        }//hou tui

                        sem_post(&sem_id);
                    }// forward+backward end

                    else if (showCode[k]==2)   //左移右移
                    {
                        sem_wait(&sem_id);
                        g_v[0] =0; g_v[1]=0;g_w[2]=0;

                        int stepDistance_H = stepDistance;
                        if (stepDistance_H>100) stepDistance_H=100;  //左移/右移步长最高为120mm。
                        //move left
                        if (showValue[k]>0)
                        {
                            int numb=0; //定义要move步数
                            int shang = floor( abs(showValue[k])/stepDistance_H);
                            int yushu = fmod(abs(showValue[k]), stepDistance_H);
                            if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要左移步数%d",shang,yushu,numb);

                            int stepD_tmp;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次左移。", i, numb);

                                stepD_tmp = stepDistance_H;
                                if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                //13-body-24
                                //腿1,3进
                                ROS_INFO("1.leg1+leg3");
                                error_back = Make_Two_Legs_Runonce(1, 3, a, c, 0, stepD_tmp, stepHeight,
                                                                   0, stepD_tmp, stepHeight, playtime);
                                g_foot1[1]=g_foot1[1]+stepD_tmp;   //后来移动机身的参数
                                g_foot3[1]=g_foot3[1]+stepD_tmp;
                                if (error_back >0 || !showMOVE)
                                {
                                    ROS_WARN("error_back = %d", error_back);
                                    sem_post(&sem_id);   return;
                                }

                                //body
                                ROS_INFO("2.body move");
                                g_v[1] = stepD_tmp;
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0 && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
                                    usleep( 0.8*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }

                                //腿2,4进
                                ROS_INFO("3.leg2+leg4");
                                error_back = Make_Two_Legs_Runonce(2, 4, b, d, 0, stepD_tmp, stepHeight,
                                                                   0, stepD_tmp, stepHeight, playtime);
                                g_foot2[1]=g_foot2[1]+stepD_tmp;   //后来移动机身的参数
                                g_foot4[1]=g_foot4[1]+stepD_tmp;
                                if (error_back >0 || !showMOVE)
                                {
                                    ROS_WARN("error_back = %d", error_back);
                                    sem_post(&sem_id);   return;
                                }



                                ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                         g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);
                                //以上步骤结束后，四条腿回归原位置
                            }

                        }// move left end


                        //move right
                        else if (showValue[k]<0)
                        {
                            int numb=0; //定义要move步数
                            int shang = floor( abs(showValue[k])/stepDistance_H);
                            int yushu = fmod(abs(showValue[k]), stepDistance_H);
                            if (yushu<10) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要右移步数%d",shang,yushu,numb);

                            int stepD_tmp;
                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次右移。", i, numb);

                                stepD_tmp = stepDistance_H;
                                if ( (numb>shang) && (i==numb))  stepD_tmp=yushu;
                                ROS_INFO("stepD_tmp=%d",stepD_tmp);

                                //42-body-31
                                //腿2,4 back
                                ROS_INFO("1.leg2+leg4");
                                error_back = Make_Two_Legs_Runonce(2, 4, b, d, 0, -stepD_tmp, stepHeight,
                                                                   0, -stepD_tmp, stepHeight, playtime);
                                g_foot2[1]=g_foot2[1]-stepD_tmp;   //后来移动机身的参数
                                g_foot4[1]=g_foot4[1]-stepD_tmp;
                                if (error_back >0 || !showMOVE)
                                {
                                    ROS_WARN("error_back = %d", error_back);
                                    sem_post(&sem_id);   return;
                                }

                                //body
                                ROS_INFO("2.body move");
                                g_v[1] = -stepD_tmp;
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0 && showMOVE)
                                {
                                    Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
                                    usleep( 0.8*playtime*1000);
                                }
                                else
                                {
                                    ROS_INFO("机身进出错。 error_codes=%d",error_back );
                                    sem_post(&sem_id);   return;
                                }

                                //腿2,4 back
                                ROS_INFO("3.leg1+leg3");
                                error_back = Make_Two_Legs_Runonce(1, 3, a, c, 0, -stepD_tmp, stepHeight,
                                                                   0, -stepD_tmp, stepHeight, playtime);
                                g_foot1[1]=g_foot1[1]-stepD_tmp;   //后来移动机身的参数
                                g_foot3[1]=g_foot3[1]-stepD_tmp;
                                if (error_back >0 || !showMOVE)
                                {
                                    ROS_WARN("error_back = %d", error_back);
                                    sem_post(&sem_id);   return;
                                }
                                //以上步骤结束后，四条腿回归原位置

                                ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                         g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);
                                //以上步骤结束后，四条腿回归原位置
                            }

                        }// move right end

                        sem_post(&sem_id);
                    }//else if (showCode[k]==2) //左移右移 jieshu

                    else if (showCode[k]==3)   //左转右转
                    {
                        sem_wait(&sem_id);
                        g_v[0] =0; g_v[1]=0;g_w[2]=0;

                        int eachRoa = 15;  //每20度转一次
                        int eachRoa_tmp;
                        // 定义aa,bb,cc,dd 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm, 为目标; abcd为实时当前
                        float aa[3],bb[3],cc[3],dd[3];
                        float g_foot1_tmp[3],g_foot2_tmp[3],g_foot3_tmp[3],g_foot4_tmp[3];
                        for (int ii=0; ii<3; ii++)
                        {
                            aa[ii]=a[ii]; bb[ii]=b[ii]; cc[ii]=c[ii]; dd[ii]=d[ii];
                            g_foot1_tmp[ii] = g_foot1[ii];   g_foot2_tmp[ii] = g_foot2[ii];
                            g_foot3_tmp[ii] = g_foot3[ii];   g_foot4_tmp[ii] = g_foot4[ii];
                        }

                        //turn left
                        if (showValue[k]>0)
                        {
                            int numb=0; //定义要turn cishu
                            int shang = floor( abs(showValue[k])/eachRoa);
                            int yushu = fmod(abs(showValue[k]), eachRoa);
                            if (yushu<1) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要左转步数%d",shang,yushu,numb);

                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次左转。", i, numb);

                                eachRoa_tmp = eachRoa;
                                if ( (numb>shang) && (i==numb))  eachRoa_tmp=yushu;
                                ROS_INFO("eachRoa_tmp=%d",eachRoa_tmp);

                                //2-1-4-3
                                //body turn + adj
                                ROS_INFO("1.body");

                                ROS_INFO("\n\
                                         g_v=%f, %f, %f \n\
                                         g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                                         g_v[0] = -bodyadjX;
                                g_w[2] = eachRoa_tmp;
                                ROS_INFO("\n\
                                         g_v=%f, %f, %f \n\
                                         g_w=%f, %f, %f", g_v[0], g_v[1], g_v[2], g_w[0], g_w[1], g_w[2]);

                                         ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
                                                  g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
                                        g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
                                ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
                                         a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.8*playtime );
                                    usleep( 0.9*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }



                                //leg
                                ROS_INFO("2.leg2");
                                int dx, dy;
                                dx = bb[0]-b[0]+bodyadjX;   dy = bb[1]-b[1];
                                error_back = Make_One_Leg_Runonce(2, b, dx, dy, stepHeight, playtime);
                                g_foot2[0]=g_foot2[0]+ dx;   //后来移动机身的参数
                                g_foot2[1]=g_foot2[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }


                                ROS_INFO("3.leg1");
                                dx = aa[0]-a[0]+bodyadjX;   dy = aa[1]-a[1];
                                error_back = Make_One_Leg_Runonce(1, a, dx, dy, stepHeight, playtime);
                                g_foot1[0]=g_foot1[0]+ dx;   //后来移动机身的参数
                                g_foot1[1]=g_foot1[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("4.body");
                                //                            g_v[0] = +2*bodyadjX;
                                //                            g_w[2] = 0;
                                //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                //上面方法结果有误，下面的思路更直接,机身前进，相当于足底相对坐标均后退
                                a[0]=a[0]-2*bodyadjX; b[0]=b[0]-2*bodyadjX; c[0]=c[0]-2*bodyadjX; d[0]=d[0]-2*bodyadjX;
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.4*playtime );
                                    usleep( 0.5*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }


                                ROS_INFO("5.leg4");
                                dx = dd[0]-d[0]-bodyadjX;   dy = dd[1]-d[1];
                                error_back = Make_One_Leg_Runonce(4, d, dx, dy, stepHeight, playtime);
                                g_foot4[0]=g_foot4[0]+ dx;   //后来移动机身的参数
                                g_foot4[1]=g_foot4[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("6.leg3");
                                dx = cc[0]-c[0]-bodyadjX;   dy = cc[1]-c[1];
                                error_back = Make_One_Leg_Runonce(3, c, dx, dy, stepHeight, playtime);
                                g_foot3[0]=g_foot3[0]+ dx;   //后来移动机身的参数
                                g_foot3[1]=g_foot3[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }




                                ROS_INFO("7.body to zero");
                                //                            g_v[0] = -1*bodyadjX;
                                //                            g_w[2] = 0;
                                //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);

                                a[0]=a[0]+bodyadjX;  b[0]=b[0]+bodyadjX;  c[0]=c[0]+bodyadjX;  d[0]=d[0]+bodyadjX;
                                int error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.4*playtime );
                                    usleep( 0.4*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }


                                for (int ii=0; ii<3; ii++)
                                {
                                    g_foot1[ii] = g_foot1_tmp[ii];  g_foot2[ii] = g_foot2_tmp[ii];
                                    g_foot3[ii] = g_foot3_tmp[ii];  g_foot4[ii] = g_foot4_tmp[ii];
                                }


                                //to zero
                            }
                        }


                        //turn right
                        else if (showValue[k]<0)
                        {
                            int numb=0; //定义要turn cishu
                            int shang = floor( abs(showValue[k])/eachRoa);
                            int yushu = fmod(abs(showValue[k]), eachRoa);
                            if (yushu<1) numb=shang;     //余数太小，最后一步就不走了
                            else numb=shang+1;
                            ROS_INFO("商%d,余数%d,需要右转步数%d",shang,yushu,numb);

                            for (int i=1; i<numb+1; i++)
                            {
                                ROS_WARN("第%d/%d次右转。", i, numb);

                                eachRoa_tmp = eachRoa;
                                if ( (numb>shang) && (i==numb))  eachRoa_tmp=yushu;
                                ROS_INFO("eachRoa_tmp=%d",eachRoa_tmp);

                                //1-2-3-4
                                //body turn + adj
                                ROS_INFO("1.body");
                                g_v[0] = -bodyadjX;
                                g_w[2] = -eachRoa_tmp;
                                CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.8*playtime );
                                    usleep( 0.9*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }

                                //leg
                                ROS_INFO("2.leg1");
                                int dx, dy;
                                dx = aa[0]-a[0]+bodyadjX;   dy = aa[1]-a[1];
                                error_back = Make_One_Leg_Runonce(1, a, dx, dy, stepHeight, playtime);
                                g_foot1[0]=g_foot1[0]+ dx;   //后来移动机身的参数
                                g_foot1[1]=g_foot1[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("3.leg2");
                                dx = bb[0]-b[0]+bodyadjX;   dy = bb[1]-b[1];
                                error_back = Make_One_Leg_Runonce(2, b, dx, dy, stepHeight, playtime);
                                g_foot2[0]=g_foot2[0]+ dx;   //后来移动机身的参数
                                g_foot2[1]=g_foot2[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("4.body");
                                //                            g_v[0] = +2*bodyadjX;
                                //                            g_w[2] = 0;
                                //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
                                //上面方法结果有误，下面的思路更直接,机身hou，相当于足底相对坐标均qian
                                a[0]=a[0]-2*bodyadjX; b[0]=b[0]-2*bodyadjX; c[0]=c[0]-2*bodyadjX; d[0]=d[0]-2*bodyadjX;
                                error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.4*playtime );
                                    usleep( 0.5*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("5.leg3");
                                dx = cc[0]-c[0]-bodyadjX;   dy = cc[1]-c[1];
                                error_back = Make_One_Leg_Runonce(3, c, dx, dy, stepHeight, playtime);
                                g_foot3[0]=g_foot3[0]+ dx;   //后来移动机身的参数
                                g_foot3[1]=g_foot3[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("6.leg4");
                                dx = dd[0]-d[0]-bodyadjX;   dy = dd[1]-d[1];
                                error_back = Make_One_Leg_Runonce(4, d, dx, dy, stepHeight, playtime);
                                g_foot4[0]=g_foot4[0]+ dx;   //后来移动机身的参数
                                g_foot4[1]=g_foot4[1]+ dy;
                                if (error_back == 1 || !showMOVE)
                                {
                                    ROS_WARN("error_back == 1");
                                    sem_post(&sem_id);   return;
                                }

                                ROS_INFO("7.body to zero");
                                //                            g_v[0] = -1*bodyadjX;
                                //                            g_w[2] = 0;
                                //                            CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);

                                a[0]=a[0]+bodyadjX;  b[0]=b[0]+bodyadjX;  c[0]=c[0]+bodyadjX;  d[0]=d[0]+bodyadjX;
                                int error_back = Set_ALL_Leg_Positon(a,b,c,d);
                                if(error_back == 0  && showMOVE)
                                {
                                    Send_ALL_Leg_Positon( 0.4*playtime );
                                    usleep( 0.35*playtime*1000);
                                }
                                else
                                {
                                    ROS_ERROR("第%d次调整机身方向出错！", i);
                                    sem_post(&sem_id);   return;
                                }

                                for (int ii=0; ii<3; ii++)
                                {
                                    g_foot1[ii] = g_foot1_tmp[ii];  g_foot2[ii] = g_foot2_tmp[ii];
                                    g_foot3[ii] = g_foot3_tmp[ii];  g_foot4[ii] = g_foot4_tmp[ii];
                                }
                                //to zero
                            }
                        }

                        sem_post(&sem_id);
                    }// turn left+right end

                }






            }

        }



    }

    gaitReadDxldataNow = false;
}


//======================  保存力数据 dataSave.src     ros::ServiceServer SrvSavedate    =======================
bool service_dataSave(xleg_msgs::dataSave::Request  &req,  xleg_msgs::dataSave::Response &res)
{
    dataname = req.filename;
    //        dataname= "/home/qxs-kjsp/" + dataname + ".txt";
    cout<<"dataname:  "<<dataname<<endl;

    if(req.startSaveData == true)
    {
        ROS_INFO("Start to save force data.\n\n");
        stopSaveData = false;

        std::thread force_thread(readForce_thread);
        force_thread.detach();

        res.error_codes = 0;
    }
    else
    {
        ROS_WARN("STOP to save data.\n\n");
        stopSaveData = true;
        res.error_codes = 1;
    }
}


/* =================================   采集各腿力数据，并保存    =================================
     *
     */
unsigned char togetF[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0E, 0xC4, 0x0E};
unsigned int dn = 33;       // dn = ser.available();
unsigned char getforce[33];    //接收缓存
void readForce_thread()
{
    ofstream out_text_file;

    string path_name = "/home/qxs-kjsp/" + dataname + ".txt";
    cout<<"path_name:  "<<path_name<<endl;
    out_text_file.open(path_name, ios::out | ios::trunc);
    out_text_file<<fixed;

    while(1)
    {
        //            ROS_INFO("Start to save force data.\n");
        ser.write(togetF, 8);
        ser.read( getforce, dn);       // dn =33?
        ser.flush();

        // 返回33字节，第8-31共24字节是力值
        if ( dn==33 )
        {
            for (int j=0; j<12; j++)
            {
                Foot_present_forces[j] = ( (signed short int) ( getforce[2*(j+4)-1]*256+getforce[2*(j+4)] )  ) /10.0 ;      // 有符号数   足底力信息全局变量，存在数组中。
                //                    ROS_INFO("Foot_present_forces[%d]=%f\n", j, Foot_present_forces[j]);
            }

            if(!stopSaveData)
            {
                for (int j=0; j<12; j++)
                {
                    out_text_file << setprecision(2) << Foot_present_forces[j] << " ";
                }
                out_text_file << endl;
            }
            else
                out_text_file.close();
        }

        usleep(2*1000);   // 2ms  采集频率 500Hz
    }
    //        out_text_file.close();
    ROS_WARN("STOP readForce_thread.\n\n");
}


/* =================================   开启机器视觉    =======================================================================
     *
     */
Mat imageRaw;
float pos_x=0;     //鼠标位置 全局变量   图片中的相对位置 1280x720
float pos_y=0;
float effective_distance = 0;
int exit_flag = 0;     //









标右键 exit_flag=1
int error_num = 1;    //0无错，1没有点选， 2,3无相机
int mouse_flag =0;    // 1左键
float position_in_Img[2]={0};
float position_to_Camera[3]={0};
bool DataReady = false;  //获取到目标就 true
int ChooseObject = 0;

#define IMG_W   640   // 1280    //640
#define IMG_H    480    //  720   480

//============================= 获取深度像素对应长度单位（米）的换算比例   =====================================
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
//==================================    深度图对齐到彩色图函数     =========================================
Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile){
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    //auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics  extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3],Pcc3[3];

    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    int y=0,x=0;
    //初始化结果
    //Mat result=Mat(color.rows,color.cols,CV_8UC3,Scalar(0,0,0));
    Mat result=Mat(color.rows,color.cols,CV_16U,Scalar(0));
    //对深度图像遍历
    for(int row=0;row<depth.rows;row++){
        for(int col=0;col<depth.cols;col++){
            //将当前的(x,y)放入数组pd_uv，表示当前深度图的点
            pd_uv[0]=col;
            pd_uv[1]=row;
            //取当前点对应的深度值
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            //换算到米
            float depth_m=depth_value*depth_scale;
            //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //将彩色摄像头坐标系下的深度三维点映射到二维平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            //取得映射后的（u,v)
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];
            //            if(x<0||x>color.cols)
            //                continue;
            //            if(y<0||y>color.rows)
            //                continue;
            //最值限定
            x=x<0? 0:x;
            x=x>depth.cols-1 ? depth.cols-1:x;
            y=y<0? 0:y;
            y=y>depth.rows-1 ? depth.rows-1:y;

            result.at<uint16_t>(y,x)=depth_value;
        }
    }
    //返回一个与彩色图对齐了的深度信息图像
    return result;
}
// ------------------------- 鼠标选点
void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
    // cout<<"   测试测试   "<<endl;

    Mat hh;
    hh = *(Mat*)userdata;
    Point pp(x, y);

    switch (EVENT)
    {
    // 每个case后，必须要有break
    case  EVENT_LBUTTONDOWN:     //左键单击
    {
        //            char Txt_Point[20];
        //            sprintf(Txt_Point, "(%d,%d)", x, y);
        //            putText(hh, Txt_Point, pp,	FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0, 0, 255), 2, 8);      //依次参数：图片，带put内容，位置，字体，字体大小，颜色，粗细，线型
        //            circle(hh, pp, 8, Scalar(0, 0, 255), 2, CV_AA, 0);//画圆依次参数：图片，圆心位置，半径，颜色，线宽(如<0则实心)，线型，圆心和半径小数点位数
        pos_x=x;
        pos_y=y;
        mouse_flag=1;
    }
        break;
    case CV_EVENT_MBUTTONDOWN:   //中 键单击   或： EVENT_MBUTTONDOWN
    {
        exit_flag=1;
    }
        break;
    }
    imshow("color_Image", hh);
}

//=============================================                              ========================================
void measure_distance(Mat &color,Mat depth,cv::Size range,rs2::pipeline_profile profile,  int x, int y)
{
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    //定义测量位置
    // cv::Point center(color.cols/2,color.rows/2);
    cv::Point center(x,  y);
    //    cout<<"鼠标点击位置: x="<<x<<";     y="<<y<<endl;

    //定义计算距离的范围
    cv::Rect RectRange(center.x-range.width/2,center.y-range.height/2,range.width,range.height);
    //遍历该范围
    float distance_sum=0;
    int effective_pixel=0;
    for(int y=RectRange.y;y<RectRange.y+RectRange.height;y++){
        for(int x=RectRange.x;x<RectRange.x+RectRange.width;x++){
            //如果深度图下该点像素不为0，表示有距离信息
            if(depth.at<uint16_t>(y,x)){
                distance_sum+=depth_scale*depth.at<uint16_t>(y,x);
                effective_pixel++;
            }
        }
    }

    effective_distance=distance_sum/effective_pixel;    //XIE    float effective_distance
}
//    =================================================================================================================
void startVision_thread()
{
    try
    {
        error_num = 1;    //0无错，1没有点选， 2,3无相机
        mouse_flag =0;    // 1左键

        //创建数据管道
        rs2::pipeline pipe;
        rs2::config pipe_config;  //pipe_config
        pipe_config.enable_stream(RS2_STREAM_DEPTH,IMG_W,IMG_H,RS2_FORMAT_Z16,30);
        pipe_config.enable_stream(RS2_STREAM_COLOR,IMG_W,IMG_H,RS2_FORMAT_BGR8,30);

        //start()函数返回数据管道的profile
        rs2::pipeline_profile profile = pipe.start(pipe_config);

        //声明数据流
        auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

        //获取深度像素与现实单位比例（D435默认1毫米）
        float depth_scale = get_depth_scale(profile.get_device());

        //获取内参
        auto intrinDepth=depth_stream.get_intrinsics();
        //    auto intrinColor=color_stream.get_intrinsics();

        //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
        auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);

        //    const char* depth_win="depth_Image";
        //    namedWindow(depth_win,WINDOW_AUTOSIZE);
        const char* color_win="color_Image";
        namedWindow(color_win,1);  //WINDOW_NORMAL
        while ( cvGetWindowHandle(color_win)) // Application still alive?    cvGetWindowHandle(depth_win)&&
        {
            DataReady = false;
            //堵塞程序直到新的一帧捕获
            rs2::frameset frameset = pipe.wait_for_frames();
            //取深度图和彩色图
            rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
            rs2::frame depth_frame = frameset.get_depth_frame();
            //深度图像颜色map
            //        rs2::colorizer c;                          // Helper to colorize depth images
            //        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);       //彩色深度图，用来 show
            //获取宽高
            const int depth_w=depth_frame.as<rs2::video_frame>().get_width();
            const int depth_h=depth_frame.as<rs2::video_frame>().get_height();
            const int color_w=color_frame.as<rs2::video_frame>().get_width();
            const int color_h=color_frame.as<rs2::video_frame>().get_height();

            //创建OPENCV类型 并传入数据
            Mat depth_image(Size(depth_w,depth_h),
                            CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
            //        Mat depth_image_4_show(Size(depth_w,depth_h),
            //                                CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
            Mat color_image(Size(color_w,color_h),
                            CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);  //CV_8UC3


            //实现深度图对齐到彩色图
            Mat align_CoDe=align_Depth2Color(depth_image,color_image,profile);

            Mat image_origin = color_image;
            Mat image = color_image;
            Mat canny_out;
            imageRaw = color_image;
            //        Mat image = color_image.clone();

            //        根据距离过滤背景  问题：很多区域网格
            //        float clipping_dist = 1;  // m
            //         image = remove_background(image ,align_CoDe, profile,  clipping_dist);


            //转化为hsv空间并截取某颜色区域，返回给image
            CvMat color_image_temp=color_image;
            CvMat *output;
            //                image = colorFilter(&color_image_temp, output);

            //                cvtColor(image, image, COLOR_RGB2GRAY);
            //                dilate(image, image, Mat(), Point(-1,-1));
            //                medianBlur(image, image, 9);
            //                Canny(image, canny_out, 50, 100, 3);


            //                测量距离 (区域测量平均距离)
            measure_distance(color_image,align_CoDe,cv::Size(10,10),profile, depth_w/2, depth_h/2);
            zhangaiwuD = effective_distance;
            //                ROS_INFO("指定区域距离相机距离力：d=%f", zhangaiwuD );


            //===============         获取鼠标点击点坐标
            setMouseCallback("color_Image", on_mouse, &color_image);
            if (mouse_flag == 1 )
            {
                mouse_flag = 0;
                error_num = 0;


                //平面点定义
                float pd_uv[2]={pos_x, pos_y};
                //空间点定义
                float Pdc3[3],Pcc3[3];

                //取当前点对应的深度值
                uint16_t depth_value = depth_image.at<uint16_t>(pos_y,pos_x);
                //换算到米
                float depth_m=depth_value * depth_scale;
                //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点 Pdc3
                rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
                //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下 Pcc3
                rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
                //            qDebug("  depth_scale = %f", depth_scale );
                //            qDebug("  深度摄像头坐标系下的三维点(x,y,z)=(%f, %f, %f)",Pdc3[0],Pdc3[1],Pdc3[2] );
                //            qDebug("  彩色摄像头坐标系下的三维点(x,y,z)=(%f, %f, %f)",Pcc3[0],Pcc3[1],Pcc3[2] );

                //显示在图片上
                char Txt_Point[20];
                sprintf(Txt_Point, "(%0.1f,%0.1f)", pos_x, pos_y);
                putText(color_image, Txt_Point, Point(pos_x,pos_y),	FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2, 8);
                circle(color_image, Point(pos_x,pos_y), 8, Scalar(0, 0, 255), 2, CV_AA, 0);

                char distance_str[30];
                sprintf(distance_str,"The distance is:%f m",depth_m);
                cv::putText(color_image,(string)distance_str,cv::Point(10,50),
                            cv::FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2,8);           //FONT_HERSHEY_PLAIN


                position_in_Img[0]=pos_x;
                position_in_Img[1]=pos_y;
                position_to_Camera[0]=Pcc3[0];
                position_to_Camera[1]=Pcc3[1];
                position_to_Camera[2]=Pcc3[2];
                DataReady = true;
                zhangaiwuD = Pcc3[2];
                ROS_INFO("障碍物距离 = %f m", zhangaiwuD);
            }


            imshow(color_win,color_image);

            if (exit_flag==1)
            {
                exit_flag=0;
                cv::destroyAllWindows();
            }

            int c = waitKey(100);    //XIE   刷新 ms  800
            if ((char)c == 27)   //27=esc
                exit_flag=1;

        }


    }


    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        ROS_ERROR("未获取到图像，请检查相机连接！！");
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        //    ROS_ERROR("未获取到图像，请检查相机连接！！");
        exit_flag=1;
    }


}





















// ******************************************************************************************************************************



//                                                        主函数


// ******************************************************************************************************************************
int main(int argc,char** argv)
{
    //            test();


    setlocale( LC_ALL, "" );    //让ROS显示中文
    const char *log;
    sem_init(&sem_id, 0, 1);
    ros::init(argc, argv, "xleg_lowersys_node");     //节点名称  launch里面的name
    ros::NodeHandle n;

    bool result = false;
    bool NO_dxl_port;
    //            bool NO_force_port;
    bool NO_imu_port;

    // 初始化串口-舵机
    const char* port_name = "/dev/dxl_port";   // /dev/ttyUSB0
    int baud_rate = 57600; // /dev/ttyUSB0
    uint16_t model_number = 0;

    // 串口初始化是否成功
    result = dxl_wb.init(port_name, baud_rate, &log);
    if (result == false)
    {
        ROS_ERROR("机器人舵机未连接成功！");
        NO_dxl_port = true;     //如果没连接成功
        //        return 0;
    }

    if (!NO_dxl_port)
    {
        // 按照 id 依次尝试ping舵机
        for (uint8_t dxl_id=1; dxl_id< MOTORSNUM+1; dxl_id++)
        {
            result = dxl_wb.ping(dxl_id, &model_number, &log);
            if (result == false)
            {
                ROS_ERROR("Failed to ping Motor %d", dxl_id);
                return 0;
            }
            else  ROS_INFO("Succeed to ping Motor %d", dxl_id);
        }

        // 按照id依次将舵机设置为joint模式
        for (uint8_t dxl_id=1; dxl_id< MOTORSNUM+1; dxl_id++)
        {
            int32_t velocity = 30/0.229/6;   // du/s -> rpm -> data
            int32_t acceleration = 0;
            result = dxl_wb.jointMode(dxl_id, (int32_t) velocity, acceleration, &log);    //结束后自动torqueOn

            if (result == false)
            {
                ROS_ERROR("Failed to change joint mode of Motor %d\n" ,  dxl_id);
                return 0;
            }
        }

        // 设置每个舵机， 包括转角范围，最大速度 Velocity_limit
        for (uint8_t dxl_id=1; dxl_id< MOTORSNUM+1; dxl_id++)
        {
            dxl_wb.torqueOff(dxl_id, &log);
            result = dxl_wb.itemWrite( dxl_id, "Velocity_Limit", (int32_t) (8/2.06), &log);  //1 unit=0.229rpm=2.06du/s  最大速度
            if (!result)
            {
                ROS_ERROR("Failed to set Velocity_limit of Motor %d\n" ,  dxl_id);
                return 0;
            }
            if ( dxl_id==1|| dxl_id==4 || dxl_id==7 || dxl_id==10 )   //关节1
            {
                result=dxl_wb.itemWrite( dxl_id, "Min_Position_Limit", (int32_t) (ANGLE1_MIN/0.0879), &log);
                result=dxl_wb.itemWrite( dxl_id, "Max_Position_Limit", (int32_t) (ANGLE1_MAX/0.0879), &log);
                if (result == false)
                {
                    ROS_ERROR("Failed to set JOINT_1 Angle_Limit of Motor %d\n" ,  dxl_id);
                    return 0;
                }
            }
            else if ( dxl_id==2|| dxl_id==5 || dxl_id==8 || dxl_id==11 )  //关节2
            {
                result=dxl_wb.itemWrite( dxl_id, "Min_Position_Limit", (int32_t) (ANGLE2_MIN/0.0879), &log);
                result=dxl_wb.itemWrite( dxl_id, "Max_Position_Limit", (int32_t) (ANGLE2_MAX/0.0879), &log);
                if (result == false)
                {
                    ROS_ERROR("Failed to set JOINT_2 Angle_Limit id2 of Motor %d\n" ,  dxl_id);
                    return 0;
                }
            }
            else   //关节3
            {
                result=dxl_wb.itemWrite( dxl_id, "Min_Position_Limit", (int32_t) (ANGLE3_MIN/0.0879), &log);
                result=dxl_wb.itemWrite( dxl_id, "Max_Position_Limit", (int32_t) (ANGLE3_MAX/0.0879), &log);
                if (result == false)
                {
                    ROS_ERROR("Failed to set JOINT_3 Angle_Limit id3 of Motor %d\n" ,  dxl_id);
                    return 0;
                }
            }
            dxl_wb.torqueOn(dxl_id, &log);
        }
        ....................................................
        ROS_INFO("ALL MOTORS ARE ONLINE AND SET.");
    }


    //    尝试打开力传感器接的串口
    try
    {
        ser.setPort("/dev/force_port");      // /dev/ttyUSB0
        ser.setBaudrate(19200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("机器人足底力传感器未连接成功!");
        NO_force_port = true;
        //        return 0;

    }
    if(ser.isOpen())
    {
        ROS_INFO("机器人足底力传感器已连接。");
        NO_force_port = false;

        std::thread force_thread(readForce_thread);
        force_thread.detach();
    }

    //    尝试打开imu传感器接的串口
    try
    {
        ser_imu.setPort("/dev/imu_port");    //  /dev/ttyUSB2
        ser_imu.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_imu.setTimeout(to);
        ser_imu.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("机器人IMU未连接成功!");
        NO_imu_port = true;
        //        return 0;
    }
    if(ser_imu.isOpen())
    {
        ROS_INFO("机器人IMU已连接。");
    }

    // 开启视觉线程
//    std::thread vision_thread(startVision_thread);
//    vision_thread.detach();


    // 发布service
    ros::ServiceServer srvMotor = n.advertiseService("xleg/motorCode", service_motorCode);      //仅测试用 3舵机同时控制
    ros::ServiceServer srvServo = n.advertiseService("xleg/servoControl", service_servoControl);
    ros::ServiceServer srvBody = n.advertiseService("xleg/bodyControl", service_bodyControl);
    ros::ServiceServer srvGait = n.advertiseService("xleg/gaitControl", service_gaitControl);
    ros::ServiceServer SrvDatesave = n.advertiseService("xleg/dataSave", service_dataSave);

    // 发布topic
    ros::Publisher footForce_pub = n.advertise<xleg_msgs::FootsForce>("xleg/footForce",1);
    ros::Publisher jointState_pub = n.advertise<xleg_msgs::JointsState>("xleg/jointState",1);
    ros::Publisher bodystate_pub = n.advertise<xleg_msgs::BodyState>("xleg/bodyState",1);
    xleg_msgs::FootsForce footsForce;
    xleg_msgs::JointsState jointState;
    xleg_msgs::BodyState bodySta;

    image_transport::ImageTransport it(n);  //  类似ROS句柄
    image_transport::Publisher image_pub = it.advertise("/cameraImage", 1);   // 发布话题名/cameraImage


    // 设置控制周期
    ros::Rate loop_rate((double)1000/CONTROLER_INTERVAL);
    //            ros::Rate loop_rate(1000);
    ROS_INFO("Ready to ros::ok");
    int tryrebootdxlnum = 0;
    bool tryrebootdxl = true;
    int present_temperature;  
    float present_current;

    while(ros::ok())
    {
        //舵机信息读取发布
        if (!NO_dxl_port)
        {
            //=========== 获取四腿12舵机信息，并发布
            for (int idz=0; idz<MOTORSNUM; idz++)    //4 legs; ID=1-2-3  4-5-6  7-8-9  10-11-12
            {
                int motor_hasError = 0;
                uint8_t dxl_id = idz+1;
                jointState.joints_state[idz].ID = dxl_id;

                if (gaitReadDxldataNow == false)
                {
                    int32_t get_data = 0;
                    sem_wait(&sem_id);
                    result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data, &log);   //或者用函数 getPresentPositionData()
                    sem_post(&sem_id);
                    Leg_present_angle[idz] = get_data * 0.0879;     //角度信息存储到全局变量中
                    if (result == false)
                    {
                        //                ROS_ERROR("Failed to get Present_position of Motor %d.",  dxl_id);
                        motor_hasError++;
                    }

                    sem_wait(&sem_id);
                    result = dxl_wb.itemRead(dxl_id, "Present_Current", &get_data, &log);
                    sem_post(&sem_id);
                    int16_t present_current_data =  get_data;
                    present_current = present_current_data * 3.36/1000;   //A
                    //            ROS_INFO("present_current_data = %d, Present_Current = %f", present_current_data,present_current);
                    if (result == false)
                    {
                        //                ROS_ERROR("Failed to get Present_Current of Motor %d.",  dxl_id);
                        motor_hasError++;
                    }

                    sem_wait(&sem_id);
                    result = dxl_wb.itemRead(dxl_id, "Present_Temperature", &get_data, &log);
                    sem_post(&sem_id);
                    present_temperature = get_data;
                    //           ROS_INFO("Present_Temperature = %d", present_temperature);
                    if (result == false)
                    {
                        //                ROS_ERROR("Failed to get Present_Temperature of Motor %d.",  dxl_id);
                        motor_hasError++;
                    }
                }

                jointState.joints_state[idz].present_angle = Leg_present_angle[idz];
                jointState.joints_state[idz].error = motor_hasError ;
                jointState.joints_state[idz].present_current = present_current ;
                jointState.joints_state[idz].present_temperature = present_temperature ;

                if ( (motor_hasError==3) && (tryrebootdxl==true) )
                {
                    ROS_ERROR("舵机%d似乎已掉线，正尝试Reboot该舵机...", dxl_id);
                    bool res = dxl_wb.reboot(dxl_id, &log);
                    if (res == false)
                    {
                        ROS_ERROR("Failed to reboot Motor %d.",  dxl_id);
                        tryrebootdxlnum++;
                    }
                    else
                        ROS_INFO("Succeed to reboot Motor %d.",  dxl_id);
                    dxl_wb.torqueOn(dxl_id, &log);
                }

                if( tryrebootdxlnum>3 )
                {
                    ROS_ERROR("多次掉线，不再尝试重启舵机，停机。");
                    tryrebootdxl = false;
                    stopFlag = true;  showMOVE = 0;
                    tryrebootdxlnum=0;
                }

            }

            // 根据舵机角度计算足端相对位置
            for (int fn=0; fn<4; fn++)
            {
                float *repositon;
                //按照关节角度a1, a2, a3 a1,a2,a3计算每条腿脚的相对位置x, y, z x,y,z
                repositon = jointsToPosition(Leg_present_angle[3*fn], Leg_present_angle[3*fn+1], Leg_present_angle[3*fn+2]);  //输入三关节角度(a1,a2,a3)，正解出末端坐标(x,y,z，error)，
                if (repositon[3]==0)
                {
                    jointState.foot_reposition[3*fn] = repositon[0];
                    jointState.foot_reposition[3*fn+1] = repositon[1];
                    jointState.foot_reposition[3*fn+2] = repositon[2];
                    //                ROS_INFO("foot_reposition[%d,%d,%d]=%f,%f,%f",3*fn,3*fn+1,3*fn+2, repositon[0],repositon[1],repositon[2]);
                }
                else
                    ROS_WARN("未正解出腿%d足端相对位置", fn+1);
            }

            jointState_pub.publish( jointState );

        }

        //足底力信息读取发布
        if (!NO_force_port)
        {
            //        =========  获取四腿力传感器信息+计算质心坐标，并发布。         腿编号1234（不是0 1 2 3）
            // 循环不是等间隔的，原因是该主程序循环里舵机串口读和线程舵机读冲突，互锁，等待
            // 为了等间隔获取力数据，另加线程来读，这边更新全局变量

            for(int i=0;i<4;i++)
            {
                footsForce.foots_force[i].num = i+1;
                footsForce.foots_force[i].X = Foot_present_forces[3*i];
                footsForce.foots_force[i].Y = Foot_present_forces[3*i+1];
                footsForce.foots_force[i].Z = Foot_present_forces[3*i+2];

                if((float)footsForce.foots_force[i].Z <LEGFORCEMIN)
//判断当前腿部受到的Z方向上的力是否小于预设的力阈值LEGFORCEMIN，
//如果小于，则标记该腾部未着陆，即footsForce.foots_force[i].isLanded为false；否则，表示腿部着陆，标记为true。

                    footsForce.foots_force[i].isLanded = false;
                else
                    footsForce.foots_force[i].isLanded = true;
            }
            // 计算 质心坐标 消息
            LegForceToCenterG(Foot_present_forces);
            footsForce.massCenterX = Center_rel_massxyz[0] ;     // Center_rel_massxyz[0]
            footsForce.massCenterY = Center_rel_massxyz[1] ;
            footsForce.massCenterZ = Center_rel_massxyz[2] ;
            footForce_pub.publish(footsForce);
        }

        //IMU信息读取发布
        if (!NO_imu_port)
        {
            /*
                    按出厂默认输出协议接收:
                        0x5A+0xA5+LEN_LOW+LEN_HIGH+CRC_LOW+CRC_HIGH
                        + 0x90+ID(1字节)
                        + 0xA0+Acc(加速度6字节) + 0xB0+Gyo(角速度6字节) + 0xC0+Mag(地磁6字节) + 0xD0 +AtdE(欧拉角6字节)
                        + 0xF0+Pressure(压力4字节)
                         总共 6+2+28+5=41 字节

            IMU向上发数据频率1k，如果读的速率低于1k，那么数据刷新就慢.方法：串口缓存区清空，读新的数
                        */

            //                    ROS_INFO("11");
            // 向串口读imu数据
            float Eular[3];
            uint8_t buf[41];
            ser_imu.read(buf, 1);
            if( buf[0]==0x5A )
            {
                //                        ROS_INFO("22");
                ser_imu.read(buf, 40);
                if( buf[0]==0xA5  && buf[28]==0xD0)
                {

                    //                            ROS_INFO("44");
                    Eular[0] = ((float)(int16_t)(buf[29] + (buf[30]<<8)))/100.0;
                    Eular[1] = ((float)(int16_t)(buf[31] + (buf[32]<<8)))/100.0;
                    Eular[2] = ((float)(int16_t)(buf[33] + (buf[34]<<8)))/10.0;

                    bodySta.roll = Eular[1];
                    bodySta.pitch = Eular[0];
                    bodySta.yaw = Eular[2];
                    //                    Center_rel_EXP[3] = -roll;
                    //                    Center_rel_EXP[4] = -pitch;
                    //                    Center_rel_EXP[5] = yaw;

                    //                            ROS_INFO("r,p,y=%f,%f,%f", Eular[0], Eular[1], Eular[2]);

                    bodystate_pub.publish(bodySta);
                    ser_imu.flushInput ();  //方法：串口缓存区清空，读新的数
                }
            }
        }

        //图像信息发布（另一个线程读取）
        sensor_msgs::ImagePtr  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageRaw).toImageMsg();  // 图像格式转换
        image_pub.publish(msg);         // 发布图像信息



        //-------------控制延时----------------------------------------
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}





