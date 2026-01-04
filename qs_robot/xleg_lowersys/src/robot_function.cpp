#include <math.h>
#include "robot_function.h"
#include <iostream>
#include <ros/ros.h>

float Center_rel_massxyz[3] = { 0 };


/***************************定义常量************************/

float Leg_aim_angle[12] = { 0 };
//
float Leg_Arm_Flag = 0;

float Leg_aim_position[12] = { 0 };
//float Arm_aim_position[3] = { 0 };
//float Arm_aim_positionR[9] = { 0 };
//float Arm_aim_angle[5] = { 0 };

float Leg_rel_aforce[12] = { 0 };
float Leg_rel_pforce[12] = { 0 };

char ALL_Servo_rel_error[14] = { 0 };
char ALL_Servo_rel_torque[14] = { 0 };
char ALL_Servo_rel_led[14] = { 0 };
float ALL_Servo_rel_angle[14] = { 0 };

bool result = false;

int LegForceToCenterG(double *force);


//setlocale (LC_CTYPE, "zh_CN.utf8");

//====================   控制单个舵机： 位置+运行时间  ===========================================================
/*
* dxl_id: 1-2-3   4-5-6  7-8-9  10-11-12   四条腿，每腿3个舵机
* playtime[ms]:
* 舵机角度 angle[°]  绝对量。
* 舵机速度 v[°/s] = angle增量[°]/(1000*playtime[ms])= 1000*fabs(目标angle[°]-当前角度)/playtime[ms]
*
* 舵机位置代码(0-4095) = angle[°]/0.0879
* 舵机速度代码(0-1023) = v[°/s]/6/0.229
*/
void Send_Servo_Position(char dxl_id, uint16_t playtime, float angle)     // playtime:s
//用于设置给定的舵机限制其运动到指定的角度   舵机id  运动时间  期望舵机转动的角度 

{
    int32_t get_data = 0;
    bool result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data);
    //读取当前舵机的位置
    if (result == false)//判断读取位置是否成功
    {
        ROS_WARN("Failed to get Present_Position of Motor %d.\n",  dxl_id);
        get_data = 0;
    }
    else
    {
        //计算舵机需要达到的速度
        float velocity= 1000.0*fabs(angle - get_data*0.0879) /playtime  ;     //  °/s->data
        result=dxl_wb.itemWrite( dxl_id, "Profile_Velocity", (int32_t) (velocity/0.229/6) );
//计算出来的速度写入到指定的舵机ID

//让舵机以给定的速度平滑地转动到目标角度
        if (result == false)
        {
            ROS_WARN("Failed to set Profile_Velocity of Motor %d.\n" ,  dxl_id);
        }
        else
        {
            dxl_wb.goalPosition(dxl_id, (int) (angle/0.0879) );
        }
    }
}
//====================   控制单个舵机： 位置+运行速度  ===========================================================
/*
* dxl_id: 1-2-3   4-5-6  7-8-9  10-11-12   四条腿，每腿3个舵机
* 输入：v[°/s]，angle[°]
* 舵机位置代码(0-4095) = angle[°]/0.0879
* 舵机速度代码(0-1023) = v[°/s]/6/0.229
*/
void Send_Servo_Position_speed(char dxl_id, float speed, float angle)
//期望运动的速度,舵机期望停止的角度
{
    bool result=dxl_wb.itemWrite( dxl_id, "Profile_Velocity", (int32_t) (speed/0.229/6));
   // 对舵机速度的设置是否成功
    if (result == false)
    {
        ROS_WARN("Failed to set Profile_Velocity of Motor %d.\n" ,  dxl_id);
    }
    else
    {
        dxl_wb.goalPosition(dxl_id, (int) (angle/0.0879) );//转换过的目标角度值写入到对应舵机的目标位置
    }
}




//====================   发送单条腿3个关节舵机角度  =========================================================================
/* 输入： legID(1 或 2,3,4)  , playtime 运行时间，
 *       关节角度 Leg_aim_angle[12]，由 Set_One_Leg_Positon(char legID, float leg[3]) 函数求解。
*/
void Send_One_Leg_Positon(char legID, uint16_t playtime)
{
//     Send_Servo_Position(( (legID-1)*3+1 ),  playtime, Leg_aim_angle[(legID * 3 - 3)]);   //腰关节
//     Send_Servo_Position(( (legID-1)*3+2 ),  playtime, Leg_aim_angle[(legID * 3 - 2)]);   //髋关节
//     Send_Servo_Position(( (legID-1)*3+3 ),  playtime, Leg_aim_angle[(legID * 3 - 1)]);   //膝关节

     //为了提高同步性，将要解算的要设置的统一设置完成，再统一下达运动指令， 目标角度 Leg_aim_angle[12]
     int32_t get_data = 0;
     char dxl_id = 0;
     bool result = false;
     int errornum = 0;
     float velocity = 1000*30/playtime;
     for(int i=0; i<3; i++)
    {
         dxl_id = (legID-1)*3 + i+1;

         result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data);
         if (result == false)
         {
//             ROS_WARN("Failed to get Present_Position of Motor %d, use Leg_present_angle[%d] value.\n",  dxl_id, dxl_id-1);
             get_data = (int32_t) (Leg_present_angle[dxl_id-1]/0.089);
             errornum++;
         }

         Leg_present_angle[dxl_id-1] = get_data*0.0879;  //更新全局变量
         velocity = 1000.0*fabs(Leg_aim_angle[dxl_id-1] - Leg_present_angle[dxl_id-1]) /playtime  ;     //  °/s->data
         result=dxl_wb.itemWrite( dxl_id, "Profile_Velocity", (int32_t) (velocity/0.229/6) );
         if (result == false)
         {
//             ROS_WARN("Failed to set Profile_Velocity of Motor %d.\n" ,  dxl_id);
             errornum++;
         }

         //用当前角度的全局变量来计算速度，可能更新不及时
//         ROS_INFO("Leg_present_angle[%d] = %f",dxl_id-1, Leg_present_angle[dxl_id-1]);
//         velocity = 1000.0*fabs(Leg_aim_angle[dxl_id-1] - Leg_present_angle[dxl_id-1]) /playtime  ;     //  °/s->data
//         result=dxl_wb.itemWrite( dxl_id, "Profile_Velocity", (int32_t) (velocity/0.229/6) );
//         if (result == false)   //false
//         {
//             ROS_WARN("Failed to set Profile_Velocity of Motor %d.\n" ,  dxl_id);
//             errornum++;
//         }

     }

     if (errornum == 0)
     {
         for(int i=0; i<3; i++)
         {
             dxl_id = (legID-1)*3 + i+1;
             dxl_wb.goalPosition(dxl_id, (int) ( Leg_aim_angle[dxl_id-1] /0.0879) );
         }
         ROS_INFO("已依次下达腿%d所有舵机控制指令。", legID );
     }


}

//========================   发送所有四足舵机角度  ==============================================================================
/* 输入： playtime 运行时间 ms， 目标角度 Leg_aim_angle[12]
 * 当前舵机角度（全局变量） Leg_present_angle[12]
*/
void Send_ALL_Leg_Positon(uint16_t playtime)
{
//    Send_One_Leg_Positon(1, playtime);
//    Send_One_Leg_Positon(2, playtime);
//    Send_One_Leg_Positon(3, playtime);
//    Send_One_Leg_Positon(4, playtime);

    //为了提高同步性，将要解算的要设置的统一设置完成，再统一下达运动指令， 目标角度 Leg_aim_angle[12]
    //求解12舵机应运行速度 vv[12] (单位：°/s）
    int32_t get_data = 0;
    char dxl_id = 0;
    bool result = false;
    int errornum = 0;
    float velocity = 1000*30/playtime;
    for(int i=0; i<12; i++)
    {
        dxl_id = i+1;

        result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data);
        if (result == false)
        {
//            ROS_WARN("Failed to get Present_Position of Motor %d, use Leg_present_angle[%d] value.\n",  dxl_id, dxl_id-1);
            get_data = (int32_t) (Leg_present_angle[dxl_id-1]/0.089);
//            errornum++;
        }
        Leg_present_angle[dxl_id-1] = get_data*0.0879;  //更新全局变量

        velocity = 1000.0*fabs(Leg_aim_angle[dxl_id-1] - get_data*0.0879) /playtime  ;     //  °/s->data
        result=dxl_wb.itemWrite( dxl_id, "Profile_Velocity", (int32_t) (velocity/0.229/6) );
        if (result == false)   //false
        {
//            ROS_WARN("Failed to set Profile_Velocity of Motor %d.\n" ,  dxl_id);
            errornum++;
        }

        //用当前角度的全局变量来计算速度，可能更新不及时
//        ROS_INFO("Leg_present_angle[%d] = %f",i, Leg_present_angle[i]);
//        velocity = 1000.0*fabs(Leg_aim_angle[i] - Leg_present_angle[i]) /playtime  ;     //  °/s->data
//        result=dxl_wb.itemWrite( dxl_id, "Profile_Velocity", (int32_t) (velocity/0.229/6) );
//        if (result == false)   //false
//        {
//            ROS_WARN("Failed to set Profile_Velocity of Motor %d.\n" ,  dxl_id);
//            errornum++;
//        }

    }

    if (errornum == 0)
    {
        for(int i=0; i<12; i++)
        {
            dxl_id = i+1;
            dxl_wb.goalPosition(dxl_id, (int) (Leg_aim_angle[i]/0.0879) );
        }
        ROS_INFO("已依次下达所有12舵机控制指令。");
    }
}



//====================   设置四足位置(包含检测） =============================================================================
// 输入为每条腿末端相对三维坐标xyz
//
int Set_ALL_Leg_Positon(float leg1[3], float leg2[3], float leg3[3], float leg4[3])
{
    int leg_error = 0;
    leg_error = Set_One_Leg_Positon(4, leg4);
    leg_error = leg_error*2 + Set_One_Leg_Positon(3, leg3);
    leg_error = leg_error*2 + Set_One_Leg_Positon(2, leg2);
    leg_error = leg_error*2 + Set_One_Leg_Positon(1, leg1);
    return leg_error;
}

//====================    设置单足位置(包含检测）=============================================================================
/*
 * 输入：legID(1或2,3,4)  和 腿末端（足端）相对于腿起点的坐标(x,y,z) mm
 * 输出：该腿三个关节所需角度(全局变量) Leg_aim_angle[3]
 * 返回：error 成功 0,  错误 1
*/
int Set_One_Leg_Positon(char legID, float leg[3])
{
    if (legID==2 || legID==3 )   leg[1] = -leg[1];  //2,3腿-y改为+y

    float *joints;
    joints = positionToJoints(leg[0], leg[1], leg[2]);       //返回数组 （q1 q2 q3 error）
    if (joints[3]==0)
    {
//         free(joints);   //释放，防止内存泄漏

        if (legID==2 || legID==3 )   joints[0] = 360 - joints[0];  //y和-y方向反着

//        ROS_INFO("Set_Leg%d_Positon: joints = %f, %f, %f",legID, joints[0],joints[1],joints[2]);
        Leg_aim_position[(legID * 3 - 3)] = leg[0];
        Leg_aim_position[(legID * 3 - 2)] = leg[1];
        Leg_aim_position[(legID * 3 - 1)] = leg[2];
        Leg_aim_angle[(legID * 3 - 3)] = joints[0];
        Leg_aim_angle[(legID * 3 - 2)] = joints[1];
        Leg_aim_angle[(legID * 3 - 1)] = joints[2];

        if (legID==2 || legID==3 )   leg[1] = -leg[1];  //2,3腿-y改为+y  再改回去
        return 0;
    }
    else
        return 1;
}





//====================   中心位姿求出四足的末端相对位置  ===========================================
/* 零点坐标在本体中心
 * 输入：v[3] 机器人期望中心位置(x,y相对上一时刻)（mm）， w[3] 机器人期望中心姿态度（度）
 *      P_foot 当前四条腿末端的绝对三维坐标 （相对于原点。 本体中心(0,0,H)在地面的投影位置为原点 ）
 * 输出：p1[3]~p4[4] 四条腿末端的相对三维坐标(x,y,z)mm （腿末端相对于起始端位置坐标）
*/
int CCenterToLeg(float v[3], float w[3], float P_foot1[3], float P_foot2[3], float P_foot3[3], float P_foot4[3], float p1[3], float p2[3], float p3[3], float p4[3])
{

//    ROS_INFO("p_foot1 = %f,%f,%f",P_foot1[0], P_foot1[1],P_foot1[2]);
//    ROS_INFO("p_foot2 = %f,%f,%f",P_foot2[0], P_foot2[1],P_foot2[2]);
//    ROS_INFO("p_foot3 = %f,%f,%f",P_foot3[0], P_foot3[1],P_foot3[2]);
//    ROS_INFO("p_foot4 = %f,%f,%f",P_foot4[0], P_foot4[1],P_foot4[2]);

//    ROS_INFO("p1[x,y,z] = %f,%f,%f",p1[0], p1[1],p1[2]);
//    ROS_INFO("p2[x,y,z] = %f,%f,%f",p2[0], p2[1],p2[2]);
//    ROS_INFO("p3[x,y,z] = %f,%f,%f",p3[0], p3[1],p3[2]);
//    ROS_INFO("p4[x,y,z] = %f,%f,%f",p4[0], p4[1],p4[2]);


    w[0] = w[0] / 180.0 * M_PI;  //转为弧度
    w[1] = w[1] / 180.0 * M_PI;
    w[2] = w[2] / 180.0 * M_PI;
    //初始四肢(四肢起点)绝对位置，以为
    float P1_leg1[3] = {  CENTER_L/2,   CENTER_B/2,  v[2] };
    float P1_leg2[3] = {  CENTER_L/2,  -CENTER_B/2,  v[2] };
    float P1_leg3[3] = { -CENTER_L/2,  -CENTER_B/2,  v[2] };
    float P1_leg4[3] = { -CENTER_L/2,   CENTER_B/2,  v[2] };
    //求解变换矩阵
    float T_Center[3] = {v[0], v[1], 0};  //{v[0],v[1],v[2]};
    float R_Center[3][3];

    float rx[3][3] = {1, 0, 0,     0, (float) cos( w[0]), (float) -sin( w[0]),     0, (float) sin( w[0]), (float) cos( w[0])};
    float ry[3][3] = {(float) cos( w[1]), 0, (float) sin( w[1]),      0, 1, 0,   (float) -sin( w[1]), 0,  (float) cos( w[1])};
    float rz[3][3] = {(float) cos( w[2]),(float) -sin( w[2]),0,    (float) sin( w[2]), (float) cos( w[2]),0,    0, 0,1 };

    //        R_Center= rz*ry*rx;
    float **tmp1, tmp2[3][3], **tmp3;
    tmp1 = multiply_twoMats(rz,ry);
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            tmp2[i][j] = tmp1[i][j];

    tmp3 = multiply_twoMats(tmp2,rx);
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            R_Center[i][j] = tmp3[i][j];

    //变换后四肢(四肢起点)绝对位置
    float P2_leg1[3], P2_leg2[3], P2_leg3[3], P2_leg4[3];
    for (int k=0; k<3; k++)
    {
        P2_leg1[k] = R_Center[k][0]*P1_leg1[0] + R_Center[k][1]*P1_leg1[1]+ R_Center[k][2]*P1_leg1[2];
        P2_leg2[k] = R_Center[k][0]*P1_leg2[0] + R_Center[k][1]*P1_leg2[1]+ R_Center[k][2]*P1_leg2[2];
        P2_leg3[k] = R_Center[k][0]*P1_leg3[0] + R_Center[k][1]*P1_leg3[1]+ R_Center[k][2]*P1_leg3[2];
        P2_leg4[k] = R_Center[k][0]*P1_leg4[0] + R_Center[k][1]*P1_leg4[1]+ R_Center[k][2]*P1_leg4[2];
    }


//    ROS_INFO("P2_leg1 = %f,%f,%f",P2_leg1[0], P2_leg1[1],P2_leg1[2]);
//    ROS_INFO("P2_leg2 = %f,%f,%f",P2_leg2[0], P2_leg2[1],P2_leg2[2]);
//    ROS_INFO("P2_leg3 = %f,%f,%f",P2_leg3[0], P2_leg3[1],P2_leg3[2]);
//    ROS_INFO("P2_leg4 = %f,%f,%f",P2_leg4[0], P2_leg4[1],P2_leg4[2]);

    //变换后四足相对四腿起端的相对位置，再平移
    for (int k=0; k<3; k++)
    {
        p1[k] = P_foot1[k] - P2_leg1[k] - T_Center[k];
        p2[k] = P_foot2[k] - P2_leg2[k] - T_Center[k];
        p3[k] = P_foot3[k] - P2_leg3[k] - T_Center[k];
        p4[k] = P_foot4[k] - P2_leg4[k] - T_Center[k];

        P_foot1[k]=P_foot1[k]-T_Center[k];   //腿末端的绝对三维坐标（相对于原点。 本体中心(0,0,H)在地面的投影位置为原点 ），机身本体转动不影响
        P_foot2[k]=P_foot2[k]-T_Center[k];
        P_foot3[k]=P_foot3[k]-T_Center[k];
        P_foot4[k]=P_foot4[k]-T_Center[k];
    }

//    ROS_INFO("T_Center[x,y,z] = %f,%f,%f",T_Center[0], T_Center[1],T_Center[2]);

//    ROS_INFO("p1[x,y,z] = %f,%f,%f",p1[0], p1[1],p1[2]);
//    ROS_INFO("p2[x,y,z] = %f,%f,%f",p2[0], p2[1],p2[2]);
//    ROS_INFO("p3[x,y,z] = %f,%f,%f",p3[0], p3[1],p3[2]);
//    ROS_INFO("p4[x,y,z] = %f,%f,%f",p4[0], p4[1],p4[2]);

//    ROS_INFO("p_foot1 = %f,%f,%f",P_foot1[0], P_foot1[1],P_foot1[2]);
//    ROS_INFO("p_foot2 = %f,%f,%f",P_foot2[0], P_foot2[1],P_foot2[2]);
//    ROS_INFO("p_foot3 = %f,%f,%f",P_foot3[0], P_foot3[1],P_foot3[2]);
//    ROS_INFO("p_foot4 = %f,%f,%f",P_foot4[0], P_foot4[1],P_foot4[2]);


    return RIGHT;
}


//====================   根据足端力计算重心数据  ===========================================
//必须进行Set_One_Leg_Positon未报错,足末端被压为z正，受到向前为x正
/* 输入： 四足底力数组 force[12],  单位N， xyz
 * 输出： 机器人当前质心
*/
int LegForceToCenterG(float *force)
{
    Center_rel_massxyz[0]=1;
    Center_rel_massxyz[1]=3;
    Center_rel_massxyz[2]=4;

    return RIGHT;
}


//====================   单腿走一次：抬腿-前进-落腿  ===========================================
/* 输入： 腿ID， 当前足端相对位置leg[3]=（x,y,z）mm，步长(沿xy分别dx，dy)mm， 抬脚高度dzmm， 时间ms
 * 则目标足端相对位置，x+stepDistance
 * 返回error：成功0, 错误1
*/
int  Make_One_Leg_Runonce(char legID, float leg[3], int dx, int dy, int dz, uint16_t playtime)
{
    ROS_INFO("Make_One_Leg_Runonce (before): leg%d[3]=%f, %f, %f", legID, leg[0], leg[1], leg[2]);
    leg[2] = leg[2] + dz; //抬腿
    int errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 抬 解算出错。",legID);
        return 1;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.33*playtime));
    usleep( floor(playtime*0.33*1000) );

    leg[0] = leg[0] + dx; //进
    leg[1] = leg[1] + dy;
    errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 进 解算出错。",legID);
        return 1;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.33*playtime));
    usleep( floor(playtime*0.33*1000) );

    leg[2] = leg[2] - dz; //落
    errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 落 解算出错。",legID);
        return 1;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.33*playtime));
    usleep( floor(playtime*0.35*1000) );
    ROS_INFO("Make_One_Leg_Runonce (after): leg%d[3]=%f, %f, %f", legID, leg[0], leg[1], leg[2]);

    return 0;
}



//====================   单腿走一次：抬腿-前进-落腿（落腿高度与抬腿高度不同）  ===========================================
/* 输入： 腿ID， 当前足端相对位置leg[3]=（x,y,z）mm，步长(沿xy分别dx，dy)mm， 抬脚高度dzmm，落dz2， 时间ms
 * 则目标足端相对位置，x+stepDistance
 * 返回error：成功0, 错误1
*/
int  Make_One_Leg_Runonce_dz(char legID, float leg[3], int dx, int dy, int dz, int dz2, uint16_t playtime)
{
    ROS_INFO("Make_One_Leg_Runonce (before): leg%d[3]=%f, %f, %f", legID, leg[0], leg[1], leg[2]);
    leg[2] = leg[2] + dz; //抬腿
    int errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 抬 解算出错。",legID);
        return 1;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.33*playtime));
    usleep( floor(playtime*0.33*1000) );

    leg[0] = leg[0] + dx; //进
    leg[1] = leg[1] + dy;
    errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 进 解算出错。",legID);
        return 1;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.33*playtime));
    usleep( floor(playtime*0.33*1000) );

    leg[2] = leg[2] - dz2; //落
    errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 落 解算出错。",legID);
        return 1;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.33*playtime));
    usleep( floor(playtime*0.35*1000) );
    ROS_INFO("Make_One_Leg_Runonce (after): leg%d[3]=%f, %f, %f", legID, leg[0], leg[1], leg[2]);

    return 0;
}





//====================   单腿走一次：抬腿-前进-落腿 - 单腿越障模式，检索到足底力停止足下落 =================== 
/* 输入： 腿ID， 当前足端相对位置leg[3]=（x,y,z）mm，步长(沿xy分别dx，dy)mm， 抬脚高度dzmm， 时间ms
 * 则目标足端相对位置，x+stepDistance
 * 返回数组[q1 q2 q3 hz error]：当前腿三关节舵机角度q1,q2,q3，足相对腿起点的z坐标hz（用于求解障碍高度）,error
 * error=0 成功；=1 舵机角度失败，程序终止; =2 到时间了，程序继续，还可以继续行走
*/
float* Make_One_Leg_Runonce_withForce(char legID, float leg[3], int dx, int dy, int dz, uint16_t playtime)
{
    // dz=( 120 - abs(leg[2]));

    float *res;
    res = (float *)malloc(5);
    res[0]= 0;    res[1]= 0;    res[2]= 0;
    res[3]= 0;    res[4]= 1;
//    return res;

    ROS_INFO("Make_One_Leg_Runonce_withForce (before): leg%d[3]=%f, %f, %f", legID, leg[0], leg[1], leg[2]);
    leg[2] = leg[2] + dz; //抬腿
    int errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 抬 解算出错。",legID);
        return res;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.33*playtime));
    usleep( floor(playtime*0.33*1000) );

    leg[0] = leg[0] + dx; //进
    leg[1] = leg[1] + dy;
    errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 进 解算出错。",legID);
        return res;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.25*playtime));
    usleep( floor(playtime*0.33*1000) );

    leg[2] = leg[2] - dz; //落   多下落些，比如有坑    leg[2] - dz
    errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 落 解算出错。",legID);
        return res;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (0.42*playtime));
//    usleep( floor(playtime*0.35*1000) );

    int pastcount = 0;   int32_t get_data = 0;

    res[4]= 2;
    while( pastcount< (uint16_t) (0.37*playtime / 1.2 ) )
    {
        if (Foot_present_forces[ legID*3-1 ]>0.5)  //leg 1 2 3 4  <-> FZ 2 5  8  11
        {
            ROS_WARN("Detect impact!  Foot_present_forces[%d]=%f", legID, Foot_present_forces[legID*3-1]);
            res[4] = 0;
            for (int i = 0; i < 3; i++)
            {
                int dxl_id = (legID-1)*3+i+1;
                bool result = dxl_wb.itemRead( dxl_id, "Present_Position", &get_data);
                if (result == false)
                {
                    ROS_WARN("Failed to get Present_Position of Motor %d, use Leg_present_angle[%d] value.\n",  dxl_id, dxl_id-1);
                    get_data = (int32_t) (Leg_present_angle[dxl_id-1]/0.089);
                    //            errornum++;
                }
                dxl_wb.goalPosition(dxl_id, get_data );     //重新下达指令，间接让舵机停止运动。
                res[i] = get_data*0.0879;
                Leg_present_angle[dxl_id-1] = get_data*0.0879;  //更新全局变量
            }

            float *repositon;
            repositon = jointsToPosition(res[0], res[1], res[2]);  //输入三关节角度(a1,a2,a3)，正解出末端坐标(x,y,z，error)，
            if (repositon[3]==0)
            {
                res[3] = repositon[2];
            }
            else
            {
                ROS_WARN("未正解出腿%d足端相对位置", legID);
                res[4] = 1;
            }
            break;
        }

        usleep(1.5*1000);  //2ms/1xunhuan
//        ROS_INFO("pastcount=%d",pastcount);
        pastcount++;
    }

    ROS_INFO("Make_One_Leg_Runonce_withForce (after): leg%d[3]=%f, %f, %f", legID, leg[0], leg[1], leg[2]-res[3]);
    return res;
}



//====================   2腿tong shi走一次：抬腿-前进-落腿  ===========================================
/* 输入： 腿ID， 当前足端相对位置leg[3]=（x,y,z）mm，步长(沿xy分别dx，dy)mm， 抬脚高度dzmm， 时间ms
 * 则目标足端相对位置，x+stepDistance
 * 返回error：成功0, 错误1
*/
int Make_Two_Legs_Runonce(char legID1, char legID2, float leg1[3], float leg2[3], int dx1, int dy1, int dz1,  int dx2, int dy2, int dz2, uint16_t playtime)
{
    ROS_INFO("Make_Two_Legs_Runonce (before): leg%d[3]=%f, %f, %f;  leg%d[3]=%f, %f, %f", legID1, leg1[0], leg1[1], leg1[2], legID2, leg2[0], leg2[1], leg2[2]);

    leg1[2] = leg1[2] + dz1; //抬腿
    leg2[2] = leg2[2] + dz2;
    int errorback = Set_One_Leg_Positon(legID1, leg1);
    errorback = errorback + Set_One_Leg_Positon(legID2, leg2);
    if (errorback > 0 )
    {
        ROS_ERROR("腿 抬 解算出错。");
        return 1;
    }
    else
    {
        Send_One_Leg_Positon(legID1, (uint16_t) (0.33*playtime));
        Send_One_Leg_Positon(legID2, (uint16_t) (0.33*playtime));
    }
    usleep( floor(playtime*0.33*1000) );

    leg1[0] = leg1[0] + dx1;  leg1[1] = leg1[1] + dy1;      //进
    leg2[0] = leg2[0] + dx2;  leg2[1] = leg2[1] + dy2;
    errorback = Set_One_Leg_Positon(legID1, leg1);
    errorback = errorback + Set_One_Leg_Positon(legID2, leg2);
    if (errorback > 0  )
    {
        ROS_ERROR("腿 进 解算出错。");
        return 1;
    }
    else
    {
        Send_One_Leg_Positon(legID1, (uint16_t) (0.33*playtime));
        Send_One_Leg_Positon(legID2, (uint16_t) (0.33*playtime));
    }
    usleep( floor(playtime*0.33*1000) );

    leg1[2] = leg1[2] - dz1; //落
    leg2[2] = leg2[2] - dz2;
    errorback = Set_One_Leg_Positon(legID1, leg1);
    errorback = errorback + Set_One_Leg_Positon(legID2, leg2);
    if (errorback > 0 )
    {
        ROS_ERROR("腿 落 解算出错。");
        return 1;
    }
    else
    {
        Send_One_Leg_Positon(legID1, (uint16_t) (0.4*playtime));
        Send_One_Leg_Positon(legID2, (uint16_t) (0.4*playtime));
    }
    usleep( floor(playtime*0.4*1000) );

    ROS_INFO("Make_Two_Legs_Runonce (after): leg%d[3]=%f, %f, %f;  leg%d[3]=%f, %f, %f", legID1, leg1[0], leg1[1], leg1[2], legID2, leg2[0], leg2[1], leg2[2]);
    return 0;
}








//====================   单腿到达位置(目标位置)：抬腿-落腿  ===========================================
/* 输入： 腿ID， 目标足端相对位置leg[3]=（x,y,z）mm，抬脚高度dzmm， 时间ms
 * 返回error：成功0, 错误1
*/
int Make_One_Leg_toPositon(char legID, float leg[3], int dz, uint16_t playtime)
{
    leg[0] = leg[0];
    leg[1] = leg[1];
    leg[2] = leg[2] + dz; //抬腿

    int errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 抬解算出错。",legID);
        return 1;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (playtime*0.5));

    usleep( floor(playtime*0.5*1000) );

    leg[2] = leg[2] - dz;
    errorback = Set_One_Leg_Positon(legID, leg);
    if (errorback)
    {
        ROS_ERROR("腿%d 落解算出错。",legID);
        return 1;
    }
    else
        Send_One_Leg_Positon(legID, (uint16_t) (playtime*0.5));

    usleep( floor(playtime*0.55*1000) );
    return 0;
}


//====================   考虑重力（有重心调整）下的单步周期：四腿  ===========================================
/* 输入：vw_foot_abcd[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]]; 步长(沿xy分别dx，dy)mm， 抬脚高度dzmm，重心调整 单步周期ms
 * 则目标足端相对位置，x+stepDistance
 * 返回失败代码：1失败， 0成功
 * 更新数组（30个数）backdata[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]
*/
int Make_Runonce_withGravity( float vw_foot_abcd[30],  int dx, int dy, int dz, int bodyadjX, int bodyadjY, uint16_t playtime, float vwfa_backdata[30] )
{
    ROS_WARN("进入单步行走子程序。");


    float g_v[3], g_w[3], g_foot1[3], g_foot2[3], g_foot3[3], g_foot4[3], a[3], b[3], c[3], d[3];
    for (int i=0; i<3; i++)
    {
        g_v[i] = vw_foot_abcd[i];     g_w[i] = vw_foot_abcd[i+3];

        g_foot1[i] = vw_foot_abcd[i+6];
        g_foot2[i] = vw_foot_abcd[i+9];
        g_foot3[i] = vw_foot_abcd[i+12];
        g_foot4[i] = vw_foot_abcd[i+15];

        a[i] = vw_foot_abcd[i+18];
        b[i] = vw_foot_abcd[i+21];
        c[i] = vw_foot_abcd[i+24];
        d[i] = vw_foot_abcd[i+27];
    }


    ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
             g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);

    int error_back=0;
    int stepD_tmp = 0;
    int stepHeight = 0;
    //前进1-3-2-4
    if ( (dy==0) && (dx>0) )
    {
        ROS_INFO("前进");
       stepD_tmp = dx;
       stepHeight = dz;
        //1-3-2-4
        //腿1进,调整重心
        ROS_INFO("1.body+leg1");
        g_v[0] = -bodyadjX;   //相对前一时刻，相对值
        g_v[1] = -bodyadjY;   //-Y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
            usleep( 0.4*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }


        g_foot1[0]=g_foot1[0]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //腿3进,先调整重心
        ROS_INFO("2.body+leg3");
        g_v[0] = bodyadjX + stepD_tmp/2;
        g_v[1] = 2*bodyadjY;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
            usleep( 0.8*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(3, c, stepD_tmp, 0, stepHeight, playtime);
        g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }


        //腿2进
        ROS_INFO("3.body+leg2");
        error_back = Make_One_Leg_Runonce(2, b, stepD_tmp, 0, stepHeight, playtime);
        g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //腿4进,调整重心
        ROS_INFO("4.body+leg4");
        g_v[0] =  +bodyadjX + stepD_tmp/2;   //相对前一时刻，相对值
        g_v[1] = -2*bodyadjY;   //-y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
            usleep( 0.8*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(4, d, stepD_tmp, 0, stepHeight, playtime);
        g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //5,调整重心,回归初始姿态
        ROS_INFO("5.body return zero");
        g_v[0] = -bodyadjX;   //相对前一时刻，相对值
        g_v[1] = bodyadjY;   //y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
            usleep( 0.35*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }
        //以上步骤结束后，四条腿回归原位置

    }
    else if ( (dy==0) && (dx<0) )  //后退
    {
        ROS_INFO("后退");
       stepD_tmp = -dx;
       stepHeight = dz;

        //腿4后退,调整重心
       ROS_INFO("1.body+leg4");
        g_v[0] = +bodyadjX;   //相对前一时刻，相对值
        g_v[1] = -bodyadjY;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
            usleep( 0.4*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }
        error_back = Make_One_Leg_Runonce(4, d, -stepD_tmp, 0, stepHeight, playtime);
        g_foot4[0]=g_foot4[0]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //腿2后退,先调整重心
        ROS_INFO("2.body+leg2");
        g_v[0] = -bodyadjX - stepD_tmp/2;
        g_v[1] = +2*bodyadjY;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
            usleep( 0.8*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(2, b, -stepD_tmp, 0, stepHeight, playtime);
        g_foot2[0]=g_foot2[0]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }


        //腿3后退
        ROS_INFO("3.body+leg3");
        error_back = Make_One_Leg_Runonce(3, c, -stepD_tmp, 0, stepHeight, playtime);
        g_foot3[0]=g_foot3[0]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }


        //腿1后退,调整重心
        ROS_INFO("4.body+leg1");
        g_v[0] =  -bodyadjX - stepD_tmp/2;   //相对前一时刻，相对值
        g_v[1] = -2*bodyadjY;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
            usleep( 0.8*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(1, a, -stepD_tmp, 0, stepHeight, playtime);
        g_foot1[0]=g_foot1[0]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }


        //5,调整重心,回归初始姿态
        ROS_INFO("5.body return zero");
        g_v[0] = +bodyadjX;   //相对前一时刻，相对值
        g_v[1] = +bodyadjY;   //y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，

        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
            usleep( 0.35*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }
        //以上步骤结束后，四条腿回归原位置

    }

    else if ( (dy>0) && (dx==0) )  //左移动
    {
        ROS_INFO("左移");
        stepD_tmp = dy;
        stepHeight = dz;

        //1-4-3-2
        //腿1进,调整重心
        ROS_INFO("1.body+leg1");
        g_v[0] = -bodyadjX;   //相对前一时刻，相对值
        g_v[1] = -bodyadjY;   //-Y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.3*playtime) );
            usleep( 0.3*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(1, a, 0, stepD_tmp, stepHeight, playtime);
        g_foot1[1]=g_foot1[1]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //腿4进,调整重心
        ROS_INFO("2.body+leg4");
        g_v[0] = 2*bodyadjX;   //相对前一时刻，相对值
        g_v[1] = 0;   //-Y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.5*playtime) );
            usleep( 0.5*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(4, d, 0, stepD_tmp, stepHeight, playtime);
        g_foot4[1]=g_foot4[1]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }


        //body move + 腿3进,调整重心
        ROS_INFO("3+4.body+leg3");
        g_v[0] = 0;   //相对前一时刻，相对值
        g_v[1] = stepD_tmp + 2*bodyadjY;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (1*playtime) );
            usleep( 1*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(3, c, 0, stepD_tmp, stepHeight, playtime);
        g_foot3[1]=g_foot3[1]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //腿2进,调整重心
        ROS_INFO("5.body+leg2");
        g_v[0] = -2*bodyadjX;   //相对前一时刻，相对值
        g_v[1] = 0;   //-Y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.5*playtime) );
            usleep( 0.5*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(2, b, 0, stepD_tmp, stepHeight, playtime);
        g_foot2[1]=g_foot2[1]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //5,调整重心,回归初始姿态
        ROS_INFO("6.body return zero");
        g_v[0] = bodyadjX;   //相对前一时刻，相对值
        g_v[1] = -bodyadjY;   //y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，

        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.3*playtime) );
            usleep( 0.25*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        //以上步骤结束后，四条腿回归原位置


    }

    else if ( (dy<0) && (dx==0) )  //右平移
    {
        ROS_INFO("右移");
        stepD_tmp = -dy;
        stepHeight = dz;

        //2-3-4-1
        //腿2进,调整重心
        ROS_INFO("1.body+leg2");
        g_v[0] = -bodyadjX;   //相对前一时刻，相对值
        g_v[1] = +bodyadjY;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.3*playtime) );
            usleep( 0.3*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(2, b, 0, -stepD_tmp, stepHeight, playtime);
        g_foot2[1]=g_foot2[1]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //腿3进,调整重心
        ROS_INFO("2.body+leg3");
        g_v[0] = +2*bodyadjX;   //相对前一时刻，相对值
        g_v[1] = 0;   //-Y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.5*playtime) );
            usleep( 0.5*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(3, c, 0, -stepD_tmp, stepHeight, playtime);
        g_foot3[1]=g_foot3[1]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }


        //body move + 腿4进,调整重心
        ROS_INFO("3+4.body+leg4");
        g_v[0] = 0;   //相对前一时刻，相对值
        g_v[1] = -stepD_tmp - 2*bodyadjY;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (1*playtime) );
            usleep( 1*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(4, d, 0, -stepD_tmp, stepHeight, playtime);
        g_foot4[1]=g_foot4[1]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //腿1进,调整重心
        ROS_INFO("5.body+leg1");
        g_v[0] = -2*bodyadjX;   //相对前一时刻，相对值
        g_v[1] = 0;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.5*playtime) );
            usleep( 0.5*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

        error_back = Make_One_Leg_Runonce(1, a, 0, -stepD_tmp, stepHeight, playtime);
        g_foot1[1]=g_foot1[1]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back == 1");
            return error_back;
        }

        //5,调整重心,回归初始姿态
        ROS_INFO("6.body return zero");
        g_v[0] = +bodyadjX;   //相对前一时刻，相对值
        g_v[1] = +bodyadjY;   //y
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，

        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.3*playtime) );
            usleep( 0.25*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;
        }

       //以上步骤结束后，四条腿回归原位置


    }



    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);

    for (int i=0;i<3;i++)   //更新vwfa_backdata， 其实不需要更新，保证前后腿回归原位就行
    {
        vwfa_backdata[i] = g_v[i];
        vwfa_backdata[i+3] = g_w[i];
        vwfa_backdata[i+6] = g_foot1[i];
        vwfa_backdata[i+9] = g_foot2[i];
        vwfa_backdata[i+12] = g_foot3[i];
        vwfa_backdata[i+15] = g_foot4[i];
        vwfa_backdata[i+18] = a[i];
        vwfa_backdata[i+21] = b[i];
        vwfa_backdata[i+24] = c[i];
        vwfa_backdata[i+27] = d[i];
    }

}





//====================   考虑重力（有重心调整）下的跨上台阶面  ===========================================
/* 输入：vw_foot_abcd[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]]; 距台阶面ssf_D， 台阶高度ssf_H，抬脚高度dz， 单步周期ms
 * 则目标足端相对位置，x+stepDistance
 * 返回失败代码：1失败， 0成功
 * 更新数组（30个数）backdata[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]
*/
int Climb_stepsurface_withGravity( float vw_foot_abcd[30],  int ssf_D, int ssf_H, int dz, int bodyadjX, int bodyadjY, uint16_t playtime, float vwfa_backdata[30] )
{
    ROS_WARN("进入跨上台阶面子程序（考虑重力）。");

    ssf_D = 80;
    ssf_H = 100;
    ROS_ERROR("记得修改！！！！！！实际台阶高度和距离");


    float g_v[3], g_w[3], g_foot1[3], g_foot2[3], g_foot3[3], g_foot4[3], a[3], b[3], c[3], d[3];
    for (int i=0; i<3; i++)
    {
        g_v[i] = vw_foot_abcd[i];     g_w[i] = vw_foot_abcd[i+3];

        g_foot1[i] = vw_foot_abcd[i+6];
        g_foot2[i] = vw_foot_abcd[i+9];
        g_foot3[i] = vw_foot_abcd[i+12];
        g_foot4[i] = vw_foot_abcd[i+15];

        a[i] = vw_foot_abcd[i+18];
        b[i] = vw_foot_abcd[i+21];
        c[i] = vw_foot_abcd[i+24];
        d[i] = vw_foot_abcd[i+27];
    }


    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);

    playtime = playtime * 1.2 ;

    //机身抬起 - 准备跨上台阶
    g_v[0] = 0; g_v[1] = 0;
    g_v[2] = g_v[2] + ssf_H; //body up, foot down
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
    int error_codes=Set_ALL_Leg_Positon(a,b,c,d);
    if(error_codes == 0 )
    {
        Send_ALL_Leg_Positon( 1.6* playtime );  //0x3c  2000ms
        usleep( 1.6*playtime*1000 );

    }
    else
        return 1;

    int error_back = 0;
    int stepD_tmp = 200;
    int stepHeight = dz + ssf_H;

    //1-3-2-4  第一环节，腿1和2上台阶
    ROS_INFO("1/3 腿1和2上台阶");
    //1-3-2-4
    //腿1进,调整重心
    ROS_INFO("1.body+leg1");
    g_v[0] = -bodyadjX;   //相对前一时刻，相对值
    g_v[1] = -bodyadjY;   //-Y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.6*playtime) );
        usleep( 0.6*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }
    error_back = Make_One_Leg_Runonce_dz(1, a, stepD_tmp, 0, stepHeight, dz, playtime);
    g_foot1[0]=g_foot1[0]+stepD_tmp;   //腿末端位置是后来移动机身的参数
    g_foot1[2]=g_foot1[2]+stepHeight-dz;
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //腿3进,先调整重心
    ROS_INFO("2.body+leg3");
    g_v[0] = bodyadjX + stepD_tmp/2;
    g_v[1] = 2*bodyadjY;
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }

    error_back = Make_One_Leg_Runonce_dz(3, c, stepD_tmp, 0, dz, dz, playtime);
    g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }


    //腿2进
    ROS_INFO("3.body+leg2");
    error_back = Make_One_Leg_Runonce_dz(2, b, stepD_tmp, 0, stepHeight, dz, playtime);
    g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
    g_foot2[2]=g_foot2[2]+stepHeight-dz;
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //腿4进,调整重心
    ROS_INFO("4.body+leg4");
    g_v[0] =  +bodyadjX + stepD_tmp/2;   //相对前一时刻，相对值
    g_v[1] = -2*bodyadjY;   //-y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }

    error_back = Make_One_Leg_Runonce_dz(4, d, stepD_tmp, 0, dz, dz, playtime);
    g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //5,调整重心,回归初始姿态
    ROS_INFO("5.body return zero");
    g_v[0] = -bodyadjX;   //相对前一时刻，相对值
    g_v[1] = bodyadjY;   //y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
        usleep( 0.35*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }

    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);



    //第二环节，腿12在台阶上，腿34在台阶下，前进一步
    ROS_INFO("2/3 腿12在台阶上，腿34在台阶下，前进一步");
    //1-3-2-4
    //腿1进,调整重心
    ROS_INFO("1.body+leg1");
    g_v[0] = -bodyadjX;   //相对前一时刻，相对值
    g_v[1] = -bodyadjY;   //-Y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
        usleep( 0.4*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }
    error_back = Make_One_Leg_Runonce_dz(1, a, stepD_tmp, 0, dz, dz, playtime);
    g_foot1[0]=g_foot1[0]+stepD_tmp;   //腿末端位置是后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //腿3进,先调整重心
    ROS_INFO("2.body+leg3");
    g_v[0] = bodyadjX + stepD_tmp/2;
    g_v[1] = 2*bodyadjY;
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }

    error_back = Make_One_Leg_Runonce_dz(3, c, stepD_tmp, 0, dz, dz, playtime);
    g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }


    //腿2进
    ROS_INFO("3.body+leg2");
    error_back = Make_One_Leg_Runonce_dz(2, b, stepD_tmp, 0, dz, dz, playtime);
    g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //腿4进,调整重心
    ROS_INFO("4.body+leg4");
    g_v[0] =  +bodyadjX + stepD_tmp/2;   //相对前一时刻，相对值
    g_v[1] = -2*bodyadjY;   //-y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }

    error_back = Make_One_Leg_Runonce_dz(4, d, stepD_tmp, 0, dz, dz, playtime);
    g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //5,调整重心,回归初始姿态
    ROS_INFO("5.body return zero");
    g_v[0] = -bodyadjX;   //相对前一时刻，相对值
    g_v[1] = bodyadjY;   //y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
        usleep( 0.35*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }


    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);



    //第三环节，腿3,4上台阶
    ROS_INFO("3/3 腿3,4上台阶");
    //1-3-2-4
    //腿1进,调整重心
    ROS_INFO("1.body+leg1");
    g_v[0] = -bodyadjX;   //相对前一时刻，相对值
    g_v[1] = -bodyadjY;   //-Y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
        usleep( 0.4*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }
    error_back = Make_One_Leg_Runonce_dz(1, a, stepD_tmp, 0, dz, dz, playtime);
    g_foot1[0]=g_foot1[0]+stepD_tmp;   //腿末端位置是后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //腿3进,先调整重心
    ROS_INFO("2.body+leg3");
    g_v[0] = bodyadjX + stepD_tmp/2;
    g_v[1] = 2*bodyadjY;
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }

    error_back = Make_One_Leg_Runonce_dz(3, c, stepD_tmp, 0, stepHeight, dz, playtime);
    g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
    g_foot3[2]=g_foot3[2]+stepHeight-dz;
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }


    //腿2进
    ROS_INFO("3.body+leg2");
    error_back = Make_One_Leg_Runonce_dz(2, b, stepD_tmp, 0, dz, dz, playtime);
    g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //腿4进,调整重心
    ROS_INFO("4.body+leg4");
    g_v[0] =  +bodyadjX + stepD_tmp/2;   //相对前一时刻，相对值
    g_v[1] = -2*bodyadjY;   //-y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }

    error_back = Make_One_Leg_Runonce_dz(4, d, stepD_tmp, 0, stepHeight, dz, playtime);
    g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
    g_foot4[2]=g_foot4[2]+stepHeight-dz;
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back == 1");
        return error_back;
    }

    //5,调整重心,回归初始姿态
    ROS_INFO("5.body return zero");
    g_v[0] = -bodyadjX;   //相对前一时刻，相对值
    g_v[1] = bodyadjY;   //y
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 && !stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.4*playtime) );
        usleep( 0.35*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;
    }



    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);


    for (int i=0;i<3;i++)   //更新vwfa_backdata
    {
        vwfa_backdata[i] = g_v[i];
        vwfa_backdata[i+3] = g_w[i];
        vwfa_backdata[i+6] = g_foot1[i];
        vwfa_backdata[i+9] = g_foot2[i];
        vwfa_backdata[i+12] = g_foot3[i];
        vwfa_backdata[i+15] = g_foot4[i];
        vwfa_backdata[i+18] = a[i];
        vwfa_backdata[i+21] = b[i];
        vwfa_backdata[i+24] = c[i];
        vwfa_backdata[i+27] = d[i];
    }
}











//====================   不考虑重力（无重心调整）下的单步周期：四腿  ===========================================
/* 输入：vw_foot_abcd[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]]; 步长(沿xy分别dx，dy)mm， 抬脚高度dzmm， 单步周期ms
 * 则目标足端相对位置，x+stepDistance
 * 返回失败代码：1失败， 0成功
 * 更新数组（30个数）backdata[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]
*/
//float* Make_Runonce_noGravity( float g_v[3], float g_w[3], float g_foot[12], float abcd[12], int dx, int dy, int dz, uint16_t playtime )
//int Make_Runonce_noGravity( float g_v[3], float g_w[3], float g_foot[12], float abcd[12], int dx, int dy, int dz, uint16_t playtime, float backdata[30] )
int Make_Runonce_noGravity( float vw_foot_abcd[30],  int dx, int dy, int dz, uint16_t playtime, float vwfa_backdata[30] )
{
    ROS_WARN("进入单步行走子程序。");

    float g_v[3], g_w[3], g_foot1[3], g_foot2[3], g_foot3[3], g_foot4[3], a[3], b[3], c[3], d[3];
    for (int i=0; i<3; i++)
    {
        g_v[i] = vw_foot_abcd[i];     g_w[i] = vw_foot_abcd[i+3];

        g_foot1[i] = vw_foot_abcd[i+6];
        g_foot2[i] = vw_foot_abcd[i+9];
        g_foot3[i] = vw_foot_abcd[i+12];
        g_foot4[i] = vw_foot_abcd[i+15];

        a[i] = vw_foot_abcd[i+18];
        b[i] = vw_foot_abcd[i+21];
        c[i] = vw_foot_abcd[i+24];
        d[i] = vw_foot_abcd[i+27];
    }


    ROS_INFO("各足末端绝对位置（原点为本体中心在地面投影）:\n g_foot1 = %f,%f,%f \n g_foot2 = %f,%f,%f \n g_foot3 = %f,%f,%f \n g_foot4 = %f,%f,%f",
             g_foot1[0], g_foot1[1],g_foot1[2], g_foot2[0], g_foot2[1],g_foot2[2],
            g_foot3[0], g_foot3[1],g_foot3[2], g_foot4[0], g_foot4[1],g_foot4[2]);
    ROS_INFO("各足末端相对（腿起始端）位置：\n a[x,y,z] = %f,%f,%f \n b[x,y,z] = %f,%f,%f \n c[x,y,z] = %f,%f,%f \n d[x,y,z] = %f,%f,%f",
             a[0], a[1],a[2], b[0], b[1],b[2] ,c[0], c[1],c[2] ,d[0], d[1],d[2]);


    int error_back=0;
    //前进1-3-2-4
    if ( (dy==0) && (dx>0) )
    {
        ROS_INFO("前进");
        int stepD_tmp = dx;
        int stepHeight = dz;
        //1-3-2-4
        //腿1进
        ROS_INFO("1.leg1");
        error_back = Make_One_Leg_Runonce(1, a, stepD_tmp, 0, stepHeight, playtime);
        g_foot1[0]=g_foot1[0]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }

        //腿3进
        ROS_INFO("2.leg3");
        error_back = Make_One_Leg_Runonce(3, c, stepD_tmp, 0, stepHeight, playtime);
        g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;;
        }

        //body
        ROS_INFO("3.body move");
        g_v[0] = stepD_tmp;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 || stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
            usleep( 0.8*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;;
        }

        //腿2进
        ROS_INFO("4.leg2");
        error_back = Make_One_Leg_Runonce(2, b, stepD_tmp, 0, stepHeight, playtime);
        g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;;
        }

        //腿4进
        ROS_INFO("5.leg4");
        error_back = Make_One_Leg_Runonce(4, d, stepD_tmp, 0, stepHeight, playtime);
        g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;;
        }
//        以上步骤结束后，四条腿回归原位置

    }

    else if ( (dy==0) && (dx<0) )  //后退
    {
         ROS_INFO("后退");
        int stepD_tmp = -dx;
        int stepHeight = dz;
        //4-2-3-1
        //腿4后退
        ROS_INFO("1.leg4");
        error_back = Make_One_Leg_Runonce(4, d, -stepD_tmp, 0, stepHeight, playtime);
        g_foot4[0]=g_foot4[0]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;;
        }

        //腿2后退
        ROS_INFO("2.leg2");
        error_back = Make_One_Leg_Runonce(2, b, -stepD_tmp, 0, stepHeight, playtime);
        g_foot2[0]=g_foot2[0]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }

        //body
        ROS_INFO("3.body move");
        g_v[0] = -stepD_tmp;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0 || stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
            usleep( 0.8*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;;
        }

        //腿3后退
        ROS_INFO("4.leg3");
        error_back = Make_One_Leg_Runonce(3, c, -stepD_tmp, 0, stepHeight, playtime);
        g_foot3[0]=g_foot3[0]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }


        //腿1后退
        ROS_INFO("5.leg1");
        error_back = Make_One_Leg_Runonce(1, a, -stepD_tmp, 0, stepHeight, playtime);
        g_foot1[0]=g_foot1[0]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }
    }

    else if ( (dx==0) && (dy>0) )  //左移动
    {
         ROS_INFO("左移");
        int stepD_tmp = dy;
        int stepHeight = dz;

        //1-4-3-2
        //腿1进
        ROS_INFO("1.leg1");
        error_back = Make_One_Leg_Runonce(1, a, 0, stepD_tmp, stepHeight, playtime);
        g_foot1[1]=g_foot1[1]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }

        //腿4进
        ROS_INFO("2.leg4");
        error_back = Make_One_Leg_Runonce(4, d, 0, stepD_tmp, stepHeight, playtime);
        g_foot4[1]=g_foot4[1]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }


        //body move + 腿3进,调整重心
        ROS_INFO("3+4.body+leg3");
        g_v[1] = stepD_tmp;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0  && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (1*playtime) );
            usleep( 1*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;;
        }

        error_back = Make_One_Leg_Runonce(3, c, 0, stepD_tmp, stepHeight, playtime);
        g_foot3[1]=g_foot3[1]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }

        //腿2进
        ROS_INFO("5.leg2");
        error_back = Make_One_Leg_Runonce(2, b, 0, stepD_tmp, stepHeight, playtime);
        g_foot2[1]=g_foot2[1]+stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }
    }

    else if ( (dx==0) && (dy<0) )  //右平移
    {
         ROS_INFO("右移");
        int stepD_tmp = -dy;
        int stepHeight = dz;

        //2-3-4-1
        //腿2进
        ROS_INFO("1.leg2");
        error_back = Make_One_Leg_Runonce(2, b, 0, -stepD_tmp, stepHeight, playtime);
        g_foot2[1]=g_foot2[1]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }

        //腿3进
        ROS_INFO("2.leg3");
        error_back = Make_One_Leg_Runonce(3, c, 0, -stepD_tmp, stepHeight, playtime);
        g_foot3[1]=g_foot3[1]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }

        //body move + 腿4进,调整重心
        ROS_INFO("3+4.body+leg4");
        g_v[1] = -stepD_tmp;
        CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
        error_back = Set_ALL_Leg_Positon(a,b,c,d);
        if(error_back == 0  && !stopFlag)
        {
            Send_ALL_Leg_Positon(  (uint16_t) (1*playtime) );
            usleep( 1*playtime*1000);
        }
        else
        {
            ROS_INFO("机身进出错。 error_codes=%d",error_back );
            return error_back;;
        }

        error_back = Make_One_Leg_Runonce(4, d, 0, -stepD_tmp, stepHeight, playtime);
        g_foot4[1]=g_foot4[1]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }

        //腿1进
        ROS_INFO("5.leg1");
        error_back = Make_One_Leg_Runonce(1, a, 0, -stepD_tmp, stepHeight, playtime);
        g_foot1[1]=g_foot1[1]-stepD_tmp;   //后来移动机身的参数
        if (error_back == 1 || stopFlag)
        {
            ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
            return error_back;
        }
    }









    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);

    for (int i=0;i<3;i++)   //更新vwfa_backdata， 其实不需要更新，保证前后腿回归原位就行
    {
        vwfa_backdata[i] = g_v[i];
        vwfa_backdata[i+3] = g_w[i];
        vwfa_backdata[i+6] = g_foot1[i];
        vwfa_backdata[i+9] = g_foot2[i];
        vwfa_backdata[i+12] = g_foot3[i];
        vwfa_backdata[i+15] = g_foot4[i];
        vwfa_backdata[i+18] = a[i];
        vwfa_backdata[i+21] = b[i];
        vwfa_backdata[i+24] = c[i];
        vwfa_backdata[i+27] = d[i];
    }

}



//====================   不考虑重力（无重心调整）下的跨上台阶面  ===========================================
/* 输入：vw_foot_abcd[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]]; 距台阶面ssf_D， 台阶高度ssf_H，抬脚高度dz， 单步周期ms
 * 则目标足端相对位置，x+stepDistance
 * 返回失败代码：1失败， 0成功
 * 更新数组（30个数）backdata[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]
*/
int Climb_stepsurface_noGravity( float vw_foot_abcd[30],  int ssf_D, int ssf_H, int dz, uint16_t playtime, float vwfa_backdata[30] )
{
    ROS_INFO("进入跨上台阶面子程序。");
    ssf_D = 80;
    ssf_H = 100;
    ROS_ERROR("记得修改！！！！！！实际台阶高度和距离");


    float g_v[3], g_w[3], g_foot1[3], g_foot2[3], g_foot3[3], g_foot4[3], a[3], b[3], c[3], d[3];
    for (int i=0; i<3; i++)
    {
        g_v[i] = vw_foot_abcd[i];     g_w[i] = vw_foot_abcd[i+3];

        g_foot1[i] = vw_foot_abcd[i+6];
        g_foot2[i] = vw_foot_abcd[i+9];
        g_foot3[i] = vw_foot_abcd[i+12];
        g_foot4[i] = vw_foot_abcd[i+15];

        a[i] = vw_foot_abcd[i+18];
        b[i] = vw_foot_abcd[i+21];
        c[i] = vw_foot_abcd[i+24];
        d[i] = vw_foot_abcd[i+27];
    }


    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);



    if (ssf_D>100) return 1;

    //机身抬起 - 准备跨上台阶
    g_v[0] = 0; g_v[1] = 0;
    g_v[2] = g_v[2] + ssf_H; //body up, foot down
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);
    int error_codes=Set_ALL_Leg_Positon(a,b,c,d);
    if(error_codes == 0 )
    {
        Send_ALL_Leg_Positon( 2000 );  //playtime,  0x3c  2000ms
        usleep(2000*1000 );    // playtime*1000
    }
    else
        return 1;

    int error_back = 0;
    int stepD_tmp = 200;
    int stepHeight = dz + ssf_H;

    //1-3-2-4  第一环节，腿1和2上台阶
    //腿1进
    ROS_INFO("1.leg1");
    error_back = Make_One_Leg_Runonce_dz(1, a, stepD_tmp, 0, stepHeight, dz, playtime);
    g_foot1[0]=g_foot1[0]+stepD_tmp;   //腿末端位置是后来移动机身的参数
    g_foot1[2]=g_foot1[2]+stepHeight-dz;
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;
    }
    //腿3进
    ROS_INFO("2.leg3");
    error_back = Make_One_Leg_Runonce_dz(3, c, stepD_tmp, 0, dz, dz, playtime);
    g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }
    //body
    ROS_INFO("3.body move");

    g_v[0] = stepD_tmp;
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 || stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;;
    }
    //腿2进
    ROS_INFO("4.leg2");
    error_back = Make_One_Leg_Runonce_dz(2, b, stepD_tmp, 0, stepHeight, dz, playtime);
    g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
    g_foot2[2]=g_foot2[2]+stepHeight-dz;
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }
    //腿4进
    ROS_INFO("5.leg4");
    error_back = Make_One_Leg_Runonce_dz(4, d, stepD_tmp, 0, dz, dz, playtime);
    g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }


    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);



    //第二环节，腿12在台阶上，腿34在台阶下，前进一步
    //腿1进
    ROS_INFO("1.leg1");
    error_back = Make_One_Leg_Runonce_dz(1, a, stepD_tmp, 0, dz, dz, playtime);
    g_foot1[0]=g_foot1[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;
    }
    //腿3进
    ROS_INFO("2.leg3");
    error_back = Make_One_Leg_Runonce_dz(3, c, stepD_tmp, 0, dz, dz, playtime);
    g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }
    //body
    ROS_INFO("3.body move");
    g_v[0] = stepD_tmp;
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 || stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;;
    }
    //腿2进
    ROS_INFO("4.leg2");
    error_back = Make_One_Leg_Runonce_dz(2, b, stepD_tmp, 0, dz, dz, playtime);
    g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }
    //腿4进
    ROS_INFO("5.leg4");
    error_back = Make_One_Leg_Runonce_dz(4, d, stepD_tmp, 0, dz, dz, playtime);
    g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }




    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);



    //第三环节，腿3,4上台阶
    //腿1进
    ROS_INFO("1.leg1");
    error_back = Make_One_Leg_Runonce_dz(1, a, stepD_tmp, 0, dz, dz, playtime);
    g_foot1[0]=g_foot1[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;
    }
    //腿3进
    ROS_INFO("2.leg3");
    error_back = Make_One_Leg_Runonce_dz(3, c, stepD_tmp, 0, stepHeight, dz, playtime);
    g_foot3[0]=g_foot3[0]+stepD_tmp;   //后来移动机身的参数
    g_foot3[2]=g_foot3[2]+stepHeight-dz;
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }
    //body
    ROS_INFO("3.body move");
    g_v[0] = stepD_tmp;
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    error_back = Set_ALL_Leg_Positon(a,b,c,d);
    if(error_back == 0 || stopFlag)
    {
        Send_ALL_Leg_Positon(  (uint16_t) (0.8*playtime) );
        usleep( 0.8*playtime*1000);
    }
    else
    {
        ROS_INFO("机身进出错。 error_codes=%d",error_back );
        return error_back;;
    }
    //腿2进
    ROS_INFO("4.leg2");
    error_back = Make_One_Leg_Runonce_dz(2, b, stepD_tmp, 0, dz, dz, playtime);
    g_foot2[0]=g_foot2[0]+stepD_tmp;   //后来移动机身的参数
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }
    //腿4进
    ROS_INFO("5.leg4");
    error_back = Make_One_Leg_Runonce_dz(4, d, stepD_tmp, 0, stepHeight, dz, playtime);
    g_foot4[0]=g_foot4[0]+stepD_tmp;   //后来移动机身的参数
    g_foot4[2]=g_foot4[2]+stepHeight-dz;
    if (error_back == 1 || stopFlag)
    {
        ROS_WARN("error_back=%d, stopFlag=%d", error_back, stopFlag);
        return error_back;;
    }

    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot4 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);


    for (int i=0;i<3;i++)   //更新vwfa_backdata
    {
        vwfa_backdata[i] = g_v[i];
        vwfa_backdata[i+3] = g_w[i];
        vwfa_backdata[i+6] = g_foot1[i];
        vwfa_backdata[i+9] = g_foot2[i];
        vwfa_backdata[i+12] = g_foot3[i];
        vwfa_backdata[i+15] = g_foot4[i];
        vwfa_backdata[i+18] = a[i];
        vwfa_backdata[i+21] = b[i];
        vwfa_backdata[i+24] = c[i];
        vwfa_backdata[i+27] = d[i];
    }
}




//====================   bodypose_Control  ===========================================
/* 输入：当前各足端相对位置leg[3]=（x,y,z）mm，步长(沿xy分别dx，dy)mm， 抬脚高度dzmm， 单步周期ms
 * 则目标足端相对位置，x+stepDistance
 * 返回失败代码：1失败， 0成功
 * 更新数组（30个数）backdata[30] = [g_v[3], g_w[3], g_foot[12], abcd[12]
*/
//int bodypose_Control( float g_v[3], float g_w[3], float g_foot[12], float abcd[12], int dx, int dy, int dz, uint16_t playtime, float backdata[30] )
//{


//}



// ========================     正解+逆解 函数     ============================================================
//每条腿三连杆长度
#define L1  54
#define L2  187
#define L3  230
//输入三关节舵机角度(a1,a2,a3)，正解出末端坐标(x,y,z，error)，  ，角度单位度, mm
float* jointsToPosition(float q1, float q2, float q3)
{
    setlocale (LC_CTYPE, "zh_CN.utf8");
    int error= 1 ;
    float px=0, py=0, pz=0;

    // 输入的是舵机角度，转为模型的角度, 角度转为弧度制
    float a1 = (180 + q1)  *M_PI/180 ;   //  本体两侧的腿 加减不一样！！
    float a2 = (180 - q2)  *M_PI/180 ;
    float a3 = (q3 - 180)  *M_PI/180 ;

    //求解末端坐标 xyz mm
    py = (L1+L2*cos(a2)+L3*cos(a2+a3)) *sin(a1) ;
    px = (L1+L2*cos(a2)+L3*cos(a2+a3)) *cos(a1) ;
    pz = L2*sin(a2)+L3*sin(a2+a3) ;
    error=0;

    float *res;
    res = (float *)malloc(4);
    res[0]= py;   // 改为X方向为前进方向
    res[1]= px;
    res[2]= pz;
    res[3]= error;
    return res;

}


//输入末端坐标(x,y,z)，逆解出三关节角度并返回(a1,a2,a3,error)   ，角度单位度
//  末端坐标是相对于腿起始端坐标,针对腿1,4
//  腿2,3在 Y- 侧， 也改为正值
float* positionToJoints(float px,  float py,  float pz)
{
    setlocale (LC_CTYPE, "zh_CN.utf8");
//    ROS_INFO("\npx,py,pz = %f, %f, %f", px,py,pz);

    int error= 0 ;
    float a1=0, a2=0, a3=0;


    if (py<0)
        ROS_WARN("逆解输入Y为负！足端位置在身体内测！");
    else
    {
    // 求解关节角a1 a2 a3
        a1 = atan(px/py);
//        ROS_INFO("px/py=%f, a1=%f",px/py, a1/M_PI*180);
        float A = ( (py/cos(a1)-L1)*(py/cos(a1)-L1) + pz*pz+ L2*L2- L3*L3 )/( 2*(py/cos(a1)-L1)*L2 );  //定义
        float B = pz*L2/( (py/cos(a1)-L1)*L2 );
        float D = 4*A*A*B*B-4*(B*B+1)*(A*A-1);
//        ROS_INFO("A=%f, B=%f, D=%f", A,B,D);
        if (D>=0 )
        {
            error = 0;

            float sina2 = (2*A*B + sqrt(D))/(2*(B*B+1));      // - 是足底朝上的构型
            a2 = asin(sina2);
            float sina2a3 = (pz-L2*sin(a2))/L3;
            float p2y = (L1+L2*cos(a2))*cos(a1);
            float p2z = L2*sin(a2); //按照构型要求，连杆L2末端应比足底（连杆L3末端）位置高
//            ROS_INFO("sina2=%f, a2=%f",sina2, a2/M_PI*180);
//            ROS_INFO("sina2a3=%f, a2+a3=%f",sina2a3, asin(sina2a3)/M_PI*180);
//            ROS_INFO("p2y=%f", p2y);
//            ROS_INFO("p2z=%f", p2z);

            if (p2y < py)    //如果足底在最外侧
            {
                a3 = asin(sina2a3)-a2 ;
//                ROS_INFO("如果足底最外p2y < py: a3=%f", a3/M_PI*180);
            }
            else  //如果足底内收
            {
                a3 = M_PI-asin(sina2a3) - a2 - 2*M_PI;    // a2+a3范围-270~180
//                ROS_INFO("如果足底内收p2y > py: a3=%f", a3/M_PI*180);
            }

            //反求解末端坐标 xyz mm, 判断差值
            float px2 = (L1+L2*cos(a2)+L3*cos(a2+a3)) *sin(a1);
            float py2 = (L1+L2*cos(a2)+L3*cos(a2+a3)) *cos(a1);
            float pz2 = L2*sin(a2)+L3*sin(a2+a3);
            float dx = px2 - px;
            float dy = py2 - py;
            float dz = pz2 - pz;
//            ROS_WARN("反求末端坐标xyz值= %f, %f, %f", px2,py2,pz2);

            if ( abs(dx)>5  || abs(dy)>5  || abs(dz)>5 )
            {
                error = 1;
//                ROS_WARN("逆解值与输入值有差异，取另一组解");

                sina2 = (2*A*B - sqrt(D))/(2*(B*B+1));      // - 是足底朝上的构型
                a2 = asin(sina2);
                sina2a3 = (pz-L2*sin(a2))/L3;
                float p2y = L1+L2*cos(a2);
                float p2z = L2*sin(a2);

                if (p2y > py)    //如果足底内收
                {
                    a3 = M_PI-asin(sina2a3) - a2 - 2*M_PI;    // a2+a3范围-270~180
                }
                else
                {
                    a3 = asin(sina2a3)-a2 ;   //如果足底在最外侧
                }

                //反求解末端坐标 xyz mm, 判断差值
                float px2 = (L1+L2*cos(a2)+L3*cos(a2+a3)) *sin(a1);
                float py2 = (L1+L2*cos(a2)+L3*cos(a2+a3)) *cos(a1);
                float pz2 = L2*sin(a2)+L3*sin(a2+a3);
                float dx = px2 - px;
                float dy = py2 - py;
                float dz = pz2 - pz;
//                ROS_WARN("反求末端坐标xyz值= %f, %f, %f", px2,py2,pz2);

                if ( abs(dx)>5  || abs(dy)>5  || abs(dz)>5 )
                {
                    error = 1;
//                    ROS_WARN("另一组解的逆解值与输入值也有差异，失败，请核查目标位置");
                }
//                else
//                    ROS_INFO("另一组解逆解验证成功。");

            }
//            else
//                ROS_INFO("逆解成功。");


//            if (p2z < pz)  //如果构型有误，取另一组解

        }
        else
        {
            error = 1;
            a1=0;a2=0;a3=0;
        }
    }



    //转为 度° 输出
    a1=a1*180.0/M_PI;
    a2=a2*180.0/M_PI;
    a3=a3*180.0/M_PI;
    //转为舵机定义的角度，全伸直是180度
    float q1 = 180 + a1 ;
    float q2 = 180 - a2 ;
    float q3 = a3 + 180 ;
//    ROS_INFO("joints = %f,%f,%f",q1, q2, q3);

    if (  (q1<ANGLE1_MIN)||(q1>ANGLE1_MAX) || (q2<ANGLE2_MIN)||(q2>ANGLE2_MAX) || (q3<ANGLE3_MIN)||(q3>ANGLE3_MAX) )
    {
        error=2;
        ROS_WARN("逆解值超出舵机设定，请核查目标位置");
    }

    float *res;
    res = (float *)malloc(4);
    res[0]= q1;
    res[1]= q2;
    res[2]= q3;
    res[3]= error;
    return res;
}



//=============================   两个3X3矩阵相乘        ==========================
//矩阵A：3x3, 矩阵B：3x3
float** multiply_twoMats(float (*A)[3], float B[][3])
{
    float **C = new float*[3];
    for (int i = 0; i < 3; i++)
        C[i] = new float[3];
    for (int m = 0; m < 3; m++)
        for (int n = 0; n < 3; n++)
        {
            C[m][n] = 0;
            for (int k = 0; k < 3; k++)
                C[m][n] += A[m][k] * B[k][n];
        }
    return C;
}






// ====================
void test()
{
    ROS_INFO("void test()");
    int stepDistance = 100;
    int stepHeight=50;
    int n_x = 0;
    int n_y = 150;
    float g_v[3] = {0, 0, 100};
    float g_w[3] = {0, 0, 0};  // X Y Z 三轴角度
    float leg[3];
    char legID;
    int error_back;

    ROS_WARN("0");
    float g_foot1[3], g_foot2[3], g_foot3[3], g_foot4[3];
    g_foot1[0] =  CENTER_L/2 + n_x;    g_foot1[1] =  CENTER_B/2 + n_y;     g_foot1[2] =30;
    g_foot2[0] =  CENTER_L/2 + n_x;    g_foot2[1] = -(CENTER_B/2 + n_y);     g_foot2[2] =0;
    g_foot3[0] = -(CENTER_L/2 + n_x);  g_foot3[1] = -(CENTER_B/2 + n_y);     g_foot3[2] =0;
    g_foot4[0] = -(CENTER_L/2 + n_x);  g_foot4[1] =  CENTER_B/2 + n_y;     g_foot4[2] =0;

    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    float a[3],b[3],c[3],d[3];  // 定义a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm
    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);

    ROS_INFO("after:g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    //奇怪，Set_ALL_Leg_Positon之后b[1],c[1]符号变了  后来改了
    int error_codes = Set_ALL_Leg_Positon(a,b,c,d);
//    ROS_INFO("after:\na[3]=%f, %f, %f; \nb[3]=%f, %f, %f; \nc[3]=%f, %f, %f; \nd[3]=%f, %f, %f",a[0],a[1],a[2],b[0],b[1],b[2],c[0],c[1],c[2],d[0],d[1],d[2]);

    ROS_WARN("1. 机身进步长 %dmm。 步长:%dmm",stepDistance, stepDistance);
    g_v[0] =  stepDistance;  //g_v[0] - 0.5*stepDistance
    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);
    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
    ROS_INFO("g_v = %f,%f,%f",g_v[0], g_v[1],g_v[2]);

    ROS_INFO("after:g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
    ROS_INFO("g_foot3 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

    ROS_INFO("a[x,y,z] = %f,%f,%f",a[0], a[1],a[2]);
    ROS_INFO("b[x,y,z] = %f,%f,%f",b[0], b[1],b[2]);
    ROS_INFO("c[x,y,z] = %f,%f,%f",c[0], c[1],c[2]);
    ROS_INFO("d[x,y,z] = %f,%f,%f",d[0], d[1],d[2]);
    error_codes = Set_ALL_Leg_Positon(a,b,c,d);



    float *repositon;
    repositon = jointsToPosition(Leg_aim_angle[0], Leg_aim_angle[1], Leg_aim_angle[2]);  //输入三关节角度(a1,a2,a3)，正解出末端坐标(x,y,z，error)，
    if (repositon[3]==0)
    {
        ROS_INFO("(X,Y,Z)=%f, %f, %f", repositon[0],repositon[1],repositon[2] );
    }
    else
    {
        ROS_WARN("未正解出腿%d足端相对位置", legID);
    }

}












