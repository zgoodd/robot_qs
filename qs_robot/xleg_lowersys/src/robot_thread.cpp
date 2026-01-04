//#include <math.h>
//#include "robot_function.h"
//#include <iostream>
//#include <ros/ros.h>

//void gait_thread()
//{
//    // define i here???
//    int ii = 0;
//    int error_back=1;
//    float a[3],b[3],c[3],d[3];  // 定义a,b,c,d 各条腿末端（足端）相对于腿起点的坐标(x,y,z) mm
//    float leg[3];
//    char legID;

//    if(stopFlag)
//    {
//        ii = 0;
//    }
//    else
//    {
//        if(climbMode==true)     //越障模式
//        {
//            ROS_WARN("越障模式还没完成...");
//        }
//        else
//        {
//            if(gaitType == xleg_msgs::gaitControl::Request::GAIT_TRANS)   //3+1
//            {
//                uint16_t playtime = floor(T*1000/5.8);  //6步，每步时间ms, 循环里采集力信息舵机信息也需要时间
////                    ticks = 24;
//                int nstep = floor(ticks/6);   //一个步态周期执行 ticks 次循环，6步，每步 nstep 次循环
////                    ROS_INFO("into GAIT_TRANS:\n    step.ii=%d of %d,   playtime=%dms", ii, nstep,playtime);

//                //机身退
//                if (ii == 0)
//                {
//                    ROS_INFO("0: g_v=%f, %f, %f", g_v[0], g_v[1], g_v[2]);
//                    ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
//                    ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
//                    ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
//                    ROS_INFO("g_foot3 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

//                    g_v[0] = - 0.5*stepDistance;
//                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
//                    ROS_INFO("before:\na[3]=%f, %f, %f; \nb[3]=%f, %f, %f; \nc[3]=%f, %f, %f; \nd[3]=%f, %f, %f",a[0],a[1],a[2],b[0],b[1],b[2],c[0],c[1],c[2],d[0],d[1],d[2]);
//                    int error_codes=Set_ALL_Leg_Positon(a,b,c,d);
////                        ROS_INFO("after:\na[3]=%f, %f, %f; \nb[3]=%f, %f, %f; \nc[3]=%f, %f, %f; \nd[3]=%f, %f, %f",a[0],a[1],a[2],b[0],b[1],b[2],c[0],c[1],c[2],d[0],d[1],d[2]);

//                    if(error_codes == 0)
//                        Send_ALL_Leg_Positon( playtime );
//                    else
//                        ROS_INFO("机身退出错。 error_codes=%d",error_codes );
//                }
//                //腿1进
//                else if ( ii == nstep )
//                {
//                    leg[0]=a[0]+ 1*stepDistance;
//                    leg[1]=a[1];
//                    leg[2]=a[2]+ stepHeight;
//                    legID=1;
//                    ROS_INFO("1: a[3]=%f, %f, %f", leg[0], leg[1], leg[2]);

//                    error_back = Set_One_Leg_Positon(legID, leg);
//                    if (error_back == 0)
//                        Send_One_Leg_Positon(legID, (uint16_t) (playtime/2));  //时间长点，没停就紧接着下一步
//                    else
//                    {
//                        ROS_ERROR("腿%d 抬 解算出错。",legID);
//                        stopFlag = true;
//                    }
//                }

//                else if ( ii == floor(1.5*nstep) )   //落
//                {
//                    leg[2]=leg[2] - stepHeight;

//                    ROS_INFO("1:1 a[3]=%f, %f, %f", leg[0], leg[1], leg[2]);

//                    error_back = Set_One_Leg_Positon(legID, leg);
//                    if (error_back == 0)
//                        Send_One_Leg_Positon(legID, (uint16_t) (playtime/2));  //时间长点，没停就紧接着下一步
//                    else
//                    {
//                        ROS_ERROR("腿%d 落 解算出错。",legID);
//                        stopFlag = true;
//                    }
//                }

//                //腿2进
//                else if ( ii == 2*nstep )
//                {
//                    leg[0]=b[0]+ 1.0*stepDistance;
//                    leg[1]=b[1];
//                    leg[2]=b[2]+ stepHeight;
//                    legID=2;
//                    ROS_INFO("2: b[3]=%f, %f, %f", leg[0], leg[1], leg[2]);

//                    error_back = Set_One_Leg_Positon(legID, leg);
//                    if (error_back == 0)
//                        Send_One_Leg_Positon(legID, (uint16_t) (playtime/2));  //时间长点，没停就紧接着下一步
//                    else
//                    {
//                        ROS_ERROR("腿%d 抬 解算出错。",legID);
//                        stopFlag = true;
//                    }
//                }
//                else if ( ii == floor(2.5*nstep) )   //落
//                {
//                    leg[2]=leg[2] - stepHeight;
//                    error_back = Set_One_Leg_Positon(legID, leg);
//                    if (error_back == 0)
//                        Send_One_Leg_Positon(legID, (uint16_t) (playtime/2));  //时间长点，没停就紧接着下一步
//                    else
//                    {
//                        ROS_ERROR("腿%d 落 解算出错。",legID);
//                        stopFlag = true;
//                    }
//                }

//                //机身进
//                else if (ii == 3*nstep)
//                {
//                    g_v[0] = g_v[0] + 1.5*stepDistance;
//                    // 之前腿1和2各进了步长，加上。
//                    g_foot1[0]=g_foot1[0]+1*stepDistance;
//                    g_foot2[0]=g_foot2[0]+1*stepDistance;

//                    CCenterToLeg(g_v,g_w,g_foot1,g_foot2,g_foot3,g_foot4,a,b,c,d);   //a,b,c,d均为输出参数，
//                    int error_codes=Set_ALL_Leg_Positon(a,b,c,d);
//                    if(error_codes == 0)
//                        Send_ALL_Leg_Positon( playtime );
//                }
//                //腿3进
//                else if ( ii == 4*nstep )
//                {
//                    leg[0]=c[0]+ stepDistance;
//                    leg[1]=c[1];
//                    leg[2]=c[2]+ stepHeight;
//                    legID=3;
//                    ROS_INFO("3: c[3]=%f, %f, %f", leg[0], leg[1], leg[2]);

//                    error_back = Set_One_Leg_Positon(legID, leg);
//                    if (error_back == 0)
//                        Send_One_Leg_Positon(legID, (uint16_t) (playtime/2));  //时间长点，没停就紧接着下一步
//                    else
//                    {
//                        ROS_ERROR("腿%d 抬 解算出错。",legID);
//                        stopFlag = true;
//                    }
//                }
//                else if ( ii == floor(4.5*nstep) )   //落
//                {
//                    leg[2]=leg[2] - stepHeight;
//                    error_back = Set_One_Leg_Positon(legID, leg);
//                    if (error_back == 0)
//                        Send_One_Leg_Positon(legID, (uint16_t) (playtime/2));  //时间长点，没停就紧接着下一步
//                    else
//                    {
//                        ROS_ERROR("腿%d 落 解算出错。",legID);
//                        stopFlag = true;
//                    }
//                }

//                //腿4进
//                else if ( ii == 5*nstep )
//                {
//                    leg[0]=d[0]+ stepDistance;
//                    leg[1]=d[1];
//                    leg[2]=d[2]+ stepHeight;
//                    legID=4;
//                    ROS_INFO("4: d[3]=%f, %f, %f", leg[0], leg[1], leg[2]);

//                    error_back = Set_One_Leg_Positon(legID, leg);
//                    if (error_back == 0)
//                        Send_One_Leg_Positon(legID, (uint16_t) (playtime/2));  //时间长点，没停就紧接着下一步
//                    else
//                    {
//                        ROS_ERROR("腿%d 抬 解算出错。",legID);
//                        stopFlag = true;
//                    }
//                }
//                else if ( ii == floor(5.5*nstep) )   //落
//                {
//                    leg[2]=leg[2] - stepHeight;
//                    error_back = Set_One_Leg_Positon(legID, leg);
//                    if (error_back == 0)
//                        Send_One_Leg_Positon(legID, (uint16_t) (playtime/2));  //时间长点，没停就紧接着下一步
//                    else
//                    {
//                        ROS_ERROR("腿%d 落 解算出错。",legID);
//                        stopFlag = true;
//                    }
//                }

//            }

//        }

//        ii++;
//        ROS_INFO("ii=%d",ii);

//        if (ii == ticks)
//        {
//            ROS_INFO("一次步进周期结束。ii=%d",ii);
//            ii=0;

//            ROS_INFO("end: g_v=%f, %f, %f", g_v[0], g_v[1], g_v[2]);
//            ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
//            ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
//            ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
//            ROS_INFO("g_foot3 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);
//            g_v[0]=0; g_v[1]=0;
////                g_foot1[0]=g_foot1[0] -stepDistance;
////                g_foot2[0]=g_foot2[0] -stepDistance;

//            ROS_INFO("after: g_v=%f, %f, %f", g_v[0], g_v[1], g_v[2]);
//            ROS_INFO("g_foot1 = %f,%f,%f",g_foot1[0], g_foot1[1],g_foot1[2]);
//            ROS_INFO("g_foot2 = %f,%f,%f",g_foot2[0], g_foot2[1],g_foot2[2]);
////            ROS_INFO("g_foot3 = %f,%f,%f",g_foot3[0], g_foot3[1],g_foot3[2]);
//            ROS_INFO("g_foot3 = %f,%f,%f",g_foot4[0], g_foot4[1],g_foot4[2]);

//            if(locomotionMode == xleg_msgs::gaitControl::Request::MOVEONCE)
//                stopFlag = true;
//        }
//    }



//}
