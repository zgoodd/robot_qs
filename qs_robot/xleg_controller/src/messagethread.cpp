#include <QDebug>
#include <QMutexLocker>
#include <QTableWidget>

#include "mainwindow.h"
//#include "ui_mainwindow.h"

#include "messagethread.h"

messageThread::messageThread(QObject *parent) :
    QThread(parent)//初始化了父类'QThread'的构造函数
{
    stop = false;

    pNode  =new ros::NodeHandle; //新节点
    footForceSub = pNode->subscribe<xleg_msgs::FootsForce>("xleg/footForce", 1, &messageThread::footForceCallback, this);
    jointStateSub = pNode->subscribe<xleg_msgs::JointsState>("xleg/jointState", 1, &messageThread::jointStateCallback, this);
    bodyStateSub = pNode->subscribe<xleg_msgs::BodyState>("xleg/bodyState", 1, &messageThread::bodyStateCallback, this);
//订阅了三个主题后面是回调函数

}
void messageThread::SetStopFlg(bool flg)
{
    QMutexLocker locker(&qm);
    stop=flg;
}
//设置'stop'变量的值
void messageThread::run()
{
//    QMutexLocker locker(&qm);

    ros::spin();

        stop = false;

}
//调用'start'启动线程时被执行

void messageThread::footForceCallback(const xleg_msgs::FootsForce::ConstPtr& ff)
{
//    ROS_INFO("messageThread::footForceCallback RUN");
    emit footForceRecv(ff);
}

void messageThread::jointStateCallback(const xleg_msgs::JointsState::ConstPtr& js)
{
    emit jointStateRecv(js);
}
void messageThread::bodyStateCallback(const xleg_msgs::BodyState::ConstPtr& bs)
{
    emit bodyStateRecv(bs);
}

//回调函数，它们在收到新的消息时被ROS调用。在函数中，它们分别发出一个信号，这个信号可以被同一线程中的其他对象捕获并做出响应。
//管理在ROS网络上接收的消息，并同步到Qt线程，让这些消息可以被Qt应用程序的其他部分使用