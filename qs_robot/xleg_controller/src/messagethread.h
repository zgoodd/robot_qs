#ifndef MESSAGETHREAD_H
#define MESSAGETHREAD_H

#include <QThread>
#include <QMutex>

#include "ros/ros.h"
#include "xleg_msgs/FootsForce.h"
#include "xleg_msgs/JointsState.h"
#include "xleg_msgs/BodyState.h"

class messageThread : public QThread
{
    Q_OBJECT
private:
    QMutex qm;
    bool stop;

public:
    explicit messageThread(QObject *parent = 0);
    void run();
    void SetStopFlg(bool flg);

signals:
    void footForceRecv(xleg_msgs::FootsForceConstPtr);
    void jointStateRecv(xleg_msgs::JointsStateConstPtr);
    void bodyStateRecv(xleg_msgs::BodyStateConstPtr);


public slots:

    // ROS part
private:
    ros::NodeHandle* pNode;

    ros::Subscriber footForceSub;
    ros::Subscriber imuSub;
    ros::Subscriber jointStateSub;
    ros::Subscriber bodyStateSub;


private:
    void footForceCallback(const xleg_msgs::FootsForce::ConstPtr& msg);
    void jointStateCallback(const xleg_msgs::JointsState::ConstPtr& js);
    void bodyStateCallback(const xleg_msgs::BodyState::ConstPtr& bs);

};

#endif // MESSAGETHREAD_H
