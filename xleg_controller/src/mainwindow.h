#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
// #include <qnode.h>

#include "messagethread.h"
#include "ros/ros.h"
#include "xleg_msgs/FootsForce.h"
#include "xleg_msgs/JointsState.h"

// using namespace up_controller;

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

    Ui::MainWindow *ui;

// private slots:
//     void on_pushButtonConnect_clicked();
//     void on_pushButtonSend_clicked();
//     void on_actionControl_triggered();
//     void on_BtnConnectRobot_clicked();

// private:
//     // QNode qnode;  
//     int m_argc;
//     char** m_argv;

//     ros::NodeHandle* pNode;

signals:
private slots:
    void on_testdirectControl_triggered();
    void on_cameraControl_triggered();

private Q_SLOTS:
    void handleFootForceChange(xleg_msgs::FootsForceConstPtr ff);
    void handleJointStateChange(xleg_msgs::JointsStateConstPtr js);
    void handleBodyStateChange(xleg_msgs::BodyStateConstPtr bs);

    void on_actionControl_triggered();
    void on_BtnConnectRobot_clicked();


private:
    int m_argc;
    char** m_argv;

    ros::NodeHandle* pNode;
    messageThread *pMythread;

    int t_argc;
    char** t_argv;

};


#endif // MAINWINDOW_H
