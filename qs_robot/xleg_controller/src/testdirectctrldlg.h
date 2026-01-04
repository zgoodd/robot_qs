#ifndef TESTDIRECTCTRLDLG_H
#define TESTDIRECTCTRLDLG_H

#include <QWidget>
#include "ros/ros.h"
#include "xleg_msgs/servoControl.h"
#include "xleg_msgs/motorCode.h"


//腰关节-髋关节-膝关节 角度区间
#define ANGLE1_MAX  260
#define ANGLE1_MIN  95
#define ANGLE2_MAX  260
#define ANGLE2_MIN  94
#define ANGLE3_MAX  280
#define ANGLE3_MIN  20


namespace Ui {
class testdirectCtrlDlg;
}

class testdirectCtrlDlg : public QWidget
{
    Q_OBJECT

public:
    explicit testdirectCtrlDlg(int argc, char **argv, QWidget *parent = 0);
    ~testdirectCtrlDlg();


private slots:
    void on_Btn_clearErr_clicked();

private slots:
    void on_Check_playtime_clicked();
    void on_Check_speed_clicked();
    void on_Btn_torqueOff_clicked();
    void on_Btn_torqueOn_clicked();
    void on_Btn_runOnce_clicked();
    void on_Btn_getReady_clicked();
    void on_Btn_run_clicked();

    void will_to_position();
    void need_to_joints();

    float* positionToJoints(float px,  float py,  float pz);
    float* jointsToPosition(float a1, float a2, float a3);

private:
    Ui::testdirectCtrlDlg *ui;

    // ROS part
    ros::NodeHandle* p2Node;

    // 创建client
    xleg_msgs::servoControl servoSrv;
    ros::ServiceClient servoclient;

    xleg_msgs::motorCode motorSrv;
    ros::ServiceClient motorclient;


};

#endif // TESTDIRECTCTRLDLG_H
