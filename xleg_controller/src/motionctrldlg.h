#ifndef MOTIONCTRLDLG_H
#define MOTIONCTRLDLG_H

#include <QWidget>
#include "ros/ros.h"
#include "xleg_msgs/servoControl.h"
#include "xleg_msgs/bodyControl.h"
#include "xleg_msgs/gaitControl.h"
// #include "xleg_msgs/motorCode.h"
#include "xleg_msgs/dataSave.h"

#include <QDialog>

//机构本体长宽。 L：X方向尺寸， B：Y方向尺寸
#define CENTER_L    356.0
#define CENTER_B    320.0

namespace Ui {
class motionCtrlDlg;
}

class motionCtrlDlg : public QWidget
{
    Q_OBJECT

public:
    explicit motionCtrlDlg(int argc, char **argv, QWidget *parent = 0);
    ~motionCtrlDlg();

private slots:
    void on_checkBox_bizhang_clicked();

private slots:
    void on_Btn_moveforward_clicked();
    void on_checkBox_noGravity_stateChanged(int arg1);
    void on_tabWidget_currentChanged(int index);

    void on_checkBox_saveData_clicked();
    void on_Btn_show_stop_clicked();
    void on_Btn_show_move_clicked();

    void on_checkBox_slowPos_clicked();
    void on_checkBox_directPos_clicked();

    //on off
    void on_pushButton_torqueOff_clicked();
    void on_pushButton_clearErr_clicked();
    void on_pushButton_torqueOn_clicked();

    //ready
    void on_pushButton_posReady_clicked();
    void on_pushButton_posFinish_clicked();
    void on_pushButton_initPos_clicked();
    void on_pushButton_directInitPos_clicked();
    
    //3+1
    void on_radioButton_trans_forward_toggled(bool checked);
    void on_radioButton_trans_left_toggled(bool checked);
    void on_radioButton_trans_right_toggled(bool checked);
    void on_radioButton_trans_backward_toggled(bool checked);
    void on_radioButton_trans_customDir_toggled(bool checked);
    void on_pushButton_trans_moveOnce_clicked();
    void on_pushButton_trans_stop_clicked();
    void on_pushButton_trans_move_clicked();

    void on_comboBox_commonPos_currentIndexChanged(const QString &arg1);


private:
    Ui::motionCtrlDlg *ui;

    // ROS part
    ros::NodeHandle* pNode;

    // 创建client
    ros::ServiceClient servoclient;
    ros::ServiceClient bodyclient;
    ros::ServiceClient gaitclient;
    ros::ServiceClient savedataclient;

    //创建消息
    xleg_msgs::servoControl servoSrv;
    xleg_msgs::bodyControl bodySrv;
    xleg_msgs::gaitControl gaitSrv;
    xleg_msgs::dataSave dataSrv;

};

#endif // MOTIONCTRLDLG_H
