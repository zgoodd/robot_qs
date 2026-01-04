#include <QtCore>
#include <QMessageBox>

#include "motionctrldlg.h"
#include "ui_motionctrldlg.h"

#include <QDebug>
#include <iostream>
#include <QMessageBox>
#include <serial/serial.h>
#include <ros/ros.h>
#include <QApplication>
#include <QTimer>
#include <QColor>
//点击gui按钮或选怎下拉框的调用服务与ros

motionCtrlDlg::motionCtrlDlg(int argc, char **argv, QWidget *parent) :
    QWidget(parent),//构造函数
    ui(new Ui::motionCtrlDlg)//初始化了一个用户界面UI对象，接着调用 ui->setupUi(this) 来设置UI组件
{
    ui->setupUi(this);

    // 初始化部分变量
    bodySrv.request.gaitType = bodySrv.request.GAIT_USER_DEFINED;//将步态类型定义为用户定义的步态类型

    // 初始化部分控件状态,设置GUI的初始状态
    ui->radioButton_trans_forward->setChecked(true);
    //设置"trans_forward"这个radio button在用户打开应用时应该被选中。这意味着初始状态下，所选择的移动方式是"trans_forward
    ui->radioButton_trot_forward->setChecked(true);
    ui->lineEdit_trans_movedirection->setEnabled(false);
    ui->lineEdit_trot_movedirection->setEnabled(false);
    //禁用，常用来控制输入框在某些条件下用户无法输入
//    ui->lineEdit_direction->setToolTip(QStringLiteral("正前方为0，逆时针(左转)为 +，顺时针(右转)为 - 。"));

    pNode = new ros::NodeHandle;
    //ros节点的主访问点，它用于启动或关闭节点，对话题，服务和参数进行操作。在ROS中，需要至少一个NodeHandle来与ROS系统进行通信
    servoclient = pNode->serviceClient<xleg_msgs::servoControl>("xleg/servoControl");
   // 创建了一个名为"xleg/servoControl"的服务客户端，这个客户端发送的服务类型为xleg_msgs::servoControl
    bodyclient = pNode->serviceClient<xleg_msgs::bodyControl>("xleg/bodyControl");
    gaitclient = pNode->serviceClient<xleg_msgs::gaitControl>("xleg/gaitControl");
    savedataclient = pNode->serviceClient<xleg_msgs::dataSave>("xleg/dataSave");

    //slot
    // connect( this->ui->pushButton_torqueOn,  SIGNAL(clicked(bool)), this, SLOT(motorsTorqueOn()) );

}

motionCtrlDlg::~motionCtrlDlg()//释放对象占用的内存
{
    delete ui;
}




// ===========  加载力矩    ========================================================================================
void motionCtrlDlg::on_pushButton_torqueOn_clicked()
//ROS中的回调函数
//按钮被点击时，它会发送一个请求开启所有伺服的力矩的服务
{
        ROS_INFO("加载力矩。");//ROS的日志功能，当按钮被点击时，它会在终端打印出 "加载力矩。" 的信息
    servoSrv.request.allServos =true;
    //这个服务请求将应用到所有伺服上
    servoSrv.request.servoCmd = servoSrv.request.TORQUE_ON;   // CMD=1
//这个服务请求的命令是开启TORQUE
    if(! servoclient.call(servoSrv))//检查服务是否成功被调用
    {
        ROS_ERROR("call service:servoControl -Torque on- failed!");
    }
}

// ===========  释放力矩    ========================================================================================
void motionCtrlDlg::on_pushButton_torqueOff_clicked()
{
         ROS_INFO("释放力矩。");
    servoSrv.request.allServos =true;
    servoSrv.request.servoCmd = servoSrv.request.TORQUE_OFF;  // CMD=0

    if(! servoclient.call(servoSrv))
    {
        ROS_ERROR("call service:servoControl -Torque off- failed!");
    }
}

// ===========  清除错误 - reboot错误的舵机    ========================================================================================
void motionCtrlDlg::on_pushButton_clearErr_clicked()
{
//    servoSrv.request.allServos = false;
//    servoSrv.request.servoCmd = servoSrv.request.CLEAR_ERROR;  // CMD=2
//    servoSrv.request.servoID = ui->Lin_errID->text().toInt();  //对指定ID舵机重启

//    if(! servoclient.call(servoSrv))
//    {
//        ROS_ERROR("call service:servoControl -Reboot Motor- failed!");
//    }
}




// ===========  机器人就绪    ======================================================================================
void motionCtrlDlg::on_pushButton_posReady_clicked()
{
    int waittime_tmp = ui->lineEdit_waittime->text().toInt();
    int playtime_tmp = ui->lineEdit_playtime->text().toInt();

    //    int waittime_tmp = 1200;
    //    int playtime_tmp = 1200;

    //方法一
//    bodySrv.request.waittime = 0;
//    bodySrv.request.initPosMode = bodySrv.request.DIRECT;
//    bodySrv.request.x = 0;
//    bodySrv.request.y = 0;
//    bodySrv.request.z = -10;
//    bodySrv.request.Wx = 0;
//    bodySrv.request.Wy = 0;
//    bodySrv.request.Wz = 0;
//    bodySrv.request.footholdDistanceX = 0;
//    bodySrv.request.footholdDistanceY = 220;
//    bodySrv.request.playtime =playtime_tmp;
//    bodySrv.request.gaitType = bodySrv.request.GAIT_USER_DEFINED;  //0

//    if(! bodyclient.call(bodySrv))
//    {
//        ROS_ERROR("call service: bodyControl failed!");
//    }
//    else
//    {
//        if((int)bodySrv.response.error_codes != 0)
//        QMessageBox::critical(this,tr("初始位置\n"),tr("参数错误！       \nError code:%1").arg((int)bodySrv.response.error_codes),QMessageBox::Ok);
//    }

//    bodySrv.request.waittime = waittime_tmp;
//    bodySrv.request.z = 100;
//    bodySrv.request.footholdDistanceY = 220;
//    if(! bodyclient.call(bodySrv))
//    {
//        ROS_ERROR("call service: bodyControl failed!");
//    }
//    else
//    {
//        if((int)bodySrv.response.error_codes != 0)
//        QMessageBox::critical(this,tr("初始位置\n"),tr("参数错误！       \nError code:%1").arg((int)bodySrv.response.error_codes),QMessageBox::Ok);
//    }

//    bodySrv.request.waittime = waittime_tmp;
//    bodySrv.request.playtime = playtime_tmp;
//    bodySrv.request.initPosMode = bodySrv.request.INDIRECT;
//    bodySrv.request.footholdDistanceY = 150;
//    if(! bodyclient.call(bodySrv))
//    {
//        ROS_ERROR("call service: bodyControl failed!");
//    }
//    else
//    {
//        ROS_INFO("call service: bodyControl succeed.");
//        if((int)bodySrv.response.error_codes != 0)
//        QMessageBox::critical(this,tr("初始位置\n"),tr("参数错误！       \nError code:%1").arg((int)bodySrv.response.error_codes),QMessageBox::Ok);
//    }





//另一种方法
    bodySrv.request.waittime = waittime_tmp;
    bodySrv.request.playtime = playtime_tmp;

    QString arg1 = ui->comboBox_commonPos->currentText();
    //根据UI中的comboBox_commonPos控件的文本，设定步态类型（gaitType）

    bodySrv.request.footholdDistanceX = ui->lineEdit_initFootDisX->text().toDouble();
    bodySrv.request.footholdDistanceY = ui->lineEdit_initFootDisY->text().toDouble();
    bodySrv.request.z = ui->lineEdit_initZ->text().toFloat();
//从后控件中获取相应的参数值或设置到前

    if( (arg1.compare(tr("全方位对角")) == 0) || (arg1.compare(tr("快速对角")) == 0) )
    {
        bodySrv.request.gaitType = bodySrv.request.GAIT_TROT;//步态类型
        gaitSrv.request.gaitType = gaitSrv.request.GAIT_TROT;
    }
    else
    {
        bodySrv.request.gaitType = bodySrv.request.GAIT_TRANS;
        gaitSrv.request.gaitType = gaitSrv.request.GAIT_TRANS;
    }

    bodySrv.request.get_robotReady = 1;//尝试调用名为bodyControl的服务
    if(! bodyclient.call(bodySrv))
    {
        ROS_ERROR("call service: bodyControl failed!");
    }
    else
    {
        ROS_INFO("call service: bodyControl succeed.");
        if((int)bodySrv.response.error_codes != 0)
            QMessageBox::critical(this,tr("初始位置\n"),tr("参数错误！       \nError code:%1").arg((int)bodySrv.response.error_codes),QMessageBox::Ok);
    }
    bodySrv.request.get_robotReady = 0;
}

// ===========  机器人结束    ==================================================================================
void motionCtrlDlg::on_pushButton_posFinish_clicked()
//根据用户在界面上输入和选择的步态类型以及footholdDistanceX，footholdDistanceY参数，
//向bodyControl的服务发送请求，改变机器人的当前状态或动作
{
    int waittime_tmp = ui->lineEdit_waittime->text().toInt();
    int playtime_tmp = ui->lineEdit_playtime->text().toInt();

//    int waittime_tmp = 1200;
//    int playtime_tmp = 1200;

//    bodySrv.request.waittime = 0;
//    bodySrv.request.initPosMode = bodySrv.request.DIRECT;
//    bodySrv.request.x = 0;
//    bodySrv.request.y = 0;
//    bodySrv.request.z = 35;
//    bodySrv.request.Wx = 0;
//    bodySrv.request.Wy = 0;
//    bodySrv.request.Wz = 0;
//    bodySrv.request.footholdDistanceX = 0;
//    bodySrv.request.footholdDistanceY = 150;
//    bodySrv.request.playtime = playtime_tmp;
//    bodySrv.request.gaitType = bodySrv.request.GAIT_USER_DEFINED;

//    if(! bodyclient.call(bodySrv))
//    {
//        qDebug()<<("call service: bodyControl failed!");
//    }
//    else
//    {
//        if((int)bodySrv.response.error_codes != 0)
//        QMessageBox::critical(this,tr("初始位置\n"),tr("参数错误！       \nError code:%1").arg((int)bodySrv.response.error_codes),QMessageBox::Ok);
//    }
////    usleep(1200*1000);

//    bodySrv.request.waittime = waittime_tmp;
//    bodySrv.request.playtime = playtime_tmp;
//    bodySrv.request.initPosMode = bodySrv.request.DIRECT;
//    bodySrv.request.footholdDistanceY = 220;
//    bodySrv.request.z = -10;





    //另一种方法
    bodySrv.request.waittime = waittime_tmp;
    bodySrv.request.playtime = playtime_tmp;

    QString arg1 = ui->comboBox_commonPos->currentText();

    bodySrv.request.footholdDistanceX = ui->lineEdit_initFootDisX->text().toDouble();
    bodySrv.request.footholdDistanceY = ui->lineEdit_initFootDisY->text().toDouble();

    if( (arg1.compare(tr("全方位对角")) == 0) || (arg1.compare(tr("快速对角")) == 0) )
    {
        bodySrv.request.gaitType = bodySrv.request.GAIT_TROT;
        gaitSrv.request.gaitType = gaitSrv.request.GAIT_TROT;
    }
    else
    {
        bodySrv.request.gaitType = bodySrv.request.GAIT_TRANS;
        gaitSrv.request.gaitType = gaitSrv.request.GAIT_TRANS;
    }


    bodySrv.request.get_robotReady = 2;
    if(! bodyclient.call(bodySrv))
    {
        ROS_ERROR("call service: bodyControl failed!");
    }
    else
    {
        ROS_INFO("call service: bodyControl succeed.");
        if((int)bodySrv.response.error_codes != 0)
        QMessageBox::critical(this,tr("初始位置\n"),tr("参数错误！       \nError code:%1").arg((int)bodySrv.response.error_codes),QMessageBox::Ok);
    }
    bodySrv.request.get_robotReady = 0;
}




// ===========  设置初始姿态   =====================================================================================
//当按钮被点击，这个函数将执行上述操作以初始化一些设定值。
void motionCtrlDlg::on_checkBox_directPos_clicked()
//on_checkBox_directPos_clicked()函数在checkBox_directPos被点击时被执行，
//并且会去切换（翻转）checkBox_slowPos的选中状态
{
    ui->checkBox_slowPos->toggle();
}

void motionCtrlDlg::on_checkBox_slowPos_clicked()
{
    ui->checkBox_directPos->toggle();
}
//勾选了一项，那么另一项自动取消勾选

// ===========  点击GO  直接设置初始状态
void motionCtrlDlg::on_pushButton_directInitPos_clicked()
{
    //隐藏了输入按钮
}

// ===========  移到初始状态
void motionCtrlDlg::on_pushButton_initPos_clicked()
{
    int playtime_tmp = ui->lineEdit_playtime->text().toInt();
        //    int playtime_tmp = 1200;

     if (ui->checkBox_directPos->isChecked())//检查checkBox_directPos的复选框是否被选中
     //来决定位置模式initPosMode和播放时间playtime
     {
         bodySrv.request.initPosMode = bodySrv.request.DIRECT;
         bodySrv.request.playtime =playtime_tmp;
     }
     else
     {
         bodySrv.request.initPosMode = bodySrv.request.INDIRECT;
         bodySrv.request.playtime =playtime_tmp;
         bodySrv.request.stepHeight = ui->LineEdit_trans_stepH->text().toInt();
         bodySrv.request.stepDistance = ui->LineEdit_trans_stepDis->text().toInt();
     }

     QString arg1 = ui->comboBox_commonPos->currentText();
     //从comboBox_commonPos的下拉菜单中获取当前选择的文本，并根据这个文本设定步态类型gaitType
     if(arg1.compare(tr("全方位3+1")) == 0)
     {
         bodySrv.request.gaitType = bodySrv.request.GAIT_TRANS;
         gaitSrv.request.gaitType = gaitSrv.request.GAIT_TRANS;//是什么
     }
     else if(arg1.compare(tr("快速3+1(X)")) == 0)
     {
         bodySrv.request.gaitType = bodySrv.request.GAIT_TRANS_FAST;
         gaitSrv.request.gaitType = gaitSrv.request.GAIT_TRANS_FAST;
     }
     else if(arg1.compare(tr("全方位对角")) == 0)
     {
         bodySrv.request.gaitType = bodySrv.request.GAIT_TROT;
         gaitSrv.request.gaitType = gaitSrv.request.GAIT_TROT;
     }
     else if(arg1.compare(tr("快速对角")) == 0)
     {
         bodySrv.request.gaitType = bodySrv.request.GAIT_TROT_FAST;
         gaitSrv.request.gaitType = gaitSrv.request.GAIT_TROT_FAST;
     }

    bodySrv.request.x = ui->lineEdit_initX->text().toDouble();
    bodySrv.request.y = ui->lineEdit_initY->text().toDouble();
    bodySrv.request.z = ui->lineEdit_initZ->text().toDouble();
    bodySrv.request.Wx = ui->lineEdit_initWx->text().toDouble();
    bodySrv.request.Wy = ui->lineEdit_initWy->text().toDouble();
    bodySrv.request.Wz = ui->lineEdit_initWz->text().toDouble();
    bodySrv.request.footholdDistanceX = ui->lineEdit_initFootDisX->text().toDouble();
    bodySrv.request.footholdDistanceY = ui->lineEdit_initFootDisY->text().toDouble();

//     if(ui->comboBox_commonPos->currentText().compare(tr("快速3+1(X)")) == 0)
//     {
//         QString str;
//         str.setNum(ui->lineEdit_initFootDisX->text().toInt()*3);
//         ui->LineEdit_trans_stepDis->setText(str);   //步长根据lineEdit_initFootDisX来自动确定
//     }
//     else if(ui->comboBox_commonPos->currentText().compare(tr("快速对角")) == 0)
//     {
//         QString str;
//         str.setNum(ui->lineEdit_initFootDisX->text().toInt()*3);
//         ui->LineEdit_trot_stepDis->setText(str);
//     }
//    qDebug()<< (int)bodySrv.request.z;

    if(! bodyclient.call(bodySrv))
    {
       ROS_ERROR("call service: bodyControl failed!");
    }
    else
    {
        if((int)bodySrv.response.error_codes != 0)
        QMessageBox::critical(this,tr("初始位置\n"),tr("参数错误！       \nError code:%1").arg((int)bodySrv.response.error_codes),QMessageBox::Ok);
    }
}





/**********************************************************************************************************

                                            3+1

**********************************************************************************************************/
//每一个函数在其对应的单选按钮的状态改变时会被触发执行
{
    //        gaitSrv.request.directionX = 0;
    //        gaitSrv.request.directionY = 1;

//    ui->spinBox_trans_customDirX->setValue(0);
//    ui->spinBox_trans_customDirY->setValue(1);

    ui->lineEdit_trans_movedirection->setText("0");
}

void motionCtrlDlg::on_radioButton_trans_left_toggled(bool checked)
{
    //        gaitSrv.request.directionX = -1;
    //        gaitSrv.request.directionY = 0;

//    ui->spinBox_trans_customDirX->setValue(-1);
//    ui->spinBox_trans_customDirY->setValue(0);

    ui->lineEdit_trans_movedirection->setText("90");
}

void motionCtrlDlg::on_radioButton_trans_right_toggled(bool checked)
{
    //        gaitSrv.request.directionX = 1;
    //        gaitSrv.request.directionY = 0;

//    ui->spinBox_trans_customDirX->setValue(1);
//    ui->spinBox_trans_customDirY->setValue(0);

    ui->lineEdit_trans_movedirection->setText("-90");
}

void motionCtrlDlg::on_radioButton_trans_backward_toggled(bool checked)
{
    if(checked)
    {
        //        gaitSrv.request.directionX = 0;
        //        gaitSrv.request.directionY = -1;

//        ui->spinBox_trans_customDirX->setValue(0);
//        ui->spinBox_trans_customDirY->setValue(-1);

        ui->lineEdit_trans_movedirection->setText("180");
    }
}
void motionCtrlDlg::on_radioButton_trans_customDir_toggled(bool checked)
{
    // 改变“自定义方向”输入框 的状态
    if(checked)
    {
        ui->lineEdit_trans_movedirection->setEnabled(true);

//        ui->spinBox_trans_customDirX->setEnabled(true);
//        ui->spinBox_trans_customDirY->setEnabled(true);
    }
    else
    {
        ui->lineEdit_trans_movedirection->setEnabled(false);

//        ui->spinBox_trans_customDirX->setEnabled(false);
//        ui->spinBox_trans_customDirY->setEnabled(false);
    }
}

// ===========  3+1步态  单步运动   ==============
void motionCtrlDlg::on_pushButton_trans_moveOnce_clicked()
{
    gaitSrv.request.showMOVE = 2;

   
    if(ui->comboBox_commonPos->currentText().compare(tr("快速3+1(X)")) == 0 || ui->comboBox_commonPos->currentText().compare(tr("全方位3+1")) == 0)
   // 检查组合框（combobox） comboBox_commonPos 选中的文本是否为 "快速3+1XX" 或 "全方位3+1"
    {
//        gaitSrv.request.filename = ui->lineEdit_filename->text().toStdString();

        gaitSrv.request.x = ui->lineEdit_initX->text().toDouble();
        gaitSrv.request.y = ui->lineEdit_initY->text().toDouble();
        gaitSrv.request.z = ui->lineEdit_initZ->text().toDouble();
        gaitSrv.request.Wx = ui->lineEdit_initWx->text().toDouble();
        gaitSrv.request.Wy = ui->lineEdit_initWy->text().toDouble();
        gaitSrv.request.Wz = ui->lineEdit_initWz->text().toDouble();
        gaitSrv.request.footholdDistanceX = ui->lineEdit_initFootDisX->text().toDouble();
        gaitSrv.request.footholdDistanceY = ui->lineEdit_initFootDisY->text().toDouble();

        gaitSrv.request.stepHeight = ui->LineEdit_trans_stepH->text().toInt();
        gaitSrv.request.stepDistance = ui->LineEdit_trans_stepDis->text().toInt();
//        gaitSrv.request.k = ui->LineEdit_trans_k->text().toDouble();   //稳定裕度
        gaitSrv.request.T = ui->LineEdit_trans_T->text().toInt();   // 运动周期 s
        gaitSrv.request.bodyadjustX = ui->LineEdit_trans_baX->text().toInt();   //每次抬腿重心调整 X
        gaitSrv.request.bodyadjustY = ui->LineEdit_trans_baY->text().toInt();   //每次抬腿重心调整 Y


//        gaitSrv.request.directionX = ui->spinBox_trans_customDirX->value();
//        gaitSrv.request.directionY = ui->spinBox_trans_customDirY->value();
//        gaitSrv.request.directionZ = ui->LineEdit_trans_DirZ->text().toDouble();   //旋转角度
        gaitSrv.request.move_direction = ui->lineEdit_trans_movedirection->text().toDouble();
        gaitSrv.request.body_direction = ui->lineEdit_trans_bodydirection->text().toDouble();

        gaitSrv.request.locomotionMode = gaitSrv.request.MOVEONCE;

        if(ui->checkBox_climb->checkState())//如果复选框 checkBox_climb 被选中则 gaitSrv.request.climbMode 被设置为 true，否则为 false
            gaitSrv.request.climbMode = true;
        else
            gaitSrv.request.climbMode = false;


        if(! gaitclient.call(gaitSrv))
        {
            ROS_ERROR("call service: gaitControl failed!");
        }
        else
        {
            QString str;
            double speed = ui->LineEdit_trans_stepDis->text().toDouble() / ui->LineEdit_trans_T->text().toDouble();
            ui->LineEdit_trans_speed->setText(str.setNum(speed,'f',1));
        }
    }
    else
    {
        QMessageBox::warning(this,tr("运动一次"),tr("请选择对应的步态！"),QMessageBox::Ok);
    }
}


// ===========  3+1步态  连续运动   ==============
void motionCtrlDlg::on_pushButton_trans_move_clicked()
{
    gaitSrv.request.showMOVE = 2;

    if(ui->comboBox_commonPos->currentText().compare(tr("快速3+1(X)")) == 0 || ui->comboBox_commonPos->currentText().compare(tr("全方位3+1")) == 0)
    {
//        gaitSrv.request.filename = ui->lineEdit_filename->text().toStdString();

        gaitSrv.request.x = ui->lineEdit_initX->text().toDouble();
        gaitSrv.request.y = ui->lineEdit_initY->text().toDouble();
        gaitSrv.request.z = ui->lineEdit_initZ->text().toDouble();
        gaitSrv.request.Wx = ui->lineEdit_initWx->text().toDouble();
        gaitSrv.request.Wy = ui->lineEdit_initWy->text().toDouble();
        gaitSrv.request.Wz = ui->lineEdit_initWz->text().toDouble();
        gaitSrv.request.footholdDistanceX = ui->lineEdit_initFootDisX->text().toDouble();
        gaitSrv.request.footholdDistanceY = ui->lineEdit_initFootDisY->text().toDouble();



        gaitSrv.request.stepHeight = ui->LineEdit_trans_stepH->text().toInt();
        gaitSrv.request.stepDistance = ui->LineEdit_trans_stepDis->text().toInt();
//        gaitSrv.request.k = ui->LineEdit_trans_k->text().toDouble();   //稳定裕度
        gaitSrv.request.bodyadjustX = ui->LineEdit_trans_baX->text().toInt();   //每次抬腿重心调整 X
        gaitSrv.request.bodyadjustY = ui->LineEdit_trans_baY->text().toInt();   //每次抬腿重心调整 Y
        gaitSrv.request.T = ui->LineEdit_trans_T->text().toInt();   // 运动周期 s


//        gaitSrv.request.directionX = ui->spinBox_trans_customDirX->value();
//        gaitSrv.request.directionY = ui->spinBox_trans_customDirY->value();
//        gaitSrv.request.directionZ = ui->LineEdit_trans_DirZ->text().toDouble();   //旋转角度
        gaitSrv.request.move_direction = ui->lineEdit_trans_movedirection->text().toDouble();
        gaitSrv.request.body_direction = ui->lineEdit_trans_bodydirection->text().toDouble();

        gaitSrv.request.locomotionMode = gaitSrv.request.MOVE;

        if(ui->checkBox_climb->checkState())
            gaitSrv.request.climbMode = true;
        else
            gaitSrv.request.climbMode = false;


        if(! gaitclient.call(gaitSrv))
        {
            ROS_ERROR("call service: gaitControl failed!");
        }
        else
        {
            QString str;
            double speed = ui->LineEdit_trans_stepDis->text().toDouble() / ui->LineEdit_trans_T->text().toDouble();
            ui->LineEdit_trans_speed->setText(str.setNum(speed,'f',1));
        }
    }
    else
    {
        QMessageBox::warning(this,tr("连续运动"),tr("请选择对应的步态！"),QMessageBox::Ok);
    }
}

// ===========  3+1步态  停止运动   ==============
void motionCtrlDlg::on_pushButton_trans_stop_clicked()
{
//    gaitSrv.request.showMOVE = 2;
//    gaitSrv.request.locomotionMode = gaitSrv.request.STOP;

//    if(! gaitclient.call(gaitSrv))
//    {
//        ROS_ERROR("call service: gaitControl failed!");
//    }

    ROS_INFO("机器人停止运动。");
    servoSrv.request.servoCmd = servoSrv.request.STOP_ROBOT;   // CMD=1

    if(! servoclient.call(servoSrv))
    {
        ROS_ERROR("call service:servoControl - STOP_ROBOT- failed!");
    }

    ui->LineEdit_trans_speed->clear();
}





/**********************************************************************************************************

                                         trot

**********************************************************************************************************/




void motionCtrlDlg::on_comboBox_commonPos_currentIndexChanged(const QString &arg1)
//下拉菜单有五个选项
{
    ui->LineEdit_trans_stepDis->setEnabled(true);

    ui->radioButton_trans_forward->setEnabled(true);
    ui->radioButton_trans_backward->setEnabled(true);
    ui->radioButton_trans_customDir->setEnabled(true);
    ui->radioButton_trans_forward->setChecked(true);

    ui->LineEdit_trot_stepDis->setEnabled(true);
    ui->radioButton_trot_forward->setEnabled(true);
    ui->radioButton_trot_backward->setEnabled(true);
    ui->radioButton_trot_customDir->setEnabled(true);
    ui->radioButton_trot_forward->setChecked(true);


    if(arg1.compare(tr("全方位3+1")) == 0)
    {
        ui->radioButton_trans_left->setEnabled(true);
        ui->radioButton_trans_right->setEnabled(true);
        ui->lineEdit_initX->setText("0");
        ui->lineEdit_initY->setText("0");
        ui->lineEdit_initZ->setText("120");
        ui->lineEdit_initWx->setText("0");
        ui->lineEdit_initWy->setText("0");
        ui->lineEdit_initWz->setText("0");
        ui->lineEdit_initFootDisX->setText("0"); //120
        ui->lineEdit_initFootDisY->setText("150");  //150

        bodySrv.request.gaitType = bodySrv.request.GAIT_TRANS;
        gaitSrv.request.gaitType = gaitSrv.request.GAIT_TRANS;

        ui->checkBox_climb->setEnabled(true);
    }
    else if(arg1.compare(tr("快速3+1(X)")) == 0)
    {
        ui->checkBox_slowPos->setChecked(true);
        ui->checkBox_directPos->setChecked(false);

//        ui->LineEdit_trans_stepDis->setEnabled(false);
        ui->radioButton_trans_forward->setEnabled(true);
        ui->radioButton_trans_backward->setEnabled(false);
        ui->radioButton_trans_left->setEnabled(false);
        ui->radioButton_trans_right->setEnabled(false);
        ui->radioButton_trans_customDir->setEnabled(false);
        ui->lineEdit_initX->setText("0");
        ui->lineEdit_initY->setText("0");
        ui->lineEdit_initZ->setText("120");
        ui->lineEdit_initWx->setText("0");
        ui->lineEdit_initWy->setText("0");
        ui->lineEdit_initWz->setText("0");
        ui->lineEdit_initFootDisX->setText("0");   //70
        ui->lineEdit_initFootDisY->setText("150");   //160
        bodySrv.request.gaitType = bodySrv.request.GAIT_TRANS_FAST;
        gaitSrv.request.gaitType = gaitSrv.request.GAIT_TRANS_FAST;

        ui->checkBox_climb->setEnabled(true);
    }
    else if(arg1.compare(tr("全方位对角")) == 0)
    {
        ui->lineEdit_initX->setText("0");
        ui->lineEdit_initY->setText("0");
        ui->lineEdit_initZ->setText("120");
        ui->lineEdit_initWx->setText("0");
        ui->lineEdit_initWy->setText("0");
        ui->lineEdit_initWz->setText("0");
        ui->lineEdit_initFootDisX->setText("120");    //100
        ui->lineEdit_initFootDisY->setText("150");     //100
        bodySrv.request.gaitType = bodySrv.request.GAIT_TROT;
        gaitSrv.request.gaitType = gaitSrv.request.GAIT_TROT;

        ui->checkBox_climb->setDisabled(true);
        ui->checkBox_climb->setChecked(false);
        gaitSrv.request.climbMode = false;

    }
    else if(arg1.compare(tr("快速对角")) == 0)
    {
        ui->LineEdit_trot_stepDis->setEnabled(false);
        ui->radioButton_trot_forward->setEnabled(false);
        ui->radioButton_trot_backward->setEnabled(false);
        ui->radioButton_trot_customDir->setEnabled(false);
        ui->radioButton_trot_right->setChecked(true);
        ui->lineEdit_initX->setText("0");
        ui->lineEdit_initY->setText("0");
        ui->lineEdit_initZ->setText("120");
        ui->lineEdit_initWx->setText("0");
        ui->lineEdit_initWy->setText("0");
        ui->lineEdit_initWz->setText("0");
        ui->lineEdit_initFootDisX->setText("120");   //10
        ui->lineEdit_initFootDisY->setText("150");  //160
        bodySrv.request.gaitType = bodySrv.request.GAIT_TROT_FAST;
        gaitSrv.request.gaitType = gaitSrv.request.GAIT_TROT_FAST;

        ui->checkBox_climb->setDisabled(true);
        ui->checkBox_climb->setChecked(false);
        gaitSrv.request.climbMode = false;
    }
    else if(arg1.compare(tr("自定义位置")) == 0)
    {
        ui->lineEdit_initX->setText("0");
        ui->lineEdit_initY->setText("0");
        ui->lineEdit_initZ->setText("100");
        ui->lineEdit_initWx->setText("0");
        ui->lineEdit_initWy->setText("0");
        ui->lineEdit_initWz->setText("0");
        ui->lineEdit_initFootDisX->setText("0");
        ui->lineEdit_initFootDisY->setText("150");
        bodySrv.request.gaitType = bodySrv.request.GAIT_USER_DEFINED;

        ui->checkBox_climb->setEnabled(true);
    }
}




/**********************************************************************************************************

                                         show 演示

**********************************************************************************************************/

void motionCtrlDlg::on_Btn_show_move_clicked()
{
   //捕获当前选中项的索引
    int showCode_tmp[8]={0};
    showCode_tmp[0]=ui->comboBox_show1->currentIndex();
    showCode_tmp[1]=ui->comboBox_show2->currentIndex();
    showCode_tmp[2]=ui->comboBox_show3->currentIndex();
    showCode_tmp[3]=ui->comboBox_show4->currentIndex();
    showCode_tmp[4]=ui->comboBox_show5->currentIndex();
    showCode_tmp[5]=ui->comboBox_show6->currentIndex();
    showCode_tmp[6]=ui->comboBox_show7->currentIndex();
    showCode_tmp[7]=ui->comboBox_show8->currentIndex();

    int showValue_tmp[8]={0};
    showValue_tmp[0]=ui->lineEdit_show1->text().toInt();
    showValue_tmp[1]=ui->lineEdit_show2->text().toInt();
    showValue_tmp[2]=ui->lineEdit_show3->text().toInt();
    showValue_tmp[3]=ui->lineEdit_show4->text().toInt();
    showValue_tmp[4]=ui->lineEdit_show5->text().toInt();
    showValue_tmp[5]=ui->lineEdit_show6->text().toInt();
    showValue_tmp[6]=ui->lineEdit_show7->text().toInt();
    showValue_tmp[7]=ui->lineEdit_show8->text().toInt();

    for(int i=0; i<8; i++)
    {
        gaitSrv.request.showCode[i] = showCode_tmp[i];
        gaitSrv.request.showValue[i] = showValue_tmp[i];
    }
    int effnum=0;
    for (int i=0; i<8; i++)
    {
       if (showCode_tmp[i]!=0 && abs(showValue_tmp[i])>1.5)
           effnum++;
    }
    ROS_INFO("共规划有效动作%d步，准备执行...",effnum);


    if (ui->checkBox_noGravity->isChecked())
        gaitSrv.request.isNoGravity = true;
    else
        gaitSrv.request.isNoGravity = false;

    if (ui->checkBox_climb->checkState())
        gaitSrv.request.climbMode = true;
    else
        gaitSrv.request.climbMode = false;

    gaitSrv.request.x = ui->lineEdit_initX->text().toDouble();
    gaitSrv.request.y = ui->lineEdit_initY->text().toDouble();
    gaitSrv.request.z = ui->lineEdit_initZ->text().toDouble();
    gaitSrv.request.Wx = ui->lineEdit_initWx->text().toDouble();
    gaitSrv.request.Wy = ui->lineEdit_initWy->text().toDouble();
    gaitSrv.request.Wz = ui->lineEdit_initWz->text().toDouble();
    gaitSrv.request.footholdDistanceX = ui->lineEdit_initFootDisX->text().toDouble();
    gaitSrv.request.footholdDistanceY = ui->lineEdit_initFootDisY->text().toDouble();

    gaitSrv.request.stepHeight = ui->LineEdit_show_stepH->text().toInt();
    gaitSrv.request.stepDistance = ui->LineEdit_show_stepDis->text().toInt();
    gaitSrv.request.T = ui->LineEdit_show_T->text().toInt();   // 运动周期 s
    gaitSrv.request.bodyadjustX = ui->LineEdit_show_baX->text().toInt();   //每次抬腿重心调整 X
    gaitSrv.request.bodyadjustY = ui->LineEdit_show_baY->text().toInt();   //每次抬腿重心调整 Y

    gaitSrv.request.locomotionMode = gaitSrv.request.STOP;
    gaitSrv.request.showMOVE = 1;


    if(! gaitclient.call(gaitSrv))
    {
        ROS_ERROR("call service: gaitControl failed!");
    }
}


void motionCtrlDlg::on_Btn_moveforward_clicked()
{
    ROS_INFO("前进演示，准备执行...");


    if (ui->checkBox_noGravity->isChecked())
        gaitSrv.request.isNoGravity = true;
    else
        gaitSrv.request.isNoGravity = false;

    if (ui->checkBox_climb->checkState())
        gaitSrv.request.climbMode = true;
    else
        gaitSrv.request.climbMode = false;
    if (ui->checkBox_bizhang->checkState())
    {
        gaitSrv.request.bizhangMode = true;
//        ui->radioButton_trans_forward->setChecked(true);
    }
    else
        gaitSrv.request.bizhangMode = false;

    gaitSrv.request.x = ui->lineEdit_initX->text().toDouble();
    gaitSrv.request.y = ui->lineEdit_initY->text().toDouble();
    gaitSrv.request.z = ui->lineEdit_initZ->text().toDouble();
    gaitSrv.request.Wx = ui->lineEdit_initWx->text().toDouble();
    gaitSrv.request.Wy = ui->lineEdit_initWy->text().toDouble();
    gaitSrv.request.Wz = ui->lineEdit_initWz->text().toDouble();
    gaitSrv.request.footholdDistanceX = ui->lineEdit_initFootDisX->text().toDouble();
    gaitSrv.request.footholdDistanceY = ui->lineEdit_initFootDisY->text().toDouble();

    gaitSrv.request.stepHeight = ui->LineEdit_show_stepH->text().toInt();
    gaitSrv.request.stepDistance = ui->LineEdit_show_stepDis->text().toInt();
    gaitSrv.request.T = ui->LineEdit_show_T->text().toInt();   // 运动周期 s
    gaitSrv.request.bodyadjustX = ui->LineEdit_show_baX->text().toInt();   //每次抬腿重心调整 X
    gaitSrv.request.bodyadjustY = ui->LineEdit_show_baY->text().toInt();   //每次抬腿重心调整 Y

    gaitSrv.request.locomotionMode = gaitSrv.request.STOP;
    gaitSrv.request.showMOVE = 3;


    if(! gaitclient.call(gaitSrv))
    {
        ROS_ERROR("call service: gaitControl failed!");
    }



}










void motionCtrlDlg::on_Btn_show_stop_clicked()
{
//    gaitSrv.request.showMOVE = 0;
//    gaitSrv.request.locomotionMode = gaitSrv.request.STOP;

//    if(! gaitclient.call(gaitSrv))
//    {
//        ROS_ERROR("call service: gaitControl failed!");
//    }


    ROS_INFO("机器人停止运动。");
    servoSrv.request.servoCmd = servoSrv.request.STOP_ROBOT;   // CMD=1

    if(! servoclient.call(servoSrv))
    {
        ROS_ERROR("call service:servoControl - STOP_ROBOT- failed!");
    }

}



//=====================  保存力数据 ====
void motionCtrlDlg::on_checkBox_saveData_clicked()
{
    dataSrv.request.filename = ui->lineEdit_filename->text().toStdString();

    if(ui->checkBox_saveData->checkState())
        dataSrv.request.startSaveData = true;
    else
        dataSrv.request.startSaveData = false;

    if(! savedataclient.call(dataSrv))
    {
        qDebug()<<("call service: saveData failed!");
    }
    else
    {
        if((int)dataSrv.response.error_codes == 0)
            ROS_INFO("Start save force data to TXT file.");
        else
            ROS_INFO("Stop save force data.");
    }
}



void motionCtrlDlg::on_tabWidget_currentChanged(int index)
{
    if (ui->tabWidget->currentIndex()==3)
    {
        ROS_INFO("不考虑重力，即无重心调整。");
        ui->checkBox_noGravity->setChecked(true);
//        gaitSrv.request.isNoGravity = true;
        ui->LineEdit_show_baX->setDisabled(true);
        ui->LineEdit_show_baY->setDisabled(true);
    }
    else
    {
        ROS_INFO("考虑重力，即行走过程有重心调整。");
        ui->checkBox_noGravity->setChecked(false);
//        gaitSrv.request.isNoGravity = false;
        ui->LineEdit_show_baX->setDisabled(false);
        ui->LineEdit_show_baY->setDisabled(false);
    }
}


void motionCtrlDlg::on_checkBox_noGravity_stateChanged(int arg1)
{
    if (ui->checkBox_noGravity->isChecked())
    {
//        gaitSrv.request.isNoGravity = true;
        ROS_INFO("不考虑重力。");
        ui->LineEdit_show_baX->setDisabled(true);
        ui->LineEdit_show_baY->setDisabled(true);
    }
    else
    {
//        gaitSrv.request.isNoGravity = false;
        ROS_INFO("考虑重力。");
        ui->LineEdit_show_baX->setDisabled(false);
        ui->LineEdit_show_baY->setDisabled(false);
    }
}



void motionCtrlDlg::on_checkBox_bizhang_clicked()
{
    if (ui->checkBox_bizhang->checkState())
    {
        gaitSrv.request.bizhangMode = true;
        ui->radioButton_trans_forward->setChecked(true);
        ui->radioButton_trans_left->setDisabled(true);
        ui->radioButton_trans_right->setDisabled(true);
        ui->radioButton_trans_backward->setDisabled(true);
    }
    else
    {
        gaitSrv.request.bizhangMode = false;
        ui->radioButton_trans_forward->setChecked(true);
        ui->radioButton_trans_left->setDisabled(false);
        ui->radioButton_trans_right->setDisabled(false);
        ui->radioButton_trans_backward->setDisabled(false);
    }

}
