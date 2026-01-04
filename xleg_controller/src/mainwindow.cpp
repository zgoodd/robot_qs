#include <QDebug>
#include <QMetaType>
#include <QColor>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "motionctrldlg.h"
#include "testdirectctrldlg.h"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)://ROS的消息处理和Qt的UI处理结合起来，ROS的消息通过Qt的信号和槽机制被转发到UI线程进行处理
    QMainWindow(parent),
    ui(new Ui::MainWindow) //调用父类QMainWindow的构造函数并初始化UI的诸多属性
    // qnode(argc,argv)
{
    ui->setupUi(this);
////初始化UI，并建立一个新的线程pMythread，该线程用于接收从ROS设备发送来的不同消息。


//    ros::init(argc, argv, "xlegContorl");
//    pNode  =new ros::NodeHandle;   //先后 不能访问主机

    pMythread =  new messageThread();//启动一个messageThread类型的线程，用于接收ROS消息
    pMythread->start();

    qRegisterMetaType< xleg_msgs::FootsForceConstPtr >("xleg_msgs::FootsForceConstPtr");
    //注册xleg_msgs::FootsForceConstPtr类型到Qt元类型系统中，从而可以使用Qt的信号和槽机制来传递这种类型的对象。
    qRegisterMetaType< xleg_msgs::JointsStateConstPtr >("xleg_msgs::JointsStateConstPtr");
    qRegisterMetaType< xleg_msgs::BodyStateConstPtr >("xleg_msgs::BodyStateConstPtr");
//qRegisterMetaType允许将自定义类型与Qt信号槽系统无缝集成，以便在信号槽传递过程中自动进行类型转换和传递
//为了确保这些自定义的ROS消息类型可以被Qt的信号/插槽机制正确地使用


    connect(pMythread,SIGNAL(footForceRecv(xleg_msgs::FootsForceConstPtr)),this,SLOT(handleFootForceChange(xleg_msgs::FootsForceConstPtr)));
    //pMyThread对象，后对象里的信号
    //当pMyThread对象从ROS消息回调接收到类型为xleg_msgs::FootsForceConstPtr的消息，就触发footForceRecv信号，然后调用槽handleFootForceChange进行处理
    connect(pMythread,SIGNAL(jointStateRecv(xleg_msgs::JointsStateConstPtr)),this,SLOT(handleJointStateChange(xleg_msgs::JointsStateConstPtr)));
    connect(pMythread,SIGNAL(bodyStateRecv(xleg_msgs::BodyStateConstPtr)),this,SLOT(handleBodyStateChange(xleg_msgs::BodyStateConstPtr)));
   //信号和槽连接在一起，这样，当一个信号被触发时，关联的槽就会被调用
   //在不同的ROS消息回调接收到对应的消息时，连接到对应的处理槽函数里去
   
    //在线状态
//    QTableWidgetItem *pItem;
//    pItem = new QTableWidgetItem();
//    pItem->setBackgroundColor(QColor(255,153,153));
//    for(int i=0;i<12;i++)
//    {
//        ui->tableWidget_jointState->setItem(0,i,pItem);
//    }

}

MainWindow::~MainWindow()
{
    delete ui;
}


// ===================================  Main ====================================================================
int main(int argc, char **argv)
{
    /*********************
    ** Qt
    **********************/
    // ROS part
    ros::init(argc, argv, "xlegContorl");


    QApplication app(argc, argv);
    MainWindow w(argc,argv);
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    return result;
}



// 打开控制窗口   ================================================================
void MainWindow::on_actionControl_triggered()
{
    motionCtrlDlg *motionCtrl = new motionCtrlDlg(m_argc, m_argv);
    motionCtrl->show();
    ROS_INFO("机器人运动控制窗口已打开.");
}

void MainWindow::on_testdirectControl_triggered()
{
    testdirectCtrlDlg *testdirectCtrl = new testdirectCtrlDlg(t_argc, t_argv);
    testdirectCtrl->show();
    ROS_INFO("机器人关节舵机调试串口已打开.");
}

void MainWindow::on_cameraControl_triggered()
{

//    system("rosrun image_view image_view image:=/cameraImage &");   //调用其他节点
    system("rosrun xleg_controller process_cam &");
    ROS_INFO("相机已打开.");

}



void MainWindow::on_BtnConnectRobot_clicked()
{
    ROS_INFO("ROS_INFO");
    ROS_WARN("ROS_WARN");
    ROS_ERROR("ROS_ERROR");
    ROS_FATAL("ROS_FATAL");
//打印
    //从表格取数  都是字符
    QString str = ui->tableWidget_jointState->item(1,1)->text();//(1,1)位置
    float num = str.toFloat();//转成int
    ROS_INFO("JointState num= %f", num);//数据类型转换

    //向表格写数
    QTableWidgetItem *pItem;
    str= str.setNum(5.0, 'f',2);
    //浮点数5.0转换为QString类型,并保留两位小数,在GUI中显示数字的情况
    QString str2 = str + "%";

    pItem = new QTableWidgetItem(str2);创建一个新的表格
    ui->tableWidget_massCenter->setItem(0,0,pItem);//将新创建的表格项pItem设置到表格的第一行第一列

    ui->tableWidget_massCenter->setItem(0, 1, new QTableWidgetItem( QString::number(3.4,'f',2) + '%'));
}同上


// 订阅到下位机发布的 足底力信息 ，输出在表格控件中  ================================================================
void MainWindow::handleFootForceChange(xleg_msgs::FootsForceConstPtr ff)
{
    QString str;
    QTableWidgetItem *pItem;

    for(int i=0;i<4;i++)
    {
        str= str.setNum((double)(ff->foots_force[i].X),'f',1);
        pItem = new QTableWidgetItem(str);
        ui->tableWidget_footForce->setItem(0,i*3,pItem);

        str= str.setNum((double)(ff->foots_force[i].Y),'f',1);
        pItem = new QTableWidgetItem(str);
        ui->tableWidget_footForce->setItem(0,i*3+1,pItem);

        str= str.setNum((double)(ff->foots_force[i].Z),'f',1);
        pItem = new QTableWidgetItem(str);
        ui->tableWidget_footForce->setItem(0,i*3+2,pItem);

        pItem = new QTableWidgetItem();
        if(ff->foots_force[i].isLanded == true)
        {
            pItem->setText(tr("触地"));
            pItem->setCheckState(Qt::Checked);
        }
        else
        {
            pItem->setText(tr("否"));
            pItem->setCheckState(Qt::Unchecked);
        }
        ui->tableWidget_isLanded->setItem(0,i,pItem);
    }

    // 质心发布在表格中
    str= str.setNum((double)(ff->massCenterX),'f',1);
    pItem = new QTableWidgetItem(str);
    ui->tableWidget_massCenter->setItem(0,0,pItem);

    str= str.setNum((double)(ff->massCenterY),'f',1);
    pItem = new QTableWidgetItem(str);
    ui->tableWidget_massCenter->setItem(0,1,pItem);

    str= str.setNum((double)(ff->massCenterZ),'f',1);
    pItem = new QTableWidgetItem(str);
    ui->tableWidget_massCenter->setItem(0,2,pItem);

}

// 订阅到下位机发布的 各关节舵机信息 ，输出在表格控件中  ======================================================
void MainWindow::handleJointStateChange(xleg_msgs::JointsStateConstPtr js)
{
    QString str;
    QTableWidgetItem *pItem;

    for(int i=0;i<12;i++)
    {
        //角度信息
        str= str.setNum((double)(js->joints_state[i].present_angle),'f',1);
        pItem = new QTableWidgetItem(str);
        ui->tableWidget_jointState->setItem(1,i,pItem);  //第二行

//        //是否在moving？
//        pItem = new QTableWidgetItem();
//        if ((int)js->joints_state[i].ismoving ==1 )
//            pItem->setBackgroundColor(QColor(102,102,255));  //浅蓝
//        ui->tableWidget_jointState->setItem(3,i,pItem);

        //温度
        str= str.setNum((int)(js->joints_state[i].present_temperature)) ;  //  + "%"
        pItem = new QTableWidgetItem(str);
        ui->tableWidget_jointState->setItem(3,i,pItem);   //第4行

        //在线状态
        pItem = new QTableWidgetItem();
        pItem->setBackgroundColor(QColor(0,255,0));   //绿色
        switch((int)js->joints_state[i].error )
        {
        case 1:
            pItem->setBackgroundColor(QColor(255,153,153));  //红 RGB(255,0,0)
            break;
        case 2:
            pItem->setBackgroundColor(QColor(255,77,77));  //红 RGB(255,0,0)
            break;
        case 3:
            pItem->setBackgroundColor(QColor(255,0,0));
            break;
        }
        ui->tableWidget_jointState->setItem(0,i,pItem);   //第1行

        //Load  -> current
        str= str.setNum((float)(js->joints_state[i].present_current), 'f',2 ) ;  //  + "%"   ,'f',1
        pItem = new QTableWidgetItem(str);
        ui->tableWidget_jointState->setItem(2,i,pItem);   //第3行


        //足端相对位置
        str= str.setNum((double)(js->foot_reposition[i]),'f',0);
        pItem = new QTableWidgetItem(str);
        ui->tableWidget_footForce->setItem(1,i,pItem);

    }


}

void MainWindow::handleBodyStateChange(xleg_msgs::BodyStateConstPtr bs)
{

    QString str;
    QTableWidgetItem *pItem;

    str= str.setNum((double)(bs->roll),'f',1);
    pItem = new QTableWidgetItem(str);
    ui->tableWidget_bodyState->setItem(0,0,pItem);

    str= str.setNum((double)(bs->pitch),'f',1);
    pItem = new QTableWidgetItem(str);
    ui->tableWidget_bodyState->setItem(0,1,pItem);

    str= str.setNum((double)(bs->yaw),'f',1);
    pItem = new QTableWidgetItem(str);
    ui->tableWidget_bodyState->setItem(0,2,pItem);
}

//当机器人的roll、pitch和yaw的状态发生改变时，就会自动在指定的tableWidget里面更新这些状态的显示



