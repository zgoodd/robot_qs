#include "testdirectctrldlg.h"
#include "ui_testdirectctrldlg.h"

#include "motionctrldlg.h"
#include "ui_motionctrldlg.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <ros/ros.h>
#include <QDialog>
#include <ctime>
#include <QDebug>
#include <QMessageBox>
#include <QColor>

// #include <xleg_lowersys/robot_function.h>

// #define M_PI  3.141592653
#define L1  54
#define L2  187
#define L3  230

testdirectCtrlDlg::testdirectCtrlDlg(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::testdirectCtrlDlg)
{
    ui->setupUi(this);

    p2Node = new ros::NodeHandle;
    servoclient = p2Node->serviceClient<xleg_msgs::servoControl>("xleg/servoControl");
    motorclient = p2Node->serviceClient<xleg_msgs::motorCode>("xleg/motorCode");

    connect(this->ui->Lin_angle11, SIGNAL(textChanged(QString)), this, SLOT( will_to_position() ));  //SIGNAL(editingFinished()
    connect(this->ui->Lin_angle11_2, SIGNAL(textChanged(QString)), this, SLOT( will_to_position() ));  // textChanged
    connect(this->ui->Lin_angle11_3, SIGNAL(textChanged(QString)), this, SLOT( will_to_position() ));

    connect(this->ui->Lin_foot_px, SIGNAL(textChanged(QString)), this, SLOT( need_to_joints() ));
    connect(this->ui->Lin_foot_py, SIGNAL(textChanged(QString)), this, SLOT( need_to_joints() ));
    connect(this->ui->Lin_foot_pz, SIGNAL(textChanged(QString)), this, SLOT( need_to_joints() ));
    will_to_position();
    need_to_joints();
}

testdirectCtrlDlg::~testdirectCtrlDlg()
{
    delete ui;
}


// ===========  加载力矩    ========================================================================================
void testdirectCtrlDlg::on_Btn_torqueOn_clicked()
{
    //    ROS_INFO("加载力矩。");
    servoSrv.request.allServos =true;
    servoSrv.request.servoCmd = servoSrv.request.TORQUE_ON;   // CMD=1

    if(! servoclient.call(servoSrv))
    {
        ROS_ERROR("call service:servoControl -Torque on- failed!");
    }
}

// ===========  释放力矩    ========================================================================================
void testdirectCtrlDlg::on_Btn_torqueOff_clicked()
{
    servoSrv.request.allServos =true;
    servoSrv.request.servoCmd = servoSrv.request.TORQUE_OFF;  // CMD=0

    if(! servoclient.call(servoSrv))
    {
        ROS_ERROR("call service:servoControl -Torque off- failed!");
    }
}

// ===========  清除错误 - reboot错误的舵机    ========================================================================================
void testdirectCtrlDlg::on_Btn_clearErr_clicked()
{
    servoSrv.request.allServos = false;
    servoSrv.request.servoCmd = servoSrv.request.CLEAR_ERROR;  // CMD=2
    servoSrv.request.servoID = ui->Lin_errID->text().toInt();  //对指定ID舵机重启

    if(! servoclient.call(servoSrv))
    {
        ROS_ERROR("call service:servoControl -Reboot Motor- failed!");
    }
}


// ===========  点击运行按钮    ========================================================================================
void testdirectCtrlDlg::on_Btn_run_clicked()
{
    bool ok;

    motorSrv.request.servoID[0] = ui->Lin_ID11->text().toInt();
    motorSrv.request.servoID[1] = ui->Lin_ID11_2->text().toInt();
    motorSrv.request.servoID[2] = ui->Lin_ID11_3->text().toInt();

    // 下位机判断是「速度控制」还是「时间控制」
    if (ui->Check_speed->isChecked())     //如果勾选了电机1
    {
        ROS_INFO("速度控制模式下达指令。");
        motorSrv.request.speedR[0] = ui->Lin_speed11->text().toFloat(&ok);     //  °/s
        motorSrv.request.speedR[1] = ui->Lin_speed11_2->text().toFloat(&ok);
        motorSrv.request.speedR[2] = ui->Lin_speed11_3->text().toFloat(&ok);

        motorSrv.request.playtime[0] = 0;
    }
    else   // 时间控制
    {
        ROS_INFO("时间控制模式下达指令。");
        motorSrv.request.playtime[0] = ui->Lin_playtime1->text().toInt();       // s
        motorSrv.request.playtime[1] = ui->Lin_playtime1_2->text().toInt();
        motorSrv.request.playtime[2] = ui->Lin_playtime1_3->text().toInt();

        motorSrv.request.speedR[0] = 0;
    }
    motorSrv.request.drive_alllegs = 0;

    // 先初始化个值
    float q[3] = { 180, 180, 180};
    q[0] = ui->Lin_angle11->text().toFloat();    // °舵机的角度（0-360du）
    q[1] = ui->Lin_angle11_2->text().toFloat();
    q[2] = ui->Lin_angle11_3->text().toFloat();

    motorSrv.request.angleR[0] = q[0];
    motorSrv.request.angleR[1] = q[1];
    motorSrv.request.angleR[2] = q[2];

    if(! motorclient.call(motorSrv))
    {
        ROS_ERROR("call service:motorCode failed!");
    }
}


// ===========  点击 就绪 按钮    ==============================================================================
void testdirectCtrlDlg::on_Btn_getReady_clicked()
{
    float px = ui->Lin_foot_px->text().toFloat();
    float py = ui->Lin_foot_py->text().toFloat();
    float pz = ui->Lin_foot_pz->text().toFloat();
    float *joints;
    joints = positionToJoints(px, py, pz);       //返回数组 （q1 q2 q3 error）

    if ( joints[3]==0 )
    {
        if (ui->comboBox_legs->currentIndex()==0)   motorSrv.request.drive_alllegs = 1;
        else if (ui->comboBox_legs->currentIndex()==1)
        {
            motorSrv.request.drive_alllegs = 0;
            motorSrv.request.servoID[0] = 1;
            motorSrv.request.servoID[1] = 2;
            motorSrv.request.servoID[2] = 3;
        }
        else if (ui->comboBox_legs->currentIndex()==2)
        {
            motorSrv.request.drive_alllegs = 0;
            motorSrv.request.servoID[0] = 4;
            motorSrv.request.servoID[1] = 5;
            motorSrv.request.servoID[2] = 6;
        }
        else if (ui->comboBox_legs->currentIndex()==3)
        {
            motorSrv.request.drive_alllegs = 0;
            motorSrv.request.servoID[0] = 7;
            motorSrv.request.servoID[1] = 8;
            motorSrv.request.servoID[2] = 9;
        }
        else if (ui->comboBox_legs->currentIndex()==4)
        {
            motorSrv.request.drive_alllegs = 0;
            motorSrv.request.servoID[0] = 10;
            motorSrv.request.servoID[1] = 11;
            motorSrv.request.servoID[2] = 12;
        }


        if (ui->Check_speed->isChecked())     //如果勾选了速度控制
        {
            ROS_INFO("速度控制模式下达指令 - 位置控制。");
            motorSrv.request.speedR[0] = ui->Lin_speedwz->text().toFloat();     //  °/s
            motorSrv.request.speedR[1] = ui->Lin_speedwz->text().toFloat();
            motorSrv.request.speedR[2] = ui->Lin_speedwz->text().toFloat();

            motorSrv.request.playtime[0] = 0;
        }
        else   // 时间控制
        {
            ROS_INFO("时间控制模式下达指令 - 位置控制。");
            motorSrv.request.playtime[0] = ui->Lin_playtimewz->text().toInt();       // s
            motorSrv.request.playtime[1] = ui->Lin_playtimewz->text().toInt();
            motorSrv.request.playtime[2] = ui->Lin_playtimewz->text().toInt();

            motorSrv.request.speedR[0] = 0;
        }

        motorSrv.request.angleR[0] = joints[0];    // 单位°
        motorSrv.request.angleR[1] = joints[1];
        motorSrv.request.angleR[2] = joints[2];

        if(! motorclient.call(motorSrv))
            ROS_ERROR("call service: motorCode failed -- robot get ready failed!");
    }
    else  QMessageBox::warning(this, "注意", "逆解出错，请核查！");

    free(joints);
}

// ===========   点击    ==============================================================================
void testdirectCtrlDlg::on_Btn_runOnce_clicked()
{
//    int  sp = 15;
//    //    float speedro[12] ={1, 1, 0.264,     1, 0.176,  0.217,   1,1, 0.268 ,     1, 0.009, 0.181};
//    //    float anglero[12] ={180, 96.8, 38.6,      157.9, 100.7, 43.4,     157.9, 140.2, 54,    180, 140, 50};

//    float speedro[12] ={1, 1, 0.347,     1, 0.097,  0.207,   1,1, 0.352 ,     1, 0.009, 0.181};
//    float anglero[12] ={180, 114.3, 41.07,      157.93, 116.45, 45.64,     157.93, 140.2, 54,    180, 140, 50};


//    for(int i=0; i<4; i++)
//    {
//        int j = 3*i;
//        motorSrv.request.servoID[0] = 1;
//        motorSrv.request.servoID[1] = 2;
//        motorSrv.request.servoID[2] = 3;

//        motorSrv.request.speedR[0] = sp* speedro[j] /6/0.114 ;     //   °/s转为舵机的速度代码（0-1023）
//        motorSrv.request.speedR[1] = sp* speedro[j+1] /6/0.114;
//        motorSrv.request.speedR[2] = sp* speedro[j+2] /6/0.114;

//        motorSrv.request.positionR[0] = anglero[j] /0.088;    // °转为舵机的角度（0-4095）
//        motorSrv.request.positionR[1] = anglero[j+1] /0.088;
//        motorSrv.request.positionR[2] = anglero[j+2] /0.088;

//        if(! motorclient.call(motorSrv))
//        {
//            ROS_ERROR("call service: motorCode failed -- robot get ready failed!");
//            //        qDebug()<<("call service: motorCode failed!");
//        }
//        else
//        {
//            ROS_INFO("call service: motorCode success!  step=%d",   i);
//        }

//        // sleep(3);   // 单位 秒
//        usleep(2*10000000);     // 单位 us
//    }
}






// ========================     正解+逆解 函数     ============================================================

//输入三关节角度(a1,a2,a3)，正解出末端坐标(x,y,z，error)，  ，角度单位度, mm
float* testdirectCtrlDlg::jointsToPosition(float q1, float q2, float q3)
{
    int error= 1 ;
    float px=0, py=0, pz=0;

    // 输入的是舵机角度，转为模型的角度, 角度转为弧度制
    float a1 = (q1 - 180)  *M_PI/180.0 ;   //  本体两侧的腿 加减不一样！！
    float a2 = (180 - q2)  *M_PI/180.0 ;
    float a3 = (q3 - 180)  *M_PI/180.0 ;

    //求解末端坐标 xyz mm
    px = (L1+L2*cos(a2)+L3*cos(a2+a3)) *sin(a1) ;
    py = (L1+L2*cos(a2)+L3*cos(a2+a3)) *cos(a1) ;
    pz = L2*sin(a2)+L3*sin(a2+a3) ;
    error=0;

    float *res;
    res = (float *)malloc(4);
    res[0]= px;
    res[1]= py;
    res[2]= pz;
    res[3]= error;
    return res;

}


//输入末端坐标(x,y,z)，逆解出三关节角度并返回(a1,a2,a3,error)   ，角度单位度
float* testdirectCtrlDlg::positionToJoints(float px,  float py,  float pz)
{
    ROS_INFO("\npx,py,pz = %f, %f, %f", px,py,pz);
    int error= 0 ;
    float a1=0, a2=0, a3=0;

    if (py<0)
        ROS_WARN("逆解输入Y为负！足端位置在身体内测！");
    else
    {
    // 求解关节角a1 a2 a3
        a1 = atan(px/py);
        ROS_INFO("px/py=%f, a1=%f",px/py, a1/M_PI*180);
        float A = ( (py/cos(a1)-L1)*(py/cos(a1)-L1) + pz*pz+ L2*L2- L3*L3 )/( 2*(py/cos(a1)-L1)*L2 );  //定义
        float B = pz*L2/( (py/cos(a1)-L1)*L2 );
        float D = 4*A*A*B*B-4*(B*B+1)*(A*A-1);
        ROS_INFO("A=%f, B=%f, D=%f", A,B,D);
        if (D>=0 )
        {
            error = 0;

            float sina2 = (2*A*B + sqrt(D))/(2*(B*B+1));      // - 是足底朝上的构型
            a2 = asin(sina2);
            float sina2a3 = (pz-L2*sin(a2))/L3;
            float p2y = (L1+L2*cos(a2))*cos(a1);
            float p2z = L2*sin(a2); //按照构型要求，连杆L2末端应比足底（连杆L3末端）位置高
            ROS_INFO("sina2=%f, a2=%f",sina2, a2/M_PI*180);
            ROS_INFO("sina2a3=%f, a2+a3=%f",sina2a3, asin(sina2a3)/M_PI*180);
            ROS_INFO("p2y=%f", p2y);
            ROS_INFO("p2z=%f", p2z);

            if (p2y < py)    //如果足底在最外侧
            {
                a3 = asin(sina2a3)-a2 ;
                ROS_INFO("如果足底最外p2y < py: a3=%f", a3/M_PI*180);
            }
            else  //如果足底内收
            {
                a3 = M_PI-asin(sina2a3) - a2 - 2*M_PI;    // a2+a3范围-270~180
                ROS_INFO("如果足底内收p2y > py: a3=%f", a3/M_PI*180);
            }

            //反求解末端坐标 xyz mm, 判断差值
            float px2 = (L1+L2*cos(a2)+L3*cos(a2+a3)) *sin(a1);
            float py2 = (L1+L2*cos(a2)+L3*cos(a2+a3)) *cos(a1);
            float pz2 = L2*sin(a2)+L3*sin(a2+a3);
            float dx = px2 - px;
            float dy = py2 - py;
            float dz = pz2 - pz;
            ROS_WARN("反求末端坐标xyz值= %f, %f, %f", px2,py2,pz2);

            if ( abs(dx)>5  || abs(dy)>5  || abs(dz)>5 )
            {
                error = 1;
                ROS_WARN("逆解值与输入值有差异，取另一组解");

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
                ROS_WARN("反求末端坐标xyz值= %f, %f, %f", px2,py2,pz2);

                if ( abs(dx)>5  || abs(dy)>5  || abs(dz)>5 )
                {
                    error = 1;
                    ROS_WARN("另一组解的逆解值与输入值也有差异，失败，请核查目标位置");
                }
                else
                    ROS_INFO("另一组解逆解验证成功。");

            }
            else
                ROS_INFO("逆解成功。");


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
    ROS_INFO("joints = %f,%f,%f",q1, q2, q3);

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





// =============================     选择速度还是时间控制     =====================================
void testdirectCtrlDlg::on_Check_speed_clicked()
{
    ui->Check_playtime->toggle();
//    if (ui->Check_speed->isChecked())      ui->Check_playtime->setChecked(false);
//    else ui->Check_playtime->setChecked(true);
}

void testdirectCtrlDlg::on_Check_playtime_clicked()
{
     ui->Check_speed->toggle();
}
// =============================     自动计算     =========================================================
void testdirectCtrlDlg::will_to_position()
{
    float q[3] = {0};
    q[0] = ui->Lin_angle11->text().toFloat();    // °舵机的角度（0-360du）
    q[1] = ui->Lin_angle11_2->text().toFloat();
    q[2] = ui->Lin_angle11_3->text().toFloat();

    // 正解出将到达位置
    float *position;
    position = jointsToPosition(q[0], q[1], q[2]);       //返回数组 （px py pz error）
    for (int i=0; i<3; i++)  //位置在表格中显示
    {
        ui->needtotargetTable->setItem(0, i, new QTableWidgetItem( QString::number(position[i],'f',2) ));   // 或去掉 'f',2
    }
    free(position);
}
void testdirectCtrlDlg::need_to_joints()
{
    float px = ui->Lin_foot_px->text().toFloat();
    float py = ui->Lin_foot_py->text().toFloat();
    float pz = ui->Lin_foot_pz->text().toFloat();
    float *joints;
    joints = positionToJoints(px, py, pz);       //返回数组 （q1 q2 q3 error）

    for (int i=0; i<3; i++)  //关节角度在表格中显示
        ui->needtoangleTable->setItem(0, i, new QTableWidgetItem( QString::number(joints[i],'f',2)));

    free(joints);
}



//QMessageBox::StandardButton result;  //返回选择的按钮
//result=QMessageBox::question(this, "下达控制指令", "目标位置有效，是否执行？ ",
//                             QMessageBox::Yes|QMessageBox::Cancel,
//                             QMessageBox::Yes);  //默认是 Yes
//if (result==QMessageBox::Yes)
//{
//}









