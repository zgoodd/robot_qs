#ifndef ROBOT_FUNCTION
#define ROBOT_FUNCTION

//#include "xleg_controller/testdirectctrldlg.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
extern DynamixelWorkbench dxl_wb;
/**********************************
//机构定义   X为前进方向
4号腿     1号腿
      Y+
X-———————————X+  --->>>>
      Y-
3号腿     2号腿


//腿关节定义
  腿ID          1   2   3   4
腰关节（舵机ID）  1   4   7   10     关节范围：ANGLE1_MIN - ANGLE1_MAX
髋关节（舵机ID）  2   5   8   11
膝关节（舵机ID）  3   6   9   12

*************************/


//=================       定义常量   ===================================================
#define M_PI       3.14159265358979323846   // pi
#define RIGHT 1
#define ERROR 0

//腰关节-髋关节-膝关节 角度区间
#define ANGLE1_MAX  265
#define ANGLE1_MIN  95
#define ANGLE2_MAX  260
#define ANGLE2_MIN  94
#define ANGLE3_MAX  280
#define ANGLE3_MIN  20


//机构本体长宽。 L：X方向尺寸， B：Y方向尺寸
#define CENTER_L    356.0
#define CENTER_B    320.0
#define LEGFORCEMIN  1.5    //判断力触地阈值


// 全局
extern float Leg_aim_angle[12];
extern float Leg_present_angle[12];
extern float Foot_present_forces[12];

extern bool stopFlag;

//=================       函数申明   ===================================================
void Send_Servo_Position(char dxl_id, uint16_t playtime, float angle);
void Send_Servo_Position_speed(char dxl_id, float speed, float angle);  // 速度+位置
void Send_One_Leg_Positon(char legID, uint16_t playtime);
void Send_ALL_Leg_Positon(uint16_t playtime);

int Set_ALL_Leg_Positon(float leg1[3], float leg2[3], float leg3[3], float leg4[3]);
int Set_One_Leg_Positon(char legID, float leg[]);
int CCenterToLeg(float v[3], float w[3], float P_foot1[3], float P_foot2[3], float P_foot3[3], float P_foot4[3], float p1[3], float p2[3], float p3[3], float p4[3]);
int LegForceToCenterG(float *force);

// XIE
float* jointsToPosition(float q1, float q2, float q3);
float* positionToJoints(float px,  float py,  float pz);

void test();
float** multiply_twoMats(float (*A)[3], float B[][3]);

//int Make_One_Leg_Runonce(char legID, float leg[3], int stepDistance, int stepHeight, uint16_t playtime);
int Make_One_Leg_Runonce(char legID, float leg[3], int dx, int dy, int dz, uint16_t playtime);
int  Make_One_Leg_Runonce_dz(char legID, float leg[3], int dx, int dy, int dz, int dz2, uint16_t playtime);
int Make_One_Leg_toPositon(char legID, float leg[3], int dz, uint16_t playtime);
int Make_Two_Legs_Runonce(char legID1, char legID2, float leg1[3], float leg2[3], int dx1, int dy1, int dz1,  int dx2, int dy2, int dz2, uint16_t playtime);

//单腿越障模式，检索到足底力停止足下落
//返回数组[q1 q2 q3 hz error]内容：当前腿三关节舵机角度q1,q2,q3，足相对腿起点的z坐标hz（用于求解障碍高度）,error（成功0, 错误1）
float* Make_One_Leg_Runonce_withForce(char legID, float leg[3], int dx, int dy, int dz, uint16_t playtime);

int Make_Runonce_noGravity(float vw_foot_abcd[30], int dx, int dy, int dz, uint16_t playtime , float vwfa_backdata[30]);
int Make_Runonce_withGravity( float vw_foot_abcd[30],  int dx, int dy, int dz, int bodyadjX, int bodyadjY, uint16_t playtime, float vwfa_backdata[30] );
int Climb_stepsurface_noGravity( float vw_foot_abcd[30],  int ssf_D, int ssf_H, int dz, uint16_t playtime, float vwfa_backdata[30] );
int Climb_stepsurface_withGravity( float vw_foot_abcd[30],  int ssf_D, int ssf_H, int dz, int bodyadjX, int bodyadjY, uint16_t playtime, float vwfa_backdata[30] );

//============================
#define ANGLE_K    6.14035
#define HERKULEX_ZERO  1024

#define       Leg_L1         48
#define       Leg_L2         190
#define       Leg_L3         248
#define       ARM_L1         48
#define       ARM_L2         190
#define       ARM_L3         166
#define       ARM_L4		 110
#define       ARM_L5		 65
                 

//
#define ANGLE4_MAX  0
#define ANGLE4_MIN  -120
#define ANGLE4_5_Z  10
//


const int Leg_Zero[4] = { 135,225,315,45 };

//extern float Leg_Arm_Flag;
//extern float Leg_aim_position[12];
//extern float Arm_aim_position[3];
//extern float Arm_aim_positionR[9];
//extern float Arm_aim_angle[5];

extern float Leg_rel_aforce[12];
extern float Leg_rel_pforce[12];

extern char ALL_Servo_rel_error[14];
extern char ALL_Servo_rel_torque[14];
extern char ALL_Servo_rel_led[14];
extern float ALL_Servo_rel_angle[14];










#define CENTER_LK   0.845
#define CENTER_BK   0.535
#define CENTER_H    30
#define CENTER_L2    163.5
#define CENTER_B2    103.5



#define RESULTFORCEXY   25.0

/***************************定义常量************************/
extern float Center_aim_EXP[6];
extern float Center_rel_EXP[6];
extern float Center_rel_Speed[6];
extern float Center_rel_Accelate[6];
extern float Center_rel_force[6];
extern float Center_rel_massxyz[3];
extern int    Leg_rel_Up;
extern int    Leg_aim_Up;


extern const float Center_body_X[4];
extern const float Center_body_Y[4];





#endif
