#include "vex.h"

using namespace vex;

//此文件用于机器人相关参数的配置,根据注释和参数的定义,请保证每一个参数都和你的机器人一致
//你应当修改以下内容：
// 1.物理配置参数
// 2.底盘电机端口号、电机类型、正反转。
//   请将底盘电机的数量修改到和你的机器人一致,请不要只使用PORT22方式定义无效电机,这会导致moveInches函数出错
// 3.imu惯性传感器端口号

/*************************************

        physical configurations

*************************************/
const double r_wheel = 3.15/ 2;                      // 车轮半径(英寸)
const double gear_ratio = 0.8;                         // 底盘电机-轮的齿轮传动比（加速配比就大于1，减速配比就小于1）
const double r_motor = r_wheel * gear_ratio;         // 电机转角-电机转周的换算比
const double cell = 24;                              // 一个地垫长度(inches)

/*************************************

            VEX devices

*************************************/
// V5智能主控定义
brain  Brain;

// 底盘电机部分
// motor L1 = motor(PORT22, ratio6_1, true);
// motor L2 = motor(PORT22, ratio6_1, false);
// motor R1 = motor(PORT22, ratio6_1, false);
// motor R2 = motor(PORT22, ratio6_1, true);

// std::vector<vex::motor*> _leftMotors = {&L1, &L2};
// std::vector<vex::motor*> _rightMotors = {&R1, &R2};


motor L1 = motor(PORT11, ratio18_1, true);
motor L2 = motor(PORT12, ratio18_1, true);
motor R1 = motor(PORT13, ratio18_1, false);
motor R2 = motor(PORT14, ratio18_1, false);



std::vector<vex::motor*> _leftMotors = {&L1, &L2}; //修改到和你的机器人一致
std::vector<vex::motor*> _rightMotors = {&R1, &R2};
//std::vector<vex::motor*> _leftMotors = {&L1, &L2}; //修改到和你的机器人一致
//std::vector<vex::motor*> _rightMotors = {&R1, &R2};

// 遥控器
controller Controller1 = controller(primary); //不要修改
// imu惯性传感器
inertial imu = inertial(PORT10);

// 其他电机
motor ball = motor(PORT15,ratio18_1,false);
motor up = motor(PORT16,ratio36_1,false);

//气缸
pwm_out pwm_extend = pwm_out(Brain.ThreeWirePort.B);


// 这个函数不要删也不要动
void vexcodeInit( void ) {
  // nothing to initialize
}