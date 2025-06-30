/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Author:       NEW-TJU-LIB-EDU                                           */
/*    Created:      2025.6.1                                                  */
/*    Description:  tjulib新生赛版本                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/*提示：在此main.cpp文件,你应当只修改以下部分:
    1.必要的头文件引入,不应该删除vex.h和tjulib.h
    2.PID参数的配置
    3.函数pre_auton()，autonomous()和usercontrol()的内容
*/

#include "vex.h"
#include "tjulib.h"

using namespace vex;
using namespace tjulib;

/**************************调参区域***********************************/
const double PI = 3.1415926535;

// 竞赛模板类
competition Competition;

/*************************************

        pid configurations

*************************************/

/*configure meanings：
    kp，ki，kd
    积分的作用区域 
    integral's active zone (either inches or degrees),
    误差的容许范围
    error's thredhold      (either inches or degrees),
    最小速度
    minSpeed               (in voltage),
    停止PID控制需要的停留在容差范围内的循环次数
    stop_num               (int_type)
*/

pidParams fwd_pid(0, 0, 0, 0, 0, 0, 0),  //往前直行一段指定距离所使用的PID参数
    turn_pid(0, 0, 0, 0, 0, 0, 0);       //转向指定角度所使用的PID参数                     
                                         //这里的PID参数需要根据实际情况进行调整,调PID是重要环节
                                         //一个可能的PID参数配置是(不准确,只是表示每个参数对应的数量级)：
                                         //pid(3, 0.1, 0.05, 0.5, 1, 10, 10);
                                         
/*************************************

        Instance for control

*************************************/

// ====Declaration of PID parameters and PID controllers ====
pidControl fwdControl(&fwd_pid);
pidControl turnControl(&turn_pid);

/**************************底盘定义***********************************/
// ordinary chassis define
std::vector<std::vector<vex::motor *> *> _chassisMotors = {&_leftMotors, &_rightMotors};
// 底盘电机编码器定位系统运行
Position position;
// 直线底盘
Ordi_StraChassis FDrive(_chassisMotors, &fwdControl, &turnControl);

/***************************

    pre-autonomous run

 **************************/
// 线程函数，实时更新底盘电机编码器的里程计数
int _positionThread()
{
    position.OdomRun();
    return 0;
}
// 这一部分会在比赛开始之前执行，用于完成机器人的初始化
void pre_auton()
{
    
    /***********imu等设备初始化************/
    printf("pre-auton start\n");
    while (imu.isCalibrating())
    {
        task::sleep(8);
    }
    if (imu.installed())
    {
        printf("imu is installed\n");
    }
    else
    {
        printf("imu is not installed\n");
    }
    // 启动线程，实时更新底盘电机编码器的里程计数
    thread positionThread(_positionThread);
}

void autonomous()
{
    // 你的自动程序写在这里面
}

/***************************

      usercontrol run

 **************************/
void usercontrol()
{
    // lambda表达式
    // 这个lambda表达式作为Controller1.ButtonDown.pressed()方法的参数传递，
    // 表示当按钮被按下时要执行的回调函数
    // 也就是，这里按下buttonDown按钮时，就会执行这个lambda表达式中的代码
    // 这里可以更换按键，也可以更换按键状态，共有两种：pressed()、released()
    extern pwm_out pwm_extend;
    extern motor ball;
    extern motor up;

    //按下Y键则气缸调用展开侧翼，再按一次取消
    Controller1.ButtonY.pressed([]()
                                   {
                                    static bool state_extend = false;
                                    if (!state_extend)
                                    {
                                        pwm_extend.state(100,percent);
                                        state_extend=1;
                                    }
                                    else
                                    {
                                        pwm_extend.state(0,percent);
                                        state_extend=0;
                                    }
                                       // 要执行的代码可以写在这里                                      
                                   });
    //按下A键开始进球（正转），再按一次反转
    Controller1.ButtonA.pressed([]()
                                   {
                                    static bool ballroll = false;
                                    if (!ballroll)
                                    {
                                        ball.spin(directionType::fwd, 100, velocityUnits::pct);
                                        ballroll=1;
                                    }
                                    else
                                    {
                                        ball.spin(directionType::fwd, -100, velocityUnits::pct);
                                        ballroll=0;
                                    }
                                       // 要执行的代码可以写在这里                                     
                                   });
    
    
    

   
    while (true)
    {
        // 调试时通过按键进入自动，比赛开始的时候记得注释！
        // 记得注释！
        // 记得注释！
        // 记得注释！
        if (Controller1.ButtonUp.pressing())
        {
            autonomous();
        }
        // 手动控制的死区，防止微小的摇杆输入导致底盘抖动
        // 遥感输入的数值在-127到127之间
        double deadzone = 10; 
        // 手动底盘行动代码示例
        int leftY = Controller1.Axis3.position(percent);
        if (abs(leftY) < deadzone)
            leftY = 0;

        int rightX = Controller1.Axis1.position(percent);
        if (abs(rightX) < deadzone)
            rightX = 0;
        // 如果需要更改手感，可以在这里把输出乘上系数
        // 例如：leftY *= 1.2; rightX *= 1.2;
        // 不建议把系数设置小于1，这会让车达不到最高速度
        // 也可以自己设置一个函数来实现更复杂的手感变化
        if (rightX)
        {
            FDrive.VRUN(rightX,-rightX);
        }
        else
        {
            FDrive.VRUN(leftY,leftY);
        }
        //一直按X则抬升，一直按B则下降
    if(Controller1.ButtonX.pressing())
    {
        up.spin(directionType::fwd, 1, velocityUnits::pct);
    }
    else if(Controller1.ButtonB.pressing())
    {
        up.spin(directionType::fwd, -1, velocityUnits::pct);
    }
    else
    {
        up.spin(directionType::fwd, 0, velocityUnits::pct);
    }
        
    }
}

int main()
{
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    pre_auton();
    return 0;
}
