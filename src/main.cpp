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

pidParams fwd_pid(5, 0.1, 0.05, 0.5, 1, 10, 10),  //往前直行一段指定距离所使用的PID参数
    turn_pid(2.5, 0, 0.12, 0.2, 0.75, 10, 2);       //转向指定角度所使用的PID参数                     
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
Ordi_StraChassis FDrive(_chassisMotors, &fwdControl, &turnControl,&position);

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
    extern motor ball;
    extern pwm_out pwm_extend_left;
    extern pwm_out pwm_extend_right;

    bool ontheleft=0;
    const double ballSpeed = 50;
    T maxSpeed=80;
    imu.calibrate();
    while(imu.isCalibrating())
    {
        task::sleep(8);
    }
    imu.resetHeading();
    FDrive.moveInches(cell,maxSpeed);

    // if(ontheleft)
    // {
    //     pwm_extend_left.state(0,percent);
    // }
    // else
    // {
    //     pwm_extend_right.state(0,percent);
    // }
    // FDrive.moveInches(0.5*cell,maxSpeed);
    // if(ontheleft)
    // {
    //     FDrive.turnToAngle(90,80,5000.0);
    // }
    // else
    // {
    //     FDrive.turnToAngle(-90,80,5000.0);
    // }
    
    // FDrive.moveInches(cell,maxSpeed);
    // task::sleep(8);


    // FDrive.moveInches(-cell,maxSpeed);

    // if(ontheleft)
    // {
    //     FDrive.turnToAngle(90,80,5000.0);
    // }
    // else
    // {
    //     FDrive.turnToAngle(-90,80,5000.0);
    // }

    // task::sleep(8);
    // if(ontheleft)
    // {
    //     pwm_extend_left.state(100,percent);
    // }
    // else
    // {
    //     pwm_extend_right.state(100,percent);
    // }
    // task::sleep(8);

    // FDrive.moveInches(2*cell,maxSpeed);


    // FDrive.moveInches(2*cell,maxSpeed);
    // if(ontheleft)
    // {
    //      FDrive.turnToAngle(90,maxSpeed,5000.0);

    // }
    // else{
    //     FDrive.turnToAngle(-90,maxSpeed,5000.0);
    // }
    // FDrive.moveInches(0.5*cell,maxSpeed);
    // ball.spin(directionType::fwd, ballSpeed, velocityUnits::pct);
    // ball.spinFor(2000.0,timeUnits::msec,-ballSpeed,velocityUnits::pct);
    // FDrive.moveInches(-0.5*cell,maxSpeed);
    // if(ontheleft)
    // {
    //      FDrive.turnToAngle(90,maxSpeed,5000.0);

    // }
    // else{
    //     FDrive.turnToAngle(-90,maxSpeed,5000.0);
    // }


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
    extern pwm_out pwm_extend_right;
    extern motor ball;
    extern motor up;



    //按下R1键则气缸展开侧翼，再按一次取消
    Controller1.ButtonR1.pressed([]()
                                   {
                                    static bool state_extend = false;
                                    if (!state_extend)
                                    {
                                        pwm_extend_right.state(100,percent);
                                        state_extend=1;
                                    }
                                    else
                                    {
                                        pwm_extend_right.state(0,percent);
                                        state_extend=0;
                                    }
                                       // 要执行的代码可以写在这里                                      
                                   });


    
    //按下A键开始进球（正转），再按一次反转
    
    static int ballroll = 0;


    Controller1.ButtonA.pressed([]()
                                   {
                                    ball.spin(directionType::fwd, 100, velocityUnits::pct);
                                    
                                       // 要执行的代码可以写在这里                                     
                                   });
    

    Controller1.ButtonB.pressed([]()
                                   {
                                    // if (ball.isSpinning()) 
                                    // {
                                    //     ball.stop();
                                    //     ballroll = 0;
                                    // }
                                    // if(!ballroll)
                                    // {
                                    //     ball.spin(directionType::fwd, -100, velocityUnits::pct);
                                    //     ballroll = 1;
                                    // }
                                    // else
                                    // {
                                    //     ball.stop();
                                    //     ballroll = 0;
                                    // }
                                    ball.spin(directionType::fwd, -100, velocityUnits::pct);

    
                                       // 要执行的代码可以写在这里                                     
                                   });
    
    
    

   
    while (true)
    {
        // 调试时通过按键进入自动，比赛开始的时候记得注释！
        // 记得注释！
        // 记得注释！
        // 记得注释！
        bool ontheleft=1;
        if (Controller1.ButtonUp.pressing())
        {
            autonomous();
        }
        // 手动控制的死区，防止微小的摇杆输入导致底盘抖动
        // 遥感输入的数值在-127到127之间
         double deadzone = 10; 
        // 手动底盘行动代码示例
        int leftY = Controller1.Axis3.position(percent);
        int rightX = Controller1.Axis1.position(percent);
        if (abs(leftY) < deadzone)
            leftY = 0;
        if (abs(rightX) < deadzone)
            rightX = 0;
        
        // 如果需要更改手感，可以在这里把输出乘上系数
        // 例如：leftY *= 1.2; rightX *= 1.2;
        // 不建议把系数设置小于1，这会让车达不到最高速度
        // 也可以自己设置一个函数来实现更复杂的手感变化
        
        //L1.spin(directionType::fwd, (-leftY+rightX), velocityUnits::pct);
        //L2.spin(directionType::fwd, (-leftY+rightX), velocityUnits::pct);
        //R1.spin(directionType::fwd,x*(-leftY-rightX), velocityUnits::pct);
        //R2.spin(directionType::fwd, x*(-leftY-rightX), velocityUnits::pct);

        FDrive.VRUN(leftY+rightX,leftY-rightX);

        T x=50;
        if(Controller1.ButtonX.pressing())
        {
            up.spin(directionType::fwd, x, velocityUnits::pct);
        }
        else if(Controller1.ButtonY.pressing())
        {
            up.spin(directionType::fwd, -x, velocityUnits::pct);
        }
        else
        {
            up.stop(hold);
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
