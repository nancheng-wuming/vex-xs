#include "pidControl.hpp"

//提示：在此pidControl.cpp文件,你应当只修改pidCalcu方法,自行完成微分项计算和输出速度的公式

namespace tjulib
{   
    // 用于反映当前的pid控制是否在结果容差范围内停留了足够的时间
    bool pidControl :: overflag()
    {
        if( cnt >= params->stop_num ){
            return true;
        }
        else{ return false; }
    }

    // 重置pid控制器的状态
    void pidControl::reset(){
        integral = 0;
        derivative = 0;
        error = 0;
        lastError = 0;
    }

    // 重置pid控制器的状态
    void pidControl::resetpid(){
        integral = 0;
        derivative = 0;
        error = 0;
        lastError = 0;
        cnt =  0;
    }

    T pidControl::pidCalcu(T target, T maxSpeed, T feedback)
    {
        // 限制积分项输出的上限，不让积分项无限制叠加
        T integralPowerLimit = 20 / params->ki; 
        T speed = 0;

        // error是目标值和反馈值之间的差距
        error = target - feedback; 

        // 如果error仍然没有达到目标容差
        if (fabs(error) > params->errorThreshold) 
        {
            // 只在积分生效的范围内计算积分项
            if (fabs(error) < params->integralActiveZone)
            {
                // 把当前误差累加到积分项上
                integral = integral + error; 
            }
            else
            {
                integral = 0;
            }
            // 限制积分上限
            if (integral < -integralPowerLimit)
                integral = -integralPowerLimit;
            if (integral > integralPowerLimit)
                integral = integralPowerLimit;

            // 微分项的计算：近似为当前误差和上个pid循环误差的差值
            // 自行填写 derivative = ...;
            derivative = error - lastError;

            // pid控制器的核心部分，通过pid公式计算速度
            // 自行填写,speed = ...;
            // speed = 
            speed = params->kp * error + params->ki * integral + params->kd * derivative;

             // 把最高速度限制在maxSpeed范围内
            if (speed < -maxSpeed)
                speed = -maxSpeed;
            if (speed > maxSpeed)
                speed = maxSpeed;

            // 让最小的速度也能够克服运动的的阻力缓慢移动
            if (speed > 0 && speed < params->minSpeed)
            {
                speed = params->minSpeed;
            }
            if (speed < 0 && speed > -params->minSpeed)
            {
                speed = -params->minSpeed;
            }
        }
        else
        {
            // 如果已经到达容差范围内，结束pid控制
            speed = 0;
        }
        // 保存本次pid控制的误差，传递给下一次pid循环
        lastError = error; 
        // 返回计算结果
        return speed; 
    }
};