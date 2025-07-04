#pragma once
#include "tjulib.h"

extern double zero_drift_error;

namespace tjulib
{
    using namespace vex;
    class Ordi_BaseChassis : public BaseChassis
    {
    protected:
        std::vector<vex::motor *> &_leftMotors;
        std::vector<vex::motor *> &_rightMotors;

    public:
        Ordi_BaseChassis(std::vector<std::vector<vex::motor *> *> &_chassisMotors,
                         pidControl *_fwdpidControl, pidControl *_turnpidControl)
                        : BaseChassis(_chassisMotors, _fwdpidControl, _turnpidControl),
              _leftMotors(*_chassisMotors[0]),
              _rightMotors(*_chassisMotors[1])
        {
        }
        // 直接给底盘电机电压，进行运动
        void VRUN(T Lspeed, T Rspeed)
        {
            Lspeed=Lspeed*0.825;
            for (vex::motor *motor : (_leftMotors))
            {
                vexMotorVoltageSet(motor->index(), Lspeed * 12700.0 / 100.0);
            }
            for (vex::motor *motor : (_rightMotors))
            {
                vexMotorVoltageSet(motor->index(), Rspeed * 12700.0 / 100.0);
            }
        }

    };

} // namespace tjulib
