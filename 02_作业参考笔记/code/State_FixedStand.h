/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"
#include <yaml-cpp/yaml.h>

class State_FixedStand : public FSMState
{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand() {}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    float _targetPos[12] = {
                                -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                                -0.1, 0.0, 0.0, 0.3, -0.2, 0.0
                                };

    float kp[12] = {
                    60, 60, 60, 100, 40, 40,      // legs
                    60, 60, 60, 100, 40, 40      // legs
                };

    float kd[12] = {
                    1, 1, 1, 2, 1, 1,     // legs
                    1, 1, 1, 2, 1, 1     // legs
                };

    float _startPos[12];
    float _duration = 1000;   
    float _percent = 0;   
};
#endif // FIXEDSTAND_H
