
#include "PID.h"

#ifndef __MOTOR_DRIVER__H

typedef int16_t pid_ctrl_t;
typedef uint16_t motor_ctrl_t;
typedef uint16_t pwm_top_value_t;
typedef uint16_t pwm_seq_value_t;

constexpr pwm_top_value_t MOTOR_DRIVER_TOP_VALUE = 8192;
constexpr int16_t PID_CONTROL_SETTING_MAX = MOTOR_DRIVER_TOP_VALUE-1;
constexpr uint16_t MOTOR_CONTROL_SETTING_MASK = 0x7FFF;
// Setpoint default of PID controller
constexpr float MOTOR_DRIVER_SP_DEFAULT = 0.0;
constexpr pwm_seq_value_t PWM_POL_FALLING_EDGE = 0x8000;
constexpr pwm_seq_value_t PWM_POL_RISING_EDGE = 0x0000;

constexpr float MOTOR_PID_KP = PID_CONTROL_SETTING_MAX / 90.0;
constexpr float MOTOR_PID_KI = MOTOR_PID_KP / 2.0;
constexpr float MOTOR_PID_KD = 0.0;
constexpr float MOTOR_PID_SP = 0.0;

class MotorDriver {
    public:
        MotorDriver();
        void init();
        void setValues(pid_ctrl_t driver0, pid_ctrl_t driver1);
        void setPitchAngle(const float pitch);
    private:
        PID<pid_ctrl_t> pidCtrl;
};

#endif
