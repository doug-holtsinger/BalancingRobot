
#ifndef __NOTIFY_ROBOT_H__
#define __NOTIFY_ROBOT_H__

#define NOTIFY_PRINT_STR_MAX_LEN (size_t)256

#define PID_NOTIFY(_base, _idx) (_base + 6*_idx)

typedef enum
{
    PWM_CLOCK = NOTIFY_BASE_MAX + 1,
    MOTOR_ENABLED, 
    MOTOR_DRIVER,
    MOTOR_DISPLAY,
    PID_KP,
    PID_KI,
    PID_KD,
    PID_SP,
    PID_PV,
    PID_OUTPUT,

    PID_KP1,
    PID_KI1,
    PID_KD1,
    PID_SP1,
    PID_PV1,
    PID_OUTPUT1,

} DATA_NOTIFY_ROBOT_t;



#endif
