
#ifndef __PERF__H
#define __PERF__H

#include "MotorDriver.h"

constexpr float roll_max = MOTOR_DISABLE_ROLL_ANGLE / 5.0f;

class PERF {
    public:
        void update(float roll, pid_ctrl_t pidCtrlValue, int32_t wheel_encoder_speed);
        void send_client_data(char *p);
        void send_all_client_data();
    private:
	float roll_normalized = 0.0;
	float pid_ctrl_normalized = 0.0;
	float wheel_speed_normalized = 0.0;
	int32_t wheel_speed_max = 0;
};

#endif
