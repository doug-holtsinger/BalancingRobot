
#ifndef __PID__H

#include "app_config.h"
#include "sdk_config.h"

#include <stdio.h>
#include <string.h>
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "nrfx_pwm.h"
#include "nrfx_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "logdef.h"
#include <math.h>
#include <vector>

#define PID_KP_DEFAULT 1.0
#define PID_KI_DEFAULT 0.0
#define PID_KD_DEFAULT 0.0
#define PID_SP_DEFAULT 0.0

constexpr uint32_t PID_ERROR_HISTORY_DEPTH = 3;

struct pidParameters
{
    float KP {PID_KP_DEFAULT};
    float KI {PID_KI_DEFAULT};
    float KD {PID_KD_DEFAULT};
};

template <typename T>
class PID {
    public:
        PID(const pidParameters i_param, const float i_SP, const T i_pidControlMax) :
		pidParams(i_param),
		SP(i_SP),
		controlSettingMax(i_pidControlMax)
	{
	}

        void setpoint(const float i_SP)
	{
            SP = i_SP;
	}

	T update(const float i_PV)
	{
            float errorDiff = i_PV - SP;
	    float controlSetFloat;

	    // error history
	    errorHistory.push_back(errorDiff);
	    if (errorHistory.size() > PID_ERROR_HISTORY_DEPTH)
	    {
	        errorHistory.erase(errorHistory.cbegin());
	    }

            // KD contribution
	    controlSetFloat = pidParams.KP * errorDiff;

            // KI contribution
	    for (short unsigned int i=0 ; i < errorHistory.size() ; ++i)
	    {
	        controlSetFloat += (pidParams.KI * errorHistory[i]);
	    }

	    // KD contribution
            size_t errLen = errorHistory.size();
	    if (errLen >= 2)
	    {
	        controlSetFloat += (pidParams.KD * (errorHistory[errLen-1] - errorHistory[errLen-2]));
	    }

	    // Max setting
	    if (controlSetFloat > controlSettingMax)
	    {
                controlSetting = controlSettingMax;
	    } else if (controlSetFloat < -controlSettingMax)
	    {
                controlSetting = -controlSettingMax;
	    } else {
	        controlSetting = floor(controlSetFloat + 0.5);
	    }

            return controlSetting;
	}
    private:
	pidParameters pidParams;
        float SP {PID_SP_DEFAULT};
	T controlSetting {0};
	T controlSettingMax {0};
	std::vector<float> errorHistory;
};
#endif

