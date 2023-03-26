
#ifndef __PID__H

#include "app_config.h"
#include "sdk_config.h"

#include <stdio.h>
#include <stdint.h>
// #include <stdlib.h>
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
#include "PIDCmd.h"

#include "notify.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"


#define PID_KP_DEFAULT 1.0
#define PID_KI_DEFAULT 0.0
#define PID_KD_DEFAULT 0.0
#define PID_SP_DEFAULT 0.0

#define PID_KP_INCREMENT 1.0
#define PID_KI_INCREMENT 1.0
#define PID_KD_INCREMENT 0.1

#define PID_KP_VALID	0x0001
#define PID_KI_VALID	0x0002
#define PID_KD_VALID	0x0004
#define PID_PARAM_VALID	0x0007

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

        void cmd(const PID_CMD_t i_cmd)
        {
            switch (i_cmd)
            {
		case PID_CMD_t::PID_KP_UP:
                    pidParams.KP += PID_KP_INCREMENT;
                    break;
		case PID_CMD_t::PID_KP_DOWN:
                    pidParams.KP -= PID_KP_INCREMENT;
                    break;
		case PID_CMD_t::PID_KI_UP:
                    pidParams.KI += PID_KI_INCREMENT;
                    break;
		case PID_CMD_t::PID_KI_DOWN:
                    pidParams.KI -= PID_KI_INCREMENT;
                    break;
		case PID_CMD_t::PID_KD_UP:
                    pidParams.KD += PID_KD_INCREMENT;
                    break;
		case PID_CMD_t::PID_KD_DOWN:
                    pidParams.KD -= PID_KD_INCREMENT;
                    break;
                default:
                    NRF_LOG_INFO("Invalid cmd %d", i_cmd);
                    break;
            }
        }

	void send_client_data(char *p)
	{
            uint8_t *p_data = (uint8_t *)p;
            ble_svcs_send_client_notification(p_data, strlen(p));
	}

	void send_all_client_data()
	{
            char s[NOTIFY_PRINT_STR_MAX_LEN];

	    if ( !ble_svcs_connected() ) {
                return;
            }
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT,
                PID_KP,
                PRINTF_FLOAT_VALUE(pidParams.KP));
            send_client_data(s);
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT,
                PID_KI,
                PRINTF_FLOAT_VALUE(pidParams.KI));
            send_client_data(s);
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT,
                PID_KD,
                PRINTF_FLOAT_VALUE(pidParams.KD));
            send_client_data(s);
	}

	void setParameters(const pidParameters i_param, const uint16_t flags)
	{
            if (flags & PID_KP_VALID)
	    {
                pidParams.KP = i_param.KP;
	    }
            if (flags & PID_KI_VALID)
	    {
                pidParams.KI = i_param.KI;
	    }
            if (flags & PID_KD_VALID)
	    {
                pidParams.KD = i_param.KD;
	    }
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

