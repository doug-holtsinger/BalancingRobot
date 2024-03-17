
#ifndef __PID__H
#define __PID__H

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
#include "notify_robot.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"

#include "param_store.h"

#define PID_KP_DEFAULT 1.0
#define PID_KI_DEFAULT 0.0
#define PID_KD_DEFAULT 0.0
#define PID_SP_DEFAULT 0.0

#define PID_KP_VALID	0x0001
#define PID_KI_VALID	0x0002
#define PID_KD_VALID	0x0004
#define PID_PARAM_VALID	0x0007

constexpr uint32_t PID_ERROR_HISTORY_DEPTH = 3;

typedef struct 
{
    float KP {PID_KP_DEFAULT};
    float KI {PID_KI_DEFAULT};
    float KD {PID_KD_DEFAULT};
    float SP {PID_SP_DEFAULT};
} pidParameters_t;

template <typename T>
class PID {
    public:
        PID(const pidParameters_t i_param, const pidParameters_t i_increment, 
			const T i_pidControlMax, const uint16_t paramRecordKey, 
			const uint16_t i_pidNum, const bool i_reverse_output = false,
			const bool i_low_pass_filter = false) :
		pidParams(i_param),
		pidIncrement(i_increment),
		controlSettingMax(i_pidControlMax),
		param_store(paramRecordKey),
		pidNum(i_pidNum),
		reverseOutput(i_reverse_output),
		lowPassFilter(i_low_pass_filter),
		l_PV(0.0)
	{
	}

	void init()
	{
            pidParameters_t pp;
            param_store.init(&pidParams);
            pp = param_store.get();
	    init_params(pp);
	}

	void init_params(pidParameters_t params)
	{
            pidParams = params;
	}

	void params_save()
	{
            param_store.set(&pidParams); 
	}

        void cmd(const PID_CMD_t i_cmd)
        {
            switch (i_cmd)
            {
		case PID_CMD_t::PID_KP_UP:
                    pidParams.KP += pidIncrement.KP;
                    break;
		case PID_CMD_t::PID_KP_DOWN:
                    pidParams.KP -= pidIncrement.KP;
		    if (pidParams.KP < 0.0)
		    {
                        pidParams.KP = 0.0;
		    }
                    break;
		case PID_CMD_t::PID_KI_UP:
                    pidParams.KI += pidIncrement.KI;
                    break;
		case PID_CMD_t::PID_KI_DOWN:
                    pidParams.KI -= pidIncrement.KI;
		    if (pidParams.KI < 0.0)
		    {
                        pidParams.KI = 0.0;
		    }
                    break;
		case PID_CMD_t::PID_KD_UP:
                    pidParams.KD += pidIncrement.KD;
                    break;
		case PID_CMD_t::PID_KD_DOWN:
                    pidParams.KD -= pidIncrement.KD;
		    if (pidParams.KD < 0.0)
		    {
                        pidParams.KD = 0.0;
		    }
                    break;
		case PID_CMD_t::PID_SP_UP:
                    pidParams.SP += pidIncrement.SP;
                    break;
		case PID_CMD_t::PID_SP_DOWN:
                    pidParams.SP -= pidIncrement.SP;
                    break;
		case PID_CMD_t::PID_PARAMS_SAVE:
		    params_save();
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
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_KP, pidNum),
                PRINTF_FLOAT_VALUE2(pidParams.KP));
            send_client_data(s);

            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_KI, pidNum),
                PRINTF_FLOAT_VALUE2(pidParams.KI));
            send_client_data(s);

            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_KD, pidNum),
                PRINTF_FLOAT_VALUE2(pidParams.KD));
            send_client_data(s);

            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_SP, pidNum),
                PRINTF_FLOAT_VALUE2(pidParams.SP));
            send_client_data(s);

	    // For PID=1, this is the motor encoding in floating point
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT,
                PID_NOTIFY(PID_PV, pidNum),
                PRINTF_FLOAT_VALUE(l_PV_save));
            send_client_data(s);

	    // For PID=1, this becomes the setpoint for the Motor PID
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_OUTPUT, pidNum),
                PRINTF_FLOAT_VALUE2(controlSetting));
            send_client_data(s);
	}

	void setParameters(const pidParameters_t i_param, const uint16_t flags)
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

        void setSP(const float i_SP)
	{
            pidParams.SP = i_SP;
	}

        float getSP()
	{
            return pidParams.SP; 
	}

	T update(const float i_PV)
	{
            if (lowPassFilter)
	    {
                l_PV = 0.8 * l_PV + 0.2 * i_PV;
	    } else {
                l_PV = i_PV; 
	    }
	    l_PV_save = i_PV;
            const float errorDiff = l_PV - pidParams.SP;

	    // error history
	    errorHistory.push_back(errorDiff);
	    if (errorHistory.size() > PID_ERROR_HISTORY_DEPTH)
	    {
	        errorHistory.erase(errorHistory.cbegin());
	    }

            // KP contribution
	    float controlSetFloat = pidParams.KP * errorDiff;

            // KI contribution
	    if (pidParams.KI > 0.0)
	    {
	        for (short unsigned int i=0 ; i < errorHistory.size() ; ++i)
	        {
	            controlSetFloat += (pidParams.KI * errorHistory[i]);
	        }
	    }

	    // KD contribution
	    if (pidParams.KD > 0.0)
	    {
                size_t errLen = errorHistory.size();
	        if (errLen >= 2)
	        {
	            controlSetFloat += (pidParams.KD * (errorHistory[errLen-1] - errorHistory[errLen-2]));
	        }
	    }

	    // Max setting
	    if (controlSetFloat > controlSettingMax)
	    {
                controlSetFloat = controlSettingMax;
	    } else if (controlSetFloat < -controlSettingMax)
	    {
                controlSetFloat = -controlSettingMax;
	    } 

	    if (typeid(controlSetting) != typeid(float)) {
		// round to integer
	        controlSetting = floor(controlSetFloat + 0.5);
	    } else {
	        controlSetting = controlSetFloat;
	    }

	    if (reverseOutput)
	    {
                controlSetting = -controlSetting;
	    }
#if 0
	    if (pidNum == 1)
	    {
                NRF_LOG_INFO("i_PV" PRINTF_FLOAT_FORMAT " l_PV " PRINTF_FLOAT_FORMAT2 " ctrl " PRINTF_FLOAT_FORMAT2, PRINTF_FLOAT_VALUE(i_PV), PRINTF_FLOAT_VALUE2(l_PV), PRINTF_FLOAT_VALUE2(controlSetting));
	    }
#endif

            return controlSetting;
	}
    private:
	pidParameters_t pidParams;		// local copy of PID parameters
	const pidParameters_t pidIncrement;		// local copy of PID Increment values
	const T controlSettingMax {0};
	T controlSetting {0};
        ParamStore<pidParameters_t> param_store;
	std::vector<float> errorHistory;
        const uint16_t pidNum;
	const bool reverseOutput;
	const bool lowPassFilter;

	float l_PV;
	float l_PV_save;
	float controlSettingSave;
};
#endif

