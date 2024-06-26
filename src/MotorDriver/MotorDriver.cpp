/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include "app_config.h"
#include "sdk_config.h"

#include <stdio.h>
#include <string.h>
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "nrfx_pwm.h"
#include "nrfx_clock.h"
#include "nrfx_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "notify_robot.h"
#include "MotorDriver.h"
#include "MotorDriverCmd.h"

#include "ble_svcs_cmd.h"
#include "ble_svcs.h"

static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);
// static int print_debug = 0;

// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
#define USED_PWM(idx) (1UL << idx)
static uint8_t m_used = 0;

// 4 channels , only 2 used
static uint16_t seq_values[] =
{
    0x0000,
    0x0000,
    0x0000,
    0x0000,
};

MotorDriver::MotorDriver() :
    pidCtrl({MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_SP},
       {MOTOR_PID_KP_INCR, MOTOR_PID_KI_INCR, MOTOR_PID_KD_INCR, MOTOR_PID_SP_INCR},
        PID_CONTROL_SETTING_MAX, MOTOR_PID_RECORD_KEY, MOTOR_PID_NUM, false),
    motor_enabled(true),
    display_enabled(true),
    drv_ctrla(0), 
    drv_ctrlb(0), 
    pwm_base_clock(NRF_PWM_CLK_8MHz)
{
}

void MotorDriver::setActualRollAngle(float i_roll)
{
    drv_ctrla = pidCtrl.update(i_roll);
    drv_ctrlb = -drv_ctrla;
    if (abs(i_roll) > MOTOR_DISABLE_ROLL_ANGLE)
    {
        drv_ctrla = drv_ctrlb = 0;
    }
    this->setValues(drv_ctrla, drv_ctrlb);
}

void MotorDriver::setDesiredRollAngle(float i_roll)
{
    pidCtrl.setSP(i_roll);
}

void MotorDriver::getValues(pid_ctrl_t& driver0, pid_ctrl_t& driver1)
{
    driver0 = drv_ctrla;
    driver1 = drv_ctrlb;
}

pid_ctrl_t MotorDriver::getValue()
{
    return drv_ctrla;
}

void MotorDriver::setValues(pid_ctrl_t driver0, pid_ctrl_t driver1)
{
    if (!motor_enabled)
    {
        driver0 = driver1 = 0;
    }
    if (driver0 >= 0)
    {
        nrfx_gpiote_out_set(MOTOR_DRIVER_APHASE);
    } else {
        nrfx_gpiote_out_clear(MOTOR_DRIVER_APHASE);
    }
    if (driver1 >= 0)
    {
        nrfx_gpiote_out_set(MOTOR_DRIVER_BPHASE);
    } else {
        nrfx_gpiote_out_clear(MOTOR_DRIVER_BPHASE);
    }
    seq_values[0] = PWM_POL_FALLING_EDGE | (abs(driver0) & MOTOR_CONTROL_SETTING_MASK);
    seq_values[1] = PWM_POL_FALLING_EDGE | (abs(driver1) & MOTOR_CONTROL_SETTING_MASK); 
}

void MotorDriver::init()
{
    pidCtrl.init();

    if (!nrfx_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrfx_gpiote_init());
    }
    
    // Configure GPIO pins for Motor direction
    nrfx_gpiote_out_config_t gpio_configA = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    APP_ERROR_CHECK(nrfx_gpiote_out_init(MOTOR_DRIVER_APHASE, &gpio_configA));

    nrfx_gpiote_out_config_t gpio_configB = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    APP_ERROR_CHECK(nrfx_gpiote_out_init(MOTOR_DRIVER_BPHASE, &gpio_configB));

    // Configure PWM pins for Motor PWM
    nrfx_pwm_config_t const config0 =
    {
        .output_pins =
        {
            MOTOR_DRIVER_AENBL,        // channel 0
            MOTOR_DRIVER_BENBL,        // channel 1
            NRFX_PWM_PIN_NOT_USED,     // channel 2
            NRFX_PWM_PIN_NOT_USED,     // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = pwm_base_clock,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = MOTOR_DRIVER_TOP_VALUE,
        //.load_mode    = NRF_PWM_LOAD_COMMON,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrfx_pwm_init(&m_pwm0, &config0, NULL));
    m_used |= USED_PWM(0);

    nrf_pwm_sequence_t const seq =
    {
        seq_values,            // values.p_common
        .length = NRF_PWM_VALUES_LENGTH(seq_values),  // length
        .repeats = 0,             // repeats
        .end_delay = 0              // end_delay
    };

    (void)nrfx_pwm_simple_playback(&m_pwm0, &seq, 1, NRFX_PWM_FLAG_LOOP);

}

void MotorDriver::send_client_data(char *p)
{
    uint8_t *p_data = (uint8_t *)p;
    ble_svcs_send_client_notification(p_data, strlen(p));
}

void MotorDriver::send_all_client_data()
{
    char s[NOTIFY_PRINT_STR_MAX_LEN];

    if ( !ble_svcs_connected() || !display_enabled) {
        return;
    }
    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %d", 
                MOTOR_ENABLED, motor_enabled); 
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %d", 
                MOTOR_DRIVER, drv_ctrla);
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %d", 
                MOTOR_DISPLAY, display_enabled); 
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d %d", 
                PWM_CLOCK, pwm_base_clock); 
    send_client_data(s);

    pidCtrl.send_all_client_data();
}

void MotorDriver::pwm_base_clock_modify(const bool up)
{
    // clock up, requires decreasing PWM CLK enum value
    // clock down, requires increasing PWM CLK enum value
    if (!up)
    {
        switch (pwm_base_clock)
        {
            case NRF_PWM_CLK_125kHz:
                pwm_base_clock = NRF_PWM_CLK_250kHz;
                break;
            case NRF_PWM_CLK_250kHz:
                pwm_base_clock = NRF_PWM_CLK_500kHz;
                break;
            case NRF_PWM_CLK_500kHz:
                pwm_base_clock = NRF_PWM_CLK_1MHz;
                break;
            case NRF_PWM_CLK_1MHz:
                pwm_base_clock = NRF_PWM_CLK_2MHz;
                break;
            case NRF_PWM_CLK_2MHz:
                pwm_base_clock = NRF_PWM_CLK_4MHz;
                break;
            case NRF_PWM_CLK_4MHz:
                pwm_base_clock = NRF_PWM_CLK_8MHz;
                break;
            case NRF_PWM_CLK_8MHz:
                pwm_base_clock = NRF_PWM_CLK_16MHz;
                break;
            case NRF_PWM_CLK_16MHz:
            default: break;
        }
    } else 
    {
        switch (pwm_base_clock)
        {
            case NRF_PWM_CLK_125kHz:
                break;
            case NRF_PWM_CLK_250kHz:
                pwm_base_clock = NRF_PWM_CLK_125kHz;
                break;
            case NRF_PWM_CLK_500kHz:
                pwm_base_clock = NRF_PWM_CLK_250kHz;
                break;
            case NRF_PWM_CLK_1MHz:
                pwm_base_clock = NRF_PWM_CLK_500kHz;
                break;
            case NRF_PWM_CLK_2MHz:
                pwm_base_clock = NRF_PWM_CLK_1MHz;
                break;
            case NRF_PWM_CLK_4MHz:
                pwm_base_clock = NRF_PWM_CLK_2MHz;
                break;
            case NRF_PWM_CLK_8MHz:
                pwm_base_clock = NRF_PWM_CLK_4MHz;
                break;
            case NRF_PWM_CLK_16MHz:
                pwm_base_clock = NRF_PWM_CLK_8MHz;
                break;
            default: break;
        }
    }
    nrf_pwm_configure(m_pwm0.p_registers,
        pwm_base_clock, NRF_PWM_MODE_UP, MOTOR_DRIVER_TOP_VALUE);
}

void MotorDriver::cmd_internal(const MOTOR_DRIVER_CMD_t i_cmd)
{
    switch (i_cmd)
    {
        case MOTOR_DRIVER_CMD_t::TOGGLE_POWER:
            motor_enabled = !motor_enabled;
            break;
        case MOTOR_DRIVER_CMD_t::TOGGLE_DISPLAY:
            display_enabled = !display_enabled;
            break;
        case MOTOR_DRIVER_CMD_t::PWM_CLK_UP:
	    pwm_base_clock_modify(true);
            break;
        case MOTOR_DRIVER_CMD_t::PWM_CLK_DOWN:
	    pwm_base_clock_modify(false);
            break;
        default: NRF_LOG_INFO("Invalid Motor cmd %d", i_cmd); break;
    }
}

void MotorDriver::cmd(const uint8_t i_cmd)
{
    cmd_internal(static_cast<MOTOR_DRIVER_CMD_t>(i_cmd));
}

void MotorDriver::PIDCmd(const uint8_t i_cmd)
{
    pidCtrl.cmd(static_cast<PID_CMD_t>(i_cmd));
}

