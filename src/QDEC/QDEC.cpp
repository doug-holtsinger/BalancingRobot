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

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "QDEC.h"

//static volatile bool m_report_ready_flag = false;
static volatile uint32_t m_accdblread;
extern int32_t wheel_encoder;

#ifdef __cplusplus
extern "C" {
#endif
void qdec_event_handler(nrfx_qdec_event_t event) {
    // NRF_LOG_INFO("Event: %s", QDEC_EVT_TO_STR(event.type));

    if (event.type == NRF_QDEC_EVENT_REPORTRDY)
    {
        wheel_encoder = event.data.report.acc;
        //NRF_LOG_INFO("accread: %d", m_accread);
        //m_accdblread        = event.data.report.accdbl;
        //NRF_LOG_INFO("accreaddbl: %d", m_accdblread);
        // m_report_ready_flag = true;
        // nrf_drv_qdec_disable();
    }
}
#ifdef __cplusplus
};
#endif


QDEC::QDEC() 
{
    // use default QDEC config specified in sdk_config.h with overrides for pins
    // because I can't seem to set pins in sdk_config.h greater than 32 
    qdec_cfg.psela = NRF_QDEC_C1;
    qdec_cfg.pselb = NRF_QDEC_C2;
    qdec_cfg.pselled = NRF_QDEC_LED_NOT_CONNECTED;
#if 1
    qdec_cfg.reportper          = (nrf_qdec_reportper_t)NRFX_QDEC_CONFIG_REPORTPER;
    qdec_cfg.sampleper          = (nrf_qdec_sampleper_t)NRFX_QDEC_CONFIG_SAMPLEPER;
    qdec_cfg.ledpre             = NRFX_QDEC_CONFIG_LEDPRE;
    qdec_cfg.ledpol             = (nrf_qdec_ledpol_t)NRFX_QDEC_CONFIG_LEDPOL;
    qdec_cfg.dbfen              = NRFX_QDEC_CONFIG_DBFEN;
    qdec_cfg.sample_inten       = NRFX_QDEC_CONFIG_SAMPLE_INTEN;
    qdec_cfg.interrupt_priority = NRFX_QDEC_CONFIG_IRQ_PRIORITY;
#endif
    // qdec_cfg.reportper = NRF_QDEC_REPORTPER_DISABLED;
    APP_ERROR_CHECK(nrfx_qdec_init(&qdec_cfg, qdec_event_handler));
    //DSH4
#if 0
    // keep all QDEC interrupts disabled.
    nrf_qdec_int_disable( NRF_QDEC_INT_SAMPLERDY_MASK |
		          NRF_QDEC_INT_REPORTRDY_MASK |
                          NRF_QDEC_INT_ACCOF_MASK );
#endif
    // start burst sampling clock
    nrfx_qdec_enable();
}

QDEC::~QDEC() 
{
    nrfx_qdec_disable();
}

void QDEC::read_acc(int16_t *p_acc, int16_t *p_accdbl)
{
    // capture the ACC in the ACCREAD register, clear the ACC,
    // and return the ACCREAD register. 
    nrfx_qdec_accumulators_read(p_acc, p_accdbl);
    //*p_acc = m_accread;
    //*p_accdbl = m_accdblread;
}

