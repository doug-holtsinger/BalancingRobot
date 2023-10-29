
#ifndef __QDEC__H

#include "nrfx_qdec.h"
#include "hal/nrf_gpio.h"

#define QDEC_EVT_TO_STR(event)                                             \
    (event == NRF_QDEC_EVENT_SAMPLERDY ? "NRF_QDEC_EVENT_SAMPLERDY" : \
    (event == NRF_QDEC_EVENT_REPORTRDY ? "NRF_QDEC_EVENT_REPORTRDY" : \
    (event == NRF_QDEC_EVENT_ACCOF     ? "NRF_QDEC_EVENT_ACCOF"     : \
                                         "UNKNOWN EVENT")))

class QDEC {
    public:
        QDEC();
        ~QDEC();
        void read_acc(int16_t *p_acc, int16_t *p_accdbl);
    private:
        nrfx_qdec_config_t qdec_cfg = NRFX_QDEC_DEFAULT_CONFIG;
};

#endif
