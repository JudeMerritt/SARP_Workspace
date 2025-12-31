//TODO: Add desciptive comments with @brief

#include <stdint.h>
#include "internal/include/mmio.h"

// ((100 * 1,000,000) * 0.001 - 1)
#define RELOAD_VAL 0x1869F

void delay(uint32_t delay) {
    if (delay == 0) return;

    //Program reload value
    WRITE_FIELD(STK_RVR, STK_RVR_RELOAD, 0x1869F);

    //Clear current value
    WRITE_FIELD(STK_CVR, STK_CVR_CURRENT, 0);

    //Set clock source 
    WRITE_FIELD(STK_CSR, STK_CSR_CLKSOURCE, 1);

    //Enable SysTick
    WRITE_FIELD(STK_CSR, STK_CSR_ENABLE, 1);

    //Run delay loop
    for (int i = 0; i < delay; i++) {   
        while (READ_FIELD(STK_CSR, STK_CSR_COUNTFLAG) == 0){}
    }

    WRITE_FIELD(STK_CSR, STK_CSR_ENABLE, 0);
}