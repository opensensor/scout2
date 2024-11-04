#include "system.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "hardware/vreg.h"

int system_init(void) {
    // Initialize stdlib for Pico
    stdio_init_all();
    
    // Set voltage to support higher frequencies
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);  // Allow voltage to stabilize
    
    // Set system clock to 250MHz
    clock_configure(clk_sys,
                   CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                   CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                   250 * MHZ,
                   250 * MHZ);
    
    // Configure peripheral clocks
    clock_configure(clk_peri,
                   0,
                   CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                   250 * MHZ,
                   250 * MHZ);
    
    return 0;
}

