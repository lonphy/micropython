#include "pin_remap.h"

void board_early_init(void) {
    
    // I2C1 remap to PB8, PB9
    pin_remap_config(REMAP_PIN_I2C1, REMAP_VAL_PART1);
    
    // SPI1 remap to PB3, PB4, PB5
    // pin_remap_config(REMAP_PIN_SPI1, REMAP_VAL_PART1);
}
