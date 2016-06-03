#ifndef __MIDEA_IR_H__
#define __MIDEA_IR_H__

#include <stdint.h>
#include <stdbool.h>


typedef enum {
    MODE_COOL       = 0b0000,
    MODE_HEAT       = 0b1100,
    MODE_AUTO       = 0b1000,
    MODE_FAN        = 0b0100,
    MODE_DEHUMIDIFY = 0b1000,
} MideaMode;

typedef struct {
    uint8_t temperature; // in Celsius
    uint8_t fan_level;   // 0..3
    MideaMode mode;
    bool enabled;        // on/off air conditioner
} MideaIR;

/**
 * Initialize Ir module and ir structure with default values
 */
void midea_ir_init(MideaIR *ir, const uint8_t pin_number);

/**
 * Send Ir signal to air conditioner
 */
void midea_ir_send(MideaIR *ir);

/**
 * Send Ir signal to move deflector
 */
void midea_ir_move_deflector(MideaIR *ir);

#endif  // __MIDEA_IR_H__
