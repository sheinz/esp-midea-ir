#ifndef __MIDEA_IR_H__
#define __MIDEA_IR_H__

#include <stdint.h>
#include <stdbool.h>


typedef enum {
    MODE_COOL       = 0b0000,  // confirmed
    MODE_HEAT       = 0b1100,
    MODE_AUTO       = 0b1000,  // confirmed
    MODE_DEHUMIDIFY = 0b1000,
} MideaMode;

typedef enum {
    FAN_OFF     = 0b0111,   // confirmed
    FAN_LOW     = 0b1001,
    FAN_MEDIUM  = 0b0101,
    FAN_HIGH    = 0b0011,
    FAN_AUTO    = 0b0001,   // confirmed
} MideaFanLevel;

typedef struct {
    uint8_t temperature; // in Celsius
    MideaFanLevel fan_level;
    MideaMode mode;
    bool enabled;        // on/off air conditioner
} MideaIR;

/**
 * Initialize Ir module and ir structure with default values
 */
void midea_ir_init(MideaIR *ir);

/**
 * Send Ir signal to air conditioner
 */
void midea_ir_send(MideaIR *ir);

/**
 * Send Ir signal to move deflector
 */
void midea_ir_move_deflector(MideaIR *ir);

#endif  // __MIDEA_IR_H__
