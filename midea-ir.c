#include "midea-ir.h"
#include <stdio.h>
#include "espressif/esp_misc.h"
#include "esp/interrupts.h"
#include "esp/gpio.h"
#include "esp/timer.h"

/**
 * Midea Air conditioner protocol consists of 3 data bytes.
 * After each data byte follows its inverted byte (0 an 1 are switched). It
 * provides errors checking on the receiver side.
 *
 * Each 6 total bytes follows with additional repeat of the same 6 bytes to
 * provide even more errors checking.
 */

/**
 * Bits encoding.
 *
 * Bit 0 is encoded as 1T of high level and 1T of low level.
 * Bit 1 is encoded as 1T of high level and 3T of low level.
 *
 * Start condition: 8T of hight level and 8T of low level.
 * Middle condition (beetween two 6 bytes): 
 * TODO: describe
 */

/**
 * Data packet (3 bytes):
 * [1010 0010] [ffff ssss] [ttttcccc]
 *
 * 1010 0010 - (0xB2) is a constant
 *
 * ffff - Fan control
 *      1001 - low speed
 *      0101 - medium speed
 *      0011 - high speed
 *      1011 - automatic
 *      0111 - off
 *
 * ssss - State control (unconfirmed!)
 *      1111 - on
 *      1011 - off
 *
 * tttt - Temperature control (unconfirmed!)
 *      0000 - 17 Celsius
 *      ...
 *      1011 - 30 Celsius
 *      1110 - off
 *
 * cccc - Command
 *      0000 - cool
 *      1100 - heat
 *      1000 - automatic
 *      1101 - dehumidify
 */

// #define DEBUG_PRINT

#define TEMP_LOW  17
#define TEMP_HIGH 30

#define RAW_DATA_PACKET_SIZE   6   // each byte is sent two times

typedef struct 
{
    uint8_t magic;      // 0xB2 always
    uint8_t state : 4;
    uint8_t fan : 4; 
    uint8_t command : 4;
    uint8_t temp : 4;
} DataPacket;

// Table to convert temperature in Celcius to a strange Midea AirCon values
const static uint8_t temperature_table[] = {
    0b0000,   // 17 C
    0b0001,   // 18 C
    0b0011,   // 19 C
    0b0010,   // 20 C
    0b0110,   // 21 C
    0b0111,   // 22 C
    0b0101,   // 23 C
    0b0100,   // 24 C
    0b1100,   // 25 C
    0b1101,   // 26 C
    0b1001,   // 27 C
    0b1000,   // 28 C
    0b1010,   // 29 C
    0b1011    // 30 C
//  0b1110    // off
};

#define PULSES_CAPACITY         29    // 8T + 8T + (4T * 8 * 6) + 8T
#define SUB_PULSES_PER_PULSE    42    // (high + low) * 21

typedef struct 
{
    uint8_t pin_number;
    uint8_t pulses[PULSES_CAPACITY];  // pulses to spit out
    uint8_t pulses_size;
    uint8_t repeat_count;            // how many times to repeat
    uint8_t current_pulse;            // current processing pulse
    uint8_t current_sub_pulse;        // 38000 kHz pulse
} IrState;

static volatile IrState ir_state;

static void timer_interrupt_handler(void)
{
    // get bit number 'current_pulse'
    bool pulse_val = ir_state.pulses[ir_state.current_pulse/8] 
        & (1<<(ir_state.current_pulse%8));

    if (ir_state.current_sub_pulse < SUB_PULSES_PER_PULSE) {
        if (!(ir_state.current_sub_pulse % 2) && pulse_val) {
            gpio_write(ir_state.pin_number, true);
        } else {
            gpio_write(ir_state.pin_number, false);
        }
        ir_state.current_sub_pulse++;
    } else { // pulse is finished
        ir_state.current_sub_pulse = 0;
        ir_state.current_pulse++;
        if (ir_state.current_pulse >= ir_state.pulses_size) {
            ir_state.repeat_count--;
            if (ir_state.repeat_count) {
                ir_state.current_pulse = 0;
                ir_state.current_sub_pulse = 0;
            } else {
                timer_set_run(FRC1, false);
            }
        }
    }
}

static inline void pack_data(MideaIR *ir, DataPacket *data)
{
    data->magic = 0xB2;
    if (ir->enabled) {
        data->fan = ir->fan_level;
        data->state = 0b1111;  // on
        data->command = ir->mode;

        if (ir->temperature >= TEMP_LOW && ir->temperature <= TEMP_HIGH) {
            data->temp = temperature_table[ir->temperature - TEMP_LOW];
        } else {
            data->temp = 0b0100;
        }
    } else {
        data->fan = 0b0111;
        data->state = 0b1011; // off
        data->command = 0b0000;
        data->temp = 0b1110;
    }
}

void midea_ir_init(MideaIR *ir)
{
    ir->temperature = 24;
    ir->enabled = true;
    ir->mode = MODE_AUTO;
    ir->fan_level = FAN_AUTO;

    gpio_enable(ir_state.pin_number, GPIO_OUTPUT);
    gpio_write(ir_state.pin_number, false);

    _xt_isr_attach(INUM_TIMER_FRC1, timer_interrupt_handler);
    timer_set_frequency(FRC1, 38000 * 2);   // two iterrupts per period
    timer_set_interrupts(FRC1, true);

    ir_state.repeat_count = 0;   // indicates IDLE state
}

#ifdef DEBUG_PRINT
static inline void print_bit(bool bit)
{
    if (bit) {
        printf("1");
    } else {
        printf("0");
    }
}
#endif

static inline void init_buff()
{
    ir_state.current_pulse = 0;
    ir_state.current_sub_pulse = 0;

    for (uint8_t i = 0; i < PULSES_CAPACITY; i++) {
        ir_state.pulses[i] = 0;
    }
}

static inline void add_start()
{
    ir_state.pulses[0] = 0b11111111; 
    ir_state.pulses[1] = 0b00000000; 
    ir_state.current_pulse = 8 * 2;
}

static inline void add_bit(bool bit)
{
    // add 1 to the pulses
    ir_state.pulses[ir_state.current_pulse/8] |= 
        (1<<(ir_state.current_pulse%8));

    ir_state.current_pulse++;

    if (bit) {
        ir_state.current_pulse += 3;  // bit 1 -> pulses 1000
    } else {
        ir_state.current_pulse++;     // bit 0 -> pulses 10
    }
}

static inline void add_stop()
{
    add_bit(true);
    ir_state.current_pulse += 8;
}

static inline void start(const uint8_t repeat)
{
    ir_state.pulses_size= ir_state.current_pulse; 
    ir_state.current_pulse = 0;
    ir_state.current_sub_pulse = 0;
    ir_state.repeat_count = repeat;
    timer_set_run(FRC1, true);
}

/* For each byte in src add two bytes in dst (normal and bit-wise inverted)
 */
static inline void add_complementary_bytes(const uint8_t *src, uint8_t *dst)
{
    for (int i = 0; i < 3; i++) {
        *dst = *src;
        dst++;
        *dst = ~(*src);
        dst++;
        src++;
    }
}

static inline void send_ir_data(const uint8_t data[RAW_DATA_PACKET_SIZE], 
        const uint8_t repeat)
{
    init_buff();
    add_start();

#ifdef DEBUG_PRINT
    printf("Data: ");
#endif

    for (int b = 0; b < RAW_DATA_PACKET_SIZE; b++) {
        uint8_t v = data[b];
        for (uint8_t i = 0; i < 8; i++) {
#ifdef DEBUG_PRINT
            print_bit(v & (1<<7));
#endif
            add_bit(v & (1<<7));
            v <<= 1;
        } 
    }
#ifdef DEBUG_PRINT
    printf("\n");
#endif

    add_stop();
    start(repeat);
}

void midea_ir_send(MideaIR *ir)
{
    DataPacket packet; 
    pack_data(ir, &packet);
    uint8_t data[RAW_DATA_PACKET_SIZE];
    add_complementary_bytes((uint8_t*)&packet, data);

    send_ir_data(data, 2);
}

void midea_ir_move_deflector(MideaIR *ir)
{
    uint8_t data[RAW_DATA_PACKET_SIZE/2];
    uint8_t raw_data[RAW_DATA_PACKET_SIZE];

    // TODO: fill data to move deflector

    add_complementary_bytes(data, raw_data);
    
    send_ir_data(data, 1);
}
