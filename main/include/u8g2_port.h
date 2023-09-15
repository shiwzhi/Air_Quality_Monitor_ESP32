#ifndef __U8G2_PORT__
#define __U8G2_PORT__

#include "u8g2.h"

uint8_t u8x8_byte_arduino_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                                 void *arg_ptr);

uint8_t u8x8_gpio_and_delay_template(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                                     void *arg_ptr);

#endif