#ifndef __FIX_H
#define __FIX_H

// SEE https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/fix16.h

#include <stdint.h>

typedef int32_t fix_t; // signed 16 bit integer with 16 bit fraction

static const fix_t FIX_MAX  = 0x7FFFFFFF;
static const fix_t FIX_MIN  = 0x80000000;
static const fix_t FIX_OVERFLOW  = 0x80000000;

extern fix_t fix_adds(fix_t a, fix_t b);
extern fix_t fix_mul(fix_t a, fix_t b);

#endif