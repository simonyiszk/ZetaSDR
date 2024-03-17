#include "fix.h"

// see https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/fix16.c

fix_t fix_adds(fix_t a, fix_t b){

    fix_t result = a + b;

	// Overflow can only happen if sign of a == sign of b, and then it causes sign of sum != sign of a.
	if (!(((uint32_t)a ^ (uint32_t)b) & 0x80000000) && (((uint32_t)a ^ (uint32_t)result) & 0x80000000))
		result = (a >= 0) ? FIX_MAX : FIX_MIN;

	return result;
}

fix_t fix_mul(fix_t a, fix_t b)
{
	int64_t product = (int64_t) a * (int64_t) b;
	
	// The upper 17 bits should all be the same (the sign).
	uint32_t upper = (product >> 47);
	
	if (product < 0){
		if (~upper)
            return FIX_OVERFLOW;
		
		// This adjustment is required in order to round -1/2 correctly
		product--;
	}else{
		if (upper)
            return FIX_OVERFLOW;
	}
	
	fix_t result = product >> 16;
	result += (product & 0x8000) >> 15;
	
	return result;
}