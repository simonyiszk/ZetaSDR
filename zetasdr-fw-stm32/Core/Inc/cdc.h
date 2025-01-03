#include <stdbool.h>
#include <stdint.h>

void cdc_task(void);
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts);
void tud_cdc_rx_cb(uint8_t itf);