#include <stdint.h>
#include <math.h>
#include "tusb.h"

#define AUDIO_SAMPLE_RATE   CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE

extern volatile uint16_t i2s_dummy_buffer[];

void audio_init(void);
void audio_task(void);
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff);
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff);
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff);
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request);
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request);
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request);
bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting);
bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting);
bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request);