#pragma once

#include <stdint.h>
#include <tusb.h>

// Experimental USB Switch Pro emulation for the HID Remapper's internal
// HORIPAD personality (our_descriptors[2]).
#define SWITCH_PRO_REPORT_DESCRIPTOR_SIZE 203
extern const uint8_t switch_pro_report_descriptor[SWITCH_PRO_REPORT_DESCRIPTOR_SIZE];

void switch_pro_init();
void switch_pro_reset_session();
void switch_pro_set_horipad(const uint8_t *data, uint8_t len);
void switch_pro_capture_steam(uint8_t dev_addr, uint8_t const *report, uint16_t len);
void switch_pro_handle_output(uint8_t report_id, hid_report_type_t type,
                              const uint8_t *buffer, uint16_t len);
uint16_t switch_pro_get_report(uint8_t report_id, hid_report_type_t type,
                               uint8_t *buffer, uint16_t max_len);
void switch_pro_task();
