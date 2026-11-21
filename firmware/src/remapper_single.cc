from pathlib import Path
import difflib
out=Path('/mnt/data')
original='''#include <stdio.h>

#include <tusb.h>

#include "pio_usb.h"

#include "pico/platform.h"
#include "pico/time.h"
'''
modified='''#include <stdio.h>

#include <tusb.h>

#include "pio_usb.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/platform.h"
#include "pico/time.h"

// Debug output: GP16 (UART0 TX), 921600 baud, 3.3 V logic.
// Never connect a 5 V serial adapter TX to the RP2040.
#define STEAM_RAW_REPORT_DEBUG 1
#if STEAM_RAW_REPORT_DEBUG
static uint32_t last_dump_ms = 0;
static void dump_hid_report(uint8_t dev_addr, uint8_t instance,
                            const uint8_t* report, uint16_t len) {
    // Limit serial traffic so normal USB processing can continue.
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((uint32_t)(now - last_dump_ms) < 100) return;
    last_dump_ms = now;
    printf("HID dev=%u itf=%u len=%u :", dev_addr, instance, len);
    const uint16_t shown = len < 64 ? len : 64;
    for (uint16_t i = 0; i < shown; ++i) printf(" %02X", report[i]);
    if (len > shown) printf(" ...");
    printf("\\\\r\\\\n");
}
#endif
'''
# use direct construction for context patch from exact fetched content
import subprocess
print("Preparing a small diagnostic patch for the existing GP10/GP11 firmware.")
