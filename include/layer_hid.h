#pragma once

/*
 * ZMK Layer HID Reporter
 *
 * Custom HID descriptor using the Vendor-Defined Usage Page (0xFF00).
 * Report ID 0x04 carries a single byte: the index of the current
 * top-most active ZMK layer.
 *
 * This is completely separate from the standard keyboard/consumer/mouse
 * HID interfaces, so it never interferes with normal typing.
 */

#include <zephyr/kernel.h>

/**
 * @brief Force an immediate layer-report transmission.
 *
 * Normally the module listens to ZMK layer-state change events
 * automatically.  Call this if you need to re-send the current value.
 */
void layer_hid_report_current(void);
