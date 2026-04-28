/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_force_upper

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/behavior.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid_indicators.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    uint32_t keycode = binding->param1;

    /* Read current CapsLock state as reported by the host */
    zmk_hid_indicators_t indicators = zmk_hid_indicators_get_current_profile();
   // bool caps_active = (indicators & ZMK_HID_INDICATORS_CAPSLOCK) != 0;
    bool caps_active = (indicators & HID_KBD_MODIFIER_LEFT_SHIFT) != 0;

    int ret;

    if (!caps_active) {
        /*
         * CapsLock is OFF: we need Shift to produce uppercase.
         * Press LSHIFT first, then the key.
         */
        ret = raise_zmk_keycode_state_changed_from_encoded(LSHIFT, true, event.timestamp);
        if (ret < 0) {
            return ret;
        }
    }

    ret = raise_zmk_keycode_state_changed_from_encoded(keycode, true, event.timestamp);
    return ret;
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    uint32_t keycode = binding->param1;

    zmk_hid_indicators_t indicators = zmk_hid_indicators_get_current_profile();
    bool caps_active = (indicators & ZMK_HID_INDICATORS_CAPSLOCK) != 0;

    int ret;

    ret = raise_zmk_keycode_state_changed_from_encoded(keycode, false, event.timestamp);
    if (ret < 0) {
        return ret;
    }

    if (!caps_active) {
        /* Release the LSHIFT we pressed */
        ret = raise_zmk_keycode_state_changed_from_encoded(LSHIFT, false, event.timestamp);
    }

    return ret;
}

static const struct behavior_driver_api force_upper_driver_api = {
    .binding_pressed  = on_force_upper_binding_pressed,
    .binding_released = on_force_upper_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0,
                        NULL,
                        NULL,
                        NULL,
                        NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_upper_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
