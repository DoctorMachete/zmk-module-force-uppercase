/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/behavior.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid_indicators.h>


#define ZMK_LED_CAPSLOCK_BIT BIT(1)         // *** from https://github.com/darknao/zmk/blob/2fad527cc5abed5bb59b4d4a4b0ee511d0e514e9/app/src/rgb_underglow.c#L320 ***

/* -----------------------------------------------------------------------
 * Shared helper: press or release a key, with an optional LSHIFT modifier.
 * shift_needed: true  → hold LSHIFT around the keycode event
 *               false → send keycode bare
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool pressed, bool shift_needed,
                    int64_t timestamp) {
    int ret;

    if (pressed && shift_needed) {
        ret = raise_zmk_keycode_state_changed_from_encoded(LSHIFT, true, timestamp);
        if (ret < 0) {
            return ret;
        }
    }

    ret = raise_zmk_keycode_state_changed_from_encoded(keycode, pressed, timestamp);
    if (ret < 0) {
        return ret;
    }

    if (!pressed && shift_needed) {
        ret = raise_zmk_keycode_state_changed_from_encoded(LSHIFT, false, timestamp);
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER
 * Goal: always send the uppercase form of the letter.
 *
 * CapsLock OFF → need Shift to capitalise    → shift_needed = true
 * CapsLock ON  → host already capitalises    → shift_needed = false
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    /* Uppercase = bare key when caps ON, shifted key when caps OFF */
    bool need_shift = !caps_active;
    return send_key(binding->param1, true, need_shift, event.timestamp);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool need_shift = !caps_active;
    return send_key(binding->param1, false, need_shift, event.timestamp);
}

static const struct behavior_driver_api force_upper_driver_api = {
    .binding_pressed  = on_force_upper_binding_pressed,
    .binding_released = on_force_upper_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_upper_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* -----------------------------------------------------------------------
 * FORCE-LOWER
 * Goal: always send the lowercase form of the letter.
 *
 * CapsLock OFF → host sends lowercase bare   → shift_needed = false
 * CapsLock ON  → host would capitalise, so
 *                we need Shift to cancel it  → shift_needed = true
 *
 * The Shift+letter combination in the presence of CapsLock produces the
 * lowercase letter on all major operating systems (Windows, macOS, Linux).
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    /* Lowercase = bare key when caps OFF, shifted key when caps ON */
    bool need_shift = caps_active;
    return send_key(binding->param1, true, need_shift, event.timestamp);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool need_shift = caps_active;
    return send_key(binding->param1, false, need_shift, event.timestamp);
}

static const struct behavior_driver_api force_lower_driver_api = {
    .binding_pressed  = on_force_lower_binding_pressed,
    .binding_released = on_force_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
