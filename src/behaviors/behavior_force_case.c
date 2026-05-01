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
#include <zmk/hid.h>
#include <zmk/hid_indicators.h>

#include <dt-bindings/zmk/modifiers.h>

#define ZMK_LED_CAPSLOCK_BIT BIT(1) // *** from https://github.com/darknao/zmk/blob/2fad527cc5abed5bb59b4d4a4b0ee511d0e514e9/app/src/rgb_underglow.c#L320 ***

/* Shift modifier flags covering both left and right shift */
#define ZMK_SHIFT_MODS (MOD_LSFT | MOD_RSFT)

/* -----------------------------------------------------------------------
 * Shared helper.
 *
 * want_upper: true  → produce uppercase
 *             false → produce lowercase
 *
 * We never raise fake LSHIFT press/release events onto the bus — that
 * would interfere with hold-tap, combos, and sticky-key behaviors that
 * listen to keycode_state_changed.
 *
 * Instead we use only two HID-layer mechanisms that don't touch the bus:
 *
 *   • zmk_hid_masked_modifiers_set()      — hides an already-held shift
 *     from the HID report for the duration of the keycode event.
 *
 *   • zmk_hid_implicit_modifiers_press()  — injects a shift into the HID
 *     report for the duration of the keycode event without generating any
 *     modifier event on the bus.  Released immediately after via
 *     zmk_hid_implicit_modifiers_release().
 *
 * Both techniques are used by ZMK's own behavior_mod_morph.c and
 * hid_listener.c respectively, so they are safe in this ZMK version.
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool pressed, bool want_upper,
                    int64_t timestamp) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool shift_held  = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;

    /*
     * What the host would naturally produce:
     *   caps XOR shift → uppercase,  otherwise → lowercase
     */
    bool natural_upper = caps_active ^ shift_held;

    bool mask_shift   = false;
    bool inject_shift = false;

    if (want_upper && !natural_upper) {
        inject_shift = true;   /* need uppercase, host would give lowercase */
    } else if (!want_upper && natural_upper) {
        mask_shift = true;     /* need lowercase, host would give uppercase */
    }
    /* natural already matches → send bare, no correction needed */

    int ret;

    if (mask_shift) {
        zmk_hid_masked_modifiers_set(ZMK_SHIFT_MODS);
    }
    if (inject_shift) {
        zmk_hid_implicit_modifiers_press(MOD_LSFT);
    }

    ret = raise_zmk_keycode_state_changed_from_encoded(keycode, pressed, timestamp);

    /* Always restore HID state, regardless of ret */
    if (inject_shift) {
        zmk_hid_implicit_modifiers_release();
    }
    if (mask_shift) {
        zmk_hid_masked_modifiers_clear();
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER
 * Goal: always produce uppercase. With shift held, produce lowercase.
 *
 * shift not held → want_upper = true
 * shift held     → want_upper = false  (shift inverts)
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, !shift_held, event.timestamp);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, !shift_held, event.timestamp);
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
 * Goal: always produce lowercase. With shift held, produce uppercase.
 *
 * shift not held → want_upper = false
 * shift held     → want_upper = true   (shift inverts)
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, shift_held, event.timestamp);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, shift_held, event.timestamp);
}

static const struct behavior_driver_api force_lower_driver_api = {
    .binding_pressed  = on_force_lower_binding_pressed,
    .binding_released = on_force_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
