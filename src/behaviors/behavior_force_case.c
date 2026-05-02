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
#include <zmk/endpoints.h>

#include <dt-bindings/zmk/modifiers.h>
#include <dt-bindings/zmk/hid_usage_pages.h>

#define ZMK_LED_CAPSLOCK_BIT BIT(1) // *** from https://github.com/darknao/zmk/blob/2fad527cc5abed5bb59b4d4a4b0ee511d0e514e9/app/src/rgb_underglow.c#L320 ***

#define ZMK_SHIFT_MODS (MOD_LSFT | MOD_RSFT)

/* -----------------------------------------------------------------------
 * SET_MODIFIERS in hid.c is:
 *   keyboard_report.body.modifiers = (explicit & ~masked) | implicit
 *
 * Key insight:
 *   - masked_modifiers suppresses bits from explicit_modifiers
 *   - implicit_modifiers is OR'd in AFTER the mask → never masked out
 *
 * Strategy:
 *   1. mask = ZMK_SHIFT_MODS  → wipes physical shift from report
 *   2. if report needs shift  → set implicit = MOD_LSFT
 *                               (survives the mask because of the | implicit)
 *   3. press/release key directly via zmk_hid_press/release
 *   4. send report via zmk_endpoints_send_report
 *   5. restore: clear implicit, clear mask
 *
 * We bypass raise_zmk_keycode_state_changed_from_encoded entirely because
 * hid_listener would call zmk_hid_implicit_modifiers_press(ev->implicit=0)
 * which would overwrite our implicit before the report is sent.
 *
 * Desired report shift = want_upper XOR caps_active:
 *   caps=0 want_upper=1 → shift=1  (shift produces uppercase)
 *   caps=0 want_upper=0 → shift=0  (bare  produces lowercase)
 *   caps=1 want_upper=1 → shift=0  (bare  produces uppercase via caps)
 *   caps=1 want_upper=0 → shift=1  (shift cancels caps → lowercase)
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool pressed, bool want_upper) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active  = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool report_shift = want_upper ^ caps_active;

    /* Step 1: mask physical shift — (explicit & ~mask) zeroes shift bits */
    zmk_hid_masked_modifiers_set(ZMK_SHIFT_MODS);

    /* Step 2: inject shift via implicit if needed — survives mask in SET_MODIFIERS */
    if (report_shift) {
        zmk_hid_implicit_modifiers_press(MOD_LSFT);
    }

    /* Step 3: update the key state directly, no event bus */
    int ret;
    if (pressed) {
        ret = zmk_hid_press(ZMK_HID_USAGE(HID_USAGE_KEY, keycode));
    } else {
        ret = zmk_hid_release(ZMK_HID_USAGE(HID_USAGE_KEY, keycode));
    }

    /* Step 4: send the report now — modifiers are exactly as needed */
    if (ret == 0) {
        ret = zmk_endpoints_send_report(HID_USAGE_KEY);
    }

    /* Step 5: restore HID state unconditionally */
    if (report_shift) {
        zmk_hid_implicit_modifiers_release();
    }
    zmk_hid_masked_modifiers_clear();

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER
 * shift not held → want_upper = true  (default: uppercase)
 * shift held     → want_upper = false (inverted: lowercase)
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, !shift_held);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, !shift_held);
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
 * shift not held → want_upper = false (default: lowercase)
 * shift held     → want_upper = true  (inverted: uppercase)
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, shift_held);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, shift_held);
}

static const struct behavior_driver_api force_lower_driver_api = {
    .binding_pressed  = on_force_lower_binding_pressed,
    .binding_released = on_force_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
