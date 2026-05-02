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

/* HID usage codes for Left/Right Shift keys (USB HID keyboard page) */
#define LSHIFT_USAGE 0xE1
#define RSHIFT_USAGE 0xE5

/* -----------------------------------------------------------------------
 * Per-key press state.
 *
 * We need to snapshot shift state at press time and carry it through to
 * release time. Re-evaluating at release is wrong because:
 *   - sticky shift may already be gone by release time
 *   - physical shift may have been released between press and release
 *
 * ZMK behaviors are called from a single thread so no locking needed.
 * We store state per-behavior-instance (one for fucase, one for flcase).
 * ----------------------------------------------------------------------- */
struct force_case_state {
    bool shift_held;
    bool shift_sticky;
};

/* -----------------------------------------------------------------------
 * Detect sticky shift vs physical shift.
 *
 * Physical shift: hid_listener called zmk_hid_press(LSHIFT/RSHIFT usage),
 *   so the keycode appears in the pressed-keys bitmap.
 * Sticky shift: behavior_sticky_key called zmk_hid_register_mod() only,
 *   never zmk_hid_press(), so the keycode is NOT in the pressed-keys bitmap.
 * ----------------------------------------------------------------------- */
static bool is_sticky_shift(void) {
    if (!(zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS)) {
        return false;
    }
    bool lshift_key_pressed = zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, LSHIFT_USAGE));
    bool rshift_key_pressed = zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, RSHIFT_USAGE));
    return !lshift_key_pressed && !rshift_key_pressed;
}

/* -----------------------------------------------------------------------
 * Shared helper.
 *
 * want_upper:       true  → produce uppercase regardless of CapsLock
 *                   false → produce lowercase regardless of CapsLock
 * shift_was_sticky: true  → raise LSHIFT release on bus after key release
 *                           so sticky key cleans itself up
 *                   false → leave shift state untouched
 *
 * report_shift = want_upper XOR caps_active:
 *   caps=0 want_upper=1 → shift=1
 *   caps=0 want_upper=0 → shift=0
 *   caps=1 want_upper=1 → shift=0
 *   caps=1 want_upper=0 → shift=1
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool pressed, bool want_upper, bool shift_was_sticky) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active  = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool report_shift = want_upper ^ caps_active;

    zmk_hid_masked_modifiers_set(ZMK_SHIFT_MODS);

    if (report_shift) {
        zmk_hid_implicit_modifiers_press(MOD_LSFT);
    }

    int ret;
    if (pressed) {
        ret = zmk_hid_press(ZMK_HID_USAGE(HID_USAGE_KEY, keycode));
    } else {
        ret = zmk_hid_release(ZMK_HID_USAGE(HID_USAGE_KEY, keycode));
    }

    if (ret == 0) {
        ret = zmk_endpoints_send_report(HID_USAGE_KEY);
    }

    if (report_shift) {
        zmk_hid_implicit_modifiers_release();
    }
    zmk_hid_masked_modifiers_clear();

    /* On key release: trigger sticky shift cleanup using the state
     * snapshotted at press time, not re-evaluated now. */
    if (!pressed && shift_was_sticky) {
        raise_zmk_keycode_state_changed_from_encoded(LSHIFT, false, k_uptime_get());
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER (fucase)
 * Ignores CapsLock. Shift inverts: no shift → upper, shift → lower.
 * Sticky shift is consumed after the key; physical shift is not released.
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* One instance of state per fucase node in the devicetree */
static struct force_case_state force_upper_state[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    /* Snapshot both at press time */
    state->shift_held   = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    state->shift_sticky = is_sticky_shift();
    return send_key(binding->param1, true, !state->shift_held, state->shift_sticky);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    /* Use press-time snapshot, do not re-evaluate */
    return send_key(binding->param1, false, !state->shift_held, state->shift_sticky);
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
 * FORCE-LOWER (flcase)
 * Ignores CapsLock. Shift inverts: no shift → lower, shift → upper.
 * Sticky shift is consumed after the key; physical shift is not released.
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static struct force_case_state force_lower_state[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int on_force_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_lower_state[0];
    state->shift_held   = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    state->shift_sticky = is_sticky_shift();
    return send_key(binding->param1, true, state->shift_held, state->shift_sticky);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_lower_state[0];
    return send_key(binding->param1, false, state->shift_held, state->shift_sticky);
}

static const struct behavior_driver_api force_lower_driver_api = {
    .binding_pressed  = on_force_lower_binding_pressed,
    .binding_released = on_force_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* -----------------------------------------------------------------------
 * FORCE-TRUE-UPPER (ftucase)
 * Always outputs uppercase. Ignores BOTH CapsLock and Shift entirely.
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_true_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_true_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                               struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, true, true, false);
}

static int on_force_true_upper_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, false, true, false);
}

static const struct behavior_driver_api force_true_upper_driver_api = {
    .binding_pressed  = on_force_true_upper_binding_pressed,
    .binding_released = on_force_true_upper_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_true_upper_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* -----------------------------------------------------------------------
 * FORCE-TRUE-LOWER (ftlcase)
 * Always outputs lowercase. Ignores BOTH CapsLock and Shift entirely.
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_true_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_true_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                               struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, true, false, false);
}

static int on_force_true_lower_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, false, false, false);
}

static const struct behavior_driver_api force_true_lower_driver_api = {
    .binding_pressed  = on_force_true_lower_binding_pressed,
    .binding_released = on_force_true_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_true_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
