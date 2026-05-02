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

#define LSHIFT_USAGE 0xE1
#define RSHIFT_USAGE 0xE5

/*
 * HID keyboard usage 0x00 is "Reserved" — hid_listener skips it
 * (zmk_hid_press returns -EINVAL for usage 0, so nothing is added to the
 * keyboard report). But sticky key's keycode_state_changed listener does
 * NOT filter by keycode validity — it fires on any press event that isn't
 * its own, which is all we need to trigger its release scheduling.
 */
#define DUMMY_KEYCODE 0x00

/* -----------------------------------------------------------------------
 * Per-key press state — snapshotted at press time, consumed at release.
 * ----------------------------------------------------------------------- */
struct force_case_state {
    bool shift_held;
    bool shift_sticky;
};

/* -----------------------------------------------------------------------
 * Detect sticky shift vs physical shift.
 * Sticky: in explicit_mods but NOT in pressed-keys bitmap.
 * Physical: in explicit_mods AND in pressed-keys bitmap.
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
 * Notify sticky key that a key was pressed/released, by raising a
 * keycode_state_changed event on the bus with a reserved (no-op) keycode.
 *
 * hid_listener will attempt zmk_hid_press(HID_USAGE_KEY, 0x00) which
 * returns -EINVAL and does nothing to the report. But sticky key's
 * listener fires on ANY keycode_state_changed event that isn't its own,
 * so it will schedule its modifier release in response.
 * ----------------------------------------------------------------------- */
static void notify_sticky_key_bus(bool pressed, int64_t timestamp) {
    raise_zmk_keycode_state_changed_from_encoded(DUMMY_KEYCODE, pressed, timestamp);
}

/* -----------------------------------------------------------------------
 * Shared helper.
 *
 * want_upper:       true  → produce uppercase regardless of CapsLock
 *                   false → produce lowercase regardless of CapsLock
 * shift_was_sticky: true  → notify bus so sticky key schedules release
 *                   false → leave shift state untouched
 *
 * Sequence for sticky shift case:
 *   PRESS:
 *     1. notify_sticky_key_bus(press)  → sticky key sees a key pressed,
 *                                        schedules its modifier release
 *     2. zmk_hid_masked_modifiers_set  → wipe shift from report
 *     3. zmk_hid_implicit_modifiers    → inject correct shift if needed
 *     4. zmk_hid_press + send_report   → send our key with correct case
 *     5. restore implicit + mask
 *   RELEASE:
 *     1. notify_sticky_key_bus(release) → sticky key sees key released,
 *                                         fires its pending release now
 *     2. same HID sequence as press
 *     3. consume_sticky_shift()         → direct cleanup in case sticky
 *                                         key's release hasn't fired yet
 *
 * report_shift = want_upper XOR caps_active
 * ----------------------------------------------------------------------- */

static void consume_sticky_shift(void) {
    zmk_mod_flags_t active = zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS;
    if (!active) {
        return;
    }
    if ((active & MOD_LSFT) &&
        !zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, LSHIFT_USAGE))) {
        zmk_hid_unregister_mods(MOD_LSFT);
    }
    if ((active & MOD_RSFT) &&
        !zmk_hid_is_pressed(ZMK_HID_USAGE(HID_USAGE_KEY, RSHIFT_USAGE))) {
        zmk_hid_unregister_mods(MOD_RSFT);
    }
    zmk_endpoints_send_report(HID_USAGE_KEY);
}

static int send_key(uint32_t keycode, bool pressed, bool want_upper,
                    bool shift_was_sticky, int64_t timestamp) {
    /*
     * Notify the event bus BEFORE touching HID state so sticky key's
     * listener fires while shift is still in explicit_modifiers.
     * On press: sticky key schedules release (to fire on next key release).
     * On release: sticky key fires its pending release immediately.
     */
    if (shift_was_sticky) {
        notify_sticky_key_bus(pressed, timestamp);
    }

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

    /*
     * On release: also call consume_sticky_shift() as a safety net in
     * case sticky key's own release machinery hasn't fired yet by the
     * time we send our report. The unregister is ref-counted so if
     * sticky key already cleaned up this is a no-op.
     */
    if (!pressed && shift_was_sticky) {
        consume_sticky_shift();
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER (fucase)
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static struct force_case_state force_upper_state[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    state->shift_held   = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    state->shift_sticky = is_sticky_shift();
    return send_key(binding->param1, true, !state->shift_held,
                    state->shift_sticky, event.timestamp);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    return send_key(binding->param1, false, !state->shift_held,
                    state->shift_sticky, event.timestamp);
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
    return send_key(binding->param1, true, state->shift_held,
                    state->shift_sticky, event.timestamp);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_lower_state[0];
    return send_key(binding->param1, false, state->shift_held,
                    state->shift_sticky, event.timestamp);
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
 * Always uppercase. Ignores CapsLock and Shift entirely.
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_true_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_true_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                               struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, true, true, false, event.timestamp);
}

static int on_force_true_upper_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, false, true, false, event.timestamp);
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
 * Always lowercase. Ignores CapsLock and Shift entirely.
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_true_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_force_true_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                               struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, true, false, false, event.timestamp);
}

static int on_force_true_lower_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    return send_key(binding->param1, false, false, false, event.timestamp);
}

static const struct behavior_driver_api force_true_lower_driver_api = {
    .binding_pressed  = on_force_true_lower_binding_pressed,
    .binding_released = on_force_true_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_true_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
