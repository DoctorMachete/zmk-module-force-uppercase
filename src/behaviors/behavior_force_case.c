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

/* -----------------------------------------------------------------------
 * Per-key press state — snapshotted at press time, reused at release.
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
 * Sticky key release via LSHIFT bus event.
 *
 * Sticky key listens to zmk_keycode_state_changed. Its self-exclusion
 * filter (behavior_sticky_key.c line 278-284) only skips events where:
 *   behavior_dev == KEY_PRESS AND keycode == sticky_key->param1
 *
 * For &sk LSFT, param1 encodes LSHIFT. So we CANNOT use LSHIFT keycode
 * to trigger it — it would be filtered out as "the sticky key's own event".
 *
 * Instead we use LSHIFT on the CONSUMER page — same keycode value 0xE1
 * but different usage_page, so the filter does not match (it checks both
 * usage_page and keycode). hid_listener ignores consumer keycodes above
 * ZMK_HID_CONSUMER_MAX_USAGE so it's a no-op for the report.
 *
 * Actually simpler: looking at sticky_key source line 278-284:
 *   strcmp(behavior_dev, KEY_PRESS) == 0       ← must be key_press behavior
 *   ZMK_HID_USAGE_ID(param1) == ev->keycode    ← keycode must match
 *   ZMK_HID_USAGE_PAGE(param1) == ev->usage_page ← page must match
 *
 * So if we use a DIFFERENT keycode that isn't LSHIFT, sticky key WILL
 * process it. We use keycode 0x00 on HID_USAGE_KEY page.
 * hid_listener tries zmk_hid_press(ZMK_HID_USAGE(HID_USAGE_KEY, 0x00)).
 * zmk_hid_keyboard_press(0x00): 0x00 < LEFTCONTROL (0xE0) so it calls
 * select_keyboard_usage(0x00) = TOGGLE_KEYBOARD(0x00, 1) which sets
 * keys[0] bit 0. This IS a problem — it puts a phantom key in the report.
 *
 * But we have masked_modifiers set during this call, so the modifier
 * byte is correct. The keys byte will have bit 0 of keys[0] set briefly.
 * We need to clean it up. We do: zmk_hid_release the same usage after.
 *
 * Actually the cleanest approach: use a keycode in the modifier range
 * (0xE0-0xE7) that is NOT LSHIFT (0xE1) or RSHIFT (0xE5).
 * E.g. LCTRL = 0xE0. sticky key's filter checks param1's keycode/page.
 * &sk LSFT has param1 = LSHIFT usage, so filter only blocks LSHIFT events.
 * A LCTRL event passes the filter, sticky key processes it.
 * hid_listener calls zmk_hid_keyboard_press(0xE0) = zmk_hid_register_mod(0)
 * = registers LCTRL in explicit_modifiers. But masked_modifiers only masks
 * ZMK_SHIFT_MODS — LCTRL would leak into the report!
 *
 * The truly clean solution: expand masked_modifiers to cover ALL modifiers
 * during the sticky trigger event, then restore.
 *
 * We use LCTRL (0xE0) as the trigger keycode. masked = all modifiers.
 * hid_listener registers LCTRL, but mask blocks it from the report.
 * After the event, we unregister LCTRL to keep explicit_modifiers clean.
 * Sticky key sees a non-LSHIFT key pressed and fires its release.
 * ----------------------------------------------------------------------- */

/* All 8 modifier bits */
#define ALL_MODS 0xFF

/* Trigger keycode: LCTRL — in modifier range, not LSHIFT or RSHIFT */
#define STICKY_TRIGGER_USAGE 0xE0

static void notify_sticky_key_for_release(int64_t timestamp) {
    /* Mask ALL modifiers so hid_listener's register_mod call is hidden */
    zmk_hid_masked_modifiers_set(ALL_MODS);

    /* Raise LCTRL press — sticky key sees a non-own key, fires release */
    raise_zmk_keycode_state_changed_from_encoded(STICKY_TRIGGER_USAGE, true, timestamp);

    /* Raise LCTRL release — clean up; sticky key records key-up */
    raise_zmk_keycode_state_changed_from_encoded(STICKY_TRIGGER_USAGE, false, timestamp);

    /* Unregister LCTRL from explicit_modifiers to keep state clean.
     * hid_listener called zmk_hid_register_mods for LCTRL on press
     * and zmk_hid_unregister_mods on release, so they cancel out.
     * explicit_modifiers is already clean. Just clear the mask. */
    zmk_hid_masked_modifiers_clear();
}

/* -----------------------------------------------------------------------
 * Shared helper — direct HID bypass (proven to work for caps/shift).
 *
 * For sticky shift: call notify_sticky_key_for_release() at press time
 * BEFORE our direct HID work. This fires sticky key's release machinery
 * without corrupting our report.
 *
 * report_shift = want_upper XOR caps_active
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool pressed, bool want_upper,
                    bool shift_was_sticky, int64_t timestamp) {
    /*
     * If shift was sticky, notify sticky key FIRST so it releases.
     * This must happen before we set our own mask/implicit so that
     * sticky key's cleanup (unregister LSHIFT from explicit_modifiers)
     * is complete before we compute report_shift.
     */
    if (pressed && shift_was_sticky) {
        notify_sticky_key_for_release(timestamp);
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
                    false, event.timestamp);
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
                    false, event.timestamp);
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
