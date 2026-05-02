/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT 
 */

#define DT_DRV_COMPAT zmk_behavior_force_upper  /* redefined per-driver below */

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

/* Same pattern as behavior_sticky_key.c line 27 */
#define KEY_PRESS DEVICE_DT_NAME(DT_INST(0, zmk_behavior_key_press))

/* -----------------------------------------------------------------------
 * Per-key press state — snapshotted at press time, reused at release.
 * ----------------------------------------------------------------------- */
struct force_case_state {
    bool shift_held;
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
 * Shared helper — identical strategy to mod-morph.
 *
 * Mod-morph does:
 *   1. zmk_hid_masked_modifiers_set(masked_mods)   ← BEFORE invoke
 *   2. zmk_behavior_invoke_binding(key_press, event, pressed)
 *        → raises keycode_state_changed on bus
 *        → sticky_key listener fires, sees real keycode, releases naturally
 *        → hid_listener fires, calls SET_MODIFIERS(explicit & ~masked | implicit)
 *          masked covers shift → shift stripped from report
 *        → hid_listener sends report — CORRECT output
 *   3. On release: zmk_hid_masked_modifiers_clear()
 *
 * We do the same, but we also control implicit_modifiers to inject shift
 * when caps_active requires it (report_shift = want_upper XOR caps_active).
 *
 * We set implicit BEFORE invoke so hid_listener's SET_MODIFIERS sees it.
 * hid_listener calls zmk_hid_implicit_modifiers_press(ev->implicit=0)
 * which sets implicit_modifiers = 0 — this OVERWRITES our value.
 *
 * So we cannot use implicit_modifiers to inject shift through invoke.
 * Instead, when we need to inject shift (report_shift=true), we must NOT
 * mask shift — we let explicit shift through or inject via a different
 * mechanism.
 *
 * Re-examining: report_shift = want_upper XOR caps_active.
 * We need the report modifier byte to have shift = report_shift,
 * regardless of what is in explicit_modifiers.
 *
 * Cases:
 *   report_shift=false, shift in explicit → mask shift         (mask=ZMK_SHIFT_MODS)
 *   report_shift=true,  shift in explicit → don't mask shift   (mask=0)
 *   report_shift=true,  no shift          → need to ADD shift
 *   report_shift=false, no shift          → nothing needed     (mask=0)
 *
 * The "need to ADD shift" case (report_shift=true, no shift held):
 *   We cannot use implicit because hid_listener overwrites it.
 *   Solution: temporarily register shift in explicit_modifiers BEFORE
 *   invoke using zmk_hid_register_mods(MOD_LSFT), then unregister after.
 *   hid_listener will see shift in explicit_modifiers and include it.
 *   This is safe because masked_modifiers is 0 in this case so shift
 *   passes through SET_MODIFIERS correctly.
 *   We unregister after invoke to keep explicit_modifiers clean.
 *
 * Summary of what to set before zmk_behavior_invoke_binding:
 *   caps=0 want_upper=1 (report_shift=1, no shift):
 *     → register MOD_LSFT, mask=0 → hid_listener sees shift, sends shift+key ✓
 *   caps=0 want_upper=0 (report_shift=0, no shift):
 *     → mask=0, nothing → bare key ✓
 *   caps=1 want_upper=1 (report_shift=0, no shift):
 *     → mask=0, nothing → bare key, caps makes it upper ✓
 *   caps=1 want_upper=0 (report_shift=1, no shift):
 *     → register MOD_LSFT, mask=0 → shift+key cancels caps → lower ✓
 *
 * With shift held (physical or sticky):
 *   caps=0 want_upper=1 (report_shift=1, shift held):
 *     → mask=0, shift already in explicit → shift+key → upper ✓
 *   caps=0 want_upper=0 (report_shift=0, shift held):
 *     → mask=ZMK_SHIFT_MODS → shift stripped → bare key → lower ✓
 *   caps=1 want_upper=1 (report_shift=0, shift held):
 *     → mask=ZMK_SHIFT_MODS → shift stripped → bare key, caps → upper ✓
 *   caps=1 want_upper=0 (report_shift=1, shift held):
 *     → mask=0, shift in explicit → shift+key cancels caps → lower ✓
 *
 * So:
 *   shift_in_report  = explicit_shift_held AND NOT masked
 *                    + registered_shift (if we register one)
 *
 * Simplified:
 *   bool shift_held = explicit_mods has shift
 *   bool need_mask  = shift_held && !report_shift
 *   bool need_reg   = !shift_held && report_shift
 *   mask = need_mask ? ZMK_SHIFT_MODS : 0
 *   if need_reg: zmk_hid_register_mods(MOD_LSFT) before invoke
 *               zmk_hid_unregister_mods(MOD_LSFT) after invoke
 * ----------------------------------------------------------------------- */
static int send_key(uint32_t keycode, bool pressed, bool want_upper,
                    bool shift_held, struct zmk_behavior_binding_event event) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps_active  = (ind & ZMK_LED_CAPSLOCK_BIT) != 0;
    bool report_shift = want_upper ^ caps_active;

    bool need_mask = shift_held && !report_shift;
    bool need_reg  = !shift_held && report_shift;

    if (need_mask) {
        zmk_hid_masked_modifiers_set(ZMK_SHIFT_MODS);
    }
    if (need_reg) {
        zmk_hid_register_mods(MOD_LSFT);
    }

    /* Invoke key_press binding — goes through full ZMK chain.
     * sticky key sees keycode_state_changed and releases naturally.
     * hid_listener sees correct modifier state and sends correct report. */
    struct zmk_behavior_binding key_binding = {
        .behavior_dev = KEY_PRESS,
        .param1 = keycode,
    };
    int ret = zmk_behavior_invoke_binding(&key_binding, event, pressed);

    if (need_reg) {
        zmk_hid_unregister_mods(MOD_LSFT);
        /* Send one more report so the host sees shift released after the key.
         * Without this, shift stays in explicit_modifiers visibly. */
        zmk_endpoints_send_report(HID_USAGE_KEY);
    }
    if (need_mask) {
        zmk_hid_masked_modifiers_clear();
    }

    return ret;
}

/* -----------------------------------------------------------------------
 * FORCE-UPPER (fucase)
 * Ignores CapsLock. Shift inverts: no shift → upper, shift → lower.
 * ----------------------------------------------------------------------- */
#define DT_DRV_COMPAT zmk_behavior_force_upper

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static struct force_case_state force_upper_state[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int on_force_upper_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    state->shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, !state->shift_held, state->shift_held, event);
}

static int on_force_upper_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_upper_state[0];
    return send_key(binding->param1, false, !state->shift_held, state->shift_held, event);
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
 * ----------------------------------------------------------------------- */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_force_lower

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static struct force_case_state force_lower_state[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int on_force_lower_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_lower_state[0];
    state->shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, state->shift_held, state->shift_held, event);
}

static int on_force_lower_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    struct force_case_state *state = &force_lower_state[0];
    return send_key(binding->param1, false, state->shift_held, state->shift_held, event);
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
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, true, shift_held, event);
}

static int on_force_true_upper_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, true, shift_held, event);
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
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, true, false, shift_held, event);
}

static int on_force_true_lower_binding_released(struct zmk_behavior_binding *binding,
                                                struct zmk_behavior_binding_event event) {
    bool shift_held = (zmk_hid_get_explicit_mods() & ZMK_SHIFT_MODS) != 0;
    return send_key(binding->param1, false, false, shift_held, event);
}

static const struct behavior_driver_api force_true_lower_driver_api = {
    .binding_pressed  = on_force_true_lower_binding_pressed,
    .binding_released = on_force_true_lower_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &force_true_lower_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
