#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/indicators.h>        // For host indicator state
#include <dt-bindings/zmk/keys.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_behavior_force_upper

static bool is_caps_lock_active(void)
{
    // Get the current host LED/indicator state
    struct zmk_hid_indicators indicators = zmk_hid_indicators_get();
    return (indicators & HID_INDICATOR_CAPS_LOCK) != 0;
}

static int behavior_force_upper_pressed(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event)
{
    uint32_t keycode = binding->param1;  // e.g. HID_USAGE_KEY_A, etc.

    bool caps_on = is_caps_lock_active();

    if (caps_on) {
        // Caps Lock is on → just send the letter (host will make it uppercase)
        zmk_hid_keyboard_press(keycode);
    } else {
        // Caps Lock is off → send explicit Left Shift + letter
        zmk_hid_keyboard_mods_set(BIT(HID_USAGE_KEY_LEFTSHIFT - 0xE0), true);
        zmk_hid_keyboard_press(keycode);
    }

    zmk_endpoints_send_report(ZMK_HID_USAGE_KEY);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_force_upper_released(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event)
{
    uint32_t keycode = binding->param1;

    bool caps_on = is_caps_lock_active();

    if (caps_on) {
        zmk_hid_keyboard_release(keycode);
    } else {
        zmk_hid_keyboard_release(keycode);
        zmk_hid_keyboard_mods_set(BIT(HID_USAGE_KEY_LEFTSHIFT - 0xE0), false);
    }

    zmk_endpoints_send_report(ZMK_HID_USAGE_KEY);

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct zmk_behavior_driver_api behavior_force_upper_driver_api = {
    .binding_pressed = behavior_force_upper_pressed,
    .binding_released = behavior_force_upper_released,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                      &behavior_force_upper_driver_api);
