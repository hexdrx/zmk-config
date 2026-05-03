/*
 * Per-key reactive RGB: white flash on key press, fading over ~400ms.
 * Owns the WS2812 LED strip exposed as chosen `zmk,underglow`.
 * Only fires on the central side (peripheral does not receive
 * zmk_position_state_changed in ZMK v0.3).
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_REGISTER(per_key_rgb, CONFIG_LOG_DEFAULT_LEVEL);

#define STRIP_NODE   DT_CHOSEN(zmk_underglow)
#define STRIP_LEN    DT_PROP(STRIP_NODE, chain_length)
#define FADE_MS      400
#define TICK_MS      33

/* Position -> local LED index on the LEFT (central) half. -1 = not on this side.
 * Initial guess assumes a row-major chain; calibrate by pressing keys
 * one-by-one and observing which LED lights, then reorder this array. */
static const int8_t pos_to_led[] = {
    /* row 0 (positions 0..13): 7 left then 7 right */
     0,  1,  2,  3,  4,  5,  6, -1, -1, -1, -1, -1, -1, -1,
    /* row 1 (positions 14..25): 6 left then 6 right */
     7,  8,  9, 10, 11, 12, -1, -1, -1, -1, -1, -1,
    /* row 2 (positions 26..37): 6 left then 6 right */
    13, 14, 15, 16, 17, 18, -1, -1, -1, -1, -1, -1,
    /* thumbs (positions 38..43): 3 left then 3 right */
    19, 20, 21, -1, -1, -1,
};

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels[STRIP_LEN];
static int64_t press_time_ms[STRIP_LEN];
static struct k_work_delayable tick_work;

static void render_tick(struct k_work *work) {
    int64_t now = k_uptime_get();
    bool any_active = false;

    for (size_t i = 0; i < STRIP_LEN; i++) {
        uint8_t v = 0;
        int64_t t0 = press_time_ms[i];
        if (t0 > 0) {
            int64_t dt = now - t0;
            if (dt >= FADE_MS) {
                press_time_ms[i] = 0;
            } else {
                v = (uint8_t)(255 - (dt * 255) / FADE_MS);
                any_active = true;
            }
        }
        pixels[i].r = v;
        pixels[i].g = v;
        pixels[i].b = v;
    }

    led_strip_update_rgb(strip, pixels, STRIP_LEN);

    if (any_active) {
        k_work_schedule(&tick_work, K_MSEC(TICK_MS));
    }
}

static int on_position_state_changed(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL || !ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (ev->position >= ARRAY_SIZE(pos_to_led)) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    int8_t idx = pos_to_led[ev->position];
    if (idx < 0) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    press_time_ms[idx] = k_uptime_get();
    k_work_schedule(&tick_work, K_NO_WAIT);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(per_key_rgb, on_position_state_changed);
ZMK_SUBSCRIPTION(per_key_rgb, zmk_position_state_changed);

static int per_key_rgb_init(void) {
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }
    k_work_init_delayable(&tick_work, render_tick);
    memset(pixels, 0, sizeof(pixels));
    memset(press_time_ms, 0, sizeof(press_time_ms));
    led_strip_update_rgb(strip, pixels, STRIP_LEN);
    return 0;
}

SYS_INIT(per_key_rgb_init, APPLICATION, 90);
