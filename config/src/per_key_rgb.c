/*
 * Per-key reactive RGB: white flash on each key press, fading over ~400ms.
 * Owns the WS2812 LED strip exposed as chosen `zmk,underglow`.
 *
 * On Jorne the chain is 28 LEDs per side: indices 0..5 are underglow
 * (kept dark), indices 6..27 are per-key (1 per switch).
 *
 * Each half is independent — we only react to position events whose source
 * is the LOCAL kscan, so the central does not double-flash for keys
 * physically pressed on the peripheral (and vice versa).
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

#define STRIP_NODE        DT_CHOSEN(zmk_underglow)
#define STRIP_LEN         DT_PROP(STRIP_NODE, chain_length)
#define UNDERGLOW_COUNT   6
#define FADE_MS           400
#define TICK_MS           33

/* Position -> local per-key LED index (6..27). Both halves use this same
 * table; on each half it's only consulted for positions whose event arrived
 * from the LOCAL kscan, so left positions are used on central and right
 * positions on peripheral.
 *
 * Chain order on the Jorne PCB (deduced from calibration: snake by columns
 * starting from the inner thumb):
 *   6:RET 7:B 8:G 9:T 10:R 11:F 12:V 13:SPC 14:TAB
 *   15:C 16:D 17:E 18:W 19:S 20:X 21:Z 22:A 23:Q 24:` 25:- 26:= 27:LWIN
 * The right half is mirrored, so each right position points at the same
 * local LED index as its left-side mirror. */
static const int8_t pos_to_led[] = {
    /* row 0, positions 0..13 (left LWIN ` Q W E R T | right Y U I O P [ ]/RWIN) */
    27, 24, 23, 18, 17, 10,  9,     9, 10, 17, 18, 23, 24, 27,
    /* row 1, positions 14..25 (left - A S D F G | right H J K L ; ') */
    25, 22, 19, 16, 11,  8,         8, 11, 16, 19, 22, 25,
    /* row 2, positions 26..37 (left = Z X C V B | right N M , . / \) */
    26, 21, 20, 15, 12,  7,         7, 12, 15, 20, 21, 26,
    /* thumbs, positions 38..43 (left TAB SPC RET | right ESC BSPC DEL) */
    14, 13,  6,                     6, 13, 14,
};

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels[STRIP_LEN];
static int64_t press_time_ms[STRIP_LEN];
static struct k_work_delayable tick_work;

static void render_tick(struct k_work *work) {
    int64_t now = k_uptime_get();
    bool any_active = false;

    /* Underglow LEDs always dark. */
    for (size_t i = 0; i < UNDERGLOW_COUNT && i < STRIP_LEN; i++) {
        pixels[i].r = pixels[i].g = pixels[i].b = 0;
    }

    for (size_t i = UNDERGLOW_COUNT; i < STRIP_LEN; i++) {
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
    /* Only react to keys physically pressed on THIS half. Events relayed
     * from the other half over the split BLE link have a non-LOCAL source. */
    if (ev->source != ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (ev->position >= ARRAY_SIZE(pos_to_led)) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    int8_t idx = pos_to_led[ev->position];
    if (idx < 0 || idx >= STRIP_LEN) {
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
